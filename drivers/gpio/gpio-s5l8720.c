// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO driver for the Samsung/Apple S5L8720
 *
 * The controller has banks of eight pins. Each bank has a 0x20-byte register
 * block: CON holds a 4-bit function per pin and DAT the pin levels. Pin
 * functions are changed through a single write-only register (FSEL) that
 * takes the bank, the pin and the new function; function 0 is input and
 * 0xe | value drives the pin as an output.
 *
 * Interrupts go through a separate controller (GPIOIC) in groups of 32. Each
 * group has level, status, enable and type registers and its own parent
 * interrupt. GPIO interrupt numbers are pin + 24, so pin interrupts start in
 * group 0. Both-edge triggers are emulated with a level trigger whose
 * polarity flips every time it fires.
 *
 * There is no pin controller: the boot chain sets the pins' alternate
 * functions and they stay. For pads the board depends on, "apple,pin-functions"
 * (pairs of pin and function 2..7) writes them at probe, so they are right
 * whatever ran before (another kernel through kexec, a different loader).
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define S5L8720_GPIO_PINS_PER_BANK	8
#define S5L8720_GPIO_BANK_STRIDE	0x20
#define S5L8720_GPIO_CON		0x00
#define S5L8720_GPIO_DAT		0x04
#define S5L8720_GPIO_FSEL		0x1e0
/* FSEL sits right after the last bank. */
#define S5L8720_GPIO_NBANKS		(S5L8720_GPIO_FSEL / S5L8720_GPIO_BANK_STRIDE)

#define S5L8720_FSEL_BANK(b)		(((b) & 0x1f) << 16)
#define S5L8720_FSEL_PIN(p)		(((p) & 0x7) << 8)
#define S5L8720_FUNC_INPUT		0x0
#define S5L8720_FUNC_OUTPUT(v)		(0xe | !!(v))

/* GPIO interrupt controller, per group of 32 interrupts */
#define GPIOIC_LEVEL(g)			(0x80 + (g) * 4)	/* 1: high/rising */
#define GPIOIC_STAT(g)			(0xa0 + (g) * 4)	/* write 1 to clear */
#define GPIOIC_EN(g)			(0xc0 + (g) * 4)
#define GPIOIC_TYPE(g)			(0xe0 + (g) * 4)	/* 1: level */
#define GPIOIC_MAX_GROUPS		7
#define GPIOIC_PIN_OFFSET		24	/* interrupt number = pin + 24 */

struct s5l8720_gpio;

struct s5l8720_gpio_group {
	struct s5l8720_gpio *sg;
	unsigned int index;
};

struct s5l8720_gpio {
	struct gpio_chip chip;
	void __iomem *base;
	void __iomem *ic;
	spinlock_t lock;
	raw_spinlock_t irq_lock;
	struct irq_domain *domain;
	unsigned int ngroups;
	u32 autoflip[GPIOIC_MAX_GROUPS];
	struct s5l8720_gpio_group groups[GPIOIC_MAX_GROUPS];

	/* saved across a suspend that may cut the SoC's power */
	u32 con[S5L8720_GPIO_NBANKS];
	/* PUD1, PUD2 (pulls) and the four sleep-state registers, 0x08..0x1c */
	u32 bankregs[S5L8720_GPIO_NBANKS][6];
	u32 ic_level[GPIOIC_MAX_GROUPS], ic_type[GPIOIC_MAX_GROUPS];
	u32 ic_en[GPIOIC_MAX_GROUPS];
};

static void __iomem *s5l8720_bank(struct s5l8720_gpio *sg, unsigned int off)
{
	return sg->base + (off / S5L8720_GPIO_PINS_PER_BANK) *
	       S5L8720_GPIO_BANK_STRIDE;
}

static void s5l8720_gpio_fsel(struct s5l8720_gpio *sg, unsigned int off,
			      u32 func)
{
	unsigned long flags;

	spin_lock_irqsave(&sg->lock, flags);
	writel(S5L8720_FSEL_BANK(off / S5L8720_GPIO_PINS_PER_BANK) |
	       S5L8720_FSEL_PIN(off % S5L8720_GPIO_PINS_PER_BANK) | func,
	       sg->base + S5L8720_GPIO_FSEL);
	spin_unlock_irqrestore(&sg->lock, flags);
}

static int s5l8720_gpio_get(struct gpio_chip *chip, unsigned int off)
{
	struct s5l8720_gpio *sg = gpiochip_get_data(chip);

	return !!(readl(s5l8720_bank(sg, off) + S5L8720_GPIO_DAT) &
		  BIT(off % S5L8720_GPIO_PINS_PER_BANK));
}

static int s5l8720_gpio_get_direction(struct gpio_chip *chip, unsigned int off)
{
	struct s5l8720_gpio *sg = gpiochip_get_data(chip);
	u32 con = readl(s5l8720_bank(sg, off) + S5L8720_GPIO_CON);
	u32 func = (con >> ((off % S5L8720_GPIO_PINS_PER_BANK) * 4)) & 0xf;

	return (func & 0xe) == 0xe ? GPIO_LINE_DIRECTION_OUT :
				     GPIO_LINE_DIRECTION_IN;
}

static int s5l8720_gpio_direction_input(struct gpio_chip *chip,
					unsigned int off)
{
	s5l8720_gpio_fsel(gpiochip_get_data(chip), off, S5L8720_FUNC_INPUT);
	return 0;
}

static int s5l8720_gpio_direction_output(struct gpio_chip *chip,
					 unsigned int off, int value)
{
	s5l8720_gpio_fsel(gpiochip_get_data(chip), off,
			  S5L8720_FUNC_OUTPUT(value));
	return 0;
}

static void s5l8720_gpio_set(struct gpio_chip *chip, unsigned int off, int value)
{
	/* An output's level is part of its function code. */
	s5l8720_gpio_fsel(gpiochip_get_data(chip), off,
			  S5L8720_FUNC_OUTPUT(value));
}

static void s5l8720_ic_rmw(struct s5l8720_gpio *sg, u32 reg, u32 mask, u32 val)
{
	writel((readl(sg->ic + reg) & ~mask) | (val & mask), sg->ic + reg);
}

static void s5l8720_irq_ack(struct irq_data *d)
{
	struct s5l8720_gpio *sg = irq_data_get_irq_chip_data(d);

	writel(BIT(d->hwirq % 32), sg->ic + GPIOIC_STAT(d->hwirq / 32));
}

static void s5l8720_irq_mask(struct irq_data *d)
{
	struct s5l8720_gpio *sg = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&sg->irq_lock, flags);
	s5l8720_ic_rmw(sg, GPIOIC_EN(d->hwirq / 32), BIT(d->hwirq % 32), 0);
	raw_spin_unlock_irqrestore(&sg->irq_lock, flags);
}

static void s5l8720_irq_unmask(struct irq_data *d)
{
	struct s5l8720_gpio *sg = irq_data_get_irq_chip_data(d);
	u32 bit = BIT(d->hwirq % 32);
	unsigned long flags;

	raw_spin_lock_irqsave(&sg->irq_lock, flags);
	writel(bit, sg->ic + GPIOIC_STAT(d->hwirq / 32));
	s5l8720_ic_rmw(sg, GPIOIC_EN(d->hwirq / 32), bit, bit);
	raw_spin_unlock_irqrestore(&sg->irq_lock, flags);
}

static int s5l8720_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct s5l8720_gpio *sg = irq_data_get_irq_chip_data(d);
	unsigned int g = d->hwirq / 32;
	u32 bit = BIT(d->hwirq % 32);
	bool level, high, flip = false;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_HIGH:
		level = true; high = true;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		level = true; high = false;
		break;
	case IRQ_TYPE_EDGE_RISING:
		level = false; high = true;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		level = false; high = false;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		/* Level trigger on the opposite of the pin's current level. */
		if (d->hwirq < GPIOIC_PIN_OFFSET ||
		    d->hwirq - GPIOIC_PIN_OFFSET >= sg->chip.ngpio)
			return -EINVAL;
		level = true;
		high = !s5l8720_gpio_get(&sg->chip, d->hwirq - GPIOIC_PIN_OFFSET);
		flip = true;
		break;
	default:
		return -EINVAL;
	}

	/* An interrupt pin has to be an input to see its level change. */
	if (d->hwirq >= GPIOIC_PIN_OFFSET &&
	    d->hwirq - GPIOIC_PIN_OFFSET < sg->chip.ngpio)
		s5l8720_gpio_fsel(sg, d->hwirq - GPIOIC_PIN_OFFSET,
				  S5L8720_FUNC_INPUT);

	raw_spin_lock_irqsave(&sg->irq_lock, flags);
	s5l8720_ic_rmw(sg, GPIOIC_TYPE(g), bit, level ? bit : 0);
	s5l8720_ic_rmw(sg, GPIOIC_LEVEL(g), bit, high ? bit : 0);
	if (flip)
		sg->autoflip[g] |= bit;
	else
		sg->autoflip[g] &= ~bit;
	raw_spin_unlock_irqrestore(&sg->irq_lock, flags);

	irq_set_handler_locked(d, level ? handle_level_irq : handle_edge_irq);
	return 0;
}

static struct irq_chip s5l8720_irq_chip = {
	.name		= "gpio-s5l8720",
	.irq_ack	= s5l8720_irq_ack,
	.irq_mask	= s5l8720_irq_mask,
	.irq_unmask	= s5l8720_irq_unmask,
	.irq_set_type	= s5l8720_irq_set_type,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static void s5l8720_gpio_irq_handler(struct irq_desc *desc)
{
	struct s5l8720_gpio_group *grp = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct s5l8720_gpio *sg = grp->sg;
	unsigned int g = grp->index;
	unsigned long pending;
	unsigned int b;

	chained_irq_enter(chip, desc);
	pending = readl(sg->ic + GPIOIC_STAT(g)) & readl(sg->ic + GPIOIC_EN(g));
	for_each_set_bit(b, &pending, 32) {
		if (sg->autoflip[g] & BIT(b)) {
			raw_spin_lock(&sg->irq_lock);
			s5l8720_ic_rmw(sg, GPIOIC_LEVEL(g), BIT(b),
				       ~readl(sg->ic + GPIOIC_LEVEL(g)));
			raw_spin_unlock(&sg->irq_lock);
		}
		generic_handle_domain_irq(sg->domain, g * 32 + b);
	}
	chained_irq_exit(chip, desc);
}

static int s5l8720_irq_map(struct irq_domain *dom, unsigned int virq,
			   irq_hw_number_t hw)
{
	irq_set_chip_data(virq, dom->host_data);
	irq_set_chip_and_handler(virq, &s5l8720_irq_chip, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops s5l8720_irq_ops = {
	.map	= s5l8720_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static int s5l8720_gpio_to_irq(struct gpio_chip *chip, unsigned int off)
{
	struct s5l8720_gpio *sg = gpiochip_get_data(chip);
	unsigned int hw = off + GPIOIC_PIN_OFFSET;

	if (!sg->domain || hw >= sg->ngroups * 32)
		return -ENXIO;
	return irq_create_mapping(sg->domain, hw);
}

/* GPIOIC start-up sequence. */
static void s5l8720_ic_start(struct device *dev, struct s5l8720_gpio *sg)
{
	u32 val;

	writel(0x32, sg->ic + 0x70);
	writel(3, sg->ic + 0x6c);
	if (readl_poll_timeout_atomic(sg->ic + 0x7c, val, val & 1, 1, 10000))
		dev_warn(dev, "GPIOIC did not report ready\n");
	writel(2, sg->ic + 0x6c);
}

static int s5l8720_gpio_irq_init(struct platform_device *pdev,
				 struct s5l8720_gpio *sg)
{
	struct device *dev = &pdev->dev;
	unsigned int g;
	int n, irq;

	sg->ic = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(sg->ic))
		return PTR_ERR(sg->ic);

	n = platform_irq_count(pdev);
	if (n <= 0 || n > GPIOIC_MAX_GROUPS)
		return dev_err_probe(dev, -EINVAL, "need 1-%d group interrupts\n",
				     GPIOIC_MAX_GROUPS);
	sg->ngroups = n;
	raw_spin_lock_init(&sg->irq_lock);

	s5l8720_ic_start(dev, sg);

	for (g = 0; g < sg->ngroups; g++) {
		writel(0, sg->ic + GPIOIC_EN(g));
		writel(~0U, sg->ic + GPIOIC_STAT(g));
	}

	sg->domain = irq_domain_add_linear(dev->of_node, sg->ngroups * 32,
					   &s5l8720_irq_ops, sg);
	if (!sg->domain)
		return -ENOMEM;

	for (g = 0; g < sg->ngroups; g++) {
		irq = platform_get_irq(pdev, g);
		if (irq < 0)
			return irq;
		sg->groups[g].sg = sg;
		sg->groups[g].index = g;
		irq_set_chained_handler_and_data(irq, s5l8720_gpio_irq_handler,
						 &sg->groups[g]);
	}

	dev_dbg(dev, "%u interrupt groups\n", sg->ngroups);
	return 0;
}

static int s5l8720_gpio_probe(struct platform_device *pdev)
{
	struct s5l8720_gpio *sg;

	sg = devm_kzalloc(&pdev->dev, sizeof(*sg), GFP_KERNEL);
	if (!sg)
		return -ENOMEM;

	sg->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sg->base))
		return PTR_ERR(sg->base);

	spin_lock_init(&sg->lock);

	sg->chip.label = dev_name(&pdev->dev);
	sg->chip.parent = &pdev->dev;
	sg->chip.owner = THIS_MODULE;
	sg->chip.base = -1;
	sg->chip.ngpio = S5L8720_GPIO_NBANKS * S5L8720_GPIO_PINS_PER_BANK;
	sg->chip.get = s5l8720_gpio_get;
	sg->chip.set = s5l8720_gpio_set;
	sg->chip.get_direction = s5l8720_gpio_get_direction;
	sg->chip.direction_input = s5l8720_gpio_direction_input;
	sg->chip.direction_output = s5l8720_gpio_direction_output;

	/* The interrupt controller is optional (second "reg" entry). */
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1)) {
		int ret = s5l8720_gpio_irq_init(pdev, sg);

		if (ret)
			return ret;
		sg->chip.to_irq = s5l8720_gpio_to_irq;
	}

	/* Board pads that must be in a peripheral function (pin, function). */
	{
		struct device_node *np = pdev->dev.of_node;
		int n = of_property_count_u32_elems(np, "apple,pin-functions");
		u32 pin, func;
		int i;

		for (i = 0; i + 1 < n; i += 2) {
			if (of_property_read_u32_index(np, "apple,pin-functions", i, &pin) ||
			    of_property_read_u32_index(np, "apple,pin-functions", i + 1, &func))
				break;
			if (pin >= sg->chip.ngpio || func < 2 || func > 7) {
				dev_warn(&pdev->dev, "pin-functions: bad entry %u %u\n",
					 pin, func);
				continue;
			}
			s5l8720_gpio_fsel(sg, pin, func);
		}
	}

	platform_set_drvdata(pdev, sg);
	return devm_gpiochip_add_data(&pdev->dev, &sg->chip, sg);
}

/*
 * Suspend to RAM cuts the SoC's power and the wake goes through LLB, which
 * leaves the pins and the GPIO interrupt controller in their reset state.
 * Save both, and on resume put them back if they changed. After s2idle
 * nothing has changed and nothing is written.
 */
static int s5l8720_gpio_suspend_noirq(struct device *dev)
{
	struct s5l8720_gpio *sg = dev_get_drvdata(dev);
	unsigned int b, g;

	for (b = 0; b < S5L8720_GPIO_NBANKS; b++) {
		void __iomem *bank = sg->base + b * S5L8720_GPIO_BANK_STRIDE;
		unsigned int r;

		sg->con[b] = readl(bank + S5L8720_GPIO_CON);
		for (r = 0; r < 6; r++)
			sg->bankregs[b][r] = readl(bank + 0x8 + r * 4);
	}
	for (g = 0; sg->ic && g < sg->ngroups; g++) {
		sg->ic_level[g] = readl(sg->ic + GPIOIC_LEVEL(g));
		sg->ic_type[g] = readl(sg->ic + GPIOIC_TYPE(g));
		sg->ic_en[g] = readl(sg->ic + GPIOIC_EN(g));
	}
	return 0;
}

static int s5l8720_gpio_resume_noirq(struct device *dev)
{
	struct s5l8720_gpio *sg = dev_get_drvdata(dev);
	unsigned int b, p, g;
	bool lost = false;

	for (b = 0; b < S5L8720_GPIO_NBANKS; b++) {
		u32 now = readl(sg->base + b * S5L8720_GPIO_BANK_STRIDE + S5L8720_GPIO_CON);

		if (now == sg->con[b])
			continue;
		lost = true;
		for (p = 0; p < 6; p++)
			writel(sg->bankregs[b][p],
			       sg->base + b * S5L8720_GPIO_BANK_STRIDE + 0x8 + p * 4);
		for (p = 0; p < S5L8720_GPIO_PINS_PER_BANK; p++)
			s5l8720_gpio_fsel(sg, b * S5L8720_GPIO_PINS_PER_BANK + p,
					  (sg->con[b] >> (p * 4)) & 0xf);
	}
	/*
	 * Pin functions only change on a power loss; interrupt enables also
	 * change when a disabled interrupt fires during s2idle, so they are not
	 * the test.
	 */
	if (lost && sg->ic) {
		s5l8720_ic_start(dev, sg);
		for (g = 0; g < sg->ngroups; g++) {
			writel(0, sg->ic + GPIOIC_EN(g));
			writel(sg->ic_level[g], sg->ic + GPIOIC_LEVEL(g));
			writel(sg->ic_type[g], sg->ic + GPIOIC_TYPE(g));
			writel(~0U, sg->ic + GPIOIC_STAT(g));
			writel(sg->ic_en[g], sg->ic + GPIOIC_EN(g));
		}
	}
	if (lost)
		dev_info(dev, "pin and interrupt state restored after power loss\n");
	return 0;
}

static const struct dev_pm_ops s5l8720_gpio_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(s5l8720_gpio_suspend_noirq, s5l8720_gpio_resume_noirq)
};

static const struct of_device_id s5l8720_gpio_of_match[] = {
	{ .compatible = "apple,s5l8720-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8720_gpio_of_match);

static struct platform_driver s5l8720_gpio_driver = {
	.probe = s5l8720_gpio_probe,
	.driver = {
		.name = "gpio-s5l8720",
		.of_match_table = s5l8720_gpio_of_match,
		.pm = pm_sleep_ptr(&s5l8720_gpio_pm_ops),
	},
};
module_platform_driver(s5l8720_gpio_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8720 GPIO driver");
MODULE_LICENSE("GPL");
