// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pin controller, GPIO and GPIO interrupt driver for the Samsung/Apple S5L8720
 *
 * The controller has banks of eight pins. Each bank has a 0x20-byte register
 * block: CON holds a 4-bit function per pin, DAT the pin levels, and PUNB/PUNC
 * (+0x0c/+0x10) the pull-up/pull-down enables. Pin functions are changed through a single
 * write-only register (FSEL) that takes the bank, the pin and the new function;
 * function 0 is input, 0xe | value drives the pin as an output (CON reads
 * back 1 for outputs), and 2..7 select peripheral alternate functions.
 *
 * Interrupts go through a separate controller (GPIOIC) in groups of 32. Each
 * group has level, status, enable and type registers and its own parent
 * interrupt. GPIO interrupt numbers are pin + 24, so pin interrupts start in
 * group 0. Both-edge triggers are emulated with a level trigger whose
 * polarity flips every time it fires.
 */

#include <dt-bindings/pinctrl/apple.h>

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
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "core.h"
#include "pinconf.h"
#include "pinctrl-utils.h"
#include "pinmux.h"

#define S5L8720_GPIO_PINS_PER_BANK	8
#define S5L8720_GPIO_BANK_STRIDE	0x20
#define S5L8720_GPIO_CON		0x00
#define S5L8720_GPIO_DAT		0x04
#define S5L8720_GPIO_PUD1		0x0c	/* PUNB: 1 = pull-up enabled */
#define S5L8720_GPIO_PUD2		0x10	/* PUNC: 1 = pull-down enabled */
#define S5L8720_GPIO_FSEL		0x1e0
/* FSEL sits right after the last bank. */
#define S5L8720_GPIO_NBANKS		(S5L8720_GPIO_FSEL / S5L8720_GPIO_BANK_STRIDE)
#define S5L8720_GPIO_NPINS		(S5L8720_GPIO_NBANKS * S5L8720_GPIO_PINS_PER_BANK)

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
	struct device *dev;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pctl_desc;
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
	u32 dat[S5L8720_GPIO_NBANKS];
	/* saved +0x08..+0x1c (includes PUNB +0x0c, PUNC +0x10, sleep-state regs) */
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

/* ----------------------------------------------------------- pinctrl */

static int s5l8720_dt_node_to_map(struct pinctrl_dev *pctldev,
				  struct device_node *node,
				  struct pinctrl_map **map,
				  unsigned int *num_maps)
{
	struct s5l8720_gpio *sg = pinctrl_dev_get_drvdata(pctldev);
	unsigned long *configs = NULL;
	unsigned int num_configs = 0, reserved_maps = 0;
	int num_pins, i, ret;

	*map = NULL;
	*num_maps = 0;

	num_pins = of_property_count_u32_elems(node, "pinmux");
	if (num_pins <= 0) {
		dev_err(sg->dev, "missing or empty pinmux in %pOFn\n", node);
		return num_pins ?: -EINVAL;
	}

	ret = pinconf_generic_parse_dt_config(node, pctldev, &configs, &num_configs);
	if (ret)
		return ret;

	ret = pinctrl_utils_reserve_map(pctldev, map, &reserved_maps, num_maps,
					num_pins * (1 + !!num_configs));
	if (ret)
		goto free_configs;

	for (i = 0; i < num_pins; i++) {
		u32 pinfunc, pin, func;
		const char *group_name, *func_name;

		ret = of_property_read_u32_index(node, "pinmux", i, &pinfunc);
		if (ret)
			goto free_map;

		pin = APPLE_PIN(pinfunc);
		func = APPLE_FUNC(pinfunc);
		if (pin >= S5L8720_GPIO_NPINS || func == 1 || func > 7) {
			ret = -EINVAL;
			goto free_map;
		}

		group_name = pinctrl_generic_get_group_name(pctldev, pin);
		func_name = pinmux_generic_get_function_name(pctldev, func);
		ret = pinctrl_utils_add_map_mux(pctldev, map, &reserved_maps,
						num_maps, group_name, func_name);
		if (ret)
			goto free_map;

		if (num_configs) {
			ret = pinctrl_utils_add_map_configs(pctldev, map,
							    &reserved_maps,
							    num_maps, group_name,
							    configs, num_configs,
							    PIN_MAP_TYPE_CONFIGS_PIN);
			if (ret)
				goto free_map;
		}
	}

	kfree(configs);
	return 0;

free_map:
	pinctrl_utils_free_map(pctldev, *map, *num_maps);
free_configs:
	kfree(configs);
	return ret;
}

static const struct pinctrl_ops s5l8720_pinctrl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = s5l8720_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static int s5l8720_pinmux_set(struct pinctrl_dev *pctldev, unsigned int func,
			      unsigned int group)
{
	struct s5l8720_gpio *sg = pinctrl_dev_get_drvdata(pctldev);

	if (group >= S5L8720_GPIO_NPINS || func == 1 || func > 7)
		return -EINVAL;

	s5l8720_gpio_fsel(sg, group, func);
	return 0;
}

static const struct pinmux_ops s5l8720_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = s5l8720_pinmux_set,
	.strict = true,
};

static int s5l8720_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			       unsigned long *config)
{
	struct s5l8720_gpio *sg = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	void __iomem *bank = s5l8720_bank(sg, pin);
	u32 bit = BIT(pin % S5L8720_GPIO_PINS_PER_BANK);
	bool up = !!(readl(bank + S5L8720_GPIO_PUD1) & bit);
	bool dn = !!(readl(bank + S5L8720_GPIO_PUD2) & bit);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (up || dn)
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!up || dn)
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (up || !dn)
			return -EINVAL;
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, 1);
	return 0;
}

static int s5l8720_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			       unsigned long *configs, unsigned int num_configs)
{
	struct s5l8720_gpio *sg = pinctrl_dev_get_drvdata(pctldev);
	void __iomem *bank = s5l8720_bank(sg, pin);
	u32 bit = BIT(pin % S5L8720_GPIO_PINS_PER_BANK);
	unsigned int i;

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		unsigned long flags;
		u32 p1, p2;

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			spin_lock_irqsave(&sg->lock, flags);
			p1 = readl(bank + S5L8720_GPIO_PUD1) & ~bit;
			p2 = readl(bank + S5L8720_GPIO_PUD2) & ~bit;
			writel(p1, bank + S5L8720_GPIO_PUD1);
			writel(p2, bank + S5L8720_GPIO_PUD2);
			spin_unlock_irqrestore(&sg->lock, flags);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			spin_lock_irqsave(&sg->lock, flags);
			p1 = readl(bank + S5L8720_GPIO_PUD1) | bit;
			p2 = readl(bank + S5L8720_GPIO_PUD2) & ~bit;
			writel(p1, bank + S5L8720_GPIO_PUD1);
			writel(p2, bank + S5L8720_GPIO_PUD2);
			spin_unlock_irqrestore(&sg->lock, flags);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			spin_lock_irqsave(&sg->lock, flags);
			p1 = readl(bank + S5L8720_GPIO_PUD1) & ~bit;
			p2 = readl(bank + S5L8720_GPIO_PUD2) | bit;
			writel(p1, bank + S5L8720_GPIO_PUD1);
			writel(p2, bank + S5L8720_GPIO_PUD2);
			spin_unlock_irqrestore(&sg->lock, flags);
			break;
		default:
			return -ENOTSUPP;
		}
	}
	return 0;
}

static int s5l8720_pinconf_group_get(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     unsigned long *config)
{
	return s5l8720_pinconf_get(pctldev, selector, config);
}

static int s5l8720_pinconf_group_set(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     unsigned long *configs,
				     unsigned int num_configs)
{
	return s5l8720_pinconf_set(pctldev, selector, configs, num_configs);
}

static const struct pinconf_ops s5l8720_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = s5l8720_pinconf_get,
	.pin_config_set = s5l8720_pinconf_set,
	.pin_config_group_get = s5l8720_pinconf_group_get,
	.pin_config_group_set = s5l8720_pinconf_group_set,
};

/* ------------------------------------------------------------- GPIO */

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

	return func == 1 || (func & 0xe) == 0xe ? GPIO_LINE_DIRECTION_OUT :
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

/* ----------------------------------------------------------- GPIOIC */

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
	.name		= "pinctrl-s5l8720",
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

/* ------------------------------------------------------------ probe */

static int s5l8720_pinctrl_register(struct platform_device *pdev,
				    struct s5l8720_gpio *sg)
{
	static const char *const pinmux_funcs[] = {
		"gpio", "reserved", "func2", "func3",
		"func4", "func5", "func6", "func7",
	};
	struct pinctrl_pin_desc *pins;
	const char **pin_names;
	unsigned int *pin_nums;
	unsigned int i;
	int ret;

	pins = devm_kmalloc_array(&pdev->dev, S5L8720_GPIO_NPINS,
				  sizeof(*pins), GFP_KERNEL);
	pin_names = devm_kmalloc_array(&pdev->dev, S5L8720_GPIO_NPINS,
				       sizeof(*pin_names), GFP_KERNEL);
	pin_nums = devm_kmalloc_array(&pdev->dev, S5L8720_GPIO_NPINS,
				      sizeof(*pin_nums), GFP_KERNEL);
	if (!pins || !pin_names || !pin_nums)
		return -ENOMEM;

	for (i = 0; i < S5L8720_GPIO_NPINS; i++) {
		pins[i].number = i;
		pins[i].name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "PIN%u", i);
		if (!pins[i].name)
			return -ENOMEM;
		pins[i].drv_data = sg;
		pin_names[i] = pins[i].name;
		pin_nums[i] = i;
	}

	sg->pctl_desc.name = dev_name(&pdev->dev);
	sg->pctl_desc.pins = pins;
	sg->pctl_desc.npins = S5L8720_GPIO_NPINS;
	sg->pctl_desc.pctlops = &s5l8720_pinctrl_ops;
	sg->pctl_desc.pmxops = &s5l8720_pinmux_ops;
	sg->pctl_desc.confops = &s5l8720_pinconf_ops;
	sg->pctl_desc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(&pdev->dev, &sg->pctl_desc, sg,
					     &sg->pctldev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "pinctrl init failed\n");

	for (i = 0; i < S5L8720_GPIO_NPINS; i++) {
		ret = pinctrl_generic_add_group(sg->pctldev, pins[i].name,
						pin_nums + i, 1, sg);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(pinmux_funcs); i++) {
		ret = pinmux_generic_add_function(sg->pctldev, pinmux_funcs[i],
						  pin_names, S5L8720_GPIO_NPINS,
						  sg);
		if (ret < 0)
			return ret;
	}

	return pinctrl_enable(sg->pctldev);
}

static int s5l8720_gpio_probe(struct platform_device *pdev)
{
	struct s5l8720_gpio *sg;
	int ret;

	sg = devm_kzalloc(&pdev->dev, sizeof(*sg), GFP_KERNEL);
	if (!sg)
		return -ENOMEM;

	sg->dev = &pdev->dev;
	sg->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sg->base))
		return PTR_ERR(sg->base);

	spin_lock_init(&sg->lock);
	platform_set_drvdata(pdev, sg);

	sg->chip.label = dev_name(&pdev->dev);
	sg->chip.parent = &pdev->dev;
	sg->chip.owner = THIS_MODULE;
	sg->chip.base = -1;
	sg->chip.ngpio = S5L8720_GPIO_NPINS;
	sg->chip.request = gpiochip_generic_request;
	sg->chip.free = gpiochip_generic_free;
	sg->chip.set_config = gpiochip_generic_config;
	sg->chip.get = s5l8720_gpio_get;
	sg->chip.set = s5l8720_gpio_set;
	sg->chip.get_direction = s5l8720_gpio_get_direction;
	sg->chip.direction_input = s5l8720_gpio_direction_input;
	sg->chip.direction_output = s5l8720_gpio_direction_output;

	/* The interrupt controller is optional (second "reg" entry). */
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1)) {
		ret = s5l8720_gpio_irq_init(pdev, sg);
		if (ret)
			return ret;
		sg->chip.to_irq = s5l8720_gpio_to_irq;
	}

	ret = s5l8720_pinctrl_register(pdev, sg);
	if (ret)
		return ret;

	/* Legacy fallback for DTBs using apple,pin-functions. */
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
		sg->dat[b] = readl(bank + S5L8720_GPIO_DAT);
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
		for (p = 0; p < S5L8720_GPIO_PINS_PER_BANK; p++) {
			u32 func = (sg->con[b] >> (p * 4)) & 0xf;

			/* CON reads back 1 for GPIO outputs; FSEL takes 0xe/0xf. */
			if (func == 1)
				func = S5L8720_FUNC_OUTPUT(sg->dat[b] & BIT(p));
			s5l8720_gpio_fsel(sg, b * S5L8720_GPIO_PINS_PER_BANK + p,
					  func);
		}
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
	{ .compatible = "apple,s5l8720-pinctrl" },
	{ .compatible = "apple,s5l8720-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8720_gpio_of_match);

static struct platform_driver s5l8720_gpio_driver = {
	.probe		= s5l8720_gpio_probe,
	.driver = {
		.name		= "pinctrl-s5l8720",
		.of_match_table = s5l8720_gpio_of_match,
		.pm		= pm_sleep_ptr(&s5l8720_gpio_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(s5l8720_gpio_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8720 pinctrl and GPIO controller");
MODULE_LICENSE("GPL");
