// SPDX-License-Identifier: GPL-2.0-only
/*
 * PMU of the Apple iPod touch (2nd generation), N72
 *
 * A Dialog D1759 on I2C0 at 0x73. This driver and its sub-devices cover the
 * display backlight, power-off, the real-time clock, the battery (voltage,
 * capacity, USB power) and the regulators (the LDOs and the core rail).
 * This core driver handles the I2C device, the interrupt controller, the
 * GPIOs and power-off; the sub-devices' drivers are apple-n72-pmu-backlight,
 * rtc-apple-n72-pmu, apple-n72-pmu-power and apple-n72-pmu-regulator.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/core.h>
#include <linux/mod_devicetable.h>
#include <linux/mfd/apple-n72-pmu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

#define N72_PMU_STATUS_HPDET	BIT(2)	/* byte 0: PMU GPIO 2, high = plugged */

/*
 * The PMU's state survives suspend to RAM, but if hibernate turned
 * LDOs/GPIOs off, put back what was set before (enables after voltages).
 */
static const u8 n72_ldo_saved[] = {
	0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x10, 0x11, 0x13,
	0x0e,
};

struct n72_pmu {
	struct i2c_client *client;
	struct regmap *regmap;
	struct blocking_notifier_head events;	/* the interrupt's, for the sub-devices */

	struct mutex int2_lock;		/* INT2 is shared by the ADC and the GPIO interrupt */
	u8 int2_pending;		/* INT2 bits read but not yet consumed */

	/* PMU GPIOs (iOS "GPIO" functions on pmu@73) and wake IRQ lines */
	struct gpio_chip gc;
	struct irq_domain *irq_domain;
	struct mutex irq_lock;		/* the lines' state below, and their masks */
	bool hp_irq_on, hp_irq_was_on;
	bool wl_irq_on, wl_irq_was_on;
	bool bt_irq_on, bt_irq_was_on;
	unsigned long irq_wake;		/* lines whose consumer asked for the wake */
	bool hp_slept, wl_slept, bt_slept;	/* lines masked for this system sleep */
	bool usb_irq_on;		/* USB plug/unplug unmasked */
	u8 ldo_regs[ARRAY_SIZE(n72_ldo_saved)];
};

#define N72_PMU_INT1		0x01	/* interrupt status, cleared on read */
#define N72_PMU_INT1_MASK	0x07	/* interrupt masks 0x07..0x09 */
#define N72_PMU_OOCSHDWN	0x0a
#define N72_PMU_OOCSHDWN_GOSTBY	BIT(0)

/*
 * USB power: INT1 bit 3 latches when it comes, INT3 bit 2 when it goes
 * (with STATUS byte 0 bit 3; INT1 bit 6 and INT3 bit 4 move with them).
 * Observed on the N72 with the cable pulled and replugged, masks closed.
 */
#define N72_PMU_INT1_USB_IN	BIT(3)
#define N72_PMU_INT3		0x03
#define N72_PMU_INT3_USB_OUT	BIT(2)
#define N72_PMU_INT3_MASK	0x09
#define N72_PMU_INT2		0x02
#define N72_PMU_INT2_HPDET	BIT(6)	/* PMU GPIO 2 (headphone detect) changed */
/*
 * iOS's device tree numbers the PMU's interrupts; read as bit numbers across
 * INT1..INT3, the Mikey headset chip (PMU GPIO 7) has 0x0e = INT2 bit 6 and
 * the codec node, which owns the headphone detect GPIO, 0x16 = INT3 bit 6.
 * A headphone plug was seen on INT2 bit 6 here; which of the two lines that
 * was is not settled, so both bits report as the one "jack" interrupt (line
 * 2), shared by the sound card's jack and the Mikey driver.
 */
#define N72_PMU_INT3_JACK	BIT(6)
/* GPIO configuration: one nibble per GPIO from 0x0b; iOS activates GPIO 7 with 9 */
#define N72_PMU_GPIO67_CFG	0x0e
#define N72_PMU_GPIO6_MASK	0x0f
#define N72_PMU_GPIO6_OUT	BIT(1)
#define N72_PMU_GPIO6_HIGH	BIT(2)
#define N72_PMU_GPIO6_LO	N72_PMU_GPIO6_OUT
#define N72_PMU_GPIO6_HI	(N72_PMU_GPIO6_OUT | N72_PMU_GPIO6_HIGH)
#define N72_PMU_INT2_MASK	0x08

/*
 * The PMU's "on" input, iOS's wake event "wlan": the BCM4325's host wake line
 * (its GPIO 0, NVRAM sd_gpout). Found by observation: INT1 bit 0 is set, and
 * set again after every read, while the chip in host sleep has something for
 * the host. It is line 8 of the PMU's interrupt domain, after GPIOs 0-7.
 */
#define N72_PMU_INT1_ON		BIT(0)
#define N72_PMU_IRQ_WLAN	8
/*
 * The PMU's "ldo2_en" input ("bt" wake event): the BCM4325's Bluetooth
 * host wake line. It is reported on INT2 bit 0 (mask bit 0 of 0x08),
 * alongside INT1 bit 0 ("on") and INT3 bit 6 ("gpio3"). Mapped to line 9 of
 * the PMU's interrupt domain. Which level latches the bit is unverified.
 */
#define N72_PMU_INT2_BT		BIT(0)
#define N72_PMU_IRQ_BT		9

/* Clear pending interrupts, mask them, and transition to standby. */
static int n72_pmu_power_off(struct sys_off_data *data)
{
	struct n72_pmu *pmu = data->cb_data;
	static const u8 masks[] = { 0xd1, 0xff, 0xf0 };
	u8 discard[3];

	i2c_smbus_read_i2c_block_data(pmu->client, N72_PMU_INT1,
				      sizeof(discard), discard);
	i2c_smbus_write_i2c_block_data(pmu->client, N72_PMU_INT1_MASK,
				       sizeof(masks), masks);
	i2c_smbus_write_byte_data(pmu->client, N72_PMU_OOCSHDWN,
				  N72_PMU_OOCSHDWN_GOSTBY);
	mdelay(1000);

	return NOTIFY_DONE;
}

/*
 * INT1's mask register has bits of three users (the alarm, the Wi-Fi host
 * wake line, the USB plug) that change them from different contexts.
 */
int n72_pmu_int1_mask(struct n72_pmu *pmu, u8 bits, bool masked)
{
	return regmap_update_bits(pmu->regmap, N72_PMU_INT1_MASK, bits,
				  masked ? bits : 0);
}
EXPORT_SYMBOL_GPL(n72_pmu_int1_mask);

/*
 * INT2 is cleared on read and carries both the ADC's done bit (polled) and
 * the headphone-detect change (an interrupt), so every read goes through
 * here and keeps the bits the other user has not taken yet.
 */
int n72_pmu_int2_take(struct n72_pmu *pmu, u8 bits)
{
	unsigned int v = 0;
	int ret, got;

	mutex_lock(&pmu->int2_lock);
	ret = regmap_read(pmu->regmap, N72_PMU_INT2, &v);
	if (!ret)
		pmu->int2_pending |= v;
	got = pmu->int2_pending & bits;
	pmu->int2_pending &= ~bits;
	mutex_unlock(&pmu->int2_lock);
	return ret < 0 ? ret : got;
}
EXPORT_SYMBOL_GPL(n72_pmu_int2_take);

/* Give back INT2 bits that were taken and could not be delivered. */
static void n72_pmu_int2_keep(struct n72_pmu *pmu, u8 bits)
{
	mutex_lock(&pmu->int2_lock);
	pmu->int2_pending |= bits;
	mutex_unlock(&pmu->int2_lock);
}

/*
 * Run the handler of one of the PMU's lines. False if the line is disabled
 * just now (system resume has not reached it yet): handle_nested_irq() would
 * only mark it pending, and a nested interrupt without a parent is not sent
 * again.
 */
static bool n72_pmu_line_irq(struct n72_pmu *pmu, unsigned int line)
{
	unsigned int virq = pmu->irq_domain ?
			    irq_find_mapping(pmu->irq_domain, line) : 0;
	struct irq_data *d = virq ? irq_get_irq_data(virq) : NULL;

	if (!d)
		return true;
	if (irqd_irq_disabled(d))
		return false;
	handle_nested_irq(virq);
	return true;
}

static irqreturn_t n72_pmu_irq(int irq, void *data)
{
	struct n72_pmu *pmu = data;
	unsigned int v = 0, v3 = 0;
	int r1, v2, r3 = 0;

	r1 = regmap_read(pmu->regmap, N72_PMU_INT1, &v);
	v2 = n72_pmu_int2_take(pmu, N72_PMU_INT2_HPDET | N72_PMU_INT2_BT);
	if (READ_ONCE(pmu->usb_irq_on) || READ_ONCE(pmu->hp_irq_was_on))
		r3 = regmap_read(pmu->regmap, N72_PMU_INT3, &v3);

	if (r1 < 0 && v2 < 0)
		return IRQ_NONE;
	/* (a level: the PMU raises it again for as long as it is asserted) */
	if (!r1 && (v & N72_PMU_INT1_ON) && READ_ONCE(pmu->wl_irq_was_on))
		n72_pmu_line_irq(pmu, N72_PMU_IRQ_WLAN);
	if (!r1 && (v & N72_PMU_INT1_ALARM))
		blocking_notifier_call_chain(&pmu->events, N72_PMU_EVENT_ALARM, NULL);
	if (!r1 && (v & N72_PMU_INT1_USB_IN))
		blocking_notifier_call_chain(&pmu->events, N72_PMU_EVENT_USB_IN, NULL);
	else if (!r3 && (v3 & N72_PMU_INT3_USB_OUT))
		blocking_notifier_call_chain(&pmu->events, N72_PMU_EVENT_USB_OUT, NULL);
	/* one latched event: kept if it cannot be delivered, for n72_pmu_resume() */
	if (v2 > 0 && (v2 & N72_PMU_INT2_BT) && READ_ONCE(pmu->bt_irq_was_on) &&
	    !n72_pmu_line_irq(pmu, N72_PMU_IRQ_BT))
		n72_pmu_int2_keep(pmu, N72_PMU_INT2_BT);
	/* (the plug's state is reported afresh at resume) */
	if ((v2 > 0 && (v2 & N72_PMU_INT2_HPDET)) ||
	    (!r3 && (v3 & N72_PMU_INT3_JACK) && READ_ONCE(pmu->hp_irq_was_on)))
		n72_pmu_line_irq(pmu, 2);
	return IRQ_HANDLED;
}

/* -------------------------------------------------------------- GPIOs */

#define N72_PMU_NGPIO		8

/*
 * The PMU's GPIOs as iOS's device tree uses them: 2 = headphone detect
 * (input; its level in STATUS bit 2, a change in INT2 bit 6), 6 = BCM4325
 * power enable (output; reg 0x0e low nibble: 0x6 high, 0x2 low), and 7 = Mikey
 * interrupt (0x90).
 */
static int n72_pmu_gpio_get(struct gpio_chip *gc, unsigned int off)
{
	struct n72_pmu *pmu = gpiochip_get_data(gc);
	unsigned int v;
	int ret;

	if (off == 2) {
		ret = regmap_read(pmu->regmap, N72_PMU_STATUS, &v);
		return ret < 0 ? ret : !!(v & N72_PMU_STATUS_HPDET);
	}
	if (off == 6) {
		ret = regmap_read(pmu->regmap, N72_PMU_GPIO67_CFG, &v);
		return ret < 0 ? ret : !!(v & N72_PMU_GPIO6_HIGH);
	}
	return -EOPNOTSUPP;
}

static void n72_pmu_gpio_set(struct gpio_chip *gc, unsigned int off, int val)
{
	struct n72_pmu *pmu = gpiochip_get_data(gc);

	if (off != 6)
		return;
	regmap_update_bits(pmu->regmap, N72_PMU_GPIO67_CFG, N72_PMU_GPIO6_MASK,
			   val ? N72_PMU_GPIO6_HI : N72_PMU_GPIO6_LO);
}

static int n72_pmu_gpio_get_direction(struct gpio_chip *gc, unsigned int off)
{
	struct n72_pmu *pmu = gpiochip_get_data(gc);
	unsigned int v;
	int ret;

	if (off == 2)
		return GPIO_LINE_DIRECTION_IN;
	if (off == 6) {
		ret = regmap_read(pmu->regmap, N72_PMU_GPIO67_CFG, &v);
		if (ret < 0)
			return ret;
		return (v & N72_PMU_GPIO6_OUT) ? GPIO_LINE_DIRECTION_OUT :
						 GPIO_LINE_DIRECTION_IN;
	}
	return -EOPNOTSUPP;
}

static int n72_pmu_gpio_direction_input(struct gpio_chip *gc, unsigned int off)
{
	return off == 2 ? 0 : -EOPNOTSUPP;
}

static int n72_pmu_gpio_direction_output(struct gpio_chip *gc, unsigned int off,
					 int val)
{
	struct n72_pmu *pmu = gpiochip_get_data(gc);

	if (off != 6)
		return -EOPNOTSUPP;
	return regmap_update_bits(pmu->regmap, N72_PMU_GPIO67_CFG,
				  N72_PMU_GPIO6_MASK,
				  val ? N72_PMU_GPIO6_HI : N72_PMU_GPIO6_LO);
}

static int n72_pmu_gpio_request(struct gpio_chip *gc, unsigned int off)
{
	return (off == 2 || off == 6) ? 0 : -EOPNOTSUPP;
}

static int n72_pmu_gpio_to_irq(struct gpio_chip *gc, unsigned int off)
{
	struct n72_pmu *pmu = gpiochip_get_data(gc);

	if (!pmu->irq_domain || off >= N72_PMU_NGPIO)
		return -ENXIO;
	return irq_create_mapping(pmu->irq_domain, off);
}

static void n72_pmu_irq_mask(struct irq_data *d)
{
	struct n72_pmu *pmu = irq_data_get_irq_chip_data(d);

	if (irqd_to_hwirq(d) == N72_PMU_IRQ_WLAN)
		pmu->wl_irq_on = false;
	else if (irqd_to_hwirq(d) == N72_PMU_IRQ_BT)
		pmu->bt_irq_on = false;
	else
		pmu->hp_irq_on = false;
}

static void n72_pmu_irq_unmask(struct irq_data *d)
{
	struct n72_pmu *pmu = irq_data_get_irq_chip_data(d);

	if (irqd_to_hwirq(d) == N72_PMU_IRQ_WLAN)
		pmu->wl_irq_on = true;
	else if (irqd_to_hwirq(d) == N72_PMU_IRQ_BT)
		pmu->bt_irq_on = true;
	else
		pmu->hp_irq_on = true;
}

/* the PMU reports any change of the level: both edges, nothing else */
static int n72_pmu_irq_set_type(struct irq_data *d, unsigned int type)
{
	/* the host wake line: reported for as long as it is asserted */
	if (irqd_to_hwirq(d) == N72_PMU_IRQ_WLAN)
		return (type & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_LEVEL_HIGH ||
		       (type & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_EDGE_RISING ? 0 : -EINVAL;
	/*
	 * The Bluetooth host wake line: one latched event, whichever way the
	 * consumer describes it (hci_bcm asks for one edge).
	 */
	if (irqd_to_hwirq(d) == N72_PMU_IRQ_BT)
		return (type & IRQ_TYPE_SENSE_MASK) != IRQ_TYPE_NONE ? 0 : -EINVAL;
	if (irqd_to_hwirq(d) != 2)
		return -EINVAL;
	return (type & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_EDGE_BOTH ||
	       (type & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_EDGE_RISING ||
	       (type & IRQ_TYPE_SENSE_MASK) == IRQ_TYPE_EDGE_FALLING ? 0 : -EINVAL;
}

/*
 * Which lines may wake the system: their consumers say so in their suspend
 * callbacks (enable_irq_wake()); n72_pmu_suspend() runs after those and
 * masks the other lines for the sleep.
 */
static int n72_pmu_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct n72_pmu *pmu = irq_data_get_irq_chip_data(d);

	assign_bit(irqd_to_hwirq(d), &pmu->irq_wake, on);
	return 0;
}

static void n72_pmu_irq_bus_lock(struct irq_data *d)
{
	struct n72_pmu *pmu = irq_data_get_irq_chip_data(d);

	mutex_lock(&pmu->irq_lock);
}

static void n72_pmu_irq_bus_sync_unlock(struct irq_data *d)
{
	struct n72_pmu *pmu = irq_data_get_irq_chip_data(d);

	if (pmu->hp_irq_on != pmu->hp_irq_was_on) {
		if (pmu->hp_irq_on)
			n72_pmu_int2_take(pmu, N72_PMU_INT2_HPDET);	/* stale */
		if (!regmap_update_bits(pmu->regmap, N72_PMU_INT2_MASK,
					N72_PMU_INT2_HPDET,
					pmu->hp_irq_on ? 0 : N72_PMU_INT2_HPDET) &&
		    !regmap_update_bits(pmu->regmap, N72_PMU_INT3_MASK,
					N72_PMU_INT3_JACK,
					pmu->hp_irq_on ? 0 : N72_PMU_INT3_JACK))
			WRITE_ONCE(pmu->hp_irq_was_on, pmu->hp_irq_on);
	}
	if (pmu->wl_irq_on != pmu->wl_irq_was_on) {
		if (pmu->wl_irq_on) {
			/*
			 * Drop what the line latched while it was off
			 * (it is set again at once if still asserted);
			 * the read takes the other INT1 events too.
			 */
			unsigned int v = 0;

			if (!regmap_read(pmu->regmap, N72_PMU_INT1, &v)) {
				if (v & N72_PMU_INT1_ALARM)
					blocking_notifier_call_chain(&pmu->events,
								     N72_PMU_EVENT_ALARM, NULL);
				if (v & N72_PMU_INT1_USB_IN)
					blocking_notifier_call_chain(&pmu->events,
								     N72_PMU_EVENT_USB_IN, NULL);
			}
		}
		if (!n72_pmu_int1_mask(pmu, N72_PMU_INT1_ON, !pmu->wl_irq_on))
			WRITE_ONCE(pmu->wl_irq_was_on, pmu->wl_irq_on);
	}
	if (pmu->bt_irq_on != pmu->bt_irq_was_on) {
		if (pmu->bt_irq_on)
			n72_pmu_int2_take(pmu, N72_PMU_INT2_BT);	/* stale */
		if (!regmap_update_bits(pmu->regmap, N72_PMU_INT2_MASK,
					N72_PMU_INT2_BT,
					pmu->bt_irq_on ? 0 : N72_PMU_INT2_BT))
			WRITE_ONCE(pmu->bt_irq_was_on, pmu->bt_irq_on);
	}
	mutex_unlock(&pmu->irq_lock);
}

static const struct irq_chip n72_pmu_irq_chip = {
	.name			= "n72-pmu-irq",
	.irq_mask		= n72_pmu_irq_mask,
	.irq_unmask		= n72_pmu_irq_unmask,
	.irq_set_type		= n72_pmu_irq_set_type,
	.irq_set_wake		= n72_pmu_irq_set_wake,
	.irq_bus_lock		= n72_pmu_irq_bus_lock,
	.irq_bus_sync_unlock	= n72_pmu_irq_bus_sync_unlock,
	.flags			= IRQCHIP_IMMUTABLE,
};

static int n72_pmu_irq_domain_map(struct irq_domain *d, unsigned int virq,
				  irq_hw_number_t hw)
{
	struct n72_pmu *pmu = d->host_data;

	irq_set_chip_data(virq, pmu);
	irq_set_chip_and_handler(virq, &n72_pmu_irq_chip, handle_simple_irq);
	irq_set_nested_thread(virq, true);
	irq_set_noprobe(virq);
	return 0;
}

static const struct irq_domain_ops n72_pmu_irq_domain_ops = {
	.map = n72_pmu_irq_domain_map,
	.xlate = irq_domain_xlate_twocell,
};

static void n72_pmu_irq_domain_remove(void *data)
{
	irq_domain_remove(data);
}

static int n72_pmu_gpio(struct n72_pmu *pmu)
{
	struct device *dev = &pmu->client->dev;

	if (!of_property_read_bool(dev->of_node, "gpio-controller"))
		return 0;
	pmu->gc.label = "n72-pmu";
	pmu->gc.parent = dev;
	pmu->gc.owner = THIS_MODULE;
	pmu->gc.base = -1;
	pmu->gc.ngpio = N72_PMU_NGPIO;
	pmu->gc.can_sleep = true;
	pmu->gc.request = n72_pmu_gpio_request;
	pmu->gc.get = n72_pmu_gpio_get;
	pmu->gc.set = n72_pmu_gpio_set;
	pmu->gc.get_direction = n72_pmu_gpio_get_direction;
	pmu->gc.direction_input = n72_pmu_gpio_direction_input;
	pmu->gc.direction_output = n72_pmu_gpio_direction_output;
	pmu->gc.to_irq = n72_pmu_gpio_to_irq;
	/*
	 * GPIO 7 carries the Mikey chip's interrupt. iOS activates it with
	 * configuration 9 (gpio-activate-defaults <.. f9>, 0x96 in its PMU
	 * dumps); the boot chain leaves 0xc.
	 */
	regmap_update_bits(pmu->regmap, N72_PMU_GPIO67_CFG, 0xf0, 0x90);
	return devm_gpiochip_add_data(dev, &pmu->gc, pmu);
}

/*
 * Unmask (or mask) the USB plug and unplug interrupts. Once they have been
 * unmasked, INT3 is read for the unplug at every interrupt.
 */
void n72_pmu_usb_irq(struct n72_pmu *pmu, bool on)
{
	unsigned int v;

	mutex_lock(&pmu->irq_lock);
	if (on) {
		WRITE_ONCE(pmu->usb_irq_on, true);
		/* drop what latched while masked; the state is read afresh */
		regmap_read(pmu->regmap, N72_PMU_INT3, &v);
	}
	n72_pmu_int1_mask(pmu, N72_PMU_INT1_USB_IN, !on);
	regmap_update_bits(pmu->regmap, N72_PMU_INT3_MASK, N72_PMU_INT3_USB_OUT,
			   on ? 0 : N72_PMU_INT3_USB_OUT);
	mutex_unlock(&pmu->irq_lock);
}
EXPORT_SYMBOL_GPL(n72_pmu_usb_irq);

/*
 * Asleep, a line stays unmasked only if its consumer asked for the wake
 * (n72_pmu_irq_set_wake()): the Wi-Fi and Bluetooth host wake lines when
 * their drivers arm them, the headphone plug never.
 */
static void n72_pmu_lines_sleep(struct n72_pmu *pmu)
{
	u8 m2;

	if (!pmu->irq_domain)
		return;
	mutex_lock(&pmu->irq_lock);
	pmu->hp_slept = pmu->hp_irq_was_on && !test_bit(2, &pmu->irq_wake);
	pmu->wl_slept = pmu->wl_irq_was_on && !test_bit(N72_PMU_IRQ_WLAN, &pmu->irq_wake);
	pmu->bt_slept = pmu->bt_irq_was_on && !test_bit(N72_PMU_IRQ_BT, &pmu->irq_wake);
	m2 = (pmu->hp_slept ? N72_PMU_INT2_HPDET : 0) | (pmu->bt_slept ? N72_PMU_INT2_BT : 0);
	if (m2)
		regmap_update_bits(pmu->regmap, N72_PMU_INT2_MASK, m2, m2);
	if (pmu->hp_slept)
		regmap_update_bits(pmu->regmap, N72_PMU_INT3_MASK,
				   N72_PMU_INT3_JACK, N72_PMU_INT3_JACK);
	if (pmu->wl_slept)
		n72_pmu_int1_mask(pmu, N72_PMU_INT1_ON, true);
	mutex_unlock(&pmu->irq_lock);
}

static void n72_pmu_lines_wake(struct n72_pmu *pmu)
{
	bool hp;
	u8 m2;

	if (!pmu->irq_domain)
		return;
	mutex_lock(&pmu->irq_lock);
	hp = pmu->hp_slept;
	m2 = (hp ? N72_PMU_INT2_HPDET : 0) | (pmu->bt_slept ? N72_PMU_INT2_BT : 0);
	if (hp)
		regmap_update_bits(pmu->regmap, N72_PMU_INT3_MASK,
				   N72_PMU_INT3_JACK, 0);
	if (m2)
		regmap_update_bits(pmu->regmap, N72_PMU_INT2_MASK, m2, 0);
	if (pmu->wl_slept)
		n72_pmu_int1_mask(pmu, N72_PMU_INT1_ON, false);
	pmu->hp_slept = false;
	pmu->wl_slept = false;
	pmu->bt_slept = false;
	mutex_unlock(&pmu->irq_lock);

	/* (the handlers may enable or disable their lines: not under the lock) */
	if (hp) {
		/* report a plug change made while asleep */
		n72_pmu_int2_take(pmu, N72_PMU_INT2_HPDET);
		n72_pmu_line_irq(pmu, 2);
	}
	/* a host wake that n72_pmu_irq() took before its line was resumed */
	if (pmu->bt_irq_was_on && n72_pmu_int2_take(pmu, N72_PMU_INT2_BT) > 0 &&
	    !n72_pmu_line_irq(pmu, N72_PMU_IRQ_BT))
		n72_pmu_int2_keep(pmu, N72_PMU_INT2_BT);
}

/*
 * Late, after the consumers' suspend (they ask for their lines' wake in it)
 * and after the sub-devices' late suspend, in which the power supply has
 * stopped the gauge and masked the USB plug.
 */
static int n72_pmu_suspend(struct device *dev)
{
	struct n72_pmu *pmu = dev_get_drvdata(dev);
	unsigned int v;
	int i;

	n72_pmu_lines_sleep(pmu);

	for (i = 0; i < ARRAY_SIZE(n72_ldo_saved); i++)
		pmu->ldo_regs[i] = regmap_read(pmu->regmap, n72_ldo_saved[i], &v) ? 0 : v;
	return 0;
}

static int n72_pmu_resume(struct device *dev)
{
	struct n72_pmu *pmu = dev_get_drvdata(dev);
	unsigned int v;
	int i;

	for (i = 0; i < ARRAY_SIZE(n72_ldo_saved); i++) {
		if (!regmap_read(pmu->regmap, n72_ldo_saved[i], &v) &&
		    v != pmu->ldo_regs[i]) {
			dev_dbg(dev, "PMU %02x: %02x after sleep, restoring %02x\n",
				n72_ldo_saved[i], v, pmu->ldo_regs[i]);
			regmap_write(pmu->regmap, n72_ldo_saved[i], pmu->ldo_regs[i]);
		}
	}
	n72_pmu_lines_wake(pmu);
	/* (the power supply's early resume comes next: USB plug, gauge) */
	return 0;
}

static const struct dev_pm_ops n72_pmu_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(n72_pmu_suspend, n72_pmu_resume)
};

/* ------------------------------------------------ for the sub-devices */

struct n72_pmu_notifier {
	struct n72_pmu *pmu;
	struct notifier_block *nb;
};

static void n72_pmu_notifier_release(struct device *dev, void *res)
{
	struct n72_pmu_notifier *n = res;

	blocking_notifier_chain_unregister(&n->pmu->events, n->nb);
}

int devm_n72_pmu_notifier(struct device *dev, struct n72_pmu *pmu,
			  struct notifier_block *nb)
{
	struct n72_pmu_notifier *n;
	int ret;

	if (pmu->client->irq <= 0)
		return -ENXIO;
	n = devres_alloc(n72_pmu_notifier_release, sizeof(*n), GFP_KERNEL);
	if (!n)
		return -ENOMEM;
	ret = blocking_notifier_chain_register(&pmu->events, nb);
	if (ret) {
		devres_free(n);
		return ret;
	}
	n->pmu = pmu;
	n->nb = nb;
	devres_add(dev, n);
	return 0;
}
EXPORT_SYMBOL_GPL(devm_n72_pmu_notifier);

/* The alarm starts off, as the RTC turns it off (0x5a's "on" bit, then INT1's mask). */
static void n72_pmu_alarm_off(struct n72_pmu *pmu)
{
	if (!regmap_update_bits(pmu->regmap, N72_PMU_RTC_ALARM_Y,
				N72_PMU_RTC_ALARM_ON, 0))
		n72_pmu_int1_mask(pmu, N72_PMU_INT1_ALARM, true);
}

static const struct mfd_cell n72_pmu_cells[] = {
	MFD_CELL_NAME("n72-pmu-backlight"),
	MFD_CELL_NAME("n72-pmu-rtc"),
	MFD_CELL_NAME("n72-pmu-regulator"),
	MFD_CELL_NAME("n72-pmu-power"),
};

static const struct regmap_config n72_pmu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7f,
	.cache_type = REGCACHE_NONE,
};

static int n72_pmu_probe(struct i2c_client *client)
{
	struct n72_pmu *pmu;
	unsigned int id;
	int ret;

	pmu = devm_kzalloc(&client->dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;
	pmu->client = client;
	mutex_init(&pmu->int2_lock);
	mutex_init(&pmu->irq_lock);
	BLOCKING_INIT_NOTIFIER_HEAD(&pmu->events);

	pmu->regmap = devm_regmap_init_i2c(client, &n72_pmu_regmap_config);
	if (IS_ERR(pmu->regmap))
		return PTR_ERR(pmu->regmap);

	ret = regmap_read(pmu->regmap, 0x00, &id);
	if (ret)
		return dev_err_probe(&client->dev, ret, "PMU not responding\n");

	if (of_property_read_bool(client->dev.of_node, "system-power-controller")) {
		ret = devm_register_sys_off_handler(&client->dev,
						    SYS_OFF_MODE_POWER_OFF,
						    SYS_OFF_PRIO_DEFAULT,
						    n72_pmu_power_off, pmu);
		if (ret)
			return ret;
	}

	i2c_set_clientdata(client, pmu);
	if (client->irq > 0) {
		pmu->irq_domain = irq_domain_add_linear(client->dev.of_node,
							N72_PMU_IRQ_BT + 1,
							&n72_pmu_irq_domain_ops,
							pmu);
		if (!pmu->irq_domain)
			return -ENOMEM;
		ret = devm_add_action_or_reset(&client->dev,
					       n72_pmu_irq_domain_remove,
					       pmu->irq_domain);
		if (ret)
			return ret;

		/* the alarm starts off; nothing else is unmasked here */
		n72_pmu_alarm_off(pmu);
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						n72_pmu_irq, IRQF_ONESHOT,
						"n72-pmu", pmu);
		if (ret)
			return ret;
		device_init_wakeup(&client->dev, true);
		dev_pm_set_wake_irq(&client->dev, client->irq);
	}

	ret = n72_pmu_gpio(pmu);
	if (ret)
		return dev_err_probe(&client->dev, ret, "GPIOs\n");

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_NONE, n72_pmu_cells,
				   ARRAY_SIZE(n72_pmu_cells), NULL, 0, NULL);
	if (ret)
		return ret;

	dev_info(&client->dev, "PMU id 0x%02x\n", id);
	return 0;
}

static const struct of_device_id n72_pmu_of_match[] = {
	{ .compatible = "apple,n72-pmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, n72_pmu_of_match);

static struct i2c_driver n72_pmu_driver = {
	.driver = {
		.name = "apple-n72-pmu",
		.of_match_table = n72_pmu_of_match,
		.pm = pm_sleep_ptr(&n72_pmu_pm_ops),
	},
	.probe = n72_pmu_probe,
};
module_i2c_driver(n72_pmu_driver);

MODULE_DESCRIPTION("Apple iPod touch 2G (Dialog D1759) PMU core: interrupt, GPIOs, power-off");
MODULE_LICENSE("GPL");
