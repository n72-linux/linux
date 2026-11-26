// SPDX-License-Identifier: GPL-2.0-only
/*
 * iPod touch 2G: the low core voltage state.
 *
 * Hardware clock/power rules keep two core voltage states: 1.20 V, the only
 * one at which the system clock may run undivided, and 1.05 V, with the
 * system clock capped at /2. It drops to 1.05 V whenever the clocks of the
 * display (CLCD, LCDIF, MIPI), TV-out and the USB PHY are all closed, and
 * goes back to 1.20 V, waiting 150 us, before any of them opens. The rail
 * is the PMU's DOWN0 converter, this device's "core" supply.
 *
 * Here: low while the system runs with the display off and no USB power. USB
 * power stands in for the USB PHY, whose clock Linux does not gate when the
 * cable is out. Every system sleep is entered and left at 1.20 V (the boot
 * loader's wake path after suspend to RAM assumes it, and suspend to idle
 * has not been tried at 1.05 V yet), and so is a restart or kexec.
 *
 * Ordering: switch system clock to /2 first, then lower voltage; raise voltage
 * and wait for settling first, then switch clock back to /1.
 *
 * A crash or reset while low leaves the PMU at 1.05 V for the boot chain,
 * which is fine for the boot ROM/chain reset path.
 *
 * The state below is kept from boot on: the display and USB power reported
 * before the device has its supply (and probes) are acted on once it has.
 */

#include <linux/clk/s5l8720.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/apple/n72-lcd.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>

#define N72_COREV_HIGH_UV	1200000
#define N72_COREV_LOW_UV	1050000
/* stay high this long after the last reason for it went away */
#define N72_COREV_LOWER_DELAY	(2 * HZ)

static DEFINE_MUTEX(corev_lock);
static struct regulator *corev_rail;	/* DOWN0 */
static bool corev_ready;	/* probed, notifiers registered */
static bool corev_display = true;
static bool corev_usb = true;	/* until the supply has been asked */
static bool corev_sleep;
static bool corev_down;
static bool corev_low;

static void corev_lower_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(corev_lower_work, corev_lower_fn);

static bool corev_wants_high(void)
{
	return corev_display || corev_usb || corev_sleep || corev_down;
}

/* lock held; sleeps */
static void corev_raise(void)
{
	int ret;

	if (!corev_low)
		return;
	ret = regulator_set_voltage(corev_rail, N72_COREV_HIGH_UV, N72_COREV_HIGH_UV);
	/*
	 * A lowering that failed may have left the rail low with 1.20 V still
	 * the request, which the call above then does not write again.
	 */
	if (!ret && regulator_get_voltage(corev_rail) != N72_COREV_HIGH_UV)
		ret = regulator_sync_voltage(corev_rail);
	if (ret) {
		/* still at /2, which is right for whatever the rail is at */
		pr_err("n72-corev: the core voltage could not be raised (%d)\n", ret);
		return;
	}
	s5l8720_idle_set_active_div(1);
	corev_low = false;
}

static void corev_lower_fn(struct work_struct *work)
{
	int ret;

	mutex_lock(&corev_lock);
	if (corev_wants_high() || corev_low)
		goto out;
	ret = s5l8720_idle_set_active_div(2);
	if (ret)
		goto out;
	ret = regulator_set_voltage(corev_rail, N72_COREV_LOW_UV, N72_COREV_LOW_UV);
	if (ret && regulator_get_voltage(corev_rail) == N72_COREV_HIGH_UV) {
		pr_err("n72-corev: the core voltage could not be lowered (%d)\n", ret);
		s5l8720_idle_set_active_div(1);
		goto out;
	}
	/* lowered, or unknown: treat it as low and stay at /2 */
	corev_low = true;
out:
	mutex_unlock(&corev_lock);
}

/* lock held; sleeps when it has to raise */
static void corev_update(void)
{
	if (!corev_ready)
		return;
	if (corev_wants_high())
		corev_raise();
	else if (!corev_low)
		schedule_delayed_work(&corev_lower_work, N72_COREV_LOWER_DELAY);
}

void n72_corev_display(bool on)
{
	mutex_lock(&corev_lock);
	corev_display = on;
	corev_update();
	mutex_unlock(&corev_lock);
}
EXPORT_SYMBOL_GPL(n72_corev_display);

/* USB power came or went: the supply is asked from a work (it sleeps) */
static void corev_usb_fn(struct work_struct *work)
{
	struct power_supply *psy = power_supply_get_by_name("usb");
	union power_supply_propval val = { .intval = 1 };

	if (!psy)
		return;			/* stays "present": high */
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val))
		val.intval = 1;
	power_supply_put(psy);

	mutex_lock(&corev_lock);
	corev_usb = val.intval;
	corev_update();
	mutex_unlock(&corev_lock);
}

static DECLARE_WORK(corev_usb_work, corev_usb_fn);

static int corev_psy_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED && psy->desc->type == POWER_SUPPLY_TYPE_USB)
		schedule_work(&corev_usb_work);
	return NOTIFY_DONE;
}

static struct notifier_block corev_psy_nb = { .notifier_call = corev_psy_event };

static int corev_pm_event(struct notifier_block *nb, unsigned long event, void *data)
{
	bool low;

	switch (event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		mutex_lock(&corev_lock);
		corev_sleep = true;
		corev_update();
		low = corev_low;
		if (low)
			corev_sleep = false;	/* no POST for a refused sleep */
		mutex_unlock(&corev_lock);
		/*
		 * Never into a sleep at 1.05 V: the boot loader's wake path
		 * after suspend to RAM runs undivided.
		 */
		if (low)
			return notifier_from_errno(-EIO);
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		mutex_lock(&corev_lock);
		corev_sleep = false;
		corev_update();
		mutex_unlock(&corev_lock);
		break;
	}
	return NOTIFY_DONE;
}

/*
 * First of the sleep notifiers: the others (Bluetooth sends its commands to
 * the controller from one) then run at 1.20 V and /1, as they always have.
 */
static struct notifier_block corev_pm_nb = {
	.notifier_call = corev_pm_event,
	.priority = INT_MAX,
};

/* restart, power-off, kexec: the next code expects 1.20 V and /1 */
static int corev_reboot_event(struct notifier_block *nb, unsigned long event, void *data)
{
	mutex_lock(&corev_lock);
	corev_down = true;
	corev_update();
	mutex_unlock(&corev_lock);
	return NOTIFY_DONE;
}

static struct notifier_block corev_reboot_nb = {
	.notifier_call = corev_reboot_event,
	.priority = INT_MAX,
};

static int n72_corev_probe(struct platform_device *pdev)
{
	struct regulator *rail;

	if (corev_ready)
		return -EBUSY;		/* one core rail, one set of notifiers */
	rail = devm_regulator_get(&pdev->dev, "core");
	if (IS_ERR(rail))
		return dev_err_probe(&pdev->dev, PTR_ERR(rail), "core supply\n");

	power_supply_reg_notifier(&corev_psy_nb);
	register_pm_notifier(&corev_pm_nb);
	register_reboot_notifier(&corev_reboot_nb);
	mutex_lock(&corev_lock);
	corev_rail = rail;
	corev_ready = true;
	mutex_unlock(&corev_lock);
	/* USB power as it is now; the display's state was kept from boot on */
	schedule_work(&corev_usb_work);
	return 0;
}

static const struct of_device_id n72_corev_of_match[] = {
	{ .compatible = "apple,n72-core-voltage" },
	{ }
};

static struct platform_driver n72_corev_driver = {
	.driver = {
		.name = "n72-corev",
		.of_match_table = n72_corev_of_match,
		.suppress_bind_attrs = true,	/* the notifiers stay */
	},
	.probe = n72_corev_probe,
};
builtin_platform_driver(n72_corev_driver);
