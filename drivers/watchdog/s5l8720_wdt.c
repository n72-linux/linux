// SPDX-License-Identifier: GPL-2.0
/*
 * Watchdog of the Samsung S5L8720 (iPod touch 2G).
 *
 * One control register (the syscon at 0x3c800000 that also serves
 * syscon-reboot) and an 11-bit counter at +4:
 *
 *  - 0x001f4a00 to start and restart the count (enable, prescaler 15,
 *    clock select 4, clear);
 *  - 0 to stop;
 *  - 0x00100000 (enable with no divider) to reset the SoC immediately.
 *
 * The watchdog counter is kicked every 2^25 bus clock cycles (0.25 s at
 * 133 MHz); the watchdog core is told max_hw_heartbeat_ms = 500 ms so it
 * pings within that window while exposing standard multi-second timeouts to
 * userspace.
 *
 * The watchdog only runs once /dev/watchdog is opened. It is stopped
 * across system sleep and before a restart or kexec so a next kernel is not
 * reset before its driver initializes.
 */

#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/watchdog.h>

#define S5L8720_WDTCON		0x0
#define WDTCON_ENABLE		BIT(20)
#define WDTCON_PRE(n)		((n) << 16)	/* 4 bits */
#define WDTCON_CS(n)		((n) << 12)	/* 3 bits */
#define WDTCON_CLR		0xa00
#define WDTCON_RUN		(WDTCON_ENABLE | WDTCON_PRE(15) | WDTCON_CS(4) | \
				 WDTCON_CLR)

/* iOS's interval between two restarts of the count, in bus clock cycles */
#define S5L8720_WDT_PET_CYCLES	BIT_ULL(25)

struct s5l8720_wdt {
	struct watchdog_device wdd;
	struct regmap *map;
	struct notifier_block pm_nb;
	struct mutex lock;
	bool suspended;
};

static int s5l8720_wdt_ping(struct watchdog_device *wdd)
{
	struct s5l8720_wdt *w = watchdog_get_drvdata(wdd);
	int ret = 0;

	mutex_lock(&w->lock);
	if (!w->suspended)
		ret = regmap_write(w->map, S5L8720_WDTCON, WDTCON_RUN);
	mutex_unlock(&w->lock);
	return ret;
}

static int s5l8720_wdt_stop(struct watchdog_device *wdd)
{
	struct s5l8720_wdt *w = watchdog_get_drvdata(wdd);
	int ret;

	mutex_lock(&w->lock);
	ret = regmap_write(w->map, S5L8720_WDTCON, 0);
	mutex_unlock(&w->lock);
	return ret;
}

static const struct watchdog_ops s5l8720_wdt_ops = {
	.owner	= THIS_MODULE,
	.start	= s5l8720_wdt_ping,
	.stop	= s5l8720_wdt_stop,
	.ping	= s5l8720_wdt_ping,
};

static const struct watchdog_info s5l8720_wdt_info = {
	.identity = "S5L8720 watchdog",
	.options  = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

/*
 * Runs after the watchdog core's PM notifier (priority 0) so the hardware
 * counter stops as soon as the core cancels its ping worker and only restarts
 * after PM_POST_SUSPEND re-arms the worker.
 */
static int s5l8720_wdt_pm_notify(struct notifier_block *nb,
				 unsigned long mode, void *data)
{
	struct s5l8720_wdt *w = container_of(nb, struct s5l8720_wdt, pm_nb);
	int ret = 0;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
	case PM_SUSPEND_PREPARE:
		mutex_lock(&w->lock);
		w->suspended = true;
		if (watchdog_active(&w->wdd))
			ret = regmap_write(w->map, S5L8720_WDTCON, 0);
		mutex_unlock(&w->lock);
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		mutex_lock(&w->lock);
		w->suspended = false;
		if (watchdog_active(&w->wdd))
			ret = regmap_write(w->map, S5L8720_WDTCON, WDTCON_RUN);
		mutex_unlock(&w->lock);
		break;
	}

	return ret ? NOTIFY_BAD : NOTIFY_DONE;
}

static void s5l8720_wdt_pm_unreg(void *data)
{
	unregister_pm_notifier(data);
}

static int s5l8720_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s5l8720_wdt *w;
	u32 rate;
	int ret;

	if (!dev->parent)
		return -ENODEV;
	w = devm_kzalloc(dev, sizeof(*w), GFP_KERNEL);
	if (!w)
		return -ENOMEM;
	w->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(w->map))
		return dev_err_probe(dev, PTR_ERR(w->map), "no control register\n");
	if (of_property_read_u32(dev->of_node, "clock-frequency", &rate) || !rate)
		return dev_err_probe(dev, -EINVAL, "no bus clock rate\n");

	mutex_init(&w->lock);
	w->wdd.parent = dev;
	w->wdd.info = &s5l8720_wdt_info;
	w->wdd.ops = &s5l8720_wdt_ops;
	w->wdd.min_timeout = 1;
	w->wdd.timeout = 30;
	w->wdd.max_hw_heartbeat_ms =
		2 * (u32)div_u64(S5L8720_WDT_PET_CYCLES * MSEC_PER_SEC, rate);
	if (!w->wdd.max_hw_heartbeat_ms)
		return dev_err_probe(dev, -EINVAL, "bus clock rate %u too high\n", rate);
	watchdog_set_drvdata(&w->wdd, w);
	watchdog_init_timeout(&w->wdd, 0, dev);
	watchdog_stop_on_reboot(&w->wdd);
	/* PM notifier stops hardware counter right after core stops ping worker and restarts it after PM_POST_SUSPEND */
	watchdog_stop_ping_on_suspend(&w->wdd);
	watchdog_stop_on_unregister(&w->wdd);
	platform_set_drvdata(pdev, w);

	ret = devm_watchdog_register_device(dev, &w->wdd);
	if (ret)
		return ret;

	w->pm_nb.notifier_call = s5l8720_wdt_pm_notify;
	w->pm_nb.priority = -1;
	ret = register_pm_notifier(&w->pm_nb);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, s5l8720_wdt_pm_unreg, &w->pm_nb);
}

static const struct of_device_id s5l8720_wdt_of_match[] = {
	{ .compatible = "apple,s5l8720-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8720_wdt_of_match);

static struct platform_driver s5l8720_wdt_driver = {
	.probe	= s5l8720_wdt_probe,
	.driver	= {
		.name		= "s5l8720-wdt",
		.of_match_table	= s5l8720_wdt_of_match,
	},
};
module_platform_driver(s5l8720_wdt_driver);

MODULE_DESCRIPTION("Samsung S5L8720 watchdog");
MODULE_LICENSE("GPL");
