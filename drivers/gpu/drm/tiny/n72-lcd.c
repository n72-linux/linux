// SPDX-License-Identifier: GPL-2.0-only
/*
 * Display pipe of the iPod touch 2G (N72) across system sleep: the S5L8720
 * display controller ("CLCD", 0x38900000), its MIPI DSI master (0x3d800000)
 * and the Pinot panel behind it.
 *
 * The boot loader leaves the display running and Linux draws through
 * simplefb, so nothing is written at boot: the driver records the running
 * configuration. On the way into sleep the panel is shut down, and after a
 * suspend to RAM (the SoC lost power; LLB does not bring the display up on
 * a wake) it is brought up again cleanly:
 *
 *  - CLCD power-on: reset pulse (0x100 set, 1 ms, cleared), saved
 *    configuration written back (0x400 off, then 0x3c, 0x5c, 0x114,
 *    window regs, 0x300, 0x304, 0x1b10..0x1b24), 0x1b2c = 0xf, enable bit.
 *  - MIPI DSI controller power-on: saved clock, PLL, resolution, porch and
 *    config registers, software reset, then lane stop state -> ULPS ->
 *    configured -> out of ULPS. Short packets write PKTHDR, wait 1 us, and
 *    poll lane 0 stop state.
 *  - Panel bring-up: panel LDO enabled, >= 10 ms while DSI master comes up,
 *    reset released, 6 ms, sleep out (0x11) in LP, HS clock, video (class 1
 *    panels first), scan-out, display on (0x29) 134/100 ms after sleep out,
 *    then 101/67 ms (class 2: video after). Shutdown reverses the sequence:
 *    video off, 0x28, 0x10, 100 ms, HS clock off, reset asserted, LDO off.
 *
 * The panel supply is the PMU's LDO4 at 3.0 V ("power" supply), and the
 * panel ID (class) comes from DT property apple,panel-id.
 *
 * While the system runs, the panel can also be powered down and the CLCD
 * and DSI clocks gated (sysfs panel_power): the other users of the CLCD
 * registers are told first through the client notifier
 * (include/linux/soc/apple/n72-lcd.h).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/soc/apple/n72-lcd.h>

/* display controller */
#define CLCD_CTRL	0x0000	/* bit 0 enable, bit 1 stopped, bit 8 reset */
#define CLCD_LUT_EN	0x0400
#define CLCD_VIDCON0	0x1b10	/* bit 0 scan-out */
#define CLCD_IRQCLR	0x1b2c

/* registers saved across sleep and restored in order */
static const u16 clcd_regs[] = {
	0x3c, 0x5c, 0x114,
	0x04, 0x10, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38,	/* window 0 */
	0x300, 0x304,
	0x1b10, 0x1b14, 0x1b18, 0x1b1c, 0x1b20, 0x1b24,
	/*
	 * Layer 1 (n72-clcd's second device), not in iOS's list: iOS programs
	 * it again at its next swap. Its enable is in 0x04 above, which was
	 * written before; the layer shows nothing until scan-out starts.
	 */
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58,
};

/* DSI master */
#define DSIM_STATUS	0x00
#define  ST_LANE0_STOP	BIT(0)
#define  ST_STOP	BIT(8)
#define  ST_ULPS	BIT(9)
#define  ST_HS_READY	BIT(10)
#define  ST_SWRST	BIT(20)
#define  ST_PLL_STABLE	BIT(31)
#define DSIM_SWRST	0x04
#define DSIM_CLKCTRL	0x08
#define  CK_TX_HSCLK	BIT(31)
#define DSIM_CONFIG	0x10
#define DSIM_ESCMODE	0x14
#define DSIM_MDRESOL	0x18	/* bit 31: video */
#define DSIM_MVPORCH	0x1c
#define DSIM_MHPORCH	0x20
#define DSIM_MSYNC	0x24
#define DSIM_SDRESOL	0x28
#define DSIM_PKTHDR	0x34
#define DSIM_FIFOTHLD	0x40
#define DSIM_FIFOCTRL	0x44
#define DSIM_PLLCTRL	0x4c
#define  PLL_EN		BIT(23)
#define DSIM_PLLTMR	0x50
#define DSIM_PHYACCHR	0x54
#define DSIM_PHYACCHR1	0x58

#define N72_LANES	0x3	/* two data lanes (iOS #lanes = 2) */
#define DCS_SHORT_WRITE	0x05

struct n72_dsim_regs {
	u32 clkctrl, config, mdresol, mvporch, mhporch, msync, sdresol;
	u32 pllctrl, plltmr, phyacchr, phyacchr1;
};

struct n72_lcd {
	struct device *dev;
	void __iomem *clcd;
	void __iomem *dsim;
	struct gpio_desc *reset;
	struct regulator *supply;
	bool supply_on;
	u32 panel_id;
	int panel_class;		/* 1 or 2, as iOS */

	u32 ctrl;			/* CLCD control as running, enable/reset bits off */
	u32 clcd_saved[ARRAY_SIZE(clcd_regs)];
	struct n72_dsim_regs dsim_saved;
	bool panel_off;
	struct backlight_device *bl;	/* off while the panel comes back up */
	int bl_power;			/* its power state before the sleep */
	bool dark;			/* blanked early, at PM_SUSPEND_PREPARE */
	struct notifier_block pm_nb;
	/*
	 * CLCD and MIPI DSIM clock gates ("clocks"): on from probe, off from
	 * suspend_late to resume_early while the panel is powered down in a
	 * sleep (iOS gates them whenever the display is off). Late, so that
	 * every user of the CLCD registers (n72-clcd, its vblank timer) has
	 * stopped: a gated block must not be read.
	 */
	struct clk_bulk_data *clks;
	int nclks;
	bool clks_off;
	/*
	 * Powered down and gated while the system runs (panel_power = 0).
	 * Across a system sleep it stays that way: nothing here touches the
	 * gated registers, and only panel_power = 1 brings the panel back.
	 */
	bool blanked;
	struct mutex blank_lock;	/* serialises panel_power writes */
};

static BLOCKING_NOTIFIER_HEAD(n72_lcd_clients);

int n72_lcd_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&n72_lcd_clients, nb);
}
EXPORT_SYMBOL_GPL(n72_lcd_register_client);

void n72_lcd_unregister_client(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&n72_lcd_clients, nb);
}
EXPORT_SYMBOL_GPL(n72_lcd_unregister_client);

static int poll_set(void __iomem *reg, u32 mask, u32 want)
{
	u32 v;

	return readl_poll_timeout(reg, v, (v & mask) == want, 10, 100000);
}

static void sleep_until(ktime_t deadline)
{
	s64 us = ktime_us_delta(deadline, ktime_get());

	if (us > 0)
		fsleep(us);
}

/* ---------------------------------------------------------- panel LDO */

static int panel_ldo(struct n72_lcd *l, bool on)
{
	int ret = 0;

	if (on == l->supply_on)
		return 0;
	ret = on ? regulator_enable(l->supply) : regulator_disable(l->supply);
	if (!ret)
		l->supply_on = on;
	return ret;
}

/* ---------------------------------------------------------------- DSI */

static int dsi_short(struct n72_lcd *l, u8 id, u8 d0, u8 d1)
{
	writel((id & 0x3f) | d0 << 8 | d1 << 16, l->dsim + DSIM_PKTHDR);
	udelay(1);
	return poll_set(l->dsim + DSIM_STATUS, ST_LANE0_STOP, ST_LANE0_STOP);
}

static int dcs(struct n72_lcd *l, u8 cmd)
{
	return dsi_short(l, DCS_SHORT_WRITE, cmd, 0);
}

static int dsi_hs_clock(struct n72_lcd *l, bool on)
{
	u32 v = readl(l->dsim + DSIM_CLKCTRL);

	writel(on ? v | CK_TX_HSCLK : v & ~CK_TX_HSCLK, l->dsim + DSIM_CLKCTRL);
	return poll_set(l->dsim + DSIM_STATUS, ST_HS_READY, on ? ST_HS_READY : 0);
}

static void dsi_video(struct n72_lcd *l, bool on)
{
	u32 v = l->dsim_saved.mdresol;

	writel(on ? v | BIT(31) : v, l->dsim + DSIM_MDRESOL);
}

/* DSI power on with the registers the boot loader left configured */
static int dsi_power_on(struct n72_lcd *l)
{
	const struct n72_dsim_regs *r = &l->dsim_saved;
	void __iomem *d = l->dsim;
	u32 pll = r->pllctrl;
	int ret;

	writel(r->clkctrl & 0x1e00ffff, d + DSIM_CLKCTRL);
	writel(pll & 0xf000000, d + DSIM_PLLCTRL);
	if (pll & PLL_EN) {
		writel(r->phyacchr, d + DSIM_PHYACCHR);
		writel(r->phyacchr1, d + DSIM_PHYACCHR1);
		writel(r->plltmr, d + DSIM_PLLTMR);
		writel(pll & ~PLL_EN, d + DSIM_PLLCTRL);
		writel(pll, d + DSIM_PLLCTRL);
		ret = poll_set(d + DSIM_STATUS, ST_PLL_STABLE, ST_PLL_STABLE);
		if (ret)
			return ret;
	} else if (pll & 0xf0ffffff) {
		writel(pll, d + DSIM_PLLCTRL);
	}

	writel(1, d + DSIM_SWRST);
	writel(pll & ~PLL_EN, d + DSIM_PLLCTRL);
	writel(r->phyacchr, d + DSIM_PHYACCHR);
	writel(r->phyacchr1, d + DSIM_PHYACCHR1);
	writel(pll, d + DSIM_PLLCTRL);
	ret = poll_set(d + DSIM_STATUS, ST_SWRST, ST_SWRST);
	if (ret)
		return ret;

	writel(r->mdresol, d + DSIM_MDRESOL);
	writel(r->mvporch, d + DSIM_MVPORCH);
	writel(r->mhporch, d + DSIM_MHPORCH);
	writel(r->msync, d + DSIM_MSYNC);
	writel(r->sdresol, d + DSIM_SDRESOL);
	writel((r->config & 0xffffff81) | N72_LANES << 1 | 1 << 5, d + DSIM_CONFIG);
	writel((r->clkctrl & 0x7f0fffff) | N72_LANES << 20, d + DSIM_CLKCTRL);
	writel(0x1f, d + DSIM_FIFOCTRL);
	writel(0x1ff, d + DSIM_FIFOTHLD);

	writel(0x100080, d + DSIM_ESCMODE);		/* force stop, command LP */
	msleep(1);
	writel(0x80, d + DSIM_ESCMODE);
	ret = poll_set(d + DSIM_STATUS, ST_STOP | N72_LANES, ST_STOP | N72_LANES);
	if (ret)
		return ret;
	writel(0x8a, d + DSIM_ESCMODE);			/* lanes into ULPS */
	ret = poll_set(d + DSIM_STATUS, ST_ULPS | N72_LANES << 4, ST_ULPS | N72_LANES << 4);
	if (ret)
		return ret;
	writel(r->config, d + DSIM_CONFIG);
	writel(r->clkctrl & ~CK_TX_HSCLK, d + DSIM_CLKCTRL);
	writel(0x8f, d + DSIM_ESCMODE);			/* out of ULPS */
	ret = poll_set(d + DSIM_STATUS, ST_ULPS | N72_LANES << 4, 0);
	if (ret)
		return ret;
	writel(0x8a, d + DSIM_ESCMODE);
	msleep(1);
	writel(0x80, d + DSIM_ESCMODE);
	return poll_set(d + DSIM_STATUS, ST_STOP | N72_LANES, ST_STOP | N72_LANES);
}

/* ------------------------------------------------------------- CLCD */

/* Power on: reset pulse, saved configuration, bypass colour LUTs, enable */
static int clcd_power_on(struct n72_lcd *l)
{
	void __iomem *c = l->clcd;
	int i;

	writel(readl(c + CLCD_CTRL) | 0x100, c + CLCD_CTRL);
	msleep(1);
	writel(readl(c + CLCD_CTRL) & ~0x100, c + CLCD_CTRL);
	writel(l->ctrl, c + CLCD_CTRL);

	writel(0, c + CLCD_LUT_EN);
	for (i = 0; i < ARRAY_SIZE(clcd_regs); i++) {
		u32 v = l->clcd_saved[i];

		if (clcd_regs[i] == CLCD_VIDCON0)
			v &= ~1;	/* scan-out starts with the panel */
		writel(v, c + clcd_regs[i]);
	}
	writel(0xf, c + CLCD_IRQCLR);
	writel(readl(c + CLCD_CTRL) | 1, c + CLCD_CTRL);
	return poll_set(c + CLCD_CTRL, 1, 1);
}

static void clcd_scan(struct n72_lcd *l, bool on)
{
	u32 v = readl(l->clcd + CLCD_VIDCON0);

	writel(on ? v | 1 : v & ~1, l->clcd + CLCD_VIDCON0);
}

static void clcd_power_off(struct n72_lcd *l)
{
	clcd_scan(l, false);
	msleep(20);
	writel(readl(l->clcd + CLCD_CTRL) & ~1, l->clcd + CLCD_CTRL);
	poll_set(l->clcd + CLCD_CTRL, 2, 2);
}

/* ------------------------------------------------------------ panel */

/*
 * The panel's waits on bring-up, from iOS: after sleep-out before display-on
 * (134 ms for class 1 panels, 100 otherwise) and after display-on before the
 * backlight may come on (101 / 67 ms). Too short and the panel shows a white
 * flash or a stale picture when the backlight comes on.
 */

/* Panel enable after power loss */
static int n72_lcd_bringup(struct n72_lcd *l)
{
	bool class1 = l->panel_class < 2;
	ktime_t t;
	int ret;

	ret = clcd_power_on(l);
	if (ret)
		return ret;

	ret = panel_ldo(l, true);
	if (ret)
		return ret;
	t = ktime_add_ms(ktime_get(), 10);
	ret = dsi_power_on(l);
	if (ret)
		return ret;
	sleep_until(t);
	gpiod_set_value_cansleep(l->reset, 0);	/* out of reset */
	msleep(6);
	dcs(l, 0x11);				/* sleep out, in LP */
	/* 67 ms: half of iOS's 134, checked by eye (no white flash) */
	t = ktime_add_ms(ktime_get(), class1 ? 67 : 100);
	if (class1)
		msleep(5);
	else
		udelay(10);
	ret = dsi_hs_clock(l, true);
	if (ret)
		return ret;
	if (class1)
		dsi_video(l, true);
	clcd_scan(l, true);
	sleep_until(t);
	dcs(l, 0x29);				/* display on */
	/*
	 * Content-adaptive brightness off (DCS 0x55 = 0): the panel comes out
	 * of reset with it on and ramps its luminance after display-on, a
	 * visible dim-then-normal step on every wake (checked by eye).
	 */
	dsi_short(l, 0x15, 0x55, 0x00);
	msleep(class1 ? 101 : 67);
	if (!class1)
		dsi_video(l, true);
	return 0;
}

static void n72_lcd_shutdown(struct n72_lcd *l)
{
	dsi_video(l, false);
	dcs(l, 0x28);				/* display off */
	dcs(l, 0x10);				/* sleep in */
	msleep(100);
	dsi_hs_clock(l, false);
	clcd_power_off(l);
	gpiod_set_value_cansleep(l->reset, 1);
	panel_ldo(l, false);
}

/* ---------------------------------------------------------------- PM */

/* Put the backlight's power state back as it was before the sleep. */
static void n72_lcd_backlight_restore(struct n72_lcd *l)
{
	if (!l->bl)
		return;
	l->bl->props.power = l->bl_power;
	backlight_update_status(l->bl);
}

static int n72_lcd_suspend(struct device *dev)
{
	struct n72_lcd *l = dev_get_drvdata(dev);

	if (l->blanked)			/* already down, its registers gated */
		return 0;
	/*
	 * The backlight resumes before this device and would light the panel
	 * while it is still coming back up (a white flash): keep it off until
	 * the panel shows a picture again.
	 */
	if (l->bl && !l->dark) {
		/*
		 * Not backlight_disable(): its "blanked" flag would outlive the
		 * sleep and keep every later brightness at 0 when userspace had
		 * already switched the backlight off.
		 */
		l->bl_power = l->bl->props.power;
		l->bl->props.power = BACKLIGHT_POWER_OFF;
		backlight_update_status(l->bl);
	}
	/*
	 * Full panel power-down: restarting only scan-out and the DSI video
	 * stream after sleep came back white or shifted, while the full
	 * bring-up is reliable.
	 */
	n72_lcd_shutdown(l);
	l->panel_off = true;
	return 0;
}

static int n72_lcd_resume(struct device *dev)
{
	struct n72_lcd *l = dev_get_drvdata(dev);
	int ret;

	if (!l->panel_off)
		return 0;
	l->panel_off = false;

	l->dark = false;
	/* after suspend to RAM or an s2idle panel sleep alike */
	ret = n72_lcd_bringup(l);
	if (ret)
		dev_err(dev, "display bring-up failed: %d\n", ret);
	else
		dev_dbg(dev, "display up again\n");
	/*
	 * The bring-up cleared the CLCD's interrupt enables, and the DRM
	 * driver may have resumed (and enabled its frame interrupt) first:
	 * without it every flip waited for its 20 Hz fallback beat.
	 */
	blocking_notifier_call_chain(&n72_lcd_clients, N72_LCD_RESUMED, NULL);
	n72_lcd_backlight_restore(l);
	return 0;
}

/*
 * Go dark as soon as a sleep starts, before tasks are frozen and the other
 * drivers suspend (Wi-Fi alone takes half a second): backlight off and
 * the panel's display off. The suspend hook does the rest; if the sleep
 * is aborted before it runs, light the panel again.
 */
static int n72_lcd_pm_notify(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct n72_lcd *l = container_of(nb, struct n72_lcd, pm_nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		if (l->panel_off || l->blanked)
			break;
		if (l->bl) {
			l->bl_power = l->bl->props.power;
			l->bl->props.power = BACKLIGHT_POWER_OFF;
			backlight_update_status(l->bl);
		}
		dcs(l, 0x28);			/* display off */
		l->dark = true;
		break;
	case PM_POST_SUSPEND:
		if (!l->dark)
			break;
		/* the suspend hook never ran: the panel is still awake */
		l->dark = false;
		dcs(l, 0x29);			/* display on */
		n72_lcd_backlight_restore(l);
		break;
	}
	return NOTIFY_DONE;
}

static void n72_lcd_unregister_pm(void *nb)
{
	unregister_pm_notifier(nb);
}

static int n72_lcd_suspend_late(struct device *dev)
{
	struct n72_lcd *l = dev_get_drvdata(dev);

	if (!l->panel_off || l->clks_off)
		return 0;
	clk_bulk_disable(l->nclks, l->clks);
	l->clks_off = true;
	return 0;
}

static int n72_lcd_resume_early(struct device *dev)
{
	struct n72_lcd *l = dev_get_drvdata(dev);
	int ret;

	if (!l->clks_off || l->blanked)
		return 0;
	ret = clk_bulk_enable(l->nclks, l->clks);
	if (!ret)
		l->clks_off = false;
	return ret;
}

/* ------------------------------------------------------ display off */

static void n72_lcd_blank(struct n72_lcd *l)
{
	int i;

	/* every other user of the CLCD registers stops first */
	blocking_notifier_call_chain(&n72_lcd_clients, N72_LCD_OFF, NULL);
	if (l->bl) {
		l->bl_power = l->bl->props.power;
		l->bl->props.power = BACKLIGHT_POWER_OFF;
		backlight_update_status(l->bl);
	}
	/* the configuration as running now (window 0's buffer among it) */
	for (i = 0; i < ARRAY_SIZE(clcd_regs); i++)
		l->clcd_saved[i] = readl(l->clcd + clcd_regs[i]);
	n72_lcd_shutdown(l);
	clk_bulk_disable(l->nclks, l->clks);
	l->clks_off = true;
	l->blanked = true;
	n72_corev_display(false);	/* the display's clocks are closed */
}

static int n72_lcd_unblank(struct n72_lcd *l)
{
	int ret;

	n72_corev_display(true);	/* 1.20 V before the display's clocks open */
	ret = clk_bulk_enable(l->nclks, l->clks);
	if (ret)
		return ret;
	l->clks_off = false;
	l->blanked = false;
	ret = n72_lcd_bringup(l);
	if (ret)
		dev_err(l->dev, "display bring-up failed: %d\n", ret);
	blocking_notifier_call_chain(&n72_lcd_clients, N72_LCD_ON, NULL);
	/*
	 * The clients have just written the layers' current buffers; they are
	 * taken at the next frame. Lit before that, the panel showed the
	 * picture from before the display went off for a frame (the home
	 * screen before the lock screen).
	 */
	msleep(34);
	n72_lcd_backlight_restore(l);
	return ret;
}

static ssize_t panel_power_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct n72_lcd *l = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", !l->blanked);
}

static ssize_t panel_power_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct n72_lcd *l = dev_get_drvdata(dev);
	bool on;
	int ret = 0;

	if (kstrtobool(buf, &on))
		return -EINVAL;
	mutex_lock(&l->blank_lock);
	if (on && l->blanked)
		ret = n72_lcd_unblank(l);
	else if (!on && !l->blanked)
		n72_lcd_blank(l);
	mutex_unlock(&l->blank_lock);
	return ret ?: count;
}
static DEVICE_ATTR_RW(panel_power);

static struct attribute *n72_lcd_attrs[] = {
	&dev_attr_panel_power.attr,
	NULL
};
ATTRIBUTE_GROUPS(n72_lcd);

static const struct dev_pm_ops n72_lcd_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(n72_lcd_suspend, n72_lcd_resume)
	LATE_SYSTEM_SLEEP_PM_OPS(n72_lcd_suspend_late, n72_lcd_resume_early)
};

static int n72_lcd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n72_dsim_regs *r;
	struct n72_lcd *l;
	int i, cls, ret;

	l = devm_kzalloc(dev, sizeof(*l), GFP_KERNEL);
	if (!l)
		return -ENOMEM;
	l->dev = dev;
	mutex_init(&l->blank_lock);
	l->clcd = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(l->clcd))
		return PTR_ERR(l->clcd);
	l->dsim = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(l->dsim))
		return PTR_ERR(l->dsim);
	/* leave the running panel alone: no level change at probe */
	l->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(l->reset))
		return dev_err_probe(dev, PTR_ERR(l->reset), "no reset GPIO\n");
	/* optional: the backlight, kept off while the panel restarts */
	l->bl = devm_of_find_backlight(dev);
	if (IS_ERR(l->bl))
		return dev_err_probe(dev, PTR_ERR(l->bl), "no backlight\n");

	/* the panel is powered: take a reference so the supply stays on */
	/* the display runs from the boot loader: keep its gates open */
	l->nclks = devm_clk_bulk_get_all_enabled(dev, &l->clks);
	if (l->nclks < 0)
		return dev_err_probe(dev, l->nclks, "clocks\n");
	l->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(l->supply))
		return dev_err_probe(dev, PTR_ERR(l->supply), "no panel supply\n");
	if (regulator_enable(l->supply))
		dev_warn(dev, "could not take the panel supply\n");
	else
		l->supply_on = true;

	/* panel class as iOS derives it from lcd-panel-id */
	of_property_read_u32(dev->of_node, "apple,panel-id", &l->panel_id);
	cls = (l->panel_id >> 8) & 0xff;
	l->panel_class = (cls == 3 || cls == 4 || cls == 7 || cls == 8) ? 1 : 2;

	/* the running configuration, as the boot loader set it up */
	l->ctrl = readl(l->clcd + CLCD_CTRL) & ~0x103;
	for (i = 0; i < ARRAY_SIZE(clcd_regs); i++)
		l->clcd_saved[i] = readl(l->clcd + clcd_regs[i]);
	r = &l->dsim_saved;
	r->clkctrl = readl(l->dsim + DSIM_CLKCTRL);
	r->config = readl(l->dsim + DSIM_CONFIG);
	r->mdresol = readl(l->dsim + DSIM_MDRESOL) & ~BIT(31);
	r->mvporch = readl(l->dsim + DSIM_MVPORCH);
	r->mhporch = readl(l->dsim + DSIM_MHPORCH);
	r->msync = readl(l->dsim + DSIM_MSYNC);
	r->sdresol = readl(l->dsim + DSIM_SDRESOL);
	r->pllctrl = readl(l->dsim + DSIM_PLLCTRL);
	r->plltmr = readl(l->dsim + DSIM_PLLTMR);
	r->phyacchr = readl(l->dsim + DSIM_PHYACCHR);
	r->phyacchr1 = readl(l->dsim + DSIM_PHYACCHR1);

	platform_set_drvdata(pdev, l);
	if (!(readl(l->clcd + CLCD_CTRL) & 1)) {
		/*
		 * Stopped: a kexec from a kernel that had the display off
		 * (panel_power 0: the launcher's screen-off). The controller
		 * keeps its configuration, so it comes up as after a sleep.
		 * Left as it was, the panel stayed dark and the controller
		 * made no frame events (VIDCON0 0x2): no frame interrupt, every
		 * flip waiting for n72-clcd's 20 Hz fallback, until a screen
		 * off/on happened to bring it up.
		 */
		dev_info(dev, "display not running at probe: bringing it up\n");
		/*
		 * As a screen off/on: clients (n72-clcd may be up already)
		 * keep off the registers while the bring-up resets the
		 * controller, then put their layers and interrupts back.
		 */
		blocking_notifier_call_chain(&n72_lcd_clients, N72_LCD_OFF, NULL);
		ret = n72_lcd_bringup(l);
		if (ret)
			dev_err(dev, "display bring-up failed: %d\n", ret);
		blocking_notifier_call_chain(&n72_lcd_clients, N72_LCD_ON, NULL);
	}
	dev_dbg(dev, "panel %08x (class %d); clcd ctrl %08x vidcon %08x win %08x/%08x; dsi clk %08x cfg %08x res %08x pll %08x\n",
		l->panel_id, l->panel_class, l->ctrl, l->clcd_saved[14],
		l->clcd_saved[5], l->clcd_saved[6], r->clkctrl, r->config,
		r->mdresol, r->pllctrl);

	l->pm_nb.notifier_call = n72_lcd_pm_notify;
	ret = register_pm_notifier(&l->pm_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, n72_lcd_unregister_pm, &l->pm_nb);
	if (ret)
		return ret;
	/* bring the picture back alongside slower devices, not after them */
	device_enable_async_suspend(dev);
	return 0;
}

static void n72_lcd_platform_shutdown(struct platform_device *pdev)
{
	struct n72_lcd *l = platform_get_drvdata(pdev);

	mutex_lock(&l->blank_lock);
	if (l->blanked)
		n72_lcd_unblank(l);
	mutex_unlock(&l->blank_lock);
}

static const struct of_device_id n72_lcd_of_match[] = {
	{ .compatible = "apple,n72-lcd" },
	{ }
};

static struct platform_driver n72_lcd_driver = {
	.probe = n72_lcd_probe,
	.shutdown = n72_lcd_platform_shutdown,
	.driver = {
		.name = "n72-lcd",
		.of_match_table = n72_lcd_of_match,
		.pm = pm_sleep_ptr(&n72_lcd_pm_ops),
		.dev_groups = n72_lcd_groups,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(n72_lcd_driver);
