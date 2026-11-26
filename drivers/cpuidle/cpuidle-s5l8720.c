// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU idle for the Apple iPod touch 2nd generation (N72).
 *
 * Programs CLKCON0[3:0] to (divisor - 1) around WFI using divisors 1, 2, 5,
 * and 11, capped by whichever clock gates are open. Before WFI, cleans L1
 * data cache and drains the write buffer; after wake, spins 1,200
 * decrement-and-branch iterations inside the WFI I-cache line without data
 * accesses before touching memory, the timer, or the clock controller.
 */

#include <linux/bits.h>
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irqflags.h>
#include <linux/kstrtox.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/suspend.h>

#include <linux/clk/s5l8720.h>

#include <asm/cpuidle.h>

#define S5L8720_CLKCON0		0x00
#define S5L8720_SYS_DIV_MASK	GENMASK(3, 0)

/*
 * Clock gates (a set bit is gated) and the largest system clock divider each
 * open consumer allows. Everything else allows /11.
 */
#define S5L8720_GATES0		0x48
#define S5L8720_GATES3		0x68
#define S5L8720_GATES4		0x6c

struct s5l8720_div_limit {
	u32 gates0, gates3, gates4;
	unsigned int max_div;
};

static const struct s5l8720_div_limit s5l8720_div_limits[] = {
	/* FMSS; MBX; H264; MPVD */
	{ BIT(5) | BIT(9), GENMASK(7, 5) | BIT(2) | BIT(3), BIT(18), 1 },
	/* TVOUT (the AMC votes per codec, while it runs: s5l8720_amc_max_div) */
	{ BIT(10), BIT(0) | BIT(1), 0, 2 },
	/* CLCD; LCDIF; USB OTG */
	{ BIT(14) | BIT(1) | BIT(2), GENMASK(11, 9), BIT(16), 5 },
};

static void __iomem *s5l8720_clk_base;

/*
 * The AMC audio coprocessor's clock divider while it runs: its clock comes
 * from the system clock and may not exceed 34 MHz, so lower its divider
 * after a slowdown and raise it before a speedup. NULL while AMC is idle.
 * Only touched with interrupts off (here) or by its driver outside idle.
 */
static void __iomem *s5l8720_amc_div;
static unsigned long s5l8720_amc_bus_hz;
/*
 * While it runs, the AMC also limits the system divider like iOS's
 * logical clocks AMC-HE (HE-AAC, encoder: /2) and AMC-ALAC (/5); other
 * codecs leave the SYSTEM baseline. Gated between bursts, it limits nothing.
 */
static unsigned int s5l8720_amc_max_div = 11;

void s5l8720_idle_amc_clock(void __iomem *div_reg, unsigned long bus_hz,
			    unsigned int max_div)
{
	s5l8720_amc_bus_hz = bus_hz;
	s5l8720_amc_max_div = max_div ?: 11;
	WRITE_ONCE(s5l8720_amc_div, div_reg);
}
EXPORT_SYMBOL_GPL(s5l8720_idle_amc_clock);

static void __cpuidle s5l8720_amc_follow(unsigned int sysdiv)
{
	void __iomem *reg = READ_ONCE(s5l8720_amc_div);
	unsigned long hz = s5l8720_amc_bus_hz / sysdiv;

	if (reg)
		writel_relaxed(clamp(DIV_ROUND_UP(hz, S5L8720_AMC_MAX_HZ), 1UL, 16UL) - 1, reg);
}

/* the system divider of each state; 1 = plain WFI */
static const unsigned int s5l8720_state_div[] = { 1, 2, 5, 11 };

/*
 * The divider the system runs at outside idle: 1, or 2 for the low
 * voltage state. With display, TV-out and USB PHY all gated, the core can
 * run at 1.05 V with system clock capped at /2; /1 requires 1.20 V. Set /2
 * before lowering voltage and /1 only after raising it. Idle states at or
 * above this divider execute plain WFI.
 */
static unsigned int s5l8720_active_div = 1;

int s5l8720_idle_set_active_div(unsigned int div)
{
	void __iomem *clkcon0;
	unsigned long flags;
	u32 clk;

	if (div != 1 && div != 2)
		return -EINVAL;
	if (!s5l8720_clk_base)
		return -ENODEV;
	clkcon0 = s5l8720_clk_base + S5L8720_CLKCON0;

	local_irq_save(flags);
	if (div < s5l8720_active_div)
		s5l8720_amc_follow(div);		/* before the speedup */
	clk = readl_relaxed(clkcon0);
	writel_relaxed((clk & ~S5L8720_SYS_DIV_MASK) | (div - 1), clkcon0);
	readl_relaxed(clkcon0);
	if (div > s5l8720_active_div)
		s5l8720_amc_follow(div);		/* after the slowdown */
	s5l8720_active_div = div;
	local_irq_restore(flags);
	return 0;
}
EXPORT_SYMBOL_GPL(s5l8720_idle_set_active_div);

/*
 * The AMC's driver has just written its clock divider for the full system
 * clock: put it right for the running divider (it may be 2), atomically
 * with respect to a change of that divider.
 */
void s5l8720_idle_amc_resync(void)
{
	unsigned long flags;

	local_irq_save(flags);
	s5l8720_amc_follow(s5l8720_active_div);
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(s5l8720_idle_amc_resync);

/* largest divider every open consumer allows (iOS's idle-state selector) */
static unsigned int __cpuidle s5l8720_max_div(void)
{
	u32 open0 = ~readl_relaxed(s5l8720_clk_base + S5L8720_GATES0);
	u32 open3 = ~readl_relaxed(s5l8720_clk_base + S5L8720_GATES3);
	u32 open4 = ~readl_relaxed(s5l8720_clk_base + S5L8720_GATES4);
	unsigned int amc = READ_ONCE(s5l8720_amc_div) ? s5l8720_amc_max_div : 11;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(s5l8720_div_limits); i++) {
		const struct s5l8720_div_limit *l = &s5l8720_div_limits[i];

		if ((open0 & l->gates0) || (open3 & l->gates3) ||
		    (open4 & l->gates4))
			return min(l->max_div, amc);
	}
	return amc;
}

/* XNU's settling loop after the wake, in iterations (SLEEP.md) */
#define S5L8720_WAKE_SPIN	1200

/*
 * Clean the whole L1 data cache, drain the write buffer, WFI, then the
 * settling loop. The WFI and the loop share one 32-byte line, fetched with
 * the drain, so nothing after the wake needs a memory access until the
 * loop ends.
 */
static void __cpuidle s5l8720_wfi_settle(void)
{
	unsigned long zero = 0, n = S5L8720_WAKE_SPIN;

	asm volatile(
	"	mcr	p15, 0, %0, c7, c10, 0\n"	/* clean entire D-cache */
	"	.align	5\n"
	"	mcr	p15, 0, %0, c7, c10, 4\n"	/* drain write buffer */
	"	mcr	p15, 0, %0, c7, c0, 4\n"	/* wait for interrupt */
	"1:	subs	%1, %1, #1\n"
	"	bne	1b\n"
		: "+r" (zero), "+r" (n) : : "memory", "cc");
}

static int __cpuidle s5l8720_enter_divided(struct cpuidle_device *dev,
					  struct cpuidle_driver *drv, int index)
{
	void __iomem *clkcon0 = s5l8720_clk_base + S5L8720_CLKCON0;
	unsigned int max = s5l8720_max_div();
	unsigned int active = s5l8720_active_div;
	u32 clk;

	/*
	 * Also for suspend to idle (->enter_s2idle), where the devices are
	 * suspended and most gates closed. Suspend to RAM never idles here
	 * (the SoC powers off), but keep its way down to plain WFI.
	 */
	if (pm_suspend_target_state == PM_SUSPEND_MEM)
		return arm_cpuidle_simple_enter(dev, drv, 0);

	/*
	 * The gate bits are read here, so even asynchronous users such as
	 * NAND and MBX are seen. On the single-core N72, IRQs stay disabled
	 * from these reads until the normal divider is restored on wake.
	 * Too deep for the open consumers: take the deepest enabled state
	 * that fits, and report it, so the statistics show what ran.
	 */
	while (index && (s5l8720_state_div[index] > max ||
			 dev->states_usage[index].disable))
		index--;
	clk = readl_relaxed(clkcon0);
	if (!index || s5l8720_state_div[index] <= active ||
	    (clk & S5L8720_SYS_DIV_MASK) != active - 1)
		return arm_cpuidle_simple_enter(dev, drv, 0);

	writel_relaxed((clk & ~S5L8720_SYS_DIV_MASK) | (s5l8720_state_div[index] - 1),
		       clkcon0);
	readl_relaxed(clkcon0);
	s5l8720_amc_follow(s5l8720_state_div[index]);	/* after the slowdown */
	s5l8720_wfi_settle();
	s5l8720_amc_follow(active);			/* before the speedup */
	clk = readl_relaxed(clkcon0);
	writel_relaxed((clk & ~S5L8720_SYS_DIV_MASK) | (active - 1), clkcon0);
	readl_relaxed(clkcon0);

	return index;
}

static struct cpuidle_driver s5l8720_idle_driver = {
	.name = "s5l8720_idle",
	.states = {
		ARM_CPUIDLE_WFI_STATE,
		/*
		 * The latencies are estimates: the wake settling loop runs at
		 * the divided clock, so it lengthens with the divider. All on,
		 * as on iOS. /2 and /5 validated (timekeeping, USB, Wi-Fi,
		 * Bluetooth, audio, NAND, sensors, GPU); /11 is reached with
		 * the display and USB gated (screen off, cable out).
		 */
		{
			.enter = s5l8720_enter_divided,
			.enter_s2idle = s5l8720_enter_divided,
			.exit_latency = 20,
			.target_residency = 200,
			.power_usage = UINT_MAX,
			.name = "SYSCLK_DIV2",
			.desc = "WFI, system clock at /2",
		},
		{
			.enter = s5l8720_enter_divided,
			.enter_s2idle = s5l8720_enter_divided,
			.exit_latency = 50,
			.target_residency = 500,
			.power_usage = UINT_MAX,
			.name = "SYSCLK_DIV5",
			.desc = "WFI, system clock at /5",
		},
		{
			.enter = s5l8720_enter_divided,
			.enter_s2idle = s5l8720_enter_divided,
			.exit_latency = 100,
			.target_residency = 1000,
			.power_usage = UINT_MAX,
			.name = "SYSCLK_DIV11",
			.desc = "WFI, system clock at /11",
		},
	},
	.state_count = 4,
	.safe_state_index = 0,
};

static int __init s5l8720_cpuidle_init(void)
{
	struct device_node *np;
	int ret;

	if (!of_machine_is_compatible("apple,n72") || num_possible_cpus() != 1)
		return -ENODEV;

	np = of_find_compatible_node(NULL, NULL, "apple,s5l8720-clock-gates");
	if (!np)
		return -ENODEV;

	s5l8720_clk_base = of_iomap(np, 0);
	of_node_put(np);
	if (!s5l8720_clk_base)
		return -ENOMEM;

	ret = cpuidle_register(&s5l8720_idle_driver, NULL);
	if (ret)
		iounmap(s5l8720_clk_base);

	return ret;
}
device_initcall(s5l8720_cpuidle_init);
