// SPDX-License-Identifier: GPL-2.0-only
/*
 * Suspend to RAM on the iPod touch 2G (N72), through the firmware wake path.
 *
 * The PMU cuts SoC power with DRAM left in self-refresh; wake restarts in
 * the boot ROM, which runs LLB from NOR. LLB treats a boot as a wake when:
 *
 *  - PMU register 0x6f has bit 7 set and bits 6 and 4 clear (LLB clears it),
 *  - DRAM 0x08000080 holds "XSOM" "PSUS" (0x4d4f5358 0x53555350; LLB clears
 *    both words), and
 *  - DRAM 0x08000090 holds the 16-byte sleep token: AES-128(K,
 *    "iBootSleepValid\0") with K = AES-128-UID of the same block, both one
 *    CBC block with a zero IV.
 *
 * LLB quiesces hardware, turns the MMU/caches off, cleans D-cache, and
 * jumps to physical address 0 (start of DRAM). Before hibernating, Linux
 * writes the magic to DRAM + 0x80, token to DRAM + 0x90, wake stub at
 * DRAM + 0, sets 0x6f bit 7 (preserving bit 5), and sets PMU OOCSHDWN
 * (0x0a) to (old & ~0x10) | 0x0a. PMU interrupt masks and GPIO settings
 * (0x0b..0x0e) are left as configured. The first DRAM page is reserved in
 * the device tree, clock gates are saved and restored across sleep, and the
 * wake token uses the UID AES key and is computed lazily on first deep
 * sleep (never at boot).
 *
 * Offered as "deep" in /sys/power/mem_sleep; mem_sleep_default=s2idle keeps
 * s2idle as the default until userspace explicitly selects deep.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <asm/cacheflush.h>
#include <asm/idmap.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>

#define N72_DRAM_BASE		0x08000000
#define N72_WAKE_MAGIC		0x80
#define N72_WAKE_TOKEN		0x90
#define MAGIC_MOSX		0x4d4f5358
#define MAGIC_SUSP		0x53555350

#define PMU_INT1		0x01
#define PMU_OOCSHDWN		0x0a
#define PMU_WAKE_FLAG		0x6f	/* LLB: (v & 0xd0) == 0x80 is a wake */

/* AES engine registers */
#define AES_CONTROL		0x00
#define AES_GO			0x04
#define AES_UNK0		0x08
#define AES_STATUS		0x0c
#define AES_UNK1		0x10
#define AES_KEYLEN		0x14
#define AES_INSIZE		0x18
#define AES_INADDR		0x20
#define AES_OUTSIZE		0x24
#define AES_OUTADDR		0x28
#define AES_AUXSIZE		0x2c
#define AES_AUXADDR		0x30
#define AES_SIZE3		0x34
#define AES_KEY			0x4c
#define AES_TYPE		0x6c
#define AES_TYPE_INV		0x88	/* ~AES_TYPE; hardware requires both */
#define AES_IV			0x74
#define AES_TYPE_USER		0
#define AES_TYPE_UID		2
#define AES_KEY128		0x5c	/* a 128-bit user key: 0x5c..0x68 */

/*
 * Power domains, in the system controller at 0x39700000: ONCTRL/OFFCTRL
 * request, SETSTATE is the requested and STATE the actual state, one bit
 * per domain. Deep sleep leaves on only what LLB needs, so restore the rest.
 */
#define N72_SYSCTRL		0x39700000
#define PWR_SETSTATE		0x08
#define PWR_ONCTRL		0x0c
#define PWR_OFFCTRL		0x10
#define PWR_STATE		0x14

#define N72_NGATES		5
static const u16 n72_gate_regs[N72_NGATES] = { 0x48, 0x4c, 0x58, 0x68, 0x6c };

struct n72_sleep {
	struct device *dev;
	struct i2c_client *pmu;
	struct clk *aes_clk;
	void __iomem *aes;
	void __iomem *clk;		/* clock controller, 0x3c500000 */
	u32 *page;			/* DRAM + 0, linear map */
	u8 token[16];
	bool token_ok;
	u32 gates[N72_NGATES];
	u32 clkregs[64];		/* whole clock block, for the wake report */
	void __iomem *sysctrl;		/* power domains (shared with the GPIO IC) */
	u32 pwr_set, pwr_state;		/* power domains before the sleep */
	u8 oocshdwn, wake_flag;
	bool woke;			/* the PMU cut the power and we came back */
	u32 llb_config0;		/* clock register 0 as LLB left it */
};

static struct n72_sleep *n72s;

static int pmu_wr(struct n72_sleep *s, u8 reg, u8 val);

static const u8 sleep_plain[16] = "iBootSleepValid";

/*
 * Encrypt one 16-byte CBC block with a zero IV: key type goes in both
 * AES_TYPE (0x6c) and AES_TYPE_INV (~type in 0x88, required for UID key
 * selection). A user key (type 0) is 128 bits written into 0x5c..0x68,
 * copied straight from memory.
 */
static int n72_aes_block(struct device *dev, void __iomem *aes, u32 type,
			 const u8 *key, const u8 *in, u8 *out)
{
	dma_addr_t dma;
	u8 *buf;
	u32 v, k[4];
	int i, ret;

	buf = dma_alloc_coherent(dev, 16, &dma, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memcpy(buf, in, 16);

	writel(1, aes + AES_CONTROL);
	writel(0, aes + AES_UNK1);
	writel(1, aes + AES_UNK0);
	writel(0, aes + AES_UNK0);
	writel(1, aes + AES_CONTROL);
	writel(type, aes + AES_TYPE);
	writel(~type, aes + AES_TYPE_INV);	/* without it the UID key is not used */
	if (type == AES_TYPE_USER) {
		memcpy(k, key, 16);
		for (i = 0; i < 4; i++)
			writel(k[i], aes + AES_KEY128 + i * 4);
		memzero_explicit(k, sizeof(k));
	}
	writel(6, aes + AES_KEYLEN);
	writel((readl(aes + AES_KEYLEN) & ~1) | 1, aes + AES_KEYLEN);	/* encrypt */
	writel(readl(aes + AES_KEYLEN) & ~0x30, aes + AES_KEYLEN);	/* 128-bit key */
	writel(readl(aes + AES_KEYLEN) | 0x8, aes + AES_KEYLEN);
	for (i = 0; i < 4; i++)
		writel(0, aes + AES_IV + i * 4);
	writel(16, aes + AES_INSIZE);
	writel(dma, aes + AES_INADDR);
	writel(16, aes + AES_OUTSIZE);
	writel(dma, aes + AES_OUTADDR);
	writel(16, aes + AES_AUXSIZE);
	writel(dma, aes + AES_AUXADDR);
	writel(16, aes + AES_SIZE3);
	writel(1, aes + AES_GO);
	ret = readl_poll_timeout(aes + AES_STATUS, v, v & 0xf, 10, 100000);
	for (i = 0; i < 8; i++)
		writel(0, aes + AES_KEY + i * 4);
	for (i = 0; i < 4; i++)
		writel(0, aes + AES_IV + i * 4);
	writel(0xf, aes + AES_STATUS);
	if (!ret && !memcmp(buf, in, 16))
		ret = -EIO;	/* untouched: the engine did not run */
	if (!ret)
		memcpy(out, buf, 16);
	memzero_explicit(buf, 16);
	dma_free_coherent(dev, 16, buf, dma);
	return ret;
}

/*
 * Compute LLB wake token: K = AES-128-UID(sleep_plain), then
 * token = AES-128(K, sleep_plain).
 */
static int n72_make_token(struct n72_sleep *s)
{
	u8 key[16];
	int ret;

	ret = clk_prepare_enable(s->aes_clk);
	if (ret)
		return ret;
	ret = n72_aes_block(s->dev, s->aes, AES_TYPE_UID, NULL, sleep_plain, key);
	if (!ret)
		ret = n72_aes_block(s->dev, s->aes, AES_TYPE_USER, key, sleep_plain, s->token);
	clk_disable_unprepare(s->aes_clk);
	memzero_explicit(key, sizeof(key));
	return ret;
}

/* ---------------------------------------------------------------- PMU */

static int pmu_rd(struct n72_sleep *s, u8 reg)
{
	return i2c_smbus_read_byte_data(s->pmu, reg);
}

static int pmu_wr(struct n72_sleep *s, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(s->pmu, reg, val);
}

/*
 * Last step, with the CPU state saved by cpu_suspend(). Same order as iOS:
 * arm the wake events, flag the wake for LLB, then ask the PMU to hibernate
 * and wait for the power to go. If it does not within two seconds, undo and
 * return, and cpu_suspend() returns the error.
 */
static int n72_finish(unsigned long arg)
{
	struct n72_sleep *s = n72s;
	u8 discard[3];
	int v, i;

	flush_cache_all();

	/* clear pending events, so only new ones wake us */
	i2c_smbus_read_i2c_block_data(s->pmu, PMU_INT1, sizeof(discard), discard);
	v = pmu_rd(s, PMU_WAKE_FLAG);
	s->wake_flag = v < 0 ? 0 : v;
	pmu_wr(s, PMU_WAKE_FLAG, (s->wake_flag & 0x20) | 0x80);
	v = pmu_rd(s, PMU_OOCSHDWN);
	s->oocshdwn = v < 0 ? 0 : v;
	dsb();
	pmu_wr(s, PMU_OOCSHDWN, (s->oocshdwn & ~0x10) | 0x0a);

	for (i = 0; i < 2000; i++)
		mdelay(1);

	/* still here: the PMU did not take the hibernate request */
	pmu_wr(s, PMU_OOCSHDWN, s->oocshdwn);
	pmu_wr(s, PMU_WAKE_FLAG, s->wake_flag);
	return -EIO;
}

/* ----------------------------------------------------------- suspend ops */

static int n72_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM && n72s;
}

/* Process context: compute the token the first time it is needed. */
static int n72_begin(suspend_state_t state)
{
	struct n72_sleep *s = n72s;
	int ret;

	if (s->token_ok)
		return 0;
	ret = n72_make_token(s);
	if (ret) {
		dev_err(s->dev, "no sleep token (AES with the UID key): %d\n", ret);
		return ret;
	}
	s->token_ok = true;
	return 0;
}

static int n72_enter(suspend_state_t state)
{
	struct n72_sleep *s = n72s;
	u32 *p = s->page;
	int ret;

	/* Wake stub (MMU and caches off, ARM state): jump straight to cpu_resume. */
	p[0] = 0xe59ff000;	/* ldr pc, [pc]	@ =cpu_resume */
	p[1] = 0;
	p[2] = __pa_symbol(cpu_resume);
	p[N72_WAKE_MAGIC / 4] = MAGIC_MOSX;
	p[N72_WAKE_MAGIC / 4 + 1] = MAGIC_SUSP;
	memcpy(&p[N72_WAKE_TOKEN / 4], s->token, 16);
	flush_cache_all();

	s->woke = false;
	ret = cpu_suspend(0, n72_finish);
	s->woke = !ret;
	/*
	 * First, before touching any other block: LLB gates clocks before its
	 * jump. Put back the gates Linux had (saved by the syscore suspend
	 * hook; the syscore resume hook writes them again).
	 */
	if (s->woke) {
		int i;

		/*
		 * LLB sets the base clock divisor (0x00 bits 0-3) on its wake
		 * path; put Linux's back before using any bus.
		 */
		s->llb_config0 = readl(s->clk);
		writel((s->llb_config0 & ~0xf) | (s->clkregs[0] & 0xf), s->clk);
		for (i = 0; i < N72_NGATES; i++)
			writel(s->gates[i], s->clk + n72_gate_regs[i]);
	}

	/* awake again (or the PMU refused): LLB clears the magic on a real wake */
	p[N72_WAKE_MAGIC / 4] = 0;
	p[N72_WAKE_MAGIC / 4 + 1] = 0;
	flush_cache_all();
	if (!ret)
		dev_info(s->dev, "woke through LLB (clock config0: LLB %08x, Linux %08x)\n",
			 s->llb_config0, s->clkregs[0]);
	return ret;
}

static const struct platform_suspend_ops n72_suspend_ops = {
	.valid = n72_valid,
	.begin = n72_begin,
	.enter = n72_enter,
};

/*
 * After a real wake every clock gate is as LLB left it: put back what
 * Linux had (before the drivers' own resume hooks run). Cheap and harmless
 * after s2idle too, where nothing changed.
 */
static int n72_syscore_suspend(void)
{
	int i;

	if (!n72s)
		return 0;
	for (i = 0; i < N72_NGATES; i++)
		n72s->gates[i] = readl(n72s->clk + n72_gate_regs[i]);
	for (i = 0; i < ARRAY_SIZE(n72s->clkregs); i++)
		n72s->clkregs[i] = readl(n72s->clk + i * 4);
	if (n72s->sysctrl) {
		n72s->pwr_set = readl(n72s->sysctrl + PWR_SETSTATE);
		n72s->pwr_state = readl(n72s->sysctrl + PWR_STATE);
	}
	return 0;
}

static void n72_power_domains_restore(struct n72_sleep *s)
{
	u32 set, state, on;
	int n = 0;

	if (!s->sysctrl)
		return;
	set = readl(s->sysctrl + PWR_SETSTATE);
	state = readl(s->sysctrl + PWR_STATE);
	on = s->pwr_set & ~set;
	if (on) {
		writel(on, s->sysctrl + PWR_ONCTRL);
		while (n++ < 100000 &&
		       (readl(s->sysctrl + PWR_SETSTATE) & on) !=
		       (readl(s->sysctrl + PWR_STATE) & on))
			cpu_relax();
	}
	if (set != s->pwr_set || state != s->pwr_state || on)
		pr_info("n72-sleep: power domains set %08x state %08x before, %08x/%08x after wake; on %08x -> %08x/%08x\n",
			s->pwr_set, s->pwr_state, set, state, on,
			readl(s->sysctrl + PWR_SETSTATE), readl(s->sysctrl + PWR_STATE));
}

static void n72_syscore_resume(void)
{
	int i;

	if (!n72s || pm_suspend_target_state != PM_SUSPEND_MEM)
		return;
	n72_power_domains_restore(n72s);
	/*
	 * Everything else iBSS set and LLB does not (on the N72 at least 0x28,
	 * 0x54 and 0x60; without them the USB PHY gets no clock and dwc2
	 * cannot even enter device mode): write back what Linux had. 0x40 is
	 * the read-only PLL lock status.
	 */
	for (i = 0; i < ARRAY_SIZE(n72s->clkregs); i++) {
		u32 now = readl(n72s->clk + i * 4);

		if (now == n72s->clkregs[i] || i * 4 == 0x40)
			continue;
		pr_info("n72-sleep: clock reg %02x: %08x before, %08x after wake; restored\n",
			i * 4, n72s->clkregs[i], now);
		writel(n72s->clkregs[i], n72s->clk + i * 4);
	}
	for (i = 0; i < N72_NGATES; i++)
		writel(n72s->gates[i], n72s->clk + n72_gate_regs[i]);
}

static int n72_pm_notify(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct n72_sleep *s = n72s;

	if (event == PM_POST_SUSPEND && s && s->woke) {
		/* all the way back: leave 0x6f as LLB leaves it after a boot */
		s->woke = false;
		pmu_wr(s, PMU_WAKE_FLAG, s->wake_flag & 0x20);
	}
	return NOTIFY_DONE;
}

static struct notifier_block n72_pm_nb = {
	.notifier_call = n72_pm_notify,
};

static struct syscore_ops n72_syscore_ops = {
	.suspend = n72_syscore_suspend,
	.resume = n72_syscore_resume,
};

static int n72_sleep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct n72_sleep *s;
	struct resource *r;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	/* the clock controller is shared (MBX, SDIO gates): map, don't claim */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	s->clk = r ? devm_ioremap(dev, r->start, resource_size(r)) : NULL;
	if (!s->clk)
		return -ENODEV;
	np = of_parse_phandle(dev->of_node, "apple,pmu", 0);
	s->pmu = np ? of_find_i2c_device_by_node(np) : NULL;
	of_node_put(np);
	if (!s->pmu)
		return dev_err_probe(dev, -EPROBE_DEFER, "no PMU yet\n");
	s->page = phys_to_virt(N72_DRAM_BASE);

	s->aes = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->aes))
		return PTR_ERR(s->aes);
	s->aes_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->aes_clk))
		return dev_err_probe(dev, PTR_ERR(s->aes_clk), "AES clock\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	register_pm_notifier(&n72_pm_nb);
	n72s = s;
	dev_info(dev, "suspend to RAM through LLB offered (mem_sleep \"deep\")\n");
	suspend_set_ops(&n72_suspend_ops);
	return 0;
}

static const struct of_device_id n72_sleep_of_match[] = {
	{ .compatible = "apple,n72-sleep" },
	{ }
};

static struct platform_driver n72_sleep_driver = {
	.probe = n72_sleep_probe,
	.driver = {
		.name = "n72-sleep",
		.of_match_table = n72_sleep_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(n72_sleep_driver);

/*
 * syscore resume runs in reverse registration order: registering last puts
 * the clock gates back before timekeeping restarts the clocksource and
 * before the VIC is restored.
 */
static int __init n72_sleep_syscore_init(void)
{
	if (n72s)
		register_syscore_ops(&n72_syscore_ops);
	return 0;
}
late_initcall_sync(n72_sleep_syscore_init);
