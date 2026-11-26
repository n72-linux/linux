// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock gates of the Samsung/Apple S5L8720 clock controller (0x3c500000).
 *
 * Five gate registers (0x48, 0x4c, 0x58, 0x68, 0x6c, the order of iOS's
 * device-clocks table); a set bit stops the clock. A clock is named by
 * <register-index bit> and is created only when a device asks for it, so
 * the gates the boot loader left open but nothing in the DT names are never
 * registered, and never turned off as unused.
 *
 * The register block is shared with the PLL/divider registers and other
 * drivers that ungate their own blocks, so it is mapped, not claimed.
 *
 * "apple,gates-off" = <register-index bit>, ... lists gates to close at
 * boot: blocks the boot loader leaves running that no Linux driver uses (iOS
 * keeps them gated). A clocked but idle block still costs power.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

static const u16 s5l8720_gate_reg[] = { 0x48, 0x4c, 0x58, 0x68, 0x6c };

static DEFINE_SPINLOCK(s5l8720_gate_lock);
static DEFINE_MUTEX(s5l8720_gate_reg_lock);

struct s5l8720_gates {
	void __iomem *base;
	const char *parent;
	struct clk_hw mbx_hw;
	struct clk_hw *hw[ARRAY_SIZE(s5l8720_gate_reg)][32];
};

/*
 * MBX clock gating: register 0x68 bits 5-7 must clear/set in one write
 * (ungating only some bits hangs the SoC). Exposed as <3 5>.
 */
#define S5L8720_MBX_GATE_MASK	GENMASK(7, 5)

static int s5l8720_mbx_gate_enable(struct clk_hw *hw)
{
	struct s5l8720_gates *g = container_of(hw, struct s5l8720_gates, mbx_hw);
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&s5l8720_gate_lock, flags);
	v = readl(g->base + 0x68);
	writel(v & ~S5L8720_MBX_GATE_MASK, g->base + 0x68);
	spin_unlock_irqrestore(&s5l8720_gate_lock, flags);
	return 0;
}

static void s5l8720_mbx_gate_disable(struct clk_hw *hw)
{
	struct s5l8720_gates *g = container_of(hw, struct s5l8720_gates, mbx_hw);
	unsigned long flags;
	u32 v;

	spin_lock_irqsave(&s5l8720_gate_lock, flags);
	v = readl(g->base + 0x68);
	writel(v | S5L8720_MBX_GATE_MASK, g->base + 0x68);
	spin_unlock_irqrestore(&s5l8720_gate_lock, flags);
}

static int s5l8720_mbx_gate_is_enabled(struct clk_hw *hw)
{
	struct s5l8720_gates *g = container_of(hw, struct s5l8720_gates, mbx_hw);

	return !(readl(g->base + 0x68) & S5L8720_MBX_GATE_MASK);
}

static const struct clk_ops s5l8720_mbx_gate_ops = {
	.enable = s5l8720_mbx_gate_enable,
	.disable = s5l8720_mbx_gate_disable,
	.is_enabled = s5l8720_mbx_gate_is_enabled,
};

static struct clk_hw *s5l8720_gate_get(struct of_phandle_args *spec, void *data)
{
	struct s5l8720_gates *g = data;
	unsigned int r, b;
	struct clk_hw *hw;
	char name[24];

	if (spec->args_count != 2)
		return ERR_PTR(-EINVAL);
	r = spec->args[0];
	b = spec->args[1];
	if (r >= ARRAY_SIZE(s5l8720_gate_reg) || b >= 32)
		return ERR_PTR(-EINVAL);
	if (r == 3 && (b == 6 || b == 7))
		return ERR_PTR(-EINVAL);

	mutex_lock(&s5l8720_gate_reg_lock);
	hw = g->hw[r][b];
	if (!hw) {
		if (r == 3 && b == 5) {
			struct clk_init_data init = {
				.name = "gate-68.mbx",
				.ops = &s5l8720_mbx_gate_ops,
				.parent_names = g->parent ? &g->parent : NULL,
				.num_parents = g->parent ? 1 : 0,
			};
			int ret;

			g->mbx_hw.init = &init;
			ret = clk_hw_register(NULL, &g->mbx_hw);
			hw = ret ? ERR_PTR(ret) : &g->mbx_hw;
		} else {
			snprintf(name, sizeof(name), "gate-%02x.%u", s5l8720_gate_reg[r], b);
			hw = clk_hw_register_gate(NULL, name, g->parent, 0,
						  g->base + s5l8720_gate_reg[r], b,
						  CLK_GATE_SET_TO_DISABLE, &s5l8720_gate_lock);
		}
		if (!IS_ERR(hw))
			g->hw[r][b] = hw;
	}
	mutex_unlock(&s5l8720_gate_reg_lock);
	return hw;
}

static void __init s5l8720_gates_init(struct device_node *np)
{
	struct s5l8720_gates *g;

	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return;
	g->base = of_iomap(np, 0);
	if (!g->base) {
		kfree(g);
		return;
	}
	g->parent = of_clk_get_parent_name(np, 0);

	{
		u32 off[ARRAY_SIZE(s5l8720_gate_reg)] = { 0 };
		int n = of_property_count_u32_elems(np, "apple,gates-off"), i;
		u32 r, b;

		for (i = 0; i + 1 < n; i += 2) {
			if (of_property_read_u32_index(np, "apple,gates-off", i, &r) ||
			    of_property_read_u32_index(np, "apple,gates-off", i + 1, &b) ||
			    r >= ARRAY_SIZE(s5l8720_gate_reg) || b >= 32)
				continue;
			off[r] |= BIT(b);
		}
		for (r = 0; r < ARRAY_SIZE(s5l8720_gate_reg); r++) {
			void __iomem *a = g->base + s5l8720_gate_reg[r];
			u32 v = readl(a);

			if (!(off[r] & ~v))
				continue;
			pr_info("s5l8720 gates: %02x %08x -> %08x (unused blocks off)\n",
				s5l8720_gate_reg[r], v, v | off[r]);
			writel(v | off[r], a);
		}
	}
	if (of_clk_add_hw_provider(np, s5l8720_gate_get, g)) {
		iounmap(g->base);
		kfree(g);
	}
}

CLK_OF_DECLARE(s5l8720_gates, "apple,s5l8720-clock-gates", s5l8720_gates_init);
