// SPDX-License-Identifier: GPL-2.0-only
/*
 * PMU of the Apple iPod touch (2nd generation), N72: the LDO regulators and
 * DOWN0, the SoC core rail. See drivers/mfd/apple-n72-pmu-core.c.
 */

#include <linux/bits.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

/*
 * LDOs: LDO n has its voltage in register 0x19 + n, bits 0-4,
 * (mV - base) / step, with base 2000/50 mV for LDO 0, 1200/100 for LDO 1-4
 * and 8, 2500/50 for 5-7 and 9 (LDO 9 has no voltage register); enable bit
 * n + 2 of 0x10 for n < 6, n - 6 of 0x11 for 6-8, bit 2 of 0x13 for LDO 9.
 * Board rails: LDO2 TV-out, LDO3 touch (1.8 V), LDO4 LCD (3.0 V), LDO5/9
 * dock accessory power.
 */
#define N72_PMU_NLDO		10
#define N72_PMU_LDO_V(n)	(0x19 + (n))
#define N72_PMU_VSEL		GENMASK(4, 0)

/*
 * DOWN0, the SoC core rail: bits 4:0 = (mV - 725) / 25, preserving the
 * upper bits. Switches between 1200 mV (0x13) and 1050 mV (0x0d) and waits
 * 150 us after a raise.
 */
#define N72_PMU_DOWN0		0x14
#define N72_PMU_DOWN0_MIN_MV	725
#define N72_PMU_DOWN0_STEP_MV	25
#define N72_PMU_DOWN0_SETTLE_US	150
#define N72_PMU_DOWN0_ID	N72_PMU_NLDO
#define N72_PMU_NREG		(N72_PMU_NLDO + 1)

/* --------------------------------------------------------- regulators */

/* DOWN0 has settled 150 us after a raise (iOS waits as long) */
static int n72_down0_time_sel(struct regulator_dev *rdev, unsigned int old_sel,
			      unsigned int new_sel)
{
	return new_sel > old_sel ? N72_PMU_DOWN0_SETTLE_US : 0;
}

static const struct regulator_ops n72_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
};

/*
 * DOWN0 is never switched off: no enable operations, so the regulator core
 * treats it as always on.
 */
static const struct regulator_ops n72_down0_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel = n72_down0_time_sel,
};

/* LDO 9 has an enable bit only */
static const struct regulator_ops n72_ldo_fixed_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

#define N72_LDO(_n, _min, _step)					\
	[_n] = {							\
		.name = "ldo" #_n,					\
		.of_match = "ldo" #_n,					\
		.regulators_node = "regulators",			\
		.id = _n,						\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.ops = &n72_ldo_ops,					\
		.min_uV = (_min) * 1000,				\
		.uV_step = (_step) * 1000,				\
		.n_voltages = 32,					\
		.vsel_reg = N72_PMU_LDO_V(_n),				\
		.vsel_mask = N72_PMU_VSEL,				\
		.enable_reg = (_n) < 6 ? 0x10 : 0x11,			\
		.enable_mask = BIT((_n) < 6 ? (_n) + 2 : (_n) - 6),	\
	}

static const struct regulator_desc n72_pmu_reg_desc[N72_PMU_NREG] = {
	N72_LDO(0, 2000, 50),
	N72_LDO(1, 1200, 100), N72_LDO(2, 1200, 100), N72_LDO(3, 1200, 100),
	N72_LDO(4, 1200, 100),
	N72_LDO(5, 2500, 50), N72_LDO(6, 2500, 50), N72_LDO(7, 2500, 50),
	N72_LDO(8, 1200, 100),
	[9] = {
		.name = "ldo9", .of_match = "ldo9", .regulators_node = "regulators",
		.id = 9, .type = REGULATOR_VOLTAGE, .owner = THIS_MODULE,
		.ops = &n72_ldo_fixed_ops,
		.enable_reg = 0x13, .enable_mask = BIT(2),
	},
	[N72_PMU_DOWN0_ID] = {
		.name = "down0",
		.of_match = "down0",
		.regulators_node = "regulators",
		.id = N72_PMU_DOWN0_ID,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &n72_down0_ops,
		.min_uV = N72_PMU_DOWN0_MIN_MV * 1000,
		.uV_step = N72_PMU_DOWN0_STEP_MV * 1000,
		.n_voltages = 32,			/* 725 to 1500 mV */
		.vsel_reg = N72_PMU_DOWN0,
		.vsel_mask = N72_PMU_VSEL,
	},
};

/*
 * Only regulators described in the DT get constraints (and so can be
 * switched or set); the rest are registered for reading only and never
 * touched. They are found in the PMU's node and registered under its
 * device.
 */
static int n72_pmu_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = {
		.dev = dev->parent,
		.regmap = dev_get_regmap(dev->parent, NULL),
	};
	struct regulator_dev *rdev;
	int i;

	if (!cfg.regmap)
		return -ENODEV;
	for (i = 0; i < N72_PMU_NREG; i++) {
		rdev = devm_regulator_register(dev, &n72_pmu_reg_desc[i], &cfg);
		if (IS_ERR(rdev))
			return PTR_ERR(rdev);
	}
	return 0;
}

static struct platform_driver n72_pmu_regulator_driver = {
	.driver = {
		.name = "n72-pmu-regulator",
	},
	.probe = n72_pmu_regulator_probe,
};
module_platform_driver(n72_pmu_regulator_driver);

MODULE_ALIAS("platform:n72-pmu-regulator");
MODULE_DESCRIPTION("Apple iPod touch 2G (Dialog D1759) PMU regulators");
MODULE_LICENSE("GPL");
