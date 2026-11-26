// SPDX-License-Identifier: GPL-2.0-only
/*
 * PMU of the Apple iPod touch (2nd generation), N72: the display backlight.
 * See drivers/mfd/apple-n72-pmu-core.c.
 */

#include <linux/backlight.h>
#include <linux/mfd/apple-n72-pmu.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define N72_PMU_BL_MAX		245	/* maximum panel backlight level */

static int n72_pmu_bl_update(struct backlight_device *bl)
{
	struct regmap *regmap = bl_get_data(bl);
	int level = backlight_get_brightness(bl);
	int ret;

	if (!level)
		return regmap_write(regmap, N72_PMU_BL_CTRL, 0);

	ret = regmap_write(regmap, N72_PMU_BL_LEVEL, level);
	if (ret)
		return ret;
	return regmap_write(regmap, N72_PMU_BL_CTRL, N72_PMU_BL_CTRL_ON);
}

static const struct backlight_ops n72_pmu_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = n72_pmu_bl_update,
};

static int n72_pmu_bl_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct backlight_properties props = { };
	struct backlight_device *bl;
	unsigned int level;
	int ret;

	if (!regmap)
		return -ENODEV;
	ret = regmap_read(regmap, N72_PMU_BL_LEVEL, &level);
	if (ret)
		return ret;

	props.type = BACKLIGHT_RAW;
	props.max_brightness = N72_PMU_BL_MAX;
	props.brightness = min_t(unsigned int, level, N72_PMU_BL_MAX);
	/* under the PMU's device: the panel finds it by the PMU's node */
	bl = devm_backlight_device_register(&pdev->dev, "n72-backlight",
					    pdev->dev.parent, regmap,
					    &n72_pmu_bl_ops, &props);
	return PTR_ERR_OR_ZERO(bl);
}

static struct platform_driver n72_pmu_bl_driver = {
	.driver = {
		.name = "n72-pmu-backlight",
	},
	.probe = n72_pmu_bl_probe,
};
module_platform_driver(n72_pmu_bl_driver);

MODULE_ALIAS("platform:n72-pmu-backlight");
MODULE_DESCRIPTION("Apple iPod touch 2G (Dialog D1759) PMU backlight");
MODULE_LICENSE("GPL");
