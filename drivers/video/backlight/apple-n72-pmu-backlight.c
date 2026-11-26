// SPDX-License-Identifier: GPL-2.0-only
/*
 * PMU of the Apple iPod touch (2nd generation), N72: the display backlight.
 * See drivers/mfd/apple-n72-pmu-core.c.
 */

#include <linux/backlight.h>
#include <linux/mfd/apple-n72-pmu.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define N72_PMU_BL_MAX		245	/* maximum panel backlight level */

struct n72_pmu_bl {
	struct regmap *regmap;
	struct backlight_device *bl;
	u8 mie_scale;
};

static DEFINE_MUTEX(n72_pmu_bl_lock);
static struct n72_pmu_bl *n72_pmu_bl_inst;

static int n72_pmu_bl_update(struct backlight_device *bl)
{
	struct n72_pmu_bl *p = bl_get_data(bl);
	int level = backlight_get_brightness(bl);
	int hw, ret;

	if (!level)
		return regmap_write(p->regmap, N72_PMU_BL_CTRL, 0);

	hw = max_t(int, 1, (level * READ_ONCE(p->mie_scale) + 127) / 255);
	ret = regmap_write(p->regmap, N72_PMU_BL_LEVEL, hw);
	if (ret)
		return ret;
	return regmap_write(p->regmap, N72_PMU_BL_CTRL, N72_PMU_BL_CTRL_ON);
}

void n72_pmu_bl_set_mie_scale(u8 scale)
{
	struct n72_pmu_bl *p;
	bool update = false;

	mutex_lock(&n72_pmu_bl_lock);
	p = n72_pmu_bl_inst;
	if (p) {
		u8 clamped = max_t(u8, 1, scale);

		if (p->mie_scale != clamped) {
			WRITE_ONCE(p->mie_scale, clamped);
			update = backlight_get_brightness(p->bl) > 0;
		}
	}
	mutex_unlock(&n72_pmu_bl_lock);
	if (update)
		backlight_update_status(p->bl);
}
EXPORT_SYMBOL_GPL(n72_pmu_bl_set_mie_scale);

static const struct backlight_ops n72_pmu_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = n72_pmu_bl_update,
};

static void n72_pmu_bl_unregister_inst(void *data)
{
	struct n72_pmu_bl *p = data;

	mutex_lock(&n72_pmu_bl_lock);
	if (n72_pmu_bl_inst == p)
		n72_pmu_bl_inst = NULL;
	mutex_unlock(&n72_pmu_bl_lock);
}

static int n72_pmu_bl_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct backlight_properties props = { };
	struct n72_pmu_bl *p;
	unsigned int level;
	int ret;

	if (!regmap)
		return -ENODEV;
	ret = regmap_read(regmap, N72_PMU_BL_LEVEL, &level);
	if (ret)
		return ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->regmap = regmap;
	p->mie_scale = 255;

	props.type = BACKLIGHT_RAW;
	props.max_brightness = N72_PMU_BL_MAX;
	props.brightness = min_t(unsigned int, level, N72_PMU_BL_MAX);
	/* under the PMU's device: the panel finds it by the PMU's node */
	p->bl = devm_backlight_device_register(&pdev->dev, "n72-backlight",
					       pdev->dev.parent, p,
					       &n72_pmu_bl_ops, &props);
	if (IS_ERR(p->bl))
		return PTR_ERR(p->bl);

	mutex_lock(&n72_pmu_bl_lock);
	n72_pmu_bl_inst = p;
	mutex_unlock(&n72_pmu_bl_lock);

	return devm_add_action_or_reset(&pdev->dev, n72_pmu_bl_unregister_inst, p);
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
