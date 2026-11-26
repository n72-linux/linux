#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>

/*
 * The boot loader (iBSS, for its own USB DFU) programs more of the PHY than
 * the power-on sequence below writes: 0x14, 0x20, 0x24, 0x28, 0x40, 0x44.
 * They are back at their reset values after a suspend that cut the SoC's
 * power, so keep what the boot loader set and write it back on power on.
 * Without 0x44 the PHY never links.
 */
#define S5L8720_OTGPHY_SAVE0 0x0c
#define S5L8720_OTGPHY_NSAVE ((0x4c - 0x0c) / 4 + 1)

struct s5l8720_usbphy {
	struct device *dev;
	struct phy *phy;
	void __iomem *base;
	u32 saved[S5L8720_OTGPHY_NSAVE];	/* 0x0c..0x4c as the boot loader set them */
};

#define S5L8720_OTGPHY_PWR 0x00
#define S5L8720_OTGPHY_CON 0x04
#define S5L8720_OTGPHY_RSTCON 0x08
#define S5L8720_OTGPHY_UNKCON 0x1C
#define S5L8720_OTGPHY_UNK2 0x44
#define S5L8720_OTGPHY_UNK2_DEFAULT 0xe3f	/* if the boot loader left it 0 */

static int s5l8720_usbphy_phy_init(struct phy *phy)
{
	return 0;
}

static int s5l8720_usbphy_phy_exit(struct phy *phy)
{
	return 0;
}

/*
 * Power-on sequence: power on, tuning registers and clock select, then a
 * 20 us reset pulse and 1 ms settle. Uses sleeping delays rather than
 * busy-waiting so other devices' resume paths are not stalled.
 */
static int s5l8720_usbphy_phy_power_on(struct phy *phy)
{
	struct s5l8720_usbphy *usbphy = phy_get_drvdata(phy);
	int i;

	writel_relaxed(0, usbphy->base + S5L8720_OTGPHY_PWR);
	for (i = 0; i < S5L8720_OTGPHY_NSAVE; i++)
		writel_relaxed(usbphy->saved[i],
			       usbphy->base + S5L8720_OTGPHY_SAVE0 + i * 4);
	writel_relaxed(6, usbphy->base + S5L8720_OTGPHY_UNKCON);
	usleep_range(10, 20);
	writel_relaxed(1, usbphy->base + S5L8720_OTGPHY_CON);	/* 24 MHz reference */
	writel_relaxed(1, usbphy->base + S5L8720_OTGPHY_RSTCON);
	udelay(20);
	writel_relaxed(0, usbphy->base + S5L8720_OTGPHY_RSTCON);
	msleep(10);		/* iBoot waits 1 ms */
	return 0;
}

static int s5l8720_usbphy_phy_power_off(struct phy *phy)
{
	struct s5l8720_usbphy *usbphy = phy_get_drvdata(phy);

	writel_relaxed(0xff, usbphy->base + S5L8720_OTGPHY_PWR);
	usleep_range(10, 20);
	writel_relaxed(0xff, usbphy->base + S5L8720_OTGPHY_RSTCON);
	usleep_range(10, 20);
	writel_relaxed(4, usbphy->base + S5L8720_OTGPHY_UNKCON);
	return 0;
}


static const struct phy_ops s5l8720_usbphy_phy_ops = {
	.init = s5l8720_usbphy_phy_init,
	.exit = s5l8720_usbphy_phy_exit,
	.power_on = s5l8720_usbphy_phy_power_on,
	.power_off = s5l8720_usbphy_phy_power_off,
	.owner = THIS_MODULE,
};

static int s5l8720_usbphy_probe(struct platform_device *pdev)
{
	struct s5l8720_usbphy *usbphy;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	int ret, i;

	usbphy = devm_kzalloc(dev, sizeof(*usbphy), GFP_KERNEL);
	if (!usbphy)
		return -ENOMEM;
	usbphy->dev = dev;
	dev_set_drvdata(dev, usbphy);

	usbphy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(usbphy->base))
		return PTR_ERR(usbphy->base);

	for (i = 0; i < S5L8720_OTGPHY_NSAVE; i++)
		usbphy->saved[i] = readl_relaxed(usbphy->base + S5L8720_OTGPHY_SAVE0 + i * 4);
	i = (S5L8720_OTGPHY_UNK2 - S5L8720_OTGPHY_SAVE0) / 4;
	if (!usbphy->saved[i])
		usbphy->saved[i] = S5L8720_OTGPHY_UNK2_DEFAULT;

	usbphy->phy = devm_phy_create(dev, NULL, &s5l8720_usbphy_phy_ops);
	if (IS_ERR(usbphy->phy)) {
		ret = PTR_ERR(usbphy->phy);
		dev_err(dev, "failed to create phy: %d\n", ret);
		return ret;
	}

	phy_set_drvdata(usbphy->phy, usbphy);
	phy_provider = devm_of_phy_provider_register(&pdev->dev,
							 of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static void s5l8720_usbphy_remove(struct platform_device *pdev)
{
}

static const struct of_device_id s5l8720_usbphy_of_match[] = {
	{ .compatible = "apple,s5l8720-otgphy", },
	{ },
};
MODULE_DEVICE_TABLE(of, s5l8720_usbphy_of_match);

static struct platform_driver s5l8720_usbphy_driver = {
	.probe = s5l8720_usbphy_probe,
	.remove = s5l8720_usbphy_remove,
	.driver = {
		.of_match_table = s5l8720_usbphy_of_match,
		.name = "s5l8720-usbphy",
	}
};
module_platform_driver(s5l8720_usbphy_driver);
