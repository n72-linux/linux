/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * S5L8720 USB OTG PHY D+/D- analog monitor mux (DMON) control for
 * charger brick identification via PMU ADC channel 6.
 */
#ifndef __LINUX_SOC_APPLE_S5L8720_USBPHY_H
#define __LINUX_SOC_APPLE_S5L8720_USBPHY_H

#include <linux/errno.h>
#include <linux/kconfig.h>

#define S5L8720_USBPHY_DMON_OFF		0
#define S5L8720_USBPHY_DMON_DP		1
#define S5L8720_USBPHY_DMON_DM		2

#if IS_ENABLED(CONFIG_PHY_S5L8720_USB2)
int s5l8720_usbphy_set_dmonitor(unsigned int mode);
#else
static inline int s5l8720_usbphy_set_dmonitor(unsigned int mode)
{
	return -ENODEV;
}
#endif

#endif
