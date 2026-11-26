/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The iPod touch 2G display (drivers/gpu/drm/tiny/n72-lcd.c) can power the
 * panel down and gate the CLCD and DSI clocks while the system runs (its
 * sysfs attribute panel_power), as iOS does whenever the display is off.
 *
 * Other users of the CLCD registers (the DRM driver, the MBX page flips)
 * register a client notifier. On N72_LCD_OFF they stop touching those
 * registers before returning: a gated block hangs the bus when accessed.
 * On N72_LCD_ON the controller is clocked and set up again, the window
 * registers as they were at N72_LCD_OFF; they may write them again.
 * N72_LCD_RESUMED follows a system sleep's panel bring-up, which sets the
 * controller up from scratch (its interrupt enables among it) after the
 * clients have resumed: they put back what they had enabled.
 */
#ifndef _LINUX_SOC_APPLE_N72_LCD_H
#define _LINUX_SOC_APPLE_N72_LCD_H

#include <linux/errno.h>
#include <linux/notifier.h>

enum n72_lcd_event {
	N72_LCD_OFF,
	N72_LCD_ON,
	N72_LCD_RESUMED,
};

#if IS_ENABLED(CONFIG_DRM_N72_LCD)
int n72_lcd_register_client(struct notifier_block *nb);
void n72_lcd_unregister_client(struct notifier_block *nb);
#else
static inline int n72_lcd_register_client(struct notifier_block *nb)
{
	return 0;
}

static inline void n72_lcd_unregister_client(struct notifier_block *nb)
{
}
#endif

/*
 * The low core voltage state (drivers/soc/apple/n72-corev.c): told
 * before the display's clocks open (true: returns at 1.20 V, once that
 * driver has probed) and after they have closed (false).
 */
#if IS_ENABLED(CONFIG_APPLE_N72_COREV)
void n72_corev_display(bool on);
#else
static inline void n72_corev_display(bool on)
{
}
#endif

#endif
