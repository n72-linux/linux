/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PMU of the Apple iPod touch (2nd generation), N72: what its core
 * (drivers/mfd/apple-n72-pmu-core.c) gives its sub-devices and the rest of
 * the kernel.
 */
#ifndef __LINUX_MFD_APPLE_N72_PMU_H
#define __LINUX_MFD_APPLE_N72_PMU_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

#define N72_PMU_BL_LEVEL	0x30
#define N72_PMU_BL_CTRL		0x31
#define N72_PMU_BL_CTRL_ON	0x05

#define N72_PMU_STATUS		0x04

#define N72_PMU_INT1_ALARM	BIT(2)
#define N72_PMU_RTC_ALARM_Y	0x5a
#define N72_PMU_RTC_ALARM_ON	0x40

struct device;
struct notifier_block;
struct n72_pmu;

/*
 * Bits of INT1's mask (0x07) for a sub-device: the core and the other
 * sub-devices change the register too.
 */
int n72_pmu_int1_mask(struct n72_pmu *pmu, u8 bits, bool masked);

/*
 * INT2 (0x02, cleared on read) for a sub-device: the bits asked for that
 * were set since they were last taken, or a negative error. The interrupt
 * takes the others.
 */
int n72_pmu_int2_take(struct n72_pmu *pmu, u8 bits);

/* The USB plug and unplug interrupts unmasked (true) or masked. */
void n72_pmu_usb_irq(struct n72_pmu *pmu, bool on);

/*
 * The PMU's interrupt, as events to a notifier (event: enum n72_pmu_event),
 * called in process context where the core reads the interrupt status.
 * -ENXIO if the PMU has no interrupt: there are no events then.
 */
enum n72_pmu_event {
	N72_PMU_EVENT_ALARM,		/* INT1 bit 2: the RTC alarm rang */
	N72_PMU_EVENT_USB_IN,		/* INT1 bit 3: USB power came */
	N72_PMU_EVENT_USB_OUT,		/* INT3 bit 2: USB power went (and none came) */
};

int devm_n72_pmu_notifier(struct device *dev, struct n72_pmu *pmu,
			  struct notifier_block *nb);

#if IS_REACHABLE(CONFIG_BACKLIGHT_APPLE_N72_PMU)
void n72_pmu_bl_set_mie_scale(u8 scale);
#else
static inline void n72_pmu_bl_set_mie_scale(u8 scale) {}
#endif

#endif
