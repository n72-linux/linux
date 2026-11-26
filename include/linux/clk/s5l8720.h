/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_CLK_S5L8720_H
#define __LINUX_CLK_S5L8720_H

#include <linux/errno.h>
#include <linux/types.h>

/*
 * The AMC audio coprocessor's clock divider register (AMC + 0x1000) and
 * its codec's largest system divider while its clock gate is open, NULL
 * when it is gated: the divided idle states keep its clock at or under
 * S5L8720_AMC_MAX_HZ and the system divider within max_div, as iOS does.
 */
#define S5L8720_AMC_MAX_HZ	34000000UL
#ifdef CONFIG_ARM_S5L8720_CPUIDLE
void s5l8720_idle_amc_clock(void __iomem *div_reg, unsigned long bus_hz,
			    unsigned int max_div);
/*
 * The system clock divider outside idle: 1, or 2 while the core runs at
 * 1.05 V (iOS's low voltage state). Set 2 before lowering the voltage and
 * 1 only after raising it.
 */
int s5l8720_idle_set_active_div(unsigned int div);
void s5l8720_idle_amc_resync(void);
#else
static inline int s5l8720_idle_set_active_div(unsigned int div) { return -ENODEV; }
static inline void s5l8720_idle_amc_resync(void) { }
static inline void s5l8720_idle_amc_clock(void __iomem *div_reg, unsigned long bus_hz,
					  unsigned int max_div) { }
#endif

#endif
