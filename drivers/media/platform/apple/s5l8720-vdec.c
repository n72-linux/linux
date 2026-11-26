// SPDX-License-Identifier: GPL-2.0-only
/* Who has the S5L8720 video decoders' memory path; see s5l8720-vdec.h. */

#include <linux/export.h>
#include <linux/module.h>
#include <linux/semaphore.h>

#include "s5l8720-vdec.h"

/* released from interrupt handlers, hence not a mutex */
static DEFINE_SEMAPHORE(s5l8720_vdec_sem, 1);

void s5l8720_vdec_claim(void __iomem *rst, u32 sel)
{
	down(&s5l8720_vdec_sem);
	if (rst)
		writel(sel, rst + S5L8720_VDEC_SEL);
}
EXPORT_SYMBOL_GPL(s5l8720_vdec_claim);

void s5l8720_vdec_release(void)
{
	up(&s5l8720_vdec_sem);
}
EXPORT_SYMBOL_GPL(s5l8720_vdec_release);

MODULE_DESCRIPTION("Samsung/Apple S5L8720 video decoder memory path");
MODULE_LICENSE("GPL");
