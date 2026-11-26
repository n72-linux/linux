/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The S5L8720's two video decoder blocks share one path to memory. A
 * register of the reset block at 0x3e000000 says which of them has it:
 * iOS's H.264 driver writes 1 there ("the AXI bus"), and the platform
 * writes 0 around every picture of the MPEG-4 block. A block without the
 * path fetches nothing, and reads of it that wait for bitstream then stop
 * the bus for good, so a decoder takes the path for each job.
 */
#ifndef S5L8720_VDEC_H
#define S5L8720_VDEC_H

#include <linux/io.h>

#define S5L8720_VDEC_SEL	0x04
#define S5L8720_VDEC_SEL_MPVD	0
#define S5L8720_VDEC_SEL_H264	1

/* May sleep while the other block finishes its job. rst may be NULL. */
void s5l8720_vdec_claim(void __iomem *rst, u32 sel);
/* From any context, once for every claim. */
void s5l8720_vdec_release(void);

#endif
