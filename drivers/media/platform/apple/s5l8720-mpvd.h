/* SPDX-License-Identifier: GPL-2.0 */
/* Internal definitions for the S5L8720 MPEG-4 / H.263 decoder driver. */
#ifndef S5L8720_MPVD_H
#define S5L8720_MPVD_H

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

/* the units, in the order of the device's memory regions */
enum { DDM, MP4, VLD, IQ, IMC, MI, MPVD_UNITS };

/* common to the units: interrupt status at 0 (VLD: 0x100), command at 0x0c */
#define DDM_IRQC		0x00
#define  DDM_IRQC_VLD_STUCK	0x40	/* exactly this: leave VLD alone */
#define  DDM_IRQC_TIMEOUT	BIT(9)
#define DDM_IRQE		0x04	/* 0x142 */
#define DDM_CMD			0x0c
#define DDM_SIZE		0x10	/* bit 31, MBs - 1: width << 16, height << 8 */
#define DDM_IRQ_ALL		0x1c
#define DDM_CLC			0x2c	/* clock control: 2 = on, 0 = off */
#define MP4_IRQC		0x00
#define  MP4_IRQC_DONE		BIT(2)
#define MP4_IRQE		0x04	/* 0x3f */
#define MP4_CMD			0x0c
#define  MP4_CMD_START		0xc
#define MP4_PIC			0x10
#define  MP4_PIC_SHORT_HEADER	BIT(10)
#define MP4_QUANT		0x14
#define MP4_1C			0x1c	/* 1 */
#define MP4_34			0x34	/* 3, 0 for H.263 */
#define MP4_TIME		0x9c
#define MP4_A0			0xa0
#define VLD_GETBITS(n)		(4 * (n))	/* read: skip n bits, n < 32 */
#define VLD_IRQC		0x100
#define  VLD_IRQC_DONE		BIT(5)
#define VLD_IRQE		0x104	/* 0x14 */
#define VLD_MODE		0x110
#define IQ_INTRA(i)		(0x200 + 4 * (i))	/* raster order */
#define IQ_INTER(i)		(0x300 + 4 * (i))
#define IQ_MODE			0x804
#define IMC_IRQC		0x00
#define  IMC_IRQC_DONE		BIT(1)
#define IMC_MODE		0x10
#define MI_IRQC			0x00
#define  MI_IRQC_STARVED	BIT(6)
#define MI_IRQE			0x04	/* 0x50 */
#define MI_CMD			0x0c
#define  MI_CMD_FETCH		3
#define MI_10			0x10	/* 0x210 */
#define MI_STREAM		0x18	/* moves on as the unit fetches */
#define MI_STREAM_END		0x1c
#define MI_USER			0x20
#define MI_USER_END		0x24
#define MI_REF_Y		0x2c
#define MI_REF_C		0x34
#define MI_OUT_Y		0x3c
#define MI_OUT_C		0x44
#define MI_SIZE			0x6c	/* MBs: width << 16 | height */
#define MI_DP			0x78
#define MI_DP_SIZE		0x7c

#define MPVD_USER_SIZE		SZ_1K
#define MPVD_DP_SIZE		SZ_256K
#define MPVD_STREAM_SLACK	0x103	/* end = start + length + slack */
#define MPVD_TIMEOUT_MS		300
#define MPVD_FLUSH_US		150	/* from the interrupt to the block's sleep */

#define MPVD_MIN_SIZE		48U
#define MPVD_MAX_WIDTH		720U
#define MPVD_MAX_HEIGHT		576U

struct mpvd_vol {
	bool valid, short_header, quant_type;
	u8 vti_bits, quant_precision;
	u16 width, height;
	u8 intra[64], inter[64];	/* raster order */
};

struct mpvd_vop {
	u8 type, rounding, intra_dc_thr, quant, fcode;
	bool coded;
	u16 time_inc;
	u8 num_gobs;
	u16 mbs_per_gob;
	u32 offset;		/* of the start code in the buffer */
	u32 header_bits;	/* from the start code to the picture data */
	u32 len;		/* from the start code to the end of the data */
};

struct mpvd_dev;

struct mpvd_ctx {
	struct v4l2_fh fh;
	struct mpvd_dev *md;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_pix_format src_fmt, dst_fmt;
	struct mpvd_vol vol;
	/* the last picture decoded: the next one's reference */
	struct vb2_v4l2_buffer *ref;
	struct vb2_v4l2_buffer *dst;	/* of the running job */
	u32 sequence;
	bool announced;		/* the stream's size was told (SOURCE_CHANGE) */
};

struct mpvd_dev {
	struct device *dev;
	void __iomem *unit[MPVD_UNITS], *rst;
	struct clk_bulk_data *clks;
	int nclks;
	struct v4l2_device v4l2;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;		/* the video device and its queues */
	spinlock_t job_lock;		/* cur: the interrupt against the timeout */
	struct mpvd_ctx *cur;
	struct delayed_work timeout;
	unsigned int streaming;		/* queues streaming: the gates are open */
	void *user, *dp;		/* work memory of the block */
	dma_addr_t user_dma, dp_dma;
	unsigned int pictures, errors, timeouts, refused;
	u32 last[5];			/* DDM, MP4, VLD, IMC, MI status */
};

static inline u32 m_rd(struct mpvd_dev *md, int unit, u32 reg)
{
	return readl(md->unit[unit] + reg);
}

static inline void m_wr(struct mpvd_dev *md, int unit, u32 reg, u32 val)
{
	writel(val, md->unit[unit] + reg);
}

int mpvd_parse(struct mpvd_ctx *ctx, const u8 *d, u32 len, struct mpvd_vop *p);

#endif
