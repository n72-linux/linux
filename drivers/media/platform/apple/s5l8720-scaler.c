// SPDX-License-Identifier: GPL-2.0-only
/*
 * Scaler and colour space converter of the Samsung/Apple S5L8720
 * (iPod touch 2G), as a V4L2 memory-to-memory device.
 *
 * This driver handles one job at a time from one physically contiguous
 * picture to another, an 8-tap horizontal and a 4-tap vertical polyphase
 * filter, and a 3x3 colour matrix. The block reports version 13 (register
 * 0x260), which does not support rotation or tiled buffers.
 *
 * A job configures source and destination (format, plane addresses, pitch in
 * pixels, size), the two scale steps, the filter taps for those steps, and
 * the matrix, then writes ENABLE and START; an interrupt signals completion.
 * A timeout resets the block.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "s5l8720-scaler-coef.h"

#define SC_CTRL			0x004
#define  SC_CTRL_START		BIT(0)
#define  SC_CTRL_RESET		BIT(1)	/* reads 0 when the reset is over */
#define  SC_CTRL_LIMITED	BIT(9)	/* the YCbCr side is limited range */
#define SC_ENABLE		0x008
#define SC_IRQSTAT		0x00c
#define  SC_IRQSTAT_DONE	BIT(0)	/* write 1 to acknowledge */
#define SC_SRC_CFG		0x010
#define  SC_CFG_FMT		GENMASK(2, 0)
#define  SC_CFG_BIT8		BIT(8)	/* iOS writes 0 */
#define  SC_CFG_TILED		BIT(10)
#define  SC_SRC_CFG_READY	BIT(0)	/* second step of the reset: set, wait for 1 */
#define SC_SRC_ADDR0		0x014
#define SC_SRC_ADDR1		0x018
#define SC_SRC_PITCH		0x01c	/* pixels; the same value in both halves */
#define SC_SRC_OFS		0x020
#define SC_SRC_SIZE		0x024	/* width << 16 | height */
#define SC_DST_CFG		0x030
#define SC_DST_ADDR0		0x034
#define SC_DST_ADDR1		0x038
#define SC_DST_PITCH		0x03c
#define SC_DST_SIZE		0x040
#define SC_HSCALE		0x050	/* source pixels per destination pixel, 3.16 */
#define SC_VSCALE		0x054
#define  SC_SCALE_MASK		GENMASK(18, 0)
#define SC_ROT			0x058
#define SC_HCOEF_A		0x060	/* 9 phases of 4 words */
#define SC_VCOEF_A		0x0f0	/* 9 phases of 2 words */
#define SC_HCOEF_B		0x140
#define SC_VCOEF_B		0x1d0
#define SC_CSC			0x220	/* 9 words, Q9, 12 bits two's complement */
#define SC_VERSION		0x260

#define SC_MIN_SIZE		4
#define SC_MAX_SRC		4092
#define SC_MAX_DST		4096
#define SC_JOB_TIMEOUT_MS	500

struct sc_fmt {
	u32 fourcc;
	u8 code;	/* SC_CFG_FMT */
	u8 bpp;		/* bytes per pixel of plane 0 */
	bool yuv;
	bool nv12;	/* a second plane of interleaved chroma follows plane 0 */
};

/*
 * Format code 1, packed YUYV, is in iOS's driver but not among the formats
 * it reports for this block, and on the device it does not work: a YUYV
 * source reads as zeros and a YUYV destination is not written. Left out.
 */
static const struct sc_fmt sc_formats[] = {
	{ V4L2_PIX_FMT_NV12,	0, 1, true,  true  },
	{ V4L2_PIX_FMT_RGB565,	4, 2, false, false },
	{ V4L2_PIX_FMT_XBGR32,	6, 4, false, false },	/* B, G, R, x in memory */
	{ V4L2_PIX_FMT_ABGR32,	6, 4, false, false },
};

/*
 * The BT.601 and BT.709 matrices, limited and full range, from the
 * standards' luma coefficients: rows of the result, columns of the input,
 * 512 = 1.0. Index: to RGB, BT.709, full range.
 */
static const s16 sc_csc[2][2][2][9] = {
	{	/* RGB to YCbCr: rows Y, Cb, Cr */
		{	/* BT.601 */
			{ 132, 258, 50, -76, -149, 225, 225, -188, -36 },
			{ 153, 301, 58, -86, -169, 255, 255, -214, -41 },
		}, {	/* BT.709 */
			{ 93, 313, 32, -51, -173, 224, 224, -203, -21 },
			{ 109, 366, 37, -59, -197, 255, 255, -232, -23 },
		},
	}, {	/* YCbCr to RGB: rows R, G, B */
		{
			{ 596, 0, 817, 596, -200, -416, 596, 1033, 0 },
			{ 512, 0, 718, 512, -176, -366, 512, 907, 0 },
		}, {
			{ 596, 0, 918, 596, -109, -273, 596, 1081, 0 },
			{ 512, 0, 806, 512, -96, -240, 512, 950, 0 },
		},
	},
};

struct sc_q {
	const struct sc_fmt *fmt;
	struct v4l2_pix_format pix;
	struct v4l2_rect crop;		/* source only: the part that is scaled */
	struct v4l2_rect compose;	/* destination only: where the scaled frame lands */
};

struct sc_dev;

struct sc_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;
	struct sc_dev *sc;
	struct sc_q src, dst;
	bool nearest;
};

struct sc_dev {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct v4l2_device v4l2;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;		/* the video device and its queues */
	spinlock_t job_lock;		/* cur: the interrupt against the timeout */
	struct sc_ctx *cur;
	struct delayed_work timeout;
	unsigned int streaming;		/* queues streaming: the block is on */
	unsigned int jobs, timeouts;
};

static inline u32 sc_rd(struct sc_dev *sc, u32 reg)
{
	return readl(sc->base + reg);
}

static inline void sc_wr(struct sc_dev *sc, u32 reg, u32 val)
{
	writel(val, sc->base + reg);
}

static const struct sc_fmt *sc_find_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sc_formats); i++)
		if (sc_formats[i].fourcc == fourcc)
			return &sc_formats[i];
	return NULL;
}

static struct sc_q *sc_get_q(struct sc_ctx *ctx, enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->src : &ctx->dst;
}

/* ------------------------------------------------------------ hardware */

/*
 * iOS's setActive(true) after the clocks: reset and wait for it, then set
 * bit 0 of the source configuration and wait until it reads back (what
 * that step is for is not known), then drop a stale completion.
 */
static int sc_hw_reset(struct sc_dev *sc)
{
	u32 v;
	int ret;

	sc_wr(sc, SC_CTRL, sc_rd(sc, SC_CTRL) | SC_CTRL_RESET);
	ret = readl_poll_timeout(sc->base + SC_CTRL, v, !(v & SC_CTRL_RESET),
				 100, 100000);
	if (ret)
		return ret;
	sc_wr(sc, SC_SRC_CFG, sc_rd(sc, SC_SRC_CFG) | SC_SRC_CFG_READY);
	ret = readl_poll_timeout(sc->base + SC_SRC_CFG, v, v & SC_SRC_CFG_READY,
				 100, 100000);
	if (ret)
		return ret;
	sc_wr(sc, SC_IRQSTAT, sc_rd(sc, SC_IRQSTAT) | SC_IRQSTAT_DONE);
	return 0;
}

/* Filter set index for a 16.16 fixed-point scale step. */
static unsigned int sc_filter_set(u32 step)
{
	static const u32 limit[] = {
		0x10000, 0x12492, 0x15555, 0x19999, 0x20000, 0x2aaaa, 0x40000,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(limit); i++)
		if (step <= limit[i])
			return i;
	return 0;	/* beyond 4:1 fall back to the sharp set */
}

/* Two taps a word, 9 bits each, the last pair first. */
static void sc_write_taps(struct sc_dev *sc, u32 reg, const s16 *tap, int taps)
{
	int i;

	for (i = taps - 2; i >= 0; i -= 2, reg += 4)
		sc_wr(sc, reg, (tap[i] & 0x1ff) | (tap[i + 1] & 0x1ff) << 16);
}

static void sc_hw_filters(struct sc_dev *sc, u32 hstep, u32 vstep, bool nearest)
{
	static const s16 h_near[8] = { 0, 0, 0, 0, 128, 0, 0, 0 };
	static const s16 v_near[4] = { 0, 0, 128, 0 };
	unsigned int hset = sc_filter_set(hstep), vset = sc_filter_set(vstep);
	unsigned int p;

	for (p = 0; p < 9; p++) {
		const s16 *ht = nearest ? h_near : s5l_scaler_h8[hset][p];
		const s16 *vt = nearest ? v_near : s5l_scaler_v4[vset][p];

		sc_write_taps(sc, SC_HCOEF_A + 16 * p, ht, 8);
		sc_write_taps(sc, SC_HCOEF_B + 16 * p, ht, 8);
		sc_write_taps(sc, SC_VCOEF_A + 8 * p, vt, 4);
		sc_write_taps(sc, SC_VCOEF_B + 8 * p, vt, 4);
	}
}

/*
 * One side's format, planes and pitch. A crop is in the addresses, as in
 * iOS: there is no register for an origin (x and y are even for YCbCr).
 */
static void sc_hw_side(struct sc_dev *sc, u32 cfg_reg, const struct sc_q *q,
		       dma_addr_t addr, u32 x, u32 y)
{
	u32 bpl = q->pix.bytesperline;
	u32 pitch = bpl / q->fmt->bpp;
	u32 cfg = sc_rd(sc, cfg_reg);

	cfg &= ~(SC_CFG_FMT | SC_CFG_BIT8 | SC_CFG_TILED);
	sc_wr(sc, cfg_reg, cfg | q->fmt->code);
	sc_wr(sc, cfg_reg + 0x04, addr + y * bpl + x * q->fmt->bpp);
	sc_wr(sc, cfg_reg + 0x08, q->fmt->nv12 ?
	      addr + bpl * q->pix.height + y / 2 * bpl + x : 0);
	sc_wr(sc, cfg_reg + 0x0c, pitch | pitch << 16);
}

static int sc_hw_start(struct sc_ctx *ctx, dma_addr_t src, dma_addr_t dst)
{
	struct sc_dev *sc = ctx->sc;
	const struct sc_q *s = &ctx->src, *d = &ctx->dst;
	/* the direction as iOS takes it: from the source alone */
	bool to_rgb = s->fmt->yuv;
	const struct v4l2_pix_format *yuv = to_rgb ? &s->pix : &d->pix;
	bool is709 = yuv->ycbcr_enc == V4L2_YCBCR_ENC_709;
	bool full = yuv->quantization == V4L2_QUANTIZATION_FULL_RANGE;
	/* sizes are below 4096, so 16.16 fits in 32 bits */
	u32 hstep = (s->crop.width << 16) / d->compose.width;
	u32 vstep = (s->crop.height << 16) / d->compose.height;
	const s16 *m = sc_csc[to_rgb][is709][full];
	u32 ctrl;
	int i;

	/* 19 bits: less than 8 source pixels a destination pixel */
	if (hstep > SC_SCALE_MASK || vstep > SC_SCALE_MASK)
		return -ERANGE;

	sc_hw_side(sc, SC_SRC_CFG, s, src, s->crop.left, s->crop.top);
	sc_wr(sc, SC_SRC_OFS, 0);
	sc_wr(sc, SC_SRC_SIZE, s->crop.width << 16 | s->crop.height);
	sc_hw_side(sc, SC_DST_CFG, d, dst, d->compose.left, d->compose.top);
	sc_wr(sc, SC_DST_SIZE, d->compose.width << 16 | d->compose.height);
	sc_wr(sc, SC_ROT, 0);

	ctrl = sc_rd(sc, SC_CTRL) & ~(SC_CTRL_LIMITED | SC_CTRL_START);
	if (!full)
		ctrl |= SC_CTRL_LIMITED;
	sc_wr(sc, SC_CTRL, ctrl);
	for (i = 0; i < 9; i++)
		sc_wr(sc, SC_CSC + 4 * i, m[i] & 0xfff);

	sc_wr(sc, SC_HSCALE, (sc_rd(sc, SC_HSCALE) & ~SC_SCALE_MASK) | hstep);
	sc_wr(sc, SC_VSCALE, (sc_rd(sc, SC_VSCALE) & ~SC_SCALE_MASK) | vstep);
	sc_hw_filters(sc, hstep, vstep, ctx->nearest);

	sc_wr(sc, SC_ENABLE, sc_rd(sc, SC_ENABLE) | 1);
	sc_wr(sc, SC_CTRL, ctrl | SC_CTRL_START);
	return 0;
}

/* --------------------------------------------------------------- jobs */

static void sc_job_done(struct sc_dev *sc, struct sc_ctx *ctx,
			enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *src, *dst;

	src = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (src && dst) {
		v4l2_m2m_buf_copy_metadata(src, dst, true);
		vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->dst.pix.sizeimage);
	}
	if (src)
		v4l2_m2m_buf_done(src, state);
	if (dst)
		v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_job_finish(sc->m2m, ctx->fh.m2m_ctx);
}

/* Whoever takes the running job, the interrupt or the timeout, ends it. */
static struct sc_ctx *sc_take_job(struct sc_dev *sc)
{
	struct sc_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&sc->job_lock, flags);
	ctx = sc->cur;
	sc->cur = NULL;
	spin_unlock_irqrestore(&sc->job_lock, flags);
	return ctx;
}

static irqreturn_t sc_irq(int irq, void *data)
{
	struct sc_dev *sc = data;
	struct sc_ctx *ctx;
	u32 stat = sc_rd(sc, SC_IRQSTAT);

	if (!(stat & SC_IRQSTAT_DONE))
		return IRQ_NONE;
	sc_wr(sc, SC_IRQSTAT, stat | SC_IRQSTAT_DONE);

	ctx = sc_take_job(sc);
	if (ctx) {
		cancel_delayed_work(&sc->timeout);
		sc->jobs++;
		sc_job_done(sc, ctx, VB2_BUF_STATE_DONE);
	}
	return IRQ_HANDLED;
}

static void sc_timeout(struct work_struct *work)
{
	struct sc_dev *sc = container_of(work, struct sc_dev, timeout.work);
	struct sc_ctx *ctx = sc_take_job(sc);

	if (!ctx)
		return;
	sc->timeouts++;
	dev_err(sc->dev, "job timed out: CTRL %08x IRQSTAT %08x; resetting\n",
		sc_rd(sc, SC_CTRL), sc_rd(sc, SC_IRQSTAT));
	sc_hw_reset(sc);
	sc_job_done(sc, ctx, VB2_BUF_STATE_ERROR);
}

static void sc_device_run(void *priv)
{
	struct sc_ctx *ctx = priv;
	struct sc_dev *sc = ctx->sc;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&sc->job_lock, flags);
	sc->cur = ctx;
	spin_unlock_irqrestore(&sc->job_lock, flags);
	/* armed before the start: a job can end before this function does */
	mod_delayed_work(system_wq, &sc->timeout,
			 msecs_to_jiffies(SC_JOB_TIMEOUT_MS));

	ret = sc_hw_start(ctx, vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0),
			  vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0));
	if (ret && sc_take_job(sc)) {
		cancel_delayed_work(&sc->timeout);
		sc_job_done(sc, ctx, VB2_BUF_STATE_ERROR);
	}
}

static const struct v4l2_m2m_ops sc_m2m_ops = {
	.device_run = sc_device_run,
};

/* ------------------------------------------------------------- queues */

static int sc_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			  unsigned int *nplanes, unsigned int sizes[],
			  struct device *alloc_devs[])
{
	struct sc_ctx *ctx = vb2_get_drv_priv(vq);
	struct sc_q *q = sc_get_q(ctx, vq->type);

	if (*nplanes)
		return sizes[0] < q->pix.sizeimage ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = q->pix.sizeimage;
	return 0;
}

static int sc_buf_prepare(struct vb2_buffer *vb)
{
	struct sc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct sc_q *q = sc_get_q(ctx, vb->vb2_queue->type);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vb2_plane_size(vb, 0) < q->pix.sizeimage)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE)
			return -EINVAL;
	}
	return 0;
}

static void sc_buf_queue(struct vb2_buffer *vb)
{
	struct sc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void sc_return_bufs(struct sc_ctx *ctx, struct vb2_queue *vq,
			   enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, state);
	}
}

/* The block is clocked and reset while any queue streams, as iOS has it on only while it has work. */
static int sc_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct sc_ctx *ctx = vb2_get_drv_priv(vq);
	struct sc_dev *sc = ctx->sc;
	int ret = 0;

	if (!sc->streaming) {
		ret = clk_prepare_enable(sc->clk);
		if (!ret) {
			ret = sc_hw_reset(sc);
			if (ret) {
				dev_err(sc->dev, "reset: %d (CTRL %08x)\n", ret,
					sc_rd(sc, SC_CTRL));
				clk_disable_unprepare(sc->clk);
			}
		}
	}
	if (ret) {
		sc_return_bufs(ctx, vq, VB2_BUF_STATE_QUEUED);
		return ret;
	}
	sc->streaming++;
	return 0;
}

static void sc_stop_streaming(struct vb2_queue *vq)
{
	struct sc_ctx *ctx = vb2_get_drv_priv(vq);
	struct sc_dev *sc = ctx->sc;

	/* the core has waited for a running job of this context */
	sc_return_bufs(ctx, vq, VB2_BUF_STATE_ERROR);
	if (!--sc->streaming) {
		cancel_delayed_work_sync(&sc->timeout);
		sc_wr(sc, SC_ENABLE, sc_rd(sc, SC_ENABLE) & ~1);
		clk_disable_unprepare(sc->clk);
	}
}

static const struct vb2_ops sc_qops = {
	.queue_setup = sc_queue_setup,
	.buf_prepare = sc_buf_prepare,
	.buf_queue = sc_buf_queue,
	.start_streaming = sc_start_streaming,
	.stop_streaming = sc_stop_streaming,
};

static int sc_queue_init(void *priv, struct vb2_queue *src_vq,
			 struct vb2_queue *dst_vq)
{
	struct sc_ctx *ctx = priv;
	struct vb2_queue *vq;
	int i, ret;

	for (i = 0; i < 2; i++) {
		vq = i ? dst_vq : src_vq;
		vq->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
		vq->io_modes = VB2_MMAP | VB2_DMABUF;
		vq->drv_priv = ctx;
		vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		vq->ops = &sc_qops;
		vq->mem_ops = &vb2_dma_contig_memops;
		vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		vq->lock = &ctx->sc->lock;
		vq->dev = ctx->sc->dev;
		ret = vb2_queue_init(vq);
		if (ret)
			return ret;
	}
	return 0;
}

/* ------------------------------------------------------------- ioctls */

static int sc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "s5l8720-scaler", sizeof(cap->driver));
	strscpy(cap->card, "S5L8720 scaler", sizeof(cap->card));
	return 0;
}

static int sc_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(sc_formats))
		return -EINVAL;
	f->pixelformat = sc_formats[f->index].fourcc;
	return 0;
}

static int sc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc_ctx *ctx = container_of(file->private_data, struct sc_ctx, fh);

	f->fmt.pix = sc_get_q(ctx, f->type)->pix;
	return 0;
}

static const struct sc_fmt *sc_adjust(struct v4l2_pix_format *pix, bool output)
{
	const struct sc_fmt *fmt = sc_find_fmt(pix->pixelformat);
	unsigned int max = output ? SC_MAX_SRC : SC_MAX_DST;
	u32 min_bpl;

	if (!fmt) {
		fmt = &sc_formats[0];
		pix->pixelformat = fmt->fourcc;
	}
	/* iOS keeps YCbCr sizes even */
	v4l_bound_align_image(&pix->width, SC_MIN_SIZE, max, fmt->yuv,
			      &pix->height, SC_MIN_SIZE, max, fmt->yuv, 0);
	min_bpl = pix->width * fmt->bpp;
	/* the pitch register counts pixels in 16 bits */
	pix->bytesperline = clamp(pix->bytesperline, min_bpl, 0xffffU * fmt->bpp);
	pix->bytesperline = rounddown(pix->bytesperline, fmt->bpp);
	if (fmt->nv12)
		pix->bytesperline = round_up(pix->bytesperline, 2);
	pix->sizeimage = pix->bytesperline * pix->height;
	if (fmt->nv12)
		pix->sizeimage += pix->sizeimage / 2;
	pix->field = V4L2_FIELD_NONE;
	if (fmt->yuv) {
		if (pix->ycbcr_enc != V4L2_YCBCR_ENC_709)
			pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
		if (pix->quantization != V4L2_QUANTIZATION_FULL_RANGE)
			pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		if (pix->colorspace == V4L2_COLORSPACE_DEFAULT)
			pix->colorspace = pix->ycbcr_enc == V4L2_YCBCR_ENC_709 ?
					  V4L2_COLORSPACE_REC709 :
					  V4L2_COLORSPACE_SMPTE170M;
	} else {
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	}
	return fmt;
}

static int sc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	sc_adjust(&f->fmt.pix, V4L2_TYPE_IS_OUTPUT(f->type));
	return 0;
}

static int sc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct sc_ctx *ctx = container_of(file->private_data, struct sc_ctx, fh);
	struct vb2_queue *vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	struct sc_q *q = sc_get_q(ctx, f->type);

	if (vb2_is_busy(vq))
		return -EBUSY;
	q->fmt = sc_adjust(&f->fmt.pix, V4L2_TYPE_IS_OUTPUT(f->type));
	q->pix = f->fmt.pix;
	q->crop = (struct v4l2_rect){ 0, 0, q->pix.width, q->pix.height };
	q->compose = (struct v4l2_rect){ 0, 0, q->pix.width, q->pix.height };
	return 0;
}

/* The source can be cropped and the destination composed into a sub-rectangle. */
static int sc_g_selection(struct file *file, void *priv, struct v4l2_selection *sel)
{
	struct sc_ctx *ctx = container_of(file->private_data, struct sc_ctx, fh);
	bool output = sel->type == V4L2_BUF_TYPE_VIDEO_OUTPUT;
	const struct sc_q *q = output ? &ctx->src : &ctx->dst;

	if (!output && sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		if (!output)
			return -EINVAL;
		sel->r = q->crop;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (!output)
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		if (output)
			return -EINVAL;
		sel->r = q->compose;
		return 0;
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		if (output)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	sel->r = (struct v4l2_rect){ 0, 0, q->pix.width, q->pix.height };
	return 0;
}

static int sc_s_selection(struct file *file, void *priv, struct v4l2_selection *sel)
{
	struct sc_ctx *ctx = container_of(file->private_data, struct sc_ctx, fh);
	bool output;
	struct sc_q *q;
	struct v4l2_rect *r = &sel->r;
	unsigned int align;

	if (sel->type == V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    sel->target == V4L2_SEL_TGT_CROP)
		output = true;
	else if (sel->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		 sel->target == V4L2_SEL_TGT_COMPOSE)
		output = false;
	else
		return -EINVAL;
	if (vb2_is_streaming(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, sel->type)))
		return -EBUSY;
	q = output ? &ctx->src : &ctx->dst;
	align = q->fmt->yuv;	/* bit 0: YCbCr rectangles are even */
	r->left = clamp_t(int, r->left, 0, q->pix.width - SC_MIN_SIZE) & ~align;
	r->top = clamp_t(int, r->top, 0, q->pix.height - SC_MIN_SIZE) & ~align;
	r->width = clamp_t(u32, r->width, SC_MIN_SIZE, q->pix.width - r->left) & ~align;
	r->height = clamp_t(u32, r->height, SC_MIN_SIZE, q->pix.height - r->top) & ~align;
	if (output)
		q->crop = *r;
	else
		q->compose = *r;
	return 0;
}

static const struct v4l2_ioctl_ops sc_ioctl_ops = {
	.vidioc_querycap = sc_querycap,
	.vidioc_enum_fmt_vid_cap = sc_enum_fmt,
	.vidioc_enum_fmt_vid_out = sc_enum_fmt,
	.vidioc_g_fmt_vid_cap = sc_g_fmt,
	.vidioc_g_fmt_vid_out = sc_g_fmt,
	.vidioc_try_fmt_vid_cap = sc_try_fmt,
	.vidioc_try_fmt_vid_out = sc_try_fmt,
	.vidioc_s_fmt_vid_cap = sc_s_fmt,
	.vidioc_s_fmt_vid_out = sc_s_fmt,
	.vidioc_g_selection = sc_g_selection,
	.vidioc_s_selection = sc_s_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

/* --------------------------------------------------------------- file */

static int sc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc_ctx *ctx = container_of(ctrl->handler, struct sc_ctx, ctrl_handler);

	if (ctrl->id == V4L2_CID_SHARPNESS) {
		ctx->nearest = ctrl->val != 0;
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sc_ctrl_ops = {
	.s_ctrl = sc_s_ctrl,
};

static int sc_open(struct file *file)
{
	struct sc_dev *sc = video_drvdata(file);
	struct sc_ctx *ctx;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->sc = sc;
	ctx->src.pix.width = ctx->dst.pix.width = 320;
	ctx->src.pix.height = ctx->dst.pix.height = 240;
	ctx->src.pix.pixelformat = V4L2_PIX_FMT_NV12;
	ctx->dst.pix.pixelformat = V4L2_PIX_FMT_XBGR32;
	ctx->src.fmt = sc_adjust(&ctx->src.pix, true);
	ctx->dst.fmt = sc_adjust(&ctx->dst.pix, false);
	ctx->src.crop = (struct v4l2_rect){ 0, 0, ctx->src.pix.width,
					    ctx->src.pix.height };
	ctx->dst.compose = (struct v4l2_rect){ 0, 0, ctx->dst.pix.width,
					       ctx->dst.pix.height };

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 1);
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &sc_ctrl_ops,
			  V4L2_CID_SHARPNESS, 0, 1, 1, 0);
	if (ctx->ctrl_handler.error) {
		ret = ctx->ctrl_handler.error;
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		kfree(ctx);
		return ret;
	}

	if (mutex_lock_interruptible(&sc->lock)) {
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		kfree(ctx);
		return -ERESTARTSYS;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	file->private_data = &ctx->fh;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(sc->m2m, ctx, sc_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		kfree(ctx);
	} else {
		v4l2_fh_add(&ctx->fh);
	}
	mutex_unlock(&sc->lock);
	return ret;
}

static int sc_release(struct file *file)
{
	struct sc_ctx *ctx = container_of(file->private_data, struct sc_ctx, fh);
	struct sc_dev *sc = ctx->sc;

	mutex_lock(&sc->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&sc->lock);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations sc_fops = {
	.owner = THIS_MODULE,
	.open = sc_open,
	.release = sc_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ------------------------------------------------------------- driver */

#ifdef CONFIG_DEBUG_FS
/* debugfs <device>/stats */
static int sc_stats_show(struct seq_file *s, void *unused)
{
	struct sc_dev *sc = s->private;

	seq_printf(s, "jobs %u timeouts %u\n", sc->jobs, sc->timeouts);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sc_stats);

static void sc_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static int sc_debugfs_init(struct device *dev, struct sc_dev *sc)
{
	struct dentry *dir = debugfs_create_dir(dev_name(dev), NULL);

	debugfs_create_file("stats", 0444, dir, sc, &sc_stats_fops);
	return devm_add_action_or_reset(dev, sc_debugfs_remove, dir);
}
#else
static inline int sc_debugfs_init(struct device *dev, struct sc_dev *sc)
{
	return 0;
}
#endif

static int sc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc_dev *sc;
	struct video_device *vdev;
	u32 version;
	int irq, ret;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;
	sc->dev = dev;
	mutex_init(&sc->lock);
	spin_lock_init(&sc->job_lock);
	INIT_DELAYED_WORK(&sc->timeout, sc_timeout);
	platform_set_drvdata(pdev, sc);

	sc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sc->base))
		return PTR_ERR(sc->base);
	/* the gate: the block must not be touched without it */
	sc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(sc->clk))
		return dev_err_probe(dev, PTR_ERR(sc->clk), "clock\n");
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));

	/* as iOS at start: clock on, read the version without a reset, clock off */
	ret = clk_prepare_enable(sc->clk);
	if (ret)
		return ret;
	version = sc_rd(sc, SC_VERSION);
	clk_disable_unprepare(sc->clk);

	ret = devm_request_irq(dev, irq, sc_irq, 0, dev_name(dev), sc);
	if (ret)
		return ret;

	ret = sc_debugfs_init(dev, sc);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &sc->v4l2);
	if (ret)
		return ret;
	sc->m2m = v4l2_m2m_init(&sc_m2m_ops);
	if (IS_ERR(sc->m2m)) {
		ret = PTR_ERR(sc->m2m);
		goto err_v4l2;
	}

	vdev = &sc->vdev;
	strscpy(vdev->name, "s5l8720-scaler", sizeof(vdev->name));
	vdev->fops = &sc_fops;
	vdev->ioctl_ops = &sc_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &sc->lock;
	vdev->v4l2_dev = &sc->v4l2;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, sc);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	dev_info(dev, "S5L8720 scaler, version %u, %s\n", version,
		 video_device_node_name(vdev));
	return 0;

err_m2m:
	v4l2_m2m_release(sc->m2m);
err_v4l2:
	v4l2_device_unregister(&sc->v4l2);
	return ret;
}

static void sc_remove(struct platform_device *pdev)
{
	struct sc_dev *sc = platform_get_drvdata(pdev);

	video_unregister_device(&sc->vdev);
	v4l2_m2m_release(sc->m2m);
	v4l2_device_unregister(&sc->v4l2);
}

static const struct of_device_id sc_of_match[] = {
	{ .compatible = "apple,s5l8720-scaler" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc_of_match);

static struct platform_driver sc_driver = {
	.probe = sc_probe,
	.remove = sc_remove,
	.driver = {
		.name = "s5l8720-scaler",
		.of_match_table = sc_of_match,
	},
};
module_platform_driver(sc_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8720 scaler and colour space converter");
MODULE_LICENSE("GPL");
