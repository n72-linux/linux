// SPDX-License-Identifier: GPL-2.0-only
/*
 * MPEG-4 Part 2 and H.263 decoder of the Samsung/Apple S5L8720 (iPod
 * touch 2G), iOS's "mpvd,s5l8720x", as a V4L2 stateful decoder.
 *
 * The block decodes one picture per run and does everything below the
 * picture header: it fetches the bitstream from memory itself, and does
 * variable length decoding, inverse quantisation, the IDCT and motion
 * compensation from the one reference picture it is given. iOS parses the
 * headers in a userspace plug-in and hands the kernel a struct; V4L2 has
 * no stateless interface for these codecs, so the headers (VOL, VOP, the
 * H.263 picture header) are parsed here. One buffer is one picture, with
 * or without its headers in front; a buffer of headers alone is accepted.
 *
 * What it decodes: MPEG-4 Simple Profile (I and P VOPs, rectangular,
 * progressive, both quantisation methods with their matrices, AC
 * prediction, four motion vectors, video packets) and baseline H.263.
 * Not: B VOPs (one reference), quarter sample, data partitioning,
 * interlace, sprites. Those are refused where the headers show them.
 *
 * Things the block insists on:
 *  - the memory path it shares with the H.264 block must be switched to
 *    it (s5l8720-vdec.h), or it fetches nothing;
 *  - it is woken before and put to sleep after every picture;
 *  - the picture data begins where the header ended inside a 32-bit word:
 *    the block is pointed at that word and the bits before it are read
 *    away through the VLD unit's bit reader, a read that never returns if
 *    the stream is not being fetched, so that is checked first;
 *  - after a start that goes nowhere the VLD unit must not be touched at
 *    all (hardware leaves it alone when the top status reads 0x40), so a
 *    time-out only switches the block's clock control off.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "s5l8720-vdec.h"
#include "s5l8720-mpvd.h"

/* ----------------------------------------------------------- hardware */

static void mpvd_hw_ack(struct mpvd_dev *md, bool with_vld)
{
	m_wr(md, MP4, MP4_IRQC, ~0u);
	m_wr(md, IMC, IMC_IRQC, ~0u);
	if (with_vld)
		m_wr(md, VLD, VLD_IRQC, ~0u);
	m_wr(md, MI, MI_IRQC, ~0u);
	m_wr(md, DDM, DDM_IRQC, ~0u);
}

/* asleep the block reads as zeros and nothing in it can stall */
static void mpvd_hw_sleep(struct mpvd_dev *md, bool touch_units, bool with_vld)
{
	m_wr(md, DDM, DDM_IRQ_ALL, ~0u);
	if (touch_units)
		mpvd_hw_ack(md, with_vld);
	else
		m_wr(md, DDM, DDM_IRQC, ~0u);
	m_wr(md, DDM, DDM_CLC, 0);
}

/* Everything for one picture, up to and including the start. */
static int mpvd_hw_picture(struct mpvd_ctx *ctx, const struct mpvd_vop *p,
			   dma_addr_t stream, dma_addr_t ref, dma_addr_t out)
{
	struct mpvd_dev *md = ctx->md;
	const struct mpvd_vol *v = &ctx->vol;
	u32 mbw = ALIGN(v->width, 16) / 16, mbh = ALIGN(v->height, 16) / 16;
	u32 chroma = ctx->dst_fmt.bytesperline * ctx->dst_fmt.height;
	/* the data starts this many bits into the words from 'base' on */
	dma_addr_t base = stream & ~3;
	u32 bits = (stream & 3) * 8 + p->header_bits;
	dma_addr_t start = base + (bits >> 5) * 4;
	u32 n, i;

	m_wr(md, DDM, DDM_CLC, 2);
	if (!(m_rd(md, DDM, DDM_CLC) & 2))
		return -EIO;
	m_wr(md, DDM, DDM_IRQ_ALL, ~0u);
	m_wr(md, DDM, DDM_CMD, 0);
	mpvd_hw_ack(md, true);

	/* per-picture register and table setup */
	m_wr(md, MI, MI_STREAM, base);
	m_wr(md, MI, MI_USER, md->user_dma);
	m_wr(md, MI, MI_USER_END, md->user_dma + MPVD_USER_SIZE);
	m_wr(md, MI, MI_DP, md->dp_dma);
	m_wr(md, MI, MI_DP_SIZE, MPVD_DP_SIZE);
	m_wr(md, MI, 0x80, 0);
	/*
	 * Bit 31 first: before it the MP4 unit ignores writes (without that
	 * ordering its interrupt enable and 0x1c stayed 0 and no IRQ fired).
	 */
	m_wr(md, DDM, DDM_SIZE, BIT(31));
	m_wr(md, MP4, MP4_1C, 1);
	m_wr(md, DDM, DDM_IRQE, 0x142);
	m_wr(md, VLD, VLD_IRQE, 0x14);
	m_wr(md, MP4, MP4_IRQE, 0x3f);
	m_wr(md, MI, MI_IRQE, 0x50);
	for (i = 0; i < 64; i++) {
		m_wr(md, IQ, IQ_INTRA(i), v->intra[i]);
		m_wr(md, IQ, IQ_INTER(i), v->inter[i]);
	}

	m_wr(md, MI, MI_SIZE, mbw << 16 | mbh);
	m_wr(md, DDM, DDM_SIZE, BIT(31) | (mbw - 1) << 16 | (mbh - 1) << 8);
	m_wr(md, MI, MI_10, 0x210);
	m_wr(md, MI, MI_REF_Y, ref);
	m_wr(md, MI, MI_REF_Y + 4, 0);
	m_wr(md, MI, MI_REF_C, ref + chroma);
	m_wr(md, MI, MI_REF_C + 4, 0);
	m_wr(md, MI, MI_OUT_Y, out);
	m_wr(md, MI, MI_OUT_Y + 4, 0);
	m_wr(md, MI, MI_OUT_C, out + chroma);
	m_wr(md, MI, MI_OUT_C + 4, 0);

	m_wr(md, IQ, IQ_MODE, 0x20 | (v->short_header ? 0x80 : 0) |
	     v->quant_type << 8);
	m_wr(md, IMC, IMC_MODE, 0xc02 | p->rounding << 2);
	n = p->type | p->fcode << 2 | p->intra_dc_thr << 11;
	if (v->short_header) {
		m_wr(md, MP4, MP4_PIC, n | MP4_PIC_SHORT_HEADER);
		m_wr(md, MP4, MP4_QUANT, p->quant | (p->mbs_per_gob - 1) << 6 |
		     (p->num_gobs - 1) << 15);
		m_wr(md, MP4, MP4_34, 0);
		m_wr(md, MP4, MP4_TIME, 0);
	} else {
		/* bits needed for macroblock index in a video packet header */
		u32 mb_bits = fls(mbw * mbh);

		m_wr(md, MP4, MP4_PIC, n);
		m_wr(md, MP4, MP4_QUANT, p->quant);
		m_wr(md, MP4, MP4_34, 3);
		m_wr(md, MP4, MP4_TIME, v->vti_bits | mb_bits << 4 |
		     p->time_inc << 8);
	}
	m_wr(md, MP4, MP4_A0, 0);
	m_wr(md, MP4, MP4_A0 + 4, 0);
	m_wr(md, MP4, MP4_A0 + 8, 0);
	m_wr(md, VLD, VLD_IRQC, VLD_IRQC_DONE);
	m_wr(md, MP4, MP4_IRQC, MP4_IRQC_DONE);
	m_wr(md, MI, MI_IRQC, ~0u);
	m_wr(md, MI, MI_STREAM, start);
	m_wr(md, MI, MI_STREAM_END, base + (stream & 3) + p->len + MPVD_STREAM_SLACK);
	m_wr(md, VLD, VLD_MODE, v->short_header ? 0x102 : 2);
	m_wr(md, MI, MI_CMD, MI_CMD_FETCH);

	/*
	 * The bit reader answers only once the MI unit is fetching, which
	 * shows in its stream position; waiting for bits that never come
	 * would stop the machine.
	 */
	for (i = 0; m_rd(md, MI, MI_STREAM) == start; i++) {
		if (i == 500)
			return -ETIMEDOUT;
		udelay(10);
	}
	/* flush initial bit offset cleanly (also executed for 0 bits) */
	m_rd(md, VLD, VLD_GETBITS(bits & 31));

	m_wr(md, MP4, MP4_CMD, MP4_CMD_START);
	return 0;
}

/* --------------------------------------------------------------- jobs */

static const struct v4l2_event mpvd_eos_event = {
	.type = V4L2_EVENT_EOS,
};

/* Whoever takes the running job, the interrupt or the timeout, ends it. */
static struct mpvd_ctx *mpvd_take_job(struct mpvd_dev *md)
{
	struct mpvd_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&md->job_lock, flags);
	ctx = md->cur;
	md->cur = NULL;
	spin_unlock_irqrestore(&md->job_lock, flags);
	return ctx;
}

/*
 * End a job. dst is the picture produced (NULL: the buffer held none, or
 * one that is refused) and becomes the reference whatever its state, as
 * in iOS.
 */
static void mpvd_job_done(struct mpvd_ctx *ctx, struct vb2_v4l2_buffer *dst,
			  enum vb2_buffer_state state)
{
	struct mpvd_dev *md = ctx->md;
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_src_buf_remove(m2m);
	bool last = src && v4l2_m2m_is_last_draining_src_buf(m2m, src);

	if (dst) {
		if (src)
			v4l2_m2m_buf_copy_metadata(src, dst, true);
		dst->sequence = ctx->sequence++;
		dst->field = V4L2_FIELD_NONE;
		dst->flags &= ~V4L2_BUF_FLAG_LAST;
		vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->dst_fmt.sizeimage);
		ctx->ref = dst;
	} else if (last) {
		/* the stream ends without a picture: an empty last buffer */
		dst = v4l2_m2m_dst_buf_remove(m2m);
		if (dst)
			vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
		else
			m2m->next_buf_last = true;
		state = VB2_BUF_STATE_DONE;
	}
	if (last) {
		if (dst)
			dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_event_queue_fh(&ctx->fh, &mpvd_eos_event);
		v4l2_m2m_mark_stopped(m2m);
	}
	if (src)
		v4l2_m2m_buf_done(src, state);
	if (dst)
		v4l2_m2m_buf_done(dst, state);
	v4l2_m2m_job_finish(md->m2m, m2m);
}

static irqreturn_t mpvd_irq(int irq, void *data)
{
	struct mpvd_dev *md = data;
	struct mpvd_ctx *ctx;
	u32 ddm, mp4, vld = VLD_IRQC_DONE, imc, mi;
	bool ok;

	ddm = m_rd(md, DDM, DDM_IRQC);
	if (!ddm)
		return IRQ_NONE;
	ctx = mpvd_take_job(md);
	if (!ctx) {
		/* nobody is waiting: quieten it without going near VLD */
		mpvd_hw_sleep(md, false, false);
		return IRQ_HANDLED;
	}
	cancel_delayed_work(&md->timeout);

	if (ddm & DDM_IRQC_TIMEOUT)
		m_wr(md, DDM, DDM_IRQC, DDM_IRQC_TIMEOUT);
	mp4 = m_rd(md, MP4, MP4_IRQC);
	if (ddm != DDM_IRQC_VLD_STUCK)
		vld = m_rd(md, VLD, VLD_IRQC);
	imc = m_rd(md, IMC, IMC_IRQC);
	mi = m_rd(md, MI, MI_IRQC);
	/*
	 * The interrupt comes before the last macroblocks are in memory: put
	 * to sleep at once, the block left the last three of every P picture
	 * unwritten. iOS gets here through a thread and never noticed.
	 */
	usleep_range(MPVD_FLUSH_US, 2 * MPVD_FLUSH_US);
	mpvd_hw_sleep(md, true, ddm != DDM_IRQC_VLD_STUCK);
	s5l8720_vdec_release();

	md->last[0] = ddm;
	md->last[1] = mp4;
	md->last[2] = vld;
	md->last[3] = imc;
	md->last[4] = mi;
	/* iOS's test for a good picture */
	ok = mp4 == MP4_IRQC_DONE && !(vld & ~VLD_IRQC_DONE) &&
	     !(imc & ~IMC_IRQC_DONE) && ddm != DDM_IRQC_VLD_STUCK &&
	     !(mi & MI_IRQC_STARVED);
	md->pictures++;
	if (!ok)
		md->errors++;
	mpvd_job_done(ctx, ctx->dst, ok ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);
	return IRQ_HANDLED;
}

static void mpvd_timeout(struct work_struct *work)
{
	struct mpvd_dev *md = container_of(work, struct mpvd_dev, timeout.work);
	struct mpvd_ctx *ctx = mpvd_take_job(md);

	if (!ctx)
		return;
	/* stuck: only the top unit and the clock control are safe */
	md->timeouts++;
	md->last[0] = m_rd(md, DDM, DDM_IRQC);
	dev_err_ratelimited(md->dev, "picture timed out (DDM status %08x)\n",
			    md->last[0]);
	mpvd_hw_sleep(md, false, false);
	s5l8720_vdec_release();
	mpvd_job_done(ctx, ctx->dst, VB2_BUF_STATE_ERROR);
}

/* A destination that is not the reference picture. */
static struct vb2_v4l2_buffer *mpvd_pick_dst(struct mpvd_ctx *ctx)
{
	struct v4l2_m2m_buffer *b;

	v4l2_m2m_for_each_dst_buf(ctx->fh.m2m_ctx, b)
		if (&b->vb != ctx->ref)
			return &b->vb;
	return NULL;
}

static int mpvd_job_ready(void *priv)
{
	struct mpvd_ctx *ctx = priv;

	return v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) && mpvd_pick_dst(ctx);
}

static void mpvd_source_change(struct mpvd_ctx *ctx)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	v4l2_event_queue_fh(&ctx->fh, &ev);
}

static void mpvd_set_dst(struct mpvd_ctx *ctx);

static void mpvd_device_run(void *priv)
{
	struct mpvd_ctx *ctx = priv;
	struct mpvd_dev *md = ctx->md;
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(m2m);
	struct vb2_v4l2_buffer *dst, *ref = ctx->ref;
	struct vb2_queue *cap_q = v4l2_m2m_get_vq(m2m, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 used = vb2_get_plane_payload(&src->vb2_buf, 0);
	struct mpvd_vop vop;
	unsigned long flags;
	int ret;

	/*
	 * The block reads on past the end of a picture, and what it finds
	 * there matters: with the rest of an older, longer picture behind
	 * it, the last macroblocks came out wrong. Zeros, as far as iOS
	 * lets the block read.
	 */
	if (used + MPVD_STREAM_SLACK + 13 > vb2_plane_size(&src->vb2_buf, 0)) {
		md->refused++;
		mpvd_job_done(ctx, NULL, VB2_BUF_STATE_ERROR);
		return;
	}
	memset(data + used, 0, MPVD_STREAM_SLACK + 13);

	ret = mpvd_parse(ctx, data, used, &vop);
	if (ret <= 0) {
		/* headers alone, or nothing this block can decode */
		if (ret)
			md->refused++;
		mpvd_job_done(ctx, NULL, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
		return;
	}

	/* the stream decides the size; the pictures have to be made for it */
	if (ALIGN(ctx->vol.width, 16) != ctx->dst_fmt.width ||
	    ALIGN(ctx->vol.height, 16) > ctx->dst_fmt.height ||
	    ALIGN(ctx->vol.height, 16) + 48 < ctx->dst_fmt.height) {
		ctx->src_fmt.width = ctx->vol.width;
		ctx->src_fmt.height = ctx->vol.height;
		if (!vb2_is_busy(cap_q))
			mpvd_set_dst(ctx);
		mpvd_source_change(ctx);
		md->refused++;
		mpvd_job_done(ctx, NULL, VB2_BUF_STATE_ERROR);
		return;
	}
	/* B VOPs need a second reference the block is never given */
	if (vop.type > 1 || vop.header_bits >= vop.len * 8 ||
	    vop.len > SZ_1M) {
		md->refused++;
		mpvd_job_done(ctx, NULL, VB2_BUF_STATE_ERROR);
		return;
	}

	dst = mpvd_pick_dst(ctx);
	v4l2_m2m_dst_buf_remove_by_buf(m2m, dst);
	if (!vop.coded) {
		/* a VOP that is not coded repeats the picture before it */
		if (ref)
			memcpy(vb2_plane_vaddr(&dst->vb2_buf, 0),
			       vb2_plane_vaddr(&ref->vb2_buf, 0),
			       ctx->dst_fmt.sizeimage);
		mpvd_job_done(ctx, dst, VB2_BUF_STATE_DONE);
		return;
	}

	/* the memory path first: this can wait for the H.264 block */
	s5l8720_vdec_claim(md->rst, S5L8720_VDEC_SEL_MPVD);
	ctx->dst = dst;
	spin_lock_irqsave(&md->job_lock, flags);
	md->cur = ctx;
	spin_unlock_irqrestore(&md->job_lock, flags);
	/* armed before the start: a picture can end before this function does */
	mod_delayed_work(system_wq, &md->timeout, msecs_to_jiffies(MPVD_TIMEOUT_MS));

	ret = mpvd_hw_picture(ctx, &vop,
			      vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) + vop.offset,
			      vb2_dma_contig_plane_dma_addr(&(ref ? ref : dst)->vb2_buf, 0),
			      vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0));
	if (ret && mpvd_take_job(md)) {
		cancel_delayed_work(&md->timeout);
		dev_err_ratelimited(md->dev, "picture not started: %d\n", ret);
		mpvd_hw_sleep(md, false, false);
		s5l8720_vdec_release();
		md->errors++;
		mpvd_job_done(ctx, dst, VB2_BUF_STATE_ERROR);
	}
}

static const struct v4l2_m2m_ops mpvd_m2m_ops = {
	.device_run = mpvd_device_run,
	.job_ready = mpvd_job_ready,
};

/* ------------------------------------------------------------ formats */

static void mpvd_adjust_src(struct v4l2_pix_format *pix)
{
	if (pix->pixelformat != V4L2_PIX_FMT_H263)
		pix->pixelformat = V4L2_PIX_FMT_MPEG4;
	pix->width = clamp(pix->width, MPVD_MIN_SIZE, MPVD_MAX_WIDTH);
	pix->height = clamp(pix->height, MPVD_MIN_SIZE, MPVD_MAX_HEIGHT);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = 0;
	pix->sizeimage = clamp_t(u32, pix->sizeimage, SZ_128K, SZ_1M);
}

/* The pictures follow from the coded size: whole macroblocks, NV12. */
static void mpvd_set_dst(struct mpvd_ctx *ctx)
{
	struct v4l2_pix_format *pix = &ctx->dst_fmt;

	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->width = ALIGN(ctx->src_fmt.width, 16);
	pix->height = ALIGN(ctx->src_fmt.height, 16);
	/*
	 * A chroma plane that does not start on a 1 KiB boundary comes out
	 * wrong (iOS's planes are all on one), so the height grows until the
	 * luma plane is a multiple of that, as for the H.264 block.
	 */
	while ((pix->width * pix->height) & 0x3ff)
		pix->height += 16;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width;
	pix->sizeimage = pix->width * pix->height * 3 / 2;
	pix->colorspace = ctx->src_fmt.colorspace;
	pix->xfer_func = ctx->src_fmt.xfer_func;
	pix->ycbcr_enc = ctx->src_fmt.ycbcr_enc;
	pix->quantization = ctx->src_fmt.quantization;
}

static inline struct mpvd_ctx *file_to_ctx(struct file *file)
{
	return container_of(file->private_data, struct mpvd_ctx, fh);
}

static int mpvd_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "s5l8720-mpvd", sizeof(cap->driver));
	strscpy(cap->card, "S5L8720 MPEG-4/H.263 decoder", sizeof(cap->card));
	return 0;
}

static int mpvd_enum_fmt_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	static const u32 fmts[] = { V4L2_PIX_FMT_MPEG4, V4L2_PIX_FMT_H263 };

	if (f->index >= ARRAY_SIZE(fmts))
		return -EINVAL;
	f->pixelformat = fmts[f->index];
	f->flags = V4L2_FMT_FLAG_DYN_RESOLUTION;
	return 0;
}

static int mpvd_enum_fmt_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int mpvd_g_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix = file_to_ctx(file)->src_fmt;
	return 0;
}

static int mpvd_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mpvd_ctx *ctx = file_to_ctx(file);

	/* a size the stream brought in applies once the old buffers are gone */
	if (!vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE)))
		mpvd_set_dst(ctx);
	f->fmt.pix = ctx->dst_fmt;
	return 0;
}

static int mpvd_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	mpvd_adjust_src(&f->fmt.pix);
	return 0;
}

static int mpvd_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mpvd_ctx *ctx = file_to_ctx(file);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;

	if (vb2_is_busy(v4l2_m2m_get_vq(m2m, V4L2_BUF_TYPE_VIDEO_OUTPUT)))
		return -EBUSY;
	mpvd_adjust_src(&f->fmt.pix);
	ctx->src_fmt = f->fmt.pix;
	ctx->vol.valid = false;
	ctx->announced = false;
	if (!vb2_is_busy(v4l2_m2m_get_vq(m2m, V4L2_BUF_TYPE_VIDEO_CAPTURE)))
		mpvd_set_dst(ctx);
	return 0;
}

static int mpvd_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static int mpvd_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *dc)
{
	struct mpvd_ctx *ctx = file_to_ctx(file);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, dc);
	if (ret < 0)
		return ret;
	if (!vb2_is_streaming(v4l2_m2m_get_vq(m2m, V4L2_BUF_TYPE_VIDEO_OUTPUT)))
		return 0;
	ret = v4l2_m2m_ioctl_decoder_cmd(file, fh, dc);
	if (ret < 0)
		return ret;
	if (dc->cmd == V4L2_DEC_CMD_STOP && v4l2_m2m_has_stopped(m2m))
		v4l2_event_queue_fh(&ctx->fh, &mpvd_eos_event);
	if (dc->cmd == V4L2_DEC_CMD_START && v4l2_m2m_has_stopped(m2m))
		vb2_clear_last_buffer_dequeued(v4l2_m2m_get_vq(m2m,
							       V4L2_BUF_TYPE_VIDEO_CAPTURE));
	return 0;
}

static const struct v4l2_ioctl_ops mpvd_ioctl_ops = {
	.vidioc_querycap = mpvd_querycap,
	.vidioc_enum_fmt_vid_out = mpvd_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap = mpvd_enum_fmt_cap,
	.vidioc_g_fmt_vid_out = mpvd_g_fmt_out,
	.vidioc_g_fmt_vid_cap = mpvd_g_fmt_cap,
	.vidioc_try_fmt_vid_out = mpvd_try_fmt_out,
	.vidioc_try_fmt_vid_cap = mpvd_g_fmt_cap,
	.vidioc_s_fmt_vid_out = mpvd_s_fmt_out,
	.vidioc_s_fmt_vid_cap = mpvd_g_fmt_cap,	/* not a choice */
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = mpvd_decoder_cmd,
	.vidioc_subscribe_event = mpvd_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ------------------------------------------------------------- queues */

static int mpvd_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct mpvd_ctx *ctx = vb2_get_drv_priv(vq);
	u32 size = V4L2_TYPE_IS_OUTPUT(vq->type) ? ctx->src_fmt.sizeimage :
						   ctx->dst_fmt.sizeimage;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int mpvd_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static int mpvd_buf_prepare(struct vb2_buffer *vb)
{
	struct mpvd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return vb2_plane_vaddr(vb, 0) ? 0 : -EINVAL;
	if (vb2_plane_size(vb, 0) < ctx->dst_fmt.sizeimage ||
	    (vb2_dma_contig_plane_dma_addr(vb, 0) & 0x3ff))
		return -EINVAL;
	return 0;
}

/*
 * Stateful decoding as the V4L2 decoder interface has it: the client
 * queues the stream, then waits for SOURCE_CHANGE before it sets up the
 * CAPTURE queue (GStreamer does). A picture is only decoded with CAPTURE
 * buffers there, so the headers are read here as the stream comes in, and
 * the size they give is announced at once. The buffer stays queued: the
 * job parses it again and decodes it.
 */
static void mpvd_peek_header(struct mpvd_ctx *ctx, struct vb2_buffer *vb)
{
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_queue *cap_q = v4l2_m2m_get_vq(m2m, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	u32 used = vb2_get_plane_payload(vb, 0);
	const u8 *data = vb2_plane_vaddr(vb, 0);
	struct mpvd_vop vop;

	if (ctx->announced || vb2_is_streaming(cap_q) || !data || !used)
		return;
	if (mpvd_parse(ctx, data, used, &vop) < 0 || !ctx->vol.valid)
		return;
	if (ALIGN(ctx->vol.width, 16) != ctx->dst_fmt.width ||
	    ALIGN(ctx->vol.height, 16) > ctx->dst_fmt.height ||
	    ALIGN(ctx->vol.height, 16) + 48 < ctx->dst_fmt.height) {
		ctx->src_fmt.width = ctx->vol.width;
		ctx->src_fmt.height = ctx->vol.height;
		if (!vb2_is_busy(cap_q))
			mpvd_set_dst(ctx);
	}
	ctx->announced = true;
	mpvd_source_change(ctx);
}

static void mpvd_buf_queue(struct vb2_buffer *vb)
{
	struct mpvd_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;

	/* the stream has ended: this buffer says so, empty */
	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    vb2_is_streaming(vb->vb2_queue) && v4l2_m2m_dst_buf_is_last(m2m)) {
		vb2_set_plane_payload(vb, 0, 0);
		vbuf->field = V4L2_FIELD_NONE;
		vbuf->sequence = ctx->sequence++;
		v4l2_m2m_last_buffer_done(m2m, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &mpvd_eos_event);
		return;
	}
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		mpvd_peek_header(ctx, vb);
	v4l2_m2m_buf_queue(m2m, vbuf);
}

static void mpvd_return_bufs(struct mpvd_ctx *ctx, struct vb2_queue *vq,
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

/*
 * The gates are open while any queue streams; the block itself is woken
 * for each picture.
 */
static int mpvd_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct mpvd_ctx *ctx = vb2_get_drv_priv(vq);
	struct mpvd_dev *md = ctx->md;
	int ret = 0;

	if (!md->streaming) {
		md->dp = dma_alloc_coherent(md->dev, MPVD_DP_SIZE, &md->dp_dma,
					    GFP_KERNEL);
		if (!md->dp) {
			mpvd_return_bufs(ctx, vq, VB2_BUF_STATE_QUEUED);
			return -ENOMEM;
		}
		ret = clk_bulk_prepare_enable(md->nclks, md->clks);
		if (ret) {
			dma_free_coherent(md->dev, MPVD_DP_SIZE, md->dp,
					  md->dp_dma);
			md->dp = NULL;
			mpvd_return_bufs(ctx, vq, VB2_BUF_STATE_QUEUED);
			return ret;
		}
	}
	md->streaming++;
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		ctx->sequence = 0;
	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, vq);
	return 0;
}

static void mpvd_stop_streaming(struct vb2_queue *vq)
{
	struct mpvd_ctx *ctx = vb2_get_drv_priv(vq);
	struct mpvd_dev *md = ctx->md;

	/* the core has waited for a running job of this context */
	mpvd_return_bufs(ctx, vq, VB2_BUF_STATE_ERROR);
	v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, vq);
	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->announced = false;	/* a new stream is announced again */
	if (V4L2_TYPE_IS_OUTPUT(vq->type) &&
	    v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &mpvd_eos_event);
	/* the pictures may be freed now: no reference across a restart */
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		ctx->ref = NULL;
	if (!--md->streaming) {
		cancel_delayed_work_sync(&md->timeout);
		clk_bulk_disable_unprepare(md->nclks, md->clks);
		if (md->dp) {
			dma_free_coherent(md->dev, MPVD_DP_SIZE, md->dp,
					  md->dp_dma);
			md->dp = NULL;
		}
	}
}

static const struct vb2_ops mpvd_qops = {
	.queue_setup = mpvd_queue_setup,
	.buf_out_validate = mpvd_buf_out_validate,
	.buf_prepare = mpvd_buf_prepare,
	.buf_queue = mpvd_buf_queue,
	.start_streaming = mpvd_start_streaming,
	.stop_streaming = mpvd_stop_streaming,
};

static int mpvd_queue_init(void *priv, struct vb2_queue *src_vq,
			   struct vb2_queue *dst_vq)
{
	struct mpvd_ctx *ctx = priv;
	struct vb2_queue *vq;
	int i, ret;

	for (i = 0; i < 2; i++) {
		vq = i ? dst_vq : src_vq;
		vq->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
		vq->io_modes = VB2_MMAP | VB2_DMABUF;
		vq->drv_priv = ctx;
		vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		vq->ops = &mpvd_qops;
		vq->mem_ops = &vb2_dma_contig_memops;
		vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		vq->lock = &ctx->md->lock;
		vq->dev = ctx->md->dev;
		/* the reference picture and one to decode into */
		if (i)
			vq->min_reqbufs_allocation = 2;
		ret = vb2_queue_init(vq);
		if (ret)
			return ret;
	}
	return 0;
}

/* --------------------------------------------------------------- file */

static int mpvd_open(struct file *file)
{
	struct mpvd_dev *md = video_drvdata(file);
	struct mpvd_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->md = md;
	ctx->src_fmt.width = 320;
	ctx->src_fmt.height = 240;
	mpvd_adjust_src(&ctx->src_fmt);
	mpvd_set_dst(ctx);

	if (mutex_lock_interruptible(&md->lock)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	v4l2_ctrl_handler_init(&ctx->hdl, 1);
	v4l2_ctrl_new_std(&ctx->hdl, NULL, V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
			  2, 2, 1, 2);
	ret = ctx->hdl.error;
	if (ret)
		goto err;
	ctx->fh.ctrl_handler = &ctx->hdl;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(md->m2m, ctx, mpvd_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err;
	}
	v4l2_fh_add(&ctx->fh);
	mutex_unlock(&md->lock);
	return 0;

err:
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&md->lock);
	kfree(ctx);
	return ret;
}

static int mpvd_release(struct file *file)
{
	struct mpvd_ctx *ctx = file_to_ctx(file);
	struct mpvd_dev *md = ctx->md;

	mutex_lock(&md->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&md->lock);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations mpvd_fops = {
	.owner = THIS_MODULE,
	.open = mpvd_open,
	.release = mpvd_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ------------------------------------------------------------- driver */

#ifdef CONFIG_DEBUG_FS
/* debugfs <device>/stats */
static int mpvd_stats_show(struct seq_file *s, void *unused)
{
	struct mpvd_dev *md = s->private;

	seq_printf(s, "pictures %u errors %u timeouts %u refused %u last DDM %x MP4 %x VLD %x IMC %x MI %x\n",
		   md->pictures, md->errors, md->timeouts, md->refused,
		   md->last[0], md->last[1], md->last[2], md->last[3],
		   md->last[4]);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mpvd_stats);

static void mpvd_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static int mpvd_debugfs_init(struct device *dev, struct mpvd_dev *md)
{
	struct dentry *dir = debugfs_create_dir(dev_name(dev), NULL);

	debugfs_create_file("stats", 0444, dir, md, &mpvd_stats_fops);
	return devm_add_action_or_reset(dev, mpvd_debugfs_remove, dir);
}
#else
static inline int mpvd_debugfs_init(struct device *dev, struct mpvd_dev *md)
{
	return 0;
}
#endif

static int mpvd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mpvd_dev *md;
	struct video_device *vdev;
	struct resource *res;
	int irq, ret, i;

	md = devm_kzalloc(dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;
	md->dev = dev;
	mutex_init(&md->lock);
	spin_lock_init(&md->job_lock);
	INIT_DELAYED_WORK(&md->timeout, mpvd_timeout);
	platform_set_drvdata(pdev, md);

	for (i = 0; i < MPVD_UNITS; i++) {
		md->unit[i] = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(md->unit[i]))
			return PTR_ERR(md->unit[i]);
	}
	/* the reset block, for the memory path (s5l8720-vdec.h) */
	res = platform_get_resource(pdev, IORESOURCE_MEM, MPVD_UNITS);
	if (!res)
		return -ENODEV;
	md->rst = devm_ioremap(dev, res->start, resource_size(res));
	if (!md->rst)
		return -ENOMEM;
	/* the gates, parent first: the block must not be touched without them */
	md->nclks = devm_clk_bulk_get_all(dev, &md->clks);
	if (md->nclks < 1)
		return dev_err_probe(dev, md->nclks ? md->nclks : -ENODEV, "clocks\n");
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));
	md->user = dmam_alloc_coherent(dev, MPVD_USER_SIZE, &md->user_dma, GFP_KERNEL);
	if (!md->user)
		return -ENOMEM;

	/* in a thread: the handler sleeps a moment, see there */
	ret = devm_request_threaded_irq(dev, irq, NULL, mpvd_irq, IRQF_ONESHOT,
					dev_name(dev), md);
	if (ret)
		return ret;

	ret = mpvd_debugfs_init(dev, md);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &md->v4l2);
	if (ret)
		return ret;
	md->m2m = v4l2_m2m_init(&mpvd_m2m_ops);
	if (IS_ERR(md->m2m)) {
		ret = PTR_ERR(md->m2m);
		goto err_v4l2;
	}

	vdev = &md->vdev;
	strscpy(vdev->name, "s5l8720-mpvd", sizeof(vdev->name));
	vdev->fops = &mpvd_fops;
	vdev->ioctl_ops = &mpvd_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &md->lock;
	vdev->v4l2_dev = &md->v4l2;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, md);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	dev_info(dev, "S5L8720 MPEG-4/H.263 decoder, %s\n",
		 video_device_node_name(vdev));
	return 0;

err_m2m:
	v4l2_m2m_release(md->m2m);
err_v4l2:
	v4l2_device_unregister(&md->v4l2);
	return ret;
}

static void mpvd_remove(struct platform_device *pdev)
{
	struct mpvd_dev *md = platform_get_drvdata(pdev);

	video_unregister_device(&md->vdev);
	v4l2_m2m_release(md->m2m);
	v4l2_device_unregister(&md->v4l2);
}

static const struct of_device_id mpvd_of_match[] = {
	{ .compatible = "apple,s5l8720-mpvd" },
	{ }
};
MODULE_DEVICE_TABLE(of, mpvd_of_match);

static struct platform_driver mpvd_driver = {
	.probe = mpvd_probe,
	.remove = mpvd_remove,
	.driver = {
		.name = "s5l8720-mpvd",
		.of_match_table = mpvd_of_match,
	},
};
module_platform_driver(mpvd_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8720 MPEG-4 Part 2 and H.263 decoder");
MODULE_LICENSE("GPL");
