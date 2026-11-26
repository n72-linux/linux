// SPDX-License-Identifier: GPL-2.0-only
/*
 * H.264 decoder of the Samsung/Apple S5L8720 (iPod touch 2G), iOS's
 * "h264mpd,s5l8720x", as a V4L2 stateless decoder.
 *
 * The block decodes one slice per run. It fetches the NAL unit itself from
 * physical memory, drops the emulation prevention bytes, and does entropy
 * decoding (CAVLC and CABAC), prediction, reconstruction and the in-loop
 * filter; software supplies what the slice header and the reference
 * picture lists say. Its bit reader is also open to software (read n bits,
 * decode an Exp-Golomb code), and there is no register for a bit offset:
 * the slice data starts wherever software stopped reading. So this driver
 * walks every slice header through the hardware, as iOS does, and throws
 * the values away; they arrive in the controls.
 *
 * Every buffer the block uses is named by an index into a table of base
 * addresses, each stored as address >> 10: buffers start on 1 KiB
 * boundaries. A picture is NV12, the luma stride is the width in whole
 * macroblocks, and the two planes are separate table entries. For the
 * single-plane NV12 format the capture height is therefore rounded up
 * until the luma plane is a multiple of 1 KiB, which puts the chroma
 * plane on a boundary too.
 *
 * Decoding is slice based: one request per slice NAL, the capture buffer
 * held across the slices of a picture.
 *
 * Interlaced streams (field pictures and MBAFF) are not supported by
 * this driver and are rejected during parsing.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
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
#include "s5l8720-h264.h"

/* ----------------------------------------------------------- hardware */

int h264_hw_reset(struct h264_dev *hd, u32 bits)
{
	u32 v;

	h_wr(hd, H_RESET, h_rd(hd, H_RESET) | bits);
	return readl_poll_timeout_atomic(hd->base + H_RESET, v, !(v & bits),
					 0, H_POLL_TIMEOUT_US);
}

/* Initialize hardware for a new session after clocks are enabled. */
static int h264_hw_init(struct h264_dev *hd)
{
	int ret;

	ret = h264_hw_reset(hd, H_RESET_ALL);
	if (ret)
		return ret;
	h_wr(hd, H_INT_MASK, 1);
	h_wr(hd, H_STREAM_BASE_IDX, H_BUF_STREAM);
	h_wr(hd, H_STATUS_EN, 0xff);
	h_wr(hd, H_CTRL_D8, 3);
	h_wr(hd, H_CTRL_D8, 1);
	h_wr(hd, H_POSTFILTER, 0);
	hd->last = NULL;
	return 0;
}

/* Acknowledge slice completion and reset internal slice state strobe. */
static u32 h264_hw_finish(struct h264_dev *hd)
{
	u32 status = h_rd(hd, H_STATUS);

	h_wr(hd, H_STATUS, status);
	h_wr(hd, H_INT_ACK, 0);
	h_wr(hd, H_STREAM_STATUS, 0);
	h_wr(hd, H_CTRL_D8, (h_rd(hd, H_CTRL_D8) & ~3u) | 3);
	h_wr(hd, H_CTRL_D8, (h_rd(hd, H_CTRL_D8) & ~3u) | 1);
	return status & H_STATUS_MASK;
}

/* --------------------------------------------------------------- jobs */

/* Whoever takes the running job, the interrupt or the timeout, ends it. */
static struct h264_ctx *h264_take_job(struct h264_dev *hd)
{
	struct h264_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&hd->job_lock, flags);
	ctx = hd->cur;
	hd->cur = NULL;
	spin_unlock_irqrestore(&hd->job_lock, flags);
	return ctx;
}

static void h264_job_done(struct h264_dev *hd, struct h264_ctx *ctx, u32 status)
{
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;

	hd->slices++;
	hd->last_status = status;
	if (status != H_STATUS_DONE) {
		/* iOS: reset the core and go on with the next NAL unit */
		hd->errors++;
		h264_hw_reset(hd, H_RESET_CORE);
		state = VB2_BUF_STATE_ERROR;
	}
	s5l8720_vdec_release();
	v4l2_m2m_buf_done_and_job_finish(hd->m2m, ctx->fh.m2m_ctx, state);
}

static irqreturn_t h264_irq(int irq, void *data)
{
	struct h264_dev *hd = data;
	struct h264_ctx *ctx;
	u32 status;

	if (!(h_rd(hd, H_STATUS) & H_STATUS_MASK))
		return IRQ_NONE;
	status = h264_hw_finish(hd);
	ctx = h264_take_job(hd);
	if (ctx) {
		cancel_delayed_work(&hd->timeout);
		h264_job_done(hd, ctx, status);
	}
	return IRQ_HANDLED;
}

static void h264_timeout(struct work_struct *work)
{
	struct h264_dev *hd = container_of(work, struct h264_dev, timeout.work);
	struct h264_ctx *ctx = h264_take_job(hd);
	u32 status;

	if (!ctx)
		return;
	/* a slice that ended without its interrupt still counts */
	status = h264_hw_finish(hd);
	if (status) {
		hd->late_irqs++;
		dev_warn_ratelimited(hd->dev, "slice ended (status %02x) without an interrupt\n",
				     status);
	} else {
		hd->timeouts++;
		dev_err_ratelimited(hd->dev, "slice timed out at macroblock %08x\n",
				    h_rd(hd, H_MB_POS));
	}
	h264_job_done(hd, ctx, status);
}

static void h264_device_run(void *priv)
{
	struct h264_ctx *ctx = priv;
	struct h264_dev *hd = ctx->hd;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct media_request *req = src->vb2_buf.req_obj.req;
	struct h264_run run;
	unsigned long flags;
	int ret;

	if (req)
		v4l2_ctrl_request_setup(req, &ctx->hdl);
	run.sps = ctx->sps->p_cur.p;
	run.pps = ctx->pps->p_cur.p;
	run.matrix = ctx->matrix->p_cur.p;
	run.slice = ctx->slice->p_cur.p;
	run.dec = ctx->dec->p_cur.p;
	run.weights = ctx->weights->p_cur.p;
	v4l2_m2m_buf_copy_metadata(src, dst, true);

	/* the memory path, which the MPEG-4 block may have; this can wait */
	s5l8720_vdec_claim(hd->axi, S5L8720_VDEC_SEL_H264);
	spin_lock_irqsave(&hd->job_lock, flags);
	hd->cur = ctx;
	spin_unlock_irqrestore(&hd->job_lock, flags);
	/* armed before the start: a slice can end before this function does */
	mod_delayed_work(system_wq, &hd->timeout,
			 msecs_to_jiffies(H_SLICE_TIMEOUT_MS));

	/* the size the buffers were made for is the size the block is told */
	if ((run.sps->pic_width_in_mbs_minus1 + 1) * 16 != ctx->dst_fmt.width ||
	    (run.sps->pic_height_in_map_units_minus1 + 1) * 16 > ctx->dst_fmt.height ||
	    !(run.sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) ||
	    dst->vb2_buf.index >= H_MAX_SLOTS)
		ret = -EINVAL;
	else
		ret = h264_hw_slice(ctx, &run, src, dst);

	if (req)
		v4l2_ctrl_request_complete(req, &ctx->hdl);
	if (ret && h264_take_job(hd)) {
		cancel_delayed_work(&hd->timeout);
		s5l8720_vdec_release();
		hd->errors++;
		v4l2_m2m_buf_done_and_job_finish(hd->m2m, ctx->fh.m2m_ctx,
						 VB2_BUF_STATE_ERROR);
	}
}

static const struct v4l2_m2m_ops h264_m2m_ops = {
	.device_run = h264_device_run,
};

/* ------------------------------------------------------------ formats */

static void h264_adjust_src(struct v4l2_pix_format *pix)
{
	pix->pixelformat = V4L2_PIX_FMT_H264_SLICE;
	pix->width = clamp(pix->width, H_MIN_SIZE, H_MAX_WIDTH);
	pix->height = clamp(pix->height, H_MIN_SIZE, H_MAX_HEIGHT);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = 0;
	pix->sizeimage = clamp_t(u32, pix->sizeimage, SZ_64K, SZ_1M);
}

/* The pictures follow from the coded size: whole macroblocks, see the top. */
static void h264_set_dst(struct h264_ctx *ctx)
{
	struct v4l2_pix_format *pix = &ctx->dst_fmt;

	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->width = ALIGN(ctx->src_fmt.width, 16);
	pix->height = ALIGN(ctx->src_fmt.height, 16);
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

static inline struct h264_ctx *file_to_ctx(struct file *file)
{
	return container_of(file->private_data, struct h264_ctx, fh);
}

static int h264_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "s5l8720-h264", sizeof(cap->driver));
	strscpy(cap->card, "S5L8720 H.264 decoder", sizeof(cap->card));
	return 0;
}

static int h264_enum_fmt_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_H264_SLICE;
	return 0;
}

static int h264_enum_fmt_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV12;
	return 0;
}

static int h264_g_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix = file_to_ctx(file)->src_fmt;
	return 0;
}

static int h264_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix = file_to_ctx(file)->dst_fmt;
	return 0;
}

static int h264_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	h264_adjust_src(&f->fmt.pix);
	return 0;
}

static int h264_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct h264_ctx *ctx = file_to_ctx(file);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;

	/* the capture format follows, so neither queue may have buffers */
	if (vb2_is_busy(v4l2_m2m_get_vq(m2m, V4L2_BUF_TYPE_VIDEO_OUTPUT)) ||
	    vb2_is_busy(v4l2_m2m_get_vq(m2m, V4L2_BUF_TYPE_VIDEO_CAPTURE)))
		return -EBUSY;
	h264_adjust_src(&f->fmt.pix);
	ctx->src_fmt = f->fmt.pix;
	h264_set_dst(ctx);
	return 0;
}

/* The capture format is not a choice. */
static int h264_s_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	return h264_g_fmt_cap(file, priv, f);
}

static const struct v4l2_ioctl_ops h264_ioctl_ops = {
	.vidioc_querycap = h264_querycap,
	.vidioc_enum_fmt_vid_out = h264_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap = h264_enum_fmt_cap,
	.vidioc_g_fmt_vid_out = h264_g_fmt_out,
	.vidioc_g_fmt_vid_cap = h264_g_fmt_cap,
	.vidioc_try_fmt_vid_out = h264_try_fmt_out,
	.vidioc_try_fmt_vid_cap = h264_s_fmt_cap,
	.vidioc_s_fmt_vid_out = h264_s_fmt_out,
	.vidioc_s_fmt_vid_cap = h264_s_fmt_cap,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
	.vidioc_decoder_cmd = v4l2_m2m_ioctl_stateless_decoder_cmd,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ------------------------------------------------------------- queues */

static int h264_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vq);
	u32 size = V4L2_TYPE_IS_OUTPUT(vq->type) ? ctx->src_fmt.sizeimage :
						   ctx->dst_fmt.sizeimage;

	/*
	 * A picture's slot in the block's tables is its buffer's index, for
	 * as long as the buffer lives: the motion vectors kept with a
	 * picture name its references by slot.
	 */
	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		unsigned int have = vb2_get_num_buffers(vq);

		if (have >= H_MAX_SLOTS)
			return -ENOBUFS;
		*nbuffers = min(*nbuffers, H_MAX_SLOTS - have);
	}
	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

/* 128 bytes of motion vectors for each macroblock of a picture */
static int h264_buf_init(struct vb2_buffer *vb)
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct h264_buf *buf = to_h264_buf(vb);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return 0;
	buf->mv_size = ALIGN(ctx->dst_fmt.width * ctx->dst_fmt.height / 2, SZ_1K);
	buf->mv = dma_alloc_coherent(ctx->hd->dev, buf->mv_size, &buf->mv_dma,
				     GFP_KERNEL);
	return buf->mv ? 0 : -ENOMEM;
}

static void h264_buf_cleanup(struct vb2_buffer *vb)
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct h264_buf *buf = to_h264_buf(vb);

	if (buf->mv)
		dma_free_coherent(ctx->hd->dev, buf->mv_size, buf->mv, buf->mv_dma);
	buf->mv = NULL;
}

static int h264_buf_out_validate(struct vb2_buffer *vb)
{
	to_vb2_v4l2_buffer(vb)->field = V4L2_FIELD_NONE;
	return 0;
}

static int h264_buf_prepare(struct vb2_buffer *vb)
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return vb2_plane_size(vb, 0) < ctx->src_fmt.sizeimage ? -EINVAL : 0;
	if (vb2_plane_size(vb, 0) < ctx->dst_fmt.sizeimage ||
	    vb->index >= H_MAX_SLOTS ||
	    (vb2_dma_contig_plane_dma_addr(vb, 0) & 0x3ff))
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, ctx->dst_fmt.sizeimage);
	return 0;
}

static void h264_buf_queue(struct vb2_buffer *vb)
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static void h264_buf_request_complete(struct vb2_buffer *vb)
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->hdl);
}

static void h264_return_bufs(struct h264_ctx *ctx, struct vb2_queue *vq,
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
		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req, &ctx->hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static void h264_ctx_free_neigh(struct h264_ctx *ctx)
{
	struct device *dev = ctx->hd->dev;

	if (ctx->neigh_mv) {
		dma_free_coherent(dev, H_NEIGH_MV_SIZE, ctx->neigh_mv,
				  ctx->neigh_mv_dma);
		ctx->neigh_mv = NULL;
	}
	if (ctx->neigh_mb) {
		dma_free_coherent(dev, H_NEIGH_MB_SIZE, ctx->neigh_mb,
				  ctx->neigh_mb_dma);
		ctx->neigh_mb = NULL;
	}
}

/* The block is clocked and set up while any queue streams. */
static int h264_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vq);
	struct h264_dev *hd = ctx->hd;
	int ret = 0;

	if (!ctx->neigh_mv) {
		ctx->neigh_mv = dma_alloc_coherent(hd->dev, H_NEIGH_MV_SIZE,
						   &ctx->neigh_mv_dma, GFP_KERNEL);
		ctx->neigh_mb = dma_alloc_coherent(hd->dev, H_NEIGH_MB_SIZE,
						   &ctx->neigh_mb_dma, GFP_KERNEL);
		if (!ctx->neigh_mv || !ctx->neigh_mb) {
			ret = -ENOMEM;
			goto err_neigh;
		}
	}
	if (!hd->streaming) {
		ret = clk_bulk_prepare_enable(hd->nclks, hd->clks);
		if (!ret) {
			ret = h264_hw_init(hd);
			if (ret) {
				dev_err(hd->dev, "reset: %d\n", ret);
				clk_bulk_disable_unprepare(hd->nclks, hd->clks);
			}
		}
	}
	if (ret)
		goto err_neigh;
	hd->streaming++;
	return 0;

err_neigh:
	if (!vb2_is_streaming(v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			      V4L2_TYPE_IS_OUTPUT(vq->type) ?
			      V4L2_BUF_TYPE_VIDEO_CAPTURE :
			      V4L2_BUF_TYPE_VIDEO_OUTPUT)))
		h264_ctx_free_neigh(ctx);
	h264_return_bufs(ctx, vq, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void h264_stop_streaming(struct vb2_queue *vq)
{
	struct h264_ctx *ctx = vb2_get_drv_priv(vq);
	struct h264_dev *hd = ctx->hd;

	/* the core has waited for a running job of this context */
	h264_return_bufs(ctx, vq, VB2_BUF_STATE_ERROR);
	if (!vb2_is_streaming(v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
			      V4L2_TYPE_IS_OUTPUT(vq->type) ?
			      V4L2_BUF_TYPE_VIDEO_CAPTURE :
			      V4L2_BUF_TYPE_VIDEO_OUTPUT)))
		h264_ctx_free_neigh(ctx);
	if (!--hd->streaming) {
		cancel_delayed_work_sync(&hd->timeout);
		clk_bulk_disable_unprepare(hd->nclks, hd->clks);
	}
}

static const struct vb2_ops h264_qops = {
	.queue_setup = h264_queue_setup,
	.buf_init = h264_buf_init,
	.buf_cleanup = h264_buf_cleanup,
	.buf_out_validate = h264_buf_out_validate,
	.buf_prepare = h264_buf_prepare,
	.buf_queue = h264_buf_queue,
	.buf_request_complete = h264_buf_request_complete,
	.start_streaming = h264_start_streaming,
	.stop_streaming = h264_stop_streaming,
};

static int h264_queue_init(void *priv, struct vb2_queue *src_vq,
			   struct vb2_queue *dst_vq)
{
	struct h264_ctx *ctx = priv;
	struct vb2_queue *vq;
	int i, ret;

	for (i = 0; i < 2; i++) {
		vq = i ? dst_vq : src_vq;
		vq->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_VIDEO_OUTPUT;
		vq->io_modes = VB2_MMAP | VB2_DMABUF;
		vq->drv_priv = ctx;
		vq->buf_struct_size = sizeof(struct h264_buf);
		vq->ops = &h264_qops;
		vq->mem_ops = &vb2_dma_contig_memops;
		vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		vq->lock = &ctx->hd->lock;
		vq->dev = ctx->hd->dev;
		if (!i) {
			vq->supports_requests = true;
			vq->requires_requests = true;
			vq->subsystem_flags |= VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
		}
		ret = vb2_queue_init(vq);
		if (ret)
			return ret;
	}
	return 0;
}

/* ----------------------------------------------------------- controls */

static int h264_try_ctrl(struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_H264_SPS) {
		const struct v4l2_ctrl_h264_sps *sps = ctrl->p_new.p_h264_sps;

		if (sps->chroma_format_idc != 1 || sps->bit_depth_luma_minus8 ||
		    sps->bit_depth_chroma_minus8)
			return -EINVAL;
		/* field pictures and MBAFF: not tried, see the top */
		if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
			return -EINVAL;
		if ((sps->pic_width_in_mbs_minus1 + 1) * 16 > H_MAX_WIDTH ||
		    (sps->pic_height_in_map_units_minus1 + 1) * 16 > H_MAX_HEIGHT)
			return -EINVAL;
	} else if (ctrl->id == V4L2_CID_STATELESS_H264_PPS) {
		/* slice groups never reach a register in iOS either */
		if (ctrl->p_new.p_h264_pps->num_slice_groups_minus1)
			return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops h264_ctrl_ops = {
	.try_ctrl = h264_try_ctrl,
};

static const struct v4l2_ctrl_config h264_ctrls[] = {
	{ .id = V4L2_CID_STATELESS_H264_SPS, .ops = &h264_ctrl_ops },
	{ .id = V4L2_CID_STATELESS_H264_PPS, .ops = &h264_ctrl_ops },
	{ .id = V4L2_CID_STATELESS_H264_SCALING_MATRIX },
	{ .id = V4L2_CID_STATELESS_H264_SLICE_PARAMS },
	{ .id = V4L2_CID_STATELESS_H264_DECODE_PARAMS },
	{ .id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS },
	{
		.id = V4L2_CID_STATELESS_H264_START_CODE,
		.max = V4L2_STATELESS_H264_START_CODE_ANNEX_B,
		.def = V4L2_STATELESS_H264_START_CODE_NONE,
	},
	{
		.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.max = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		.def = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
	},
};

static int h264_init_ctrls(struct h264_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->hdl;
	struct v4l2_ctrl *c[ARRAY_SIZE(h264_ctrls)];
	unsigned int i;

	v4l2_ctrl_handler_init(hdl, ARRAY_SIZE(h264_ctrls) + 2);
	for (i = 0; i < ARRAY_SIZE(h264_ctrls); i++)
		c[i] = v4l2_ctrl_new_custom(hdl, &h264_ctrls[i], NULL);
	/* Supported profiles and levels verified on hardware */
	v4l2_ctrl_new_std_menu(hdl, NULL, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			       BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED),
			       V4L2_MPEG_VIDEO_H264_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(hdl, NULL, V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			       V4L2_MPEG_VIDEO_H264_LEVEL_3_1, 0,
			       V4L2_MPEG_VIDEO_H264_LEVEL_3_0);
	if (hdl->error) {
		int ret = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	ctx->sps = c[0];
	ctx->pps = c[1];
	ctx->matrix = c[2];
	ctx->slice = c[3];
	ctx->dec = c[4];
	ctx->weights = c[5];
	ctx->start_code = c[6];
	ctx->fh.ctrl_handler = hdl;
	return v4l2_ctrl_handler_setup(hdl);
}

/* --------------------------------------------------------------- file */

static int h264_open(struct file *file)
{
	struct h264_dev *hd = video_drvdata(file);
	struct h264_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->hd = hd;
	ctx->src_fmt.width = 320;
	ctx->src_fmt.height = 240;
	h264_adjust_src(&ctx->src_fmt);
	h264_set_dst(ctx);

	if (mutex_lock_interruptible(&hd->lock)) {
		kfree(ctx);
		return -ERESTARTSYS;
	}
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ret = h264_init_ctrls(ctx);
	if (ret)
		goto err_fh;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(hd->m2m, ctx, h264_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_ctrl_handler_free(&ctx->hdl);
		goto err_fh;
	}
	v4l2_fh_add(&ctx->fh);
	mutex_unlock(&hd->lock);
	return 0;

err_fh:
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&hd->lock);
	kfree(ctx);
	return ret;
}

static int h264_release(struct file *file)
{
	struct h264_ctx *ctx = file_to_ctx(file);
	struct h264_dev *hd = ctx->hd;

	mutex_lock(&hd->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_exit(&ctx->fh);
	if (hd->last == ctx)
		hd->last = NULL;
	mutex_unlock(&hd->lock);
	h264_ctx_free_neigh(ctx);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations h264_fops = {
	.owner = THIS_MODULE,
	.open = h264_open,
	.release = h264_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ------------------------------------------------------------- driver */

static int h264_request_validate(struct media_request *req)
{
	if (vb2_request_buffer_cnt(req) != 1)
		return -EINVAL;
	return vb2_request_validate(req);
}

static const struct media_device_ops h264_media_ops = {
	.req_validate = h264_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

#ifdef CONFIG_DEBUG_FS
/* debugfs <device>/stats */
static int h264_stats_show(struct seq_file *s, void *unused)
{
	struct h264_dev *hd = s->private;

	seq_printf(s, "slices %u errors %u timeouts %u without-irq %u last-status %02x\n",
		   hd->slices, hd->errors, hd->timeouts, hd->late_irqs,
		   hd->last_status);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(h264_stats);

static void h264_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static int h264_debugfs_init(struct device *dev, struct h264_dev *hd)
{
	struct dentry *dir = debugfs_create_dir(dev_name(dev), NULL);

	debugfs_create_file("stats", 0444, dir, hd, &h264_stats_fops);
	return devm_add_action_or_reset(dev, h264_debugfs_remove, dir);
}
#else
static inline int h264_debugfs_init(struct device *dev, struct h264_dev *hd)
{
	return 0;
}
#endif

static int h264_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct h264_dev *hd;
	struct video_device *vdev;
	struct resource *res;
	u32 version;
	int irq, ret;

	hd = devm_kzalloc(dev, sizeof(*hd), GFP_KERNEL);
	if (!hd)
		return -ENOMEM;
	hd->dev = dev;
	mutex_init(&hd->lock);
	spin_lock_init(&hd->job_lock);
	INIT_DELAYED_WORK(&hd->timeout, h264_timeout);
	platform_set_drvdata(pdev, hd);

	hd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hd->base))
		return PTR_ERR(hd->base);
	/* the reset block, for the memory path (s5l8720-vdec.h) */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		hd->axi = devm_ioremap(dev, res->start, resource_size(res));
		if (!hd->axi)
			return -ENOMEM;
	}
	/* the gates, parent first: the block must not be touched without them */
	hd->nclks = devm_clk_bulk_get_all(dev, &hd->clks);
	if (hd->nclks < 1)
		return dev_err_probe(dev, hd->nclks ? hd->nclks : -ENODEV, "clocks\n");
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));

	ret = clk_bulk_prepare_enable(hd->nclks, hd->clks);
	if (ret)
		return ret;
	version = h_rd(hd, H_VERSION);
	clk_bulk_disable_unprepare(hd->nclks, hd->clks);

	/* Threaded: h264_job_done() triggers v4l2_m2m_try_run() which may sleep */
	ret = devm_request_threaded_irq(dev, irq, NULL, h264_irq, IRQF_ONESHOT,
					dev_name(dev), hd);
	if (ret)
		return ret;

	ret = h264_debugfs_init(dev, hd);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &hd->v4l2);
	if (ret)
		return ret;
	hd->m2m = v4l2_m2m_init(&h264_m2m_ops);
	if (IS_ERR(hd->m2m)) {
		ret = PTR_ERR(hd->m2m);
		goto err_v4l2;
	}

	hd->mdev.dev = dev;
	strscpy(hd->mdev.model, "s5l8720-h264", sizeof(hd->mdev.model));
	strscpy(hd->mdev.bus_info, "platform:s5l8720-h264", sizeof(hd->mdev.bus_info));
	media_device_init(&hd->mdev);
	hd->mdev.ops = &h264_media_ops;
	hd->v4l2.mdev = &hd->mdev;

	vdev = &hd->vdev;
	strscpy(vdev->name, "s5l8720-h264", sizeof(vdev->name));
	vdev->fops = &h264_fops;
	vdev->ioctl_ops = &h264_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &hd->lock;
	vdev->v4l2_dev = &hd->v4l2;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, hd);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_media;
	ret = v4l2_m2m_register_media_controller(hd->m2m, vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret)
		goto err_video;
	ret = media_device_register(&hd->mdev);
	if (ret)
		goto err_mc;

	dev_info(dev, "S5L8720 H.264 decoder, version %#x, %s\n", version,
		 video_device_node_name(vdev));
	return 0;

err_mc:
	v4l2_m2m_unregister_media_controller(hd->m2m);
err_video:
	video_unregister_device(vdev);
err_media:
	media_device_cleanup(&hd->mdev);
	v4l2_m2m_release(hd->m2m);
err_v4l2:
	v4l2_device_unregister(&hd->v4l2);
	return ret;
}

static void h264_remove(struct platform_device *pdev)
{
	struct h264_dev *hd = platform_get_drvdata(pdev);

	media_device_unregister(&hd->mdev);
	v4l2_m2m_unregister_media_controller(hd->m2m);
	video_unregister_device(&hd->vdev);
	media_device_cleanup(&hd->mdev);
	v4l2_m2m_release(hd->m2m);
	v4l2_device_unregister(&hd->v4l2);
}

/* After a sleep nothing in the block is trusted: iOS resets and reloads. */
static int h264_resume(struct device *dev)
{
	struct h264_dev *hd = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&hd->lock);
	if (hd->streaming)
		ret = h264_hw_init(hd);
	mutex_unlock(&hd->lock);
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(h264_pm_ops, NULL, h264_resume);

static const struct of_device_id h264_of_match[] = {
	{ .compatible = "apple,s5l8720-h264" },
	{ }
};
MODULE_DEVICE_TABLE(of, h264_of_match);

static struct platform_driver h264_driver = {
	.probe = h264_probe,
	.remove = h264_remove,
	.driver = {
		.name = "s5l8720-h264",
		.of_match_table = h264_of_match,
		.pm = pm_sleep_ptr(&h264_pm_ops),
	},
};
module_platform_driver(h264_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8720 H.264 decoder");
MODULE_LICENSE("GPL");
