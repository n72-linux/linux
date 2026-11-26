// SPDX-License-Identifier: GPL-2.0-only
/* PowerVR MBX Lite on the Apple S5L8720: tile-accelerator frames. */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/dma-resv.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/amba/pl080.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <drm/drm_gem.h>
#include "apple-mbx.h"

static u32 mbx_next_ta_seq(u32 *counter)
{
	if (!(++*counter & ~MBX_SEQ_TA))
		++*counter;
	return MBX_SEQ_TA | (*counter & ~MBX_SEQ_TA);
}

/*
 * How the TA gets its words: by DMA through DMAC1 (a PL080) into the port,
 * 32-bit, bursts of 4, source incrementing, paced by the core's DMA request
 * (destination peripheral 1, memory-to-peripheral flow control: PL080 uses the
 * direction to select flow bits, so DMA controller config writes 0x8841).
 * The CPU does nothing until the TA's interrupt. Unpaced (memory-to-memory)
 * the channel kept writing into a full FIFO when the parameter buffer ran
 * full and the TA stopped taking words; the port held the bus and the SoC
 * froze, whereas paced DMA and polled port feed survive full-buffer stalls.
 * 0x108 bit 2 (the core's DMA request enable) is set for every frame. Short
 * streams, and any stream without DMA, go through the port: in mbx_ta_run()
 * without polling FIFO level (mbx_ta_feed_blind), and in the overlapped
 * pipeline with FIFO level polled (mbx_ta_feed).
 */

static const char *mbx_fence_driver_name(struct dma_fence *f)
{
	return "apple-mbx";
}

static const char *mbx_fence_timeline_name(struct dma_fence *f)
{
	return "ta";
}

static const struct dma_fence_ops mbx_fence_ops = {
	.get_driver_name = mbx_fence_driver_name,
	.get_timeline_name = mbx_fence_timeline_name,
};

/* A queued frame's render ended (err 0) or failed: release its fence. */
static void mbx_fence_done(struct dma_fence **fp, int err)
{
	struct dma_fence *f = xchg(fp, NULL);

	if (!f)
		return;
	if (!dma_fence_is_signaled(f)) {
		if (err)
			dma_fence_set_error(f, err);
		dma_fence_signal(f);
	}
	dma_fence_put(f);
}

/* ----------------------------------------------------------- TA frames */

/* Interleave the low 16 bits of @a (even) and @b (odd) into Morton order. */
static u32 mbx_morton(u32 a, u32 b)
{
	u32 r = 0;
	int i;

	for (i = 0; i < 16; i++)
		r |= ((a >> i) & 1) << (2 * i) | ((b >> i) & 1) << (2 * i + 1);
	return r;
}

/* IEEE single of a small positive integer (exact below 2^24). */
static u32 mbx_f32(u32 n)
{
	int e = fls(n) - 1;

	return (127 + e) << 23 | ((n << (23 - e)) & 0x7fffff);
}

/*
 * Wait for @mask in the status register (all bits), or for one of @bad.
 * Returns -EIO on a bad bit, -ETIMEDOUT otherwise; the last status in *st.
 */
static int mbx_ta_poll(struct apple_mbx *mbx, u32 mask, u32 bad, unsigned int ms, u32 *st)
{
	int ret;

	ret = readl_poll_timeout(mbx->core + MBX_STATUS, *st,
				 (*st & mask) == mask || (*st & bad), 10, ms * 1000);
	if (!ret && (*st & bad))
		return -EIO;
	return ret;
}

/*
 * Feed TA words through the port, at most the FIFO's free space
 * (100 - level) at a time; stop on a parameter-buffer-full or error
 * status. *@pos is the cursor (words already written): a feed that stopped
 * at a full parameter buffer goes on from there once the TA has been
 * resumed (the words already in the FIFO stay there).
 *
 * The TA drains the FIFO in microseconds, so a full FIFO is spun on (1 us
 * steps) for up to 2 ms before falling back to sleeping polls; and the
 * words go out with writesl() (no barrier per word: writel() drains the
 * write buffer each time, ~200 ns a word on the ARM1176).
 */
#define MBX_TA_FEED_CHUNK	32

static int mbx_ta_feed(struct apple_mbx *mbx, const u32 *words, u32 count,
		       unsigned int ms, u32 *st, bool spin, u32 *pos)
{
	u32 i = *pos, n, space, want;
	int ret;

/* (the pipeline's interrupt handler clears events as they come and keeps them in ov_seen; 0 otherwise) */
#define MBX_FEED_BAD(st)	(((st) | READ_ONCE(mbx->ov_seen)) & (MBX_STATUS_PB_FULL | MBX_STATUS_TA_ERR))
	wmb();
	while (i < count) {
		want = min_t(u32, MBX_TA_FEED_CHUNK, count - i);
		/*
		 * The worker of an asynchronous frame sleeps while the FIFO is
		 * full (the TA, the VGP doing T&L, is the slow side): the CPU
		 * builds the next frame meanwhile.
		 */
		ret = spin ? readl_poll_timeout_atomic(mbx->core + MBX_STATUS, *st,
				MBX_TA_FIFO_DEPTH - FIELD_GET(MBX_STATUS_FIFO_LEVEL, *st) >= want ||
				MBX_FEED_BAD(*st), 1, 2000) :
			readl_poll_timeout(mbx->core + MBX_STATUS, *st,
				MBX_TA_FIFO_DEPTH - FIELD_GET(MBX_STATUS_FIFO_LEVEL, *st) >= want ||
				MBX_FEED_BAD(*st), 20, 2000);
		if (ret)
			ret = readl_poll_timeout(mbx->core + MBX_STATUS, *st,
					FIELD_GET(MBX_STATUS_FIFO_LEVEL, *st) < MBX_TA_FIFO_DEPTH ||
					MBX_FEED_BAD(*st),
					10, ms * 1000);
		if (ret)
			return ret;
		if (MBX_FEED_BAD(*st)) {
			*st |= READ_ONCE(mbx->ov_seen);
			return -EIO;
		}
		space = MBX_TA_FIFO_DEPTH - FIELD_GET(MBX_STATUS_FIFO_LEVEL, *st);
		n = min(space, count - i);
		writesl(mbx->core + MBX_TA_PORT, words + i, n);
		i += n;
		*pos = i;
	}
	return 0;
}

/* No level polling: the port holds the bus while the FIFO is full. */
static int mbx_ta_feed_blind(struct apple_mbx *mbx, const u32 *words, u32 count)
{
	u32 i, n;

	wmb();
	for (i = 0; i < count; i += n) {
		n = min_t(u32, 256, count - i);
		writesl(mbx->core + MBX_TA_PORT, words + i, n);
		cond_resched();
	}
	return 0;
}

#define MBX_DMA_CCTL		0x00489000	/* dma-channels word 1: 32-bit, bursts of 4 */
#define MBX_DMA_CCFG		(0x00000040 | 1 << PL080_CONFIG_FLOW_CONTROL_SHIFT)	/* dest peripheral 1, memory to peripheral */
#define MBX_DMA_LLI_WORDS	4092		/* per LLI (the transfer size field is 12 bits) */
#define MBX_DMA_MIN_WORDS	2048		/* shorter streams: port writes are as quick */

/* A stream buffer of at least @count words (lock held). */
static int mbx_ta_buf_fit(struct apple_mbx *mbx, struct mbx_ta_buf *b, u32 count)
{
	size_t size;

	if (b->cap >= count)
		return 0;
	if (b->dma)
		dma_free_coherent(mbx->dev, (size_t)(b->cap + MBX_TA_BUF_SLACK) * 4, b->words, b->dma);
	else
		vfree(b->words);
	memset(b, 0, sizeof(*b));
	size = round_up((size_t)(count + MBX_TA_BUF_SLACK) * 4, 0x10000);
	if (mbx->dmac)
		b->words = dma_alloc_coherent(mbx->dev, size, &b->dma, GFP_KERNEL | __GFP_NOWARN);
	if (!b->words) {
		b->dma = 0;
		b->words = vmalloc(size);
		if (!b->words)
			return -ENOMEM;
	}
	b->cap = size / 4 - MBX_TA_BUF_SLACK;
	return 0;
}

/*
 * DMAC1's clock, on from the first DMA feed until the core is gated
 * (mbx_gate_locked). false: it could not be switched on, and this frame's
 * stream goes through the port instead.
 */
static bool mbx_dmac_clock(struct apple_mbx *mbx)
{
	int ret;

	if (mbx->dmac_clk_on)
		return true;
	ret = clk_enable(mbx->dmac_clk);
	if (ret) {
		dev_warn_ratelimited(mbx->dev, "DMAC1 clock: %d, feeding the TA through its port\n", ret);
		return false;
	}
	mbx->dmac_clk_on = true;
	return true;
}

/*
 * The whole stream by DMA; sleeps until the TA has taken its terminator.
 * 1: the TA is done (its status bit already cleared by the interrupt).
 */
static void __iomem *mbx_ta_dma_setup(struct apple_mbx *mbx, dma_addr_t src_dma,
				      u32 count, unsigned int slot);

static int mbx_ta_feed_dma(struct apple_mbx *mbx, dma_addr_t src_dma, u32 count,
			   unsigned int slot, unsigned int ms, u32 *st)
{
	void __iomem *ch = mbx_ta_dma_setup(mbx, src_dma, count, slot);
	unsigned long flags, left;
	u32 seen;

	/* the TA's interrupt says when it has taken the terminator */
	mbx_arm_wait(mbx, MBX_STATUS_TA_DONE);
	/* (the whole mask: its bit 15 is set in every interrupt the core raises, and
	 * without it none comes) */
	mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);
	writel(MBX_DMA_CCFG | PL080_CONFIG_ENABLE, ch + PL080_CH_CONFIG);
	left = wait_for_completion_timeout(&mbx->done3d, msecs_to_jiffies(ms));
	mbx_write(mbx, MBX_IRQ_ENABLE, 0);
	spin_lock_irqsave(&mbx->irq_lock, flags);
	mbx->wait_mask = 0;
	seen = mbx->seen;
	spin_unlock_irqrestore(&mbx->irq_lock, flags);
	*st = seen | (mbx_read(mbx, MBX_STATUS) & ~MBX_STATUS_EVENTS);
	if (readl(mbx->dmac + PL080_EN_CHAN) & BIT(mbx->dma_channel)) {
		dev_warn_ratelimited(mbx->dev, "TA DMA cut short (source %08x, control %08x, errors %02x, status %08x)\n",
				     readl(ch + PL080_CH_SRC_ADDR), readl(ch + PL080_CH_CONTROL),
				     readl(mbx->dmac + 0x18), *st);
		writel(0, ch + PL080_CH_CONFIG);
		return seen & (MBX_STATUS_PB_FULL | MBX_STATUS_TA_ERR) ? -EIO : -ETIMEDOUT;
	}
	if (seen & (MBX_STATUS_PB_FULL | MBX_STATUS_TA_ERR))
		return -EIO;
	return left ? 1 : -ETIMEDOUT;
}

/* The channel programmed with the stream's LLIs, not started yet. */
static void __iomem *mbx_ta_dma_setup(struct apple_mbx *mbx, dma_addr_t src_dma,
				      u32 count, unsigned int slot)
{
	void __iomem *ch = mbx->dmac + PL080_Cx_BASE(mbx->dma_channel);
	struct mbx_lli *lli = mbx->ta_lli + slot * MBX_TA_LLI_PER_SLOT;
	dma_addr_t lli_dma = mbx->ta_lli_dma +
			     (dma_addr_t)(slot * MBX_TA_LLI_PER_SLOT) * sizeof(*lli);
	u32 i, n, off;

	BUILD_BUG_ON(DIV_ROUND_UP(MBX_TA_MAX_WORDS, MBX_DMA_LLI_WORDS) > MBX_TA_LLI_PER_SLOT);
	n = DIV_ROUND_UP(count, MBX_DMA_LLI_WORDS);
	for (i = 0, off = 0; i < n; i++, off += MBX_DMA_LLI_WORDS) {
		lli[i].src = src_dma + off * 4;
		lli[i].dst = mbx->core_phys + MBX_TA_PORT;
		lli[i].next = i + 1 < n ? lli_dma + (i + 1) * sizeof(*lli) : 0;
		lli[i].cctl = MBX_DMA_CCTL | PL080_CONTROL_SRC_INCR |
			      min_t(u32, MBX_DMA_LLI_WORDS, count - off);
	}
	wmb();
	writel(PL080_CONFIG_ENABLE, mbx->dmac + PL080_CONFIG);
	writel(BIT(mbx->dma_channel), mbx->dmac + PL080_TC_CLEAR);
	writel(BIT(mbx->dma_channel), mbx->dmac + PL080_ERR_CLEAR);
	writel(lli[0].src, ch + PL080_CH_SRC_ADDR);
	writel(lli[0].dst, ch + PL080_CH_DST_ADDR);
	writel(lli[0].next, ch + PL080_CH_LLI);
	writel(lli[0].cctl, ch + PL080_CH_CONTROL);
	return ch;
}

/* Every page of a GPU range must be mapped, or the core stalls on it. */
static bool mbx_gpu_range_mapped(struct apple_mbx *mbx, u32 gpu, u32 size)
{
	unsigned long first = gpu >> PAGE_SHIFT, last = (gpu + size - 1) >> PAGE_SHIFT;

	if (!size || gpu + size - 1 < gpu || last >= MBX_GPU_PAGES)
		return false;
	return find_next_zero_bit(mbx->va_map, last + 1, first) > last;
}

/*
 * Synchronous single TA + render frame. Polls the status register with the
 * core interrupt masked; every wait is bounded, and failure resets the core
 * like a hung blit.
 */
static int mbx_ta_run(struct apple_mbx *mbx, struct mbx_ta_frame *f, const u32 *words,
		      dma_addr_t dma, unsigned int slot, bool stream_wc,
		      u32 ntiles, u32 tx, u32 ty, u32 tail_bytes,
		      u32 clear_color, u32 clear_depth);

/*
 * @uwords: the stream in user space, copied into the buffer of the frame's
 * queue slot (or the one for synchronous frames), unless MBX_TA_GEM_WORDS
 * supplies a validated contiguous GEM object directly (@stream_obj / @stream_bo).
 * @resv: the target's reservation object (render node, MBX_TA_FENCE), or NULL.
 */
int mbx_ta_frame(struct apple_mbx *mbx, struct mbx_ta_frame *f, const u32 __user *uwords,
		 struct dma_resv *resv, struct drm_gem_object *stream_obj,
		 struct mbx_bo *stream_bo)
{
	u32 w = f->width, h = f->height, tx, ty, ntiles, tail_bytes;
	u32 clear_color, clear_depth;
	const u32 *words;
	dma_addr_t dma;
	unsigned int slot;
	bool stream_wc = false;
	int ret;

	BUILD_BUG_ON(sizeof(*f) != 56);
	if (!f->count || f->count > MBX_TA_MAX_WORDS)
		return -EINVAL;
	if (!w || !h || w > MBX_TA_MAX_DIM || h > MBX_TA_MAX_DIM)
		return -EINVAL;
	if ((f->stride & 63) || f->stride < w * 4 || (f->dst_gpu & 0x7f))
		return -EINVAL;
	if (f->flags & ~(MBX_TA_DISCARD | MBX_TA_DEPTH | MBX_TA_DEPTH_LOAD |
			 MBX_TA_DEPTH_NOSTORE | MBX_TA_ASYNC | MBX_TA_FENCE | MBX_TA_GEM_WORDS |
			 MBX_TA_CLEAR_BG))
		return -EINVAL;
	if ((f->flags & MBX_TA_DEPTH) && (f->depth_gpu & 0x7f))
		return -EINVAL;
	clear_color = (f->flags & MBX_TA_CLEAR_BG) ? f->clear_color : 0xffff00ffu;
	clear_depth = (f->flags & MBX_TA_CLEAR_BG) ? f->clear_depth : 0x3f800001u;
	tx = ((w + 15) >> 4) * 2;	/* 8-pixel columns */
	ty = (h + 15) >> 4;		/* 16-pixel rows */
	ntiles = tx * ty;
	tail_bytes = round_up(max(mbx_morton(tx, ty), mbx_morton(ty, tx)) * 4, 32);
	if (ntiles * 8 > MBX_TILE_BUF_SIZE || tail_bytes > MBX_TA_TAIL_SIZE)
		return -EINVAL;

	mutex_lock(&mbx->lock);
	if (f->flags & MBX_TA_ASYNC) {
		/* a free slot (ta_work frees one per frame; it needs the lock) */
		while ((READ_ONCE(mbx->ta_tail) + MBX_TA_QUEUE + 1 - READ_ONCE(mbx->ta_head)) %
		       (MBX_TA_QUEUE + 1) >= MBX_TA_QUEUE) {
			mutex_unlock(&mbx->lock);
			wait_event(mbx->ta_slot_wq,
				   (READ_ONCE(mbx->ta_tail) + MBX_TA_QUEUE + 1 - READ_ONCE(mbx->ta_head)) %
				   (MBX_TA_QUEUE + 1) < MBX_TA_QUEUE);
			mutex_lock(&mbx->lock);
		}
		mbx_wake_locked(mbx);
		mbx->last_use = jiffies;
		mod_delayed_work(system_wq, &mbx->idle_work, msecs_to_jiffies(MBX_IDLE_GATE_MS));
	} else {
		/*
		 * A synchronous frame (a compositor's labels, read back at once)
		 * runs ahead of queued ones: it holds mbx->lock throughout, and
		 * queued frames have not touched the core's buffers yet.
		 */
		mbx_begin_light(mbx);
	}
	slot = (f->flags & MBX_TA_ASYNC) ? mbx->ta_tail : MBX_TA_QUEUE + 1;
	if (f->flags & MBX_TA_GEM_WORDS) {
		words = page_address(stream_bo->pages[0]);
		dma = stream_bo->dma[0];
		stream_wc = !stream_bo->cached;
	} else {
		/* the stream, into this frame's buffer (a queue slot is ours until ta_tail moves) */
		struct mbx_ta_buf *buf = &mbx->ta_buf[slot];

		ret = mbx_ta_buf_fit(mbx, buf, f->count);
		if (!ret && copy_from_user(buf->words, uwords, (size_t)f->count * 4))
			ret = -EFAULT;
		if (!ret && buf->words[f->count - 1] != 0xf0000000)
			ret = -EINVAL;
		if (ret) {
			mutex_unlock(&mbx->lock);
			return ret;
		}
		words = buf->words;
		dma = buf->dma;
	}
	if (!mbx->vgp_tried) {
		mbx->vgp_tried = true;
		if (!firmware_request_nowarn(&mbx->vgp_fw, "apple-mbx/vgp.bin", mbx->dev)) {
			if (!mbx->vgp_fw->size || mbx->vgp_fw->size % 4 ||
			    mbx->vgp_fw->size > 4 * MBX_TA_MAX_WORDS) {
				dev_warn(mbx->dev, "apple-mbx/vgp.bin: bad size %zu, ignored\n",
					 mbx->vgp_fw->size);
				release_firmware(mbx->vgp_fw);
				mbx->vgp_fw = NULL;
			} else {
				dev_dbg(mbx->dev, "VGP program: %zu words\n",
					mbx->vgp_fw->size / 4);
			}
		}
	}
	if (!mbx_gpu_range_mapped(mbx, f->dst_gpu, f->stride * h) ||
	    ((f->flags & MBX_TA_DEPTH) && !mbx_gpu_range_mapped(mbx, f->depth_gpu, ntiles * 512))) {
		mutex_unlock(&mbx->lock);
		return -EFAULT;
	}
	if (f->flags & MBX_TA_ASYNC) {
		/* queued: ta_work feeds it, the render interrupt ends it */
		struct mbx_ta_job *j;
		unsigned long fl;

		f->stage = MBX_TA_STAGE_DONE;
		f->status = 0;
		f->ta_us = 0;
		f->seq = mbx_next_ta_seq(&mbx->ta_seq_sub);
		j = &mbx->ta_q[mbx->ta_tail];
		*j = (struct mbx_ta_job){ *f, words, dma, slot, stream_obj, stream_wc,
					  ntiles, tx, ty, tail_bytes, clear_color, clear_depth,
					  NULL, f->seq };
		if (resv && (f->flags & MBX_TA_FENCE)) {
			struct dma_fence *fence = kzalloc(sizeof(*fence), GFP_KERNEL);

			if (fence) {
				dma_fence_init(fence, &mbx_fence_ops, &mbx->fence_lock,
					       mbx->fence_ctx, ++mbx->fence_seqno);
				dma_resv_lock(resv, NULL);
				if (!dma_resv_reserve_fences(resv, 1)) {
					dma_resv_add_fence(resv, fence, DMA_RESV_USAGE_WRITE);
					f->status = MBX_TA_STATUS_FENCED;
				}
				dma_resv_unlock(resv);
				/* our reference: signalled and put by ta_work */
				j->fence = fence;
			}
		}
		spin_lock_irqsave(&mbx->ta_qlock, fl);
		if (mbx->ta_head == mbx->ta_tail)
			reinit_completion(&mbx->ta_done);
		mbx->ta_tail = (mbx->ta_tail + 1) % (MBX_TA_QUEUE + 1);
		spin_unlock_irqrestore(&mbx->ta_qlock, fl);
		queue_work(system_highpri_wq, &mbx->ta_work);
		wake_up(&mbx->ov_wq);		/* (the pipeline, waiting for a render: a frame to tile meanwhile) */
		mutex_unlock(&mbx->lock);
		return 0;
	}
	ret = mbx_ta_run(mbx, f, words, dma, slot, stream_wc, ntiles, tx, ty, tail_bytes,
			 clear_color, clear_depth);
	mutex_unlock(&mbx->lock);
	return ret;
}

static void mbx_ta_pipeline(struct apple_mbx *mbx);

void mbx_ta_work(struct work_struct *work)
{
	struct apple_mbx *mbx = container_of(work, struct apple_mbx, ta_work);

	mbx_ta_pipeline(mbx);
}

/*
 * Hardware background clear: set the background clear colour words in
 * pb[0x28..0x2a] and point every empty region header at pb[0x35] (pb_gpu + 0xd4)
 * with the empty bit cleared so every tile runs the background object even
 * when the TA binned no geometry into it.
 */
static void mbx_ta_apply_clear_bg(struct apple_mbx *mbx, u32 *rh, u32 ntiles,
				  u32 clear_color, u32 clear_depth, bool write_depth)
{
	u32 *pb = mbx->pb.hdr;
	u32 i;

	pb[0x1b] = write_depth ? 0x22220e00u : 0x22220e80u;
	pb[0x22] = pb[0x23] = pb[0x24] = clear_depth;
	pb[0x28] = pb[0x29] = pb[0x2a] = clear_color;
	for (i = 0; i < ntiles; i++) {
		if (rh[i * 2] & MBX_TA_RH_EMPTY_BIT) {
			rh[i * 2] &= ~MBX_TA_RH_EMPTY_BIT;
			rh[i * 2 + 1] = mbx->pb.gpu + 0xd4;
		}
	}
	wmb();
}

static void mbx_ta_reset_clear_bg(struct apple_mbx *mbx)
{
	u32 *pb = mbx->pb.hdr;

	pb[0x1b] = 0x22220e80u;
	pb[0x22] = pb[0x23] = pb[0x24] = 0x3f800000u;
	pb[0x28] = pb[0x29] = pb[0x2a] = 0xffff00ffu;
	wmb();
}

static void mbx_ta_setup_bg(struct apple_mbx *mbx, const struct mbx_ta_frame *f,
			    u32 ty, bool discard)
{
	u32 *pb = mbx->pb.hdr;
	u32 stride_px = f->stride / 4;
	u32 tx16 = (f->width + 15) >> 4;

	pb[1] = (pb[1] & 0xf00c3ffe) | 0x18000 | 0x8800000 | (stride_px & 8 ? 1 : 0);
	pb[2] = (pb[2] & 0xfe000000) | (stride_px >> 4) << 18 | f->dst_gpu >> 7;
	pb[7] = mbx_f32(tx16 * 32);
	pb[10] = mbx_f32(ty * 32);
	pb[0x15] = pb[7] - (11 << 23);	/* / 2048: texture coordinates */
	pb[0x19] = pb[10] - (11 << 23);
	if (discard) {
		pb[0x1e] = pb[7];
		pb[0x21] = pb[10];
	}
	wmb();
}

static void mbx_ta_program_ta(struct apple_mbx *mbx, const struct mbx_ta_frame *f,
			      u32 idx, u32 tx, u32 ty, u32 ntiles,
			      u32 *tacfg_out, u32 *cfg_out)
{
	u32 *rh = idx ? mbx->tiles : mbx->tiles0;
	u32 cfg, tacfg, i;

	for (i = 0; i < ntiles; i++) {
		rh[i * 2] = MBX_TA_RH_EMPTY;
		rh[i * 2 + 1] = 0;
	}
	wmb();

	/*
	 * TA phase: 0x85c carries the tile geometry (rows - 1 at bit 10,
	 * 16-pixel columns - 1 at bit 20, 0xe1; 0xc0000 with a depth buffer).
	 */
	tacfg = (ty - 1) << 10 | 0xe1 | ((tx - 1) << 19 & 0x7f00000);
	if (f->flags & MBX_TA_DEPTH)
		tacfg |= 0xc0000;
	cfg = mbx_read(mbx, MBX_3D_CONFIG);

	mbx_write(mbx, MBX_PB_BASE, mbx->pb.gpu);
	mbx_write(mbx, 0x840, mbx->tail_gpu);
	mbx_write(mbx, 0x844, idx ? mbx->tiles_gpu : mbx->tiles0_gpu);
	mbx_write(mbx, 0x84c, f->width << 16);
	mbx_write(mbx, 0x850, f->height << 16);
	mbx_write(mbx, 0x854, 0x3727c5ac);	/* 1e-5f */
	mbx_write(mbx, 0x858, 0x3727c5ac);
	mbx_write(mbx, MBX_3D_CONFIG, (cfg & MBX_3D_CONFIG_BLIT) | (tacfg & ~MBX_3D_CONFIG_BLIT));
	mbx_write(mbx, 0x864, mbx->tactrl_gpu);
	mbx_write(mbx, 0x820, mbx->tactrl_gpu + 0x300);
	mbx_write(mbx, 0x810, idx);
	mbx_write(mbx, 0x81c, 1);

	*tacfg_out = tacfg;
	if (cfg_out)
		*cfg_out = cfg;
}

/*
 * Program common ISP/render registers for a tiled frame.
 * Header word 20 (depth buffer config, 0x610): 0x30000 without a depth
 * buffer; with one, (8-pixel columns - 1) << 7, bit 1 set (store), bit 0 =
 * load (one 512-byte block per 8x16 tile).
 */
static void mbx_ta_program_render(struct apple_mbx *mbx, const struct mbx_ta_frame *f,
				  u32 tacfg, u32 idx, u32 tx, u32 ty, u32 r63c,
				  u32 clear_depth, bool discard, bool zload,
				  bool zstore)
{
	mbx_write(mbx, MBX_3D_CONFIG, tacfg & ~MBX_3D_CONFIG_BLIT);
	mbx_write(mbx, 0x650, 6);
	mbx_write(mbx, 0x654, (f->width - 1) << 16);
	mbx_write(mbx, 0x658, (f->height - 1) << 16);
	mbx_write(mbx, 0x65c, f->dst_gpu);
	mbx_write(mbx, 0x660, f->stride / 4);
	mbx_write(mbx, 0x60c, mbx->pb.gpu);
	mbx_write(mbx, 0x6ec, idx);
	mbx_write(mbx, 0x608, idx ? mbx->tiles_gpu : mbx->tiles0_gpu);
	mbx_write(mbx, 0x610, f->flags & MBX_TA_DEPTH ?
		  (tx - 1) << 7 | (zstore ? 2 : 0) | (zload ? 1 : 0) : 0x30000);
	mbx_write(mbx, 0x614, 0x1e3ce508);
	mbx_write(mbx, 0x618, 0x1e3ce508);
	mbx_write(mbx, 0x61c, 0x00020007);
	mbx_write(mbx, 0x63c, r63c);
	mbx_write(mbx, 0x644, 0);
	mbx_write(mbx, 0x648, 1);
	mbx_write(mbx, 0x64c, 0);
	mbx_write(mbx, 0x674, f->flags & MBX_TA_DEPTH ? f->depth_gpu : 0);
	mbx_write(mbx, 0x6d0, 1);
	mbx_write(mbx, 0x6dc, (f->flags & MBX_TA_CLEAR_BG) ? (clear_depth | 1u) : 0x3f800001u);
	mbx_write(mbx, 0x860, (tx - 1) | (ty - 1) << 16);
	mbx_write(mbx, 0x66c, 0x02613098);
	mbx_write(mbx, 0x670, 0x03f79a00);
	mbx_write(mbx, 0x678, 0x00032ecc);
	mbx_write(mbx, 0x6d8, discard ? 0x0100001a : 0x09000000);
}

/*
 * The TA and render phases of a validated frame. Synchronous frames run
 * here under mbx->lock; asynchronous ones in ta_work, which has the core to
 * itself while the job is pending, and return once the render has started.
 */
static int mbx_ta_run(struct apple_mbx *mbx, struct mbx_ta_frame *f, const u32 *words,
		      dma_addr_t dma, unsigned int slot, bool stream_wc,
		      u32 ntiles, u32 tx, u32 ty, u32 tail_bytes,
		      u32 clear_color, u32 clear_depth)
{
	unsigned int ms = f->timeout_ms ?: 500;
	u32 cfg, tacfg, st = 0, *rh = mbx->tiles, *pb = mbx->pb.hdr;
	u32 cfg108;
	bool discard = (f->flags & (MBX_TA_DISCARD | MBX_TA_CLEAR_BG)) != 0;
	ktime_t t0, t1, t2;
	int ret;

	/* Poll instead of the interrupt for this frame; status bits stay put. */
	mbx_write(mbx, MBX_IRQ_ENABLE, 0);
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_EVENTS);
	cfg108 = mbx_read(mbx, MBX_CONFIG);
	/* Enable DMA on 0x108 (bit 2). */
	mbx_write(mbx, MBX_CONFIG, cfg108 | 4);

	/*
	 * Render target setup: parameter-buffer header background object sampling
	 * the target, covering (0,0)-(2W,2H) in 16-pixel units.
	 */
	mbx_ta_setup_bg(mbx, f, ty, discard);

	t0 = ktime_get();
	mbx_ta_program_ta(mbx, f, 1, tx, ty, ntiles, &tacfg, &cfg);
	f->stage = MBX_TA_STAGE_TA_INIT;
	ret = mbx_ta_poll(mbx, MBX_STATUS_SYNC, 0, ms, &st);
	if (ret)
		goto fail;
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_SYNC);
	/* Zero tail and tactrl after the sync, before the TA starts. */
	memset(mbx->tail, 0, tail_bytes);
	memset(mbx->tactrl, 0, MBX_TA_CTRL_SIZE);
	wmb();
	mbx_write(mbx, 0x800, 1);
	f->stage = MBX_TA_STAGE_FEED;
	if (!mbx->vgp_loaded && mbx->vgp_fw) {
		u32 pos = 0;

		ret = mbx_ta_feed(mbx, (const u32 *)mbx->vgp_fw->data,
				  mbx->vgp_fw->size / 4, ms, &st, true, &pos);
		if (ret)
			goto fail;
	}
	if (mbx->dmac && dma && mbx->ta_lli && f->count >= MBX_DMA_MIN_WORDS && mbx_dmac_clock(mbx)) {
		ret = mbx_ta_feed_dma(mbx, dma, f->count, slot, ms, &st);
	} else {
		if (stream_wc)
			dma_sync_single_for_cpu(mbx->dev, dma, (size_t)f->count * 4, DMA_FROM_DEVICE);
		if (f->count < MBX_DMA_MIN_WORDS) {
			/* (too short to fill the parameter buffer: ~92 of its 255 pages at most, measured) */
			ret = mbx_ta_feed_blind(mbx, words, f->count);
		} else {
			/*
			 * A long stream without DMA (a vmalloc'ed buffer, DMAC1's
			 * clock refused): with the FIFO level and the status polled,
			 * since a parameter buffer that runs full would leave blind
			 * writes holding the bus.
			 */
			u32 pos = 0;

			ret = mbx_ta_feed(mbx, words, f->count, ms, &st, true, &pos);
		}
	}
	if (ret < 0)
		goto fail;
	f->stage = MBX_TA_STAGE_TA_DONE;
	if (!ret) {
		ret = readl_poll_timeout(mbx->core + MBX_STATUS, st,
					 ((st & MBX_STATUS_TA_DONE) &&
					  !FIELD_GET(MBX_STATUS_FIFO_LEVEL, st)) ||
					 (st & (MBX_STATUS_PB_FULL | MBX_STATUS_TA_ERR)),
					 10, ms * 1000);
		if (!ret && (st & (MBX_STATUS_PB_FULL | MBX_STATUS_TA_ERR)))
			ret = -EIO;
		if (ret)
			goto fail;
	}
	ret = 0;
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_TA_DONE);
	t1 = ktime_get();

	/* Render phase. */
	mbx_write(mbx, MBX_CONFIG, mbx_read(mbx, MBX_CONFIG) & ~1);
	mbx_write(mbx, 0x818, 1);
	f->stage = MBX_TA_STAGE_RENDER_INIT1;
	ret = mbx_ta_poll(mbx, MBX_STATUS_SYNC, 0, ms, &st);
	if (ret)
		goto fail;
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_SYNC);
	mbx_write(mbx, 0x814, 1);
	f->stage = MBX_TA_STAGE_RENDER_INIT2;
	ret = mbx_ta_poll(mbx, MBX_STATUS_SYNC, 0, ms, &st);
	if (ret)
		goto fail;
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_SYNC);
	if (f->flags & MBX_TA_CLEAR_BG)
		mbx_ta_apply_clear_bg(mbx, mbx->tiles, ntiles, clear_color, clear_depth,
				      (f->flags & MBX_TA_DEPTH) != 0);
	else
		mbx_ta_reset_clear_bg(mbx);
	mbx_ta_program_render(mbx, f, tacfg, 1, tx, ty, 0, clear_depth, discard,
			      (f->flags & MBX_TA_DEPTH_LOAD) != 0,
			      !(f->flags & MBX_TA_DEPTH_NOSTORE));
	mbx_write(mbx, MBX_3D_START, 1);
	f->stage = MBX_TA_STAGE_RENDER;
	ret = mbx_ta_poll(mbx, MBX_STATUS_RENDER_DONE, 0, ms, &st);
	if (ret)
		goto fail;
	t2 = ktime_get();
	f->stage = MBX_TA_STAGE_DONE;
	f->status = st;
	f->ta_us = ktime_us_delta(t1, t0);
	f->render_us = ktime_us_delta(t2, t1);
	mbx->ta_frames++;
	if (mbx->vgp_fw)
		mbx->vgp_loaded = true;

	mbx_write(mbx, MBX_3D_CONFIG, cfg);
	mbx_write(mbx, MBX_CONFIG, cfg108);
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_EVENTS);
	mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);
	return 0;

fail:
	f->status = st;
	mbx->ta_frames++;
	mbx->ta_failures++;
	dev_warn(mbx->dev, "TA frame failed at stage %u: %d (status %08x)\n",
		 f->stage, ret, st);
	rmb();
	dev_dbg(mbx->dev, "  ctrl %08x %08x @+300 %08x, tail %08x %08x %08x %08x, rh[0] %08x %08x rh[%u] %08x %08x, pb[0x37..] %08x %08x\n",
		((u32 *)mbx->tactrl)[0], ((u32 *)mbx->tactrl)[1], ((u32 *)mbx->tactrl)[0x300 / 4],
		mbx->tail[0], mbx->tail[1], mbx->tail[2], mbx->tail[3], rh[0], rh[1],
		ntiles / 2, rh[ntiles], rh[ntiles + 1], pb[0x37], pb[0x38]);
	dev_dbg(mbx->dev, "  regs 800 %08x 804 %08x 80c %08x 810 %08x 81c %08x 830 %08x 834 %08x 108 %08x\n",
		mbx_read(mbx, 0x800), mbx_read(mbx, 0x804), mbx_read(mbx, 0x80c), mbx_read(mbx, 0x810),
		mbx_read(mbx, 0x81c), mbx_read(mbx, 0x830), mbx_read(mbx, 0x834), mbx_read(mbx, MBX_CONFIG));
	mbx_recover(mbx);
	return ret;
}

/* ------------------------------------------------ TA frames, overlapped */

/*
 * Overlapped TA/render pipeline: start the TA phase of the next queued
 * frame as soon as the previous render starts (checking whether a TA is
 * running, not whether a render is):
 *
 *  - two region-header buffers per render target, ping-ponged: TA start
 *    flips an index and writes it to 0x810 with that buffer in 0x844;
 *    render start writes the same index to 0x6ec and the buffer to 0x608;
 *  - parameter-buffer header (the background object) is patched in the
 *    render start after its two syncs, not before the TA;
 *  - tail-pointer buffer and control page are single and cleared at TA
 *    start after the 0x81c sync, while the frame before renders; the
 *    parameter buffer is shared by both frames.
 *
 * A render starts only after the previous one ends. The pipeline holds the
 * core until the queue empties or mbx_begin_light() waits in ta_waiters;
 * meanwhile the interrupt handler records events via mbx->ov.
 *
 * On failure, mbx_recover() resets the core. On parameter-buffer full
 * (status 0x20), the TA pauses, paced DMA pauses with it, and port feed
 * saves its cursor. If a prior render is still active, wait for it (freeing
 * pages) and resume TA (0x800 = 1, 0x804 = 1); if nothing is rendering, run
 * a partial render (mbx_ov_partial) and resume TA. Blind port feed is
 * never used here since a full parameter buffer stalls the bus.
 */

/* @bits cleared in the core and in what the handler recorded. */
static void mbx_ov_ack(struct apple_mbx *mbx, u32 bits)
{
	unsigned long fl;

	spin_lock_irqsave(&mbx->irq_lock, fl);
	mbx_write(mbx, MBX_IRQ_CLEAR, bits);
	mbx->ov_seen &= ~bits;
	spin_unlock_irqrestore(&mbx->irq_lock, fl);
}

/* As mbx_ta_poll, counting the events the handler has cleared. */
static int mbx_ov_poll(struct apple_mbx *mbx, u32 mask, u32 bad, unsigned int ms, u32 *st)
{
	ktime_t end = ktime_add_ms(ktime_get(), ms);
	unsigned int spin = 150;

	for (;;) {
		*st = mbx_read(mbx, MBX_STATUS);
		*st |= READ_ONCE(mbx->ov_seen);
		if (*st & bad)
			return -EIO;
		if ((*st & mask) == mask)
			return 0;
		if (spin) {
			spin--;
			udelay(1);
			continue;
		}
		if (ktime_after(ktime_get(), end))
			return -ETIMEDOUT;
		usleep_range(10, 20);
	}
}

/* The core becomes the pipeline's (it is idle; also after a reset). */
static void mbx_ov_enter(struct apple_mbx *mbx)
{
	unsigned long fl;

	mbx_write(mbx, MBX_IRQ_ENABLE, 0);
	mbx->ta_cfg = mbx_read(mbx, MBX_3D_CONFIG);
	mbx->ta_cfg108 = mbx_read(mbx, MBX_CONFIG);
	mbx_write(mbx, MBX_CONFIG, mbx->ta_cfg108 | 4);
	spin_lock_irqsave(&mbx->irq_lock, fl);
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_EVENTS);
	mbx->wait_mask = 0;
	mbx->ov_seen = 0;
	mbx->ov_rseen = 0;
	mbx->ov_rfence = NULL;
	mbx->ov_rarmed = false;
	mbx->ov_irqs = 0;
	mbx->ov_storm = false;
	mbx->ov = true;
	spin_unlock_irqrestore(&mbx->irq_lock, fl);
	mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);
}

/* ... and everyone's again (no TA, no render running). */
static void mbx_ov_leave(struct apple_mbx *mbx)
{
	unsigned long fl;

	spin_lock_irqsave(&mbx->irq_lock, fl);
	mbx->ov = false;
	mbx->ov_rfence = NULL;
	mbx->ov_rarmed = false;
	mbx->ov_seen = 0;
	spin_unlock_irqrestore(&mbx->irq_lock, fl);
	mbx_write(mbx, MBX_3D_CONFIG, mbx->ta_cfg);
	mbx_write(mbx, MBX_CONFIG, mbx->ta_cfg108);
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_EVENTS);
	mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);
	WRITE_ONCE(mbx->ta_busy, false);
	complete_all(&mbx->ta_jdone);
}

/* The frame's DMA channel off (after a failure; the channel may be waiting on a full FIFO). */
static void mbx_ov_dma_stop(struct apple_mbx *mbx, struct mbx_ta_job *j)
{
	if (j->dma_active && (readl(mbx->dmac + PL080_EN_CHAN) & BIT(mbx->dma_channel)))
		writel(0, mbx->dmac + PL080_Cx_BASE(mbx->dma_channel) + PL080_CH_CONFIG);
	j->dma_active = false;
}

/*
 * The TA of @j from where it is until it has taken the terminator (0), or
 * until the parameter buffer runs full (-ENOSPC: the TA waits for a resume,
 * the DMA channel is left running, the port feed's cursor kept; status
 * 0x10 and 0x20 acked), or another error (the DMA channel stopped).
 */
static int mbx_ov_ta_run(struct apple_mbx *mbx, struct mbx_ta_job *j)
{
	const u32 ev = MBX_STATUS_TA_DONE | MBX_STATUS_PB_FULL | MBX_STATUS_TA_ERR;
	const struct mbx_ta_frame *f = &j->f;
	unsigned int ms = f->timeout_ms ?: 500;
	int ret = 0, stage = MBX_TA_STAGE_FEED;
	u32 st = 0, v;

	if (!j->dma_active) {
		ret = mbx_ta_feed(mbx, j->words, f->count, ms, &st, false, &j->feed_pos);
		if (ret == -EIO && !(st & MBX_STATUS_TA_ERR))
			goto full;
		if (ret)
			goto fail;
	}
	/* the TA's interrupt says when it has taken the terminator, or has stopped */
	stage = MBX_TA_STAGE_TA_DONE;
	if (!wait_event_timeout(mbx->ov_wq, READ_ONCE(mbx->ov_seen) & ev, msecs_to_jiffies(ms)))
		ret = -ETIMEDOUT;
	st = READ_ONCE(mbx->ov_seen) | (mbx_read(mbx, MBX_STATUS) & ~MBX_STATUS_EVENTS);
	if (st & MBX_STATUS_TA_ERR) {
		ret = -EIO;
		goto fail;
	}
	if (st & MBX_STATUS_PB_FULL)
		goto full;
	if (ret)
		goto fail;
	if (j->dma_active && (readl(mbx->dmac + PL080_EN_CHAN) & BIT(mbx->dma_channel))) {
		void __iomem *ch = mbx->dmac + PL080_Cx_BASE(mbx->dma_channel);

		dev_warn_ratelimited(mbx->dev, "TA DMA cut short (source %08x, control %08x, errors %02x, status %08x)\n",
				     readl(ch + PL080_CH_SRC_ADDR), readl(ch + PL080_CH_CONTROL),
				     readl(mbx->dmac + 0x18), st);
		ret = -ETIMEDOUT;
		goto fail;
	}
	/* ("TA done" requires an empty FIFO as well) */
	ret = readl_poll_timeout_atomic(mbx->core + MBX_STATUS, v, !FIELD_GET(MBX_STATUS_FIFO_LEVEL, v), 1, 100);
	if (ret)
		ret = readl_poll_timeout(mbx->core + MBX_STATUS, v, !FIELD_GET(MBX_STATUS_FIFO_LEVEL, v), 10, ms * 1000);
	if (ret)
		goto fail;
	mbx_ov_ack(mbx, MBX_STATUS_TA_DONE);
	j->dma_active = false;
	j->f.ta_us = ktime_us_delta(ktime_get(), j->t_ta);
	return 0;

full:
	/* If both 0x20 and 0x10 fire together, PB_FULL takes precedence; TA_DONE follows after resume. */
	mbx->ov_pb_full++;
	mbx_ov_ack(mbx, MBX_STATUS_PB_FULL | MBX_STATUS_TA_DONE);
	/* Ask for a 1.5x parameter buffer (up to MBX_PB_MAX) for subsequent frames. */
	if (!mbx->pb_want && mbx->pb.size < MBX_PB_MAX)
		mbx->pb_want = min_t(u32, PAGE_ALIGN(mbx->pb.size + mbx->pb.size / 2), MBX_PB_MAX);
	return -ENOSPC;

fail:
	mbx_ov_dma_stop(mbx, j);
	dev_warn(mbx->dev, "pipeline: TA failed at stage %d: %d (status %08x, seen %08x%s)\n", stage, ret,
		 mbx_read(mbx, MBX_STATUS), st | READ_ONCE(mbx->ov_seen), mbx->ov_storm ? ", interrupt storm" : "");
	return ret;
}

/* Resume the TA of @j after a full parameter buffer. */
static int mbx_ov_ta_resume(struct apple_mbx *mbx, struct mbx_ta_job *j)
{
	WRITE_ONCE(mbx->ov_irqs, 0);
	mbx_write(mbx, 0x800, 1);
	mbx_write(mbx, 0x804, 1);
	mbx->ov_resumes++;
	return mbx_ov_ta_run(mbx, j);
}

/*
 * The TA phase of a queued frame, with or without a prior render going on.
 * Returns 0, -ENOSPC when parameter buffer is full (TA left paused and
 * mbx_ov_ta_resume continues), or another error leaving core state as is.
 */
static int mbx_ov_ta(struct apple_mbx *mbx, struct mbx_ta_job *j)
{
	const struct mbx_ta_frame *f = &j->f;
	unsigned int ms = f->timeout_ms ?: 500;
	u32 idx = mbx->ta_idx ^ 1, st = 0;
	int ret, stage = MBX_TA_STAGE_TA_INIT;

	mbx->ta_idx = idx;
	j->idx = idx;
	j->feed_pos = 0;
	j->dma_active = false;
	j->t_ta = ktime_get();
	j->passes = 0;
	j->r63c = 0;
	WRITE_ONCE(mbx->ov_irqs, 0);
	mbx_ta_program_ta(mbx, f, idx, j->tx, j->ty, j->ntiles, &j->tacfg, NULL);
	ret = mbx_ov_poll(mbx, MBX_STATUS_SYNC, 0, ms, &st);
	if (ret)
		goto fail;
	mbx_ov_ack(mbx, MBX_STATUS_SYNC);
	memset(mbx->tail, 0, j->tail_bytes);
	memset(mbx->tactrl, 0, MBX_TA_CTRL_SIZE);
	wmb();
	mbx_write(mbx, 0x800, 1);
	stage = MBX_TA_STAGE_FEED;
	if (!mbx->vgp_loaded && mbx->vgp_fw) {
		u32 pos = 0;

		ret = mbx_ta_feed(mbx, (const u32 *)mbx->vgp_fw->data, mbx->vgp_fw->size / 4, ms, &st, false, &pos);
		if (ret)
			goto fail;
	}
	if (mbx->dmac && j->dma && mbx->ta_lli && f->count >= MBX_DMA_MIN_WORDS && mbx_dmac_clock(mbx)) {
		void __iomem *ch = mbx_ta_dma_setup(mbx, j->dma, f->count, j->slot);

		j->dma_active = true;
		writel(MBX_DMA_CCFG | PL080_CONFIG_ENABLE, ch + PL080_CH_CONFIG);
	} else if (j->stream_wc) {
		dma_sync_single_for_cpu(mbx->dev, j->dma, (size_t)f->count * 4, DMA_FROM_DEVICE);
	}
	return mbx_ov_ta_run(mbx, j);

fail:
	st |= READ_ONCE(mbx->ov_seen);
	dev_warn(mbx->dev, "pipeline: TA failed at stage %d: %d (status %08x, seen %08x%s)\n", stage, ret,
		 mbx_read(mbx, MBX_STATUS), st, mbx->ov_storm ? ", interrupt storm" : "");
	return ret;
}

/*
 * Start rendering a tiled frame; no other render is running. Ends via
 * interrupt (ta_rdone).
 *
 * @partial: flush what the TA binned so far after the parameter buffer filled;
 * the TA resumes afterwards. The first render honours the frame flags, while
 * subsequent passes (j->passes > 0) continue from earlier partial renders:
 *  - background preserves colour target (0x6d8 = 0x09000000), never DISCARD;
 *  - with depth buffer, 0x610 bit 0 loads it (bit 1 stores when needed);
 *  - 0x63c = 0x640 saved after the previous partial render (0 otherwise).
 * Without a depth buffer (0x610 = 0x30000, 0x674 = 0), keep bit 0 clear so
 * hardware never attempts a depth load from GPU address 0 (left unmapped).
 */
static int mbx_ov_render(struct apple_mbx *mbx, struct mbx_ta_job *j, bool partial)
{
	const struct mbx_ta_frame *f = &j->f;
	unsigned int ms = f->timeout_ms ?: 500;
	u32 st = 0, pages;
	bool later = j->passes > 0, discard = (f->flags & (MBX_TA_DISCARD | MBX_TA_CLEAR_BG)) && !later;
	bool clear_bg = (f->flags & MBX_TA_CLEAR_BG) && !later;
	bool zstore = !(f->flags & MBX_TA_DEPTH_NOSTORE) || partial;
	unsigned long fl;
	int ret;

	WRITE_ONCE(mbx->ov_irqs, 0);
	mbx_write(mbx, MBX_CONFIG, mbx_read(mbx, MBX_CONFIG) & ~1);
	mbx_write(mbx, 0x818, 1);		/* "ta store" */
	ret = mbx_ov_poll(mbx, MBX_STATUS_SYNC, 0, ms, &st);
	if (ret)
		goto fail;
	mbx_ov_ack(mbx, MBX_STATUS_SYNC);
	/* control page reports how many parameter-buffer pages the frame took */
	rmb();
	pages = (*(u32 *)mbx->tactrl >> 13) & 0x1fff;
	if (pages > mbx->ov_pb_pages)
		mbx->ov_pb_pages = pages;
	mbx_write(mbx, 0x814, 1);		/* "ta load" */
	ret = mbx_ov_poll(mbx, MBX_STATUS_SYNC, 0, ms, &st);
	if (ret)
		goto fail;
	mbx_ov_ack(mbx, MBX_STATUS_SYNC);
	if (clear_bg)
		mbx_ta_apply_clear_bg(mbx, j->idx ? mbx->tiles : mbx->tiles0, j->ntiles,
				      j->clear_color, j->clear_depth,
				      (f->flags & MBX_TA_DEPTH) != 0);
	else
		mbx_ta_reset_clear_bg(mbx);
	/* background object, sampling the target */
	mbx_ta_setup_bg(mbx, f, j->ty, discard);
	mbx_ta_program_render(mbx, f, j->tacfg, j->idx, j->tx, j->ty,
			      later ? j->r63c : 0, j->clear_depth, discard,
			      (f->flags & MBX_TA_DEPTH_LOAD) || later, zstore);
	reinit_completion(&mbx->ta_rdone);
	spin_lock_irqsave(&mbx->irq_lock, fl);
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_EVENTS);	/* (no TA is running either, or it is paused) */
	mbx->ov_seen = 0;
	mbx->ov_rseen = 0;
	mbx->ov_rfence = partial ? NULL : j->fence;
	mbx->ov_rarmed = true;
	spin_unlock_irqrestore(&mbx->irq_lock, fl);
	j->t_render = ktime_get();
	mbx_write(mbx, MBX_3D_START, 1);
	if (partial)
		return 0;
	mbx->ta_frames++;
	if (mbx->vgp_fw)
		mbx->vgp_loaded = true;
	return 0;

fail:
	dev_warn(mbx->dev, "pipeline: render start failed: %d (status %08x, seen %08x)\n", ret,
		 mbx_read(mbx, MBX_STATUS), st);
	return ret;
}

/* The frame after @prev (the one rendering; NULL: the first) in the queue, if any. */
static struct mbx_ta_job *mbx_ov_next(struct apple_mbx *mbx, const struct mbx_ta_job *prev)
{
	unsigned int n = READ_ONCE(mbx->ta_head);

	if (prev)
		n = (n + 1) % (MBX_TA_QUEUE + 1);
	return n == READ_ONCE(mbx->ta_tail) ? NULL : &mbx->ta_q[n];
}

/* Wait for the end of @j's render (@partial: one of its partial renders, not counted). */
static int mbx_ov_render_wait(struct apple_mbx *mbx, struct mbx_ta_job *j, bool partial)
{
	unsigned long fl;

	if (!wait_for_completion_timeout(&mbx->ta_rdone, msecs_to_jiffies(j->f.timeout_ms ?: 500))) {
		spin_lock_irqsave(&mbx->irq_lock, fl);
		mbx->ov_rarmed = false;
		mbx->ov_rfence = NULL;
		spin_unlock_irqrestore(&mbx->irq_lock, fl);
		mbx->render_timeouts++;
		dev_warn(mbx->dev, "pipeline: render timed out (status %08x, seen %08x%s)\n",
			 mbx_read(mbx, MBX_STATUS), mbx->ov_rseen, mbx->ov_storm ? ", interrupt storm" : "");
		return -ETIMEDOUT;
	}
	if (partial)
		return 0;
	mbx->ta_stat_ta_us += j->f.ta_us;
	mbx->ta_stat_render_us += ktime_us_delta(mbx->ov_rt_done, j->t_render);
	mbx->ta_stat_jobs++;
	return 0;
}

/*
 * Flush what @j's TA binned when the parameter buffer fills with nothing
 * rendering: close what it binned (interrupts off, 0x80c = 0, 0x808 = 1,
 * status 0x10 within ~50 ms, clear, interrupts on), render partially, save
 * 0x640 for the next render's 0x63c, and resume the TA. DMA keeps running
 * (FIFO holds its words), and region headers/tail/control page stay untouched.
 * Returns as mbx_ov_ta_run.
 */
#define MBX_TA_MAX_PASSES	16

static int mbx_ov_partial(struct apple_mbx *mbx, struct mbx_ta_job *j)
{
	u32 st = 0;
	int ret;

	mbx_write(mbx, MBX_IRQ_ENABLE, 0);
	mbx_write(mbx, 0x80c, 0);
	mbx_write(mbx, 0x808, 1);
	/* (also what the handler took: one that was under way when the mask went) */
	ret = mbx_ov_poll(mbx, MBX_STATUS_TA_DONE, MBX_STATUS_TA_ERR, 50, &st);
	mbx_ov_ack(mbx, MBX_STATUS_TA_DONE);
	if (!READ_ONCE(mbx->ov_storm))
		mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);
	if (ret) {
		dev_warn(mbx->dev, "pipeline: the TA did not close its partial scene: %d (status %08x)\n", ret, st);
		return ret;
	}
	ret = mbx_ov_render(mbx, j, true);
	if (!ret)
		ret = mbx_ov_render_wait(mbx, j, true);
	if (ret)
		return ret;
	j->r63c = mbx_read(mbx, 0x640);
	j->passes++;
	mbx->ov_partials++;
	return mbx_ov_ta_resume(mbx, j);
}

/*
 * The frame at the head of the queue is done (nothing of it is on the core
 * any more). When the queue runs empty with it the core is given back
 * first: whoever waits for an empty queue (mbx_begin) goes on at once.
 */
static void mbx_ov_finish(struct apple_mbx *mbx, struct mbx_ta_job *j, int err, bool *core, bool idle)
{
	unsigned long fl;

	if (err)
		mbx->ta_failures++;
	if (*core && idle && !mbx_ov_next(mbx, j)) {
		mbx_ov_leave(mbx);
		*core = false;
	}
	if (j->stream_obj) {
		drm_gem_object_put(j->stream_obj);
		j->stream_obj = NULL;
	}
	mbx_fence_done(&j->fence, err ? -EIO : 0);
	WRITE_ONCE(mbx->ta_seq_done, j->seq);
	spin_lock_irqsave(&mbx->ta_qlock, fl);
	mbx->ta_head = (mbx->ta_head + 1) % (MBX_TA_QUEUE + 1);
	if (mbx->ta_head == mbx->ta_tail)
		complete(&mbx->ta_done);
	spin_unlock_irqrestore(&mbx->ta_qlock, fl);
	wake_up(&mbx->ta_slot_wq);
}

/* After a failure: @j's DMA channel (if any) stopped, the core reset, the pipeline's again. */
static void mbx_ov_reset(struct apple_mbx *mbx, struct mbx_ta_job *j)
{
	if (j)
		mbx_ov_dma_stop(mbx, j);
	mbx_recover(mbx);
	mbx_ov_enter(mbx);
}

static void mbx_ta_pipeline(struct apple_mbx *mbx)
{
	struct mbx_ta_job *prev = NULL;		/* its render is running */
	bool core = false;

	for (;;) {
		struct mbx_ta_job *j = mbx_ov_next(mbx, prev);
		int err, perr;

		if (!j && !prev)
			break;
		if (!j) {
			/* nothing to tile: until the render ends or a frame comes */
			wait_event_timeout(mbx->ov_wq, completion_done(&mbx->ta_rdone) || mbx_ov_next(mbx, prev),
					   msecs_to_jiffies(prev->f.timeout_ms ?: 500));
			if (!completion_done(&mbx->ta_rdone) && mbx_ov_next(mbx, prev))
				continue;
			perr = mbx_ov_render_wait(mbx, prev, false);
			if (perr)
				mbx_ov_reset(mbx, NULL);
			mbx_ov_finish(mbx, prev, perr, &core, true);
			prev = NULL;
			continue;
		}
		if (!core) {
			mutex_lock(&mbx->lock);
			mbx_idle(mbx);
			if (mbx->pb_want)
				mbx_pb_grow(mbx);	/* (nothing is in the parameter buffer) */
			reinit_completion(&mbx->ta_jdone);
			WRITE_ONCE(mbx->ta_busy, true);
			mbx_ov_enter(mbx);
			mutex_unlock(&mbx->lock);
			core = true;
		} else if (atomic_read(&mbx->ta_waiters) || mbx->pb_want) {
			/*
			 * Someone waits for the core, or the parameter buffer needs
			 * to grow (requires lock and empty buffer, no frame active).
			 */
			if (prev) {
				perr = mbx_ov_render_wait(mbx, prev, false);
				if (perr)
					mbx_ov_reset(mbx, NULL);
				mbx_ov_finish(mbx, prev, perr, &core, false);
				prev = NULL;
			}
			mbx_ov_leave(mbx);
			core = false;
			if (atomic_read(&mbx->ta_waiters))
				mbx->ov_yields++;
			continue;
		}

		err = mbx_ov_ta(mbx, j);
		if (prev) {
			perr = mbx_ov_render_wait(mbx, prev, false);
			if (perr) {
				/* the core is stuck: neither frame can be saved */
				mbx_ov_reset(mbx, j);
				mbx_ov_finish(mbx, prev, perr, &core, false);
				mbx_ov_finish(mbx, j, -EIO, &core, true);
				prev = NULL;
				continue;
			}
			mbx_ov_finish(mbx, prev, 0, &core, false);
			prev = NULL;
			/* the render before had the pages: they are free again */
			if (err == -ENOSPC)
				err = mbx_ov_ta_resume(mbx, j);
			if (!err)
				mbx->ov_tas++;
		}
		/* full with nothing rendering: render what is binned, go on */
		while (err == -ENOSPC && j->passes < MBX_TA_MAX_PASSES)
			err = mbx_ov_partial(mbx, j);
		if (!err)
			err = mbx_ov_render(mbx, j, false);
		if (err) {
			if (err == -ENOSPC)
				dev_warn_ratelimited(mbx->dev, "pipeline: the parameter buffer is too small for a frame of %u words, even in %u partial renders\n",
						     j->f.count, j->passes);
			mbx_ov_reset(mbx, j);
			mbx_ov_finish(mbx, j, err, &core, true);
			continue;
		}
		prev = j;
	}
	if (core)
		mbx_ov_leave(mbx);
}
