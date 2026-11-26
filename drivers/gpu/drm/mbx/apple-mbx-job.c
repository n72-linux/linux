// SPDX-License-Identifier: GPL-2.0-only
/* PowerVR MBX Lite on the Apple S5L8720: 2D and 3D-blit jobs, waiting for jobs. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include "apple-mbx.h"

/* ------------------------------------------------------------ 3D core */

/*
 * Initial blit template: object list and objects with float vertices.
 * Non-zero words below 0x800; a grey ramp follows at 0x800.
 */
static const struct { u16 off; u32 val; } mbx_tmpl_init[] = {
	{ 0x000, 0xe0000000 }, { 0x004, 0xa7700000 }, { 0x008, 0x0e000000 }, { 0x00c, 0xd6887610 },
	{ 0x010, 0x22220e80 }, { 0x01c, 0x45000000 }, { 0x028, 0x45000000 }, { 0x02c, 0x3f800000 },
	{ 0x030, 0x3f800000 }, { 0x034, 0x3f800000 }, { 0x038, 0x3f800000 }, { 0x03c, 0x3f800000 },
	{ 0x040, 0x3f800000 }, { 0x054, 0x40000000 }, { 0x064, 0x40000000 }, { 0x080, 0x22206f80 },
	{ 0x088, 0x45800000 }, { 0x094, 0x45800000 }, { 0x098, 0x45800000 }, { 0x09c, 0x45800000 },
	{ 0x0b4, 0x22207f80 }, { 0x0e8, 0xe0000000 }, { 0x0f4, 0x86884610 }, { 0x0f8, 0x22220e80 },
	{ 0x12c, 0x3f800000 }, { 0x130, 0x3f800000 }, { 0x134, 0x3f800000 }, { 0x138, 0x3f800000 },
	{ 0x198, 0xe0000000 }, { 0x19c, 0x22200e80 }, { 0x1d0, 0x3f800000 }, { 0x1d4, 0x3f800000 },
	{ 0x1d8, 0x3f800000 }, { 0x1dc, 0x3f800000 }, { 0x1f0, 0xe0000000 }, { 0x1fc, 0xd6887610 },
	{ 0x208, 0xab504a90 }, { 0x20c, 0x22250e80 }, { 0x240, 0x3f800000 }, { 0x244, 0x3f800000 },
	{ 0x248, 0x3f800000 }, { 0x24c, 0x3f800000 }, { 0x2a0, 0xe0000000 }, { 0x2ac, 0xa1104020 },
	{ 0x2b0, 0x22220e80 }, { 0x2e4, 0x3f800000 }, { 0x2e8, 0x3f800000 }, { 0x2ec, 0x3f800000 },
	{ 0x2f0, 0x3f800000 }, { 0x324, 0xe0000000 }, { 0x330, 0x86084610 }, { 0x334, 0x22620ea0 },
	{ 0x368, 0x3f800000 }, { 0x36c, 0x3f800000 }, { 0x370, 0x3f800000 }, { 0x374, 0x3f800000 },
	{ 0x378, 0xffffffff }, { 0x384, 0xffffffff }, { 0x390, 0xffffffff }, { 0x39c, 0xffffffff },
	{ 0x3a8, 0xe0000000 }, { 0x3b4, 0x86084610 }, { 0x3b8, 0x46622ea0 }, { 0x3ec, 0x3f800000 },
	{ 0x3f0, 0x3f800000 }, { 0x3f4, 0x3f800000 }, { 0x3f8, 0x3f800000 }, { 0x3fc, 0xffffffff },
	{ 0x408, 0xffffffff }, { 0x414, 0xffffffff }, { 0x420, 0xffffffff }, { 0x42c, 0xe0000000 },
	{ 0x438, 0x86084610 }, { 0x43c, 0x66622ea0 }, { 0x470, 0x3f800000 }, { 0x474, 0x3f800000 },
	{ 0x478, 0x3f800000 }, { 0x47c, 0x3f800000 }, { 0x480, 0xffffffff }, { 0x48c, 0xffffffff },
	{ 0x498, 0xffffffff }, { 0x4a4, 0xffffffff }, { 0x4b0, 0xe0000000 }, { 0x4bc, 0x86084610 },
	{ 0x4c0, 0x82622ea0 }, { 0x4f4, 0x3f800000 }, { 0x4f8, 0x3f800000 }, { 0x4fc, 0x3f800000 },
	{ 0x500, 0x3f800000 }, { 0x504, 0xffffffff }, { 0x510, 0xffffffff }, { 0x51c, 0xffffffff },
	{ 0x528, 0xffffffff },
};

#define MBX_OBJ_BG	0x60200020	/* object list entries (word offsets) */
#define MBX_OBJ_BG2	0x6020002d
#define MBX_OBJ_FILL	0x60a00066
#define MBX_OBJ_BLEND	0x612000a8
#define MBX_OBJ_COPY	0x6120003a
#define MBX_OBJ_COPY_BLEND 0x61a0007c
#define MBX_OBJ_END	0xf0000000

void mbx_tmpl_reset(struct apple_mbx *mbx)
{
	u32 *t = mbx->tmpl;
	int i;

	memset(t, 0, PAGE_SIZE);
	for (i = 0; i < ARRAY_SIZE(mbx_tmpl_init); i++)
		t[mbx_tmpl_init[i].off / 4] = mbx_tmpl_init[i].val;
	for (i = 0; i < 256; i++)
		t[0x800 / 4 + i] = 0xff000000 | (i * 0x10101);
}

/* Types 8/9: textured copies. */
static void mbx_patch_copy(struct apple_mbx *mbx, const struct mbx_blit3d *b)
{
	static const u16 type8[12][2] = {
		{ 0x13c, 32 }, { 0x140, 23 }, { 0x144, 24 }, { 0x148, 32 }, { 0x14c, 25 },
		{ 0x150, 26 }, { 0x154, 32 }, { 0x158, 27 }, { 0x15c, 28 }, { 0x160, 32 },
		{ 0x164, 29 }, { 0x168, 30 },
	};
	static const u16 type9[20][2] = {
		{ 0x250, 32 }, { 0x264, 32 }, { 0x278, 32 }, { 0x28c, 32 },
		{ 0x254, 23 }, { 0x258, 24 }, { 0x268, 25 }, { 0x26c, 26 }, { 0x27c, 27 },
		{ 0x280, 28 }, { 0x290, 29 }, { 0x294, 30 }, { 0x25c, 38 }, { 0x260, 39 },
		{ 0x270, 40 }, { 0x274, 41 }, { 0x284, 42 }, { 0x288, 43 }, { 0x298, 44 },
		{ 0x29c, 45 },
	};
	const u32 *p = b->words;
	u32 *t = mbx->tmpl;
	u32 tex, texa;
	int i;

	tex = p[13] | 0xa0000000 | p[11] << 24 | p[12] << 20 | (p[14] & 0xf) >> 3;
	texa = p[31] | 0xe000000 | ((b->src_gpu >> 7) & 0x3ffff) | (p[14] & 0x7f0) << 14;
	if (b->type == 8) {
		t[0xec / 4] = tex;
		t[0xf0 / 4] = texa;
		t[0xf4 / 4] = (t[0xf4 / 4] & 0xfffe7ffc) | (p[33] & 0x18003);
		for (i = 0; i < 8; i++)
			t[0xfc / 4 + i] = p[15 + i];
		for (i = 0; i < 4; i++)
			t[0x12c / 4 + i] = p[34 + i];
		for (i = 0; i < ARRAY_SIZE(type8); i++)
			t[type8[i][0] / 4] = p[type8[i][1]];
		t[0x70 / 4] = MBX_OBJ_COPY;
	} else {
		t[0x1f4 / 4] = tex;
		t[0x1f8 / 4] = texa;
		t[0x1fc / 4] = p[46];
		t[0x200 / 4] = p[4] | 0xa7700000 | (p[5] & 0xf) >> 3;
		t[0x204 / 4] = ((b->dst_gpu >> 7) & 0x3ffff) | 0xe000000 | (p[5] & 0x7f0) << 14;
		t[0x208 / 4] = p[47];
		for (i = 0; i < 8; i++)
			t[0x210 / 4 + i] = p[15 + i];
		for (i = 0; i < 4; i++)
			t[0x240 / 4 + i] = p[34 + i];
		for (i = 0; i < ARRAY_SIZE(type9); i++)
			t[type9[i][0] / 4] = p[type9[i][1]];
		t[0x70 / 4] = MBX_OBJ_COPY_BLEND;
	}
}

/* Type 6-9 3D blits. */
int mbx_blit3d(struct apple_mbx *mbx, const struct mbx_blit3d *b, bool async)
{
	const u32 *p = b->words;
	u32 *t = mbx->tmpl;
	u32 w1, w2, tx0, tx1, ty0, ty1, x, y, n = 0;
	int i, ret;

	static const u8 words_for_type[10] = { [6] = 19, [7] = 28, [8] = 38, [9] = 48 };

	if (b->type >= ARRAY_SIZE(words_for_type) || !words_for_type[b->type] ||
	    b->count != words_for_type[b->type])
		return -EINVAL;
	if ((b->dst_gpu & 0x7f) || (b->type >= 8 && (b->src_gpu & 0x7f)))
		return -EINVAL;
	tx0 = (p[8] & 0xffff) >> 3;
	tx1 = ((p[8] >> 16) + 7) >> 3;
	ty0 = (p[9] & 0xffff) >> 4;
	ty1 = ((p[9] >> 16) + 15) >> 4;
	if (tx0 >= tx1 || ty0 >= ty1 || tx1 > 128 || ty1 > 128 ||
	    (tx1 - tx0) * (ty1 - ty0) > MBX_TILE_BUF_SIZE / 8)
		return -EINVAL;

	mutex_lock(&mbx->lock);
	mbx_begin_light(mbx);

	/* Template: corners, surface words, vertices, colour, object list. */
	t[0xb8 / 4] = p[0]; t[0xbc / 4] = p[3]; t[0xc0 / 4] = p[0]; t[0xc4 / 4] = p[1];
	t[0xc8 / 4] = p[2]; t[0xcc / 4] = p[3]; t[0xd0 / 4] = p[2]; t[0xd4 / 4] = p[1];
	t[0x68 / 4] = MBX_OBJ_BG;
	t[0x6c / 4] = MBX_OBJ_BG2;
	w1 = p[4] | 0xa7700000 | (p[5] & 0xf) >> 3;
	w2 = ((b->dst_gpu >> 7) & 0x3ffff) | (p[5] >> 4) << 18 | 0xe000000;
	t[0x4 / 4] = w1;
	t[0x8 / 4] = w2;
	if (b->type >= 8) {
		mbx_patch_copy(mbx, b);
	} else if (b->type == 6) {
		for (i = 0; i < 8; i++)
			t[0x1a0 / 4 + i] = p[10 + i];
		for (i = 0; i < 4; i++)
			t[0x1e0 / 4 + i] = p[18];
		t[0x70 / 4] = MBX_OBJ_FILL;
	} else {
		static const u8 colour_map[12][2] = {
			{ 0x2f4 / 4, 18 }, { 0x2f8 / 4, 19 }, { 0x2fc / 4, 20 }, { 0x300 / 4, 18 },
			{ 0x304 / 4, 21 }, { 0x308 / 4, 22 }, { 0x30c / 4, 18 }, { 0x310 / 4, 23 },
			{ 0x314 / 4, 24 }, { 0x318 / 4, 18 }, { 0x31c / 4, 25 }, { 0x320 / 4, 26 },
		};

		t[0x2a4 / 4] = w1;
		t[0x2a8 / 4] = w2;
		t[0x2ac / 4] = p[27];
		for (i = 0; i < 8; i++)
			t[0x2b4 / 4 + i] = p[10 + i];
		for (i = 0; i < 12; i++)
			t[colour_map[i][0]] = p[colour_map[i][1]];
		t[0x70 / 4] = MBX_OBJ_BLEND;
	}
	t[0x74 / 4] = MBX_OBJ_END;

	/* Compact tile list of the covered 8x16 tiles -> object list (+0x68). */
	for (y = ty0; y < ty1; y++)
		for (x = tx0; x < tx1; x++) {
			mbx->tiles[n * 2] = (x & 0x7f) | (y & 0x7f) << 8;
			mbx->tiles[n * 2 + 1] = (mbx->tmpl_gpu + 0x68) & 0x1fffffc;
			n++;
		}
	mbx->tiles[(n - 1) * 2] |= 0x80000000;
	wmb();

	/* 3D blit register sequence */
	mbx_write(mbx, MBX_3D_CONFIG, mbx_read(mbx, MBX_3D_CONFIG) | 0x200);
	mbx_write(mbx, 0x6ec, 1);
	mbx_write(mbx, 0x608, mbx->tiles_gpu);
	mbx_write(mbx, 0x60c, mbx->tmpl_gpu);
	mbx_write(mbx, 0x610, 0);
	mbx_write(mbx, 0x614, 0x1e3ce508);
	mbx_write(mbx, 0x618, 0x1e3ce508);
	mbx_write(mbx, 0x61c, 0x00020007);
	for (i = 0x624; i <= 0x630; i += 4)
		mbx_write(mbx, i, 0);
	mbx_write(mbx, 0x63c, 0);
	mbx_write(mbx, 0x640, 0);
	mbx_write(mbx, 0x644, 0);
	mbx_write(mbx, 0x648, 1);
	mbx_write(mbx, 0x64c, 0);
	mbx_write(mbx, 0x664, 0);
	mbx_write(mbx, 0x6d0, 1);
	mbx_write(mbx, 0x6d8, 0x09000000);
	mbx_write(mbx, 0x6e0, 0);
	mbx_write(mbx, 0x6e4, 0xffffff04);
	mbx_write(mbx, 0x6e8, 1);
	mbx_write(mbx, 0x650, p[6]);
	mbx_write(mbx, 0x660, p[7]);
	mbx_write(mbx, 0x654, p[8]);
	mbx_write(mbx, 0x658, p[9]);
	mbx_write(mbx, 0x65c, b->dst_gpu);

	mbx_arm_wait(mbx, MBX_STATUS_RENDER_DONE);
	mbx_write(mbx, MBX_3D_START, 1);
	mbx->renders++;
	if (async) {
		mbx->pending = MBX_JOB_3D;
		mbx->pending_ms = b->timeout_ms ?: 500;
		if (!(++mbx->seq_sub & INT_MAX))	/* 0 means "nothing to wait for" */
			++mbx->seq_sub;
		ret = mbx->seq_sub & INT_MAX;
		mutex_unlock(&mbx->lock);
		return ret;
	}
	ret = mbx_wait_bits(mbx, b->timeout_ms ?: 500);
	if (ret) {
		mbx->render_timeouts++;
		mbx_recover(mbx);
	}
	mutex_unlock(&mbx->lock);
	return ret;
}

/* ----------------------------------------------------------- submission */

static int mbx_fifo_wait(struct apple_mbx *mbx, unsigned int *space)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(mbx->core + MBX_STATUS, status,
				 FIELD_GET(MBX_STATUS_FIFO_LEVEL, status) <
				 MBX_2D_FIFO_DEPTH, 0, 100000);
	if (!ret)
		*space = MBX_2D_FIFO_DEPTH - FIELD_GET(MBX_STATUS_FIFO_LEVEL, status);
	return ret;
}

int mbx_submit(struct apple_mbx *mbx, const u32 *words, u32 count,
		      unsigned int timeout_ms, bool async)
{
	unsigned int space = 0;
	u32 i;
	int ret = 0;
	ktime_t t0 = ktime_get(), t1, t2, t3;

	mutex_lock(&mbx->lock);
	t1 = ktime_get();
	mbx_begin_light(mbx);
	t2 = ktime_get();
	reinit_completion(&mbx->done);
	/* Make CPU writes to buffers visible before the GPU reads them. */
	wmb();

	for (i = 0; i < count; i++) {
		if (!space) {
			ret = mbx_fifo_wait(mbx, &space);
			if (ret)
				goto out;
		}
		writel(words[i], mbx->core + MBX_2D_FIFO + mbx->fifo_off);
		mbx->fifo_off = (mbx->fifo_off + 4) % MBX_2D_FIFO_WINDOW;
		space--;
	}
	/* End of batch: write at the start of the window. */
	writel(MBX_2D_END, mbx->core + MBX_2D_FIFO);
	t3 = ktime_get();

	if (async) {
		mbx->pending = MBX_JOB_2D;
		mbx->pending_ms = timeout_ms ?: 500;
		if (!(++mbx->seq_sub & INT_MAX))	/* 0 means "nothing to wait for" */
			++mbx->seq_sub;
		ret = mbx->seq_sub & INT_MAX;
		mbx->submits++;
		mbx->words += count;
		goto out;
	}
	if (!mbx_wait_completion_short(&mbx->done, timeout_ms ?: 500)) {
		dev_warn_ratelimited(mbx->dev, "2D batch timed out (status %08x)\n",
				     mbx_read(mbx, MBX_STATUS));
		ret = -ETIMEDOUT;
	}
	mbx->submits++;
	mbx->words += count;
	if (ktime_us_delta(ktime_get(), t0) > 20000)
		dev_dbg(mbx->dev, "slow submit (%u words): lock %lld begin %lld fifo %lld wait %lld us (irq after %lld us)\n",
			 count, ktime_us_delta(t1, t0), ktime_us_delta(t2, t1),
			 ktime_us_delta(t3, t2), ktime_us_delta(ktime_get(), t3),
			 ktime_us_delta(mbx->t_2d_irq, t3));
out:
	mutex_unlock(&mbx->lock);
	return ret;
}

int mbx_do_wait(struct apple_mbx *mbx, struct mbx_wait *w)
{
	int ret = 0;

	mutex_lock(&mbx->lock);
	if (w->wait_seq & MBX_SEQ_TA) {
		/* a queued TA frame: until it is finished (31-bit compare) */
#define MBX_TA_SEQ_DONE(seq) (mbx_ta_empty(mbx) || \
			      (s32)((READ_ONCE(mbx->ta_seq_done) - (seq)) << 1) >= 0)
		if (!MBX_TA_SEQ_DONE(w->wait_seq)) {
			mutex_unlock(&mbx->lock);
			if (!wait_event_timeout(mbx->ta_slot_wq, MBX_TA_SEQ_DONE(w->wait_seq),
						msecs_to_jiffies((w->timeout_ms ?: 500) * (MBX_TA_QUEUE + 1))))
				ret = -ETIMEDOUT;
			mutex_lock(&mbx->lock);
		}
#undef MBX_TA_SEQ_DONE
	} else if (mbx->pending &&
		   (!w->wait_seq || (s32)((mbx->seq_done - w->wait_seq) << 1) < 0)) {
		/* 2D/3D-blit numbers are 31 bits (an ioctl's return value): compare in 31 */
		/* wait_seq 0: just finish a job that has already ended */
		ret = mbx_idle_ms(mbx, w->wait_seq ? (w->timeout_ms ?: 500) : 0);
	}
	w->seq_submitted = mbx->seq_sub & INT_MAX;
	w->seq_done = mbx->seq_done & INT_MAX;
	mutex_unlock(&mbx->lock);
	return ret == -EBUSY ? 0 : ret;
}
