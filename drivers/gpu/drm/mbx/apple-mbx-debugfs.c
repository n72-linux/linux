// SPDX-License-Identifier: GPL-2.0-only
/* PowerVR MBX Lite on the Apple S5L8720: debugfs files. */

#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/seq_file.h>
#include "apple-mbx.h"

static int apple_mbx_regs_show(struct seq_file *s, void *unused)
{
	struct apple_mbx *mbx = s->private;
	u32 reg, val;
	int ret;

	mutex_lock(&mbx->lock);
	ret = mbx_begin(mbx);		/* registers of a gated core are not readable */
	if (ret) {
		mutex_unlock(&mbx->lock);
		return ret;
	}
	for (reg = 0; reg < MBX_REG_END; reg += 4) {
		val = mbx_read(mbx, reg);
		if (val)
			seq_printf(s, "core %03x: %08x\n", reg, val);
	}
	for (reg = MBX_MMU_DIR(0); reg <= MBX_MMU_TLB_PPAGE; reg += 4)
		seq_printf(s, "mmu  %03x: %08x\n", reg, mbx_read(mbx, reg));
	for (reg = 0; reg <= WRAP_RESET; reg += 4)
		seq_printf(s, "wrap %03x: %08x\n", reg, readl(mbx->wrap + reg));
	mutex_unlock(&mbx->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_mbx_regs);

static int apple_mbx_stats_show(struct seq_file *s, void *unused)
{
	struct apple_mbx *mbx = s->private;
	int b;

	seq_printf(s, "interrupts: %lu\n", mbx->irq_count);
	for (b = 0; b < ARRAY_SIZE(mbx->event_count); b++)
		if (mbx->event_count[b])
			seq_printf(s, "  bit %2d: %lu\n", b, mbx->event_count[b]);
	seq_printf(s, "submits: %lu, words: %lu\n", mbx->submits, mbx->words);
	seq_printf(s, "3D renders: %lu, timeouts: %lu\n", mbx->renders, mbx->render_timeouts);
	seq_printf(s, "TA frames: %lu, failures: %lu\n", mbx->ta_frames, mbx->ta_failures);
	seq_printf(s, "TA overlap: %lu TA phases during a render, %lu yields, most pages a frame %u of %u\n",
		   mbx->ov_tas, mbx->ov_yields, mbx->ov_pb_pages,
		   mbx->pb.size ? (u32)(mbx->pb.size - PAGE_SIZE) >> PAGE_SHIFT : 0);
	seq_printf(s, "parameter buffer %u KiB (%lu grows): full %lu, resumed after the render before %lu, partial renders %lu\n",
		   mbx->pb.size >> 10, mbx->pb_grows, mbx->ov_pb_full, mbx->ov_resumes - mbx->ov_partials,
		   mbx->ov_partials);
	if (mbx->ta_stat_jobs) {
		seq_printf(s, "queued frames since last read: %lu, TA %llu us, render %llu us on average\n",
			   mbx->ta_stat_jobs, div_u64(mbx->ta_stat_ta_us, mbx->ta_stat_jobs),
			   div_u64(mbx->ta_stat_render_us, mbx->ta_stat_jobs));
		mbx->ta_stat_jobs = 0;
		mbx->ta_stat_ta_us = mbx->ta_stat_render_us = 0;
	}
	seq_printf(s, "gpu pages in use: %u of %u\n",
		   bitmap_weight(mbx->va_map, MBX_GPU_PAGES), MBX_GPU_PAGES);
	seq_printf(s, "framebuffer: gpu %08x, %ux%u, stride %u\n",
		   mbx->fb.gpu_addr, mbx->fb.width, mbx->fb.height, mbx->fb.stride);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_mbx_stats);

void mbx_debugfs_init(struct apple_mbx *mbx)
{
	mbx->debugfs = debugfs_create_dir("apple-mbx", NULL);
	debugfs_create_file("regs", 0400, mbx->debugfs, mbx, &apple_mbx_regs_fops);
	debugfs_create_file("stats", 0400, mbx->debugfs, mbx, &apple_mbx_stats_fops);
}
