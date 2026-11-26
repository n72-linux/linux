/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _APPLE_MBX_H
#define _APPLE_MBX_H

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dma-resv.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <uapi/drm/apple_mbx_drm.h>
#include <drm/drm_device.h>

struct drm_gem_object;

/* Core registers */
#define MBX_SOFT_RESET		0x080
#define MBX_SOFT_RESET_ALL	0x111
#define MBX_ID_FFFF		0x100	/* reads 0xffff after power-on */
#define MBX_ID_FF		0x104	/* reads 0xff after power-on */
#define MBX_CONFIG		0x108
#define MBX_STATUS		0x12c
#define MBX_STATUS_EVENTS	0xffff
#define MBX_STATUS_2D_DONE	BIT(10)
#define MBX_STATUS_PB_DONE	BIT(6)
#define MBX_STATUS_RENDER_DONE	(BIT(2) | BIT(3) | BIT(6))
#define MBX_STATUS_FIFO_LEVEL	GENMASK(23, 16)
#define MBX_IRQ_ENABLE		0x130
#define MBX_IRQ_CLEAR		0x134
#define MBX_REVISION		0x640
#define MBX_REG_END		0x900

/* Default interrupt mask */
#define MBX_IRQ_MASK		0x867c

/* MMU: on-chip page directory of 8 entries x 4 MiB */
#define MBX_MMU_DIR(i)		(0x1000 + (i) * 4)
#define MBX_MMU_CTRL		0x1020
#define MBX_MMU_CTRL_ENABLE	BIT(0)
#define MBX_MMU_CTRL_INVAL	BIT(8)
#define MBX_MMU_STAT_ENABLED	BIT(16)
#define MBX_MMU_STAT_BUSY	BIT(24)
#define MBX_MMU_TLB_VPAGE	0x1024	/* GPU page number */
#define MBX_MMU_TLB_PPAGE	0x1028	/* physical page number */
#define MBX_MMU_DIR_ENTRIES	8
#define MBX_MMU_PT_ENTRIES	1024
#define MBX_GPU_PAGES		(MBX_MMU_DIR_ENTRIES * MBX_MMU_PT_ENTRIES)
#define MBX_PTE_INVALID		0xbad43210

/* 2D FIFO: a 64 KiB write window, 123 words deep */
#define MBX_2D_FIFO		0xa00000
#define MBX_2D_FIFO_WINDOW	0x10000
#define MBX_2D_FIFO_DEPTH	0x7b
#define MBX_2D_END		0xf0000000

#define MBX_SUBMIT_MAX_WORDS	65536

/* 3D core: parameter buffer and the blit path */
#define MBX_PB_CTRL		0x824
#define MBX_PB_FIRST		0x828
#define MBX_PB_LAST		0x82c
#define MBX_PB_START		0x838
#define MBX_PB_BASE		0x83c
#define MBX_PB_MIN		SZ_1M		/* floor for GL clients */
/*
 * Grow from 1 MiB to at most 2.25 MiB (1 -> 1.5 -> 2.25): a partial render
 * of a scene that had filled a 3 MiB buffer timed out in hardware (status
 * 00720000, ~0.5 s render against a 1 s limit), while partial renders at 1
 * and 2.25 MiB complete reliably.
 */
#define MBX_PB_MAX		(SZ_2M + SZ_256K)
#define MBX_PB_CTRL_SIZE	SZ_16K
#define MBX_TILE_BUF_SIZE	SZ_128K		/* up to 0x4000 tiles */
#define MBX_3D_CONFIG		0x85c
#define MBX_3D_START		0x680

/*
 * Tile accelerator (TA) frames: TA phase, then render phase with render-target
 * setup. The TA reads its words from a 100-entry FIFO behind a single port and
 * writes tile lists into the parameter buffer, region headers (one 8-byte
 * entry per 8x16 tile) and a tail-pointer buffer.
 */
#define MBX_TA_PORT		0x800000
#define MBX_TA_FIFO_DEPTH	100
#define MBX_TA_MAX_WORDS	262144
#define MBX_TA_MAX_DIM		1024
#define MBX_TA_TAIL_SIZE	SZ_64K		/* 16K entries: tiles up to 64 x 64 (512 x 1024) */
#define MBX_TA_CTRL_SIZE	SZ_4K
#define MBX_TA_RH_EMPTY_BIT	0x02000000
#define MBX_TA_RH_EMPTY		0x22000000
#define MBX_STATUS_TA_DONE	BIT(4)
#define MBX_STATUS_PB_FULL	BIT(5)
#define MBX_STATUS_TA_ERR	BIT(9)		/* hardware TA error; abort feed on 0x230 */
#define MBX_STATUS_SYNC		BIT(8)		/* 0x81c / 0x818 / 0x814 = 1 completed */
#define MBX_3D_CONFIG_BLIT	0x200		/* set by the blit path, cleared for TA frames */

/* Apple wrapper (s5l8720x variant) */
#define WRAP_RESET		0x0c

struct mbx_fb_info {
	u32 gpu_addr;		/* the display framebuffer, mapped at probe */
	u32 width;
	u32 height;
	u32 stride;		/* bytes */
	u32 format;		/* MBX_FMT_* */
};

struct apple_mbx {
	struct device *dev;
	void __iomem *core;
	void __iomem *wrap;
	struct clk *clk;
	bool clk_on;
	int irq;

	/* MMU */
	u32 *pt[MBX_MMU_DIR_ENTRIES];
	dma_addr_t pt_dma[MBX_MMU_DIR_ENTRIES];
	unsigned long *va_map;		/* one bit per GPU page */

	/* Display framebuffer, mapped into the GPU at probe */
	struct mbx_fb_info fb;

	/* Submission */
	struct mutex lock;		/* serialises MMU updates and submits */
	/*
	 * VGP microcode required by the TA path (passthrough is a VGP microcode
	 * section, not a bypass), loaded from /lib/firmware/apple-mbx/vgp.bin.
	 * The core keeps it until reset, so it is re-fed after probe, reset
	 * and resume.
	 */
	const struct firmware *vgp_fw;
	bool vgp_tried, vgp_loaded;
	struct completion done;
	/*
	 * Asynchronous jobs: at most one 2D batch or 3D blit runs while the
	 * caller goes on; every other GPU operation first waits for it
	 * (mbx_idle). seq_done catches up with seq_sub when it ends.
	 */
	u8 pending;			/* 0, MBX_JOB_2D or MBX_JOB_3D */
	u32 ta_cfg, ta_cfg108;		/* a queued TA frame: 0x3d config and 0x108 to put back after its render */
	/*
	 * TA frames queued by MBX_TA_ASYNC ioctls, up to
	 * MBX_TA_QUEUE of them, so the caller can build the next frames while
	 * the GPU works. ta_work runs them in order: feeds the TA, starts the
	 * render and sleeps until its interrupt (ta_rdone), then signals the
	 * frame's fence. ta_done completes when the queue runs empty. The
	 * worker takes mbx->lock only to start a frame (ta_busy); the
	 * compositor's 2D jobs wait just for the frame on the core
	 * (mbx_begin_light) and so go ahead of the rest, while MMU changes,
	 * frees and synchronous frames drain the queue (mbx_begin). ta_qlock
	 * guards the ring indices.
	 */
#define MBX_TA_QUEUE	3
	struct work_struct ta_work;
	struct completion ta_done;	/* the queue is empty */
	struct completion ta_rdone;	/* the running frame's render ended */
	struct completion ta_jdone;	/* the running frame is finished (ta_busy cleared) */
	bool ta_busy;			/* a queued frame is on the core */
	/*
	 * Overlapped pipeline (mbx_ta_pipeline): while ov is set the interrupt handler
	 * only records what it cleared: the TA's events in ov_seen (the TA
	 * phase waits on ov_wq or polls), the render's in ov_rseen, which
	 * completes ta_rdone when all of them have come. ov_irqs stops an
	 * interrupt that cannot be cleared (ov_storm).
	 */
	atomic_t ta_waiters;		/* in mbx_begin_light, for the frame(s) on the core to end */
	bool ov, ov_rarmed, ov_storm;
	u32 ov_seen, ov_rseen, ov_irqs;
	struct dma_fence *ov_rfence;
	ktime_t ov_rt_done;
	wait_queue_head_t ov_wq;
	u32 ta_idx;			/* region-header buffer of the last TA phase (0x810 / 0x6ec) */
	unsigned long ov_tas, ov_pb_full, ov_yields, ov_resumes, ov_partials;	/* debugfs */
	u32 ov_pb_pages;		/* most parameter-buffer pages a frame took */
	u32 ta_seq_sub, ta_seq_done;	/* MBX_SEQ_TA | 31 bits */
	/* queued frames since the stats were last read (debugfs) */
	u64 ta_stat_ta_us, ta_stat_render_us;
	unsigned long ta_stat_jobs;
	/*
	 * TA streams: a buffer for each queue slot and one for synchronous
	 * frames, filled from user space, DMA-able where that could be had
	 * (the PL080's LLIs sit behind the words). The DMA controller named
	 * by "apple,feed-dma" (DMAC1, a PL080 nothing else uses) feeds them to
	 * the TA port; its clock runs from the first DMA until the core is
	 * gated.
	 */
	phys_addr_t core_phys;
	void __iomem *dmac;
	struct clk *dmac_clk;
	bool dmac_clk_on;
	u32 dma_channel;
/* LLIs of one stream: one per 4092 words (MBX_DMA_LLI_WORDS, mbx_ta_dma_setup) */
#define MBX_TA_LLI_PER_SLOT	DIV_ROUND_UP(MBX_TA_MAX_WORDS, 4092)

	struct mbx_lli {
		u32 src, dst, next, cctl;
	} *ta_lli;
	dma_addr_t ta_lli_dma;
	struct mbx_ta_buf {
		u32 *words;
		dma_addr_t dma;		/* 0: plain memory, no DMA from it */
		u32 cap;		/* words */
	} ta_buf[MBX_TA_QUEUE + 2];
	spinlock_t ta_qlock;
	wait_queue_head_t ta_slot_wq;
	unsigned int ta_head, ta_tail;	/* ring of MBX_TA_QUEUE + 1 slots */
	u64 fence_ctx;
	u64 fence_seqno;
	spinlock_t fence_lock;
	spinlock_t irq_lock;
	struct mbx_ta_job {
		struct mbx_ta_frame f;
		const u32 *words;
		dma_addr_t dma;
		unsigned int slot;
		struct drm_gem_object *stream_obj;	/* MBX_TA_GEM_WORDS */
		bool stream_wc;				/* GEM BO mapped write-combined */
		u32 ntiles, tx, ty, tail_bytes;
		u32 clear_color, clear_depth;
		struct dma_fence *fence;	/* MBX_TA_FENCE */
		u32 seq;
		u32 idx;		/* pipeline: its region-header buffer */
		u32 tacfg;
		ktime_t t_render;
		/* pipeline, zeroed per frame (last, after the positional fields) */
		u32 feed_pos;		/* port feed: words written so far */
		bool dma_active;	/* fed by DMA (the channel runs until its last word) */
		ktime_t t_ta;		/* its TA phase started */
		u32 passes;		/* partial renders done (parameter buffer full) */
		u32 r63c;		/* 0x640 after the last of them: 0x63c of the next render */
	} ta_q[MBX_TA_QUEUE + 1];
	/*
	 * Idle clock gating: idle_gate_ms after the last use the core's clock
	 * is gated, as for system suspend; the next use brings it back the
	 * way resume did (reset, MMU, parameter buffer, VGP program).
	 */
	bool gated;
	ktime_t t_2d_irq;
	struct drm_device *drm;		/* the render node */
	unsigned long last_use;		/* jiffies */
	struct delayed_work idle_work;
	unsigned int pending_ms;
	u32 seq_sub, seq_done;

	/* 3D core */
	u32 *tmpl, *tiles;
	dma_addr_t tmpl_dma, tiles_dma, ctrl_dma;
	u32 tmpl_gpu, tiles_gpu, ctrl_gpu;
	/*
	 * The parameter buffer: GPU-contiguous, its pages not (the MMU maps
	 * each). Page 0 holds the header objects, which the CPU patches
	 * before every render: coherent memory. The pool behind it is the
	 * core's alone. A full buffer asks for 1.5 times the size (pb_want, at
	 * most MBX_PB_MAX); the pipeline replaces it once nothing is in it
	 * (mbx_pb_grow) at a later frame's TA start.
	 */
	struct mbx_pb {
		u32 *hdr;
		dma_addr_t hdr_dma;
		struct page **pages;	/* [1..n-1] */
		dma_addr_t *dma;
		u32 gpu, size;
	} pb;
	u32 pb_want;
	unsigned long pb_grows;
	/* the second region-header buffer (index 0; tiles is index 1) */
	u32 *tiles0;
	dma_addr_t tiles0_dma;
	u32 tiles0_gpu;
	struct completion done3d;
	u32 wait_mask, seen;
	unsigned long renders, render_timeouts;
	u32 fifo_off;

	/* TA frames: tail-pointer buffer and control page (region headers use tiles) */
	u32 *tail;
	void *tactrl;
	dma_addr_t tail_dma, tactrl_dma;
	u32 tail_gpu, tactrl_gpu;
	unsigned long ta_frames, ta_failures;

	struct dentry *debugfs;
	unsigned long irq_count;
	unsigned long event_count[16];
	unsigned long submits;
	unsigned long words;
};

struct mbx_bo {
	struct apple_mbx *mbx;
	struct kref ref;		/* held by its GEM object */
	u32 gpu_addr;
	unsigned int npages;
	bool cached;
	bool contig;
	struct page *cma;		/* contig from the CMA area: its first page */
	bool imported;			/* pages belong to a dma-buf exporter (PRIME import) */
	struct page **pages;
	dma_addr_t *dma;
};

#define MBX_JOB_2D	1
#define MBX_JOB_3D	3
/*
 * Sequence numbers: queued TA frames count in their own space, tagged with
 * bit 31 (MBX_SEQ_TA), so that a compositor's 2D job that runs ahead of them
 * is not made to wait for frames queued before it. 2D and 3D-blit numbers
 * are unchanged (31 bits, compared as 32-bit by their users); TA numbers
 * wrap in 31 bits, never 0.
 */
#define MBX_SEQ_TA	0x80000000u

#define MBX_TA_BUF_SLACK	128		/* words behind a stream (its LLIs live in ta_lli now) */

static inline u32 mbx_read(struct apple_mbx *mbx, u32 reg)
{
	return readl(mbx->core + reg);
}

static inline void mbx_write(struct apple_mbx *mbx, u32 reg, u32 val)
{
	writel(val, mbx->core + reg);
}

static inline bool mbx_ta_empty(struct apple_mbx *mbx)
{
	return READ_ONCE(mbx->ta_head) == READ_ONCE(mbx->ta_tail);
}

#define MBX_IDLE_GATE_MS	500

/* the core: MMU, idle gating, recovery, probe */
int mbx_mmu_flush(struct apple_mbx *mbx, u32 gpu_addr, unsigned int npages);
void mbx_mmu_set(struct apple_mbx *mbx, u32 gpu_addr, phys_addr_t pa);
u32 mbx_va_alloc(struct apple_mbx *mbx, unsigned int npages);
void mbx_va_free(struct apple_mbx *mbx, u32 gpu_addr, unsigned int npages);
bool mbx_wait_completion_short(struct completion *c, unsigned int ms);
int mbx_wait_bits(struct apple_mbx *mbx, unsigned int ms);
int mbx_idle_ms(struct apple_mbx *mbx, unsigned int ms);
int mbx_idle(struct apple_mbx *mbx);
int mbx_wake_locked(struct apple_mbx *mbx);
int mbx_begin(struct apple_mbx *mbx);
int mbx_begin_light(struct apple_mbx *mbx);
void mbx_arm_wait(struct apple_mbx *mbx, u32 mask);
void mbx_recover(struct apple_mbx *mbx);
void mbx_pb_grow(struct apple_mbx *mbx);

/* 2D and 3D-blit jobs, waiting */
void mbx_tmpl_reset(struct apple_mbx *mbx);
int mbx_blit3d(struct apple_mbx *mbx, const struct mbx_blit3d *b, bool async);
int mbx_submit(struct apple_mbx *mbx, const u32 *words, u32 count,
	       unsigned int timeout_ms, bool async);
int mbx_do_wait(struct apple_mbx *mbx, struct mbx_wait *w);

/* tile-accelerator frames */
int mbx_ta_frame(struct apple_mbx *mbx, struct mbx_ta_frame *f, const u32 __user *uwords,
		 struct dma_resv *resv, struct drm_gem_object *stream_obj,
		 struct mbx_bo *stream_bo);
void mbx_ta_work(struct work_struct *work);

/* the render node */
int mbx_drm_register(struct apple_mbx *mbx);

/* debugfs */
#ifdef CONFIG_DEBUG_FS
void mbx_debugfs_init(struct apple_mbx *mbx);
#else
static inline void mbx_debugfs_init(struct apple_mbx *mbx) { }
#endif

#endif /* _APPLE_MBX_H */
