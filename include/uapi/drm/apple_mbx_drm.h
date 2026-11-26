/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Render-node interface of the PowerVR MBX Lite on Apple S5L8720
 * (/dev/dri/renderD*, driver name "apple-mbx").
 *
 * Buffers are GEM objects: create, mmap through the offset from
 * GEM_INFO, share with DRM_IOCTL_PRIME_HANDLE_TO_FD / FD_TO_HANDLE (the
 * display, n72-clcd, imports contiguous ones; the GPU imports any
 * dma-buf and maps it into its address space), free with
 * DRM_IOCTL_GEM_CLOSE. Command words address buffers by their GPU
 * address (one address space for the whole GPU).
 */
#ifndef _UAPI_APPLE_MBX_DRM_H_
#define _UAPI_APPLE_MBX_DRM_H_

#include <linux/ioctl.h>
#include <linux/types.h>
#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* 2D surface formats (destination/source word bits) */
#define MBX_FMT_ARGB8888	0x60000
#define MBX_FMT_XRGB8888	0x58000
#define MBX_FMT_RGB565		0x40000

/*
 * Cache maintenance for APPLE_MBX_BO_CACHED buffers (the CPU cache is
 * non-aliasing), the op of DRM_APPLE_MBX_GEM_SYNC. CLEAN before the GPU
 * reads a range; FLUSH (clean and invalidate) before the GPU writes it, so
 * the CPU then reads fresh data.
 */
#define MBX_SYNC_CLEAN		1
#define MBX_SYNC_FLUSH		2

#define DRM_APPLE_MBX_GEM_CREATE	0x00
#define DRM_APPLE_MBX_GEM_INFO		0x01
#define DRM_APPLE_MBX_GEM_SYNC		0x02
#define DRM_APPLE_MBX_SUBMIT		0x03
#define DRM_APPLE_MBX_BLIT3D		0x04
#define DRM_APPLE_MBX_TA_FRAME		0x05
#define DRM_APPLE_MBX_WAIT		0x06
#define DRM_APPLE_MBX_GET_PARAM		0x07

/* GEM_CREATE flags */
#define APPLE_MBX_BO_CACHED	(1 << 0)	/* CPU mapping cached: GEM_SYNC around GPU use */
#define APPLE_MBX_BO_CONTIG	(1 << 1)	/* physically contiguous: the display can scan it out */
#define APPLE_MBX_BO_NO_GPU	(1 << 2)	/* CPU/DMA only: skip GPU MMU mapping (gpu_addr = 0) */
#define MBX_ALLOC_CACHED	APPLE_MBX_BO_CACHED
#define MBX_ALLOC_CONTIG	APPLE_MBX_BO_CONTIG
#define MBX_ALLOC_NO_GPU	APPLE_MBX_BO_NO_GPU

struct drm_apple_mbx_gem_create {
	__u64 size;		/* in: bytes, rounded up to pages (at most 16 MiB) */
	__u32 flags;		/* in: APPLE_MBX_BO_* */
	__u32 handle;		/* out: GEM handle */
	__u32 gpu_addr;		/* out: the buffer's address in the GPU */
	__u32 pad;
};

struct drm_apple_mbx_gem_info {
	__u32 handle;		/* in: created or imported (PRIME) object */
	__u32 gpu_addr;		/* out */
	__u64 mmap_offset;	/* out: for mmap() on the render node (created objects only) */
	__u64 size;		/* out: bytes */
};

struct drm_apple_mbx_gem_sync {
	__u32 handle;
	__u32 op;		/* MBX_SYNC_CLEAN (CPU writes -> GPU) or MBX_SYNC_FLUSH (also drop lines, GPU writes -> CPU) */
	__u32 offset;
	__u32 size;
};

#define APPLE_MBX_JOB_ASYNC	(1 << 0)	/* return once queued; wait with DRM_APPLE_MBX_WAIT */

struct drm_apple_mbx_submit {			/* 2D command words */
	__u64 words;		/* user pointer to __u32 command words */
	__u32 count;
	__u32 timeout_ms;	/* 0: 500 ms */
	__u32 flags;		/* APPLE_MBX_JOB_* */
	__u32 seq;		/* out: the job's sequence number (async) */
};

/*
 * A blit on the 3D core: types 6/7 fill, 8/9 textured copy.
 * dst_gpu and (for types 8/9) src_gpu must be 128-byte aligned.
 */
struct mbx_blit3d {
	__u32 type;
	__u32 dst_gpu;
	__u32 src_gpu;
	__u32 timeout_ms;	/* 0: default (500 ms) */
	__u32 count;		/* number of words used */
	__u32 words[48];
};

struct drm_apple_mbx_blit3d {
	struct mbx_blit3d b;
	__u32 flags;		/* APPLE_MBX_JOB_* */
	__u32 seq;		/* out */
};

/*
 * A frame through the tile accelerator (TA): one scene's TA words rendered
 * into an ARGB8888 target. The target's old contents are loaded as the
 * background unless MBX_TA_DISCARD is set. The target must be 128-byte
 * aligned, mapped in the GPU, at most 1024 x 1024, its stride a multiple of
 * 64 bytes. Every wait is bounded by timeout_ms; a timeout resets the core.
 */
#define MBX_TA_DISCARD		(1 << 0)	/* background: magenta object, not the old pixels */
#define MBX_TA_DEPTH		(1 << 6)	/* depth_gpu is a depth buffer: tiles * 512 bytes, 128-byte aligned */
#define MBX_TA_DEPTH_LOAD	(1 << 7)	/* load the depth buffer's contents instead of starting empty */
#define MBX_TA_ASYNC		(1 << 8)	/* queue on the TA worker; seq gives the sequence number for WAIT */
#define MBX_TA_FENCE		(1 << 9)	/* with MBX_TA_ASYNC: write fence on the target's dma-buf */
#define MBX_TA_DEPTH_NOSTORE	(1 << 10)	/* with MBX_TA_DEPTH: keep depth on-chip per tile, skip DRAM store */
#define MBX_TA_GEM_WORDS	(1 << 11)	/* words is a CONTIG GEM handle (zero-copy TA stream) */
#define MBX_TA_CLEAR_BG		(1u << 12)	/* clear_color/clear_depth via hardware background object */
#define MBX_TA_STATUS_FENCED	(1u << 31)

/* mbx_ta_frame.stage: which wait failed */
#define MBX_TA_STAGE_DONE	0
#define MBX_TA_STAGE_TA_INIT	1		/* 0x81c = 1 -> status 0x100 */
#define MBX_TA_STAGE_FEED	2		/* room in the TA FIFO */
#define MBX_TA_STAGE_TA_DONE	3		/* status 0x10 with an empty FIFO */
#define MBX_TA_STAGE_RENDER_INIT1 4		/* 0x818 = 1 -> status 0x100 */
#define MBX_TA_STAGE_RENDER_INIT2 5		/* 0x814 = 1 -> status 0x100 */
#define MBX_TA_STAGE_RENDER	6		/* 0x680 = 1 -> status 0x4 + 0x8 + 0x40 */

struct mbx_ta_frame {
	__u64 words;		/* user pointer to __u32 TA words */
	__u32 count;		/* number of words (last one 0xf0000000) */
	__u32 dst_gpu;		/* colour target */
	__u32 width;		/* pixels */
	__u32 height;
	__u32 stride;		/* bytes */
	__u32 flags;		/* MBX_TA_* */
	__u32 timeout_ms;	/* per wait; 0: default (500 ms) */
	__u32 stage;		/* out: MBX_TA_STAGE_* */
	union {
		__u32 status;		/* out: status register when the frame ended or failed */
		__u32 clear_depth;	/* in (MBX_TA_CLEAR_BG): background clear depth float bits */
	};
	union {
		__u32 ta_us;		/* out: TA phase, microseconds */
		__u32 clear_color;	/* in (MBX_TA_CLEAR_BG): background clear colour 0xAARRGGBB */
	};
	union {
		__u32 render_us;	/* out: render phase */
		__u32 seq;		/* out, MBX_TA_ASYNC: the job's sequence number (> 0) */
	};
	__u32 depth_gpu;	/* with MBX_TA_DEPTH: depth buffer, ((w+15)/16*2) * ((h+15)/16) * 512 bytes */
};

/*
 * Asynchronous jobs: DRM_APPLE_MBX_SUBMIT and DRM_APPLE_MBX_BLIT3D return a
 * 31-bit sequence number, MBX_TA_ASYNC frames return bit 31 set | seq.
 * DRM_APPLE_MBX_WAIT reports seq_submitted / seq_done and, with wait_seq != 0,
 * waits until seq_done reaches it (timeout_ms, 0: 500 ms).
 */
struct mbx_wait {
	__u32 wait_seq;		/* in: 0 = only report */
	__u32 timeout_ms;	/* in */
	__u32 seq_submitted;	/* out */
	__u32 seq_done;		/* out */
};

#define APPLE_MBX_PARAM_REVISION	1
#define APPLE_MBX_PARAM_FB_GPU_ADDR	2	/* boot framebuffer, mapped into the GPU at probe */
#define APPLE_MBX_PARAM_FB_WIDTH	3
#define APPLE_MBX_PARAM_FB_HEIGHT	4
#define APPLE_MBX_PARAM_FB_STRIDE	5
#define APPLE_MBX_PARAM_FB_FORMAT	6	/* MBX_FMT_* */

struct drm_apple_mbx_get_param {
	__u32 param;
	__u32 pad;
	__u64 value;		/* out */
};

#define DRM_IOCTL_APPLE_MBX_GEM_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_MBX_GEM_CREATE, struct drm_apple_mbx_gem_create)
#define DRM_IOCTL_APPLE_MBX_GEM_INFO	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_MBX_GEM_INFO, struct drm_apple_mbx_gem_info)
#define DRM_IOCTL_APPLE_MBX_GEM_SYNC	DRM_IOW(DRM_COMMAND_BASE + DRM_APPLE_MBX_GEM_SYNC, struct drm_apple_mbx_gem_sync)
#define DRM_IOCTL_APPLE_MBX_SUBMIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_MBX_SUBMIT, struct drm_apple_mbx_submit)
#define DRM_IOCTL_APPLE_MBX_BLIT3D	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_MBX_BLIT3D, struct drm_apple_mbx_blit3d)
#define DRM_IOCTL_APPLE_MBX_TA_FRAME	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_MBX_TA_FRAME, struct mbx_ta_frame)
#define DRM_IOCTL_APPLE_MBX_WAIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_MBX_WAIT, struct mbx_wait)
#define DRM_IOCTL_APPLE_MBX_GET_PARAM	DRM_IOWR(DRM_COMMAND_BASE + DRM_APPLE_MBX_GET_PARAM, struct drm_apple_mbx_get_param)

#if defined(__cplusplus)
}
#endif

#endif
