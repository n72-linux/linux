/* SPDX-License-Identifier: GPL-2.0 */
/* Internal definitions for the S5L8720 H.264 decoder driver. */
#ifndef S5L8720_H264_H
#define S5L8720_H264_H

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

#define H_REF_IDX(j)		(0x0000 + 4 * (j))	/* buffer -> list 0 index */
#define H_LIST(l, i)		(0x0100 + 0x80 * (l) + 4 * (i))
#define H_DIST_SCALE(i)		(0x0200 + 4 * (i))
#define  H_DIST_SCALE_NONE	0x400
#define H_WEIGHT(w)		(0x0400 + 4 * (w))
#define H_START			0x1000
#define H_RESET			0x1004	/* the bits clear themselves */
#define  H_RESET_READER		0x00c
#define  H_RESET_CORE		0x3e0
#define  H_RESET_ALL		0x7ff
#define H_FIELD_PIC		0x1008
#define H_TOP_FIELD		0x100c
#define H_DIRECT_SPATIAL	0x1010
#define H_MBAFF			0x1014
#define H_CONSTR_INTRA		0x1018
#define H_IS_REF		0x101c
#define H_CABAC			0x1020
#define H_CHROMA_QP_OFS		0x1024
#define H_SLICE_QP		0x1028
#define H_SLICE_TYPE		0x102c
#define H_WIDTH_MBS		0x1030
#define H_HEIGHT_MBS		0x1034
#define H_FIRST_MB_ROW		0x1038
#define H_FIRST_MB_COL		0x103c
#define H_NUM_REF_L0		0x1040
#define H_NUM_REF_L1		0x1044
#define H_COL_INFO		0x1048	/* the co-located picture, B slices */
#define  H_COL_FRAME		BIT(0)
#define  H_COL_BOTTOM		BIT(2)
#define  H_COL_SHORT_TERM	BIT(3)
#define  H_COL_DIRECT_8X8	BIT(4)
#define  H_COL_FRAME_MBS_ONLY	BIT(5)
#define H_WEIGHTED_PRED		0x104c
#define H_WEIGHTED_BIPRED	0x1050
#define H_LUMA_DENOM		0x1054
#define H_CHROMA_DENOM		0x1058
#define H_DEBLOCK_IDC		0x105c
#define H_DEBLOCK_ALPHA		0x1060
#define H_DEBLOCK_BETA		0x1064
#define H_WORK_BUFS		0x1068
#define H_DST_BUFS		0x106c
#define H_MB_POS		0x1070
#define H_STATUS		0x1074	/* bits 31:8 change while a slice runs */
#define  H_STATUS_MASK		GENMASK(7, 0)
#define  H_STATUS_DONE		BIT(0)
#define H_EG_CMD		0x1078
#define  H_EG_CMD_UE		1
#define  H_EG_CMD_SE		3
#define H_EG_RESULT		0x107c
#define  H_EG_DONE		BIT(7)
#define  H_EG_LONG		BIT(6)	/* only the leading zeros were counted */
#define  H_EG_LEN		GENMASK(5, 0)
#define H_INT_ACK		0x10c0
#define H_INT_MASK		0x10c4
#define H_CABAC_INIT_IDC	0x10cc
#define H_STATUS_EN		0x10d0
#define H_STREAM_FMT		0x10d4
#define  H_STREAM_FMT_HIGH	BIT(0)
#define  H_STREAM_FMT_8X8	BIT(3)
#define H_CTRL_D8		0x10d8	/* 3 then 1 at start and after a slice */
#define H_POSTFILTER		0x10e0
#define H_VERSION		0x10fc
#define H_MEM_BASE(i)		(0x1200 + 4 * (i))	/* address >> 10 */
#define  H_BUF_STREAM		0
#define  H_BUF_NEIGH_MV		1
#define  H_BUF_NEIGH_MB		2
#define  H_BUF_LUMA		3	/* + picture slot */
#define  H_BUF_CHROMA		23
#define  H_BUF_MV		43
#define H_GETBITS(n)		(0x1400 + 4 * ((n) & 31))	/* 0: 32 bits */
#define H_SHOWBITS		0x1480
#define H_STREAM_STATUS		0x1508
#define H_NAL_CMD		0x1600
#define  H_NAL_CMD_DECODE	0x801
#define H_NAL_HDR		0x1628
#define  H_NAL_HDR_VALID	BIT(8)
#define H_STREAM_CTRL		0x1804
#define H_STREAM_BASE_IDX	0x1808
#define H_STREAM_START		0x180c
#define H_STREAM_END		0x1810
#define H_STREAM_1814		0x1814
#define H_STREAM_1818		0x1818
#define  H_STREAM_OFS		GENMASK(22, 0)
#define H_SCALING_8X8(i)	(0x2800 + 0x100 * (i))
#define H_SCALING_4X4(i)	(0x2a00 + 0x40 * (i))
#define H_WEIGHT_128(w)		(0x3400 + 4 * (w))

#define H_NEIGH_MV_SIZE		SZ_16K
#define H_NEIGH_MB_SIZE		SZ_32K
#define H_MAX_SLOTS		20
#define H_POLL_TIMEOUT_US	10000
#define H_SLICE_TIMEOUT_MS	300

/*
 * 720x576 is the maximum size covered by the fixed neighbour buffers.
 */
#define H_MIN_SIZE		48U
#define H_MAX_WIDTH		720U
#define H_MAX_HEIGHT		576U

struct h264_dev;

struct h264_buf {
	struct v4l2_m2m_buffer m2m;
	/* capture buffers: the picture's motion vectors, for direct prediction */
	void *mv;
	dma_addr_t mv_dma;
	size_t mv_size;
};

struct h264_ctx {
	struct v4l2_fh fh;
	struct h264_dev *hd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_pix_format src_fmt, dst_fmt;
	struct v4l2_ctrl *sps, *pps, *matrix, *slice, *dec, *weights, *start_code;
	/* work memory of the block, kept apart for each stream */
	void *neigh_mv, *neigh_mb;
	dma_addr_t neigh_mv_dma, neigh_mb_dma;
};

struct h264_dev {
	struct device *dev;
	void __iomem *base, *axi;
	struct clk_bulk_data *clks;
	int nclks;
	struct v4l2_device v4l2;
	struct media_device mdev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m;
	struct mutex lock;		/* the video device and its queues */
	spinlock_t job_lock;		/* cur: the interrupt against the timeout */
	struct h264_ctx *cur;
	struct h264_ctx *last;		/* whose scaling lists are loaded */
	struct delayed_work timeout;
	unsigned int streaming;		/* queues streaming: the block is on */
	unsigned int slices, errors, timeouts, late_irqs;
	u32 last_status;
};

struct h264_run {
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_scaling_matrix *matrix;
	const struct v4l2_ctrl_h264_slice_params *slice;
	const struct v4l2_ctrl_h264_decode_params *dec;
	const struct v4l2_ctrl_h264_pred_weights *weights;
};

static inline u32 h_rd(struct h264_dev *hd, u32 reg)
{
	return readl(hd->base + reg);
}

static inline void h_wr(struct h264_dev *hd, u32 reg, u32 val)
{
	writel(val, hd->base + reg);
}

static inline struct h264_buf *to_h264_buf(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct h264_buf, m2m.vb);
}

int h264_hw_reset(struct h264_dev *hd, u32 bits);
int h264_hw_slice(struct h264_ctx *ctx, const struct h264_run *run,
		  struct vb2_v4l2_buffer *src, struct vb2_v4l2_buffer *dst);

#endif
