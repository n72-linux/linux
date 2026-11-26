// SPDX-License-Identifier: GPL-2.0
/*
 * Slice header reader and hardware slice programming for the S5L8720 H.264
 * decoder.
 */

#include <linux/iopoll.h>
#include <linux/kernel.h>

#include <media/v4l2-h264.h>
#include <media/videobuf2-dma-contig.h>

#include "s5l8720-h264.h"

/*
 * Give the block a NAL unit, header byte first, and have it decode the
 * header. Returns the header's low seven bits (nal_ref_idc, nal_unit_type).
 */
static int h264_hw_nal(struct h264_dev *hd, dma_addr_t addr, u32 len)
{
	u32 ofs = addr & 0x3ff, v;
	int ret;

	ret = h264_hw_reset(hd, H_RESET_READER);
	if (ret)
		return ret;
	h_wr(hd, H_MEM_BASE(H_BUF_STREAM), addr >> 10);
	h_wr(hd, H_STREAM_START, ofs);
	h_wr(hd, H_STREAM_END, ofs + len);
	h_wr(hd, H_NAL_HDR, h_rd(hd, H_NAL_HDR) & ~0x1ffu);
	h_wr(hd, H_STREAM_1814, 0x20);
	h_wr(hd, H_STREAM_1818, 8);
	h_wr(hd, H_STREAM_CTRL, h_rd(hd, H_STREAM_CTRL) | 1);
	h_wr(hd, H_NAL_CMD, (h_rd(hd, H_NAL_CMD) & ~0xfffu) | H_NAL_CMD_DECODE);
	ret = readl_poll_timeout_atomic(hd->base + H_NAL_HDR, v,
					 v & H_NAL_HDR_VALID, 0, H_POLL_TIMEOUT_US);
	return ret ? ret : (v & 0x7f);
}

/*
 * Hardware bit reader: GETBITS stalls the bus until bits are available, so
 * bit consumption is tracked against the NAL limit and reads never pass end.
 */
struct h264_rd {
	struct h264_dev *hd;
	u32 used, limit;
	bool bad;
};

static u32 rd_u(struct h264_rd *r, unsigned int n)
{
	if (!n)
		return 0;
	if (r->bad || n > r->limit - r->used) {
		r->bad = true;
		return 0;
	}
	r->used += n;
	return h_rd(r->hd, H_GETBITS(n));
}

/*
 * Exp-Golomb. The result does not move the reader: for a short code a
 * read of its length skips it; for a long one the count of leading zeros
 * is skipped and the following zeros + 1 bits are the code number + 1.
 */
static u32 rd_golomb(struct h264_rd *r, u32 cmd)
{
	struct h264_dev *hd = r->hd;
	u32 v = 0, len;

	if (r->bad || r->used >= r->limit) {
		r->bad = true;
		return 0;
	}
	h_wr(hd, H_EG_CMD, cmd);
	if (readl_poll_timeout_atomic(hd->base + H_EG_RESULT, v, v & H_EG_DONE,
				      0, H_POLL_TIMEOUT_US))
		goto bad;
	len = v & H_EG_LEN;
	if (!(v & H_EG_DONE) || !len || len > 31)
		goto bad;
	if (!(v & H_EG_LONG)) {
		if (len > r->limit - r->used)
			goto bad;
		r->used += len;
		h_rd(hd, H_GETBITS(len));
		return cmd == H_EG_CMD_SE ? (u32)((s32)v >> 8) : v >> 8;
	}
	if (len > 30 || 2 * len + 1 > r->limit - r->used)
		goto bad;
	r->used += 2 * len + 1;
	h_rd(hd, H_GETBITS(len));
	v = h_rd(hd, H_GETBITS(len + 1)) - 1;
	if (cmd == H_EG_CMD_SE)
		return (v & 1) ? (v >> 1) + 1 : -(v >> 1);
	return v;
bad:
	r->bad = true;
	return 0;
}

static u32 rd_ue(struct h264_rd *r)
{
	return rd_golomb(r, H_EG_CMD_UE);
}

static s32 rd_se(struct h264_rd *r)
{
	return rd_golomb(r, H_EG_CMD_SE);
}

/*
 * Read slice_header() through the hardware, which leaves its reader at
 * slice_data(). The header has to agree with the controls where that can
 * be seen, or the block would be started on something else.
 */
static int h264_walk_header(struct h264_dev *hd, const struct h264_run *run,
			    bool idr, u32 limit)
{
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	struct h264_rd r = { .hd = hd, .limit = limit };
	unsigned int nref[2], nlists, type, first_mb, i, l, op;
	bool field = false;

	first_mb = rd_ue(&r);
	type = rd_ue(&r) % 5;
	rd_ue(&r);				/* pic_parameter_set_id */
	if (r.bad || first_mb != run->slice->first_mb_in_slice ||
	    type != run->slice->slice_type)
		return -EINVAL;
	if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		rd_u(&r, 2);
	rd_u(&r, sps->log2_max_frame_num_minus4 + 4);
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
		field = rd_u(&r, 1);
		if (field)
			rd_u(&r, 1);
	}
	if (idr)
		rd_ue(&r);			/* idr_pic_id */
	if (sps->pic_order_cnt_type == 0) {
		rd_u(&r, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !field)
			rd_se(&r);
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		rd_se(&r);
		if ((pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !field)
			rd_se(&r);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		rd_ue(&r);
	if (type == V4L2_H264_SLICE_TYPE_B)
		rd_u(&r, 1);			/* direct_spatial_mv_pred_flag */

	nref[0] = pps->num_ref_idx_l0_default_active_minus1 + 1;
	nref[1] = pps->num_ref_idx_l1_default_active_minus1 + 1;
	nlists = 0;
	if (type == V4L2_H264_SLICE_TYPE_P || type == V4L2_H264_SLICE_TYPE_SP)
		nlists = 1;
	else if (type == V4L2_H264_SLICE_TYPE_B)
		nlists = 2;
	if (nlists && rd_u(&r, 1)) {		/* num_ref_idx_active_override_flag */
		nref[0] = rd_ue(&r) + 1;
		if (nlists == 2)
			nref[1] = rd_ue(&r) + 1;
	}
	if (nref[0] > 32 || nref[1] > 32)
		return -EINVAL;

	/* ref_pic_list_modification() */
	for (l = 0; l < nlists; l++) {
		if (!rd_u(&r, 1))
			continue;
		for (i = 0; i < 66 && !r.bad; i++) {
			if (rd_ue(&r) == 3)
				break;
			rd_ue(&r);
		}
		if (i == 66)
			return -EINVAL;
	}

	if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, run->slice)) {
		rd_ue(&r);
		if (sps->chroma_format_idc)
			rd_ue(&r);
		for (l = 0; l < nlists; l++) {
			for (i = 0; i < nref[l] && !r.bad; i++) {
				if (rd_u(&r, 1)) {
					rd_se(&r);
					rd_se(&r);
				}
				if (sps->chroma_format_idc && rd_u(&r, 1)) {
					rd_se(&r);
					rd_se(&r);
					rd_se(&r);
					rd_se(&r);
				}
			}
		}
	}

	/* dec_ref_pic_marking() */
	if (run->dec->nal_ref_idc) {
		if (idr) {
			rd_u(&r, 2);
		} else if (rd_u(&r, 1)) {
			for (i = 0; i < 66 && !r.bad; i++) {
				op = rd_ue(&r);
				if (!op)
					break;
				if (op > 6)
					return -EINVAL;
				if (op != 5)
					rd_ue(&r);
				if (op == 3)
					rd_ue(&r);
			}
			if (i == 66)
				return -EINVAL;
		}
	}

	if ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
	    type != V4L2_H264_SLICE_TYPE_I && type != V4L2_H264_SLICE_TYPE_SI)
		rd_ue(&r);			/* cabac_init_idc */
	rd_se(&r);				/* slice_qp_delta */
	if (type == V4L2_H264_SLICE_TYPE_SP || type == V4L2_H264_SLICE_TYPE_SI) {
		if (type == V4L2_H264_SLICE_TYPE_SP)
			rd_u(&r, 1);
		rd_se(&r);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		if (rd_ue(&r) != 1) {
			rd_se(&r);
			rd_se(&r);
		}
	}
	return r.bad ? -EINVAL : 0;
}

/* One reference picture, as far as the block and the B-slice sums need it. */
struct h264_ref {
	unsigned int slot;		/* picture slot, the capture buffer's index */
	s32 poc, top_poc, bottom_poc;
	bool long_term;
	bool valid;
};

static void h264_hw_slot(struct h264_dev *hd, const struct h264_ctx *ctx,
			 struct vb2_buffer *vb)
{
	dma_addr_t luma = vb2_dma_contig_plane_dma_addr(vb, 0);
	dma_addr_t chroma = luma + ctx->dst_fmt.bytesperline * ctx->dst_fmt.height;
	unsigned int k = vb->index;

	h_wr(hd, H_MEM_BASE(H_BUF_LUMA + k), luma >> 10);
	h_wr(hd, H_MEM_BASE(H_BUF_CHROMA + k), chroma >> 10);
	h_wr(hd, H_MEM_BASE(H_BUF_MV + k), to_h264_buf(vb)->mv_dma >> 10);
}

static s32 h264_dist_scale(s32 cur, const struct h264_ref *r0,
			   const struct h264_ref *r1)
{
	int tb = clamp(cur - r0->poc, -128, 127);
	int td = clamp(r1->poc - r0->poc, -128, 127);
	int tx;

	if (r0->long_term || !td)
		return H_DIST_SCALE_NONE;
	tx = (16384 + abs(td / 2)) / td;
	return clamp((tb * tx + 32) >> 6, -1024, 1023);
}

static void h264_hw_scaling(struct h264_dev *hd, const struct h264_run *run)
{
	const struct v4l2_ctrl_h264_scaling_matrix *m = run->matrix;
	bool flat = !(run->pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT);
	int i, j;

	/* raster order on both sides; fall back to flat 16 lists when absent */
	for (i = 0; i < 6; i++)
		for (j = 0; j < 16; j++)
			h_wr(hd, H_SCALING_4X4(i) + 4 * j,
			     flat ? 16 : m->scaling_list_4x4[i][j]);
	for (i = 0; i < 2; i++)
		for (j = 0; j < 64; j++)
			h_wr(hd, H_SCALING_8X8(i) + 4 * j,
			     flat ? 16 : m->scaling_list_8x8[i][j]);
}

static void h264_hw_weights(struct h264_dev *hd, const struct h264_run *run,
			    const struct h264_ref *list[2][32], s32 cur_poc)
{
	const struct v4l2_ctrl_h264_slice_params *sl = run->slice;
	const struct v4l2_ctrl_h264_pred_weights *pw = run->weights;
	bool b = sl->slice_type == V4L2_H264_SLICE_TYPE_B;
	unsigned int n[2] = { sl->num_ref_idx_l0_active_minus1 + 1,
			      b ? sl->num_ref_idx_l1_active_minus1 + 1 : 0 };
	unsigned int l, i, c;

	if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(run->pps, sl)) {
		/* A weight of 128 overflows signed 8 bits and sets the H_WEIGHT_128 bit. */
		for (l = 0; l < 2; l++) {
			const struct v4l2_h264_weight_factors *f = &pw->weight_factors[l];

			for (i = 0; i < n[l]; i++) {
				unsigned int w = 64 * l + i;

				h_wr(hd, H_WEIGHT(w), (f->luma_weight[i] & 0xff) |
				     (f->luma_offset[i] & 0xff) << 8);
				h_wr(hd, H_WEIGHT(w + 32),
				     (f->chroma_weight[i][0] & 0xff) |
				     (f->chroma_offset[i][0] & 0xff) << 8 |
				     (f->chroma_weight[i][1] & 0xff) << 16 |
				     (u32)(f->chroma_offset[i][1] & 0xff) << 24);
				h_wr(hd, H_WEIGHT_128(w), f->luma_weight[i] == 128);
				h_wr(hd, H_WEIGHT_128(w + 32),
				     (f->chroma_weight[i][0] == 128) |
				     (f->chroma_weight[i][1] == 128) << 1);
			}
		}
		h_wr(hd, H_LUMA_DENOM, pw->luma_log2_weight_denom);
		h_wr(hd, H_CHROMA_DENOM, pw->chroma_log2_weight_denom);
		return;
	}
	if (!b || run->pps->weighted_bipred_idc != 2) {
		h_wr(hd, H_LUMA_DENOM, 0);
		h_wr(hd, H_CHROMA_DENOM, 0);
		return;
	}

	/*
	 * Implicit weights: the weight of the list 0 picture for every pair,
	 * four list 0 entries to a word; the block forms the other one.
	 */
	for (l = 0; l < n[1]; l++) {
		for (i = 0; i < n[0]; i += 4) {
			u32 word = 0, flags = 0;

			for (c = 0; c < 4 && i + c < n[0]; c++) {
				const struct h264_ref *r0 = list[0][i + c];
				const struct h264_ref *r1 = list[1][l];
				int w0 = 32, d;

				if (r0->valid && r1->valid && !r0->long_term &&
				    !r1->long_term && r0->poc != r1->poc) {
					d = h264_dist_scale(cur_poc, r0, r1) >> 2;
					if (d >= -64 && d <= 128)
						w0 = 64 - d;
				}
				word |= (u32)(w0 & 0xff) << (8 * c);
				flags |= (w0 == 128) << c;
			}
			h_wr(hd, H_WEIGHT(l * 8 + i / 4), word);
			h_wr(hd, H_WEIGHT_128(l * 8 + i / 4), flags);
		}
	}
	h_wr(hd, H_LUMA_DENOM, 5);
	h_wr(hd, H_CHROMA_DENOM, 5);
}

/* Everything for one slice, up to and including the start. */
int h264_hw_slice(struct h264_ctx *ctx, const struct h264_run *run,
		  struct vb2_v4l2_buffer *src, struct vb2_v4l2_buffer *dst)
{
	struct h264_dev *hd = ctx->hd;
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_slice_params *sl = run->slice;
	const struct v4l2_ctrl_h264_decode_params *dec = run->dec;
	struct vb2_queue *cap_q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
						  V4L2_BUF_TYPE_VIDEO_CAPTURE);
	bool b = sl->slice_type == V4L2_H264_SLICE_TYPE_B;
	bool inter = b || sl->slice_type == V4L2_H264_SLICE_TYPE_P ||
		     sl->slice_type == V4L2_H264_SLICE_TYPE_SP;
	unsigned int width_mbs = sps->pic_width_in_mbs_minus1 + 1;
	unsigned int k = dst->vb2_buf.index, n[2], i, l;
	struct h264_ref refs[V4L2_H264_NUM_DPB_ENTRIES], self;
	const struct h264_ref *list[2][32];
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 len = vb2_get_plane_payload(&src->vb2_buf, 0), ofs = 0, v;
	s32 cur_poc = min(dec->top_field_order_cnt, dec->bottom_field_order_cnt);
	int hdr, ret;

	/* the NAL unit: after a start code if there is one, without padding */
	if (ctx->start_code->cur.val == V4L2_STATELESS_H264_START_CODE_ANNEX_B) {
		while (ofs + 3 < len && !data[ofs])
			ofs++;
		if (ofs < 2 || data[ofs] != 1)
			return -EINVAL;
		ofs++;
	}
	while (len > ofs && !data[len - 1])
		len--;
	if (len < ofs + 3)
		return -EINVAL;
	len -= ofs;

	hdr = h264_hw_nal(hd, vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) + ofs,
			  len);
	if (hdr < 0)
		return hdr;
	if ((hdr & 0x1f) != 1 && (hdr & 0x1f) != 5)
		return -EINVAL;
	ret = h264_walk_header(hd, run, (hdr & 0x1f) == 5, (len - 1) * 8);
	if (ret)
		return ret;

	/* buffers: this stream's work memory, the picture, its references */
	h_wr(hd, H_MEM_BASE(H_BUF_NEIGH_MV), ctx->neigh_mv_dma >> 10);
	h_wr(hd, H_MEM_BASE(H_BUF_NEIGH_MB), ctx->neigh_mb_dma >> 10);
	h264_hw_slot(hd, ctx, &dst->vb2_buf);
	self = (struct h264_ref){ .slot = k, .poc = cur_poc };
	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		const struct v4l2_h264_dpb_entry *e = &dec->dpb[i];
		struct vb2_buffer *vb = NULL;

		/* a missing picture is replaced by the one being decoded */
		refs[i] = self;
		if (e->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID)
			vb = vb2_find_buffer(cap_q, e->reference_ts);
		if (!vb || vb->index >= H_MAX_SLOTS || !to_h264_buf(vb)->mv)
			continue;
		refs[i].valid = true;
		refs[i].slot = vb->index;
		refs[i].top_poc = e->top_field_order_cnt;
		refs[i].bottom_poc = e->bottom_field_order_cnt;
		refs[i].poc = min(e->top_field_order_cnt, e->bottom_field_order_cnt);
		refs[i].long_term = e->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;
		if (vb != &dst->vb2_buf)
			h264_hw_slot(hd, ctx, vb);
	}

	/* a new picture, or another stream had the block: the scaling lists */
	if (sps->profile_idc >= 100 && (!dst->is_held || hd->last != ctx))
		h264_hw_scaling(hd, run);
	hd->last = ctx;

	n[0] = inter ? sl->num_ref_idx_l0_active_minus1 + 1 : 0;
	n[1] = b ? sl->num_ref_idx_l1_active_minus1 + 1 : 0;
	if (n[0] > 32 || n[1] > 32)
		return -EINVAL;
	for (l = 0; l < 2; l++) {
		const struct v4l2_h264_reference *rl = l ? sl->ref_pic_list1 :
							  sl->ref_pic_list0;

		for (i = 0; i < 32; i++) {
			const struct h264_ref *r = &self;

			if (i < n[l] && rl[i].index < V4L2_H264_NUM_DPB_ENTRIES)
				r = &refs[rl[i].index];
			list[l][i] = r;
		}
	}

	h264_hw_weights(hd, run, list, cur_poc);
	h_wr(hd, H_CABAC_INIT_IDC, sl->cabac_init_idc);

	if (inter) {
		for (l = 0; l < 2; l++)
			for (i = 0; i < 32; i++)
				h_wr(hd, H_LIST(l, i), i >= n[l] ? 0 :
				     (H_BUF_LUMA + list[l][i]->slot) |
				     (H_BUF_CHROMA + list[l][i]->slot) << 8);
		/* which list 0 entry a buffer is, the lowest entry to win */
		for (i = n[0]; i-- > 0;)
			h_wr(hd, H_REF_IDX((H_BUF_LUMA + list[0][i]->slot) & 0x1f), i);

		v = h_rd(hd, H_COL_INFO) & ~0x3fu;
		if (b) {
			const struct h264_ref *col = list[1][0];

			for (i = 0; i < n[0]; i++)
				h_wr(hd, H_DIST_SCALE(i),
				     h264_dist_scale(cur_poc, list[0][i], col) & 0xfff);
			v |= H_COL_FRAME;
			if (abs(col->bottom_poc - cur_poc) <= abs(col->top_poc - cur_poc))
				v |= H_COL_BOTTOM;
			if (!col->long_term)
				v |= H_COL_SHORT_TERM;
			if (sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE)
				v |= H_COL_DIRECT_8X8;
			if (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)
				v |= H_COL_FRAME_MBS_ONLY;
		}
		h_wr(hd, H_COL_INFO, v);
	}

	h_wr(hd, H_FIRST_MB_ROW, sl->first_mb_in_slice / width_mbs);
	h_wr(hd, H_FIRST_MB_COL, sl->first_mb_in_slice % width_mbs);
	h_wr(hd, H_FIELD_PIC, 0);
	h_wr(hd, H_TOP_FIELD, 1);
	h_wr(hd, H_DIRECT_SPATIAL,
	     !!(sl->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED));
	h_wr(hd, H_MBAFF, 0);
	h_wr(hd, H_CONSTR_INTRA,
	     !!(pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED));
	h_wr(hd, H_IS_REF, dec->nal_ref_idc != 0);
	h_wr(hd, H_CABAC, !!(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE));
	h_wr(hd, H_CHROMA_QP_OFS, (pps->chroma_qp_index_offset & 0x1f) |
	     (pps->second_chroma_qp_index_offset & 0x1f) << 5);
	h_wr(hd, H_SLICE_QP, 26 + pps->pic_init_qp_minus26 + sl->slice_qp_delta);
	h_wr(hd, H_SLICE_TYPE, sl->slice_type);
	h_wr(hd, H_WIDTH_MBS, width_mbs);
	h_wr(hd, H_HEIGHT_MBS, sps->pic_height_in_map_units_minus1 + 1);
	h_wr(hd, H_NUM_REF_L0, sl->num_ref_idx_l0_active_minus1);
	h_wr(hd, H_NUM_REF_L1, b ? sl->num_ref_idx_l1_active_minus1 : ~0u);
	h_wr(hd, H_WEIGHTED_PRED, !!(pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED));
	h_wr(hd, H_WEIGHTED_BIPRED, pps->weighted_bipred_idc);
	h_wr(hd, H_DEBLOCK_IDC, sl->disable_deblocking_filter_idc);
	h_wr(hd, H_DEBLOCK_ALPHA, sl->slice_alpha_c0_offset_div2 * 2);
	h_wr(hd, H_DEBLOCK_BETA, sl->slice_beta_offset_div2 * 2);
	h_wr(hd, H_WORK_BUFS, (H_BUF_MV + k) |
	     (b ? H_BUF_MV + list[1][0]->slot : 0) << 8 |
	     H_BUF_NEIGH_MV << 16 | H_BUF_NEIGH_MB << 24);
	h_wr(hd, H_DST_BUFS, (H_BUF_LUMA + k) | (H_BUF_CHROMA + k) << 8);
	h_wr(hd, H_STREAM_FMT, (sps->profile_idc == 100 ? H_STREAM_FMT_HIGH : 0) |
	     sps->chroma_format_idc << 1 |
	     ((pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) ?
	      H_STREAM_FMT_8X8 : 0));

	h_wr(hd, H_EG_CMD, 0);
	h_wr(hd, H_START, 1);
	return 0;
}
