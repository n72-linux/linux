// SPDX-License-Identifier: GPL-2.0
/*
 * MPEG-4 VOL/VOP and H.263 header parsers for the S5L8720 MPVD decoder driver.
 */

#include <linux/string.h>
#include <linux/videodev2.h>

#include "s5l8720-mpvd.h"

struct mpvd_bits {
	const u8 *p;
	u32 nbits, pos;
};

static u32 gb(struct mpvd_bits *b, unsigned int n)
{
	u32 v = 0;

	while (n--) {
		v <<= 1;
		if (b->pos < b->nbits)
			v |= (b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1;
		b->pos++;
	}
	return v;
}

static const u8 mpvd_zigzag[64] = {
	0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63 };

/* standard default quantization matrices */
static const u8 mpvd_def_intra[64] = {
	8, 17, 18, 19, 21, 23, 25, 27, 17, 18, 19, 21, 23, 25, 27, 28,
	20, 21, 22, 23, 24, 26, 28, 30, 21, 22, 23, 24, 26, 28, 30, 32,
	22, 23, 24, 26, 28, 30, 32, 35, 23, 24, 26, 28, 30, 32, 35, 38,
	25, 26, 28, 30, 32, 35, 38, 41, 27, 28, 30, 32, 35, 38, 41, 45 };
static const u8 mpvd_def_inter[64] = {
	16, 17, 18, 19, 20, 21, 22, 23, 17, 18, 19, 20, 21, 22, 23, 24,
	18, 19, 20, 21, 22, 23, 24, 25, 19, 20, 21, 22, 23, 24, 26, 27,
	20, 21, 22, 23, 25, 26, 27, 28, 21, 22, 23, 24, 26, 27, 28, 30,
	22, 23, 24, 26, 27, 28, 30, 31, 23, 24, 25, 27, 28, 30, 31, 33 };

static void mpvd_read_matrix(struct mpvd_bits *b, u8 *m)
{
	int i, last = 0, v;

	for (i = 0; i < 64; i++) {
		v = gb(b, 8);
		if (!v)
			break;
		m[mpvd_zigzag[i]] = last = v;
	}
	for (; i < 64; i++)
		m[mpvd_zigzag[i]] = last;
}

/* video_object_layer(), from behind its start code */
static int mpvd_parse_vol(struct mpvd_bits *b, struct mpvd_vol *v)
{
	unsigned int verid = 1, res, n;

	memset(v, 0, sizeof(*v));
	memcpy(v->intra, mpvd_def_intra, 64);
	memcpy(v->inter, mpvd_def_inter, 64);
	gb(b, 9);			/* random_accessible_vol, type indication */
	if (gb(b, 1)) {
		verid = gb(b, 4);
		gb(b, 3);
	}
	if (gb(b, 4) == 15)		/* aspect_ratio_info: extended */
		gb(b, 16);
	if (gb(b, 1)) {			/* vol_control_parameters */
		gb(b, 3);
		if (gb(b, 1))		/* vbv_parameters */
			gb(b, 79);
	}
	if (gb(b, 2))			/* shape: rectangular only */
		return -EINVAL;
	gb(b, 1);
	res = gb(b, 16);		/* vop_time_increment_resolution */
	gb(b, 1);
	for (n = res ? res - 1 : 0; n; n >>= 1)
		v->vti_bits++;
	if (!v->vti_bits)
		v->vti_bits = 1;
	if (gb(b, 1))			/* fixed_vop_rate */
		gb(b, v->vti_bits);
	gb(b, 1);
	v->width = gb(b, 13);
	gb(b, 1);
	v->height = gb(b, 13);
	gb(b, 1);
	if (gb(b, 1))			/* interlaced */
		return -EINVAL;
	gb(b, 1);			/* obmc_disable */
	if (gb(b, verid == 1 ? 1 : 2))	/* sprite_enable */
		return -EINVAL;
	v->quant_precision = 5;
	if (gb(b, 1)) {			/* not_8_bit */
		v->quant_precision = gb(b, 4);
		if (gb(b, 4) != 8)
			return -EINVAL;
	}
	v->quant_type = gb(b, 1);
	if (v->quant_type) {
		if (gb(b, 1))
			mpvd_read_matrix(b, v->intra);
		if (gb(b, 1))
			mpvd_read_matrix(b, v->inter);
	}
	if (verid != 1 && gb(b, 1))	/* quarter_sample: decodes wrongly */
		return -EINVAL;
	if (!gb(b, 1))			/* complexity_estimation_disable */
		return -EINVAL;
	gb(b, 1);			/* resync_marker_disable */
	if (gb(b, 1))			/* data_partitioned: ends in errors */
		return -EINVAL;
	if (verid != 1 && gb(b, 2))	/* newpred, reduced_resolution_vop */
		return -EINVAL;
	if (gb(b, 1))			/* scalability */
		return -EINVAL;
	if (b->pos > b->nbits || !v->width || !v->height)
		return -EINVAL;
	v->valid = true;
	return 0;
}

/* VideoObjectPlane(), from its start code, as far as the picture data */
static int mpvd_parse_vop(struct mpvd_bits *b, const struct mpvd_vol *v,
			  struct mpvd_vop *p)
{
	gb(b, 32);
	p->type = gb(b, 2);
	while (gb(b, 1) && b->pos < b->nbits)
		;			/* modulo_time_base */
	gb(b, 1);
	p->time_inc = gb(b, v->vti_bits);
	gb(b, 1);
	p->coded = gb(b, 1);
	p->fcode = 1;
	if (p->coded) {
		if (p->type == 1)
			p->rounding = gb(b, 1);
		p->intra_dc_thr = gb(b, 3);
		p->quant = gb(b, v->quant_precision);
		if (p->type)
			p->fcode = gb(b, 3);
		if (p->type == 2)
			gb(b, 3);
	}
	p->header_bits = b->pos;
	return b->pos > b->nbits ? -EINVAL : 0;
}

/* The H.263 picture header, and with it the "VOL" of such a stream. */
static int mpvd_parse_h263(struct mpvd_bits *b, struct mpvd_vol *v,
			   struct mpvd_vop *p)
{
	/* width, height, GOBs, macroblocks to a GOB; by source format */
	static const u16 fmt[5][4] = { { 0 }, { 128, 96, 6, 8 },
		{ 176, 144, 9, 11 }, { 352, 288, 18, 22 }, { 704, 576, 18, 88 } };
	unsigned int f;

	if (gb(b, 22) != 0x20)
		return -EINVAL;
	gb(b, 13);			/* temporal reference, PTYPE bits 1 to 5 */
	f = gb(b, 3);
	if (f < 1 || f > 4)
		return -EINVAL;
	p->type = gb(b, 1);
	if (gb(b, 4))			/* the optional modes */
		return -EINVAL;
	p->quant = gb(b, 5);
	gb(b, 1);
	while (gb(b, 1) && b->pos < b->nbits)
		gb(b, 8);
	p->coded = true;
	p->fcode = 1;
	p->num_gobs = fmt[f][2];
	p->mbs_per_gob = fmt[f][3];
	p->header_bits = b->pos;
	if (b->pos > b->nbits)
		return -EINVAL;

	memset(v, 0, sizeof(*v));
	memcpy(v->intra, mpvd_def_intra, 64);
	memcpy(v->inter, mpvd_def_inter, 64);
	v->width = fmt[f][0];
	v->height = fmt[f][1];
	v->short_header = true;
	v->valid = true;
	return 0;
}

/*
 * Go through one buffer. Returns 1 with *p filled in if it holds a
 * picture, 0 if it holds headers only, or an error.
 */
int mpvd_parse(struct mpvd_ctx *ctx, const u8 *d, u32 len,
	       struct mpvd_vop *p)
{
	struct mpvd_bits b;
	u32 i;
	int ret;

	memset(p, 0, sizeof(*p));
	while (len && !d[len - 1])
		len--;
	if (ctx->src_fmt.pixelformat == V4L2_PIX_FMT_H263) {
		for (i = 0; i + 4 < len; i++)
			if (!d[i] && !d[i + 1] && (d[i + 2] & 0xfc) == 0x80)
				break;
		if (i + 4 >= len)
			return -EINVAL;
		b = (struct mpvd_bits){ d + i, (len - i) * 8, 0 };
		ret = mpvd_parse_h263(&b, &ctx->vol, p);
		p->offset = i;
		p->len = len - i;
		return ret ? ret : 1;
	}
	for (i = 0; i + 4 <= len; i++) {
		if (d[i] || d[i + 1] || d[i + 2] != 1)
			continue;
		if (d[i + 3] >= 0x20 && d[i + 3] <= 0x2f) {
			b = (struct mpvd_bits){ d + i + 4, (len - i - 4) * 8, 0 };
			mpvd_parse_vol(&b, &ctx->vol);
		} else if (d[i + 3] == 0xb6) {
			if (!ctx->vol.valid)
				return -EINVAL;
			b = (struct mpvd_bits){ d + i, (len - i) * 8, 0 };
			ret = mpvd_parse_vop(&b, &ctx->vol, p);
			p->offset = i;
			p->len = len - i;
			return ret ? ret : 1;
		}
		i += 3;
	}
	return ctx->vol.valid ? 0 : -EINVAL;
}
