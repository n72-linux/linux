// SPDX-License-Identifier: GPL-2.0-only
/*
 * n72-vsvfl.c - Apple VSVFL reader for the iPod touch 2G (N72).
 */
#include <linux/string.h>
#include "n72-yaftl-core.h"

#define LOG(ops, ...) do { if ((ops)->log) (ops)->log(__VA_ARGS__); } while (0)

static uint16_t le16(const uint8_t *p)
{
	return p[0] | p[1] << 8;
}

static uint32_t le32(const uint8_t *p)
{
	return le16(p) | (uint32_t)le16(p + 2) << 16;
}

/* [V5] */
static int vfl_cxt_intact(const uint8_t *cxt)
{
	uint32_t sum = 0xaabbccdd, x = 0xaabbccdd, off;

	for (off = 0; off < VFL_CXT_SUM; off += 4) {
		sum += le32(cxt + off);
		x ^= le32(cxt + off);
	}
	return sum == le32(cxt + VFL_CXT_SUM) || x != le32(cxt + VFL_CXT_XOR);
}

/* [V8] The physical block, on its CE, of block @vblock of virtual bank @vbank. */
static uint32_t vfl_bank_block(const struct nand_geom *g, uint32_t vbank, uint32_t vblock)
{
	return 2 * vblock + (vbank / g->chip_enables) % 2;
}

/* [V9] The reserve block that entry @idx of a pool map is about. */
static uint32_t vfl_pool_block(const struct vfl *v, uint32_t idx)
{
	return 2 * (v->pool_start + idx % v->reserved_per_bank) +
	       (idx / v->reserved_per_bank) % 2;
}

/*
 * [V8] Where a virtual page lies: its CE, its block there (before any
 * stand-in) and the page within the block.
 */
static uint32_t vfl_locate(const struct vfl *v, uint32_t vpn, uint32_t *ce, uint32_t *page)
{
	const struct nand_geom *g = &v->g;
	uint32_t vbank = vpn % g->banks;

	*ce = vbank % g->chip_enables;
	*page = (vpn % g->stripe_pages) / g->banks;
	return vfl_bank_block(g, vbank, vpn / g->stripe_pages);
}

void vfl_translate(const struct vfl *v, uint32_t vpn, uint32_t *ce, uint32_t *ppage,
		   uint32_t *pblock_before_remap)
{
	uint32_t page, pb = vfl_locate(v, vpn, ce, &page);
	uint16_t s = pb < v->g.ce_blocks ? v->stand_in[*ce][pb] : VFL_GOOD;

	if (pblock_before_remap)
		*pblock_before_remap = pb;
	if (s == VFL_LOST)		/* [V10] */
		LOG(v->ops, "vfl: CE%u block %u is bad and nothing stands in for it\n", *ce, pb);
	else if (s != VFL_GOOD)
		pb = s;
	*ppage = pb * v->g.pages_per_block + page;
}

/* For programming and erasing: [V10a], and nothing outside the CE. */
static int vfl_translate_w(const struct vfl *v, uint32_t vpn, uint32_t *ce, uint32_t *ppage)
{
	uint32_t c, page, pb = vfl_locate(v, vpn, &c, &page);
	uint16_t s;

	if (pb >= v->g.ce_blocks)
		return -1;
	s = v->stand_in[c][pb];
	if (s == VFL_LOST)
		return -1;
	if (s != VFL_GOOD)
		pb = s;
	*ce = c;
	*ppage = pb * v->g.pages_per_block + page;
	return 0;
}

int vfl_read(const struct vfl *v, uint32_t vpn, void *data, struct yaftl_spare *sp)
{
	uint8_t meta[NAND_META_BYTES];
	uint32_t ce, page;
	int ret;

	vfl_translate(v, vpn, &ce, &page, NULL);
	ret = v->ops->read_page(v->ops->ctx, ce, page, data, meta);
	if (sp) {
		if (ret == NAND_BLANK)
			memset(sp, 0xff, sizeof(*sp));
		else
			memcpy(sp, meta, sizeof(*sp));
	}
	return ret;
}

int vfl_write(const struct vfl *v, uint32_t vpn, const void *data,
	      const struct yaftl_spare *sp)
{
	uint32_t ce, page;

	if (vfl_translate_w(v, vpn, &ce, &page))
		return -1;
	return v->ops->write_page(v->ops->ctx, ce, page, data, (const uint8_t *)sp);
}

/* Erase FTL block @blk: its block in every bank, and none unless all can be. */
int vfl_erase(const struct vfl *v, uint32_t blk)
{
	uint32_t ce[2 * VFL_MAX_CE], page[2 * VFL_MAX_CE], k;

	for (k = 0; k < v->g.banks; k++)
		if (vfl_translate_w(v, blk * v->g.stripe_pages + k, &ce[k], &page[k]))
			return -1;
	for (k = 0; k < v->g.banks; k++)
		if (v->ops->erase_block(v->ops->ctx, ce[k], page[k] / v->g.pages_per_block))
			return -1;
	return 0;
}

uint32_t vfl_num_blocks(const struct vfl *v)
{
	return v->ftl_blocks;
}

void vfl_ctrl_blocks(const struct vfl *v, uint16_t out[3])
{
	memcpy(out, v->ftl_ctrl, sizeof(v->ftl_ctrl));
}

/*
 * [V1] Read what should be a context page into the page buffer: NAND_OK
 * (and the counter in its metadata), or why it is not one.
 */
static int vfl_cxt_page(struct vfl *v, uint32_t ce, uint32_t block, uint32_t page,
			uint32_t *counter)
{
	uint8_t meta[NAND_META_BYTES];
	uint32_t version;
	int ret;

	ret = v->ops->read_page(v->ops->ctx, ce, block * v->g.pages_per_block + page, v->pagebuf, meta);
	if (ret != NAND_OK)
		return ret;
	if (meta[8] || meta[9] != VFL_PG_CXT)
		return NAND_EIO;
	version = le32(v->pagebuf + VFL_CXT_VERSION);
	if (version > 2) {
		LOG(v->ops, "vfl: CE%u block %u page %u: context version %u is not known\n",
		    ce, block, page, version);
		return NAND_EIO;
	}
	if (counter)
		*counter = le32(meta);
	return NAND_OK;
}

/* [V2]-[V4] Leave the CE's current context in the page buffer. 0 or -1. */
static int vfl_find_cxt(struct vfl *v, uint32_t ce, uint32_t first_block)
{
	const struct nand_geom *g = &v->g;
	uint32_t end = first_block + 199, blk, own[4], i, page;
	uint32_t counter, lowest = 0xffffffff, current = 0;
	int have = 0;

	if (end > g->ce_blocks)
		end = g->ce_blocks;
	for (blk = first_block; blk < end; blk++)		/* [V2] */
		if (vfl_cxt_page(v, ce, blk, 0, NULL) == NAND_OK)
			break;
	if (blk >= end) {
		LOG(v->ops, "vfl: CE%u: no block with a VFL context\n", ce);
		return -1;
	}
	for (i = 0; i < 4; i++)
		own[i] = le16(v->pagebuf + VFL_CXT_OWN_BLOCKS + 2 * i);

	for (i = 0; i < 4; i++) {				/* [V3] */
		if (own[i] == 0xffff)
			continue;
		if (vfl_cxt_page(v, ce, own[i], 0, &counter) != NAND_OK)
			continue;
		if (counter && counter <= lowest) {
			lowest = counter;
			current = own[i];
			have = 1;
		}
	}
	if (!have) {
		LOG(v->ops, "vfl: CE%u: none of its context blocks reads\n", ce);
		return -1;
	}

	for (page = 0; page + 8 < g->pages_per_block; page += 8)		/* [V4] */
		if (vfl_cxt_page(v, ce, current, page + 8, NULL) != NAND_OK)
			break;
	if (vfl_cxt_page(v, ce, current, page, NULL) != NAND_OK) {
		LOG(v->ops, "vfl: CE%u: its newest context does not read again\n", ce);
		return -1;
	}
	if (!vfl_cxt_intact(v->pagebuf)) {
		LOG(v->ops, "vfl: CE%u: context checksums wrong\n", ce);
		return -1;
	}
	return 0;
}

/* [V9] Work a CE's pool map into its table of stand-ins. 0 or -1. */
static int vfl_load_pool(struct vfl *v, uint32_t ce, const uint8_t *cxt)
{
	uint32_t n = v->g.ce_banks * v->reserved_per_bank, i, e;
	uint16_t *t = v->stand_in[ce];

	/* every block the map calls bad: replaced ones, and bad reserve blocks */
	for (i = 0; i < n; i++) {
		e = le16(cxt + VFL_CXT_POOL_MAP + 2 * i);
		if (e == VFL_POOL_FREE)
			continue;
		if (e < v->g.ce_blocks) {
			t[e] = VFL_LOST;
		} else if (e > VFL_POOL_FREE) {
			t[vfl_pool_block(v, i)] = VFL_LOST;
		} else {
			LOG(v->ops, "vfl: CE%u: pool map entry %u is 0x%x\n", ce, i, e);
			return -1;
		}
	}
	/* the first reserve block listed for a bad block stands in for it */
	for (i = 0; i < n; i++) {
		e = le16(cxt + VFL_CXT_POOL_MAP + 2 * i);
		if (e < v->g.ce_blocks && t[e] == VFL_LOST)
			t[e] = vfl_pool_block(v, i);
	}
	return 0;
}

int vfl_open(struct vfl *v, const struct nand_ops *ops, const struct nand_geom *g,
	     uint32_t first_block)
{
	uint8_t *cxt[VFL_MAX_CE] = { NULL };
	const uint8_t *top;
	int32_t newest = -1;
	uint32_t ce, i;
	int ret = -1;

	memset(v, 0, sizeof(*v));
	v->ops = ops;
	v->g = *g;
	if (g->chip_enables > VFL_MAX_CE || g->ce_blocks >= VFL_LOST)
		return -1;
	v->pagebuf = ops->alloc(g->page_size);
	if (!v->pagebuf)
		return -1;

	for (ce = 0; ce < g->chip_enables; ce++) {
		cxt[ce] = ops->alloc(VFL_CXT_SIZE);
		if (!cxt[ce] || vfl_find_cxt(v, ce, first_block))
			goto out;
		memcpy(cxt[ce], v->pagebuf, VFL_CXT_SIZE);
		if ((int32_t)le32(cxt[ce] + VFL_CXT_GENERATION) >= newest) {	/* [V6] */
			newest = le32(cxt[ce] + VFL_CXT_GENERATION);
			v->latest = ce;
		}
	}

	top = cxt[v->latest];						/* [V7] */
	v->ftl_blocks = le16(top + VFL_CXT_FTL_BLOCKS);
	v->pool_start = le16(top + VFL_CXT_POOL_START);
	for (i = 0; i < 3; i++)
		v->ftl_ctrl[i] = le16(top + VFL_CXT_FTL_CTRL + 2 * i);

	if (le32(top + VFL_CXT_VENDOR) != 0x150011) {			/* [V8] */
		LOG(ops, "vfl: vendor type 0x%x: only 0x150011 is known here\n",
		    le32(top + VFL_CXT_VENDOR));
		goto out;
	}
	v->g.ce_banks = 2;
	v->g.banks = v->g.chip_enables * v->g.ce_banks;
	v->g.stripe_pages = v->g.pages_per_block * v->g.banks;
	v->g.bank_blocks = v->g.ce_blocks / v->g.ce_banks;
	if (v->pool_start > v->g.bank_blocks ||
	    v->g.ce_banks * (v->g.bank_blocks - v->pool_start) > VFL_CXT_POOL_MAP_MAX) {
		LOG(ops, "vfl: a reserve pool from block %u of a bank does not fit\n", v->pool_start);
		goto out;
	}
	v->reserved_per_bank = v->g.bank_blocks - v->pool_start;

	for (ce = 0; ce < g->chip_enables; ce++) {
		v->stand_in[ce] = ops->alloc(g->ce_blocks * sizeof(uint16_t));
		if (!v->stand_in[ce])
			goto out;
		memset(v->stand_in[ce], 0xff, g->ce_blocks * sizeof(uint16_t));
		if (vfl_load_pool(v, ce, cxt[ce]))
			goto out;
	}
	ret = 0;
out:
	for (ce = 0; ce < VFL_MAX_CE; ce++)
		if (cxt[ce])
			ops->free(cxt[ce]);
	return ret;
}

void vfl_close(struct vfl *v)
{
	uint32_t ce;

	for (ce = 0; ce < VFL_MAX_CE; ce++)
		if (v->stand_in[ce])
			v->ops->free(v->stand_in[ce]);
	if (v->pagebuf)
		v->ops->free(v->pagebuf);
	memset(v, 0, sizeof(*v));
}
