// SPDX-License-Identifier: GPL-2.0-only
/*
 * n72-yaftl-core.c - Apple VSVFL + YAFTL (iPod touch 2G / N72).
 *
 * Clean-room GPLv2 implementation of Apple VSVFL + YAFTL (note: not
 * derived from GPLv3 vfl-vsvfl/ftl-yaftl implementations).
 *
 * The on-flash layouts, VSVFL layer, and YAFTL open/restore path follow
 * the exact on-disk rules expected by iOS (control blocks, context slots,
 * block sequence numbers, index-before-user restore order, newest open block
 * stop rule, block status values, and context invalidation markers).
 *
 * The restore scan, writer, index upkeep, and garbage collection use a
 * single-pass full map with sorted arrays and dirty index ranges, without
 * writing context blocks. The VFL is read-only; YAFTL writes user pages,
 * block TOCs, and index pages so the partition recovers cleanly via full
 * restore (see "writing" below).
 */
#ifdef __KERNEL__
#include <linux/math64.h>
#include <linux/string.h>
#define DIV64(a, b) div_u64((a), (b))
#else
#include <string.h>
#define DIV64(a, b) ((a) / (b))
#endif
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

/* ---------------------------------------------------------------- YAFTL --- */

static void yaftl_free_left(struct yaftl *y);
static int yaftl_index_trim(struct yaftl *y);

static uint32_t ceil_div(uint32_t a, uint32_t b)
{
	return (a + b - 1) / b;
}

/*
 * YAFTL open path ([Yn] tags cross-reference the format specification).
 */

/* [Y5] Read a page of the FTL's context: every one carries the context kind. */
static int yaftl_cxt_read(struct yaftl *y, uint32_t vpn, void *buf, struct yaftl_spare *sp)
{
	struct yaftl_spare mine;
	int ret;

	if (!sp)
		sp = &mine;
	ret = vfl_read(y->vfl, vpn, buf, sp);
	if (ret == NAND_OK && !(sp->type & YAFTL_PG_CXT))
		ret = NAND_EIO;
	return ret;
}

/* [Y3] */
static uint32_t yaftl_logical_pages(const struct yaftl *y, uint32_t index_blocks)
{
	return (y->num_blocks - 8) * (y->stripe_pages - y->toc_pages_per_block) -
	       index_blocks * y->stripe_pages;
}

/*
 * [Y5] One table of a context slot: @bytes of it lie in whole pages from
 * *@vpn on. Copied to @dst, or stepped over when there is no @dst.
 */
static int yaftl_slot_table(struct yaftl *y, uint32_t *vpn, void *dst, uint32_t bytes)
{
	uint32_t pages = ceil_div(bytes, y->page_size), n;
	uint8_t *out = dst;

	if (!dst) {
		*vpn += pages;
		return 0;
	}
	for (; pages; pages--, ++*vpn) {
		if (yaftl_cxt_read(y, *vpn, y->pagebuf, NULL) != NAND_OK)
			return -1;
		n = bytes < y->page_size ? bytes : y->page_size;
		memcpy(out, y->pagebuf, n);
		out += n;
		bytes -= n;
	}
	return 0;
}

/* [Y5] Take in the context whose slot starts at @slot. 0, or -1 if it cannot be used. */
static int yaftl_load_cxt(struct yaftl *y, uint32_t slot, uint32_t *offered_pct)
{
	const uint8_t *hdr = y->pagebuf;
	struct yaftl_spare sp;
	uint32_t vpn;

	if (yaftl_cxt_read(y, slot, y->pagebuf, &sp) != NAND_OK)
		return -1;
	if (memcmp(hdr, YAFTL_CXT_MAGIC, 4)) {
		LOG(y->ops, "yaftl: the context does not begin with %s\n", YAFTL_CXT_MAGIC);
		return -1;
	}
	if (le32(hdr + YAFTL_CXT_LOGICAL_PAGES) != y->total_pages ||		/* [Y5a] */
	    !le32(hdr + YAFTL_CXT_INDEX_BLOCKS) ||
	    le16(hdr + YAFTL_CXT_BTOC_PAGES) != y->toc_pages_per_block ||
	    le16(hdr + YAFTL_CXT_RANGE_ENTRIES) != y->toc_entries_per_page ||
	    le16(hdr + YAFTL_CXT_MAP_RANGES) != y->toc_array_len) {
		LOG(y->ops, "yaftl: the context is for another geometry (%u logical pages, here %u)\n",
		    le32(hdr + YAFTL_CXT_LOGICAL_PAGES), y->total_pages);
		return -1;
	}
	*offered_pct = hdr[YAFTL_CXT_OFFERED_PCT];
	y->cxt_usn = sp.usn;

	/* past the header, the two open blocks' tables of contents and the statistics */
	vpn = slot + 1 + 2 * y->toc_pages_per_block + 1;
	if (yaftl_slot_table(y, &vpn, y->toc_index_page, y->toc_array_len * 4) ||
	    yaftl_slot_table(y, &vpn, y->block_status, y->num_blocks) ||
	    yaftl_slot_table(y, &vpn, NULL, y->num_blocks * 2) ||	/* read counts */
	    yaftl_slot_table(y, &vpn, y->erase_count, y->num_blocks * 4))
		return -1;
	return 0;
}

/*
 * [Y2] Which of the three control blocks is current: its index, -2 when
 * none begins with a page that counts, -1 when one lies outside the FTL.
 */
static int yaftl_current_ctrl(struct yaftl *y)
{
	struct yaftl_spare sp;
	uint32_t i, newest = 0;
	int cur = -2;

	for (i = 0; i < 3; i++) {
		if (y->ctrl[i] >= y->num_blocks) {				/* [Y2a] */
			LOG(y->ops, "yaftl: control block %u is not one of the FTL's %u\n",
			    y->ctrl[i], y->num_blocks);
			return -1;
		}
		if (vfl_read(y->vfl, y->ctrl[i] * y->stripe_pages, y->pagebuf, &sp) == NAND_OK &&
		    sp.usn != 0xffffffff && sp.usn > newest) {
			newest = sp.usn;
			cur = i;
		}
	}
	return cur;
}

/* [Y4] Where control block @blk's clean context starts, or YAFTL_VPN_NONE. */
static uint32_t yaftl_clean_slot(struct yaftl *y, uint32_t blk)
{
	uint32_t slot = blk * y->stripe_pages, end = slot + y->stripe_pages;

	for (; slot + y->ctrl_block_page_offset < end; slot += y->ctrl_block_page_offset + 1) {
		if (vfl_read(y->vfl, slot, y->pagebuf, NULL) != NAND_OK)
			break;
		if (vfl_read(y->vfl, slot + y->ctrl_block_page_offset, y->pagebuf, NULL) == NAND_BLANK)
			return slot;
	}
	return YAFTL_VPN_NONE;
}

/* [Y3]-[Y5] Use control block @blk's clean context, if it has one. 0 or -1. */
static int yaftl_use_cxt(struct yaftl *y, uint32_t blk, uint32_t *offered_pct)
{
	uint32_t slot;

	if (yaftl_cxt_read(y, blk * y->stripe_pages, y->pagebuf, NULL) != NAND_OK ||
	    memcmp(y->pagebuf, YAFTL_CXT_MAGIC, 4)) {
		LOG(y->ops, "yaftl: control block %u does not begin with a context\n", blk);
		return -1;
	}
	y->total_pages = yaftl_logical_pages(y, le32(y->pagebuf + YAFTL_CXT_INDEX_BLOCKS));

	slot = yaftl_clean_slot(y, blk);
	if (slot == YAFTL_VPN_NONE) {
		LOG(y->ops, "yaftl: control block %u has no clean context slot (iOS was not unmounted cleanly)\n", blk);
		return -1;
	}
	y->cxt_page = slot;
	if (yaftl_load_cxt(y, slot, offered_pct)) {
		LOG(y->ops, "yaftl: the context at vpn %u cannot be used\n", slot);
		return -1;
	}
	return 0;
}

int yaftl_open(struct yaftl *y, struct vfl *v, int allow_restore)
{
	const struct nand_ops *ops = v->ops;
	uint32_t counts16, counts32, offered_pct = 99;
	int cur;

	memset(y, 0, sizeof(*y));
	y->vfl = v;
	y->ops = ops;
	y->page_size = v->g.page_size;
	y->stripe_pages = v->g.stripe_pages;
	y->num_blocks = vfl_num_blocks(v);
	vfl_ctrl_blocks(v, y->ctrl);

	/* [Y1] */
	y->toc_pages_per_block = ceil_div(y->stripe_pages * 4, y->page_size);
	y->toc_entries_per_page = y->page_size / 4;
	y->toc_array_len = ceil_div(y->stripe_pages * y->num_blocks * 4, y->page_size);
	counts16 = ceil_div(y->num_blocks * 2, y->page_size);
	counts32 = ceil_div(y->num_blocks * 4, y->page_size);
	y->ctrl_block_page_offset = ceil_div(y->toc_array_len * 4, y->page_size) +
		ceil_div(y->num_blocks, y->page_size) + 3 * counts16 + counts32 +
		2 * y->toc_pages_per_block + 2;

	y->pagebuf = ops->alloc(y->page_size);
	y->toc_index_page = ops->alloc(y->toc_array_len * sizeof(uint32_t));
	y->index_cache = ops->alloc(y->toc_array_len * sizeof(uint32_t *));
	y->block_status = ops->alloc(y->num_blocks);
	y->erase_count = ops->alloc(y->num_blocks * sizeof(uint32_t));
	if (!y->pagebuf || !y->toc_index_page || !y->index_cache || !y->block_status ||
	    !y->erase_count)
		return -1;
	memset(y->toc_index_page, 0xff, y->toc_array_len * sizeof(uint32_t));
	y->cxt_page = YAFTL_VPN_NONE;

	if (y->ctrl[0] == 0xffff) {						/* [Y2a] */
		LOG(ops, "yaftl: the VFL names no control blocks\n");
	} else {
		cur = yaftl_current_ctrl(y);
		if (cur == -1)
			return -1;
		if (cur < 0) {
			LOG(ops, "yaftl: no context in control blocks %u %u %u\n",
			    y->ctrl[0], y->ctrl[1], y->ctrl[2]);
		} else {
			y->ctrl_sel = cur;
			if (!yaftl_use_cxt(y, y->ctrl[cur], &offered_pct)) {
				y->clean = 1;
				y->pages_available = (uint32_t)DIV64((uint64_t)offered_pct *
								     (y->total_pages - 1), 100);	/* [Y6] */
				return 0;
			}
		}
	}

	/* no clean context: the map has to come from a scan of the blocks */
	if (!y->total_pages)
		y->total_pages = yaftl_logical_pages(y, 3);
	y->pages_available = (uint32_t)DIV64((uint64_t)99 * (y->total_pages - 1), 100);
	if (!allow_restore)
		return 1;
	return yaftl_restore(y);
}

void yaftl_close(struct yaftl *y)
{
	uint32_t i;

	if (y->index_cache) {
		for (i = 0; i < y->toc_array_len; i++)
			if (y->index_cache[i])
				y->ops->free(y->index_cache[i]);
		y->ops->free(y->index_cache);
	}
	if (y->toc_index_page)
		y->ops->free(y->toc_index_page);
	if (y->l2v)
		y->ops->free(y->l2v);
	if (y->block_status)
		y->ops->free(y->block_status);
	if (y->erase_count)
		y->ops->free(y->erase_count);
	if (y->pagebuf)
		y->ops->free(y->pagebuf);
	yaftl_free_left(y);
	if (y->valid)
		y->ops->free(y->valid);
	if (y->ivalid)
		y->ops->free(y->ivalid);
	if (y->blk_usn)
		y->ops->free(y->blk_usn);
	if (y->blk_flags)
		y->ops->free(y->blk_flags);
	if (y->idx_dirty)
		y->ops->free(y->idx_dirty);
	if (y->idx_stamp)
		y->ops->free(y->idx_stamp);
	if (y->ublk.btoc)
		y->ops->free(y->ublk.btoc);
	if (y->iblk.btoc)
		y->ops->free(y->iblk.btoc);
	if (y->gc_btoc)
		y->ops->free(y->gc_btoc);
	if (y->gcbuf)
		y->ops->free(y->gcbuf);
	if (y->idxbuf)
		y->ops->free(y->idxbuf);
	memset(y, 0, sizeof(*y));
}

/* [Y7] The index page of map range @range, read once and kept; NULL if it does not read. */
static const uint32_t *yaftl_range(struct yaftl *y, uint32_t range)
{
	uint32_t *ip = y->index_cache[range];
	struct yaftl_spare sp;
	int ret;

	if (ip)
		return ip;
	ip = y->ops->alloc(y->page_size);
	if (!ip)
		return NULL;
	ret = vfl_read(y->vfl, y->toc_index_page[range], ip, &sp);
	if (ret != NAND_OK || !(sp.type & YAFTL_PG_INDEX) || sp.lpn != range) {
		LOG(y->ops, "yaftl: the index page of range %u at vpn %u does not read (%d, kind 0x%02x, range %u)\n",
		    range, y->toc_index_page[range], ret, sp.type, sp.lpn);
		y->ops->free(ip);
		return NULL;
	}
	y->index_cache[range] = ip;
	return ip;
}

int yaftl_map(struct yaftl *y, uint32_t lpn, uint32_t *vpn)
{
	uint32_t range = lpn / y->toc_entries_per_page;
	const uint32_t *ip;

	if (lpn >= y->total_pages)
		return -1;
	if (y->l2v) {
		*vpn = y->l2v[lpn];
		return 0;
	}
	if (y->toc_index_page[range] == YAFTL_VPN_NONE) {	/* nothing written in this range */
		*vpn = YAFTL_VPN_NONE;
		return 0;
	}
	ip = yaftl_range(y, range);
	if (!ip)
		return -1;
	*vpn = ip[lpn % y->toc_entries_per_page];
	return 0;
}

int yaftl_read(struct yaftl *y, uint32_t lpn, void *buf)
{
	struct yaftl_spare sp;
	uint32_t vpn;
	int ret;

	if (yaftl_map(y, lpn, &vpn) < 0)
		return -1;
	if (vpn >= y->num_blocks * y->stripe_pages) {	/* [Y7][Y7a] never written, or nowhere */
		memset(buf, 0, y->page_size);
		return NAND_BLANK;
	}
	ret = vfl_read(y->vfl, vpn, buf, &sp);
	if (ret > 0)
		ret = NAND_EIO;		/* a blank page where the map says there is one */
	if (ret)
		return ret;
	if (sp.lpn != lpn || (sp.type & (YAFTL_PG_USER | YAFTL_PG_ERROR)) != YAFTL_PG_USER) {
		LOG(y->ops, "yaftl: lpn %u at vpn %u: its metadata says lpn %u, kind 0x%02x\n",
		    lpn, vpn, sp.lpn, sp.type);
		return NAND_EIO;
	}
	return NAND_OK;
}

/* --- restore: rebuild everything from what the blocks hold ---------------- */

/*
 * Full NAND restore path (when no valid context slot is present): rebuilds
 * the FTL translation table using the exact on-flash ordering and stop rules
 * expected by the stock partition format:
 *
 *  - a block is what its first readable page says it is;
 *  - user blocks are taken newest first (usn of that page), pages last to
 *    first, and the first copy of a logical page seen is the current one;
 *  - a closed block is described by its block TOC alone;
 *  - of the newest user block without a TOC, only the pages before the
 *    first one that is not a user page count;
 *  - index blocks the same way, for the index pages.
 */

struct scan_blk {
	uint32_t blk;
	uint32_t usn;
};

static void sort_desc(struct scan_blk *a, uint32_t n)
{
	uint32_t i, j;

	for (i = 1; i < n; i++) {		/* insertion sort; n <= 2000 */
		struct scan_blk t = a[i];

		for (j = i; j > 0 && a[j - 1].usn < t.usn; j--)
			a[j] = a[j - 1];
		a[j] = t;
	}
}

static int yaftl_is_ctrl(const struct yaftl *y, uint32_t b)
{
	return b == y->ctrl[0] || b == y->ctrl[1] || b == y->ctrl[2];
}

static void yaftl_free_left(struct yaftl *y)
{
	uint32_t i;

	for (i = 0; i < y->n_left; i++)
		y->ops->free(y->left[i].btoc);
	y->n_left = 0;
}

static void yaftl_see_usn(struct yaftl *y, const struct yaftl_spare *sp)
{
	if (sp->usn != 0xffffffff && sp->usn > y->max_usn)
		y->max_usn = sp->usn;
}

/* Remember a block found without its TOC, so that it can be closed. */
static void yaftl_keep_left(struct yaftl *y, uint32_t blk, const uint32_t *list)
{
	uint32_t bytes = y->page_size * y->toc_pages_per_block;
	uint32_t *copy;

	if (y->n_left == YAFTL_MAX_LEFT)
		return;				/* stays open: still restorable */
	copy = y->ops->alloc(bytes);
	if (!copy)
		return;
	memset(copy, 0xff, bytes);
	memcpy(copy, list, (y->stripe_pages - y->toc_pages_per_block) * sizeof(uint32_t));
	y->left[y->n_left].blk = blk;
	y->left[y->n_left].btoc = copy;
	y->n_left++;
}

/*
 * Read a block's TOC into @btoc. 1: closed (a TOC of the wanted kind),
 * 0: not closed and the TOC page is blank, -1: not closed, page not blank.
 */
static int yaftl_read_btoc(struct yaftl *y, uint32_t blk, int index, uint32_t *btoc)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t base = blk * y->stripe_pages, p;
	struct yaftl_spare sp;
	int blank = 1, closed = 1, ret;

	for (p = 0; p < y->toc_pages_per_block; p++) {
		ret = vfl_read(y->vfl, base + data_pages + p,
			       (uint8_t *)btoc + p * y->page_size, &sp);
		if (ret != NAND_BLANK)
			blank = 0;
		if (ret != NAND_OK || !(sp.type & YAFTL_PG_BTOC) ||
		    !(sp.type & YAFTL_PG_INDEX) != !index)
			closed = 0;
		else
			yaftl_see_usn(y, &sp);
	}
	if (closed)
		return 1;
	return blank ? 0 : -1;
}

static void yaftl_scan_user(struct yaftl *y, uint32_t blk, int first, uint32_t *btoc,
			    uint32_t *list)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t base = blk * y->stripe_pages, p, used = data_pages;
	struct yaftl_spare sp;
	int ret, closed;

	closed = yaftl_read_btoc(y, blk, 0, btoc);
	if (closed == 1) {
		y->blk_flags[blk] |= YBF_CLOSED;
		list = btoc;
	} else {
		for (p = 0; p < data_pages; p++) {
			ret = vfl_read(y->vfl, base + p, y->pagebuf, &sp);
			if (ret == NAND_OK && (sp.type & YAFTL_PG_USER) && sp.lpn < y->total_pages) {
				list[p] = sp.lpn;
				yaftl_see_usn(y, &sp);
				continue;
			}
			list[p] = YAFTL_VPN_NONE;
			if (first) {		/* nothing after it counts */
				used = p;
				break;
			}
		}
		for (p = used; p < data_pages; p++)
			list[p] = YAFTL_VPN_NONE;
		if (closed == 0)
			yaftl_keep_left(y, blk, list);
	}
	for (p = data_pages; p-- > 0;) {
		uint32_t lpn = list[p];

		if (lpn < y->total_pages && y->l2v[lpn] == YAFTL_VPN_NONE)
			y->l2v[lpn] = base + p;
	}
}

static void yaftl_scan_index(struct yaftl *y, uint32_t blk, int newest, uint32_t *btoc,
			     uint32_t *list)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t base = blk * y->stripe_pages, p, used = 0;
	struct yaftl_spare sp;
	int ret, closed;

	closed = yaftl_read_btoc(y, blk, 1, btoc);
	if (closed == 1) {
		y->blk_flags[blk] |= YBF_CLOSED;
		list = btoc;
	} else {
		for (p = 0; p < data_pages; p++) {
			ret = vfl_read(y->vfl, base + p, y->pagebuf, &sp);
			if (ret != NAND_BLANK)
				used = p + 1;
			if (ret == NAND_OK && (sp.type & YAFTL_PG_INDEX) &&
			    sp.lpn < y->toc_array_len) {
				list[p] = sp.lpn;
				yaftl_see_usn(y, &sp);
			} else {
				list[p] = YAFTL_VPN_NONE;
			}
		}
		if (closed == 0 && newest) {
			/*
			 * The open index block is written on: unlike a user
			 * block it may have holes (iOS reads every page of
			 * it), so the page after the last one, which a power
			 * cut may have left half programmed, is skipped.
			 */
			y->iblk.blk = blk;
			y->iblk.used = used + 1 < data_pages ? used + 1 : data_pages;
			memset(y->iblk.btoc, 0xff, y->page_size * y->toc_pages_per_block);
			memcpy(y->iblk.btoc, list, data_pages * sizeof(uint32_t));
		} else if (closed == 0) {
			yaftl_keep_left(y, blk, list);
		}
	}
	for (p = data_pages; p-- > 0;) {
		uint32_t toc = list[p];

		if (toc < y->toc_array_len && y->toc_index_page[toc] == YAFTL_VPN_NONE) {
			y->toc_index_page[toc] = base + p;
			y->ivalid[blk]++;
		}
	}
}

int yaftl_restore(struct yaftl *y)
{
	const struct nand_ops *ops = y->ops;
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t toc_bytes = y->page_size * y->toc_pages_per_block;
	struct scan_blk *users, *idx;
	uint32_t *btoc, *list;
	uint32_t b, p, n_users = 0, n_idx = 0;
	struct yaftl_spare sp;
	int ret = -1;

	LOG(ops, "yaftl: rebuilding the logical map by scanning %u blocks\n", y->num_blocks);
	if (y->prepared) {
		LOG(ops, "yaftl: no rescan once writing has begun\n");
		return -1;
	}
	if (!y->l2v)
		y->l2v = ops->alloc(y->total_pages * sizeof(uint32_t));
	if (!y->valid) {
		y->valid = ops->alloc(y->num_blocks * sizeof(uint16_t));
		y->ivalid = ops->alloc(y->num_blocks * sizeof(uint16_t));
		y->blk_usn = ops->alloc(y->num_blocks * sizeof(uint32_t));
		y->blk_flags = ops->alloc(y->num_blocks);
		y->idx_dirty = ops->alloc(y->toc_array_len);
		y->idx_stamp = ops->alloc(y->toc_array_len * sizeof(uint32_t));
		y->ublk.btoc = ops->alloc(toc_bytes);
		y->iblk.btoc = ops->alloc(toc_bytes);
		y->gc_btoc = ops->alloc(toc_bytes);
		y->gcbuf = ops->alloc(y->page_size);
		y->idxbuf = ops->alloc(y->page_size);
	}
	users = ops->alloc(y->num_blocks * sizeof(*users));
	idx = ops->alloc(y->num_blocks * sizeof(*idx));
	btoc = ops->alloc(toc_bytes);
	list = ops->alloc(toc_bytes);
	if (!y->l2v || !y->valid || !y->ivalid || !y->blk_usn || !y->blk_flags ||
	    !y->idx_dirty || !y->idx_stamp || !y->ublk.btoc || !y->iblk.btoc ||
	    !y->gc_btoc || !y->gcbuf || !y->idxbuf || !users || !idx || !btoc || !list)
		goto out;

	memset(y->l2v, 0xff, y->total_pages * sizeof(uint32_t));
	memset(y->toc_index_page, 0xff, y->toc_array_len * sizeof(uint32_t));
	memset(y->block_status, 0, y->num_blocks);
	memset(y->valid, 0, y->num_blocks * sizeof(uint16_t));
	memset(y->ivalid, 0, y->num_blocks * sizeof(uint16_t));
	memset(y->blk_usn, 0, y->num_blocks * sizeof(uint32_t));
	memset(y->blk_flags, 0, y->num_blocks);
	memset(y->idx_dirty, 0, y->toc_array_len);
	yaftl_free_left(y);
	y->index_blocks = y->user_blocks = y->free_blocks = 0;
	y->max_usn = y->cxt_usn;
	y->ublk.blk = y->iblk.blk = YAFTL_VPN_NONE;
	y->last_written = YAFTL_VPN_NONE;
	y->n_dirty = 0;
	y->idx_dirty_max = 7;		/* see "writing" below */
	y->idx_dirty_burst = 7;
	y->gc_low = 12;
	y->min_free = 6;

	for (b = 0; b < y->num_blocks; b++) {
		uint32_t base = b * y->stripe_pages;

		if (yaftl_is_ctrl(y, b)) {
			y->block_status[b] = YBS_CTRL;
			if (vfl_read(y->vfl, base, y->pagebuf, &sp) == NAND_OK)
				yaftl_see_usn(y, &sp);
			continue;
		}
		/*
		 * The first page that reads decides. A user or index page
		 * with an impossible number counts as one that does not.
		 */
		for (p = 0; p < data_pages; p++) {
			ret = vfl_read(y->vfl, base + p, y->pagebuf, &sp);
			if (ret == NAND_BLANK)
				break;
			if (ret != NAND_OK)
				continue;
			if ((sp.type & YAFTL_PG_INDEX) && sp.lpn >= y->toc_array_len)
				continue;
			if ((sp.type & YAFTL_PG_USER) && sp.lpn >= y->total_pages)
				continue;
			break;
		}
		if (p == data_pages) {
			y->block_status[b] = YBS_BAD;	/* iOS will erase it */
			continue;
		}
		if (ret == NAND_BLANK) {
			y->block_status[b] = YBS_FREE;
			y->free_blocks++;
			continue;
		}
		yaftl_see_usn(y, &sp);
		y->blk_usn[b] = sp.usn;
		if (sp.type & YAFTL_PG_INDEX) {
			y->block_status[b] = YBS_INDEX;
			idx[n_idx].blk = b;
			idx[n_idx++].usn = sp.usn;
		} else if (sp.type & YAFTL_PG_USER) {
			y->block_status[b] = YBS_USER;
			users[n_users].blk = b;
			users[n_users++].usn = sp.usn;
		} else {
			y->block_status[b] = YBS_BAD;	/* iOS will erase it */
			LOG(ops, "yaftl: block %u has unknown page type 0x%02x\n", b, sp.type);
		}
	}
	y->user_blocks = n_users;
	y->index_blocks = n_idx;
	sort_desc(users, n_users);		/* newest first */
	sort_desc(idx, n_idx);

	for (b = 0; b < n_idx; b++)
		yaftl_scan_index(y, idx[b].blk, b == 0, btoc, list);
	for (b = 0; b < n_users; b++)
		yaftl_scan_user(y, users[b].blk, b == 0, btoc, list);
	for (p = 0; p < y->total_pages; p++)
		if (y->l2v[p] != YAFTL_VPN_NONE)
			y->valid[y->l2v[p] / y->stripe_pages]++;
	y->clean = 0;
	ret = 0;
out:
	if (users)
		ops->free(users);
	if (idx)
		ops->free(idx);
	if (btoc)
		ops->free(btoc);
	if (list)
		ops->free(list);
	return ret;
}

/* --- writing ----------------------------------------------------------------
 *
 * What may be on the flash at any moment, so that a power cut costs only
 * the page being written and iOS's restore (above) rebuilds this very map:
 *
 *  - Every user block but the newest ends with its TOC. The newest is
 *    filled front to back with nothing but good user pages. So a block
 *    left open (by iOS or by Linux) is first closed and never written
 *    again: the page after its last one may be half programmed.
 *  - No two blocks begin with the same usn (iOS asserts it).
 *  - The index pages on the flash differ from the map for at most
 *    idx_dirty_max ranges of logical pages (or as many as were found
 *    stale, until the first write). iOS's restore repairs such ranges in its index
 *    caches. It has ten in normal use, so ten is what an iOS crash can
 *    leave behind and what its restore is known to cope with; for the
 *    restore itself YAFTL_Open makes room for every range (totalPages /
 *    tocEntriesPerPage + 1 caches), so a larger limit should be fine
 *    too, but that is untried.
 *  - At most three index blocks (iOS's own budget, numIBlocks).
 *  - No clean context: a context iOS left is spoiled first, as iOS does
 *    when it mounts.
 *
 * Nothing here writes the VFL. A program that fails stops all writing.
 */

static int yaftl_fail(struct yaftl *y, const char *what, uint32_t arg)
{
	LOG(y->ops, "yaftl: %s %u failed: no more writes\n", what, arg);
	y->failed = 1;
	return YAFTL_EIO;
}

static void yaftl_spare(struct yaftl_spare *sp, uint32_t lpn, uint32_t usn, uint8_t type)
{
	sp->lpn = lpn;
	sp->usn = usn;
	sp->zero = 0;
	sp->type = type;
	sp->ones = 0xffff;
}

/* iOS counts the usn up whenever it turns to another block. */
static uint32_t yaftl_page_usn(struct yaftl *y, uint32_t blk)
{
	if (y->last_written != blk) {
		y->last_written = blk;
		y->max_usn++;
	}
	return y->max_usn;
}

static int yaftl_write_btoc(struct yaftl *y, uint32_t blk, const uint32_t *btoc, int index)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t base = blk * y->stripe_pages, i;
	struct yaftl_spare sp;

	yaftl_spare(&sp, 0xffffffff, y->blk_usn[blk],
		    index ? YAFTL_PG_BTOC | YAFTL_PG_INDEX : YAFTL_PG_BTOC);
	for (i = 0; i < y->toc_pages_per_block; i++)
		if (vfl_write(y->vfl, base + data_pages + i,
			      (const uint8_t *)btoc + i * y->page_size, &sp))
			return yaftl_fail(y, "block TOC of block", blk);
	y->blk_flags[blk] |= YBF_CLOSED;
	y->st_closed++;
	return 0;
}

/*
 * Which index pages on the flash differ from the map (iOS's restore makes
 * the same comparison). A range with nothing mapped needs no index page at
 * all. With @apply the result becomes the stale set; without, nothing is
 * changed and the ranges are only counted.
 */
uint32_t yaftl_index_compare(struct yaftl *y, int apply)
{
	uint32_t t, i, per = y->toc_entries_per_page, stale = 0;
	struct yaftl_spare sp;

	if (!y->l2v)
		return 0;
	for (t = 0; t < y->toc_array_len; t++) {
		uint32_t first = t * per, n = per, vpn = y->toc_index_page[t];
		const uint32_t *page = (const uint32_t *)y->pagebuf;
		int same = 1;

		if (first >= y->total_pages)
			n = 0;			/* the table is longer than the disk */
		else if (first + n > y->total_pages)
			n = y->total_pages - first;
		for (i = 0; i < n; i++)
			if (y->l2v[first + i] != YAFTL_VPN_NONE)
				break;
		if (i == n) {
			if (apply && vpn != YAFTL_VPN_NONE) {
				y->ivalid[vpn / y->stripe_pages]--;
				y->toc_index_page[t] = YAFTL_VPN_NONE;
			}
			continue;
		}
		if (vpn == YAFTL_VPN_NONE ||
		    vfl_read(y->vfl, vpn, y->pagebuf, &sp) != NAND_OK) {
			same = 0;
		} else {
			for (i = 0; i < per && same; i++)
				same = page[i] == (i < n ? y->l2v[first + i] : YAFTL_VPN_NONE);
		}
		if (same)
			continue;
		stale++;
		if (apply) {
			y->idx_dirty[t] = 1;
			y->idx_stamp[t] = ++y->stamp;
			y->n_dirty++;
		}
	}
	return stale;
}

/*
 * For looking into the ranges that differ, nothing is changed. out[0]:
 * entries the flash index has unmapped and the map has; [1]: the reverse;
 * [2]: both mapped, the map's page in a newer block (the index is merely
 * behind); [3]: both mapped, the map's page in an OLDER block (the scan
 * and iOS disagree); [4]: same block, other page; [5]: ranges without an
 * index page on the flash.
 */
void yaftl_index_diff(struct yaftl *y, uint32_t out[6])
{
	uint32_t t, i, per = y->toc_entries_per_page;
	struct yaftl_spare sp;

	memset(out, 0, 6 * sizeof(uint32_t));
	if (!y->l2v)
		return;
	for (t = 0; t < y->toc_array_len; t++) {
		uint32_t first = t * per, n = per, vpn = y->toc_index_page[t];
		const uint32_t *page = (const uint32_t *)y->pagebuf;
		int have;

		if (first >= y->total_pages)
			continue;
		if (first + n > y->total_pages)
			n = y->total_pages - first;
		have = vpn != YAFTL_VPN_NONE && vfl_read(y->vfl, vpn, y->pagebuf, &sp) == NAND_OK;
		for (i = 0; i < n; i++) {
			uint32_t m = y->l2v[first + i], f = have ? page[i] : YAFTL_VPN_NONE;

			if (m == f)
				continue;
			if (!have) {
				out[5]++;
				break;
			}
			if (f == YAFTL_VPN_NONE)
				out[0]++;
			else if (m == YAFTL_VPN_NONE)
				out[1]++;
			else if (m / y->stripe_pages == f / y->stripe_pages)
				out[4]++;
			else if (f / y->stripe_pages < y->num_blocks &&
				 y->blk_usn[m / y->stripe_pages] >
				 y->blk_usn[f / y->stripe_pages])
				out[2]++;
			else
				out[3]++;
		}
	}
}

int yaftl_prepare_write(struct yaftl *y)
{
	const struct nand_ops *ops = y->ops;
	uint32_t i;
	struct yaftl_spare sp;
	int ret;

	if (y->failed)
		return YAFTL_EROFS;
	if (y->prepared)
		return 0;
	if (!y->l2v || !y->valid || !ops->write_page || !ops->erase_block)
		return YAFTL_EROFS;
	if (y->index_blocks > 3) {
		LOG(ops, "yaftl: %u index blocks: not writing to this\n", y->index_blocks);
		return YAFTL_EROFS;
	}

	/*
	 * A context whose next page is blank is one iOS would trust
	 * (YAFTL_Open). iOS spoils it when it mounts, by writing that page;
	 * so does this, before anything else changes.
	 */
	if (y->cxt_page != YAFTL_VPN_NONE) {
		uint32_t vpn = y->cxt_page + y->ctrl_block_page_offset;

		memset(y->pagebuf, 0, y->page_size);
		y->max_usn++;
		yaftl_spare(&sp, 0, y->max_usn, YAFTL_PG_CXT);
		if (vfl_write(y->vfl, vpn, y->pagebuf, &sp))
			return yaftl_fail(y, "spoiling the context at page", vpn);
		if (vfl_read(y->vfl, vpn, y->pagebuf, &sp) == NAND_BLANK)
			return yaftl_fail(y, "the context is still clean after page", vpn);
		y->cxt_page = YAFTL_VPN_NONE;
	}

	/* Close what was left open: those blocks are not written again. */
	for (i = 0; i < y->n_left; i++) {
		uint32_t blk = y->left[i].blk;

		ret = yaftl_write_btoc(y, blk, y->left[i].btoc,
				       y->block_status[blk] == YBS_INDEX);
		if (ret)
			return ret;
	}
	yaftl_free_left(y);

	yaftl_index_compare(y, 1);
	LOG(ops, "yaftl: writing enabled: %u free blocks, %u stale index pages, usn %#x\n",
	    y->free_blocks, y->n_dirty, y->max_usn);
	y->prepared = 1;
	return yaftl_index_trim(y);	/* a move that a power cut interrupted */
}

static int yaftl_alloc_block(struct yaftl *y, uint8_t status, uint32_t *out)
{
	uint32_t i, b;

	for (i = 0; i < y->num_blocks; i++) {
		b = (y->alloc_cursor + 1 + i) % y->num_blocks;
		if (y->block_status[b] != YBS_FREE || yaftl_is_ctrl(y, b))
			continue;
		/* a block that only looks blank is erased before use, as on iOS */
		if (!(y->blk_flags[b] & YBF_ERASED)) {
			if (vfl_erase(y->vfl, b)) {
				LOG(y->ops, "yaftl: block %u does not erase: left alone\n", b);
				y->block_status[b] = YBS_BAD;
				y->free_blocks--;
				continue;
			}
			y->st_erases++;
		}
		y->alloc_cursor = b;
		y->block_status[b] = status;
		y->blk_flags[b] = 0;
		y->blk_usn[b] = 0;
		y->valid[b] = y->ivalid[b] = 0;
		y->free_blocks--;
		*out = b;
		return 0;
	}
	return YAFTL_ENOSPC;
}

/* A block with nothing current in it goes back to the free ones, erased. */
static int yaftl_release_block(struct yaftl *y, uint32_t blk)
{
	if (y->block_status[blk] == YBS_INDEX)
		y->index_blocks--;
	else
		y->user_blocks--;
	if (vfl_erase(y->vfl, blk)) {
		LOG(y->ops, "yaftl: block %u does not erase: left alone\n", blk);
		y->block_status[blk] = YBS_BAD;
		return 1;
	}
	y->st_erases++;
	y->block_status[blk] = YBS_FREE;
	y->blk_flags[blk] = YBF_ERASED;
	y->free_blocks++;
	return 0;
}

static int yaftl_index_write(struct yaftl *y, uint32_t t);
static int yaftl_index_trim(struct yaftl *y);

/* An index block none of whose pages is current any more. */
static void yaftl_index_reap(struct yaftl *y, uint32_t blk)
{
	if (blk != y->iblk.blk && y->block_status[blk] == YBS_INDEX && !y->ivalid[blk])
		yaftl_release_block(y, blk);
}

static int yaftl_index_new_block(struct yaftl *y)
{
	uint32_t toc_bytes = y->page_size * y->toc_pages_per_block;
	uint32_t b;
	int ret;

	for (b = 0; b < y->num_blocks && y->index_blocks >= 3; b++)
		yaftl_index_reap(y, b);
	if (y->index_blocks >= 3)
		return yaftl_fail(y, "a fourth index block: have", y->index_blocks);
	ret = yaftl_alloc_block(y, YBS_INDEX, &b);
	if (ret)
		return ret;
	y->index_blocks++;
	y->iblk.blk = b;
	y->iblk.used = 0;
	memset(y->iblk.btoc, 0xff, toc_bytes);
	return yaftl_index_trim(y);
}

/*
 * Two index blocks at most stay: what the oldest still holds moves to the
 * open one (which was new when this began, and has more room than the old
 * one has current pages), then the old one is erased.
 */
static int yaftl_index_trim(struct yaftl *y)
{
	uint32_t b, t;
	int ret;

	while (y->index_blocks > 2 && y->iblk.blk != YAFTL_VPN_NONE) {
		uint32_t old = YAFTL_VPN_NONE;

		for (b = 0; b < y->num_blocks; b++)
			if (y->block_status[b] == YBS_INDEX && b != y->iblk.blk &&
			    (old == YAFTL_VPN_NONE || y->blk_usn[b] < y->blk_usn[old]))
				old = b;
		if (old == YAFTL_VPN_NONE)
			break;
		for (t = 0; t < y->toc_array_len && y->ivalid[old]; t++) {
			if (y->toc_index_page[t] == YAFTL_VPN_NONE ||
			    y->toc_index_page[t] / y->stripe_pages != old)
				continue;
			ret = yaftl_index_write(y, t);
			if (ret)
				return ret;
		}
		if (y->block_status[old] == YBS_INDEX) {
			if (y->ivalid[old])
				return yaftl_fail(y, "emptying index block", old);
			yaftl_index_reap(y, old);
			if (y->block_status[old] == YBS_INDEX)
				break;
		}
	}
	return 0;
}

/* Write the index page of range @t from the map. */
static int yaftl_index_write(struct yaftl *y, uint32_t t)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t per = y->toc_entries_per_page, first = t * per, n = per;
	struct yaftl_open_blk *ob = &y->iblk;
	uint32_t vpn, old;
	struct yaftl_spare sp;
	int ret;

	if (ob->blk != YAFTL_VPN_NONE && ob->used >= data_pages) {
		ret = yaftl_write_btoc(y, ob->blk, ob->btoc, 1);
		if (ret)
			return ret;
		ob->blk = YAFTL_VPN_NONE;
	}
	if (ob->blk == YAFTL_VPN_NONE) {
		ret = yaftl_index_new_block(y);
		if (ret)
			return ret;
		/* moving the oldest block's pages may have written this one */
		if (!y->idx_dirty[t] && y->toc_index_page[t] != YAFTL_VPN_NONE &&
		    y->toc_index_page[t] / y->stripe_pages == ob->blk)
			return 0;
	}
	if (first >= y->total_pages)
		n = 0;
	else if (first + n > y->total_pages)
		n = y->total_pages - first;
	memset(y->idxbuf, 0xff, y->page_size);
	memcpy(y->idxbuf, &y->l2v[first], n * sizeof(uint32_t));

	vpn = ob->blk * y->stripe_pages + ob->used;
	yaftl_spare(&sp, t, yaftl_page_usn(y, ob->blk), YAFTL_PG_INDEX);
	if (!ob->used)
		y->blk_usn[ob->blk] = sp.usn;
	if (vfl_write(y->vfl, vpn, y->idxbuf, &sp))
		return yaftl_fail(y, "index page at", vpn);
	ob->btoc[ob->used++] = t;
	y->st_index++;
	old = y->toc_index_page[t];
	y->toc_index_page[t] = vpn;
	y->ivalid[ob->blk]++;
	if (y->idx_dirty[t]) {
		y->idx_dirty[t] = 0;
		y->n_dirty--;
	}
	if (old != YAFTL_VPN_NONE) {
		uint32_t ob2 = old / y->stripe_pages;

		if (y->ivalid[ob2])
			y->ivalid[ob2]--;
		yaftl_index_reap(y, ob2);
	}
	return 0;
}

/* Write index pages, longest stale first, until at most @limit are stale. */
static int yaftl_index_limit(struct yaftl *y, uint32_t limit)
{
	while (y->n_dirty > limit) {
		uint32_t t, best = YAFTL_VPN_NONE;
		int ret;

		for (t = 0; t < y->toc_array_len; t++)
			if (y->idx_dirty[t] &&
			    (best == YAFTL_VPN_NONE || y->idx_stamp[t] < y->idx_stamp[best]))
				best = t;
		if (best == YAFTL_VPN_NONE) {
			y->n_dirty = 0;
			break;
		}
		ret = yaftl_index_write(y, best);
		if (ret)
			return ret;
	}
	return 0;
}

static int yaftl_gc(struct yaftl *y);

/* One user page at the end of the open block. */
static int yaftl_append(struct yaftl *y, uint32_t lpn, const void *data, int in_gc)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t toc_bytes = y->page_size * y->toc_pages_per_block;
	struct yaftl_open_blk *ob = &y->ublk;
	uint32_t vpn, old, t;
	struct yaftl_spare sp;
	int ret, tries;

	for (tries = 0; ob->blk == YAFTL_VPN_NONE || ob->used >= data_pages; tries++) {
		uint32_t b;

		if (tries == 3)
			return yaftl_fail(y, "finding room, block", ob->blk);
		if (ob->blk != YAFTL_VPN_NONE) {
			ret = yaftl_write_btoc(y, ob->blk, ob->btoc, 0);
			if (ret)
				return ret;
			ob->blk = YAFTL_VPN_NONE;
		}
		if (!in_gc) {
			ret = yaftl_gc(y);	/* may open a block itself */
			if (ret)
				return ret;
			if (ob->blk != YAFTL_VPN_NONE)
				continue;
			if (y->free_blocks < y->min_free)
				return YAFTL_ENOSPC;
		}
		ret = yaftl_alloc_block(y, YBS_USER, &b);
		if (ret)
			return ret;
		y->user_blocks++;
		ob->blk = b;
		ob->used = 0;
		memset(ob->btoc, 0xff, toc_bytes);
	}

	/*
	 * Room among the stale index pages first: once this page is on the
	 * flash its range is stale, whatever happens next.
	 */
	t = lpn / y->toc_entries_per_page;
	if (!y->idx_dirty[t] && y->n_dirty >= y->idx_dirty_burst) {
		ret = yaftl_index_limit(y, y->idx_dirty_burst ? y->idx_dirty_burst - 1 : 0);
		if (ret)
			return ret;
	}

	vpn = ob->blk * y->stripe_pages + ob->used;
	yaftl_spare(&sp, lpn, yaftl_page_usn(y, ob->blk), YAFTL_PG_USER);
	if (!ob->used)
		y->blk_usn[ob->blk] = sp.usn;
	if (vfl_write(y->vfl, vpn, data, &sp))
		return yaftl_fail(y, "user page at", vpn);
	ob->btoc[ob->used++] = lpn;
	old = y->l2v[lpn];
	if (old != YAFTL_VPN_NONE && y->valid[old / y->stripe_pages])
		y->valid[old / y->stripe_pages]--;
	y->l2v[lpn] = vpn;
	y->valid[ob->blk]++;

	if (!y->idx_dirty[t]) {
		y->idx_dirty[t] = 1;
		y->n_dirty++;
	}
	y->idx_stamp[t] = ++y->stamp;
	return 0;
}

/*
 * Move what is still current out of user block @blk and erase it.
 * 0: freed, 1: left alone (a page did not read), <0: writing failed.
 */
static int yaftl_gc_block(struct yaftl *y, uint32_t blk)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;
	uint32_t base = blk * y->stripe_pages, p, n;
	struct yaftl_spare sp;
	int ret;

	if (y->valid[blk]) {
		if (y->blk_flags[blk] & YBF_CLOSED) {
			if (yaftl_read_btoc(y, blk, 0, y->gc_btoc) != 1)
				goto leave;
		} else {
			for (p = 0; p < data_pages; p++) {
				ret = vfl_read(y->vfl, base + p, y->gcbuf, &sp);
				y->gc_btoc[p] = ret == NAND_OK && (sp.type & YAFTL_PG_USER) ?
						sp.lpn : YAFTL_VPN_NONE;
			}
		}
		/*
		 * In the order of their logical pages, so that those of one
		 * index range go together: gc_btoc[] becomes the list of
		 * pages to move, sorted by what they hold.
		 */
		n = 0;
		for (p = 0; p < data_pages; p++) {
			uint32_t lpn = y->gc_btoc[p], k;

			if (lpn >= y->total_pages || y->l2v[lpn] != base + p)
				continue;
			for (k = n++; k > 0 && y->gc_btoc[k - 1] > lpn; k--)
				y->gc_btoc[k] = y->gc_btoc[k - 1];
			y->gc_btoc[k] = lpn;
		}
		for (p = 0; p < n; p++) {
			uint32_t lpn = y->gc_btoc[p], vpn = y->l2v[lpn];

			ret = vfl_read(y->vfl, vpn, y->gcbuf, &sp);
			if (ret != NAND_OK || sp.lpn != lpn || !(sp.type & YAFTL_PG_USER))
				goto leave;
			ret = yaftl_append(y, lpn, y->gcbuf, 1);
			if (ret)
				return ret;
			y->st_gc_pages++;
		}
		if (y->valid[blk])
			goto leave;
	}
	y->st_gc_blocks++;
	return yaftl_release_block(y, blk);
leave:
	LOG(y->ops, "yaftl: block %u cannot be collected (%u pages current)\n", blk, y->valid[blk]);
	y->blk_flags[blk] |= YBF_NOGC;
	return 1;
}

/* Free blocks, the emptiest first, while fewer than gc_low are free. */
static int yaftl_gc(struct yaftl *y)
{
	uint32_t data_pages = y->stripe_pages - y->toc_pages_per_block;

	while (y->free_blocks < y->gc_low) {
		uint32_t b, best = YAFTL_VPN_NONE;
		int ret;

		for (b = 0; b < y->num_blocks; b++)
			if (y->block_status[b] == YBS_USER && b != y->ublk.blk &&
			    !(y->blk_flags[b] & YBF_NOGC) &&
			    (best == YAFTL_VPN_NONE || y->valid[b] < y->valid[best]))
				best = b;
		/* nothing to gain from moving a block that is nearly all current */
		if (best == YAFTL_VPN_NONE || y->valid[best] + 16 > data_pages)
			break;
		if (!y->free_blocks && y->valid[best])
			break;
		ret = yaftl_gc_block(y, best);
		if (ret < 0)
			return ret;
	}
	return 0;
}

int yaftl_write(struct yaftl *y, uint32_t lpn, const void *data)
{
	int ret;

	if (y->failed)
		return YAFTL_EROFS;
	if (lpn >= y->total_pages)
		return YAFTL_EIO;
	if (!y->prepared) {
		ret = yaftl_prepare_write(y);
		if (ret)
			return ret;
	}
	ret = yaftl_append(y, lpn, data, 0);
	if (!ret)
		y->st_user++;
	return ret;
}

int yaftl_sync(struct yaftl *y)
{
	if (y->failed)
		return YAFTL_EROFS;
	if (!y->prepared)
		return 0;
	return yaftl_index_limit(y, y->idx_dirty_max);
}
