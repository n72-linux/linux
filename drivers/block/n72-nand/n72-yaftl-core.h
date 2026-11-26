/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * n72-yaftl-core.h - Apple VSVFL + YAFTL for the iPod touch 2G (N72).
 *
 * The VFL is only read. YAFTL writes user pages, block TOCs and index pages
 * without writing a context slot, matching crash-recovery semantics (see
 * "writing" in n72-yaftl-core.c).
 *
 * The core talks to the flash through `struct nand_ops` so the same code can
 * sit under a userspace tool (/dev/fmss-raw) or a kernel block driver.
 */
#ifndef YAFTL_CORE_H
#define YAFTL_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stddef.h>
#include <stdint.h>
#endif

#define NAND_META_BYTES	12

/* nand_ops.read_page return values */
#define NAND_OK		0
#define NAND_BLANK	1		/* erased page: data undefined, meta = 0xff */
#define NAND_EIO	(-1)		/* uncorrectable / failed read */
#define NAND_ERANGE	(-2)		/* page not available (sample backend) */

struct nand_ops {
	void *ctx;
	/*
	 * Physical page `page` (0 .. ce_blocks*pages_per_block-1) of chip enable `ce`.
	 * `data` receives page_size bytes, `meta` the 12 metadata bytes.
	 */
	int (*read_page)(void *ctx, uint32_t ce, uint32_t page, void *data,
			 uint8_t meta[NAND_META_BYTES]);
	/* Writing (may be NULL: read-only). 0 or <0. */
	int (*write_page)(void *ctx, uint32_t ce, uint32_t page, const void *data,
			  const uint8_t meta[NAND_META_BYTES]);
	int (*erase_block)(void *ctx, uint32_t ce, uint32_t block);
	void *(*alloc)(size_t n);		/* zeroed */
	void (*free)(void *p);
	void (*log)(const char *fmt, ...);	/* may be NULL */
};

struct nand_geom {
	uint32_t page_size;		/* 8192 */
	uint32_t pages_per_block;	/* 128 */
	uint32_t ce_blocks;		/* blocks per chip enable, 4148 */
	uint32_t chip_enables;		/* 2 */
	/* derived in vfl_open() from the vendor type */
	uint32_t ce_banks;		/* 2 for vendor 0x150011 */
	uint32_t banks;			/* chip_enables * ce_banks = 4 */
	uint32_t stripe_pages;		/* pages_per_block * banks = 512 */
	uint32_t bank_blocks;		/* ce_blocks / ce_banks = 2074 */
};

/*
 * The 12 metadata bytes stored with every page.
 * What lpn means goes by the kind of page: a user page's logical page, an
 * index page's range of the map, a VFL context page's downward counter, 0
 * for an FTL context page.
 */
struct yaftl_spare {
	uint32_t lpn;
	uint32_t usn;		/* sequence number: later writes have larger ones */
	uint8_t zero;
	uint8_t type;		/* kind of page: the bits below */
	uint16_t ones;		/* 0xffff */
} __attribute__((packed));

#define YAFTL_PG_INDEX		0x04	/* index page: a range of the map */
#define YAFTL_PG_BTOC		0x08	/* a closed block's last page: what its pages hold */
#define YAFTL_PG_USER		0x10	/* user data */
#define YAFTL_PG_CXT		0x20	/* a page of the FTL's context */
#define YAFTL_PG_ERROR		0x40	/* never valid */
#define VFL_PG_CXT		0x80	/* VFL context */

/*
 * VSVFL context: the first 0x800 bytes of a context page, little-endian.
 * Only what this code reads has a name here.
 */
#define VFL_CXT_SIZE		0x800
#define VFL_CXT_GENERATION	0x000	/* u32, counts up; the CE with the largest is the newest */
#define VFL_CXT_POOL_MAP	0x026	/* u16 each: what a reserve block stands in for */
#define VFL_CXT_POOL_MAP_MAX	820
#define VFL_CXT_OWN_BLOCKS	0x68e	/* u16[4]: the CE's physical blocks that hold contexts */
#define VFL_CXT_FTL_BLOCKS	0x696	/* u16: blocks per bank the FTL has (1952) */
#define VFL_CXT_POOL_START	0x698	/* u16: where in a bank the reserve pool starts (1952) */
#define VFL_CXT_FTL_CTRL	0x69a	/* u16[3]: the FTL's control blocks */
#define VFL_CXT_VENDOR		0x6da	/* u32: how banks lie on a CE (0x150011) */
#define VFL_CXT_VERSION		0x7f4	/* u32, at most 2 */
#define VFL_CXT_SUM		0x7f8	/* u32 */
#define VFL_CXT_XOR		0x7fc	/* u32 */

#define VFL_POOL_FREE		0xfff0	/* pool map: the reserve block is not in use */

#define VFL_MAX_CE 2

/* stand_in[] values that are not block numbers */
#define VFL_GOOD	0xffff		/* the block is used as it is */
#define VFL_LOST	0xfffe		/* called bad, and nothing stands in for it */

struct vfl {
	const struct nand_ops *ops;
	struct nand_geom g;
	int latest;				/* CE holding the newest context */
	/* from the newest context, for every CE */
	uint32_t ftl_blocks;
	uint32_t pool_start;
	uint16_t ftl_ctrl[3];
	uint32_t reserved_per_bank;		/* bank_blocks - pool_start = 122 */
	/* [ce_blocks] per CE: VFL_GOOD, VFL_LOST, or the reserve block used instead */
	uint16_t *stand_in[VFL_MAX_CE];
	uint8_t *pagebuf;
};

/* 0 ok, <0 error.  `first_block` = first block to scan for a VFL cxt (1). */
int vfl_open(struct vfl *v, const struct nand_ops *ops, const struct nand_geom *g,
	     uint32_t first_block);
void vfl_close(struct vfl *v);
/* Virtual page -> CE and physical page (after bad-block remap). */
void vfl_translate(const struct vfl *v, uint32_t vpn, uint32_t *ce, uint32_t *ppage,
		   uint32_t *pblock_before_remap);
/* NAND_OK / NAND_BLANK / <0 */
int vfl_read(const struct vfl *v, uint32_t vpn, void *data, struct yaftl_spare *sp);
int vfl_write(const struct vfl *v, uint32_t vpn, const void *data,
	      const struct yaftl_spare *sp);
int vfl_erase(const struct vfl *v, uint32_t blk);
uint32_t vfl_num_blocks(const struct vfl *v);	/* FTL block count */
void vfl_ctrl_blocks(const struct vfl *v, uint16_t out[3]);

/*
 * The first page of a YAFTL context slot, little-endian.
 * Only what this code reads has a name here.
 */
#define YAFTL_CXT_MAGIC		"CX01"
#define YAFTL_CXT_INDEX_BLOCKS	0x04	/* u32: blocks kept for index pages */
#define YAFTL_CXT_LOGICAL_PAGES	0x08	/* u32 */
#define YAFTL_CXT_MAP_RANGES	0x5c	/* u16: index pages the map is made of */
#define YAFTL_CXT_BTOC_PAGES	0x5e	/* u16: pages of a block's table of contents */
#define YAFTL_CXT_RANGE_ENTRIES	0x60	/* u16: logical pages per index page */
#define YAFTL_CXT_OFFERED_PCT	0x94	/* u8: share of the logical pages iOS offers (99) */

#define YAFTL_VPN_NONE 0xffffffffu

/* yaftl.block_status[] */
#define YBS_FREE	0xff
#define YBS_USER	0x01
#define YBS_CTRL	0x02
#define YBS_INDEX	0x40
#define YBS_BAD		0x80	/* unreadable, unknown or failed: left alone */

/* yaftl.blk_flags[] */
#define YBF_CLOSED	0x01	/* ends with its block TOC */
#define YBF_ERASED	0x02	/* erased by us since the scan: blank */
#define YBF_NOGC	0x04	/* a page did not read back: never collected */

/* yaftl_write() and friends */
#define YAFTL_EIO	(-5)
#define YAFTL_ENOSPC	(-28)
#define YAFTL_EROFS	(-30)	/* a write failed earlier: nothing more is written */

/* A block being filled: user pages or index pages. */
struct yaftl_open_blk {
	uint32_t blk;		/* YAFTL_VPN_NONE: none */
	uint32_t used;
	uint32_t *btoc;		/* what each page holds, for the block TOC */
};

/* A block the scan found without its TOC (iOS's, or ours, after a cut). */
#define YAFTL_MAX_LEFT 8
struct yaftl_left {
	uint32_t blk;
	uint32_t *btoc;
};

struct yaftl {
	struct vfl *vfl;
	const struct nand_ops *ops;
	uint32_t num_blocks;		/* 1952 */
	uint32_t stripe_pages;	/* 512 */
	uint32_t page_size;
	uint32_t toc_pages_per_block;	/* 1 */
	uint32_t toc_entries_per_page;	/* 2048 */
	uint32_t toc_array_len;		/* 488 */
	uint32_t ctrl_block_page_offset;/* 10: pages per context slot (+1 marker) */
	uint32_t total_pages;		/* logical disk size in pages */
	uint32_t pages_available;	/* what iOS exposes: 99% of total-1 */
	uint16_t ctrl[3];
	int ctrl_sel;			/* ctrl[] index holding the newest cxt */
	uint32_t cxt_usn;
	uint32_t cxt_page;		/* vpn of the CX01 page, or VPN_NONE */
	int clean;			/* 1 = clean context used, 0 = rebuilt by scan */
	uint32_t *toc_index_page;	/* [toc_array_len] vpn of each index page */
	uint32_t **index_cache;		/* [toc_array_len] -> 2048 vpns or NULL */
	uint32_t *l2v;			/* restore mode: [total_pages] vpn */
	uint8_t *block_status;		/* [num_blocks] from the cxt (clean) or scan */
	uint32_t *erase_count;		/* [num_blocks] (clean only) */
	uint32_t index_blocks, user_blocks, free_blocks;	/* from scan */
	uint32_t max_usn;		/* highest usn seen (scan) or written */
	uint8_t *pagebuf;

	/* --- everything below is built by the scan and used for writing --- */
	uint16_t *valid;		/* [num_blocks] user pages the map points at */
	uint16_t *ivalid;		/* [num_blocks] index pages still current */
	uint32_t *blk_usn;		/* [num_blocks] usn of the first page */
	uint8_t *blk_flags;		/* [num_blocks] YBF_* */
	struct yaftl_left left[YAFTL_MAX_LEFT];
	uint32_t n_left;
	struct yaftl_open_blk ublk, iblk;
	uint8_t *idx_dirty;		/* [toc_array_len] flash index page out of date */
	uint32_t *idx_stamp;		/* [toc_array_len] when it last became so */
	uint32_t n_dirty, stamp;
	uint32_t last_written;		/* block of the last page, for the usn */
	uint32_t alloc_cursor;
	int prepared;			/* yaftl_prepare_write() done */
	int failed;			/* a program failed: read-only from then on */
	uint8_t *gcbuf, *idxbuf;
	uint32_t *gc_btoc;
	/* limits (set by yaftl_restore(), may be changed before writing) */
	uint32_t idx_dirty_max;		/* stale index pages left on the flash */
	uint32_t idx_dirty_burst;	/* ... tolerated between yaftl_sync() calls */
	uint32_t gc_low;		/* collect when fewer blocks are free */
	uint32_t min_free;		/* refuse writes when fewer are free */
	/* counters */
	uint32_t st_user, st_gc_pages, st_gc_blocks, st_index, st_erases, st_closed;
};

/*
 * Writing. Only on a map rebuilt by scanning (yaftl_restore()), and what is
 * left on the flash is always what iOS itself leaves after a crash: user
 * pages with their logical page in the spare, full blocks closed with a
 * block TOC, index pages for all but a few ranges, and no clean context,
 * so iOS (and Linux) rebuild the map at the next start.
 *
 * yaftl_prepare_write() makes the first changes: it invalidates a clean
 * context, closes the blocks left open and works out which index pages
 * are out of date. yaftl_write() calls it when needed.
 */
int yaftl_prepare_write(struct yaftl *y);
/* Index pages on the flash that differ from the map; @apply: internal use. */
uint32_t yaftl_index_compare(struct yaftl *y, int apply);
void yaftl_index_diff(struct yaftl *y, uint32_t out[6]);
/* One logical page. 0, or YAFTL_E*. */
int yaftl_write(struct yaftl *y, uint32_t lpn, const void *data);
/* Bring the stale index pages down to idx_dirty_max. */
int yaftl_sync(struct yaftl *y);

/* allow_restore: if no clean context, rebuild the map by scanning. */
int yaftl_open(struct yaftl *y, struct vfl *v, int allow_restore);
void yaftl_close(struct yaftl *y);
/* lpn -> vpn (YAFTL_VPN_NONE if never written). <0 on read error. */
int yaftl_map(struct yaftl *y, uint32_t lpn, uint32_t *vpn);
/* Read one logical page. NAND_OK, NAND_BLANK (unwritten, buffer zeroed), <0. */
int yaftl_read(struct yaftl *y, uint32_t lpn, void *buf);
/* Scan-based rebuild of the l2v table (read-only "restore"). */
int yaftl_restore(struct yaftl *y);

#endif
