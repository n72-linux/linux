// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Apple S5L8720 FMSS NAND controller (iPod touch 2G).
 *
 * The FMSS is a S5L8900-style flash memory controller (FMC, 0x000) plus a BCH
 * ECC engine (0x800) and a small command sequencer (CS, 0xc00) that executes
 * programs fetched from RAM. All page I/O runs through sequencer programs
 * loaded as firmware (/lib/firmware/s5l8720-fmss/{reset,readid,read}.seq)
 * and vetted before use.
 *
 * The read side cannot program or erase:
 *  - the only NAND command bytes in this file are RESET (0xff) and READ ID
 *    (0x90), issued by the CPU-driven read-ID path;
 *  - a read firmware program is refused unless every write to FMCMD is a
 *    constant from {0x00, 0x30, 0x70, 0x90, 0xff}; a computed FMCMD write is
 *    refused;
 *  - FMCTRL0 bit 11 (WP# de-assert) is never set by the direct path;
 *  - every wait has a timeout and ends in a controller reset.
 * Program and erase are a separate path with programs of their own, used by
 * the FTL layer (n72-yaftl-blk.c): see "writes" below.
 *
 * It is a platform driver, not (yet) an MTD device. The raw test files in
 * sysfs (ID reads, single page reads, the registers, erase and program of
 * listed reserve blocks) live in s5l8720-fmss-debug.c when
 * CONFIG_BLK_DEV_N72_NAND_DEBUG is enabled.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

#include "s5l8720-fmss.h"

/*
 * One /dev/fmss-raw record: the page's data, then this trailer. Records run
 * through every page of the first present CE, then the next one.
 */
struct fmss_raw_trailer {
	u8 meta[FMSS_META_BYTES];	/* the FTL's per-page metadata */
	__le32 status;			/* sequencer status (CSINTNR6) */
	__le32 ret;			/* 0, -ENODATA (blank), -EBADMSG, ... */
	__le32 ce;
	__le32 page;
	__le32 magic;			/* FMSS_RAW_MAGIC */
};

#define FMSS_RAW_MAGIC		0x52534d46	/* "FMSR" */

static void fmss_clk_on(struct fmss *f)
{
	if (!clk_bulk_enable(f->nclks, f->clks))
		WRITE_ONCE(f->clk_on, true);
}

/*
 * The interrupt line is level, and a status bit latched when the gate closes
 * keeps it asserted until a clocked write clears it: clear it first, and ack
 * one taken while gated with the clock on briefly (an unhandled return would
 * let the kernel disable the line).
 */
static void fmss_clk_off(struct fmss *f)
{
	unsigned long flags;

	local_irq_save(flags);
	writel(readl(f->base + CSIRQ), f->base + CSIRQ);
	WRITE_ONCE(f->clk_on, false);
	clk_bulk_disable(f->nclks, f->clks);
	local_irq_restore(flags);
}

void fmss_lock(struct fmss *f)
{
	mutex_lock(&f->lock);
	fmss_clk_on(f);
}

void fmss_unlock(struct fmss *f)
{
	fmss_clk_off(f);
	mutex_unlock(&f->lock);
}

/* poll until (reg & mask) == val, 10 us steps, 510 us total */
static int fmss_poll(struct fmss *f, u32 off, u32 mask, u32 val)
{
	u32 v;

	return readl_poll_timeout_atomic(f->base + off, v, (v & mask) == val,
					 FMSS_POLL_US, FMSS_POLL_TOTAL_US);
}

/* Reset command sequencer and assert controller reset */
int fmss_hw_reset(struct fmss *f)
{
	u32 v;
	int ret;

	fmss_wr(f, CSCTRL, CSCTRL_STOP);
	ret = readl_poll_timeout_atomic(f->base + CSSTAT, v, v == 0 || v == 4,
					FMSS_POLL_US, FMSS_POLL_TOTAL_US);
	if (ret) {
		dev_err(f->dev, "reset: CSSTAT stuck at %#x\n", v);
		return ret;
	}
	if (v == 4) {
		fmss_wr(f, CSCTRL, CSCTRL_RESUME);
		ret = readl_poll_timeout_atomic(f->base + CSSTAT, v,
						v == 0 || v == 3,
						FMSS_POLL_US, FMSS_POLL_TOTAL_US);
		if (ret) {
			dev_err(f->dev, "reset: CSSTAT (2) stuck at %#x\n", v);
			return ret;
		}
	}
	if (fmss_rd(f, CSBUFCOND) != CSBUFCOND_IDLE) {
		fmss_wr(f, CSBUFRST, 1);
		ret = fmss_poll(f, CSBUFRSTOK, ~0u, 1);
		fmss_wr(f, CSBUFRST, 0);
		if (ret) {
			dev_err(f->dev, "reset: CSBUFRSTOK timeout\n");
			return ret;
		}
	}
	fmss_wr(f, CSCTRL, CSCTRL_CLEAR);
	fmss_wr(f, FMCTRL1, FMCTRL1_FMCRESET);
	ret = fmss_poll(f, FMCTRL1, FMCTRL1_IDLE, FMCTRL1_IDLE);
	if (ret) {
		dev_err(f->dev, "reset: FMC not idle, FMCTRL1 %#x\n",
			fmss_rd(f, FMCTRL1));
		return ret;
	}
	fmss_wr(f, FMCTRL1, BIT(31));
	fmss_wr(f, ECC_CORE, 1);
	f->first_run = true;
	return 0;
}

/* --- CPU-driven READ ID (mirrors program #4, no firmware) ----------------- */

int fmss_fmc_wait(struct fmss *f, u32 bit)
{
	int ret = fmss_poll(f, FMCSTAT, bit, bit);

	fmss_wr(f, FMCSTAT, bit);	/* write-1-to-clear */
	return ret;
}

/* --- sequencer ------------------------------------------------------------ */

static irqreturn_t fmss_irq(int irq, void *dev_id)
{
	struct fmss *f = dev_id;
	u32 st;

	if (!READ_ONCE(f->clk_on)) {
		/* latched before the gate closed (see fmss_clk_off) */
		if (clk_bulk_enable(f->nclks, f->clks))
			return IRQ_NONE;
		st = readl(f->base + CSIRQ);
		writel(st, f->base + CSIRQ);
		clk_bulk_disable(f->nclks, f->clks);
		return st ? IRQ_HANDLED : IRQ_NONE;
	}
	st = fmss_rd(f, CSIRQ) & fmss_rd(f, CSIRQMASK);

	if (!st)
		return IRQ_NONE;
	f->irq_status = st;
	fmss_wr(f, CSIRQ, st);
	if (st & CSIRQ_DONE)
		complete(&f->done);
	return IRQ_HANDLED;
}

/* Clear pending command-sequencer DONE/ERR interrupts and wait for idle */
static int fmss_clear_irq(struct fmss *f)
{
	fmss_wr(f, CSIRQ, CSIRQ_DONE | CSIRQ_ERR);
	return readl_poll_timeout(f->base + CSIRQ, f->irq_status,
				  !(f->irq_status & (CSIRQ_DONE | CSIRQ_ERR)),
				  FMSS_POLL_US, 100000);
}

/*
 * Execute a loaded command-sequencer program. The CSGENRs must already be set.
 * Returns 0, -EBADMSG (uncorrectable ECC), -EIO (bad media), -ENODATA (blank
 * page) or -ETIMEDOUT; the caller reads the correction count from *status.
 */
int fmss_run(struct fmss *f, const struct fmss_seq *seq, u32 *status)
{
	int ret;
	u32 st;

	if (!f->fw_ok || !seq->cpu)
		return -ENOENT;

	ret = fmss_clear_irq(f);
	if (ret) {
		dev_err(f->dev, "CSIRQ would not clear\n");
		return ret;
	}
	if (f->first_run) {
		fmss_wr(f, CSBUFCOND, 0);
		fmss_wr(f, CSBUFRST, 1);
		ret = fmss_poll(f, CSBUFRSTOK, ~0u, 1);
		fmss_wr(f, CSBUFRST, 0);
		if (ret) {
			dev_err(f->dev, "CSBUFRSTOK timeout before run\n");
			return ret;
		}
		f->first_run = false;
	} else {
		fmss_wr(f, CSBUFCOND, CSBUFCOND_IDLE);
	}

	reinit_completion(&f->done);
	f->irq_status = 0;
	fmss_wr(f, CSBASEADDR, lower_32_bits(seq->dma));
	wmb();
	fmss_wr(f, CSCTRL, CSCTRL_RUN);

	if (!wait_for_completion_timeout(&f->done,
					 msecs_to_jiffies(FMSS_SEQ_TIMEOUT_MS))) {
		dev_err(f->dev, "%s: sequence timed out (CSIRQ %#x CSSTAT %#x R6 %#x)\n",
			seq->name, fmss_rd(f, CSIRQ), fmss_rd(f, CSSTAT),
			fmss_rd(f, CSSTATUS));
		fmss_hw_reset(f);
		return -ETIMEDOUT;
	}
	if (f->irq_status & CSIRQ_ERR)
		dev_warn(f->dev, "%s: CSIRQ error bit set\n", seq->name);

	st = fmss_rd(f, CSSTATUS);
	if (status)
		*status = st;
	if (st & CSSTATUS_UECC)
		return -EBADMSG;
	if (st & CSSTATUS_BADMEDIA)
		return -EIO;
	if ((st & 0x2000ffff) == CSSTATUS_BLANK)
		return -ENODATA;
	return 0;
}

/*
 * Read one page with ECC through program #0.  Layout of f->lists (u32):
 *   [0]       page number
 *   [1]       CE word (1 << ce_index)
 *   [2..2+c)  physical address of each 2 KiB chunk of f->data
 *   [8..11)   12 bytes of metadata written by the FMSS
 * CSGENRs point into f->lists[0..8].
 */
#define LIST_PAGE	0
#define LIST_CE		1
#define LIST_CHUNK	2
#define LIST_META	8
#define LIST_WORDS	12

int fmss_read_page(struct fmss *f, u32 ce, u32 page)
{
	u32 *l = f->lists;
	u32 i, st = 0;
	int ret;

	if (ce >= FMSS_NCE || !(f->ce_bitmap & BIT(ce)))
		return -EINVAL;

	memset(l, 0, LIST_WORDS * 4);
	l[LIST_PAGE] = page;
	l[LIST_CE] = BIT(ce);
	for (i = 0; i < f->chunks; i++)
		l[LIST_CHUNK + i] = lower_32_bits(f->data_dma + i * FMSS_CHUNK);
	memset(f->data, 0xa5, f->page_size);
	wmb();

	fmss_wr(f, CSGENR(3), lower_32_bits(f->lists_dma + LIST_PAGE * 4));
	fmss_wr(f, CSGENR(4), lower_32_bits(f->lists_dma + LIST_CE * 4));
	fmss_wr(f, CSGENR(5), 0);
	fmss_wr(f, CSGENR(6), 1);			/* one page */
	fmss_wr(f, CSGENR(7), lower_32_bits(f->lists_dma + LIST_META * 4));
	fmss_wr(f, CSGENR(8), lower_32_bits(f->lists_dma + LIST_CHUNK * 4));
	fmss_wr(f, CSGENR(9), 0);
	fmss_wr(f, CSGENR(0xa), f->chunks);
	fmss_wr(f, CSGENR(0xb), 1);			/* 8-bit BCH */
	fmss_wr(f, CSGENR(0xc), 0xa01);
	fmss_wr(f, CSGENR(0xd), 0x16);
	fmss_wr(f, CSGENR(0xe), 0x0c);
	fmss_wr(f, CSGENR(0x13), f->read_timing);

	ret = fmss_run(f, &f->seq_read, &st);

	rmb();
	memcpy(f->last_meta, &l[LIST_META], FMSS_META_BYTES);
	f->last_ce = ce;
	f->last_page = page;
	f->last_status = st;
	f->last_ret = ret;
	return ret;
}

/* --- firmware (sequencer programs, vetted) ------------------------------- */

enum fmss_seq_kind {
	FMSS_SEQ_READ,
	FMSS_SEQ_ERASE,
	FMSS_SEQ_PROGRAM,
};

static bool fmss_cmd_allowed(enum fmss_seq_kind kind, u32 arg)
{
	switch (kind) {
	case FMSS_SEQ_READ:
		return arg == 0x00 || arg == 0x30 || arg == 0x70 ||
		       arg == 0x90 || arg == 0xff;
	case FMSS_SEQ_ERASE:
		return arg == 0x70 || arg == 0x60 || arg == 0xd0;
	case FMSS_SEQ_PROGRAM:
		return arg == 0x70 || arg == 0x78;
	}
	return false;
}

static bool fmss_vet_seq(struct device *dev, const char *name,
			 const u8 *data, size_t size, enum fmss_seq_kind kind)
{
	size_t k;

	if (size < 16 || size % 8 || size > SZ_8K) {
		dev_err(dev, "%s: bad size %zu\n", name, size);
		return false;
	}
	for (k = 0; k < size; k += 8) {
		u32 w = get_unaligned_le32(data + k);
		u32 arg = get_unaligned_le32(data + k + 4);
		u8 op = w >> 24;

		if ((w & 0xffff) != FMCMD)
			continue;
		if (op != 0x01) {
			/* program.seq issues computed 80/10 commands from wlist */
			if (kind == FMSS_SEQ_PROGRAM)
				continue;
			dev_err(dev, "%s: computed FMCMD write at +%#zx\n", name, k);
			return false;
		}
		if (!fmss_cmd_allowed(kind, arg)) {
			dev_err(dev, "%s: FMCMD %#x at +%#zx refused\n", name, arg, k);
			return false;
		}
	}
	if (get_unaligned_le32(data + size - 8) || get_unaligned_le32(data + size - 4)) {
		dev_err(dev, "%s: no END\n", name);
		return false;
	}
	return true;
}

static int fmss_load_seq_kind(struct fmss *f, struct fmss_seq *s,
			      const char *name, enum fmss_seq_kind kind)
{
	const struct firmware *fw;
	char path[64];
	int ret;

	snprintf(path, sizeof(path), "s5l8720-fmss/%s.seq", name);
	ret = request_firmware(&fw, path, f->dev);
	if (ret) {
		if (kind == FMSS_SEQ_READ)
			dev_info(f->dev, "%s not available (%d): sequencer paths disabled\n",
				 path, ret);
		return ret;
	}
	if (!fmss_vet_seq(f->dev, path, fw->data, fw->size, kind)) {
		release_firmware(fw);
		return -EPERM;
	}
	s->name = name;
	s->size = fw->size;
	s->cpu = dmam_alloc_coherent(f->dev, s->size, &s->dma, GFP_KERNEL);
	if (!s->cpu) {
		release_firmware(fw);
		return -ENOMEM;
	}
	memcpy(s->cpu, fw->data, fw->size);
	release_firmware(fw);
	if (kind != FMSS_SEQ_READ)
		dev_dbg(f->dev, "write path: %s loaded\n", path);
	return 0;
}

static int fmss_load_write_seq(struct fmss *f, struct fmss_seq *s, const char *name)
{
	return fmss_load_seq_kind(f, s, name,
				  !strcmp(name, "program") ? FMSS_SEQ_PROGRAM : FMSS_SEQ_ERASE);
}

static int fmss_load_seq(struct fmss *f, struct fmss_seq *s, const char *name)
{
	return fmss_load_seq_kind(f, s, name, FMSS_SEQ_READ);
}

/* --- writes ----------------------------------------------------------------
 *
 * Program and erase run sequencer programs #1 and #3 (firmware
 * s5l8720-fmss/{program,erase}.seq). They are for the FTL layer. The raw test
 * files (debug builds) write nothing unless the block is listed in write_allow
 * ("ce:block,ce:block", empty = no writes), and only blocks in the VSVFL
 * reserve pool area are accepted at all: a reserve block is only used after
 * erasing it, so an unused one can be tested on safely.
 */
#define FMSS_WRITE_TIMING	0x00011000	/* write setup 1, hold 1 (DT) */
#define FMSS_RESERVE_FIRST	3904		/* 2 x reserved_block_pool_start */
#define FMSS_RESERVE_END	4140		/* 4140-4147: special pages, never */
#define FMSS_PPB		128

#ifdef CONFIG_BLK_DEV_N72_NAND_DEBUG
static char *write_allow = "";
module_param(write_allow, charp, 0600);
MODULE_PARM_DESC(write_allow, "ce:block,... the only NAND blocks the raw test files may erase or program");

static bool fmss_write_allowed(u32 ce, u32 block)
{
	const char *p = write_allow;
	u32 c, b;
	int n;

	if (block < FMSS_RESERVE_FIRST || block >= FMSS_RESERVE_END)
		return false;
	while (p && *p) {
		if (sscanf(p, "%u:%u%n", &c, &b, &n) != 2)
			return false;
		if (c == ce && b == block)
			return true;
		p += n;
		if (*p != ',')
			break;
		p++;
	}
	return false;
}
#else
static bool fmss_write_allowed(u32 ce, u32 block)
{
	return false;
}
#endif

static int fmss_load_write_seq(struct fmss *f, struct fmss_seq *s, const char *name);

/*
 * Writes for the FTL (n72-yaftl-blk.c). The FTL layer picks the blocks
 * (scan-verified free FTL blocks through the VFL map), and only once its
 * image was made writable ("rw" in its "image" attribute); here only the VFL
 * context blocks (1-4), block 0 and the special-page area are refused
 * outright. ftl_write=0 keeps the NAND read-only whatever is asked.
 */
static bool ftl_write = true;
module_param(ftl_write, bool, 0600);
MODULE_PARM_DESC(ftl_write, "allow the YAFTL layer to erase and program its own blocks (default on)");

static int fmss_write_prepare(struct fmss *f, u32 ce, u32 block, bool for_ftl)
{
	int ret;

	if (ce >= FMSS_NCE || !(f->ce_bitmap & BIT(ce)))
		return -EINVAL;
	if (for_ftl ? (!ftl_write || block <= 4 || block >= FMSS_RESERVE_END)
		    : !fmss_write_allowed(ce, block)) {
		dev_warn(f->dev, "write to CE%u block %u refused (%s)\n", ce, block,
			 for_ftl ? "ftl_write off or reserved block" : "not in write_allow");
		return -EPERM;
	}
	if (!f->seq_erase.cpu) {
		ret = fmss_load_write_seq(f, &f->seq_erase, "erase");
		if (ret)
			return ret;
	}
	if (!f->seq_program.cpu) {
		ret = fmss_load_write_seq(f, &f->seq_program, "program");
		if (ret)
			return ret;
	}
	return 0;
}

/* Erase one block with program #3. */
int fmss_erase_block(struct fmss *f, u32 ce, u32 block, bool for_ftl)
{
	u32 st = 0;
	int ret = fmss_write_prepare(f, ce, block, for_ftl);

	if (ret)
		return ret;
	/*
	 * CE list: pass the CE *index*; program #3 shifts (1 << index) itself.
	 * (Passing a mask here selects the wrong chip: 1 means CE1.)
	 */
	f->wlist[0] = ce;
	f->wlist[1] = block * FMSS_PPB;		/* address list: the block's first page */
	wmb();
	fmss_wr(f, CSGENR(3), lower_32_bits(f->wlist_dma + 4));
	fmss_wr(f, CSGENR(4), lower_32_bits(f->wlist_dma));
	fmss_wr(f, CSGENR(6), 1);		/* one block */
	fmss_wr(f, CSGENR(0x13), FMSS_WRITE_TIMING);
	ret = fmss_run(f, &f->seq_erase, &st);
	/* lock the chip again: WP# asserted, CE deselected */
	fmss_wr(f, FMCTRL0, fmss_rd(f, FMCTRL0) & ~(FMCTRL0_WPB | FMCTRL0_CE_MASK));
	if (ret || !for_ftl)
		dev_info(f->dev, "erase CE%u block %u: status %#x, %d\n", ce, block, st, ret);
	return ret;
}

/*
 * Program one page with program #1 from f->data and 12 metadata bytes,
 * ECC encoded by the FMSS (8-bit BCH, as iOS writes).
 */
int fmss_program_page(struct fmss *f, u32 ce, u32 page, const u8 *meta,
		      bool for_ftl)
{
	u32 *l = f->wlist, i, st = 0;
	int ret = fmss_write_prepare(f, ce, page / FMSS_PPB, for_ftl);

	if (ret)
		return ret;
	memset(l, 0, 16 * 4);
	l[0] = 0x00801000 | BIT(ce);		/* cmd1 0x80, cmd2 0x10, CE mask */
	l[1] = page;
	l[2] = 0;				/* end of list */
	for (i = 0; i < f->chunks; i++)
		l[4 + i] = lower_32_bits(f->data_dma + i * FMSS_CHUNK);
	memcpy(&l[8], meta, FMSS_META_BYTES);
	wmb();

	fmss_wr(f, CSGENR(2), 8);
	fmss_wr(f, CSGENR(4), lower_32_bits(f->wlist_dma));
	fmss_wr(f, CSGENR(7), lower_32_bits(f->wlist_dma + 8 * 4));
	fmss_wr(f, CSGENR(8), lower_32_bits(f->wlist_dma + 4 * 4));
	fmss_wr(f, CSGENR(0xa), f->chunks);
	fmss_wr(f, CSGENR(0xb), 2);		/* 8-bit BCH, encode */
	fmss_wr(f, CSGENR(0xc), 0xa02);
	fmss_wr(f, CSGENR(0xd), 0x16);
	fmss_wr(f, CSGENR(0xe), 0x0c);
	fmss_wr(f, CSGENR(0x12), FMSS_WRITE_TIMING);
	fmss_wr(f, CSGENR(0x1d), 0);		/* only for the 0x78 path */
	fmss_wr(f, CSGENR(0x1e), 0);		/* plain 70 status */
	ret = fmss_run(f, &f->seq_program, &st);
	fmss_wr(f, FMCTRL0, fmss_rd(f, FMCTRL0) & ~(FMCTRL0_WPB | FMCTRL0_CE_MASK));
	if (ret || !for_ftl)
		dev_info(f->dev, "program CE%u page %u: status %#x, %d\n", ce, page, st, ret);
	return ret;
}

/* --- for the YAFTL disks (n72-yaftl-blk.c) ---------------------------------- */

bool fmss_ftl_write_enabled(void)
{
	return ftl_write;
}

int fmss_ftl_erase(struct fmss *f, u32 ce, u32 block)
{
	int ret;

	fmss_lock(f);
	ret = fmss_erase_block(f, ce, block, true);
	fmss_unlock(f);
	return ret;
}

int fmss_ftl_program(struct fmss *f, u32 ce, u32 page, const void *data, const u8 *meta)
{
	int ret;

	fmss_lock(f);
	memcpy(f->data, data, f->page_size);
	ret = fmss_program_page(f, ce, page, meta, true);
	fmss_unlock(f);
	return ret;
}

int fmss_read_page_copy(struct fmss *f, u32 ce, u32 page, void *data, u8 *meta)
{
	int ret;

	fmss_lock(f);
	ret = fmss_read_page(f, ce, page);
	memcpy(data, f->data, f->page_size);
	memcpy(meta, f->last_meta, FMSS_META_BYTES);
	fmss_unlock(f);
	return ret;
}

/* --- /dev/fmss-raw --------------------------------------------------------- */

static size_t fmss_raw_rec(struct fmss *f)
{
	return f->page_size + sizeof(struct fmss_raw_trailer);
}

static bool fmss_raw_locate(struct fmss *f, u64 idx, u32 *ce, u32 *page)
{
	u32 c;

	for (c = 0; c < FMSS_NCE; c++) {
		if (!(f->ce_bitmap & BIT(c)))
			continue;
		if (idx < f->pages_per_ce) {
			*ce = c;
			*page = idx;
			return true;
		}
		idx -= f->pages_per_ce;
	}
	return false;
}

static int fmss_raw_open(struct inode *inode, struct file *file)
{
	struct fmss *f = container_of(file->private_data, struct fmss, misc);

	if (file->f_mode & FMODE_WRITE)
		return -EPERM;		/* there is no write path, on purpose */
	if (!f->fw_ok)
		return -ENOENT;		/* needs the read program */
	return 0;
}

static ssize_t fmss_raw_read(struct file *file, char __user *buf, size_t len,
			     loff_t *ppos)
{
	struct fmss *f = container_of(file->private_data, struct fmss, misc);
	size_t rec = fmss_raw_rec(f);
	struct fmss_raw_trailer t;
	ssize_t done = 0;
	u32 ce, page, rem;
	u64 idx;
	int ret;

	if (*ppos < 0)
		return -EINVAL;
	idx = div_u64_rem(*ppos, rec, &rem);
	if (rem)
		return -EINVAL;
	if (len < rec)
		return -EINVAL;

	while (len - done >= rec) {
		if (!fmss_raw_locate(f, idx, &ce, &page))
			break;		/* end of the flash */
		if (fatal_signal_pending(current))
			return done ?: -EINTR;

		fmss_lock(f);
		ret = fmss_read_page(f, ce, page);
		memcpy(t.meta, f->last_meta, sizeof(t.meta));
		t.status = cpu_to_le32(f->last_status);
		t.ret = cpu_to_le32(ret);
		t.ce = cpu_to_le32(ce);
		t.page = cpu_to_le32(page);
		t.magic = cpu_to_le32(FMSS_RAW_MAGIC);
		if (copy_to_user(buf + done, f->data, f->page_size) ||
		    copy_to_user(buf + done + f->page_size, &t, sizeof(t))) {
			fmss_unlock(f);
			return done ?: -EFAULT;
		}
		fmss_unlock(f);

		done += rec;
		*ppos += rec;
		idx++;
		cond_resched();
	}
	return done;
}

static const struct file_operations fmss_raw_fops = {
	.owner = THIS_MODULE,
	.open = fmss_raw_open,
	.read = fmss_raw_read,
	.llseek = default_llseek,
};

/* --- probe ---------------------------------------------------------------- */

static void fmss_unprepare(void *data)
{
	struct fmss *f = data;

	clk_bulk_unprepare(f->nclks, f->clks);
}

static int fmss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct fmss *f;
	u32 setup = 1, hold = 1, dccycle = 2;
	int ret;

	f = devm_kzalloc(dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	f->dev = dev;
	mutex_init(&f->lock);
	init_completion(&f->done);
	platform_set_drvdata(pdev, f);

	f->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(f->base))
		return PTR_ERR(f->base);

	f->ce_bitmap = 3;
	of_property_read_u32(np, "apple,ce-bitmap", &f->ce_bitmap);
	f->page_size = 8192;
	of_property_read_u32(np, "apple,page-size", &f->page_size);
	if (f->page_size % FMSS_CHUNK || f->page_size > 4 * FMSS_CHUNK)
		return -EINVAL;
	f->chunks = f->page_size / FMSS_CHUNK;
	of_property_read_u32(np, "apple,read-setup-clks", &setup);
	of_property_read_u32(np, "apple,read-hold-clks", &hold);
	of_property_read_u32(np, "apple,read-dccycle-clks", &dccycle);
	f->read_timing = FMCTRL0_SETUP(setup) | FMCTRL0_HOLD(hold) |
			 FMCTRL0_DCCYCLE(dccycle);
	of_property_read_u32(np, "apple,expected-readid", &f->expected_id);
	{
		u32 blocks = 4148, ppb = 128;

		of_property_read_u32(np, "apple,blocks-per-ce", &blocks);
		of_property_read_u32(np, "apple,pages-per-block", &ppb);
		f->pages_per_ce = blocks * ppb;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	f->data = dmam_alloc_coherent(dev, f->page_size, &f->data_dma, GFP_KERNEL);
	f->lists = dmam_alloc_coherent(dev, LIST_WORDS * 4, &f->lists_dma, GFP_KERNEL);
	f->idbuf = dmam_alloc_coherent(dev, 0x40, &f->idbuf_dma, GFP_KERNEL);
	f->wlist = dmam_alloc_coherent(dev, 16 * 4, &f->wlist_dma, GFP_KERNEL);
	if (!f->data || !f->lists || !f->idbuf || !f->wlist)
		return -ENOMEM;

	f->nclks = devm_clk_bulk_get_all(dev, &f->clks);
	if (f->nclks < 0)
		return dev_err_probe(dev, f->nclks, "clocks\n");
	ret = clk_bulk_prepare(f->nclks, f->clks);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, fmss_unprepare, f);
	if (ret)
		return ret;

	f->irq = platform_get_irq(pdev, 0);
	if (f->irq < 0)
		return f->irq;
	ret = devm_request_irq(dev, f->irq, fmss_irq, 0, dev_name(dev), f);
	if (ret)
		return ret;

	fmss_lock(f);
	ret = fmss_hw_reset(f);
	fmss_unlock(f);
	if (ret) {
		dev_err(dev, "FMSS did not reset (is the clock gated?)\n");
		return ret;
	}

	/* Optional: sequencer programs.  Without them only direct ID read works. */
	f->fw_ok = !fmss_load_seq(f, &f->seq_reset, "reset") &&
		   !fmss_load_seq(f, &f->seq_readid, "readid") &&
		   !fmss_load_seq(f, &f->seq_read, "read");

#ifdef CONFIG_BLK_DEV_N72_NAND_DEBUG
	ret = devm_device_add_group(dev, &fmss_debug_group);
	if (ret)
		return ret;
#endif

	f->misc.minor = MISC_DYNAMIC_MINOR;
	f->misc.name = "fmss-raw";
	f->misc.fops = &fmss_raw_fops;
	f->misc.mode = 0400;
	ret = misc_register(&f->misc);
	if (ret)
		return ret;

	if (f->fw_ok && n72_yaftl_attach(dev, f))
		dev_warn(dev, "FTL disks not available\n");

	dev_info(dev, "S5L8720 FMSS; CE bitmap %#x, %u-byte pages, timing %#x, sequencer %s\n",
		 f->ce_bitmap, f->page_size, f->read_timing,
		 f->fw_ok ? "firmware loaded" : "disabled (no firmware)");
	return 0;
}

static void fmss_remove(struct platform_device *pdev)
{
	struct fmss *f = platform_get_drvdata(pdev);

	misc_deregister(&f->misc);
	fmss_lock(f);
	fmss_hw_reset(f);
	fmss_unlock(f);
}

static int fmss_resume(struct device *dev)
{
	struct fmss *f = dev_get_drvdata(dev);

	fmss_lock(f);
	f->first_run = true;
	fmss_unlock(f);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fmss_pm_ops, NULL, fmss_resume);

static const struct of_device_id fmss_of_match[] = {
	{ .compatible = "apple,s5l8720-fmss" },
	{ }
};
MODULE_DEVICE_TABLE(of, fmss_of_match);

static struct platform_driver fmss_driver = {
	.probe = fmss_probe,
	.remove = fmss_remove,
	.driver = {
		.name = "s5l8720-fmss",
		.of_match_table = fmss_of_match,
		.pm = pm_sleep_ptr(&fmss_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(fmss_driver);

MODULE_DESCRIPTION("Apple S5L8720 FMSS NAND controller");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("s5l8720-fmss/reset.seq");
MODULE_FIRMWARE("s5l8720-fmss/readid.seq");
MODULE_FIRMWARE("s5l8720-fmss/read.seq");
MODULE_FIRMWARE("s5l8720-fmss/program.seq");
MODULE_FIRMWARE("s5l8720-fmss/erase.seq");
