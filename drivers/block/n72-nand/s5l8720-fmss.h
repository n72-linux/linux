/* SPDX-License-Identifier: GPL-2.0-only */
/* Internal interface between the S5L8720 FMSS driver and the YAFTL disks. */
#ifndef S5L8720_FMSS_H
#define S5L8720_FMSS_H

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/types.h>

struct attribute_group;
struct device;

/* FMC core */
#define FMCTRL0			0x000
#define  FMCTRL0_ON		BIT(0)
#define  FMCTRL0_CE(n)		BIT((n) + 1)	/* CE n, n = 0..7 */
#define  FMCTRL0_CE_MASK	GENMASK(8, 1)
#define  FMCTRL0_WPB		BIT(11)		/* never set here */
#define  FMCTRL0_HOLD(c)	(((c) & 7) << 12)
#define  FMCTRL0_SETUP(c)	(((c) & 7) << 16)
#define  FMCTRL0_DCCYCLE(c)	(((c) & 7) << 28)
#define  FMCTRL0_IDTIMING	0x00077000	/* setup 7, hold 7: what the ID program uses */
#define FMCTRL1			0x004
#define  FMCTRL1_SENDADDR	BIT(0)
#define  FMCTRL1_READFIFO	0xe2		/* as program #4 */
#define  FMCTRL1_FMCRESET	BIT(11)
#define  FMCTRL1_IDLE		BIT(30)
#define  FMCTRL1_SEQCFG		0x033338e0	/* written by every program */
#define FMCMD			0x008
#define  NAND_CMD_RESET		0xff
#define  NAND_CMD_READID	0x90
#define FMADDR0			0x00c
#define FMADDR1			0x010
#define FMUNIT			0x014		/* 1 << mode */
#define FMANUM			0x02c		/* address bytes - 1 */
#define FMDNUM			0x030		/* data bytes - 1 */
#define FMFIFOSTAT		0x040
#define FMCSTAT			0x048
#define  FMCSTAT_CMD		BIT(1)
#define  FMCSTAT_ADDR		BIT(2)
#define  FMCSTAT_XFER		BIT(3)
#define  FMCSTAT_ALL		0x0ff00ffe
#define FMRXDATA0		0x060
#define FMRXDATA1		0x064
#define ECC_CORE		0x81c

/* command sequencer */
#define CSCTRL			0xc00
#define  CSCTRL_STOP		8
#define  CSCTRL_RESUME		2
#define  CSCTRL_CLEAR		0x80000
#define  CSCTRL_RUN		0xfff5
#define CSBASEADDR		0xc04
#define CSSTAT			0xc08
#define CSIRQ			0xc0c
#define  CSIRQ_DONE		BIT(0)
#define  CSIRQ_ERR		BIT(2)
#define CSIRQMASK		0xc10
#define CSINTNR(n)		(0xc18 + 4 * (n))
#define  CSSTATUS		CSINTNR(6)
#define  CSSTATUS_UECC		BIT(28)
#define  CSSTATUS_BADMEDIA	BIT(30)
#define  CSSTATUS_BLANK		BIT(29)
#define CSBUFRST		0xc60
#define CSBUFRSTOK		0xc64
#define CSBUFCOND		0xc6c
#define  CSBUFCOND_IDLE		0x10
#define CSGENR(n)		(0xd00 + 4 * (n))

#define FMSS_SEQ_TIMEOUT_MS	2000
#define FMSS_POLL_US		10
#define FMSS_POLL_TOTAL_US	(51 * FMSS_POLL_US)
#define FMSS_NCE		8
#define FMSS_META_BYTES		12
#define FMSS_CHUNK		2048

struct fmss_seq {
	const char *name;
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

struct fmss {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct mutex lock;
	/*
	 * Controller clock gates ("clocks"), enabled only while the lock is
	 * held during an operation. Registers must not be accessed while
	 * gated (bus hang); clk_on guards the interrupt handler.
	 */
	struct clk_bulk_data *clks;
	int nclks;
	bool clk_on;
	struct completion done;
	u32 irq_status;
	bool first_run;

	u32 ce_bitmap;
	u32 page_size;
	u32 chunks;		/* page_size / 2048 */
	u32 read_timing;	/* FMCTRL0 timing bits, from DT */
	u32 expected_id;

	struct fmss_seq seq_reset, seq_readid, seq_read;
	bool fw_ok;
	/* writes: programs #1 and #3, loaded on first use */
	struct fmss_seq seq_program, seq_erase;
	u32 *wlist;		/* [cmd word, page, 0] / [ce word] [address] */
	dma_addr_t wlist_dma;

	/* DMA buffers for one page */
	void *data;
	dma_addr_t data_dma;
	u32 *lists;		/* page list, ce list, chunk list, meta */
	dma_addr_t lists_dma;
	u8 *idbuf;
	dma_addr_t idbuf_dma;

	/* last page read */
	u32 last_ce, last_page, last_status;
	int last_ret;
	u8 last_meta[FMSS_META_BYTES];

	/* /dev/fmss-raw: every page of every CE, read-only, for backups */
	struct miscdevice misc;
	u32 pages_per_ce;
};

static inline u32 fmss_rd(struct fmss *f, u32 off)
{
	return readl(f->base + off);
}

static inline void fmss_wr(struct fmss *f, u32 off, u32 val)
{
	writel(val, f->base + off);
}

void fmss_lock(struct fmss *f);
void fmss_unlock(struct fmss *f);
int fmss_hw_reset(struct fmss *f);
int fmss_fmc_wait(struct fmss *f, u32 bit);
int fmss_run(struct fmss *f, const struct fmss_seq *seq, u32 *status);
int fmss_read_page(struct fmss *f, u32 ce, u32 page);
int fmss_erase_block(struct fmss *f, u32 ce, u32 block, bool for_ftl);
int fmss_program_page(struct fmss *f, u32 ce, u32 page, const u8 *meta,
		      bool for_ftl);

#ifdef CONFIG_BLK_DEV_N72_NAND_DEBUG
extern const struct attribute_group fmss_debug_group;
#endif

/* Read one physical page: 0, -ENODATA (blank) or another error. */
int fmss_read_page_copy(struct fmss *f, u32 ce, u32 page, void *data, u8 *meta);
/* Erase / program for the FTL (refused unless the ftl_write parameter is set). */
bool fmss_ftl_write_enabled(void);
int fmss_ftl_erase(struct fmss *f, u32 ce, u32 block);
int fmss_ftl_program(struct fmss *f, u32 ce, u32 page, const void *data, const u8 *meta);
/* The FTL's GPT partitions as read-only disks, and the Linux image. */
int n72_yaftl_attach(struct device *dev, struct fmss *f);

#endif
