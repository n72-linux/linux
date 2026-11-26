// SPDX-License-Identifier: GPL-2.0-only
/*
 * S5L8720 AMC audio coprocessor (iPod touch 2G). A 24-bit programmable
 * "Data Engine" (DE) that runs codec firmware out of the 192 KiB SRAM at
 * 0x22000000 and decodes MP3, AAC-LC, HE-AAC (v2), ALAC and Audible packets
 * to 16-bit PCM.
 *
 * Programming model:
 *  - firmware: per codec a boot stage (stub + initialised data) and the
 *    program, loaded into the SRAM through the codec's MMU table (firmware
 *    files at apple/amc-*.bin have placement pre-resolved);
 *  - compressed bytes go to the DE's bitstream FIFO through the block's
 *    own DMA channel 4, as one continuous stream of packets;
 *  - 0x98c = 1 decodes one packet into one of the (one or two) PCM
 *    buffers in the SRAM and raises event bit 2, which is acknowledged
 *    through 0xc48; the CPU copies the PCM out (one memcpy per frame).
 *
 * Power management: the completion handler gates the block as soon as its
 * queue drains (power state 3), and the next submission reopens it without
 * a reload, so the AMCSS clocks are open only during a burst (one
 * AMC_IOC_START or AMC_IOC_DECODE call) and a paused or idle stream costs
 * nothing. Registers and SRAM retain their contents under the gate. While
 * the gate is open the divided idle states keep the DE clock up and
 * respect the codec's system divider limit (/2 for HE-AAC, /5 for ALAC,
 * coordinated with cpuidle-s5l8720). A suspend deeper than s2idle powers
 * the SoC off: the stream is saved and restored before its next packet.
 *
 * Several users (up to AMC_MAX_USERS opens: a player's decoder and the
 * Bluetooth encoder): each open file is a context, and the coprocessor
 * goes to whichever context calls, the previous one's stream saved to RAM
 * (its codec's mapped SRAM ranges) and restored with its program when it
 * calls again.
 *
 * The AAC-LC encoder (AMC_CODEC_AACENC): DMA channel 0 copies each
 * 1024-frame PCM block into a PCM buffer of the SRAM, 0x98c = 1 encodes
 * it, and the access unit leaves through the Stream Out FIFO, which DMA
 * channel 5 empties into memory; the status block gives each unit's size.
 * (CPU stores into that buffer instead of channel 0 caused periodic error
 * code 33 stalls.)
 *
 * Interface: /dev/amc, include/uapi/linux/s5l8720-amc.h.
 */
#include <linux/clk.h>
#include <linux/clk/s5l8720.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <uapi/linux/s5l8720-amc.h>

/* registers */
#define AMC_CH(n)		((n) < 4 ? (n) * 0x20 : 0x100 + ((n) - 4) * 0x80)
#define  CH_LLI			0x00	/* tagged list pointer, writing starts */
#define  CH_CMD			0x10
#define   CMD_HALT		4
#define   CMD_RESUME		8
#define   CMD_ABORT		0x10
#define   CMD_ACK		0x20
#define   CMD_CLEAR		0x60
#define   CMD_FLUSH		0x80	/* channel 5: write out the partial bytes */
#define  CH_STATUS		0x14	/* bits 2:0: 0 idle, 7 halted */
#define AMC_IN_SRC		0x118	/* 0 = channel 4 feeds the FIFO, 1 = host */
#define AMC_IN_DATA		0x15c	/* FIFO read-back (drain) */
#define AMC_IN_LEVEL		0x160
#define AMC_OUT_SRC		0x198	/* 0 = channel 5 empties the FIFO, 1 = host */
#define AMC_OUT_DATA		0x1dc
#define AMC_OUT_LEVEL		0x1e0	/* bits 4:0: bytes */
#define AMC_WIN_REQ		0x400	/* DE register window */
#define AMC_WIN_STATUS		0x404
#define   WIN_ERR		BIT(8)
#define AMC_WIN_SEL		0x40c
#define AMC_WIN_REG(i)		(0x420 + 4 * (i))
#define AMC_MBOX0_CLR		0x804
#define AMC_ERR_ACK		0x858
#define AMC_MMU(i)		(0x938 + 4 * (i))
#define AMC_START0		0x984	/* run controller 0 from this address */
#define AMC_START1		0x988
#define AMC_GO			0x98c	/* decode one packet */
#define AMC_STOP		0x99c
#define AMC_IRQ_EN		0xa8c
#define AMC_IRQ_DIS		0xa90
#define AMC_IRQ_ENABLED		0xa94
#define AMC_IRQ_STATUS		0xa98
#define AMC_IRQ1_DIS		0xb10
#define AMC_EVENT_ACK		0xc48
#define AMC_UID			0xc80
#define AMC_CLKDIV		0x1000

#define EV_DONE			BIT(2)
#define EV_ERROR		BIT(12)
#define EV_CH0			BIT(14)
#define EV_CH4			BIT(18)

#define AMC_UID_VALUE		0x004e7309

/* SRAM window offsets */
#define SRAM_SIZE		0x30000
#define SRAM_HDR		0x28000
#define  HDR_NBUF		0x02
#define  HDR_BUFLEN		0x04	/* 16-bit samples */
#define  HDR_FLAG(b)		(0x0a + 4 * (b))
#define  HDR_COUNT(b)		(0x0c + 4 * (b))
#define SRAM_PCM		0x28100
#define SRAM_PARAM		0x2ff00
#define SRAM_STATUS		0x2ff28

/* channel 4 list item: hardware reads words 0-3 */
struct amc_lli {
	__le32 next, ctrl, src, dst;
};
#define LLI_SYS			1	/* tag: system memory address */
#define LLI_CTRL(bytes)		((bytes) << 16 | 0x1f << 7 | 0x10)

struct amc_codec_info {
	const char *fw;
	unsigned int pcm_bytes, nbuf;
};

static const struct amc_codec_info amc_codecs[] = {
	[AMC_CODEC_MP3]   = { "apple/amc-mp3.bin",   0x1200, 2 },
	[AMC_CODEC_AAC]   = { "apple/amc-aac.bin",   0x1000, 2 },
	[AMC_CODEC_HEAAC] = { "apple/amc-heaac.bin", 0x2000, 2 },
	[AMC_CODEC_ALAC]  = { "apple/amc-alac.bin",  0x4000, 1 },
	[AMC_CODEC_AUDB]  = { "apple/amc-audb.bin",  0x0140, 2 },
	[AMC_CODEC_AACENC] = { "apple/amc-aacenc.bin", 0x1000, 2 },	/* PCM input */
};

/* one stream: what a context keeps while another one has the coprocessor */
struct amc_stream {
	bool started;
	bool reload;			/* nothing saved to come back to: start again */
	struct amc_start cfg;		/* how it was started */
	unsigned int max_div;		/* the codec's system divider limit while running */
	const struct amc_codec_info *ci;
	unsigned int codec, channels, frames, buf;
	/* the SRAM its codec maps (every DE data word), from the MMU table */
	unsigned int nranges;
	struct { u32 off, len; } range[18];
	u32 save_len;
};

struct amc;

/* one open file */
struct amc_ctx {
	struct amc *a;
	struct list_head node;		/* in amc.ctxs */
	struct amc_stream s;		/* valid while it is not the owner */
	void *save;			/* its SRAM ranges while another context runs */
	u32 win[16];			/* and the DE's register window (bitstream unit) */
	u32 save_cap;
	bool saved;
};

struct amc {
	struct device *dev;
	void __iomem *base, *sram;
	struct clk_bulk_data *clks;
	int nclks;
	unsigned long bus_hz;
	struct miscdevice misc;
	struct mutex lock;		/* one call at a time */
	struct completion done;
	u32 events;			/* from the interrupt handler */

	bool clk_on;			/* the AMCSS gate is open (a burst runs) */

	struct amc_stream s;		/* the stream loaded in the SRAM */
	struct amc_ctx *owner;		/* whose it is */
	struct list_head ctxs;		/* the open files */
	unsigned int users;
	const struct firmware *fw[ARRAY_SIZE(amc_codecs)];	/* kept once loaded */
	void *in;			/* AMC_MAX_INPUT + padding, coherent */
	dma_addr_t in_dma;
	struct amc_lli *lli;
	dma_addr_t lli_dma;
	void *pcm;			/* bounce for one frame */
	void *eout;			/* encoder output, coherent */
	dma_addr_t eout_dma;
	struct amc_lli *lli5;
	dma_addr_t lli5_dma;
	u32 esizes[AMC_MAX_ENC_BLOCKS];
	u32 sizes[AMC_MAX_PACKETS];
};

static inline u32 amc_r(struct amc *a, u32 o) { return readl_relaxed(a->base + o); }
static inline void amc_w(struct amc *a, u32 o, u32 v) { writel_relaxed(v, a->base + o); }
static inline u16 sram_r16(struct amc *a, u32 o) { return readw_relaxed(a->sram + o); }
static inline void sram_w16(struct amc *a, u32 o, u16 v) { writew_relaxed(v, a->sram + o); }

/* halt a channel, then reset it to idle (and clear its interrupt) */
static int amc_ch_reset(struct amc *a, unsigned int ch)
{
	u32 b = AMC_CH(ch), st;
	int ret;

	amc_w(a, b + CH_CMD, CMD_HALT);
	ret = readl_relaxed_poll_timeout_atomic(a->base + b + CH_STATUS, st,
						(st & 7) == 7, 1, 1000);
	if (ret) {
		dev_err(a->dev, "DMA channel %u does not halt (status %x)\n", ch, st);
		return ret;
	}
	amc_w(a, b + CH_CMD, CMD_ABORT);
	amc_w(a, b + CH_CMD, CMD_CLEAR);
	return 0;
}

/* everything masked, DMA channels idle, FIFOs empty, controllers stopped */
static int amc_quiesce(struct amc *a)
{
	int i, ret, err;

	amc_w(a, AMC_IRQ_DIS, 0x07ffffff);
	amc_w(a, AMC_IRQ1_DIS, 0xffffffff);
	ret = amc_ch_reset(a, 4);
	for (i = 0; i < 20 && (amc_r(a, AMC_IN_LEVEL) & 0xf); i++)
		amc_r(a, AMC_IN_DATA);
	err = amc_ch_reset(a, 5);
	if (!ret)
		ret = err;
	for (i = 0; i < 20 && (amc_r(a, AMC_OUT_LEVEL) & 0x1f); i++)
		amc_r(a, AMC_OUT_DATA);
	err = amc_ch_reset(a, 0);
	if (!ret)
		ret = err;
	amc_w(a, AMC_STOP, 3);
	return ret;
}

static u32 amc_ack(struct amc *a)
{
	u32 st = amc_r(a, AMC_IRQ_STATUS) & amc_r(a, AMC_IRQ_ENABLED);

	if (st & EV_CH4)
		amc_w(a, AMC_CH(4) + CH_CMD, CMD_ACK);
	if (st & EV_DONE)
		amc_w(a, AMC_EVENT_ACK, EV_DONE);
	if (st & EV_ERROR) {
		amc_w(a, AMC_MBOX0_CLR, 1);
		amc_w(a, AMC_ERR_ACK, 1);
	}
	return st;
}

static irqreturn_t amc_irq(int irq, void *data)
{
	struct amc *a = data;
	u32 st;

	/* no register access while gated (nothing can be pending then) */
	if (!READ_ONCE(a->clk_on))
		return IRQ_NONE;
	st = amc_ack(a);
	if (!st)
		return IRQ_NONE;
	if (st & (EV_DONE | EV_ERROR)) {
		a->events |= st;
		complete(&a->done);
	}
	return IRQ_HANDLED;
}

/* iOS's votes: AMC-HE (HE-AAC, the encoder) /2, AMC-ALAC /5, else none (/11) */
static unsigned int amc_max_div(unsigned int codec)
{
	if (codec == AMC_CODEC_HEAAC || codec == AMC_CODEC_AACENC)
		return 2;
	return codec == AMC_CODEC_ALAC ? 5 : 11;
}

/* open the gate for a burst; the clocks are prepared while the file is open */
static int amc_clk_on(struct amc *a)
{
	int ret;

	if (a->clk_on)
		return 0;
	ret = clk_bulk_enable(a->nclks, a->clks);
	if (ret)
		return ret;
	WRITE_ONCE(a->clk_on, true);
	s5l8720_idle_amc_clock(a->base + AMC_CLKDIV, a->bus_hz, a->s.max_div);
	return 0;
}

/* the burst is over: gate it, nothing reloads when it opens again */
static void amc_clk_off(struct amc *a)
{
	u32 st;

	if (!a->clk_on)
		return;
	/* the input DMA may still push a drain's padding: let it finish */
	readl_relaxed_poll_timeout_atomic(a->base + AMC_CH(4) + CH_STATUS, st,
					  !(st & 7), 10, 2000);
	amc_ack(a);
	s5l8720_idle_amc_clock(NULL, 0, 0);
	WRITE_ONCE(a->clk_on, false);
	clk_bulk_disable(a->nclks, a->clks);
}

/* firmware file: 'AMCF' 1 type 18 mmu[18], then two sections of records */
struct amc_fw_hdr {
	char magic[4];
	__le32 version, type, nmmu, mmu[18];
};

static int amc_fw_section(struct amc *a, const struct firmware *fw, size_t *pos, bool load)
{
	size_t p = *pos;
	u32 n, off, len, i;

	if (p + 4 > fw->size)
		return -EINVAL;
	n = get_unaligned_le32(fw->data + p);
	for (p += 4; n--; p += 8 + ALIGN(len, 4)) {
		if (p + 8 > fw->size)
			return -EINVAL;
		off = get_unaligned_le32(fw->data + p);
		len = get_unaligned_le32(fw->data + p + 4);
		if (off >= SRAM_SIZE || len > SRAM_SIZE - off || (off | len) & 1 ||
		    p + 8 + ALIGN(len, 4) > fw->size)
			return -EINVAL;
		/* halfword stores: the 16-bit cells take nothing else for sure */
		for (i = 0; load && i < len; i += 2)
			sram_w16(a, off + i, get_unaligned_le16(fw->data + p + 8 + i));
	}
	*pos = p;
	return 0;
}

static int amc_wait_event(struct amc *a, u32 bit, unsigned int us)
{
	u32 st;

	return readl_relaxed_poll_timeout(a->base + AMC_IRQ_STATUS, st, st & bit, 10, us);
}

static const u32 amc_rates[] = {
	96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000
};

static void amc_write_params(struct amc *a, const struct amc_start *s)
{
	unsigned int i, sri = 4;

	for (i = 0; i < ARRAY_SIZE(amc_rates); i++)
		if (amc_rates[i] == s->rate)
			sri = i;
	sram_w16(a, SRAM_PARAM, 0);
	sram_w16(a, SRAM_PARAM, 1);
	switch (s->codec) {
	case AMC_CODEC_MP3:
		sram_w16(a, SRAM_PARAM, 0x1);
		break;
	case AMC_CODEC_AAC:
		sram_w16(a, SRAM_PARAM + 4, 0);
		sram_w16(a, SRAM_PARAM + 6, sri);
		sram_w16(a, SRAM_PARAM, 0x7);
		break;
	case AMC_CODEC_HEAAC:	/* core (half) rate in Hz */
		sram_w16(a, SRAM_PARAM + 4, (s->rate / 2) >> 16);
		sram_w16(a, SRAM_PARAM + 6, (s->rate / 2) & 0xffff);
		sram_w16(a, SRAM_PARAM + 8, 0);
		sram_w16(a, SRAM_PARAM + 10, !!(s->flags & AMC_START_PS));
		sram_w16(a, SRAM_PARAM, 0x1f);
		break;
	case AMC_CODEC_ALAC:
		sram_w16(a, SRAM_PARAM + 4, s->cookie[0]);	/* bitDepth */
		sram_w16(a, SRAM_PARAM + 6, s->cookie[1]);	/* pb */
		sram_w16(a, SRAM_PARAM + 8, s->cookie[3]);	/* kb */
		sram_w16(a, SRAM_PARAM + 10, s->cookie[2]);	/* mb */
		sram_w16(a, SRAM_PARAM, 0x1f);
		break;
	case AMC_CODEC_AUDB:
		sram_w16(a, SRAM_PARAM + 4, s->rate == 8000);
		sram_w16(a, SRAM_PARAM, 0x3);
		break;
	case AMC_CODEC_AACENC:
		sram_w16(a, SRAM_PARAM + 4, s->channels);
		sram_w16(a, SRAM_PARAM + 6, s->rate >> 16);
		sram_w16(a, SRAM_PARAM + 8, s->rate & 0xffff);
		sram_w16(a, SRAM_PARAM + 10, s->bitrate >> 16);
		sram_w16(a, SRAM_PARAM + 12, s->bitrate & 0xffff);
		sram_w16(a, SRAM_PARAM + 14, 0);
		sram_w16(a, SRAM_PARAM, 0x7f);
		break;
	}
}

static unsigned int amc_frames(const struct amc_start *s)
{
	switch (s->codec) {
	case AMC_CODEC_MP3:	return s->rate >= 32000 ? 1152 : 576;
	case AMC_CODEC_HEAAC:	return 2048;
	case AMC_CODEC_ALAC:	return 4096;
	case AMC_CODEC_AUDB:	return s->rate == 8000 ? 0x90 : 0xa0;
	default:		return 1024;
	}
}

/* the DE's 16-register window (0x420-0x45c): its bitstream unit's state */
static int amc_window(struct amc *a, u32 *win, bool write)
{
	u32 v;
	int i, ret;

	amc_w(a, AMC_WIN_REQ, 1);
	ret = readl_relaxed_poll_timeout_atomic(a->base + AMC_WIN_REQ, v, v == 1, 1, 100);
	if (ret)
		goto out;
	for (i = 0; i < 16; i++) {
		if (write)
			amc_w(a, AMC_WIN_REG(i), win[i]);
		else
			win[i] = amc_r(a, AMC_WIN_REG(i));
	}
	if (amc_r(a, AMC_WIN_STATUS) & WIN_ERR)
		ret = -EIO;
out:
	amc_w(a, AMC_WIN_REQ, 0);
	return ret;
}

/*
 * Every DE data word lives in an SRAM range one of the codec's MMU entries
 * maps (validated during firmware placement): those ranges are a stream's
 * whole state between two calls, as the DE runs each packet or block from
 * its entry point. Entry: valid 31, 16-bit cells 30, region 29 (0: SRAM +
 * 0x10000, 1: SRAM + 0), base >> 8 in 28:18, end and start in units of 128
 * cells in 17:9 and 8:0.
 */
static int amc_ranges(struct amc *a, const struct amc_fw_hdr *h)
{
	struct amc_stream *st = &a->s;
	unsigned int i;

	st->nranges = 0;
	st->save_len = 0;
	for (i = 0; i < 18; i++) {
		u32 r = le32_to_cpu(h->mmu[i]), cb, start, end, off, len;

		if (!(r & BIT(31)))
			continue;
		cb = r & BIT(30) ? 2 : 4;
		start = r & 0x1ff;
		end = (r >> 9) & 0x1ff;
		off = ((((r >> 18) & 0x7ff) << 8) + start * 128 * cb) & 0x7ffff;
		off += r & BIT(29) ? 0 : 0x10000;
		len = (end - start + 1) * 128 * cb;
		if (end < start || off + len > SRAM_SIZE) {
			dev_err(a->dev, "MMU entry %u maps %#x+%#x\n", i, off, len);
			return -EINVAL;
		}
		st->range[st->nranges].off = off;
		st->range[st->nranges].len = len;
		st->nranges++;
		st->save_len += len;
	}
	return 0;
}

/* the owner's stream into its context, its SRAM ranges into RAM */
static int amc_save(struct amc *a, struct amc_ctx *c)
{
	unsigned int i, pos = 0;
	int ret;

	c->s = a->s;
	c->saved = false;
	if (!a->s.started)
		return 0;
	if (c->save_cap < a->s.save_len) {
		kvfree(c->save);
		c->save = kvmalloc(a->s.save_len, GFP_KERNEL);
		c->save_cap = c->save ? a->s.save_len : 0;
		if (!c->save)
			return -ENOMEM;
	}
	ret = amc_clk_on(a);
	if (ret)
		return ret;
	/* Save DE register window first, then active SRAM ranges */
	ret = amc_window(a, c->win, false);
	if (ret)
		goto out;
	for (i = 0; i < a->s.nranges; i++) {
		__ioread32_copy(c->save + pos, a->sram + a->s.range[i].off, a->s.range[i].len / 4);
		pos += a->s.range[i].len;
	}
	c->saved = true;
out:
	amc_clk_off(a);
	return ret;
}

/* a saved stream back: its program, its MMU table, its SRAM ranges */
static int amc_restore(struct amc *a, struct amc_ctx *c)
{
	const struct firmware *fw = a->fw[c->s.codec];
	const struct amc_fw_hdr *h = (const void *)fw->data;
	size_t pos = sizeof(*h);
	unsigned int i, div, off = 0;
	int ret;

	a->s = c->s;
	a->s.max_div = amc_max_div(a->s.codec);
	ret = amc_clk_on(a);
	if (ret)
		return ret;
	div = clamp(DIV_ROUND_UP(a->bus_hz, S5L8720_AMC_MAX_HZ), 1UL, 16UL);
	amc_w(a, AMC_CLKDIV, div - 1);
	s5l8720_idle_amc_resync();	/* the system may be running at /2 */
	ret = amc_quiesce(a);
	if (ret)
		goto out;
	for (i = 0; i < 18; i++)
		amc_w(a, AMC_MMU(i), le32_to_cpu(h->mmu[i]));
	amc_fw_section(a, fw, &pos, false);	/* skip the boot stage */
	amc_fw_section(a, fw, &pos, true);	/* the program */
	for (i = 0; i < a->s.nranges; i++) {
		__iowrite32_copy(a->sram + a->s.range[i].off, c->save + off, a->s.range[i].len / 4);
		off += a->s.range[i].len;
	}
	amc_w(a, AMC_START1, 0);
	amc_w(a, AMC_MBOX0_CLR, 1);
	amc_w(a, AMC_ERR_ACK, 1);
	amc_w(a, AMC_EVENT_ACK, EV_DONE);
	amc_w(a, AMC_IRQ_EN, EV_ERROR | EV_CH4 | EV_DONE);
	amc_w(a, AMC_IN_SRC, 0);
	/* Restore DE register window last after SRAM and interrupts */
	ret = amc_window(a, c->win, true);
out:
	amc_clk_off(a);
	return ret;
}

/*
 * Switch active coprocessor context to c (decoder and encoder instances share
 * the hardware engine this way).
 * fresh: c is about to start a new stream, nothing to restore.
 */
static void amc_switch(struct amc *a, struct amc_ctx *c, bool fresh)
{
	int ret;

	if (a->owner == c)
		return;
	if (a->owner) {
		ret = amc_save(a, a->owner);
		if (ret) {
			dev_warn(a->dev, "could not save a stream (%d): it starts over\n", ret);
			a->owner->s.reload = a->owner->s.started;
		}
	}
	a->owner = c;
	if (fresh || !c->s.started) {
		a->s = c->s;
		return;
	}
	if (!c->saved) {		/* never saved, or the save failed: start over */
		a->s = c->s;
		a->s.reload = true;
		return;
	}
	ret = amc_restore(a, c);
	if (ret) {
		dev_warn(a->dev, "could not restore a stream (%d): it starts over\n", ret);
		a->s.reload = true;
	}
}

/* Load the codec and let its boot stub set up the DE */
static int amc_start(struct amc *a, struct amc_start *s)
{
	const struct amc_codec_info *ci;
	const struct amc_fw_hdr *h;
	const struct firmware *fw;
	unsigned int div, i;
	size_t pos = sizeof(*h);
	int ret;

	if (s->codec >= ARRAY_SIZE(amc_codecs) || s->channels < 1 || s->channels > 2)
		return -EINVAL;
	if ((s->flags & ~AMC_START_PS) || memchr_inv(s->reserved, 0, sizeof(s->reserved)))
		return -EINVAL;
	if ((s->flags & AMC_START_PS) && s->codec != AMC_CODEC_HEAAC)
		return -EINVAL;
	if (s->codec == AMC_CODEC_AUDB && s->rate != 8000 && s->rate != 16000)
		return -EINVAL;
	if (s->codec != AMC_CODEC_AUDB && s->codec != AMC_CODEC_MP3 && s->codec != AMC_CODEC_ALAC) {
		for (i = 0; i < ARRAY_SIZE(amc_rates) && amc_rates[i] != s->rate; i++)
			;
		if (i == ARRAY_SIZE(amc_rates) || s->rate > 48000)
			return -EINVAL;
	}
	if (s->codec == AMC_CODEC_AACENC) {
		if (!s->bitrate)
			s->bitrate = 128000;
		if (s->bitrate < 8000 * s->channels || s->bitrate > 320000)
			return -EINVAL;
	}
	ci = &amc_codecs[s->codec];
	if (amc_frames(s) * s->channels * 2 > ci->pcm_bytes)
		return -EINVAL;

	/* whatever happens below, the old stream (or a switch's copy) is gone */
	a->s.started = false;
	a->s.reload = false;
	/* each codec's firmware stays loaded: a restart or a switch reads no file */
	if (!a->fw[s->codec]) {
		ret = request_firmware(&fw, ci->fw, a->dev);
		if (ret)
			return ret;
		h = (const void *)fw->data;
		if (fw->size < sizeof(*h) || memcmp(h->magic, "AMCF", 4) ||
		    le32_to_cpu(h->version) != 1 || le32_to_cpu(h->type) != s->codec ||
		    le32_to_cpu(h->nmmu) != 18) {
			dev_err(a->dev, "%s: not an AMC firmware file for this codec\n", ci->fw);
			release_firmware(fw);
			return -EINVAL;
		}
		/* check both sections before touching the hardware */
		if (amc_fw_section(a, fw, &pos, false) || amc_fw_section(a, fw, &pos, false)) {
			dev_err(a->dev, "%s: bad section table\n", ci->fw);
			release_firmware(fw);
			return -EINVAL;
		}
		a->fw[s->codec] = fw;
	}
	fw = a->fw[s->codec];
	h = (const void *)fw->data;
	ret = amc_ranges(a, h);
	if (ret)
		return ret;

	a->s.max_div = amc_max_div(s->codec);
	ret = amc_clk_on(a);
	if (ret)
		return ret;
	div = clamp(DIV_ROUND_UP(a->bus_hz, S5L8720_AMC_MAX_HZ), 1UL, 16UL);
	amc_w(a, AMC_CLKDIV, div - 1);
	s5l8720_idle_amc_resync();	/* the system may be running at /2 */
	ret = amc_quiesce(a);
	if (ret)
		goto out;
	for (i = 0; i < 18; i++)
		amc_w(a, AMC_MMU(i), le32_to_cpu(h->mmu[i]));

	pos = sizeof(*h);
	amc_fw_section(a, fw, &pos, true);
	amc_w(a, AMC_START0, 0);
	ret = amc_wait_event(a, EV_DONE, 100000);
	if (ret) {
		dev_err(a->dev, "boot stub did not finish (status %08x)\n",
			amc_r(a, AMC_IRQ_STATUS));
		goto out;
	}
	amc_w(a, AMC_EVENT_ACK, EV_DONE);
	amc_fw_section(a, fw, &pos, true);
	amc_w(a, AMC_START1, 0);

	if (sram_r16(a, SRAM_HDR + HDR_NBUF) != ci->nbuf ||
	    sram_r16(a, SRAM_HDR + HDR_BUFLEN) * 2 != ci->pcm_bytes) {
		dev_err(a->dev, "firmware reports %u buffers of %u samples\n",
			sram_r16(a, SRAM_HDR + HDR_NBUF), sram_r16(a, SRAM_HDR + HDR_BUFLEN));
		ret = -EIO;
		goto out;
	}

	/* stale error state, then the events we use */
	memset_io(a->sram + SRAM_STATUS, 0, 8);
	amc_w(a, AMC_MBOX0_CLR, 1);
	amc_w(a, AMC_ERR_ACK, 1);
	amc_w(a, AMC_EVENT_ACK, EV_DONE);
	amc_w(a, AMC_IRQ_EN, EV_ERROR | EV_CH4 | EV_DONE);
	amc_write_params(a, s);
	amc_w(a, AMC_IN_SRC, 0);

	a->s.ci = ci;
	a->s.codec = s->codec;
	a->s.channels = s->channels;
	a->s.frames = amc_frames(s);
	a->s.buf = 0;
	a->s.started = true;
	a->s.cfg = *s;
	s->frames_per_packet = a->s.frames;
	ret = 0;
out:
	amc_clk_off(a);
	return ret;
}

/* ALAC: tell the DE the packet length through its register window */
static int amc_alac_length(struct amc *a, u32 bytes)
{
	u32 r[16], v;
	int i, ret;

	amc_w(a, AMC_WIN_SEL, 0x801);
	amc_w(a, AMC_WIN_REQ, 1);
	ret = readl_relaxed_poll_timeout_atomic(a->base + AMC_WIN_REQ, v, v == 1, 1, 100);
	if (ret)
		goto out;
	for (i = 0; i < 16; i++)
		r[i] = amc_r(a, AMC_WIN_REG(i));
	if (amc_r(a, AMC_WIN_STATUS) & WIN_ERR) {
		ret = -EIO;
		goto out;
	}
	r[0] |= 0x10000;
	r[2] = bytes << 3;
	for (i = 0; i < 16; i++)
		amc_w(a, AMC_WIN_REG(i), r[i]);
out:
	amc_w(a, AMC_WIN_REQ, 0);
	return ret;
}

#define AMC_PAD		64	/* zeros after the last packet of a stream */

static int amc_decode(struct amc *a, struct amc_decode *d)
{
	u32 *sizes = a->sizes, total = 0, frame_bytes, st;
	int ret = 0;
	u8 __user *out = u64_to_user_ptr(d->out);
	unsigned int k;

	d->packets_done = d->out_bytes = 0;
	d->error = 0;
	if (!a->s.started)
		return -ENODATA;
	if (a->s.codec == AMC_CODEC_AACENC)
		return -EINVAL;
	if (!d->npackets || d->npackets > AMC_MAX_PACKETS)
		return -EINVAL;
	if (copy_from_user(sizes, u64_to_user_ptr(d->sizes), d->npackets * sizeof(u32)))
		return -EFAULT;
	for (k = 0; k < d->npackets; k++) {
		if (!sizes[k] || sizes[k] > AMC_MAX_INPUT)
			return -EINVAL;
		total += sizes[k];
	}
	if (total > AMC_MAX_INPUT)
		return -EINVAL;
	/* only as many packets as out can take: the rest must not reach the FIFO */
	frame_bytes = a->s.frames * a->s.channels * 2;
	if (d->out_size < frame_bytes)
		return -ENOSPC;
	if (d->npackets > d->out_size / frame_bytes) {
		d->npackets = d->out_size / frame_bytes;
		d->flags &= ~AMC_DECODE_DRAIN;
		for (k = 0, total = 0; k < d->npackets; k++)
			total += sizes[k];
	}

	if (a->s.reload) {	/* after a deep sleep: same stream, fresh decoder */
		struct amc_start s = a->s.cfg;

		dev_dbg(a->dev, "reloading the codec after a deep sleep\n");
		ret = amc_start(a, &s);
		if (ret)
			return ret;
	}
	ret = amc_clk_on(a);
	if (ret)
		return ret;

	/* the previous call's input has gone into the FIFO by now */
	ret = readl_relaxed_poll_timeout(a->base + AMC_CH(4) + CH_STATUS, st,
					 !(st & 7), 100, 50000);
	if (ret) {
		dev_err(a->dev, "input DMA still busy (status %x)\n", st);
		goto fail;
	}
	if (copy_from_user(a->in, u64_to_user_ptr(d->in), total)) {
		amc_clk_off(a);
		return -EFAULT;
	}
	if (d->flags & AMC_DECODE_DRAIN) {
		memset(a->in + total, 0, AMC_PAD);
		total += AMC_PAD;
	}
	a->lli->next = 0;
	a->lli->ctrl = cpu_to_le32(LLI_CTRL(total));
	a->lli->src = cpu_to_le32(a->in_dma);
	a->lli->dst = 0;
	wmb();
	amc_w(a, AMC_CH(4) + CH_LLI, a->lli_dma | LLI_SYS);

	for (k = 0; k < d->npackets; k++) {
		unsigned int b, n, flag0, flag1;

		sram_w16(a, SRAM_HDR + HDR_FLAG(0), 0);
		if (a->s.ci->nbuf > 1)
			sram_w16(a, SRAM_HDR + HDR_FLAG(1), 0);
		if (a->s.codec == AMC_CODEC_ALAC) {
			ret = amc_alac_length(a, sizes[k]);
			if (ret)
				goto fail;
		}
		reinit_completion(&a->done);
		a->events = 0;
		amc_w(a, AMC_GO, 1);
		if (!wait_for_completion_timeout(&a->done, msecs_to_jiffies(500))) {
			dev_err(a->dev, "packet %u: no answer (status %08x, FIFO %x, DMA %x)\n",
				k, amc_r(a, AMC_IRQ_STATUS), amc_r(a, AMC_IN_LEVEL),
				amc_r(a, AMC_CH(4) + CH_STATUS));
			ret = -ETIMEDOUT;
			goto fail;
		}
		if (a->events & EV_ERROR) {
			if (sram_r16(a, SRAM_STATUS) & 1)
				d->error = sram_r16(a, SRAM_STATUS + 2);
			d->error = d->error ?: 100;
			dev_dbg(a->dev, "packet %u: decode error %d\n", k, d->error);
			ret = 0;
			goto fail;	/* the stream needs a new AMC_IOC_START */
		}
		flag0 = sram_r16(a, SRAM_HDR + HDR_FLAG(0));
		flag1 = a->s.ci->nbuf > 1 ? sram_r16(a, SRAM_HDR + HDR_FLAG(1)) : 0;
		b = !flag0 && flag1;
		n = frame_bytes;
		if (a->s.codec == AMC_CODEC_ALAC)
			n = min_t(unsigned int, sram_r16(a, SRAM_HDR + HDR_COUNT(0)) * 2, n);
		memcpy_fromio(a->pcm, a->sram + SRAM_PCM + b * a->s.ci->pcm_bytes, n);
		if (copy_to_user(out + d->out_bytes, a->pcm, n)) {
			ret = -EFAULT;
			goto fail;
		}
		d->out_bytes += n;
		d->packets_done++;
	}
	amc_clk_off(a);
	return ret;

fail:
	a->s.started = false;
	amc_quiesce(a);
	amc_clk_off(a);
	return ret;
}

/* channel 5: write what is left in the Stream Out FIFO to memory, then idle */
static int amc_out_flush(struct amc *a)
{
	u32 b = AMC_CH(5), st;
	int i, ret;

	/* iOS's cmd 0x80: halt, resume, flush, until the FIFO is empty */
	for (i = 0; i < 50 && (amc_r(a, AMC_OUT_LEVEL) & 0x1f) && (amc_r(a, b + CH_STATUS) & 7); i++) {
		amc_w(a, b + CH_CMD, CMD_HALT);
		ret = readl_relaxed_poll_timeout_atomic(a->base + b + CH_STATUS, st,
							(st & 7) == 7, 1, 1000);
		if (ret)
			return ret;
		amc_w(a, b + CH_CMD, CMD_RESUME);
		amc_w(a, b + CH_CMD, CMD_FLUSH);
		udelay(2);
	}
	if ((amc_r(a, AMC_OUT_LEVEL) & 0x1f)) {
		dev_err(a->dev, "Stream Out FIFO keeps %u bytes (DMA %x)\n",
			(amc_r(a, AMC_OUT_LEVEL) & 0x1f), amc_r(a, b + CH_STATUS));
		return -EIO;
	}
	return amc_ch_reset(a, 5);
}

/* Encode one 1024-frame block per 0x98c trigger, reading output via ch5 */
static int amc_encode(struct amc *a, struct amc_encode *e)
{
	u32 block = 1024 * a->s.channels * 2, total = 0, nb, k, st;
	int ret = 0;

	e->blocks_done = e->packets = e->out_bytes = 0;
	e->error = 0;
	e->notes = e->last_note = 0;
	if (!a->s.started)
		return -ENODATA;
	if (a->s.codec != AMC_CODEC_AACENC)
		return -EINVAL;
	if (!e->in_bytes || e->in_bytes % block || e->in_bytes / block > AMC_MAX_ENC_BLOCKS ||
	    e->flags)
		return -EINVAL;
	nb = e->in_bytes / block;
	if (e->out_size < nb * AMC_MAX_AU)
		return -ENOSPC;
	if (copy_from_user(a->in, u64_to_user_ptr(e->in), e->in_bytes))
		return -EFAULT;

	if (a->s.reload) {
		struct amc_start s = a->s.cfg;

		ret = amc_start(a, &s);
		if (ret)
			return ret;
	}
	ret = amc_clk_on(a);
	if (ret)
		return ret;

	/* the output list: room for every block's largest access unit */
	ret = readl_relaxed_poll_timeout(a->base + AMC_CH(5) + CH_STATUS, st, !(st & 7), 10, 10000);
	if (ret) {
		dev_err(a->dev, "output DMA busy (status %x)\n", st);
		goto fail;
	}
	a->lli5->next = 0;
	a->lli5->ctrl = cpu_to_le32(LLI_CTRL(nb * AMC_MAX_AU));
	/* the item's destination: word 2 per iOS (the source is the FIFO) */
	a->lli5->src = cpu_to_le32(a->eout_dma);
	a->lli5->dst = cpu_to_le32(a->eout_dma);
	wmb();
	amc_w(a, AMC_OUT_SRC, 0);
	amc_w(a, AMC_CH(5) + CH_LLI, a->lli5_dma | LLI_SYS);

	for (k = 0; k < nb; k++) {
		unsigned int b = a->s.buf, flags, code, bits;

		/*
		 * Transfer PCM input via DMA channel 0: system memory -> SRAM
		 * (space 1, DE-internal address: host 0x22028100 is internal
		 * 0x18100), in words.
		 */
		sram_w16(a, SRAM_HDR + HDR_COUNT(b), block / 2);
		sram_w16(a, SRAM_HDR + HDR_FLAG(b), 1);
		ret = amc_ch_reset(a, 0);
		if (ret)
			goto fail;
		a->lli->next = 0;
		a->lli->ctrl = cpu_to_le32((block / 4) << 18 | 0x1f << 7 | 0x10 | 1 << 2);
		a->lli->src = cpu_to_le32(a->in_dma + k * block);
		a->lli->dst = cpu_to_le32(SRAM_PCM - 0x10000 + b * a->s.ci->pcm_bytes);
		wmb();
		amc_w(a, AMC_CH(0) + CH_LLI, a->lli_dma | LLI_SYS);
		ret = readl_relaxed_poll_timeout(a->base + AMC_IRQ_STATUS, st, st & EV_CH0, 2, 20000);
		if (ret) {
			dev_err(a->dev, "block %u: PCM DMA did not finish (ch0 %x)\n",
				k, amc_r(a, AMC_CH(0) + CH_STATUS));
			goto fail;
		}
		amc_w(a, AMC_CH(0) + CH_CMD, CMD_ACK);
		reinit_completion(&a->done);
		a->events = 0;
		amc_w(a, AMC_GO, 1);
		if (!wait_for_completion_timeout(&a->done, msecs_to_jiffies(500))) {
			dev_err(a->dev, "block %u: no answer (status %08x, out FIFO %u, DMA %x)\n",
				k, amc_r(a, AMC_IRQ_STATUS), (amc_r(a, AMC_OUT_LEVEL) & 0x1f),
				amc_r(a, AMC_CH(5) + CH_STATUS));
			ret = -ETIMEDOUT;
			goto fail;
		}
		/* status block, u16: flags, error code (the host clears it), bits */
		flags = sram_r16(a, SRAM_STATUS);
		code = flags & 1 ? sram_r16(a, SRAM_STATUS + 2) : 0;
		if (code)
			sram_w16(a, SRAM_STATUS + 2, 0);
		bits = flags & 2 ? sram_r16(a, SRAM_STATUS + 4) : 0;
		if (bits > AMC_MAX_AU * 8) {
			dev_err(a->dev, "block %u: access unit of %u bits\n", k, bits);
			ret = -EIO;
			goto fail;
		}
		/*
		 * Record non-fatal status codes while keeping the access unit;
		 * only an unconsumed buffer flag or EV_ERROR stops the stream.
		 */
		if (code) {
			e->notes++;
			e->last_note = code;
		}
		if ((a->events & EV_ERROR) || sram_r16(a, SRAM_HDR + HDR_FLAG(b))) {
			dev_err(a->dev, "block %u: encoder stopped (code %u, events %08x, buffer %u %s)\n",
				e->blocks_done, code, a->events, b,
				sram_r16(a, SRAM_HDR + HDR_FLAG(b)) ? "not taken" : "taken");
			e->error = code ?: 100;
			ret = 0;
			goto fail;
		}
		if (bits) {
			a->esizes[e->packets++] = DIV_ROUND_UP(bits, 8);
			total += DIV_ROUND_UP(bits, 8);
		}
		e->blocks_done++;
		a->s.buf ^= 1;
	}
	st = amc_r(a, AMC_CH(5) + 0x0c);
	ret = amc_out_flush(a);
	if (ret)
		goto fail;
	dev_dbg(a->dev, "%u blocks: %u units, %u bytes (ch5 at %08x, buffer %pad)\n",
		nb, e->packets, total, st, &a->eout_dma);
	amc_clk_off(a);
	if (copy_to_user(u64_to_user_ptr(e->out), a->eout, total) ||
	    copy_to_user(u64_to_user_ptr(e->sizes), a->esizes, e->packets * sizeof(u32)))
		return -EFAULT;
	e->out_bytes = total;
	return 0;

fail:
	a->s.started = false;
	amc_quiesce(a);
	amc_clk_off(a);
	return ret;
}

#define AMC_MAX_USERS	4

/*
 * The DE cannot run the encoder and a decoder together in real time
 * (120 s of audio, device-measured: stereo AAC encoding 88 s, HE-AAC
 * decoding 92 s, AAC-LC 36 s, ALAC 34 s). Whoever has a stand-in of the
 * same quality on the CPU gives way: AAC-LC and MP3 decoders (fdk-aac,
 * mpg123: the same output) to the encoder; the encoder (fdk-aac encodes
 * on the CPU) to HE-AAC (the device's fdk-aac has no SBR decoder), ALAC
 * and Audible decoders. The one giving way gets -EBUSY, also mid-stream.
 * Decoders time-share with each other.
 */
static bool amc_has_cpu_twin(unsigned int codec)
{
	return codec == AMC_CODEC_AAC || codec == AMC_CODEC_MP3 || codec == AMC_CODEC_AACENC;
}

static bool amc_yield(struct amc *a, struct amc_ctx *c, unsigned int codec)
{
	bool enc = codec == AMC_CODEC_AACENC;
	struct amc_ctx *o;

	if (!amc_has_cpu_twin(codec))
		return false;
	list_for_each_entry(o, &a->ctxs, node) {
		const struct amc_stream *st = o == a->owner ? &a->s : &o->s;

		if (o == c || !st->started)
			continue;
		if (enc ? !amc_has_cpu_twin(st->codec) : st->codec == AMC_CODEC_AACENC)
			return true;
	}
	return false;
}

/*
 * ALAC alone selects a bank of the DE's register window (0x40c = 0x801,
 * per packet), and the bank the other codecs use is not known, so a
 * window saved after ALAC ran might come from the wrong bank: ALAC does
 * not share the coprocessor with other decoders. A second decoder of
 * either kind gets -EBUSY (an AAC-LC or MP3 one decodes on the CPU).
 */
static bool amc_alac_conflict(struct amc *a, struct amc_ctx *c, unsigned int codec)
{
	struct amc_ctx *o;

	if (codec == AMC_CODEC_AACENC)
		return false;
	list_for_each_entry(o, &a->ctxs, node) {
		const struct amc_stream *st = o == a->owner ? &a->s : &o->s;

		if (o != c && st->started && st->codec != AMC_CODEC_AACENC &&
		    (st->codec == AMC_CODEC_ALAC) != (codec == AMC_CODEC_ALAC))
			return true;
	}
	return false;
}

static long amc_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct amc_ctx *c = f->private_data;
	struct amc *a = c->a;
	void __user *p = (void __user *)arg;
	union {
		struct amc_start s;
		struct amc_decode d;
		struct amc_encode e;
	} u;
	long ret;

	switch (cmd) {
	case AMC_IOC_START:
		if (copy_from_user(&u.s, p, sizeof(u.s)))
			return -EFAULT;
		mutex_lock(&a->lock);
		if (amc_yield(a, c, u.s.codec) || amc_alac_conflict(a, c, u.s.codec)) {
			ret = -EBUSY;
		} else {
			amc_switch(a, c, true);
			ret = amc_start(a, &u.s);
		}
		mutex_unlock(&a->lock);
		if (!ret && copy_to_user(p, &u.s, sizeof(u.s)))
			ret = -EFAULT;
		return ret;
	case AMC_IOC_DECODE:
		if (copy_from_user(&u.d, p, sizeof(u.d)))
			return -EFAULT;
		if ((u.d.flags & ~AMC_DECODE_DRAIN) ||
		    memchr_inv(u.d.reserved, 0, sizeof(u.d.reserved)))
			return -EINVAL;
		mutex_lock(&a->lock);
		if (amc_yield(a, c, (a->owner == c ? &a->s : &c->s)->codec)) {
			ret = -EBUSY;
		} else {
			amc_switch(a, c, false);
			ret = amc_decode(a, &u.d);
		}
		mutex_unlock(&a->lock);
		if (!ret && copy_to_user(p, &u.d, sizeof(u.d)))
			ret = -EFAULT;
		return ret;
	case AMC_IOC_ENCODE:
		if (copy_from_user(&u.e, p, sizeof(u.e)))
			return -EFAULT;
		if (u.e.flags || memchr_inv(u.e.reserved, 0, sizeof(u.e.reserved)))
			return -EINVAL;
		mutex_lock(&a->lock);
		if (amc_yield(a, c, AMC_CODEC_AACENC)) {
			ret = -EBUSY;
		} else {
			amc_switch(a, c, false);
			ret = amc_encode(a, &u.e);
		}
		mutex_unlock(&a->lock);
		if (!ret && copy_to_user(p, &u.e, sizeof(u.e)))
			ret = -EFAULT;
		return ret;
	}
	return -ENOTTY;
}

/* several users (a player's decoder and Bluetooth's encoder): one context each */
static int amc_open(struct inode *inode, struct file *f)
{
	struct amc *a = container_of(f->private_data, struct amc, misc);
	struct amc_ctx *c;
	int ret;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->a = a;
	mutex_lock(&a->lock);
	ret = -EBUSY;
	if (a->users == AMC_MAX_USERS)
		goto out;
	ret = clk_bulk_prepare(a->nclks, a->clks);
	if (ret)
		goto out;
	a->users++;
	list_add_tail(&c->node, &a->ctxs);
	f->private_data = c;
out:
	mutex_unlock(&a->lock);
	if (ret)
		kfree(c);
	return ret;
}

static int amc_release(struct inode *inode, struct file *f)
{
	struct amc_ctx *c = f->private_data;
	struct amc *a = c->a;

	mutex_lock(&a->lock);
	if (a->owner == c) {
		if (!amc_clk_on(a)) {
			amc_quiesce(a);
			amc_clk_off(a);
		}
		a->s.started = false;
		a->s.reload = false;
		a->owner = NULL;
	}
	clk_bulk_unprepare(a->nclks, a->clks);
	list_del(&c->node);
	a->users--;
	mutex_unlock(&a->lock);
	kvfree(c->save);
	kfree(c);
	return 0;
}

static const struct file_operations amc_fops = {
	.owner = THIS_MODULE,
	.open = amc_open,
	.release = amc_release,
	.unlocked_ioctl = amc_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static int amc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct amc *a;
	u32 uid, hz = 133000000;
	int irq, ret;

	a = devm_kzalloc(dev, sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	a->dev = dev;
	mutex_init(&a->lock);
	INIT_LIST_HEAD(&a->ctxs);
	init_completion(&a->done);
	a->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(a->base))
		return PTR_ERR(a->base);
	a->sram = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(a->sram))
		return PTR_ERR(a->sram);
	of_property_read_u32(dev->of_node, "apple,bus-frequency", &hz);
	a->bus_hz = hz;
	a->nclks = devm_clk_bulk_get_all(dev, &a->clks);
	if (a->nclks < 0)
		return a->nclks;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	a->in = dmam_alloc_coherent(dev, AMC_MAX_INPUT + AMC_PAD, &a->in_dma, GFP_KERNEL);
	a->lli = dmam_alloc_coherent(dev, sizeof(*a->lli), &a->lli_dma, GFP_KERNEL);
	a->pcm = devm_kmalloc(dev, 0x4000, GFP_KERNEL);
	a->eout = dmam_alloc_coherent(dev, AMC_MAX_ENC_BLOCKS * AMC_MAX_AU, &a->eout_dma, GFP_KERNEL);
	a->lli5 = dmam_alloc_coherent(dev, sizeof(*a->lli5), &a->lli5_dma, GFP_KERNEL);
	if (!a->in || !a->lli || !a->pcm || !a->eout || !a->lli5)
		return -ENOMEM;

	/* identify, then leave it masked and gated */
	ret = clk_bulk_prepare_enable(a->nclks, a->clks);
	if (ret)
		return ret;
	uid = amc_r(a, AMC_UID);
	amc_w(a, AMC_IRQ_DIS, 0x07ffffff);
	amc_w(a, AMC_IRQ1_DIS, 0xffffffff);
	clk_bulk_disable_unprepare(a->nclks, a->clks);
	if (uid != AMC_UID_VALUE) {
		dev_err(dev, "unexpected Data Engine UID %08x\n", uid);
		return -ENODEV;
	}

	ret = devm_request_irq(dev, irq, amc_irq, 0, dev_name(dev), a);
	if (ret)
		return ret;

	a->misc.minor = MISC_DYNAMIC_MINOR;
	a->misc.name = "amc";
	a->misc.fops = &amc_fops;
	a->misc.parent = dev;
	ret = misc_register(&a->misc);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, a);
	dev_info(dev, "audio coprocessor, Data Engine %08x\n", uid);
	return 0;
}

static void amc_remove(struct platform_device *pdev)
{
	struct amc *a = platform_get_drvdata(pdev);

	misc_deregister(&a->misc);
	for (unsigned int i = 0; i < ARRAY_SIZE(a->fw); i++)
		release_firmware(a->fw[i]);
}

/*
 * Between bursts the block is gated already, and tasks are frozen with no
 * call in flight. Suspend to idle keeps the SRAM; anything deeper powers
 * the SoC off: the loaded stream is saved like for a switch and restored
 * before its next packet (if that fails, it starts over).
 */
static int amc_suspend(struct device *dev)
{
	struct amc *a = dev_get_drvdata(dev);

	mutex_lock(&a->lock);
	WARN_ON(a->clk_on);
	if (a->owner && pm_suspend_target_state != PM_SUSPEND_TO_IDLE) {
		if (amc_save(a, a->owner))
			a->owner->s.reload = a->owner->s.started;
		a->owner = NULL;
	}
	mutex_unlock(&a->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(amc_pm_ops, amc_suspend, NULL);

static const struct of_device_id amc_of_match[] = {
	{ .compatible = "apple,s5l8720-amc" },
	{ }
};
MODULE_DEVICE_TABLE(of, amc_of_match);

static struct platform_driver amc_driver = {
	.probe = amc_probe,
	.remove = amc_remove,
	.driver = {
		.name = "s5l8720-amc",
		.of_match_table = amc_of_match,
		.suppress_bind_attrs = true,	/* open files would outlive it */
		.pm = pm_sleep_ptr(&amc_pm_ops),
	},
};
module_platform_driver(amc_driver);

MODULE_DESCRIPTION("S5L8720 AMC audio coprocessor (hardware audio decoding)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("apple/amc-mp3.bin");
MODULE_FIRMWARE("apple/amc-aac.bin");
MODULE_FIRMWARE("apple/amc-heaac.bin");
MODULE_FIRMWARE("apple/amc-alac.bin");
MODULE_FIRMWARE("apple/amc-audb.bin");
MODULE_FIRMWARE("apple/amc-aacenc.bin");
