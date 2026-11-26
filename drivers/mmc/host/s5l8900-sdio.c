// SPDX-License-Identifier: GPL-2.0-only
/*
 * SDIO host controller of the Samsung/Apple S5L8900 and S5L8720 (iPhone 3G,
 * iPod touch 2G), which connects the BCM4325 Wi-Fi chip.
 *
 * Commands are polled; data goes through the controller's own DMA (BADDR)
 * from a coherent bounce buffer. Card interrupts (SDIO IRQ) are forwarded
 * to the MMC core.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>

#define SDIO_CTRL		0x00
#define  CTRL_ENABLE		BIT(0)
#define  CTRL_4BIT		BIT(2)
/* Enable with 0x41 on S5L8720 (and S5L8900 boards needing CTRL_UNTERM_CLK). */
#define  CTRL_UNTERM_CLK	BIT(6)
#define  CTRL_BLOCK		BIT(15)
#define SDIO_DCTRL		0x04
#define  DCTRL_FIFO_RESET	0x3
#define  DCTRL_START		0x10
#define SDIO_CMD		0x08
#define  CMD_RESP_SHIFT		6	/* 1: response, 2: addressed, 3: addressed + data */
#define  CMD_WRITE		BIT(8)
#define  CMD_RTYPE_SHIFT	16	/* response type number: 1, 4, 5, 6 ... */
#define  CMD_EXECUTE		BIT(31)
#define SDIO_ARGU		0x0c
#define SDIO_STATE		0x10	/* 0: command and data state machines idle */
#define  STATE_CMD		GENMASK(6, 4)
#define SDIO_STAC		0x14	/* write the DSTA value to acknowledge */
#define SDIO_DSTA		0x18
#define  DSTA_CMD_READY		BIT(0)
#define  DSTA_CMD_SENT		BIT(2)
#define  DSTA_STATUS_SHIFT	15	/* error/status flags */
#define  STATUS_ERR_MASK	0xf
#define  STATUS_CRC		BIT(3)
/* Data transfer CRC error status bits */
#define  DSTA_DATA_CRC_WRITE	BIT(22)
#define  DSTA_DATA_CRC_READ	BIT(23)
#define SDIO_FSTA		0x1c
#define SDIO_RESP(n)		(0x20 + (n) * 4)
#define SDIO_CLKDIV		0x30
#define SDIO_CSR		0x34
#define  CSR_SDIO_IRQ_EN	BIT(1)
#define SDIO_IRQ		0x38	/* write 1 to clear */
#define  IRQ_DATA_DONE		BIT(0)
/*
 * The card's interrupt, latched when DAT1 falls (and at the end of a data
 * transfer that finds it low), not while it is low. Bit 4 is not the card:
 * it sets after every command, and is left masked.
 */
#define  IRQ_CARD		BIT(1)
#define SDIO_IRQMASK		0x3c
#define SDIO_BADDR		0x44
#define SDIO_BLKLEN		0x48
#define SDIO_NUMBLK		0x4c
#define SDIO_REMBLK		0x50

#define BOUNCE_SIZE		SZ_64K
/*
 * Hardware DMA erratum: outbound write buffer must start at a physical
 * address whose low five bits are all ones (32-byte aligned buffers corrupt
 * bytes at 25.6 MHz with valid CRCs).
 */
#define WRITE_OFFSET		0x1f
#define CMD_TIMEOUT_US		100000
#define DATA_TIMEOUT_US		500000

struct s5l_sdio {
	struct mmc_host *mmc;
	void __iomem *base;
	void __iomem *gate;
	u32 gate_mask;
	struct clk *clk;	/* the SDIO gate ("clocks") */
	unsigned long pclk;
	void *bounce;
	dma_addr_t bounce_dma;
	spinlock_t irq_lock;
	bool sdio_irq_on;
	u32 ctrl_init;		/* SDIO_CTRL at enable */
	u32 sv_ctrl, sv_clkdiv, sv_irqmask, sv_csr;	/* registers across suspend to RAM */
	unsigned int crc_errors;	/* data transfers that ended in a CRC error */
	struct completion data_done;	/* IRQ_DATA_DONE, for a wait past the spin */
};

static u32 rd(struct s5l_sdio *h, u32 reg) { return readl(h->base + reg); }
static void wr(struct s5l_sdio *h, u32 reg, u32 v) { writel(v, h->base + reg); }

/*
 * Response class (bits 6-7: 1 response, 2 addressed, 3 addressed + data) and
 * response type number (bits 16+) in this controller's encoding:
 * CMD0 0, CMD5 1/4, CMD3 1/6, CMD7 2/1, CMD52 2/5, CMD53 3/5.
 */
static u32 s5l_cmd_bits(struct mmc_command *cmd, bool data, bool write)
{
	u32 v = cmd->opcode & 0x3f;
	u32 rtype;

	switch (cmd->opcode) {
	case SD_IO_SEND_OP_COND:
		rtype = 4;
		break;
	case SD_SEND_RELATIVE_ADDR:
		rtype = 6;
		break;
	case SD_IO_RW_DIRECT:
	case SD_IO_RW_EXTENDED:
		rtype = 5;
		break;
	default:
		rtype = cmd->flags & MMC_RSP_136 ? 2 : 1;
	}
	if (!(cmd->flags & MMC_RSP_PRESENT))
		rtype = 0;

	if (data)
		v |= 3 << CMD_RESP_SHIFT;
	else if (cmd->opcode == MMC_SELECT_CARD || cmd->opcode == SD_IO_RW_DIRECT)
		v |= 2 << CMD_RESP_SHIFT;
	else if (rtype)
		v |= 1 << CMD_RESP_SHIFT;
	if (write)
		v |= CMD_WRITE;
	return v | rtype << CMD_RTYPE_SHIFT;
}

/*
 * Wait for (reg & mask) == want. A command and its response are 96 bit
 * times, 4 us at 25.6 MHz, so spin for a moment first (by the clock) and
 * only then sleep between looks: four sleeping polls per command, each a
 * timer interrupt and two context switches, made brcmfmac's 100 status
 * polls a second ~1,300 timer events and 4 % of the CPU.
 */
#define S5L_SPIN_US	80

static int s5l_wait(struct s5l_sdio *h, u32 reg, u32 mask, u32 want, unsigned int timeout_us)
{
	ktime_t end = ktime_add_us(ktime_get(), S5L_SPIN_US);
	u32 v;

	for (;;) {
		if ((rd(h, reg) & mask) == want)
			return 0;
		if (ktime_after(ktime_get(), end))
			break;
		cpu_relax();
	}
	return readl_poll_timeout(h->base + reg, v, (v & mask) == want, 20, timeout_us);
}

/*
 * Wait for a data transfer: spin as s5l_wait does, then sleep until the
 * controller's data-done interrupt. A transfer can take a millisecond (the
 * card waking its backplane clock for brcmfmac's status read after a card
 * interrupt); polled at 20 us that was some 20 timer interrupts and context
 * switches per transfer, most of the idle system's timer interrupts.
 */
static int s5l_wait_data(struct s5l_sdio *h)
{
	ktime_t end = ktime_add_us(ktime_get(), S5L_SPIN_US);
	unsigned long fl;
	int ret = 0;

	for (;;) {
		if (rd(h, SDIO_IRQ) & IRQ_DATA_DONE)
			return 0;
		if (ktime_after(ktime_get(), end))
			break;
		cpu_relax();
	}
	reinit_completion(&h->data_done);
	spin_lock_irqsave(&h->irq_lock, fl);
	wr(h, SDIO_IRQMASK, rd(h, SDIO_IRQMASK) | IRQ_DATA_DONE);
	spin_unlock_irqrestore(&h->irq_lock, fl);
	/* (done before the unmask: the status is latched, the line rises now) */
	if (!(rd(h, SDIO_IRQ) & IRQ_DATA_DONE) &&
	    !wait_for_completion_timeout(&h->data_done, usecs_to_jiffies(DATA_TIMEOUT_US)) &&
	    !(rd(h, SDIO_IRQ) & IRQ_DATA_DONE))
		ret = -ETIMEDOUT;
	spin_lock_irqsave(&h->irq_lock, fl);
	wr(h, SDIO_IRQMASK, rd(h, SDIO_IRQMASK) & ~IRQ_DATA_DONE);
	spin_unlock_irqrestore(&h->irq_lock, fl);
	return ret;
}

/* Send one command and wait for it; returns the status flags (DSTA >> 15). */
static int s5l_run_cmd(struct s5l_sdio *h, struct mmc_command *cmd, u32 bits, u32 *flags)
{
	int ret;

	ret = s5l_wait(h, SDIO_STATE, ~0u, 0, CMD_TIMEOUT_US);
	if (ret)
		return ret;
	wr(h, SDIO_STAC, rd(h, SDIO_DSTA));
	wr(h, SDIO_ARGU, cmd->arg);
	wr(h, SDIO_CMD, bits);
	ret = s5l_wait(h, SDIO_DSTA, DSTA_CMD_READY, DSTA_CMD_READY, CMD_TIMEOUT_US);
	if (ret)
		return ret;
	wr(h, SDIO_CMD, bits | CMD_EXECUTE);
	ret = s5l_wait(h, SDIO_DSTA, DSTA_CMD_SENT, DSTA_CMD_SENT, CMD_TIMEOUT_US);
	if (ret)
		return ret;
	ret = s5l_wait(h, SDIO_STATE, STATE_CMD, 0, CMD_TIMEOUT_US);
	if (ret)
		return ret;
	*flags = rd(h, SDIO_DSTA) >> DSTA_STATUS_SHIFT;
	return 0;
}

static void s5l_read_resp(struct s5l_sdio *h, struct mmc_command *cmd)
{
	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = rd(h, SDIO_RESP(3));
		cmd->resp[1] = rd(h, SDIO_RESP(2));
		cmd->resp[2] = rd(h, SDIO_RESP(1));
		cmd->resp[3] = rd(h, SDIO_RESP(0));
	} else {
		cmd->resp[0] = rd(h, SDIO_RESP(0));
	}
}

static int s5l_cmd_error(struct mmc_command *cmd, u32 flags)
{
	u32 err = flags & STATUS_ERR_MASK;

	/* CMD5's R4 has no CRC: ignore the CRC flag */
	if (cmd->opcode == SD_IO_SEND_OP_COND || !(cmd->flags & MMC_RSP_CRC))
		err &= ~STATUS_CRC;
	if (!err)
		return 0;
	return err & STATUS_CRC ? -EILSEQ : -ETIMEDOUT;
}

static int s5l_do_data(struct s5l_sdio *h, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;
	bool write = data->flags & MMC_DATA_WRITE;
	size_t len = data->blksz * data->blocks;
	u32 flags;
	int ret;

	if (len > BOUNCE_SIZE - 32)
		return -EINVAL;
	if (write)
		sg_copy_to_buffer(data->sg, data->sg_len, h->bounce + WRITE_OFFSET, len);

	wr(h, SDIO_BADDR, h->bounce_dma + (write ? WRITE_OFFSET : 0));
	wr(h, SDIO_BLKLEN, data->blksz);
	wr(h, SDIO_NUMBLK, data->blocks);
	if (data->blocks > 1 || (mrq->cmd->arg & BIT(27)))	/* CMD53 block mode */
		wr(h, SDIO_CTRL, rd(h, SDIO_CTRL) | CTRL_BLOCK);
	else
		wr(h, SDIO_CTRL, rd(h, SDIO_CTRL) & ~CTRL_BLOCK);
	wr(h, SDIO_DCTRL, DCTRL_FIFO_RESET);
	wr(h, SDIO_DCTRL, 0);
	ret = s5l_run_cmd(h, mrq->cmd, s5l_cmd_bits(mrq->cmd, true, write), &flags);
	if (ret)
		goto out;
	s5l_read_resp(h, mrq->cmd);
	ret = s5l_cmd_error(mrq->cmd, flags);
	if (ret)
		goto out;

	wr(h, SDIO_STAC, rd(h, SDIO_DSTA));
	wr(h, SDIO_DCTRL, DCTRL_START);
	ret = s5l_wait_data(h);
	wr(h, SDIO_IRQ, IRQ_DATA_DONE);
	if (!ret) {
		u32 dsta = rd(h, SDIO_DSTA);

		wr(h, SDIO_STAC, dsta);
		/* bits 4-6 hold the data state (2: done); low bits are errors */
		flags = dsta >> DSTA_STATUS_SHIFT;
		if (flags & STATUS_ERR_MASK)
			ret = flags & STATUS_CRC ? -EILSEQ : -EIO;
		else if (dsta & (DSTA_DATA_CRC_WRITE | DSTA_DATA_CRC_READ))
			ret = -EILSEQ;
		if (ret == -EILSEQ)
			h->crc_errors++;
	}
	if (!ret) {
		data->bytes_xfered = len;
		if (!write)
			sg_copy_from_buffer(data->sg, data->sg_len, h->bounce, len);
	}
out:
	wr(h, SDIO_CTRL, rd(h, SDIO_CTRL) & ~CTRL_BLOCK);
	return ret;
}

static void s5l_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct s5l_sdio *h = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	u32 flags;
	int ret;

	if (mrq->data) {
		ret = s5l_do_data(h, mrq);
		if (ret == -EILSEQ || ret == -ETIMEDOUT)
			mrq->data->error = ret;
		else
			cmd->error = ret;
	} else {
		ret = s5l_run_cmd(h, cmd, s5l_cmd_bits(cmd, false, false), &flags);
		if (!ret) {
			s5l_read_resp(h, cmd);
			ret = s5l_cmd_error(cmd, flags);
		}
		cmd->error = ret;
	}
	mmc_request_done(mmc, mrq);
}

static void s5l_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct s5l_sdio *h = mmc_priv(mmc);
	u32 div;

	if (ios->power_mode == MMC_POWER_UP && !IS_ERR(mmc->supply.vmmc))
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
	else if (ios->power_mode == MMC_POWER_OFF && !IS_ERR(mmc->supply.vmmc))
		mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

	if (ios->clock) {
		div = DIV_ROUND_UP(h->pclk, ios->clock);
		div = roundup_pow_of_two(clamp(div, 1u, 128u));	/* power of two divider, max 128 */
		wr(h, SDIO_CLKDIV, div);
	}
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		wr(h, SDIO_CTRL, rd(h, SDIO_CTRL) | CTRL_4BIT);
	else
		wr(h, SDIO_CTRL, rd(h, SDIO_CTRL) & ~CTRL_4BIT);
}

/*
 * Have the controller look at DAT1 now. Its card interrupt is latched on an
 * edge, so an interrupt the card raised (or kept up) before the status bit
 * was cleared would wait for the next edge, which does not come while the
 * card waits to be served. Switching the interrupt enable off and on with
 * the controller in 1-bit mode latches a DAT1 that is low at that moment;
 * in 4-bit mode it does not, and stops detection altogether. Needed after
 * every interrupt on S5L8720. No command may be running (callers hold host).
 */
static void s5l_sample_card_irq(struct s5l_sdio *h)
{
	u32 ctrl = rd(h, SDIO_CTRL);
	u32 csr = rd(h, SDIO_CSR);

	wr(h, SDIO_CTRL, ctrl & ~CTRL_4BIT);
	wr(h, SDIO_CSR, csr & ~CSR_SDIO_IRQ_EN);
	wr(h, SDIO_CSR, csr | CSR_SDIO_IRQ_EN);
	wr(h, SDIO_CTRL, ctrl);
}

static void s5l_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct s5l_sdio *h = mmc_priv(mmc);
	unsigned long fl;

	spin_lock_irqsave(&h->irq_lock, fl);
	h->sdio_irq_on = enable;
	if (enable) {
		s5l_sample_card_irq(h);
		wr(h, SDIO_IRQMASK, rd(h, SDIO_IRQMASK) | IRQ_CARD);
	} else {
		wr(h, SDIO_IRQMASK, rd(h, SDIO_IRQMASK) & ~IRQ_CARD);
	}
	spin_unlock_irqrestore(&h->irq_lock, fl);
}

/*
 * The core has run the card's handlers. The status bit is still the one
 * that brought us here: clear it, and since the card may have raised its
 * interrupt again after the handlers last looked (no new edge then, or one
 * that this clear has just wiped out), sample the line before unmasking.
 */
static void s5l_ack_sdio_irq(struct mmc_host *mmc)
{
	struct s5l_sdio *h = mmc_priv(mmc);
	unsigned long fl;

	spin_lock_irqsave(&h->irq_lock, fl);
	if (h->sdio_irq_on) {
		wr(h, SDIO_IRQ, IRQ_CARD);
		s5l_sample_card_irq(h);
		wr(h, SDIO_IRQMASK, rd(h, SDIO_IRQMASK) | IRQ_CARD);
	}
	spin_unlock_irqrestore(&h->irq_lock, fl);
}

static irqreturn_t s5l_irq(int irq, void *data)
{
	struct s5l_sdio *h = data;
	u32 mask;

	u32 st;

	spin_lock(&h->irq_lock);
	mask = rd(h, SDIO_IRQMASK);
	st = rd(h, SDIO_IRQ) & mask;
	if (!(st & (IRQ_CARD | IRQ_DATA_DONE))) {
		spin_unlock(&h->irq_lock);
		return IRQ_NONE;
	}
	/*
	 * each masked, the status left alone: the waiter (data done) or the
	 * core's ack (the card) takes it
	 */
	wr(h, SDIO_IRQMASK, mask & ~st);
	spin_unlock(&h->irq_lock);
	if (st & IRQ_DATA_DONE)
		complete(&h->data_done);
	if (st & IRQ_CARD)
		sdio_signal_irq(h->mmc);
	return IRQ_HANDLED;
}

static const struct mmc_host_ops s5l_ops = {
	.request = s5l_request,
	.set_ios = s5l_set_ios,
	.enable_sdio_irq = s5l_enable_sdio_irq,
	.ack_sdio_irq = s5l_ack_sdio_irq,
};

static int s5l_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmc_host *mmc;
	struct s5l_sdio *h;
	struct resource *r;
	u32 pclk = 51200000;
	int irq, ret;

	mmc = devm_mmc_alloc_host(dev, sizeof(*h));
	if (!mmc)
		return -ENOMEM;
	h = mmc_priv(mmc);
	h->mmc = mmc;
	spin_lock_init(&h->irq_lock);
	init_completion(&h->data_done);

	h->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(h->base))
		return PTR_ERR(h->base);
	/* clock gate (iOS device-clocks "SDIO": 0x3c500048 bit 8) */
	h->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(h->clk))
		return dev_err_probe(dev, PTR_ERR(h->clk), "clock\n");
	r = h->clk ? NULL : platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (r) {
		h->gate = devm_ioremap(dev, r->start, resource_size(r));
		if (!h->gate || of_property_read_u32(dev->of_node, "apple,gate-mask",
						     &h->gate_mask))
			return -EINVAL;
		writel(readl(h->gate) & ~h->gate_mask, h->gate);
	}
	of_property_read_u32(dev->of_node, "clock-frequency", &pclk);
	h->pclk = pclk;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	h->bounce = dmam_alloc_coherent(dev, BOUNCE_SIZE, &h->bounce_dma, GFP_KERNEL);
	if (!h->bounce)
		return -ENOMEM;

	/* reset the FIFO, enable at 1 bit, clear status */
	wr(h, SDIO_CLKDIV, 128);
	wr(h, SDIO_DCTRL, DCTRL_FIFO_RESET);
	wr(h, SDIO_DCTRL, 0);
	h->ctrl_init = CTRL_ENABLE;
	if (of_device_is_compatible(dev->of_node, "apple,s5l8720-sdio"))
		h->ctrl_init |= CTRL_UNTERM_CLK;
	wr(h, SDIO_CTRL, h->ctrl_init);
	wr(h, SDIO_STAC, 0xffffffff);
	wr(h, SDIO_IRQ, 0xffffffff);
	wr(h, SDIO_IRQMASK, 0);	/* data done: unmasked only for a long wait */
	wr(h, SDIO_CSR, CSR_SDIO_IRQ_EN);

	ret = devm_request_irq(dev, irq, s5l_irq, 0, dev_name(dev), h);
	if (ret)
		return ret;

	mmc->ops = &s5l_ops;
	mmc->f_min = DIV_ROUND_UP(pclk, 128);
	mmc->f_max = pclk / 2;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ | MMC_CAP_NONREMOVABLE;
	mmc->caps2 = MMC_CAP2_NO_SD | MMC_CAP2_NO_MMC | MMC_CAP2_SDIO_IRQ_NOTHREAD;
	mmc->max_segs = 128;
	mmc->max_seg_size = BOUNCE_SIZE / 2;	/* room for WRITE_OFFSET */
	mmc->max_req_size = BOUNCE_SIZE / 2;
	mmc->max_blk_size = 2048;
	mmc->max_blk_count = BOUNCE_SIZE / 2 / 512;

	ret = mmc_of_parse(mmc);
	if (ret)
		return ret;
	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, h);
	ret = mmc_add_host(mmc);
	if (ret)
		return ret;
	debugfs_create_u32("crc_errors", 0444, mmc->debugfs_root, &h->crc_errors);
	dev_info(dev, "S5L8900-style SDIO host, %u Hz bus clock\n", pclk);
	return 0;
}

static void s5l_remove(struct platform_device *pdev)
{
	struct s5l_sdio *h = platform_get_drvdata(pdev);

	mmc_remove_host(h->mmc);
	wr(h, SDIO_IRQMASK, 0);
	wr(h, SDIO_CTRL, 0);
}

static const struct of_device_id s5l_sdio_of_match[] = {
	{ .compatible = "apple,s5l8900-sdio" },
	{ .compatible = "apple,s5l8720-sdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l_sdio_of_match);

/*
 * The controller's clock stays on across system sleep and the card keeps
 * its power (keep-power-in-suspend). Gating the clock for the sleep, as
 * iOS does with an idle bus, was tried on the N72 (suspend to idle, ten
 * alternating runs): Wi-Fi answered 47 of 60 pings after the resume with
 * the gate and 58 of 60 without.
 */
static int s5l_sdio_suspend_noirq(struct device *dev)
{
	struct s5l_sdio *h = dev_get_drvdata(dev);

	/*
	 * Suspend to RAM powers the SoC down and the controller comes back in
	 * its reset state, while the card keeps its power and its bus setup
	 * (the core does not call set_ios again for a card kept powered).
	 * Remember the registers.
	 */
	h->sv_ctrl = rd(h, SDIO_CTRL);
	h->sv_clkdiv = rd(h, SDIO_CLKDIV);
	h->sv_irqmask = rd(h, SDIO_IRQMASK);
	h->sv_csr = rd(h, SDIO_CSR);
	return 0;
}

static int s5l_sdio_resume_noirq(struct device *dev)
{
	struct s5l_sdio *h = dev_get_drvdata(dev);

	if ((rd(h, SDIO_CTRL) & CTRL_ENABLE) && rd(h, SDIO_CTRL) == h->sv_ctrl)
		return 0;

	/*
	 * The state was lost: redo what iOS's powerOnSequence does for a
	 * resume without a card reset (divider, FIFO reset, control, clear
	 * status, then the 4-bit flag, interrupt mask and CSR), with the
	 * values from before the sleep.
	 */
	wr(h, SDIO_CLKDIV, h->sv_clkdiv);
	wr(h, SDIO_DCTRL, DCTRL_FIFO_RESET);
	wr(h, SDIO_DCTRL, 0);
	wr(h, SDIO_CTRL, h->sv_ctrl & ~CTRL_BLOCK);
	wr(h, SDIO_STAC, 0xffffffff);
	wr(h, SDIO_IRQ, 0xffffffff);
	wr(h, SDIO_IRQMASK, h->sv_irqmask);
	wr(h, SDIO_CSR, h->sv_csr);
	dev_info(dev, "controller state restored after sleep\n");
	return 0;
}

static const struct dev_pm_ops s5l_sdio_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(s5l_sdio_suspend_noirq, s5l_sdio_resume_noirq)
};

static struct platform_driver s5l_sdio_driver = {
	.probe = s5l_probe,
	.remove = s5l_remove,
	.driver = {
		.name = "s5l8900-sdio",
		.of_match_table = s5l_sdio_of_match,
		.pm = pm_sleep_ptr(&s5l_sdio_pm_ops),
	},
};
module_platform_driver(s5l_sdio_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8900/S5L8720 SDIO host");
MODULE_LICENSE("GPL");
