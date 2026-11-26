// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI controller driver for the Samsung/Apple S5L8720
 *
 * Polled streaming transfers up to 65535 words per hardware segment using the
 * 16-word TX/RX FIFOs in 8-, 16-, or 32-bit word mode. Chip selects are GPIOs
 * (cs-gpios).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define SPI_CONTROL		0x00
#define SPI_SETUP		0x04
#define SPI_STATUS		0x08
#define SPI_PIN			0x0c
#define SPI_TXDATA		0x10
#define SPI_RXDATA		0x20
#define SPI_CLKDIV		0x30
#define SPI_RXCNT		0x34
#define SPI_SPIDD		0x38
#define SPI_TXLEN		0x4c

#define CONTROL_ENABLE		BIT(0)
#define CONTROL_RESET_TX	BIT(2)
#define CONTROL_RESET_RX	BIT(3)

#define SETUP_CPHA		BIT(1)	/* "last clock edge missing" */
#define SETUP_CPOL		BIT(2)	/* "active low" */
#define SETUP_MASTER		(3 << 3)
#define SETUP_OPTION5		(1 << 5)
#define SETUP_RX_ENABLE		BIT(7)
#define SETUP_ENABLE2		BIT(8)
#define SETUP_CLKSRC		BIT(14)	/* 1: fixed clock (NCLK), 0: PCLK */
#define SETUP_WORDSIZE		GENMASK(16, 15)
#define SETUP_TX_ENABLE		BIT(21)

#define STATUS_CLEAR_MASK	0x0040000f
#define STATUS_TXLVL		GENMASK(10, 6)
#define STATUS_RXLVL		GENMASK(15, 11)

#define S5L8720_SPI_MAXDIV	0x3ff
#define S5L8720_SPI_FIFO_DEPTH	16U
#define S5L8720_SPI_MAX_WORDS	0xffffU

struct s5l8720_spi {
	void __iomem *base;
	unsigned long clk_rate;
	/*
	 * The controller's clock gates ("clocks"), on only while the SPI
	 * core has messages queued (prepare/unprepare_transfer_hardware), as
	 * iOS gates its SPI blocks when idle. Polled: nothing touches the
	 * registers outside a transfer.
	 */
	struct clk_bulk_data *clks;
	int nclks;
};

static int s5l8720_spi_prepare_hw(struct spi_controller *ctlr)
{
	struct s5l8720_spi *ss = spi_controller_get_devdata(ctlr);

	return clk_bulk_enable(ss->nclks, ss->clks);
}

static int s5l8720_spi_unprepare_hw(struct spi_controller *ctlr)
{
	struct s5l8720_spi *ss = spi_controller_get_devdata(ctlr);

	writel(0, ss->base + SPI_CONTROL);
	clk_bulk_disable(ss->nclks, ss->clks);
	return 0;
}

static u32 s5l8720_spi_setup_bits(struct spi_device *spi, unsigned int bpw)
{
	u32 setup = SETUP_CLKSRC | SETUP_MASTER;

	if (spi->mode & SPI_CPHA)
		setup |= SETUP_CPHA;
	if (spi->mode & SPI_CPOL)
		setup |= SETUP_CPOL;
	setup |= FIELD_PREP(SETUP_WORDSIZE, bpw == 32 ? 2 : bpw == 16 ? 1 : 0);
	return setup;
}

/*
 * Program idle SCLK polarity before the SPI core asserts chip select so
 * mode-3 (CPOL=1) devices never see a spurious low-to-high clock edge at CS.
 */
static int s5l8720_spi_prepare_message(struct spi_controller *ctlr,
				       struct spi_message *msg)
{
	struct s5l8720_spi *ss = spi_controller_get_devdata(ctlr);
	void __iomem *b = ss->base;
	unsigned int bpw = msg->spi->bits_per_word ?: 8;

	writel(s5l8720_spi_setup_bits(msg->spi, bpw), b + SPI_SETUP);
	writel(2, b + SPI_PIN);
	writel(STATUS_CLEAR_MASK, b + SPI_STATUS);
	writel(CONTROL_ENABLE, b + SPI_CONTROL);
	return 0;
}

static void s5l8720_spi_push_words(void __iomem *b, const u8 *tx,
				   unsigned int bytes, unsigned int from,
				   unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned int idx = from + i;
		u32 w = ~0U;

		if (tx)
			w = bytes == 4 ? get_unaligned((const u32 *)(tx + idx * 4)) :
			    bytes == 2 ? get_unaligned((const u16 *)(tx + idx * 2)) :
			    tx[idx];
		writel(w, b + SPI_TXDATA);
	}
}

static void s5l8720_spi_drain_words(void __iomem *b, u8 *rx,
				    unsigned int bytes, unsigned int from,
				    unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		u32 w = readl(b + SPI_RXDATA);
		unsigned int idx = from + i;

		if (!rx)
			continue;
		if (bytes == 4)
			put_unaligned(w, (u32 *)(rx + idx * 4));
		else if (bytes == 2)
			put_unaligned((u16)w, (u16 *)(rx + idx * 2));
		else
			rx[idx] = w;
	}
}

static int s5l8720_spi_stream_segment(struct s5l8720_spi *ss,
				      struct spi_device *spi, u32 setup,
				      const u8 *tx, u8 *rx, unsigned int bytes,
				      unsigned int seg, unsigned int timeout_us)
{
	void __iomem *b = ss->base;
	unsigned int tx_sent, rx_done = 0;
	ktime_t end;
	u32 st;

	writel(STATUS_CLEAR_MASK, b + SPI_STATUS);
	writel(setup, b + SPI_SETUP);
	writel(CONTROL_ENABLE | CONTROL_RESET_TX | CONTROL_RESET_RX,
	       b + SPI_CONTROL);
	writel(seg, b + SPI_RXCNT);
	writel(seg, b + SPI_TXLEN);
	writel(setup | SETUP_OPTION5, b + SPI_SETUP);

	tx_sent = min(seg, S5L8720_SPI_FIFO_DEPTH);
	s5l8720_spi_push_words(b, tx, bytes, 0, tx_sent);
	writel(setup | SETUP_OPTION5 | SETUP_RX_ENABLE | SETUP_ENABLE2 |
	       SETUP_TX_ENABLE, b + SPI_SETUP);

	end = ktime_add_us(ktime_get(), timeout_us);
	while (rx_done < seg) {
		unsigned int rx_avail, in_flight, tx_space, tx_push;

		st = readl(b + SPI_STATUS);
		rx_avail = min_t(unsigned int, FIELD_GET(STATUS_RXLVL, st),
				 seg - rx_done);
		if (rx_avail) {
			s5l8720_spi_drain_words(b, rx, bytes, rx_done, rx_avail);
			rx_done += rx_avail;
		}

		in_flight = tx_sent - rx_done;
		tx_space = min_t(unsigned int,
				 S5L8720_SPI_FIFO_DEPTH - FIELD_GET(STATUS_TXLVL, st),
				 S5L8720_SPI_FIFO_DEPTH - in_flight);
		tx_push = min(seg - tx_sent, tx_space);
		if (tx_push) {
			s5l8720_spi_push_words(b, tx, bytes, tx_sent, tx_push);
			tx_sent += tx_push;
		}

		if (rx_avail || tx_push) {
			end = ktime_add_us(ktime_get(), timeout_us);
			if (need_resched())
				cond_resched();
			continue;
		}

		if (ktime_after(ktime_get(), end)) {
			st = readl(b + SPI_STATUS);
			if (FIELD_GET(STATUS_RXLVL, st) > 0)
				continue;
			dev_err(&spi->dev,
				"SPI transfer timed out (%u/%u rx, %u/%u tx, status %08x, setup %08x)\n",
				rx_done, seg, tx_sent, seg, st,
				readl(b + SPI_SETUP));
			writel(setup, b + SPI_SETUP);
			writel(STATUS_CLEAR_MASK, b + SPI_STATUS);
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	writel(setup, b + SPI_SETUP);
	writel(STATUS_CLEAR_MASK, b + SPI_STATUS);
	return 0;
}

static int s5l8720_spi_transfer_one(struct spi_controller *ctlr,
				    struct spi_device *spi,
				    struct spi_transfer *xfer)
{
	struct s5l8720_spi *ss = spi_controller_get_devdata(ctlr);
	unsigned int bpw = xfer->bits_per_word ?: 8;
	unsigned int bytes = DIV_ROUND_UP(bpw, 8);
	unsigned int words = xfer->len / bytes;
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;
	u32 setup, div, spidd, timeout_us;
	int delay_ns, ret = 0;

	if (!words)
		return 0;

	if (bytes == 3)
		bytes = 4;

	div = DIV_ROUND_UP(ss->clk_rate, xfer->speed_hz ?: spi->max_speed_hz);
	div = clamp_t(u32, div, 1, S5L8720_SPI_MAXDIV);
	setup = s5l8720_spi_setup_bits(spi, bpw);
	/* Progress timeout per FIFO fill: 8x FIFO bit time, at least 2 ms. */
	timeout_us = max_t(u32, 2000,
			   div_u64(8ULL * S5L8720_SPI_FIFO_DEPTH * bytes * 8 *
				   USEC_PER_SEC * div, ss->clk_rate));

	delay_ns = spi_delay_to_ns(&xfer->word_delay, xfer);
	if (delay_ns <= 0)
		delay_ns = spi_delay_to_ns(&spi->word_delay, xfer);
	spidd = delay_ns > 0 ?
		DIV_ROUND_UP_ULL((u64)delay_ns * 66500000ULL, NSEC_PER_SEC) : 0;

	writel(div, ss->base + SPI_CLKDIV);
	writel(spidd, ss->base + SPI_SPIDD);

	while (words) {
		unsigned int seg = min(words, S5L8720_SPI_MAX_WORDS);

		ret = s5l8720_spi_stream_segment(ss, spi, setup, tx, rx, bytes,
						 seg, timeout_us);
		if (ret)
			break;
		if (tx)
			tx += seg * bytes;
		if (rx)
			rx += seg * bytes;
		words -= seg;
	}

	return ret;
}

static void s5l8720_spi_unprepare(void *data)
{
	struct s5l8720_spi *ss = data;

	clk_bulk_unprepare(ss->nclks, ss->clks);
}

static int s5l8720_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct s5l8720_spi *ss;
	u32 rate = 24000000;
	int ret;

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(*ss));
	if (!ctlr)
		return -ENOMEM;
	ss = spi_controller_get_devdata(ctlr);

	ss->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ss->base))
		return PTR_ERR(ss->base);

	/* Fixed reference clock that feeds the divider. */
	of_property_read_u32(pdev->dev.of_node, "clock-frequency", &rate);
	ss->clk_rate = rate;

	ss->nclks = devm_clk_bulk_get_all(&pdev->dev, &ss->clks);
	if (ss->nclks < 0)
		return dev_err_probe(&pdev->dev, ss->nclks, "clocks\n");
	ret = clk_bulk_prepare(ss->nclks, ss->clks);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, s5l8720_spi_unprepare, ss);
	if (ret)
		return ret;
	ret = clk_bulk_enable(ss->nclks, ss->clks);
	if (ret)
		return ret;
	writel(0, ss->base + SPI_CONTROL);
	clk_bulk_disable(ss->nclks, ss->clks);

	ctlr->dev.of_node = pdev->dev.of_node;
	ctlr->bus_num = -1;
	ctlr->use_gpio_descriptors = true;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16) |
				   SPI_BPW_MASK(32);
	ctlr->min_speed_hz = DIV_ROUND_UP(rate, S5L8720_SPI_MAXDIV);
	ctlr->max_speed_hz = rate;
	ctlr->transfer_one = s5l8720_spi_transfer_one;
	ctlr->prepare_message = s5l8720_spi_prepare_message;
	ctlr->prepare_transfer_hardware = s5l8720_spi_prepare_hw;
	ctlr->unprepare_transfer_hardware = s5l8720_spi_unprepare_hw;

	return devm_spi_register_controller(&pdev->dev, ctlr);
}

static const struct of_device_id s5l8720_spi_of_match[] = {
	{ .compatible = "apple,s5l8720-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8720_spi_of_match);

static struct platform_driver s5l8720_spi_driver = {
	.probe = s5l8720_spi_probe,
	.driver = {
		.name = "spi-s5l8720",
		.of_match_table = s5l8720_spi_of_match,
	},
};
module_platform_driver(s5l8720_spi_driver);

MODULE_DESCRIPTION("Samsung/Apple S5L8720 SPI controller");
MODULE_LICENSE("GPL");
