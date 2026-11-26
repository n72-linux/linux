// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2S controller of the Samsung/Apple S5L8720 (iPod touch 2G, N72)
 *
 * On N72 the controller is a clock consumer: the CS42L58 codec drives bit
 * and frame clock from its 6 MHz MCLK, TXCON bit 20 marks the interface as
 * externally clocked, and the divider register is left at 0 (CLKCON 0, DIV
 * 0, TXCON 0x03100019, RXCON 0x1000). The MCLK leaves the SoC on pad GPIO 6
 * in function 2 (the clock controller's AUD0 generator, free-running; the
 * GPIO node's apple,pin-functions keeps the pad there).
 *
 * Data moves through DMAC0 (PL080): request 10 to the TX FIFO at +0x10,
 * request 11 from the RX FIFO at +0x38, 16-bit beats in bursts of 4.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define S5L_I2S_CLKCON		0x00
#define S5L_I2S_CLKCON_ON	BIT(0)
#define S5L_I2S_CLKCON_IDLE	BIT(1)	/* set once the clock has stopped */
#define S5L_I2S_CLKCON_SRC	GENMASK(4, 2)	/* 0 external (used here), 5 nclk, 6 nclk/2 */
#define S5L_I2S_CLKCON_STOP	BIT(5)
#define S5L_I2S_TXCON		0x04
#define S5L_I2S_TXCON_ON	BIT(0)
#define S5L_I2S_TXCON_WIDTH	GENMASK(8, 7)	/* 0 = 16, 1 = 20, 2 = 24 bits */
#define S5L_I2S_TXCON_EXTCLK	BIT(20)		/* bit/frame clock from outside */
#define S5L_I2S_TXCOM		0x08
#define S5L_I2S_TXCOM_RUN	0x6		/* interface + DMA request enable */
#define S5L_I2S_RXCON		0x30
#define S5L_I2S_RXCOM		0x34
#define S5L_I2S_STATUS		0x3c		/* FIFO reset status; write 1 */
#define S5L_I2S_DIV		0x40

#define S5L_I2S_TXFIFO		0x10
#define S5L_I2S_RXFIFO		0x38

/* iOS audio-data words for the N72's CS42L58 */
#define N72_TXCON		0x03100019
#define N72_RXCON		0x00001000

struct s5l8720_i2s {
	struct device *dev;
	void __iomem *base;
	struct snd_dmaengine_dai_dma_data playback;
	struct snd_dmaengine_dai_dma_data capture;
	bool running[2];
};

static int s5l8720_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	/*
	 * The codec provides the bit and frame clocks, as under iOS; this end
	 * is programmed the same way whatever the card says.
	 */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
	case SND_SOC_DAIFMT_BP_FP:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s5l8720_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct s5l8720_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (params_channels(params) != 2 ||
	    params_format(params) != SNDRV_PCM_FORMAT_S16_LE)
		return -EINVAL;

	/*
	 * Configure both directions cleanly: FIFO reset (TX used), TXCON | 1,
	 * RXCON = 0x1000, TXCOM = 0, RXCOM = 0, DIV = 0, CLKCON = 1 (external
	 * clock source). A running stream in the other direction keeps its
	 * command register.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		writel(1, i2s->base + S5L_I2S_STATUS);
	if (!i2s->running[1])
		writel(N72_TXCON | S5L_I2S_TXCON_ON, i2s->base + S5L_I2S_TXCON);
	writel(N72_RXCON, i2s->base + S5L_I2S_RXCON);
	if (!i2s->running[1])
		writel(0, i2s->base + S5L_I2S_TXCOM);
	if (!i2s->running[0])
		writel(0, i2s->base + S5L_I2S_RXCOM);
	writel(0, i2s->base + S5L_I2S_DIV);
	writel(S5L_I2S_CLKCON_ON, i2s->base + S5L_I2S_CLKCON);

	return 0;
}

static int s5l8720_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct s5l8720_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 com = tx ? S5L_I2S_TXCOM : S5L_I2S_RXCOM;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* iOS: the DMA channel is started first, then TXCOM = 6 */
		writel(S5L_I2S_TXCOM_RUN, i2s->base + com);
		i2s->running[tx] = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		writel(0, i2s->base + com);
		i2s->running[tx] = false;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void s5l8720_i2s_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct s5l8720_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 val;

	if (i2s->running[0] || i2s->running[1])
		return;

	/*
	 * iOS stop: command registers off, then CLKCON = 0x20. The wait for
	 * bit 1 only applies with an internal clock; with the codec clocking
	 * us it may never come, so it is bounded and its failure ignored.
	 */
	writel(0, i2s->base + S5L_I2S_TXCOM);
	writel(0, i2s->base + S5L_I2S_RXCOM);
	writel(S5L_I2S_CLKCON_STOP, i2s->base + S5L_I2S_CLKCON);
	readl_poll_timeout(i2s->base + S5L_I2S_CLKCON, val,
			   val & S5L_I2S_CLKCON_IDLE, 10, 1000);
}

static int s5l8720_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct s5l8720_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->playback, &i2s->capture);
	return 0;
}

static const struct snd_soc_dai_ops s5l8720_i2s_dai_ops = {
	.probe		= s5l8720_i2s_dai_probe,
	.set_fmt	= s5l8720_i2s_set_fmt,
	.hw_params	= s5l8720_i2s_hw_params,
	.trigger	= s5l8720_i2s_trigger,
	.shutdown	= s5l8720_i2s_shutdown,
};

/* The codec sets the rate; this end only follows its clocks. */
#define S5L8720_I2S_RATES	(SNDRV_PCM_RATE_8000_48000)

static struct snd_soc_dai_driver s5l8720_i2s_dai = {
	.name = "s5l8720-i2s",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = S5L8720_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = S5L8720_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &s5l8720_i2s_dai_ops,
	.symmetric_rate = 1,
};

static const struct snd_soc_component_driver s5l8720_i2s_component = {
	.name = "s5l8720-i2s",
	.legacy_dai_naming = 1,
};

/*
 * The PCM buffers are cached, not uncached: ALSA's dmix reads every sample
 * of the hardware buffer it mixes into (load, add, store), and from an
 * uncached mapping each of those loads went to memory (about 16% of a
 * game's audio thread). The PCM core cleans the buffer for the DMA
 * controller whenever the application pointer moves (SYNC_PTR, write,
 * forward; status and control are then not mmapped, so every move goes
 * through it) and invalidates it for capture. The DMA only ever reads
 * what was committed: the application pointer stays ahead of the hardware
 * pointer, and everything up to it has been cleaned. Preallocated at the
 * size n72-base's dmix asks for (16384 frames of S16 stereo), since every
 * sync covers the whole buffer; a larger request gets its own buffer.
 */
static const struct snd_dmaengine_pcm_config s5l8720_i2s_pcm_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.prealloc_buffer_size = 16384 * 4,
};

static int s5l8720_i2s_probe(struct platform_device *pdev)
{
	struct s5l8720_i2s *i2s;
	struct clk_bulk_data *clks;
	struct resource *res;
	int ret;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	i2s->dev = &pdev->dev;

	i2s->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(i2s->base))
		return PTR_ERR(i2s->base);

	/*
	 * The block's gates (iOS device-clocks id 0x14 "I2S0", and "DMACM"),
	 * kept open from here on: the codec's 6 MHz MCLK is expected to come
	 * through this block, so it must be clocked before the codec is
	 * powered.
	 */
	ret = devm_clk_bulk_get_all_enabled(&pdev->dev, &clks);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "clocks\n");

	i2s->playback.addr = res->start + S5L_I2S_TXFIFO;
	i2s->playback.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->playback.maxburst = 4;
	i2s->capture.addr = res->start + S5L_I2S_RXFIFO;
	i2s->capture.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->capture.maxburst = 4;

	/* Nothing runs until a stream starts. */
	writel(0, i2s->base + S5L_I2S_TXCOM);
	writel(0, i2s->base + S5L_I2S_RXCOM);

	platform_set_drvdata(pdev, i2s);

	ret = devm_snd_soc_register_component(&pdev->dev, &s5l8720_i2s_component,
					      &s5l8720_i2s_dai, 1);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "DAI registration\n");

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &s5l8720_i2s_pcm_config,
					      SND_DMAENGINE_PCM_FLAG_NONCOHERENT);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "PCM registration\n");

	return 0;
}

static const struct of_device_id s5l8720_i2s_of_match[] = {
	{ .compatible = "apple,s5l8720-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5l8720_i2s_of_match);

static struct platform_driver s5l8720_i2s_driver = {
	.probe = s5l8720_i2s_probe,
	.driver = {
		.name = "s5l8720-i2s",
		.of_match_table = s5l8720_i2s_of_match,
	},
};
module_platform_driver(s5l8720_i2s_driver);

MODULE_DESCRIPTION("S5L8720 I2S controller (iPod touch 2G)");
MODULE_LICENSE("GPL");
