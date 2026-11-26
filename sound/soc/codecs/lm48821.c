// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI/National LM48821 speaker amplifier with I2C volume control
 *
 * Single write-only control byte: bits 7:3 select the 32-step gain, bit 2
 * is shutdown, bits 1:0 are set while running. The enable GPIO must be
 * asserted before writing the gain byte and released after writing the
 * shutdown byte. On N72 the enable GPIO is active-low.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/tlv.h>

#define LM48821_GAIN_SHIFT	3
#define LM48821_GAIN_MAX	31
#define LM48821_RUN_BITS	0x3
#define LM48821_SHUTDOWN	0x4
#define LM48821_GAIN_DEFAULT	6	/* about -30 dB */

struct lm48821 {
	struct i2c_client *client;
	struct gpio_desc *enable;
	u8 gain;
	bool on;
};

static int lm48821_write(struct lm48821 *amp, u8 val)
{
	int ret = i2c_master_send(amp->client, &val, 1);

	if (ret < 0)
		return ret;
	return ret == 1 ? 0 : -EIO;
}

static int lm48821_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct lm48821 *amp = snd_soc_component_get_drvdata(component);
	int ret = 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		gpiod_set_value_cansleep(amp->enable, 1);
		usleep_range(1000, 2000);
		ret = lm48821_write(amp, (amp->gain << LM48821_GAIN_SHIFT) |
					 LM48821_RUN_BITS);
		amp->on = !ret;
	} else {
		ret = lm48821_write(amp, LM48821_SHUTDOWN);
		gpiod_set_value_cansleep(amp->enable, 0);
		amp->on = false;
	}
	if (ret)
		dev_err(component->dev, "I2C write failed: %d\n", ret);
	return ret;
}

static int lm48821_gain_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct lm48821 *amp = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = amp->gain;
	return 0;
}

static int lm48821_gain_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct lm48821 *amp = snd_soc_component_get_drvdata(component);
	long val = ucontrol->value.integer.value[0];

	if (val < 0 || val > LM48821_GAIN_MAX)
		return -EINVAL;
	if (val == amp->gain)
		return 0;
	amp->gain = val;
	if (amp->on)
		lm48821_write(amp, (amp->gain << LM48821_GAIN_SHIFT) |
				   LM48821_RUN_BITS);
	return 1;
}

/*
 * Approximate step gains mapped onto the 5-bit attenuation codes.
 */
static const DECLARE_TLV_DB_RANGE(lm48821_gain_tlv,
	0, 0, TLV_DB_SCALE_ITEM(-8000, 0, 1),
	1, 1, TLV_DB_SCALE_ITEM(-6200, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(-5200, 0, 0),
	3, 3, TLV_DB_SCALE_ITEM(-4400, 0, 0),
	4, 4, TLV_DB_SCALE_ITEM(-3800, 0, 0),
	5, 5, TLV_DB_SCALE_ITEM(-3400, 0, 0),
	6, 9, TLV_DB_SCALE_ITEM(-3000, 300, 0),
	10, 18, TLV_DB_SCALE_ITEM(-1800, 200, 0),
	19, 25, TLV_DB_SCALE_ITEM(0, 200, 0),
	26, 31, TLV_DB_SCALE_ITEM(1300, 100, 0)
);

static const struct snd_kcontrol_new lm48821_controls[] = {
	SOC_SINGLE_EXT_TLV("Speaker Amp Volume", SND_SOC_NOPM, 0,
			   LM48821_GAIN_MAX, 0, lm48821_gain_get,
			   lm48821_gain_put, lm48821_gain_tlv),
};

/* Off by default: the speaker only runs when "Speaker Amp Switch" is on. */
static const struct snd_kcontrol_new lm48821_switch =
	SOC_DAPM_SINGLE_VIRT("Switch", 1);

static const struct snd_soc_dapm_widget lm48821_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),
	SND_SOC_DAPM_SWITCH("Speaker Amp", SND_SOC_NOPM, 0, 0, &lm48821_switch),
	SND_SOC_DAPM_OUT_DRV_E("Amp", SND_SOC_NOPM, 0, 0, NULL, 0, lm48821_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route lm48821_dapm_routes[] = {
	{ "Speaker Amp", "Switch", "IN" },
	{ "Amp", NULL, "Speaker Amp" },
	{ "OUT", NULL, "Amp" },
};

static const struct snd_soc_component_driver lm48821_component = {
	.controls		= lm48821_controls,
	.num_controls		= ARRAY_SIZE(lm48821_controls),
	.dapm_widgets		= lm48821_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(lm48821_dapm_widgets),
	.dapm_routes		= lm48821_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(lm48821_dapm_routes),
	.endianness		= 1,
};

static int lm48821_i2c_probe(struct i2c_client *client)
{
	struct lm48821 *amp;

	amp = devm_kzalloc(&client->dev, sizeof(*amp), GFP_KERNEL);
	if (!amp)
		return -ENOMEM;
	amp->client = client;
	amp->gain = LM48821_GAIN_DEFAULT;

	/* iOS function-amp_enable (N72: GPIO 114, active-low in the DT); start disabled */
	amp->enable = devm_gpiod_get(&client->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(amp->enable))
		return dev_err_probe(&client->dev, PTR_ERR(amp->enable),
				     "enable GPIO\n");

	i2c_set_clientdata(client, amp);
	return devm_snd_soc_register_component(&client->dev, &lm48821_component,
					       NULL, 0);
}

static const struct of_device_id lm48821_of_match[] = {
	{ .compatible = "ti,lm48821" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm48821_of_match);

static const struct i2c_device_id lm48821_i2c_id[] = {
	{ "lm48821" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm48821_i2c_id);

static struct i2c_driver lm48821_i2c_driver = {
	.driver = {
		.name = "lm48821",
		.of_match_table = lm48821_of_match,
	},
	.probe = lm48821_i2c_probe,
	.id_table = lm48821_i2c_id,
};
module_i2c_driver(lm48821_i2c_driver);

MODULE_DESCRIPTION("LM48821 speaker amplifier");
MODULE_LICENSE("GPL");
