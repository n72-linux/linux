// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cirrus Logic CS42L58 audio codec, as wired in the iPod touch 2G (N72)
 *
 * Register layout matches the CS42L55 register map (DS773F1): 0x1a/0x1b are
 * the HP amplifier volumes, 0x1c/0x1d the line volumes, and in PWRCTL2 (0x03)
 * the high nibble is the HP pair and the low nibble the line pair.
 *
 * On the N72 the chip's HP pair is the headphone jack and also feeds the
 * LM48821 and through it the built-in speaker (with only 0x1a/0x1b unmuted
 * both earphones and speaker play; with only 0x1c/0x1d unmuted the speaker
 * is silent). The line pair is the dock connector's line output.
 * "Headphone ..." controls are 0x1a/0x1b, "Line Out ..." 0x1c/0x1d; the
 * speaker's level is the headphone volume times the LM48821's gain.
 *
 * The codec is the clock provider: it takes a fixed 6 MHz MCLK from the SoC
 * (pad GPIO 6 in function 2, the clock controller's AUD0 generator;
 * free-running) and drives the bit and frame clocks. Without the MCLK it is
 * mute and makes no clocks.
 *
 * Capture: the N72 has no built-in microphone; the only input is the headset
 * microphone on the AIN2A pin ('emic'). It uses ADC A and PGA A only; ADC B
 * stays powered down. Line input and internal-microphone sources are not
 * implemented.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define CS42L58_HIDDENCTL	0x00	/* 0x99 unlocks the test registers, 0 locks */
#define CS42L58_CHIPID		0x01	/* bits 2:0 = revision */
#define CS42L58_CHIPID_REV	GENMASK(2, 0)
#define CS42L58_PWRCTL1		0x02
#define CS42L58_PWRCTL1_PDN	BIT(0)
#define CS42L58_PWRCTL1_ADCA	BIT(1)
#define CS42L58_PWRCTL1_ADCB	BIT(2)
#define CS42L58_PWRCTL1_CHRG	BIT(3)	/* ADC charge pump */
#define CS42L58_PWRCTL1_DACS	GENMASK(3, 1)	/* iOS clears all three */
#define CS42L58_PWRCTL1_ADCS	GENMASK(7, 6)	/* iOS: 11 = ADCs off (not in the CS42L55 map) */
#define CS42L58_PWRCTL2		0x03	/* output power, 2-bit fields */
#define CS42L58_PWRCTL2_LINE	GENMASK(3, 0)	/* LINEA b1:0, LINEB b3:2 */
#define CS42L58_PWRCTL2_HP	GENMASK(7, 4)	/* HPA b5:4, HPB b7:6 */
#define CS42L58_OUT_ON		0xa	/* both channels: 10 = always on */
#define CS42L58_OUT_OFF		0xf	/* both channels: 11 = always off */
#define CS42L58_CLKCTL1		0x04
#define CS42L58_CLKCTL1_MCLKDIS	BIT(0)
#define CS42L58_CLKCTL1_MCLKDIV2 BIT(1)
#define CS42L58_CLKCTL1_SCLKMCLK GENMASK(3, 2)	/* 0 bursted, 2 after div, 3 before div */
#define CS42L58_CLKCTL1_INV_SCLK BIT(4)
#define CS42L58_CLKCTL1_MASTER	BIT(5)	/* CS42L55: M/S; bits 7:6 reserved there */
#define CS42L58_CLKCTL1_MCLK	GENMASK(3, 1)	/* iOS: 0 = 6 MHz, 2 = other MCLK */
#define CS42L58_CLKCTL2		0x05
#define CS42L58_CLKCTL2_RATE	GENMASK(4, 0)	/* SPEED b4:3, 32kGROUP b2, RATIO b1:0 */
#define CS42L58_CLKCTL2_HIGH	GENMASK(7, 6)	/* reserved in the CS42L55; iOS writes then clears */
#define CS42L58_CLSHCTL		0x06	/* class H: b5:4 ADPTPWR (0 volume, 1 VCP/2, 2 VCP, 3 signal) */
#define CS42L58_CLSHCTL_FULL	0x20	/* iOS while the speaker amp is used */
#define CS42L58_MISCCTL		0x07	/* b7 DIGMUX, b4 undocumented, b3 ANLGZC, b2 DIGSFT, b0 FREEZE */
#define CS42L58_MISCCTL_IOS	(BIT(3) | BIT(2))
#define CS42L58_ALHMUX		0x08	/* 0 = DAC feeds HP and LINE, ADCs from PGA */
#define CS42L58_ADCRATE		0x09	/* iOS: low nibble depends on the sample rate (ADC filters?) */
#define CS42L58_ADCRATE_MASK	GENMASK(3, 0)
#define CS42L58_ADCCTL		0x0a
#define CS42L58_ADCCTL_SEL	GENMASK(5, 4)	/* iOS: 0 for the external mic, 3 for the internal one */
#define CS42L58_ADCCTL_MUTE	GENMASK(1, 0)	/* ADC A, ADC B */
#define CS42L58_PGAA		0x0b
#define CS42L58_PGAB		0x0c
#define CS42L58_PGA_GAIN	GENMASK(5, 0)	/* two's complement, 0.5 dB steps */
#define CS42L58_PGA_AIN2	BIT(6)		/* iOS sets it for the mic sources: AIN2A (PGA B: AIN2B?) */
#define CS42L58_PGA_BOOST	BIT(7)		/* +20 dB */
#define CS42L58_ADCAVOL		0x0d	/* digital, 1 dB steps; iOS writes 0 */
#define CS42L58_ADCBVOL		0x0e
#define CS42L58_PLAYCTL		0x0f
#define CS42L58_PLAYCTL_PDN_DSP	BIT(7)	/* DSP: mixers, beep, tone control, de-emphasis */
#define CS42L58_PLAYCTL_B_IS_A	BIT(4)
#define CS42L58_PLAYCTL_MSTMUTE	GENMASK(1, 0)
#define CS42L58_PLAYCTL_IOS	(CS42L58_PLAYCTL_PDN_DSP | CS42L58_PLAYCTL_B_IS_A)
#define CS42L58_AMIXA		0x10	/* ADC mixer volumes, b7 mute (reset 0x80) */
#define CS42L58_AMIXB		0x11
#define CS42L58_PMIXA		0x12	/* PCM (SDIN) mixer volumes, b7 mute (reset 0x00) */
#define CS42L58_PMIXB		0x13
#define CS42L58_BEEPFO		0x14	/* b7:4 FREQ (7 = 976 Hz), b3:0 on time */
#define CS42L58_BEEPVO		0x15	/* b7:5 off time, b4:0 volume (2 dB steps, 0 = 0 dB) */
#define CS42L58_BTCTL		0x16
#define CS42L58_BTCTL_BEEP	GENMASK(7, 6)	/* 0 off, 1 single, 2 multiple, 3 continuous */
#define CS42L58_MSTAVOL		0x18
#define CS42L58_MSTBVOL		0x19
#define CS42L58_HPA_VOL		0x1a
#define CS42L58_HPB_VOL		0x1b
#define CS42L58_LINEA_VOL	0x1c
#define CS42L58_LINEB_VOL	0x1d
#define CS42L58_VOL_MUTE	BIT(7)
#define CS42L58_VOL_DB		GENMASK(6, 0)	/* two's complement dB, -60..+12 */
#define CS42L58_STATUS		0x29	/* b7 HPDETECT pin, b6 SPCLKERR (latched, cleared by read) */
#define CS42L58_STATUS_SPCLKERR	BIT(6)
#define CS42L58_STATUS_HPDETECT	BIT(7)
#define CS42L58_MAX_REG		0x3f

#define CS42L58_CAP_MIN		(-12)	/* PGA steps of 0.5 dB: -6 dB ... */
#define CS42L58_CAP_MAX		24	/* ... +12 dB */
#define CS42L58_VOL_MIN_DB	(-60)
#define CS42L58_VOL_MAX_DB	12
#define CS42L58_VOL_DEFAULT_DB	(-20)	/* quiet first contact */
#define CS42L58_MCLK_HZ		6000000

struct cs42l58 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	unsigned int rate;
	bool powered;
	bool provider;		/* codec drives the bit and frame clocks */
	/* kept across power-down, when the chip sits in reset */
	u8 vol[4];
	/* capture controls, kept while the chip sits in reset */
	int cap_gain;		/* PGA steps, CS42L58_CAP_MIN..CS42L58_CAP_MAX */
	bool cap_boost;
	bool cap_mute;
	bool capturing;		/* the ADC widget is powered */
};

/* index = register - 0x1a: HPA, HPB, LINEA, LINEB */
static const u8 cs42l58_vol_regs[4] = {
	CS42L58_HPA_VOL, CS42L58_HPB_VOL, CS42L58_LINEA_VOL, CS42L58_LINEB_VOL,
};

/* sample rate -> CLKCTL2 bits 4:0 */
static const struct {
	unsigned int rate;
	u8 code;
} cs42l58_rates[] = {
	{ 48000, 0x09 }, { 44100, 0x0b }, { 32000, 0x0d }, { 24000, 0x11 },
	{ 22050, 0x13 }, { 16000, 0x15 }, { 12000, 0x19 }, { 11025, 0x1b },
	{  8000, 0x1d },
};

static int cs42l58_rate_code(unsigned int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cs42l58_rates); i++)
		if (cs42l58_rates[i].rate == rate)
			return cs42l58_rates[i].code;
	return -EINVAL;
}

static void cs42l58_reset_pulse(struct cs42l58 *cs)
{
	/* hold reset 10 ms, then wait 1 ms before any register access */
	gpiod_set_value_cansleep(cs->reset, 1);
	msleep(10);
	gpiod_set_value_cansleep(cs->reset, 0);
	usleep_range(1000, 2000);
}

/*
 * Program two test registers (behind 0x00 = 0x99) on revision 0 only.
 * The N72's chip is revision 1 (id 0x69) and skips them.
 */
static void cs42l58_write_hidden(struct cs42l58 *cs, unsigned int id)
{
	struct regmap *rm = cs->regmap;

	if (id & CS42L58_CHIPID_REV)
		return;
	regmap_write(rm, CS42L58_HIDDENCTL, 0x99);
	regmap_write(rm, 0x33, 0xfd);
	regmap_write(rm, 0x32, 0x27);
	regmap_write(rm, CS42L58_HIDDENCTL, 0x00);
}

/*
 * Ends with the codec powered (PDN clear, 75 ms settled), outputs off and
 * the user's volumes loaded.
 */
static int cs42l58_init(struct cs42l58 *cs)
{
	struct regmap *rm = cs->regmap;
	unsigned int id;
	int code, ret, i;

	cs42l58_reset_pulse(cs);

	ret = regmap_read(rm, CS42L58_CHIPID, &id);
	if (ret)
		return ret;
	cs42l58_write_hidden(cs, id);

	code = cs42l58_rate_code(cs->rate);
	if (code < 0)
		code = 0x0b;					/* 44.1 kHz */

	regmap_write(rm, CS42L58_PWRCTL2, 0xaa);
	regmap_update_bits(rm, CS42L58_CLKCTL1, CS42L58_CLKCTL1_MASTER,
			   cs->provider ? CS42L58_CLKCTL1_MASTER : 0);
	/* MCLK code 0: the N72's 6 MHz */
	regmap_update_bits(rm, CS42L58_CLKCTL1, CS42L58_CLKCTL1_MCLK, 0);
	regmap_write(rm, CS42L58_CLKCTL2, CS42L58_CLKCTL2_HIGH | code);
	regmap_update_bits(rm, CS42L58_MISCCTL, CS42L58_MISCCTL_IOS,
			   CS42L58_MISCCTL_IOS);
	regmap_update_bits(rm, CS42L58_PWRCTL1, CS42L58_PWRCTL1_DACS, 0);
	regmap_update_bits(rm, CS42L58_PLAYCTL, CS42L58_PLAYCTL_IOS,
			   CS42L58_PLAYCTL_IOS);
	regmap_update_bits(rm, CS42L58_PGAA, 0xc0, 0);
	regmap_update_bits(rm, CS42L58_PGAB, 0xc0, 0);
	for (i = 0; i < ARRAY_SIZE(cs42l58_vol_regs); i++)
		regmap_update_bits(rm, cs42l58_vol_regs[i], CS42L58_VOL_MUTE,
				   CS42L58_VOL_MUTE);

	regmap_update_bits(rm, CS42L58_PWRCTL1, CS42L58_PWRCTL1_PDN, 0);
	msleep(75);

	/*
	 * With capture inactive, clear register 5 bits 7:6 (set during init)
	 * and switch the ADCs off (register 2 bits 7:6 = 11).
	 */
	regmap_update_bits(rm, CS42L58_CLKCTL2, CS42L58_CLKCTL2_HIGH, 0);
	regmap_update_bits(rm, CS42L58_PWRCTL1, CS42L58_PWRCTL1_ADCS,
			   CS42L58_PWRCTL1_ADCS);
	regmap_write(rm, CS42L58_CLSHCTL, 0);
	regmap_write(rm, CS42L58_PWRCTL2, 0xff);

	/*
	 * Initialize DSP-path registers to known defaults: DAC into both
	 * amplifier pairs, ADC mixer muted, PCM mixer 0 dB unmuted, master
	 * volume 0 dB.
	 */
	regmap_write(rm, CS42L58_ALHMUX, 0x00);
	regmap_write(rm, CS42L58_AMIXA, 0x80);
	regmap_write(rm, CS42L58_AMIXB, 0x80);
	regmap_write(rm, CS42L58_PMIXA, 0x00);
	regmap_write(rm, CS42L58_PMIXB, 0x00);
	regmap_write(rm, CS42L58_MSTAVOL, 0x00);
	regmap_write(rm, CS42L58_MSTBVOL, 0x00);

	/* the volumes and mutes the user set, or the quiet defaults */
	for (i = 0; i < ARRAY_SIZE(cs42l58_vol_regs); i++)
		regmap_write(rm, cs42l58_vol_regs[i], cs->vol[i]);

	cs->powered = true;
	return 0;
}

/* Power down codec after muting and saving output volumes. */
static void cs42l58_power_down(struct cs42l58 *cs)
{
	struct regmap *rm = cs->regmap;
	unsigned int val;
	int i;

	if (!cs->powered)
		return;

	for (i = 0; i < ARRAY_SIZE(cs42l58_vol_regs); i++) {
		if (!regmap_read(rm, cs42l58_vol_regs[i], &val))
			cs->vol[i] = val;
		regmap_write(rm, cs42l58_vol_regs[i],
			     CS42L58_VOL_MUTE | (CS42L58_VOL_MIN_DB & 0x7f));
	}
	regmap_update_bits(rm, CS42L58_MISCCTL, BIT(2), 0);
	msleep(150);
	regmap_update_bits(rm, CS42L58_PWRCTL1, CS42L58_PWRCTL1_PDN,
			   CS42L58_PWRCTL1_PDN);
	gpiod_set_value_cansleep(cs->reset, 1);
	cs->powered = false;
}

static int cs42l58_set_bias_level(struct snd_soc_component *component,
				  enum snd_soc_bias_level level)
{
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_component_get_bias_level(component) ==
		    SND_SOC_BIAS_OFF) {
			ret = cs42l58_init(cs);
			if (ret) {
				dev_err(cs->dev, "power-up failed: %d\n", ret);
				return ret;
			}
		}
		break;
	case SND_SOC_BIAS_OFF:
		cs42l58_power_down(cs);
		break;
	}
	return 0;
}

/* Volume: register value is two's complement dB in bits 6:0, bit 7 mutes. */
static int cs42l58_vol_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int regs[2] = { mc->reg, mc->rreg };
	unsigned int val;
	int i;

	for (i = 0; i < 2; i++) {
		if (cs->powered) {
			if (regmap_read(cs->regmap, regs[i], &val))
				return -EIO;
		} else {
			val = cs->vol[regs[i] - CS42L58_HPA_VOL];
		}
		ucontrol->value.integer.value[i] =
			sign_extend32(val & CS42L58_VOL_DB, 6) - CS42L58_VOL_MIN_DB;
	}
	return 0;
}

static int cs42l58_vol_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int regs[2] = { mc->reg, mc->rreg };
	int i, db, changed = 0;
	u8 *saved;

	for (i = 0; i < 2; i++) {
		db = ucontrol->value.integer.value[i] + CS42L58_VOL_MIN_DB;
		if (db < CS42L58_VOL_MIN_DB || db > CS42L58_VOL_MAX_DB)
			return -EINVAL;
		saved = &cs->vol[regs[i] - CS42L58_HPA_VOL];
		if ((*saved & CS42L58_VOL_DB) != (db & CS42L58_VOL_DB))
			changed = 1;
		*saved = (*saved & CS42L58_VOL_MUTE) | (db & CS42L58_VOL_DB);
		if (cs->powered)
			regmap_update_bits(cs->regmap, regs[i], CS42L58_VOL_DB,
					   db & CS42L58_VOL_DB);
	}
	return changed;
}

static int cs42l58_mute_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int regs[2] = { mc->reg, mc->rreg };
	unsigned int val;
	int i;

	for (i = 0; i < 2; i++) {
		if (cs->powered) {
			if (regmap_read(cs->regmap, regs[i], &val))
				return -EIO;
		} else {
			val = cs->vol[regs[i] - CS42L58_HPA_VOL];
		}
		ucontrol->value.integer.value[i] = !(val & CS42L58_VOL_MUTE);
	}
	return 0;
}

static int cs42l58_mute_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int regs[2] = { mc->reg, mc->rreg };
	int i, changed = 0;
	u8 *saved, mute;

	for (i = 0; i < 2; i++) {
		mute = ucontrol->value.integer.value[i] ? 0 : CS42L58_VOL_MUTE;
		saved = &cs->vol[regs[i] - CS42L58_HPA_VOL];
		if ((*saved & CS42L58_VOL_MUTE) != mute)
			changed = 1;
		*saved = (*saved & CS42L58_VOL_DB) | mute;
		if (cs->powered)
			regmap_update_bits(cs->regmap, regs[i], CS42L58_VOL_MUTE,
					   mute);
	}
	return changed;
}

static const DECLARE_TLV_DB_SCALE(cs42l58_vol_tlv, -6000, 100, 0);

/*
 * Capture gain: PGA step count maps -6..+12 dB in 0.5 dB steps (6-bit two's
 * complement in register 0x0b, clamped to -12..+24), plus a separate +20 dB
 * boost (bit 7 = 0x80). Only the PGA is exposed; ADC digital volume
 * (0x0d/0x0e) remains at 0 dB.
 */
static const DECLARE_TLV_DB_SCALE(cs42l58_cap_tlv, -600, 50, 0);

/*
 * External mic ('emic') capture gain: PGA A (0x0b) takes gain + AIN2 select
 * (bit 6) + optional boost (bit 7), ADC A digital volume (0x0d) is 0, mute
 * is 0x0a bits 1:0 (3 = muted), and register 5 bits 7:6 are set to 11 when
 * boost is enabled and 00 otherwise.
 */
static void cs42l58_capture_gain(struct cs42l58 *cs)
{
	struct regmap *rm = cs->regmap;

	regmap_write(rm, CS42L58_PGAA, (cs->cap_gain & CS42L58_PGA_GAIN) |
		     CS42L58_PGA_AIN2 | (cs->cap_boost ? CS42L58_PGA_BOOST : 0));
	regmap_write(rm, CS42L58_ADCAVOL, 0);
	regmap_update_bits(rm, CS42L58_ADCCTL, CS42L58_ADCCTL_MUTE,
			   cs->cap_mute ? CS42L58_ADCCTL_MUTE : 0);
	regmap_update_bits(rm, CS42L58_CLKCTL2, CS42L58_CLKCTL2_HIGH,
			   cs->cap_boost ? CS42L58_CLKCTL2_HIGH : 0);
}

static int cs42l58_cap_vol_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = cs->cap_gain - CS42L58_CAP_MIN;
	return 0;
}

static int cs42l58_cap_vol_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	long val = ucontrol->value.integer.value[0];

	if (val < 0 || val > CS42L58_CAP_MAX - CS42L58_CAP_MIN)
		return -EINVAL;
	val += CS42L58_CAP_MIN;
	if (val == cs->cap_gain)
		return 0;
	cs->cap_gain = val;
	if (cs->powered && cs->capturing)
		cs42l58_capture_gain(cs);
	return 1;
}

static int cs42l58_cap_boost_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = cs->cap_boost;
	return 0;
}

static int cs42l58_cap_boost_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	bool boost = !!ucontrol->value.integer.value[0];

	if (boost == cs->cap_boost)
		return 0;
	cs->cap_boost = boost;
	if (cs->powered && cs->capturing)
		cs42l58_capture_gain(cs);
	return 1;
}

/* "Capture Switch": 1 = capturing, 0 = ADCs muted (register 0x0a bits 1:0) */
static int cs42l58_cap_switch_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = !cs->cap_mute;
	return 0;
}

static int cs42l58_cap_switch_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	bool mute = !ucontrol->value.integer.value[0];

	if (mute == cs->cap_mute)
		return 0;
	cs->cap_mute = mute;
	if (cs->powered && cs->capturing)
		regmap_update_bits(cs->regmap, CS42L58_ADCCTL, CS42L58_ADCCTL_MUTE,
				   mute ? CS42L58_ADCCTL_MUTE : 0);
	return 1;
}

static const struct snd_kcontrol_new cs42l58_controls[] = {
	/* the HP pair: the jack, and the speaker through the LM48821 */
	SOC_DOUBLE_R_EXT_TLV("Headphone Volume", CS42L58_HPA_VOL, CS42L58_HPB_VOL,
			     0, CS42L58_VOL_MAX_DB - CS42L58_VOL_MIN_DB, 0,
			     cs42l58_vol_get, cs42l58_vol_put, cs42l58_vol_tlv),
	SOC_DOUBLE_R_EXT("Headphone Switch", CS42L58_HPA_VOL, CS42L58_HPB_VOL,
			 0, 1, 0, cs42l58_mute_get, cs42l58_mute_put),
	/* the line pair: the dock connector's line output */
	SOC_DOUBLE_R_EXT_TLV("Line Out Volume", CS42L58_LINEA_VOL, CS42L58_LINEB_VOL,
			     0, CS42L58_VOL_MAX_DB - CS42L58_VOL_MIN_DB, 0,
			     cs42l58_vol_get, cs42l58_vol_put, cs42l58_vol_tlv),
	SOC_DOUBLE_R_EXT("Line Out Switch", CS42L58_LINEA_VOL, CS42L58_LINEB_VOL,
			 0, 1, 0, cs42l58_mute_get, cs42l58_mute_put),
	/* the headset microphone on AIN2A */
	SOC_SINGLE_EXT_TLV("Capture Volume", CS42L58_PGAA, 0,
			   CS42L58_CAP_MAX - CS42L58_CAP_MIN, 0,
			   cs42l58_cap_vol_get, cs42l58_cap_vol_put, cs42l58_cap_tlv),
	SOC_SINGLE_BOOL_EXT("Mic Boost Switch", 0,
			    cs42l58_cap_boost_get, cs42l58_cap_boost_put),
	SOC_SINGLE_BOOL_EXT("Capture Switch", 0,
			    cs42l58_cap_switch_get, cs42l58_cap_switch_put),
};

/*
 * PWRCTL2 output power control: high nibble = HP pair (N72: jack and speaker
 * amplifier input), low nibble = line pair (N72: dock line out).
 */
static int cs42l58_hp_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	bool on = SND_SOC_DAPM_EVENT_ON(event);

	if (!cs->powered)
		return 0;
	return regmap_update_bits(cs->regmap, CS42L58_PWRCTL2, CS42L58_PWRCTL2_HP,
				  FIELD_PREP(CS42L58_PWRCTL2_HP,
					     on ? CS42L58_OUT_ON : CS42L58_OUT_OFF));
}

static int cs42l58_line_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	bool on = SND_SOC_DAPM_EVENT_ON(event);

	if (!cs->powered)
		return 0;
	return regmap_update_bits(cs->regmap, CS42L58_PWRCTL2, CS42L58_PWRCTL2_LINE,
				  FIELD_PREP(CS42L58_PWRCTL2_LINE,
					     on ? CS42L58_OUT_ON : CS42L58_OUT_OFF));
}

/*
 * The HP pair's branch into the speaker amplifier: powered only while the
 * amplifier's path is on. Programs 0x06 = 0x20 (class H supply fixed at
 * full rail) together with the amplifier enable.
 */
static int cs42l58_spk_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);

	if (!cs->powered)
		return 0;
	return regmap_write(cs->regmap, CS42L58_CLSHCTL,
			    SND_SOC_DAPM_EVENT_ON(event) ? CS42L58_CLSHCTL_FULL : 0);
}

/*
 * Input power for the 'emic' source: after gain update, register 0x0a bits
 * 5:4 = 00 (AIN2A), register 2 bits 2:1 = 10 (ADC B down, ADC A on) and
 * register 2 bits 7:6 = 00; register 9's low nibble follows the sample rate
 * (0xf up to 16 kHz, 0xa up to 32 kHz, 5 above). Off sets register 2 bits
 * 7:6 = 11 and bits 2:1 = 11.
 */
static int cs42l58_adc_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs42l58 *cs = snd_soc_component_get_drvdata(component);
	struct regmap *rm = cs->regmap;
	unsigned int nibble;

	if (!cs->powered)
		return 0;
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		cs->capturing = true;
		cs42l58_capture_gain(cs);
		regmap_update_bits(rm, CS42L58_PWRCTL1, CS42L58_PWRCTL1_ADCS, 0);
		regmap_update_bits(rm, CS42L58_ADCCTL, CS42L58_ADCCTL_SEL, 0);
		regmap_update_bits(rm, CS42L58_PWRCTL1,
				   CS42L58_PWRCTL1_ADCA | CS42L58_PWRCTL1_ADCB,
				   CS42L58_PWRCTL1_ADCB);
		if (cs->rate <= 16000)
			nibble = 0xf;
		else if (cs->rate <= 32000)
			nibble = 0xa;
		else
			nibble = 0x5;
		return regmap_update_bits(rm, CS42L58_ADCRATE, CS42L58_ADCRATE_MASK,
					  nibble);
	}
	cs->capturing = false;
	regmap_update_bits(rm, CS42L58_PWRCTL1, CS42L58_PWRCTL1_ADCS,
			   CS42L58_PWRCTL1_ADCS);
	regmap_update_bits(rm, CS42L58_PWRCTL1,
			   CS42L58_PWRCTL1_ADCA | CS42L58_PWRCTL1_ADCB,
			   CS42L58_PWRCTL1_ADCA | CS42L58_PWRCTL1_ADCB);
	/* iOS re-runs the gain update first, which drops register 5 bits 7:6 */
	return regmap_update_bits(rm, CS42L58_CLKCTL2, CS42L58_CLKCTL2_HIGH, 0);
}

static const struct snd_soc_dapm_widget cs42l58_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("AIN2A"),
	SND_SOC_DAPM_ADC_E("ADC", NULL, SND_SOC_NOPM, 0, 0, cs42l58_adc_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_AIF_OUT("AIF Capture", "Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF Playback", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("HP Driver", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cs42l58_hp_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("LINEOUT Driver", SND_SOC_NOPM, 0, 0, NULL, 0,
			       cs42l58_line_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("Speaker Feed", SND_SOC_NOPM, 0, 0, NULL, 0,
			   cs42l58_spk_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("SPKOUT"),
	SND_SOC_DAPM_OUTPUT("HPOUTA"),
	SND_SOC_DAPM_OUTPUT("HPOUTB"),
	SND_SOC_DAPM_OUTPUT("LINEOUTA"),
	SND_SOC_DAPM_OUTPUT("LINEOUTB"),
};

static const struct snd_soc_dapm_route cs42l58_dapm_routes[] = {
	{ "DAC", NULL, "AIF Playback" },
	{ "HP Driver", NULL, "DAC" },
	{ "LINEOUT Driver", NULL, "DAC" },
	{ "HPOUTA", NULL, "HP Driver" },
	{ "HPOUTB", NULL, "HP Driver" },
	{ "Speaker Feed", NULL, "HP Driver" },
	{ "SPKOUT", NULL, "Speaker Feed" },
	{ "LINEOUTA", NULL, "LINEOUT Driver" },
	{ "LINEOUTB", NULL, "LINEOUT Driver" },
	{ "ADC", NULL, "AIN2A" },
	{ "AIF Capture", NULL, "ADC" },
};

static int cs42l58_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct cs42l58 *cs = snd_soc_component_get_drvdata(dai->component);
	int code = cs42l58_rate_code(params_rate(params));

	if (code < 0)
		return code;
	cs->rate = params_rate(params);
	if (cs->powered)
		return regmap_update_bits(cs->regmap, CS42L58_CLKCTL2,
					  CS42L58_CLKCTL2_RATE, code);
	return 0;
}

static int cs42l58_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct cs42l58 *cs = snd_soc_component_get_drvdata(dai->component);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}
	/* iOS runs the codec as the clock provider (I2S TXCON bit 20). */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BP_FP:
		cs->provider = true;
		break;
	case SND_SOC_DAIFMT_BC_FC:
		cs->provider = false;	/* the I2S block drives the clocks */
		break;
	default:
		return -EINVAL;
	}
	if (cs->powered)
		regmap_update_bits(cs->regmap, CS42L58_CLKCTL1,
				   CS42L58_CLKCTL1_MASTER,
				   cs->provider ? CS42L58_CLKCTL1_MASTER : 0);
	return 0;
}

static int cs42l58_set_sysclk(struct snd_soc_dai *dai, int clk_id,
			      unsigned int freq, int dir)
{
	if (freq && freq != CS42L58_MCLK_HZ)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops cs42l58_dai_ops = {
	.hw_params	= cs42l58_hw_params,
	.set_fmt	= cs42l58_set_fmt,
	.set_sysclk	= cs42l58_set_sysclk,
};

#define CS42L58_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
		       SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		       SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		       SNDRV_PCM_RATE_48000)

static struct snd_soc_dai_driver cs42l58_dai = {
	.name = "cs42l58-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = CS42L58_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = CS42L58_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &cs42l58_dai_ops,
	.symmetric_rate = 1,	/* one clock set for both directions */
};

static const struct snd_soc_component_driver cs42l58_component = {
	.set_bias_level		= cs42l58_set_bias_level,
	.controls		= cs42l58_controls,
	.num_controls		= ARRAY_SIZE(cs42l58_controls),
	.dapm_widgets		= cs42l58_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(cs42l58_dapm_widgets),
	.dapm_routes		= cs42l58_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(cs42l58_dapm_routes),
	.idle_bias_on		= 0,
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static const struct regmap_config cs42l58_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = CS42L58_MAX_REG,
	.cache_type = REGCACHE_NONE,
};

static int cs42l58_i2c_probe(struct i2c_client *client)
{
	struct cs42l58 *cs;
	unsigned int id;
	int ret, i;

	cs = devm_kzalloc(&client->dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	cs->dev = &client->dev;
	cs->rate = 44100;
	cs->cap_gain = 0;	/* 0 dB, unmuted, no boost: iOS's defaults */

	cs->regmap = devm_regmap_init_i2c(client, &cs42l58_regmap);
	if (IS_ERR(cs->regmap))
		return dev_err_probe(cs->dev, PTR_ERR(cs->regmap), "regmap\n");

	/* iOS function-reset: GPIO 72, flags say active-low */
	cs->reset = devm_gpiod_get(cs->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(cs->reset))
		return dev_err_probe(cs->dev, PTR_ERR(cs->reset), "reset GPIO\n");

	/* Start quiet: muted, -20 dB on every output. */
	for (i = 0; i < ARRAY_SIZE(cs->vol); i++)
		cs->vol[i] = CS42L58_VOL_MUTE | (CS42L58_VOL_DEFAULT_DB & 0x7f);
	/* Headphones (0x1a/0x1b) unmuted so the first tone is audible. */
	cs->vol[0] &= ~CS42L58_VOL_MUTE;
	cs->vol[1] &= ~CS42L58_VOL_MUTE;

	cs42l58_reset_pulse(cs);
	ret = regmap_read(cs->regmap, CS42L58_CHIPID, &id);
	if (ret)
		return dev_err_probe(cs->dev, ret, "no answer on I2C\n");
	dev_dbg(cs->dev, "chip id/revision register 0x%02x\n", id);

	/* Held in reset until a stream brings the bias up (as iOS idles). */
	gpiod_set_value_cansleep(cs->reset, 1);

	i2c_set_clientdata(client, cs);
	return devm_snd_soc_register_component(cs->dev, &cs42l58_component,
					       &cs42l58_dai, 1);
}

static const struct of_device_id cs42l58_of_match[] = {
	{ .compatible = "cirrus,cs42l58" },
	{ }
};
MODULE_DEVICE_TABLE(of, cs42l58_of_match);

static const struct i2c_device_id cs42l58_i2c_id[] = {
	{ "cs42l58" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cs42l58_i2c_id);

static struct i2c_driver cs42l58_i2c_driver = {
	.driver = {
		.name = "cs42l58",
		.of_match_table = cs42l58_of_match,
	},
	.probe = cs42l58_i2c_probe,
	.id_table = cs42l58_i2c_id,
};
module_i2c_driver(cs42l58_i2c_driver);

MODULE_DESCRIPTION("CS42L58 codec driver (iPod touch 2G)");
MODULE_LICENSE("GPL");
