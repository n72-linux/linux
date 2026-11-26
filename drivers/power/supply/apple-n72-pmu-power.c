// SPDX-License-Identifier: GPL-2.0-only
/*
 * PMU of the Apple iPod touch (2nd generation), N72: the battery (voltage,
 * gauge, temperature, charger) and USB power supplies. See
 * drivers/mfd/apple-n72-pmu-core.c.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/mfd/apple-n72-pmu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/*
 * Battery telemetry and charger status: ADC channel 4, 10 bits,
 * mV = 2500 + raw * 2000 / 1024. Conversion starts with control = channel | 0x10;
 * wait for bit 5 of interrupt status 2, then read result (0x42 << 2) | (0x41 & 3).
 * Status bytes 0x04..:
 * byte 0 bit 3 = USB power present; byte 1 bits 1-2 = charger is charging
 * (either bit) and bit 5 = charger timeout (reset timer through bits 3-6 of 0x34).
 */
#define N72_PMU_INT2_ADCDONE	BIT(5)
#define N72_PMU_ADCCON		0x40
#define N72_PMU_ADCCON_START	0x10
#define N72_PMU_ADCOUT		0x41
#define N72_PMU_ADC_VBAT	4
#define N72_PMU_ADC_NTC		2

#define N72_PMU_STATUS_USB	BIT(3)			/* byte 0 */
#define N72_PMU_STATUS_CHARGING	(BIT(1) | BIT(2))	/* byte 1: the charger charges */
#define N72_PMU_STATUS_TIMEOUT	BIT(5)			/* byte 1 */

struct n72_pmu_power {
	struct n72_pmu *core;
	struct regmap *regmap;
	struct device *dev;
	struct power_supply *bat, *usb;
	struct power_supply_battery_info *info;
	struct mutex adc_lock;		/* the ADC, and the two fields below */
	unsigned long sampled;		/* jiffies of the last battery sample */
	int vbat_mv;
	struct notifier_block nb;
	bool usb_irq;			/* USB plug and unplug come as events */

	/* the battery level, as iOS works it out (n72_gauge_*) */
	struct delayed_work gauge_work;
	struct mutex chg_lock;		/* the fields below and register 0x33 */
	u8 ring[4];			/* percent, newest first */
	u8 ring_n;
	bool boot_sample;		/* the ring holds the probe's sample only */
	int avg;			/* mean of the ring */
	int level;			/* what is reported; -1 = no sample yet */
	bool charging;			/* status byte 1 bits 1-2, as last read */
	bool paused;			/* charge current 0 for a sample */
	u8 chg_code;			/* register 0x33 bits 7:2 to put back */
	unsigned long next_sample;	/* on external power: when to pause and sample */
	unsigned long timeout_seen;	/* charger timeout first seen (jiffies | 1), 0 = none */
	int temp;			/* the battery, 0.01 degrees C, at the last sample */
	bool temp_ok;			/* temp was read at least once */
	s8 chg_row;			/* n72_chg_rows[] entry in use, -1 = none */
};

/* ------------------------------------------------------------ battery */

static int n72_pmu_adc(struct n72_pmu_power *pmu, u8 ch, unsigned int *raw)
{
	struct n72_pmu *c = pmu->core;
	u8 out[2];
	int i, v, ret = -ETIMEDOUT;

	n72_pmu_int2_take(c, N72_PMU_INT2_ADCDONE);	/* clear a stale done bit */
	if (regmap_write(pmu->regmap, N72_PMU_ADCCON, (ch & 0xf) | N72_PMU_ADCCON_START))
		return -EIO;
	for (i = 0; i < 40; i++) {
		usleep_range(1000, 1500);
		v = n72_pmu_int2_take(c, N72_PMU_INT2_ADCDONE);
		if (v > 0) {
			ret = 0;
			break;
		}
	}
	if (!ret && regmap_bulk_read(pmu->regmap, N72_PMU_ADCOUT, out, 2))
		ret = -EIO;
	regmap_write(pmu->regmap, N72_PMU_ADCCON, 0);
	if (!ret)
		*raw = (out[1] << 2) | (out[0] & 3);
	return ret;
}

static bool n72_pmu_usb_online(struct n72_pmu_power *pmu)
{
	unsigned int v;

	return !regmap_read(pmu->regmap, N72_PMU_STATUS, &v) &&
	       (v & N72_PMU_STATUS_USB);
}

/* Battery voltage, sampled at most every two seconds. */
static int n72_pmu_vbat(struct n72_pmu_power *pmu, int *mv)
{
	unsigned int raw;
	int ret = 0;

	mutex_lock(&pmu->adc_lock);
	if (!pmu->sampled || time_after(jiffies, pmu->sampled + 2 * HZ)) {
		ret = n72_pmu_adc(pmu, N72_PMU_ADC_VBAT, &raw);
		if (!ret) {
			pmu->vbat_mv = 2500 + raw * 2000 / 1024;
			pmu->sampled = jiffies;
		}
	}
	*mv = pmu->vbat_mv;
	mutex_unlock(&pmu->adc_lock);
	return ret;
}

static int n72_pmu_bat_status(struct n72_pmu_power *pmu);

static int n72_pmu_bat_get(struct power_supply *psy, enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct n72_pmu_power *pmu = power_supply_get_drvdata(psy);
	int mv, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = n72_pmu_vbat(pmu, &mv);
		val->intval = mv * 1000;
		return ret;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = READ_ONCE(pmu->level);
		if (ret < 0)
			return -ENODATA;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = n72_pmu_bat_status(pmu);
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		if (!READ_ONCE(pmu->temp_ok))
			return -ENODATA;
		val->intval = READ_ONCE(pmu->temp) / 10;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property n72_pmu_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc n72_pmu_bat_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = n72_pmu_bat_props,
	.num_properties = ARRAY_SIZE(n72_pmu_bat_props),
	.get_property = n72_pmu_bat_get,
};

/*
 * USB input current limit: register 0x33 bits 1:0 = 0 (100 mA), 2 (500 mA) or
 * 3 (1000 mA), and 0x34 bit 7 set with it. The PMU resets to 100 mA,
 * which is less than the device draws while busy.
 */
#define N72_PMU_ILIM		0x33
#define N72_PMU_ILIM_MASK	0x03
#define N72_PMU_ILIM2		0x34
#define N72_PMU_ILIM2_ON	BIT(7)

static int n72_pmu_ilim_get(struct n72_pmu_power *pmu)
{
	unsigned int v;
	int ret = regmap_read(pmu->regmap, N72_PMU_ILIM, &v);

	if (ret)
		return ret;
	switch (v & N72_PMU_ILIM_MASK) {
	case 3:  return 1000000;
	case 2:  return 500000;
	default: return 100000;
	}
}

static int n72_pmu_ilim_set(struct n72_pmu_power *pmu, int uA)
{
	u8 code = uA >= 1000000 ? 3 : uA >= 500000 ? 2 : 0;
	int ret;

	ret = regmap_update_bits(pmu->regmap, N72_PMU_ILIM2, N72_PMU_ILIM2_ON,
				 N72_PMU_ILIM2_ON);
	if (!ret)
		ret = regmap_update_bits(pmu->regmap, N72_PMU_ILIM,
					 N72_PMU_ILIM_MASK, code);
	return ret;
}

/* the charge current shares register 0x33: see n72_gauge_charge() */
static int n72_pmu_ilim_set_locked(struct n72_pmu_power *pmu, int uA)
{
	int ret;

	mutex_lock(&pmu->chg_lock);
	ret = n72_pmu_ilim_set(pmu, uA);
	mutex_unlock(&pmu->chg_lock);
	return ret;
}

/*
 * Fuel-gauge estimation for the N72 battery cell. There is no current
 * measurement: the state of charge comes from cell voltage alone, so the
 * cell must not be charging when it is read.
 *
 *  - On the battery: one sample a minute. On external power: every three
 *    minutes the charge current is set to 0, the sample is taken, and the
 *    current is put back ten seconds later. Nothing is sampled at a plug.
 *  - A sample is the voltage plus what the load takes off it (100 mV on the
 *    battery, 40 mV on a USB port that gives under 500 mA, nothing with
 *    500 mA or more; 18 mV less with the backlight off), looked up in the
 *    cell's open-circuit table, to the nearest percent.
 *  - The level is the mean of the newest four samples. The samples are
 *    dropped at a wake, which takes one at once.
 *  - With 500 mA or more available and the mean at 90 or above: at most 95
 *    while the charger charges, 100 ("full") once it has stopped.
 *  - A charger timeout below 90 is reset after five minutes.
 *  - The battery's temperature is read before each sample, and the charge
 *    current follows it: less in the cold, none below 0 or above 45 degrees.
 */
#define N72_GAUGE_PERIOD	(60 * HZ)
#define N72_GAUGE_EXT_PERIOD	(180 * HZ)
#define N72_GAUGE_PAUSE		(10 * HZ)
#define N72_GAUGE_TIMEOUT	(300 * HZ)
#define N72_PMU_CHG_CURRENT	GENMASK(7, 2)	/* of register 0x33 */
#define N72_PMU_CHG_TIMER	GENMASK(6, 3)	/* of register 0x34 */

/*
 * Charge current selected by battery temperature ("charge-currents" table).
 * The value goes into register 0x33 bits 7:2 (step size in mA unknown).
 * A row stays active until the temperature moves outside [min, max]
 * (both endpoints inclusive), preventing hysteresis chatter on a boundary.
 * Charging is disabled when outside all rows.
 *
 * PMU register 0xb1 bits 6:1 is unused here: reading 0xb1 times out on I2C
 * on this hardware (register address acked, data transfer stalls), and the
 * default cap of 62 already exceeds every table entry.
 */
static const struct {
	s16 min, max;		/* 0.01 degrees C */
	u8 code;
} n72_chg_rows[] = {
	{    0, 1000, 20 },
	{ 1000, 1500, 30 },
	{ 1500, 2000, 40 },
	{ 2000, 4500, 59 },
};

/*
 * The cell's NTC, ohms to 0.01 degrees C: iOS's table, from the highest
 * resistance the ADC can read (50 uA through it, 2.5 V full scale).
 */
static const struct {
	u16 ohm;
	s16 temp;
} n72_ntc[] = {
	{ 53460, -1500 }, { 42450, -1000 }, { 33930,  -500 }, { 27280,     0 },
	{ 22070,   500 }, { 17960,  1000 }, { 14700,  1500 }, { 12090,  2000 },
	{ 10000,  2500 }, {  8310,  3000 }, {  6940,  3500 }, {  5830,  4000 },
	{  4910,  4500 }, {  4160,  5000 }, {  3540,  5500 }, {  3020,  6000 },
	{  2590,  6500 }, {  2230,  7000 }, {  1920,  7500 }, {  1670,  8000 },
	{  1450,  8500 }, {  1270,  9000 }, {  1110,  9500 }, {   980, 10000 },
	{   860, 10500 }, {   760, 11000 }, {   670, 11500 }, {   600, 12000 },
	{   530, 12500 },
};

/* chg_lock held: the battery's temperature; a failed read keeps the last one */
static void n72_gauge_temp(struct n72_pmu_power *pmu)
{
	unsigned int raw;
	int ohm, i, ret;

	mutex_lock(&pmu->adc_lock);
	ret = n72_pmu_adc(pmu, N72_PMU_ADC_NTC, &raw);
	mutex_unlock(&pmu->adc_lock);
	if (ret)
		return;
	ohm = raw * 2500 * 20 / 1024;
	/* between entries i - 1 and i; past the last one, that line carried on */
	for (i = 1; i < ARRAY_SIZE(n72_ntc) - 1 && ohm < n72_ntc[i].ohm; i++)
		;
	WRITE_ONCE(pmu->temp, n72_ntc[i].temp +
		   (n72_ntc[i - 1].temp - n72_ntc[i].temp) * (ohm - n72_ntc[i].ohm) /
		   (n72_ntc[i - 1].ohm - n72_ntc[i].ohm));
	WRITE_ONCE(pmu->temp_ok, true);
}

/*
 * chg_lock held: register 0x33 bits 7:2 to charge with, given what they held
 * until now. With no temperature read yet that stays.
 */
static u8 n72_gauge_code(struct n72_pmu_power *pmu, u8 was)
{
	int t = pmu->temp, i = pmu->chg_row;
	u8 code = 0;

	if (!pmu->temp_ok)
		return was;
	if (i < 0 || t < n72_chg_rows[i].min || t > n72_chg_rows[i].max) {
		for (i = 0; i < ARRAY_SIZE(n72_chg_rows); i++)
			if (t >= n72_chg_rows[i].min && t <= n72_chg_rows[i].max)
				break;
		if (i == ARRAY_SIZE(n72_chg_rows))
			i = -1;
		pmu->chg_row = i;
	}
	if (i >= 0)
		code = FIELD_PREP(N72_PMU_CHG_CURRENT, n72_chg_rows[i].code);
	if (code != was)
		dev_info(pmu->dev,
			 "battery at %d degrees C: charge current %u -> %u\n",
			 DIV_ROUND_CLOSEST(t, 100), was >> 2, code >> 2);
	return code;
}

/* chg_lock held: charge current 0 for a sample, or on again */
static void n72_gauge_charge(struct n72_pmu_power *pmu, bool on)
{
	unsigned int v;
	u8 code;

	if (regmap_read(pmu->regmap, N72_PMU_ILIM, &v))
		return;
	if (!on) {
		pmu->chg_code = v & N72_PMU_CHG_CURRENT;
		if (!pmu->chg_code ||
		    !regmap_update_bits(pmu->regmap, N72_PMU_ILIM,
					N72_PMU_CHG_CURRENT, 0))
			pmu->paused = true;
	} else if (pmu->paused) {
		code = n72_gauge_code(pmu, pmu->chg_code);
		if (code == (v & N72_PMU_CHG_CURRENT) ||
		    !regmap_update_bits(pmu->regmap, N72_PMU_ILIM,
					N72_PMU_CHG_CURRENT, code))
			pmu->paused = false;
	}
}

/* chg_lock held: at a plug, the charge current for the temperature last read */
static void n72_gauge_plugged(struct n72_pmu_power *pmu)
{
	unsigned int v;
	u8 code;

	if (pmu->paused || regmap_read(pmu->regmap, N72_PMU_ILIM, &v))
		return;
	code = n72_gauge_code(pmu, v & N72_PMU_CHG_CURRENT);
	if (code != (v & N72_PMU_CHG_CURRENT))
		regmap_update_bits(pmu->regmap, N72_PMU_ILIM,
				   N72_PMU_CHG_CURRENT, code);
}

/* percent for an open-circuit voltage, to the nearest (the table falls) */
static int n72_gauge_lookup(struct n72_pmu_power *pmu, int mv)
{
	const struct power_supply_battery_ocv_table *t = pmu->info->ocv_table[0];
	int n = pmu->info->ocv_table_size[0], uv = mv * 1000, i;

	if (!t || n < 1)
		return -ENODATA;
	if (uv >= t[0].ocv)
		return t[0].capacity;
	for (i = 1; i < n; i++) {
		if (uv < t[i].ocv)
			continue;
		return t[i].capacity +
		       DIV_ROUND_CLOSEST((t[i - 1].capacity - t[i].capacity) * (uv - t[i].ocv),
					 t[i - 1].ocv - t[i].ocv);
	}
	return t[n - 1].capacity;
}

/* chg_lock held: one sample into the ring */
static void n72_gauge_sample(struct n72_pmu_power *pmu, bool usb)
{
	unsigned int raw, bl;
	int corr = 100, pct, i, sum = 0, ret;

	n72_gauge_temp(pmu);	/* as iOS: before each voltage sample */
	if (!pmu->info)
		return;
	mutex_lock(&pmu->adc_lock);
	ret = n72_pmu_adc(pmu, N72_PMU_ADC_VBAT, &raw);
	mutex_unlock(&pmu->adc_lock);
	if (ret)
		return;

	if (usb)
		corr -= n72_pmu_ilim_get(pmu) >= 500000 ? 100 : 60;
	if (regmap_read(pmu->regmap, N72_PMU_BL_CTRL, &bl) ||
	    !(bl & N72_PMU_BL_CTRL_ON))
		corr -= 18;
	pct = n72_gauge_lookup(pmu, 2500 + raw * 2000 / 1024 + max(corr, 0));
	if (pct < 0)
		return;

	/*
	 * The probe's sample is taken while the system boots, and that load
	 * pulls the cell further down than the correction above allows for
	 * (76 % seen for a nearly full cell). iOS has iBoot's "boot-voltage",
	 * measured before its kernel runs, for this first value; Linux has
	 * nothing like it. So the probe's sample is shown until the next one
	 * and then dropped, not averaged in.
	 */
	if (pmu->boot_sample) {
		pmu->ring_n = 0;
		pmu->boot_sample = false;
	}
	memmove(pmu->ring + 1, pmu->ring, sizeof(pmu->ring) - 1);
	pmu->ring[0] = pct;
	if (pmu->ring_n < ARRAY_SIZE(pmu->ring))
		pmu->ring_n++;
	for (i = 0; i < pmu->ring_n; i++)
		sum += pmu->ring[i];
	pmu->avg = sum / pmu->ring_n;
}

/* chg_lock held: the level to report; true if it changed */
static bool n72_gauge_publish(struct n72_pmu_power *pmu, bool usb)
{
	int level = pmu->avg;

	if (!pmu->ring_n)
		return false;
	if (usb && level >= 90 && n72_pmu_ilim_get(pmu) >= 500000)
		level = pmu->charging ? min(level, 95) : 100;
	if (level == pmu->level)
		return false;
	WRITE_ONCE(pmu->level, level);
	return true;
}

static void n72_gauge_work(struct work_struct *work)
{
	struct n72_pmu_power *pmu = container_of(work, struct n72_pmu_power, gauge_work.work);
	unsigned long delay = N72_GAUGE_PERIOD;
	bool usb, was_charging, changed;
	unsigned int st1, v;

	mutex_lock(&pmu->chg_lock);
	usb = n72_pmu_usb_online(pmu);
	was_charging = pmu->charging;
	if (pmu->paused) {
		/* the sample is ten seconds old: charge again */
		n72_gauge_charge(pmu, true);
		/* the probe's sample is replaced after a minute, not three */
		pmu->next_sample = jiffies + (pmu->boot_sample ? N72_GAUGE_PERIOD :
								 N72_GAUGE_EXT_PERIOD);
	} else if (!usb) {
		pmu->charging = false;
		pmu->timeout_seen = 0;
		n72_gauge_sample(pmu, false);
	} else {
		if (!regmap_read(pmu->regmap, N72_PMU_STATUS + 1, &st1)) {
			pmu->charging = st1 & N72_PMU_STATUS_CHARGING;
			if (!(st1 & N72_PMU_STATUS_TIMEOUT)) {
				pmu->timeout_seen = 0;
			} else if (!pmu->timeout_seen) {
				pmu->timeout_seen = jiffies | 1;
			} else if (time_after(jiffies, pmu->timeout_seen + N72_GAUGE_TIMEOUT)) {
				if (pmu->avg < 90 &&
				    !regmap_read(pmu->regmap, N72_PMU_ILIM2, &v)) {
					regmap_update_bits(pmu->regmap, N72_PMU_ILIM2,
							   N72_PMU_CHG_TIMER, 0);
					regmap_write(pmu->regmap, N72_PMU_ILIM2, v);
					dev_info(pmu->dev, "charger timeout reset\n");
				}
				pmu->timeout_seen = 0;
			}
		}
		/*
		 * By the clock only (a wake and the probe set it to now): a
		 * sample that fails is tried again in three minutes, not at
		 * every tick with the charger paused each time.
		 */
		if (pmu->info && time_after_eq(jiffies, pmu->next_sample)) {
			n72_gauge_charge(pmu, false);
			n72_gauge_sample(pmu, true);
			delay = N72_GAUGE_PAUSE;
		}
	}
	changed = n72_gauge_publish(pmu, usb) || was_charging != pmu->charging;
	mutex_unlock(&pmu->chg_lock);

	if (changed)
		power_supply_changed(pmu->bat);
	schedule_delayed_work(&pmu->gauge_work, delay);
}

/* stop sampling, and never leave the charger paused */
static void n72_gauge_stop(void *data)
{
	struct n72_pmu_power *pmu = data;

	cancel_delayed_work_sync(&pmu->gauge_work);
	mutex_lock(&pmu->chg_lock);
	n72_gauge_charge(pmu, true);
	mutex_unlock(&pmu->chg_lock);
}

static int n72_pmu_bat_status(struct n72_pmu_power *pmu)
{
	bool charging;
	unsigned int st1;
	int ret;

	if (!n72_pmu_usb_online(pmu))
		return POWER_SUPPLY_STATUS_DISCHARGING;
	mutex_lock(&pmu->chg_lock);
	charging = pmu->charging;
	if (!pmu->paused && !regmap_read(pmu->regmap, N72_PMU_STATUS + 1, &st1))
		charging = st1 & N72_PMU_STATUS_CHARGING;
	if (charging)
		ret = POWER_SUPPLY_STATUS_CHARGING;
	else if (pmu->avg >= 90 && pmu->ring_n && n72_pmu_ilim_get(pmu) >= 500000)
		ret = POWER_SUPPLY_STATUS_FULL;
	else
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
	mutex_unlock(&pmu->chg_lock);
	return ret;
}

/*
 * USB power is there (at probe, and at every plug: the PMU is back at
 * 100 mA after each). Like iOS once a computer has configured the iPod:
 * 500 mA. The gadget enumerates within seconds; a wall charger allows more,
 * which userspace can set. At 100 mA the running system takes all of it,
 * the battery does not charge and the charger reports itself done.
 */
/*
 * Register 0x0a bits 3:2 also gate the USB input current limit: bit 3 holds
 * the limit below 100 mA (typically left set on shutdown when supply is
 * removed). While bit 3 is set the PMU draws nothing from USB regardless of
 * 0x33, so clear bits 3:2 on supply changes.
 */
#define N72_PMU_USB_HOLD	0x0a
#define N72_PMU_USB_HOLD_MASK	(BIT(3) | BIT(2))

static void n72_pmu_usb_release(struct n72_pmu_power *pmu)
{
	unsigned int v;

	if (!regmap_read(pmu->regmap, N72_PMU_USB_HOLD, &v) &&
	    (v & N72_PMU_USB_HOLD_MASK) &&
	    !regmap_update_bits(pmu->regmap, N72_PMU_USB_HOLD,
				N72_PMU_USB_HOLD_MASK, 0))
		dev_dbg(pmu->dev, "USB supply was held under 100 mA (0x0a = %02x): released\n", v);
}

static void n72_pmu_usb_plugged(struct n72_pmu_power *pmu)
{
	if (n72_pmu_usb_online(pmu))
		n72_pmu_usb_release(pmu);
	if (n72_pmu_usb_online(pmu) && n72_pmu_ilim_get(pmu) < 500000 &&
	    !n72_pmu_ilim_set_locked(pmu, 500000))
		dev_info(pmu->dev, "USB input current limit 100 -> 500 mA\n");
	/* as iOS: no sample at the plug, the first paused one three minutes on */
	mutex_lock(&pmu->chg_lock);
	pmu->next_sample = jiffies + N72_GAUGE_EXT_PERIOD;
	n72_gauge_plugged(pmu);
	mutex_unlock(&pmu->chg_lock);
}

static int n72_pmu_usb_get(struct power_supply *psy, enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct n72_pmu_power *pmu = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = n72_pmu_usb_online(pmu);
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = n72_pmu_ilim_get(pmu);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	default:
		return -EINVAL;
	}
}

static int n72_pmu_usb_set(struct power_supply *psy, enum power_supply_property psp,
			   const union power_supply_propval *val)
{
	if (psp != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return -EINVAL;
	return n72_pmu_ilim_set_locked(power_supply_get_drvdata(psy), val->intval);
}

static int n72_pmu_usb_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
}

static const enum power_supply_property n72_pmu_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static char *n72_pmu_usb_supplies[] = { "battery" };

static const struct power_supply_desc n72_pmu_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = n72_pmu_usb_props,
	.num_properties = ARRAY_SIZE(n72_pmu_usb_props),
	.get_property = n72_pmu_usb_get,
	.set_property = n72_pmu_usb_set,
	.property_is_writeable = n72_pmu_usb_writeable,
};

/* USB plug and unplug, from the PMU's interrupt */
static int n72_pmu_power_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct n72_pmu_power *pmu = container_of(nb, struct n72_pmu_power, nb);

	if (event == N72_PMU_EVENT_USB_IN)
		n72_pmu_usb_plugged(pmu);
	if (event == N72_PMU_EVENT_USB_IN || event == N72_PMU_EVENT_USB_OUT) {
		power_supply_changed(pmu->usb);
		/* the battery's status follows the cable */
		power_supply_changed(pmu->bat);
	}
	return NOTIFY_DONE;
}

static int n72_pmu_power_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {
		/* the PMU's node, which has "monitored-battery" */
		.fwnode = dev_fwnode(dev->parent),
	};
	struct n72_pmu_power *pmu;
	struct power_supply *psy;
	int ret;

	pmu = devm_kzalloc(dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;
	pmu->core = dev_get_drvdata(dev->parent);
	pmu->regmap = dev_get_regmap(dev->parent, NULL);
	if (!pmu->regmap)
		return -ENODEV;
	pmu->dev = dev;
	platform_set_drvdata(pdev, pmu);
	cfg.drv_data = pmu;

	mutex_init(&pmu->adc_lock);
	mutex_init(&pmu->chg_lock);
	INIT_DELAYED_WORK(&pmu->gauge_work, n72_gauge_work);
	pmu->level = -1;
	pmu->chg_row = -1;
	psy = devm_power_supply_register(dev, &n72_pmu_bat_desc, &cfg);
	if (IS_ERR(psy))
		return PTR_ERR(psy);
	pmu->bat = psy;
	ret = power_supply_get_battery_info(pmu->bat, &pmu->info);
	if (ret) {
		dev_warn(dev, "no battery OCV table: capacity unavailable (%d)\n", ret);
		pmu->info = NULL;
	}
	cfg.fwnode = NULL;
	cfg.supplied_to = n72_pmu_usb_supplies;
	cfg.num_supplicants = ARRAY_SIZE(n72_pmu_usb_supplies);
	psy = devm_power_supply_register(dev, &n72_pmu_usb_desc, &cfg);
	if (IS_ERR(psy))
		return PTR_ERR(psy);
	pmu->usb = psy;
	/*
	 * USB plug and unplug are events of the "usb" supply from here on;
	 * the PMU's core can report a plug before its interrupts are
	 * unmasked below. Without the PMU's interrupt there are none.
	 */
	pmu->nb.notifier_call = n72_pmu_power_event;
	ret = devm_n72_pmu_notifier(dev, pmu->core, &pmu->nb);
	if (ret && ret != -ENXIO)
		return ret;
	pmu->usb_irq = !ret;
	n72_pmu_usb_plugged(pmu);
	/*
	 * A charge current of 0 here is a pause that was never ended (a
	 * crash or a reset in its ten seconds; restart, kexec and sleep put
	 * it back), or a battery too cold or too warm to charge when the last
	 * kernel went. The boot chain is not known to set it again: iOS's
	 * smallest value (its 0-10 degree row), so the battery charges at all
	 * should the temperature not be readable. The first sample reads it
	 * and sets what it allows. Normally the boot chain leaves 59.
	 */
	{
		unsigned int lim;

		if (!regmap_read(pmu->regmap, N72_PMU_ILIM, &lim) &&
		    !(lim & N72_PMU_CHG_CURRENT) &&
		    !regmap_update_bits(pmu->regmap, N72_PMU_ILIM, N72_PMU_CHG_CURRENT,
					FIELD_PREP(N72_PMU_CHG_CURRENT, 20)))
			dev_warn(dev, "charge current was 0: set to 20, iOS's lowest\n");
	}
	/* the first sample now (charging paused for it), then by the clock */
	ret = devm_add_action(dev, n72_gauge_stop, pmu);
	if (ret)
		return ret;
	pmu->next_sample = jiffies;
	n72_gauge_work(&pmu->gauge_work.work);
	mutex_lock(&pmu->chg_lock);
	pmu->boot_sample = pmu->ring_n > 0;
	mutex_unlock(&pmu->chg_lock);
	/* USB plug and unplug as events of the "usb" supply */
	if (pmu->usb_irq)
		n72_pmu_usb_irq(pmu->core, true);
	return 0;
}

/*
 * Late, as the PMU's core: after the other devices' suspend, and before the
 * core's own (a child's comes first), which puts its lines to sleep.
 */
static int n72_pmu_power_suspend(struct device *dev)
{
	struct n72_pmu_power *pmu = dev_get_drvdata(dev);

	/* nothing is sampled asleep, and the charger is not left paused */
	n72_gauge_stop(pmu);
	/* neither must a USB plug (the charger already wakes the PMU) */
	if (pmu->usb_irq)
		n72_pmu_usb_irq(pmu->core, false);
	return 0;
}

/* (after the core's resume, which has restored the LDOs and woken its lines) */
static int n72_pmu_power_resume(struct device *dev)
{
	struct n72_pmu_power *pmu = dev_get_drvdata(dev);

	if (pmu->usb_irq) {
		n72_pmu_usb_irq(pmu->core, true);
		/* report a cable change made while asleep */
		power_supply_changed(pmu->usb);
		power_supply_changed(pmu->bat);
	}
	/* as iOS at a wake: the old samples go, one is taken at once */
	mutex_lock(&pmu->chg_lock);
	pmu->ring_n = 0;
	pmu->next_sample = jiffies;
	mutex_unlock(&pmu->chg_lock);
	schedule_delayed_work(&pmu->gauge_work, 0);
	return 0;
}

static const struct dev_pm_ops n72_pmu_power_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(n72_pmu_power_suspend, n72_pmu_power_resume)
};

/* restart, power-off, kexec: the charger must not stay paused */
static void n72_pmu_power_shutdown(struct platform_device *pdev)
{
	n72_gauge_stop(platform_get_drvdata(pdev));
}

static struct platform_driver n72_pmu_power_driver = {
	.driver = {
		.name = "n72-pmu-power",
		.pm = pm_sleep_ptr(&n72_pmu_power_pm_ops),
	},
	.probe = n72_pmu_power_probe,
	.shutdown = n72_pmu_power_shutdown,
};
module_platform_driver(n72_pmu_power_driver);

MODULE_ALIAS("platform:n72-pmu-power");
MODULE_DESCRIPTION("Apple iPod touch 2G (Dialog D1759) PMU battery and USB power");
MODULE_LICENSE("GPL");
