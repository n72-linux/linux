// SPDX-License-Identifier: GPL-2.0-only
/*
 * PMU of the Apple iPod touch (2nd generation), N72: the real-time clock
 * and its wake alarm. See drivers/mfd/apple-n72-pmu-core.c.
 */

#include <linux/mfd/apple-n72-pmu.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

/*
 * Real-time clock: a free-running 32-bit seconds counter, and a 32-bit
 * offset that software keeps so that counter + offset is the time (Unix
 * seconds, as xnu uses it). Both little-endian.
 */
#define N72_PMU_RTC_COUNT	0x5c
#define N72_PMU_RTC_OFFSET	0x64

/*
 * Besides the seconds counter the RTC keeps a binary date clock (0x50-0x55:
 * second, minute, hour, day, month, year from 2000 in six bits) and an
 * alarm against it (0x56-0x5a: minute, hour, day, month, year; bit 6 of
 * the year turns it on), like Dialog's DA9052. Nothing reads the date clock
 * as the time: this driver uses it for the alarm only. The alarm has no
 * seconds, so the date clock is set to be at the start of the alarm's minute
 * when the time is the alarm's: write "now" less the alarm's seconds to the
 * date clock and the alarm with its "on" bit, eleven bytes at 0x50 in one
 * transfer.
 *
 * Register behaviour on this hardware:
 *
 * - A write to the date clock goes to holding registers; the clock takes
 *   them at its next tick, in place of counting. So the value has to be
 *   the one for the second after the write (otherwise the alarm fires one
 *   second late).
 * - Reading the seconds (0x50) copies the running clock into the holding
 *   registers. A read between a write and the next tick therefore undoes
 *   the write, and the tick loads the old value: the clock loses a second.
 *   Reads of the other registers do not. Nothing may read 0x50 after the
 *   write; set_alarm reads it before.
 * - The date clock ticks together with the seconds counter, and counts
 *   through the calendar (months, leap years) correctly.
 * - The alarm registers act at once, not at the tick. INT1 bit 2 latches
 *   when the date clock comes into the alarm's minute while the alarm is
 *   on (by counting, or by the tick that loads a written value), and when
 *   the year register is written with the "on" bit while the date clock
 *   is in the alarm's minute. It latches once, not again while the minute
 *   lasts, and also while masked; a set bit raises the interrupt when
 *   unmasked.
 *
 * So the alarm is turned off, then one transfer sets the date clock and
 * the alarm, in a minute the running clock is not in: it runs on until
 * the tick, and were it in the alarm's minute, the alarm would ring at
 * once. The same again if the counter ticked meanwhile: the value written
 * was for the wrong second then. The alarm rings at the tick that starts
 * its second. The PMU's interrupt line (interrupt 0x61 on the GPIO
 * controller) is low while an unmasked event is pending, and reading INT1
 * clears it.
 */
#define N72_PMU_RTC_DATE	0x50
#define N72_PMU_RTC_DATE_SEC	0x3f
#define N72_PMU_RTC_DATE_SET	0x40		/* in the seconds: iOS writes it, it reads as set */
/* what the date clock's years cover: 2000 to 2063 */
#define N72_PMU_DATE_MIN	946684800LL
#define N72_PMU_DATE_END	2966371200LL

struct n72_pmu_rtc {
	struct n72_pmu *pmu;
	struct regmap *regmap;
	struct device *dev;
	struct rtc_device *rtc_dev;
	time64_t alarm;			/* the last alarm set */
	struct notifier_block nb;
};

/* The counter ticks while it is read: read it until two reads agree. */
static int n72_pmu_rtc_count(struct regmap *regmap, u32 *count)
{
	__le32 a, b;
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = regmap_bulk_read(regmap, N72_PMU_RTC_COUNT, &a, 4);
		if (!ret)
			ret = regmap_bulk_read(regmap, N72_PMU_RTC_COUNT, &b, 4);
		if (ret)
			return ret;
		if (a == b) {
			*count = le32_to_cpu(a);
			return 0;
		}
	}
	return -EIO;
}

static int n72_pmu_rtc_read(struct device *dev, struct rtc_time *tm)
{
	struct n72_pmu_rtc *rtc = dev_get_drvdata(dev);
	__le32 off;
	u32 count;
	int ret;

	ret = n72_pmu_rtc_count(rtc->regmap, &count);
	if (ret)
		return ret;
	ret = regmap_bulk_read(rtc->regmap, N72_PMU_RTC_OFFSET, &off, 4);
	if (ret)
		return ret;
	rtc_time64_to_tm((u32)(count + le32_to_cpu(off)), tm);
	return 0;
}

static int n72_pmu_rtc_set(struct device *dev, struct rtc_time *tm)
{
	struct n72_pmu_rtc *rtc = dev_get_drvdata(dev);
	__le32 off;
	u32 count;
	int ret;

	ret = n72_pmu_rtc_count(rtc->regmap, &count);
	if (ret)
		return ret;
	off = cpu_to_le32((u32)rtc_tm_to_time64(tm) - count);
	return regmap_bulk_write(rtc->regmap, N72_PMU_RTC_OFFSET, &off, 4);
}

/*
 * INT1 is not read here to drop an old alarm: the read would take the
 * other events with it, and an alarm that rang a moment ago. The RTC core
 * goes by the time when an alarm interrupt comes.
 */
static int n72_pmu_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct n72_pmu_rtc *rtc = dev_get_drvdata(dev);
	int ret;

	if (enabled) {
		ret = n72_pmu_int1_mask(rtc->pmu, N72_PMU_INT1_ALARM, false);
		if (!ret)
			ret = regmap_update_bits(rtc->regmap, N72_PMU_RTC_ALARM_Y,
						 N72_PMU_RTC_ALARM_ON,
						 N72_PMU_RTC_ALARM_ON);
	} else {
		ret = regmap_update_bits(rtc->regmap, N72_PMU_RTC_ALARM_Y,
					 N72_PMU_RTC_ALARM_ON, 0);
		if (!ret)
			ret = n72_pmu_int1_mask(rtc->pmu, N72_PMU_INT1_ALARM, true);
	}
	return ret;
}

static int n72_pmu_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct n72_pmu_rtc *rtc = dev_get_drvdata(dev);
	unsigned int y;
	int ret;

	ret = regmap_read(rtc->regmap, N72_PMU_RTC_ALARM_Y, &y);
	if (ret)
		return ret;
	rtc_time64_to_tm(rtc->alarm, &alrm->time);
	alrm->enabled = !!(y & N72_PMU_RTC_ALARM_ON);
	return 0;
}

/*
 * The running date clock in seconds, or -1 if it holds no date. Not to be
 * called between a write to the date clock and the tick that takes it.
 */
static int n72_pmu_date_read(struct regmap *regmap, time64_t *t)
{
	struct rtc_time tm = { };
	u8 a[6], b[6];
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = regmap_bulk_read(regmap, N72_PMU_RTC_DATE, a, 6);
		if (!ret)
			ret = regmap_bulk_read(regmap, N72_PMU_RTC_DATE, b, 6);
		if (ret)
			return ret;
		if (!memcmp(a, b, 6))
			break;
	}
	if (i == 5)
		return -EIO;
	tm.tm_sec = a[0] & N72_PMU_RTC_DATE_SEC;
	tm.tm_min = a[1];
	tm.tm_hour = a[2];
	tm.tm_mday = a[3];
	tm.tm_mon = a[4] - 1;
	tm.tm_year = (a[5] & 0x3f) + 100;
	*t = rtc_valid_tm(&tm) ? -1 : rtc_tm_to_time64(&tm);
	return 0;
}

/* minute, hour, day, month, year: the date clock's and the alarm's */
static void n72_pmu_date_regs(time64_t t, u8 *b)
{
	struct rtc_time tm;

	rtc_time64_to_tm(t, &tm);
	b[0] = tm.tm_min;
	b[1] = tm.tm_hour;
	b[2] = tm.tm_mday;
	b[3] = tm.tm_mon + 1;
	b[4] = tm.tm_year - 100;
}

static int n72_pmu_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct n72_pmu_rtc *rtc = dev_get_drvdata(dev);
	struct n72_pmu *pmu = rtc->pmu;
	struct regmap *regmap = rtc->regmap;
	time64_t t = rtc_tm_to_time64(&alrm->time), now, date, minute, live;
	u32 count, after;
	__le32 off;
	u8 regs[11];
	int i, ret;

	ret = n72_pmu_int1_mask(pmu, N72_PMU_INT1_ALARM, true);
	if (ret)
		return ret;
	for (i = 0; i < 5; i++) {
		/* disable alarm before reprogramming */
		ret = regmap_write(regmap, N72_PMU_RTC_ALARM_Y, 0);
		if (!ret)
			ret = n72_pmu_rtc_count(regmap, &count);
		if (ret)
			return ret;
		ret = regmap_bulk_read(regmap, N72_PMU_RTC_OFFSET, &off, 4);
		if (ret)
			return ret;
		now = (u32)(count + le32_to_cpu(off));
		/* due already (the second ticked since the RTC core looked) */
		if (t <= now)
			return -ETIME;
		ret = n72_pmu_date_read(regmap, &live);
		if (ret)
			return ret;

		/*
		 * The alarm's minute on the date clock. That is the time's
		 * own, as iOS has it, unless the date clock cannot hold it:
		 * then counted from the start of the date clock's years.
		 */
		minute = t - alrm->time.tm_sec;
		if (now < N72_PMU_DATE_MIN + 60 || t + 180 >= N72_PMU_DATE_END) {
			minute += N72_PMU_DATE_MIN + 60 - (now - (u32)now % 60);
			if (minute + 180 >= N72_PMU_DATE_END)
				return -ERANGE;
		}
		/*
		 * Not a minute the running date clock is in, or enters with
		 * a tick before the write.
		 */
		while (live >= minute - 1 && live < minute + 60)
			minute += 60;

		/* the date clock at the next tick, when the time is now + 1 */
		date = minute - (t - (now + 1));
		regs[0] = (u32)date % 60 | N72_PMU_RTC_DATE_SET;
		n72_pmu_date_regs(date, regs + 1);
		n72_pmu_date_regs(minute, regs + 6);
		if (alrm->enabled)
			regs[10] |= N72_PMU_RTC_ALARM_ON;
		ret = regmap_bulk_write(regmap, N72_PMU_RTC_DATE, regs, sizeof(regs));
		if (!ret)
			ret = n72_pmu_rtc_count(regmap, &after);
		if (ret)
			break;
		if (after == count) {
			dev_dbg(dev, "alarm %lld in %lld s: date clock %lld, minute %lld\n",
				t, t - now, date, minute);
			rtc->alarm = t;
			return alrm->enabled ?
			       n72_pmu_int1_mask(pmu, N72_PMU_INT1_ALARM, false) : 0;
		}
		/* a tick since the time was read: that was for the second before */
	}
	regmap_write(regmap, N72_PMU_RTC_ALARM_Y, 0);
	return ret ? ret : -EIO;
}

static const struct rtc_class_ops n72_pmu_rtc_ops = {
	.read_time = n72_pmu_rtc_read,
	.set_time = n72_pmu_rtc_set,
	.read_alarm = n72_pmu_read_alarm,
	.set_alarm = n72_pmu_set_alarm,
	.alarm_irq_enable = n72_pmu_alarm_irq_enable,
};

/*
 * The alarm rang. The interrupt wakes a system that is asleep; one that is
 * on its way to sleep (processes frozen, devices being suspended) would
 * handle it and sleep on, the alarm spent. As a wakeup event it has that
 * suspend given up instead.
 */
static void n72_pmu_alarm_rang(struct n72_pmu_rtc *rtc)
{
	pm_wakeup_hard_event(rtc->dev->parent);
	rtc_update_irq(rtc->rtc_dev, 1, RTC_IRQF | RTC_AF);
}

static int n72_pmu_rtc_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct n72_pmu_rtc *rtc = container_of(nb, struct n72_pmu_rtc, nb);

	if (event == N72_PMU_EVENT_ALARM)
		n72_pmu_alarm_rang(rtc);
	return NOTIFY_DONE;
}

static int n72_pmu_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n72_pmu_rtc *rtc;
	int ret;

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;
	rtc->pmu = dev_get_drvdata(dev->parent);
	rtc->regmap = dev_get_regmap(dev->parent, NULL);
	if (!rtc->regmap)
		return -ENODEV;
	rtc->dev = dev;
	platform_set_drvdata(pdev, rtc);

	rtc->rtc_dev = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc->rtc_dev))
		return PTR_ERR(rtc->rtc_dev);
	rtc->rtc_dev->ops = &n72_pmu_rtc_ops;
	rtc->rtc_dev->range_min = 0;
	rtc->rtc_dev->range_max = U32_MAX;
	/* the alarm is off: the PMU's core turns it off before it takes the interrupt */
	rtc->nb.notifier_call = n72_pmu_rtc_event;
	ret = devm_n72_pmu_notifier(dev, rtc->pmu, &rtc->nb);
	if (ret == -ENXIO) {
		clear_bit(RTC_FEATURE_ALARM, rtc->rtc_dev->features);
	} else {
		if (ret)
			return ret;
		/* the RTC core and alarmtimer take an alarm that wakes from here */
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
	}
	return devm_rtc_register_device(rtc->rtc_dev);
}

static struct platform_driver n72_pmu_rtc_driver = {
	.driver = {
		.name = "n72-pmu-rtc",
	},
	.probe = n72_pmu_rtc_probe,
};
module_platform_driver(n72_pmu_rtc_driver);

MODULE_ALIAS("platform:n72-pmu-rtc");
MODULE_DESCRIPTION("Apple iPod touch 2G (Dialog D1759) PMU RTC");
MODULE_LICENSE("GPL");
