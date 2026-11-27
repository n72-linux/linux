// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  isl29003.c - Linux kernel module for
 * 	Intersil ISL29003 ambient light sensor
 *
 *  See file:Documentation/misc-devices/isl29003.rst
 *
 *  Copyright (c) 2009 Daniel Mack <daniel@caiaq.de>
 *
 *  Based on code written by
 *  	Rodolfo Giometti <giometti@linux.it>
 *  	Eurotech S.p.A. <info@eurotech.it>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define ISL29003_DRV_NAME	"isl29003"
#define DRIVER_VERSION		"1.0"

#define ISL29003_REG_COMMAND		0x00
#define ISL29003_ADC_ENABLED		(1 << 7)
#define ISL29003_ADC_PD			(1 << 6)
#define ISL29003_TIMING_INT		(1 << 5)
#define ISL29003_MODE_SHIFT		(2)
#define ISL29003_MODE_MASK		(0x3 << ISL29003_MODE_SHIFT)
#define ISL29003_RES_SHIFT		(0)
#define ISL29003_RES_MASK		(0x3 << ISL29003_RES_SHIFT)

#define ISL29003_REG_CONTROL		0x01
#define ISL29003_INT_FLG		(1 << 5)
#define ISL29003_RANGE_SHIFT		(2)
#define ISL29003_RANGE_MASK		(0x3 << ISL29003_RANGE_SHIFT)
#define ISL29003_INT_PERSISTS_SHIFT	(0)
#define ISL29003_INT_PERSISTS_MASK	(0xf << ISL29003_INT_PERSISTS_SHIFT)

#define ISL29003_REG_IRQ_THRESH_HI	0x02
#define ISL29003_REG_IRQ_THRESH_LO	0x03
#define ISL29003_REG_LSB_SENSOR		0x04
#define ISL29003_REG_MSB_SENSOR		0x05
#define ISL29003_REG_LSB_TIMER		0x06
#define ISL29003_REG_MSB_TIMER		0x07

#define ISL29003_NUM_CACHABLE_REGS	4
#define ISL29003_CAL_RES_12BIT		1
#define ISL29003_CAL_MODE_D1		0
#define ISL29003_CAL_MODE_D2		1

struct isl29003_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[ISL29003_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
	bool calibrated;
	u32 rext_ohm;
	u32 lux_scale;
	u32 lux_offset;
	u32 cal_k;
	u32 ratio_limits[3];
	u32 slopes[3];
	u32 intercepts[3];
	bool active;
	bool has_irq;
	struct delayed_work poll_work;
	u8 meas_state;		/* 0 = idle, 1 = waiting D1, 2 = waiting D2 */
	u16 d1;
	int cached_lux;
};

static int gain_range[] = {
	1000, 4000, 16000, 64000
};

/*
 * register access helpers
 */

static int __isl29003_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct isl29003_data *data = i2c_get_clientdata(client);

	return (data->reg_cache[reg] & mask) >> shift;
}

static int __isl29003_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct isl29003_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;

	if (reg >= ISL29003_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int isl29003_get_range(struct i2c_client *client)
{
	return __isl29003_read_reg(client, ISL29003_REG_CONTROL,
		ISL29003_RANGE_MASK, ISL29003_RANGE_SHIFT);
}

static int isl29003_set_range(struct i2c_client *client, int range)
{
	return __isl29003_write_reg(client, ISL29003_REG_CONTROL,
		ISL29003_RANGE_MASK, ISL29003_RANGE_SHIFT, range);
}

/* resolution */
static int isl29003_get_resolution(struct i2c_client *client)
{
	return __isl29003_read_reg(client, ISL29003_REG_COMMAND,
		ISL29003_RES_MASK, ISL29003_RES_SHIFT);
}

static int isl29003_set_resolution(struct i2c_client *client, int res)
{
	return __isl29003_write_reg(client, ISL29003_REG_COMMAND,
		ISL29003_RES_MASK, ISL29003_RES_SHIFT, res);
}

/* mode */
static int isl29003_get_mode(struct i2c_client *client)
{
	return __isl29003_read_reg(client, ISL29003_REG_COMMAND,
		ISL29003_MODE_MASK, ISL29003_MODE_SHIFT);
}

static int isl29003_set_mode(struct i2c_client *client, int mode)
{
	return __isl29003_write_reg(client, ISL29003_REG_COMMAND,
		ISL29003_MODE_MASK, ISL29003_MODE_SHIFT, mode);
}

/* Calibrated N72 D1/D2 3-curve lux calculation matching AppleISL29003 */
static int isl29003_calc_calibrated_lux(struct isl29003_data *data, u16 d1, u16 d2)
{
	u32 ratio, slope, intercept, lux;
	int idx;

	if (d1 > 0xffe || d2 > 0xffe)
		return 5000;
	if (!d1 || (!d2 && d1 <= 9))
		return 0;

	ratio = (((u32)d2 << 10) / d1 + 1) >> 1;
	if (ratio <= data->ratio_limits[0] || (d1 <= 9 && d1 != d2))
		idx = 0;
	else if (ratio <= data->ratio_limits[1])
		idx = 1;
	else if (ratio <= data->ratio_limits[2])
		idx = d1 > 250 ? 2 : 1;
	else
		return 0;

	slope = data->slopes[idx];
	intercept = data->intercepts[idx];
	lux = (u32)(((u64)data->cal_k *
		     ((u64)intercept * d1 + (u64)slope * d2) + 8192) >> 14);
	return min_t(u32, lux, 5000);
}

/* caller holds data->lock */
static int isl29003_cal_start_d1(struct isl29003_data *data)
{
	struct i2c_client *client = data->client;
	int ret;

	ret = i2c_smbus_write_byte_data(client, ISL29003_REG_CONTROL, 0x00);
	if (!ret)
		ret = i2c_smbus_write_byte_data(client, ISL29003_REG_IRQ_THRESH_HI, 0x00);
	if (!ret)
		ret = i2c_smbus_write_byte_data(client, ISL29003_REG_IRQ_THRESH_LO, 0xff);
	if (!ret)
		ret = i2c_smbus_write_byte_data(client, ISL29003_REG_COMMAND,
						ISL29003_ADC_ENABLED |
						(ISL29003_CAL_MODE_D1 << ISL29003_MODE_SHIFT) |
						ISL29003_CAL_RES_12BIT);
	if (ret) {
		data->meas_state = 0;
		return ret;
	}
	data->meas_state = 1;
	mod_delayed_work(system_wq, &data->poll_work, msecs_to_jiffies(90));
	return 0;
}

/* caller holds data->lock */
static int isl29003_cal_step(struct isl29003_data *data)
{
	struct i2c_client *client = data->client;
	int lsb, msb, ret;
	u16 raw;

	if (!data->active || !data->meas_state)
		return 0;

	lsb = i2c_smbus_read_byte_data(client, ISL29003_REG_LSB_SENSOR);
	if (lsb < 0)
		return lsb;
	msb = i2c_smbus_read_byte_data(client, ISL29003_REG_MSB_SENSOR);
	if (msb < 0)
		return msb;
	raw = ((msb << 8) | lsb) & 0xfff;

	i2c_smbus_write_byte_data(client, ISL29003_REG_CONTROL, 0x00);

	if (data->meas_state == 1) {
		data->d1 = raw;
		ret = i2c_smbus_write_byte_data(client, ISL29003_REG_COMMAND,
						ISL29003_ADC_PD);
		if (!ret)
			ret = i2c_smbus_write_byte_data(client, ISL29003_REG_COMMAND,
							ISL29003_ADC_ENABLED |
							(ISL29003_CAL_MODE_D2 << ISL29003_MODE_SHIFT) |
							ISL29003_CAL_RES_12BIT);
		if (ret) {
			data->meas_state = 0;
			mod_delayed_work(system_wq, &data->poll_work, msecs_to_jiffies(500));
			return ret;
		}
		data->meas_state = 2;
		mod_delayed_work(system_wq, &data->poll_work, msecs_to_jiffies(90));
		return 0;
	}

	WRITE_ONCE(data->cached_lux,
		   isl29003_calc_calibrated_lux(data, data->d1, raw));
	i2c_smbus_write_byte_data(client, ISL29003_REG_COMMAND, ISL29003_ADC_PD);
	data->meas_state = 0;
	mod_delayed_work(system_wq, &data->poll_work, msecs_to_jiffies(500));
	return 0;
}

static void isl29003_poll_work(struct work_struct *work)
{
	struct isl29003_data *data = container_of(work, struct isl29003_data,
						  poll_work.work);

	mutex_lock(&data->lock);
	if (data->active) {
		if (!data->meas_state)
			isl29003_cal_start_d1(data);
		else
			isl29003_cal_step(data);
	}
	mutex_unlock(&data->lock);
}

static irqreturn_t isl29003_irq_handler(int irq, void *dev_id)
{
	struct isl29003_data *data = dev_id;
	int ctrl;

	mutex_lock(&data->lock);
	if (!data->active || !data->meas_state) {
		mutex_unlock(&data->lock);
		return IRQ_NONE;
	}
	ctrl = i2c_smbus_read_byte_data(data->client, ISL29003_REG_CONTROL);
	if (ctrl < 0 || !(ctrl & ISL29003_INT_FLG)) {
		mutex_unlock(&data->lock);
		return IRQ_NONE;
	}
	isl29003_cal_step(data);
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

/* power_state */
static int isl29003_set_power_state(struct i2c_client *client, int state)
{
	struct isl29003_data *data = i2c_get_clientdata(client);
	int ret;

	if (!data->calibrated)
		return __isl29003_write_reg(client, ISL29003_REG_COMMAND,
					    ISL29003_ADC_ENABLED | ISL29003_ADC_PD, 0,
					    state ? ISL29003_ADC_ENABLED : ISL29003_ADC_PD);

	if (state) {
		mutex_lock(&data->lock);
		data->active = true;
		WRITE_ONCE(data->cached_lux, -EAGAIN);
		ret = isl29003_cal_start_d1(data);
		mutex_unlock(&data->lock);
		return ret;
	}

	mutex_lock(&data->lock);
	data->active = false;
	data->meas_state = 0;
	mutex_unlock(&data->lock);
	cancel_delayed_work_sync(&data->poll_work);

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(client, ISL29003_REG_COMMAND, ISL29003_ADC_PD);
	mutex_unlock(&data->lock);
	return ret < 0 ? ret : 0;
}

static int isl29003_get_power_state(struct i2c_client *client)
{
	struct isl29003_data *data = i2c_get_clientdata(client);
	u8 cmdreg;

	if (data->calibrated)
		return READ_ONCE(data->active) ? 1 : 0;

	cmdreg = data->reg_cache[ISL29003_REG_COMMAND];
	return ~cmdreg & ISL29003_ADC_PD;
}

static int isl29003_get_adc_value(struct i2c_client *client)
{
	struct isl29003_data *data = i2c_get_clientdata(client);
	int lsb, msb, range, bitdepth;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, ISL29003_REG_LSB_SENSOR);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29003_REG_MSB_SENSOR);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	range = isl29003_get_range(client);
	bitdepth = (4 - isl29003_get_resolution(client)) * 4;
	return (((msb << 8) | lsb) * gain_range[range]) >> bitdepth;
}

/*
 * sysfs layer
 */

/* range */
static ssize_t isl29003_show_range(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return sysfs_emit(buf, "%i\n", isl29003_get_range(client));
}

static ssize_t isl29003_store_range(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 3)
		return -EINVAL;

	ret = isl29003_set_range(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
		   isl29003_show_range, isl29003_store_range);


/* resolution */
static ssize_t isl29003_show_resolution(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return sysfs_emit(buf, "%d\n", isl29003_get_resolution(client));
}

static ssize_t isl29003_store_resolution(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 3)
		return -EINVAL;

	ret = isl29003_set_resolution(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(resolution, S_IWUSR | S_IRUGO,
		   isl29003_show_resolution, isl29003_store_resolution);

/* mode */
static ssize_t isl29003_show_mode(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return sysfs_emit(buf, "%d\n", isl29003_get_mode(client));
}

static ssize_t isl29003_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 2)
		return -EINVAL;

	ret = isl29003_set_mode(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		   isl29003_show_mode, isl29003_store_mode);


/* power state */
static ssize_t isl29003_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return sysfs_emit(buf, "%d\n", isl29003_get_power_state(client));
}

static ssize_t isl29003_store_power_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 1)
		return -EINVAL;

	ret = isl29003_set_power_state(client, val);
	return ret ? ret : count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		   isl29003_show_power_state, isl29003_store_power_state);


/* lux */
static ssize_t isl29003_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29003_data *data = i2c_get_clientdata(client);
	int lux;

	/* No LUX data if not operational */
	if (!isl29003_get_power_state(client))
		return -EBUSY;

	if (data->calibrated) {
		lux = READ_ONCE(data->cached_lux);
		if (lux < 0)
			return lux;
		return sysfs_emit(buf, "%d\n", lux);
	}

	return sysfs_emit(buf, "%d\n", isl29003_get_adc_value(client));
}

static DEVICE_ATTR(lux, S_IRUGO, isl29003_show_lux, NULL);

static struct attribute *isl29003_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_resolution.attr,
	&dev_attr_mode.attr,
	&dev_attr_power_state.attr,
	&dev_attr_lux.attr,
	NULL
};

static const struct attribute_group isl29003_attr_group = {
	.attrs = isl29003_attributes,
};

static int isl29003_init_client(struct i2c_client *client)
{
	struct isl29003_data *data = i2c_get_clientdata(client);
	int i;

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(client, i);

		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */
	isl29003_set_range(client, 0);
	isl29003_set_resolution(client, 0);
	isl29003_set_mode(client, 0);
	isl29003_set_power_state(client, 0);

	return 0;
}

/*
 * I2C layer
 */

static void isl29003_parse_dt(struct isl29003_data *data)
{
	struct device_node *np = data->client->dev.of_node;

	if (!np)
		return;
	if (of_property_read_u32(np, "apple,external-scaling-resistor-ohm",
				 &data->rext_ohm) || !data->rext_ohm)
		return;
	if (of_property_read_u32(np, "apple,lux-scale", &data->lux_scale))
		return;
	of_property_read_u32(np, "apple,lux-offset", &data->lux_offset);
	if (of_property_read_u32_array(np, "apple,ratio-limits",
				       data->ratio_limits, 3))
		return;
	if (of_property_read_u32_array(np, "apple,slopes", data->slopes, 3))
		return;
	if (of_property_read_u32_array(np, "apple,intercepts",
				       data->intercepts, 3))
		return;
	data->cal_k = ((97300000U / data->rext_ohm) * data->lux_scale) >> 12;
	data->calibrated = true;
}

static int isl29003_probe(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	struct isl29003_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct isl29003_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->cached_lux = -EAGAIN;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);
	INIT_DELAYED_WORK(&data->poll_work, isl29003_poll_work);
	isl29003_parse_dt(data);

	/* initialize the ISL29003 chip */
	err = isl29003_init_client(client);
	if (err)
		goto exit_kfree;

	if (data->calibrated && client->irq > 0) {
		err = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, isl29003_irq_handler,
						IRQF_ONESHOT, ISL29003_DRV_NAME,
						data);
		if (err)
			dev_warn(&client->dev,
				 "IRQ %d unavailable (%d), using polling\n",
				 client->irq, err);
		else
			data->has_irq = true;
	}

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &isl29003_attr_group);
	if (err) {
		if (data->has_irq)
			devm_free_irq(&client->dev, client->irq, data);
		goto exit_kfree;
	}

	dev_info(&client->dev, "driver version %s enabled%s\n", DRIVER_VERSION,
		 data->calibrated ? " (calibrated D1/D2)" : "");
	return 0;

exit_kfree:
	cancel_delayed_work_sync(&data->poll_work);
	kfree(data);
	return err;
}

static void isl29003_remove(struct i2c_client *client)
{
	struct isl29003_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &isl29003_attr_group);
	isl29003_set_power_state(client, 0);
	if (data->has_irq)
		devm_free_irq(&client->dev, client->irq, data);
	cancel_delayed_work_sync(&data->poll_work);
	kfree(data);
}

#ifdef CONFIG_PM_SLEEP
static int isl29003_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29003_data *data = i2c_get_clientdata(client);

	data->power_state_before_suspend = isl29003_get_power_state(client);
	return isl29003_set_power_state(client, 0);
}

static int isl29003_resume(struct device *dev)
{
	int i;
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29003_data *data = i2c_get_clientdata(client);

	if (!data->calibrated) {
		/* restore registers from cache */
		for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++)
			if (i2c_smbus_write_byte_data(client, i, data->reg_cache[i]))
				return -EIO;
	}

	return isl29003_set_power_state(client,
		data->power_state_before_suspend);
}

static SIMPLE_DEV_PM_OPS(isl29003_pm_ops, isl29003_suspend, isl29003_resume);
#define ISL29003_PM_OPS (&isl29003_pm_ops)

#else
#define ISL29003_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id isl29003_id[] = {
	{ "isl29003" },
	{}
};
MODULE_DEVICE_TABLE(i2c, isl29003_id);

static const struct of_device_id isl29003_of_match[] = {
	{ .compatible = "isil,isl29003" },
	{}
};
MODULE_DEVICE_TABLE(of, isl29003_of_match);

static struct i2c_driver isl29003_driver = {
	.driver = {
		.name	= ISL29003_DRV_NAME,
		.of_match_table = isl29003_of_match,
		.pm	= ISL29003_PM_OPS,
	},
	.probe = isl29003_probe,
	.remove	= isl29003_remove,
	.id_table = isl29003_id,
};

module_i2c_driver(isl29003_driver);

MODULE_AUTHOR("Daniel Mack <daniel@caiaq.de>");
MODULE_DESCRIPTION("ISL29003 ambient light sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);
