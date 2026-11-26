// SPDX-License-Identifier: GPL-2.0
/*
 * Apple CD3272 "Mikey" headset remote/microphone detector (iPod touch 2G).
 *
 * Registers:
 *   0  operating mode (see cd_modes[])
 *   1  bias/threshold and reset (0x80)
 *   3  interrupt MASK, written inverted (~mask)
 *   4  detection status: bit0 = nothing inserted, bit2 = microphone,
 *      bits 7:5 = transmitter tone id (1..4)
 *   5  button state
 *   6  interrupt status: 0x18 detection change,
 *      0x20 button event (reg 5), 0x03 centre button press/release
 *   7  chip revision (read once at probe)
 *
 * The interrupt line goes through the PMU and is not identified on the iPod
 * touch 2G, so by default the driver polls.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/workqueue.h>

#define CD_REG_MODE		0x00
#define CD_REG_BIAS		0x01
#define CD_REG_RESET2		0x02
#define CD_REG_IMASK		0x03
#define CD_REG_STATUS		0x04
#define CD_REG_BUTTONS		0x05
#define CD_REG_ISTATUS		0x06
#define CD_REG_REV		0x07

#define CD_ST_NOJACK		BIT(0)
#define CD_ST_MIC		BIT(2)
#define CD_ST_TONE_SHIFT	5
#define CD_ST_TONE_MASK		0x07

#define CD_INT_CENTER_DOWN	BIT(0)
#define CD_INT_CENTER_UP	BIT(1)
#define CD_INT_DETECT		0x18
#define CD_INT_BUTTON		BIT(5)

#define CD_REV_BIAS_HI		0x11	/* selects high bias nibble (0x70 vs 0x30) */

#define CD_POLL_FAST_MS		100
#define CD_POLL_SLOW_MS		1000
#define CD_ARM_TIMEOUT_POLLS	3

enum cd_mode {
	CD_MODE_IDLE0, CD_MODE_IDLE1, CD_MODE_2, CD_MODE_3, CD_MODE_4,
	CD_MODE_5, CD_MODE_SINGLE, CD_MODE_7, CD_MODE_TONE, CD_MODE_DETECT,
};

static const struct {
	u8 reg0;
	u8 mask;
} cd_modes[] = {
	[CD_MODE_IDLE0]	= { 0x01, 0x00 },
	[CD_MODE_IDLE1]	= { 0x01, 0x00 },
	[CD_MODE_2]	= { 0x42, 0x03 },
	[CD_MODE_3]	= { 0x25, 0x00 },
	[CD_MODE_4]	= { 0x26, 0x00 },
	[CD_MODE_5]	= { 0x27, 0x02 },
	[CD_MODE_SINGLE] = { 0x27, 0x03 },
	[CD_MODE_7]	= { 0x37, 0x21 },
	[CD_MODE_TONE]	= { 0x2f, 0x23 },
	[CD_MODE_DETECT] = { 0xaf, 0x18 },
};

/*
 * Headset receiver state machine states.
 */
enum cd_state {
	CD_UNKNOWN,	/* needs (re)detection */
	CD_UNPLUGGED,	/* jack GPIO says nothing is in the jack */
	CD_ARMED,	/* mode 9 armed, waiting for a detection interrupt */
	CD_DETACHED,	/* chip status bit0: no headset */
	CD_HP_ONLY,	/* plug without microphone */
	CD_WAIT_MIC,	/* microphone bit vanished after probing, re-armed */
	CD_SINGLE,	/* microphone + one button (mode 6) */
	CD_TONE,	/* microphone + tone id: multi button (mode 8) */
};

/*
 * Buttons in reg 5: code n <-> bit 2*(n-1).
 * Codes 1 and 2 map to HID consumer usages 0xea (Volume Decrement) and 0xe9
 * (Volume Increment); codes 3 and 4 map to next/previous track.
 */
static const unsigned int cd_codes[] = {
	KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_NEXTSONG, KEY_PREVIOUSSONG,
};

#define CD_NUM_CODES	ARRAY_SIZE(cd_codes)

struct cd3272 {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *hp_det;
	struct delayed_work work;
	struct mutex lock;

	enum cd_state state;
	u8 rev;
	u8 irq_mask;		/* iOS this+0xad */
	u8 last_status;		/* reg 4 at last detection */
	u8 tone_id;
	bool mic;
	bool quirk4;		/* iOS this+0x95: button 4 held at detection */
	bool opened;
	bool chip_irq;
	bool hp_irq;
	bool suspended;
	unsigned int armed_polls;
	unsigned long held;	/* bit n: code n+1 held, bit 7 centre */
};

#define CD_HELD_CENTER	7

static int cd_read(struct cd3272 *cd, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(cd->client, reg);

	/* hardware helper returns 0 on I2C read error */
	return ret < 0 ? 0 : ret;
}

static void cd_write(struct cd3272 *cd, u8 reg, u8 val)
{
	i2c_smbus_write_byte_data(cd->client, reg, val);
}

static void cd_set_mode(struct cd3272 *cd, enum cd_mode mode, bool enable)
{
	cd->irq_mask = enable ? cd_modes[mode].mask : 0;
	cd_write(cd, CD_REG_MODE, cd_modes[mode].reg0);
	msleep(10);
	cd_write(cd, CD_REG_IMASK, ~cd->irq_mask);
}

/* n = 0, 1, 2 selects bias/threshold level */
static void cd_set_bias(struct cd3272 *cd, int n)
{
	static const u8 v[] = { 0x17, 0x16, 0x1a };
	u8 val = (v[n] - 0xc) & 0xf;

	val |= cd->rev == CD_REV_BIAS_HI ? 0x70 : 0x30;
	cd_write(cd, CD_REG_BIAS, val);
}

static void cd_go_idle(struct cd3272 *cd)
{
	cd_set_mode(cd, CD_MODE_IDLE1, false);
}

/* Arm the plug detection interrupt */
static void cd_arm(struct cd3272 *cd)
{
	cd_set_mode(cd, CD_MODE_3, true);
	cd_set_bias(cd, 0);
	cd_read(cd, CD_REG_ISTATUS);		/* clear pending interrupts */
	cd_set_mode(cd, CD_MODE_DETECT, true);
	cd->armed_polls = 0;
}

static void cd_report_key(struct cd3272 *cd, unsigned int bit,
			  unsigned int key, bool down)
{
	if (down == test_bit(bit, &cd->held))
		return;
	__assign_bit(bit, &cd->held, down);
	input_report_key(cd->input, key, down);
	input_sync(cd->input);
}

static void cd_release_all(struct cd3272 *cd)
{
	unsigned int i;

	for (i = 0; i < CD_NUM_CODES; i++)
		cd_report_key(cd, i, cd_codes[i], false);
	cd_report_key(cd, CD_HELD_CENTER, KEY_PLAYPAUSE, false);
}

static void cd_report_mic(struct cd3272 *cd, bool mic)
{
	cd->mic = mic;
	input_report_switch(cd->input, SW_MICROPHONE_INSERT, mic);
	input_sync(cd->input);
}

static bool cd_has_remote(struct cd3272 *cd)
{
	return cd->state == CD_SINGLE || cd->state == CD_TONE;
}

/* read reg 4 until two reads 10 ms apart agree (iOS only waits 1/10 ms) */
static u8 cd_read_status(struct cd3272 *cd)
{
	u8 a, b;
	int i;

	a = cd_read(cd, CD_REG_STATUS);
	for (i = 0; i < 3; i++) {
		msleep(10);
		b = cd_read(cd, CD_REG_STATUS);
		if (a == b)
			break;
		a = b;
	}
	return a;
}

/*
 * Read status register and select final operating mode and bias level.
 */
static void cd_detect(struct cd3272 *cd)
{
	u8 st = cd_read_status(cd);
	unsigned int tone = (st >> CD_ST_TONE_SHIFT) & CD_ST_TONE_MASK;
	bool mic = false;

	cd->last_status = st;
	cd->quirk4 = false;
	cd->tone_id = 0;

	if (st & CD_ST_NOJACK) {
		cd->state = CD_DETACHED;
	} else if (tone >= 1 && tone <= 4) {
		cd->state = CD_TONE;
		cd->tone_id = tone;
	} else if (!(st & CD_ST_MIC)) {
		cd->state = CD_HP_ONLY;
	} else {
		cd->state = CD_SINGLE;
		mic = true;
	}

	if (cd->state == CD_SINGLE) {
		/* modes 3 and 4 with interrupts off, then re-check the mic */
		cd_set_mode(cd, CD_MODE_3, false);
		cd_set_mode(cd, CD_MODE_4, false);
		st = cd_read(cd, CD_REG_STATUS);
		mic = st & CD_ST_MIC;
		if (!mic) {
			/* wait for the next status change */
			cd_arm(cd);
			cd->state = CD_WAIT_MIC;
			cd_release_all(cd);
			cd_report_mic(cd, false);
			return;
		}
	} else if (cd->state == CD_TONE) {
		unsigned int code = 0;
		u8 b;

		cd_set_bias(cd, 1);
		msleep(1);
		mic = cd_read(cd, CD_REG_STATUS) & CD_ST_MIC;
		msleep(10);
		b = cd_read(cd, CD_REG_BUTTONS);
		/* highest button bit wins */
		if (b & 0x40)
			code = 4;
		else if (b & 0x10)
			code = 3;
		else if (b & 0x04)
			code = 2;
		else if (b & 0x01)
			code = 1;
		if (code == 4)
			cd->quirk4 = true;
	}

	switch (cd->state) {
	case CD_TONE:
		cd_set_mode(cd, CD_MODE_TONE, true);
		cd_set_bias(cd, 2);
		break;
	case CD_SINGLE:
		cd_set_bias(cd, 0);
		cd_set_mode(cd, CD_MODE_SINGLE, true);
		break;
	default:
		cd_set_mode(cd, CD_MODE_IDLE0, true);
		break;
	}
	cd_read(cd, CD_REG_ISTATUS);		/* clear pending interrupts */

	if (!cd_has_remote(cd))
		cd_release_all(cd);
	cd_report_mic(cd, mic);
}

/* Reg 5 holds the pressed button bits (2*i) */
static void cd_buttons(struct cd3272 *cd)
{
	u8 b = cd_read(cd, CD_REG_BUTTONS);
	unsigned int i;

	for (i = 0; i < CD_NUM_CODES; i++)
		cd_report_key(cd, i, cd_codes[i], b & BIT(2 * i));
}

static bool cd_plugged(struct cd3272 *cd)
{
	if (!cd->hp_det)
		return true;
	return gpiod_get_value_cansleep(cd->hp_det) > 0;
}

static void cd_service(struct cd3272 *cd)
{
	unsigned int irq;
	u8 st;

	if (cd->suspended)
		return;

	if (!cd_plugged(cd)) {
		if (cd->state != CD_UNPLUGGED) {
			cd_release_all(cd);
			cd_go_idle(cd);
			cd_report_mic(cd, false);
			cd->state = CD_UNPLUGGED;
		}
		return;
	}

	switch (cd->state) {
	case CD_UNKNOWN:
	case CD_UNPLUGGED:
		/* plug event: arm the detect interrupt */
		cd_arm(cd);
		cd->state = CD_ARMED;
		msleep(20);
		irq = cd_read(cd, CD_REG_ISTATUS) & cd->irq_mask;
		if (irq & CD_INT_DETECT)
			cd_detect(cd);
		return;

	case CD_ARMED:
	case CD_WAIT_MIC:
		irq = cd_read(cd, CD_REG_ISTATUS) & cd->irq_mask;
		if ((irq & CD_INT_DETECT) ||
		    ++cd->armed_polls >= CD_ARM_TIMEOUT_POLLS)
			cd_detect(cd);
		return;

	case CD_DETACHED:
	case CD_HP_ONLY:
		/*
		 * iOS leaves mode 0 here and relies on jack events; also
		 * notice a change on reg 4 so a missing GPIO still works
		 */
		st = cd_read(cd, CD_REG_STATUS);
		if (st != cd->last_status) {
			if (cd_read_status(cd) != cd->last_status)
				cd_detect(cd);
		}
		return;

	case CD_SINGLE:
	case CD_TONE:
		irq = cd_read(cd, CD_REG_ISTATUS) & cd->irq_mask;
		if (irq & CD_INT_DETECT) {
			cd_detect(cd);
			return;
		}
		if (irq & CD_INT_CENTER_DOWN)
			cd_report_key(cd, CD_HELD_CENTER, KEY_PLAYPAUSE, true);
		if (irq & CD_INT_CENTER_UP)
			cd_report_key(cd, CD_HELD_CENTER, KEY_PLAYPAUSE, false);
		if ((irq & CD_INT_BUTTON) ||
		    (cd->held & GENMASK(CD_NUM_CODES - 1, 0)))
			cd_buttons(cd);
		return;
	}
}

static unsigned int cd_poll_ms(struct cd3272 *cd)
{
	if (cd->suspended)
		return 0;
	/*
	 * With an interrupt nothing is polled: the PMU reports the jack and
	 * the chip's own line as one interrupt on the iPod touch 2G (hp_irq).
	 * Only the wait for the detection result after arming has a timeout.
	 */
	if (cd->chip_irq || cd->hp_irq)
		return cd->state == CD_ARMED || cd->state == CD_WAIT_MIC ?
		       CD_POLL_SLOW_MS : 0;
	/* without any interrupt: only while somebody listens */
	if (!cd->opened)
		return 0;
	if (cd->state == CD_UNPLUGGED)
		return cd->hp_irq ? 0 : CD_POLL_SLOW_MS;
	if (cd->chip_irq)
		return 0;
	if (cd_has_remote(cd) && cd->opened)
		return CD_POLL_FAST_MS;
	return CD_POLL_SLOW_MS;
}

static void cd_work(struct work_struct *w)
{
	struct cd3272 *cd = container_of(w, struct cd3272, work.work);
	unsigned int ms;

	mutex_lock(&cd->lock);
	cd_service(cd);
	ms = cd_poll_ms(cd);
	mutex_unlock(&cd->lock);

	if (ms)
		schedule_delayed_work(&cd->work, msecs_to_jiffies(ms));
}

static irqreturn_t cd_chip_irq(int irq, void *data)
{
	struct cd3272 *cd = data;

	mutex_lock(&cd->lock);
	cd_service(cd);
	mutex_unlock(&cd->lock);
	return IRQ_HANDLED;
}

static irqreturn_t cd_hp_irq(int irq, void *data)
{
	struct cd3272 *cd = data;

	/*
	 * A jack change or an event of the chip (the PMU cannot tell which):
	 * the service routine looks at the jack and at the chip's status.
	 */
	mod_delayed_work(system_wq, &cd->work, 0);
	return IRQ_HANDLED;
}

static int cd_open(struct input_dev *dev)
{
	struct cd3272 *cd = input_get_drvdata(dev);

	cd->opened = true;
	mod_delayed_work(system_wq, &cd->work, 0);
	return 0;
}

static void cd_close(struct input_dev *dev)
{
	struct cd3272 *cd = input_get_drvdata(dev);

	cd->opened = false;
}

static const char * const cd_state_names[] = {
	[CD_UNKNOWN] = "unknown", [CD_UNPLUGGED] = "unplugged",
	[CD_ARMED] = "armed", [CD_DETACHED] = "detached",
	[CD_HP_ONLY] = "headphones-only", [CD_WAIT_MIC] = "waiting-for-mic",
	[CD_SINGLE] = "single-button", [CD_TONE] = "multi-button",
};

#ifdef CONFIG_DEBUG_FS
/*
 * debugfs (the I2C client's directory) status. Skip reg 6 (CD_REG_ISTATUS)
 * because reading it clears latched events and can swallow an IRQ.
 */
static int cd_status_show(struct seq_file *s, void *unused)
{
	struct cd3272 *cd = s->private;
	static const u8 regs[] = { 0, 1, 3, 4, 5, 7 };
	unsigned int i;

	mutex_lock(&cd->lock);
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		seq_printf(s, "reg%u=0x%02x\n", regs[i], cd_read(cd, regs[i]));
	seq_printf(s,
		   "state=%s tone=%u mic=%d quirk4=%d irq_mask=0x%02x plugged=%d opened=%d\n",
		   cd_state_names[cd->state], cd->tone_id, cd->mic,
		   cd->quirk4, cd->irq_mask,
		   cd->hp_det ? cd_plugged(cd) : -1, cd->opened);
	mutex_unlock(&cd->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(cd_status);

static void cd_debugfs_init(struct i2c_client *client, struct cd3272 *cd)
{
	debugfs_create_file("status", 0444, client->debugfs, cd, &cd_status_fops);
}
#else
static inline void cd_debugfs_init(struct i2c_client *client, struct cd3272 *cd) { }
#endif

static void cd_stop(void *data)
{
	struct cd3272 *cd = data;

	cancel_delayed_work_sync(&cd->work);
	mutex_lock(&cd->lock);
	cd_go_idle(cd);
	mutex_unlock(&cd->lock);
}

static int cd_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cd3272 *cd;
	int ret, rev;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	cd = devm_kzalloc(dev, sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;
	cd->client = client;
	mutex_init(&cd->lock);
	INIT_DELAYED_WORK(&cd->work, cd_work);
	i2c_set_clientdata(client, cd);

	/* Read hardware revision once at probe */
	rev = i2c_smbus_read_byte_data(client, CD_REG_REV);
	if (rev < 0)
		return dev_err_probe(dev, rev, "chip does not answer\n");
	cd->rev = rev;
	dev_info(dev, "CD3272 revision 0x%02x\n", cd->rev);

	/*
	 * The jack line belongs to the sound card (its headphone jack), which
	 * asks for it exclusively: come second. If nobody holds the line yet,
	 * give it back and wait for the card.
	 */
	if (device_property_present(dev, "hp-det-gpios")) {
		struct gpio_desc *g = gpiod_get_optional(dev, "hp-det", GPIOD_IN);

		if (!IS_ERR_OR_NULL(g)) {
			gpiod_put(g);
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "waiting for the sound card to take the jack line\n");
		}
	}
	cd->hp_det = devm_gpiod_get_optional(dev, "hp-det",
					     GPIOD_IN |
					     GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(cd->hp_det))
		return dev_err_probe(dev, PTR_ERR(cd->hp_det),
				     "hp-det gpio\n");

	/*
	 * Reset chip: assert reset bit (0x80) in reg 1, then clear reg 2.
	 */
	cd_write(cd, CD_REG_BIAS, 0x80);
	msleep(10);
	cd_write(cd, CD_REG_RESET2, 0x00);
	cd_go_idle(cd);

	cd->input = devm_input_allocate_device(dev);
	if (!cd->input)
		return -ENOMEM;
	cd->input->name = "Apple headset remote";
	cd->input->phys = "apple-cd3272/input0";
	cd->input->id.bustype = BUS_I2C;
	cd->input->open = cd_open;
	cd->input->close = cd_close;
	input_set_drvdata(cd->input, cd);
	input_set_capability(cd->input, EV_KEY, KEY_PLAYPAUSE);
	input_set_capability(cd->input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(cd->input, EV_KEY, KEY_VOLUMEDOWN);
	input_set_capability(cd->input, EV_KEY, KEY_NEXTSONG);
	input_set_capability(cd->input, EV_KEY, KEY_PREVIOUSSONG);
	input_set_capability(cd->input, EV_SW, SW_MICROPHONE_INSERT);

	ret = input_register_device(cd->input);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, cd_stop, cd);
	if (ret)
		return ret;

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						cd_chip_irq,
						IRQF_ONESHOT, "apple-cd3272",
						cd);
		if (ret)
			return dev_err_probe(dev, ret, "irq\n");
		cd->chip_irq = true;
	}

	if (cd->hp_det) {
		int irq = gpiod_to_irq(cd->hp_det);

		/* shared with the sound card's jack: the same flags as soc-jack */
		if (irq > 0 &&
		    devm_request_any_context_irq(dev, irq, cd_hp_irq,
						 IRQF_TRIGGER_RISING |
						 IRQF_TRIGGER_FALLING |
						 IRQF_SHARED,
						 "apple-cd3272-hp", cd) >= 0)
			cd->hp_irq = true;
	}

	/* the I2C core removes its client directory before devres frees cd */
	cd_debugfs_init(client, cd);

	schedule_delayed_work(&cd->work, 0);
	return 0;
}

static int cd_suspend(struct device *dev)
{
	struct cd3272 *cd = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&cd->work);
	mutex_lock(&cd->lock);
	cd->suspended = true;
	cd_release_all(cd);
	cd_go_idle(cd);
	cd->state = CD_UNKNOWN;
	mutex_unlock(&cd->lock);
	return 0;
}

static int cd_resume(struct device *dev)
{
	struct cd3272 *cd = dev_get_drvdata(dev);

	mutex_lock(&cd->lock);
	cd->suspended = false;
	mutex_unlock(&cd->lock);
	schedule_delayed_work(&cd->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cd_pm, cd_suspend, cd_resume);

static const struct of_device_id cd_of_match[] = {
	{ .compatible = "apple,cd3272-mikey" },
	{ }
};
MODULE_DEVICE_TABLE(of, cd_of_match);

static const struct i2c_device_id cd_id[] = {
	{ "cd3272-mikey" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cd_id);

static struct i2c_driver cd_driver = {
	.driver = {
		.name = "apple-cd3272",
		.of_match_table = cd_of_match,
		.pm = pm_sleep_ptr(&cd_pm),
	},
	.probe = cd_probe,
	.id_table = cd_id,
};
module_i2c_driver(cd_driver);

MODULE_DESCRIPTION("Apple CD3272 Mikey headset remote driver");
MODULE_LICENSE("GPL");
