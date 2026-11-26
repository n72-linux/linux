// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Zephyr2 multi-touch controller (iPod touch 2nd generation, N72)
 *
 * Copyright (c) 2011 Richard Ian Taylor. Portions (c) 2008 Yidou Wang.
 *
 * The controller boots from a pre-constructed firmware stream (Z2F53,1), then
 * takes a per-device calibration blob. Both are loaded as firmware files.
 * Touch frames are read over SPI when the interrupt line is high.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define Z2_FW_NAME		"apple/z2-n72.bin"
#define Z2_CAL_NAME		"apple/z2-n72-cal.bin"

#define MT_INFO_FAMILYID	0xd1
#define MT_INFO_SENSORINFO	0xd3
#define MT_INFO_SENSORREGIONDESC 0xd0
#define MT_INFO_SENSORREGIONPARAM 0xa1
#define MT_INFO_SENSORDIM	0xd9

#define Z2_MAX_ORIENTATION	16384
#define Z2_MAX_CONTACTS		10	/* slots (the sensor reports fewer) */
#define Z2_BUF_SIZE		0x400
#define Z2_ACK_OK		0x4bc1
#define Z2_REG_ACK_OK		0x4ad1
#define Z2_CAL_ADDR		0x400200

struct z2_speed {
	u32 hz;
	u32 tx_delay_us;
	u32 rx_delay_us;
};

/* Bootloader and setup timings. */
static const struct z2_speed z2_normal = { 83000, 5000, 10000 };
static const struct z2_speed z2_fast = { 4500000, 0, 5000 };

/*
 * Touch frame reads. The controller scans at ~60 Hz; 83 kHz with 10 ms delays
 * takes 23 ms per frame (42 frames/s), whereas 2 MHz reads a frame in ~0.3 ms
 * without saturating the SPI link.
 */
#define Z2_READ_HZ	2000000

struct z2_frame_header {
	u8 type;
	u8 frame_num;
	u8 header_len;
	u8 unk_3;
	u32 timestamp;
	u8 unk_8[4];
	u16 unk_c;
	u16 is_image;
	u8 num_fingers;
	u8 finger_data_len;
	u16 unk_12[3];
} __packed;

struct z2_finger {
	u8 id;
	u8 event;
	u8 unk_2;
	u8 unk_3;
	s16 x;
	s16 y;
	s16 rel_x;
	s16 rel_y;
	u16 size_major;
	u16 size_minor;
	u16 orientation;
	u16 force_major;
	u16 force_minor;
	u16 unk_16[3];
} __packed;

struct zephyr2 {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset;
	const struct firmware *fw;
	const struct firmware *cal;
	u8 *tx;
	u8 *rx;
	u8 ptx[Z2_BUF_SIZE];
	u8 prx[Z2_BUF_SIZE];
	int max_packet;
	int sensor_width;
	int sensor_height;
	int bcd_version;
	bool flip_nop;
	u8 cur_nop;
	bool last_frame_ok;
	u32 min_pressure;

	/*
	 * Held in reset (system sleep, or the input device closed or
	 * inhibited: the screen is off), rebooted from a worker.
	 */
	struct mutex state_lock;
	struct work_struct resume_work;
	bool asleep;
	bool closed;

	/* frame read statistics (debugfs zephyr2/stats) */
	struct dentry *debugfs;
	unsigned long frames, empty, errors;
	u64 read_ns_total;
	u32 read_us_last, read_us_max;
};

/* Byte orders used by the bootloader protocol. */
static void z2_put_u16(u8 *b, u16 v) { b[0] = v >> 8; b[1] = v; }
static u16 z2_get_u16(const u8 *b) { return (b[0] << 8) | b[1]; }
static void z2_put_u16r(u8 *b, u16 v) { b[1] = v >> 8; b[0] = v; }

static void z2_put_u32(u8 *b, u32 v)
{
	b[2] = v >> 24; b[3] = v >> 16; b[0] = v >> 8; b[1] = v;
}

static u32 z2_get_u32(const u8 *b)
{
	return (b[2] << 24) | (b[3] << 16) | (b[0] << 8) | b[1];
}

static u32 z2_sum(const u8 *b, int len)
{
	u32 s = 0;

	while (len--)
		s += *b++;
	return s;
}

/* One SPI transfer of len bytes; tx is copied, rx may be NULL (TX-only for DMA). */
static int z2_xfer(struct zephyr2 *z2, const struct z2_speed *sp,
		   const u8 *tx, u8 *rx, int len, bool is_rx)
{
	struct spi_transfer t = {
		.tx_buf = z2->tx,
		.rx_buf = rx ? z2->rx : NULL,
		.len = len,
		.speed_hz = sp->hz,
		.delay = {
			.value = is_rx ? sp->rx_delay_us : sp->tx_delay_us,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};
	int ret;

	if (len > Z2_BUF_SIZE + 0x10)
		return -EINVAL;
	memcpy(z2->tx, tx, len);
	ret = spi_sync_transfer(z2->spi, &t, 1);
	if (ret)
		dev_err(&z2->spi->dev, "SPI transfer failed: %d\n", ret);
	else if (rx)
		memcpy(rx, z2->rx, len);
	return ret;
}

static u16 z2_short_ack(struct zephyr2 *z2)
{
	u8 tx[2], rx[2] = { 0 };

	z2_put_u16(tx, 0x1aa1);
	if (z2_xfer(z2, &z2_normal, tx, rx, 2, true))
		return 0;
	return z2_get_u16(rx);
}

static u32 z2_long_ack(struct zephyr2 *z2)
{
	u8 tx[8], rx[8] = { 0 };

	z2_put_u16(tx, 0x1aa1);
	z2_put_u16(tx + 2, 0x18e1);
	z2_put_u16(tx + 4, 0x18e1);
	z2_put_u16(tx + 6, 0x18e1);
	if (z2_xfer(z2, &z2_normal, tx, rx, 8, true))
		return 0;
	return z2_get_u32(rx + 2);
}

static u32 z2_read_register(struct zephyr2 *z2, u32 addr)
{
	u8 tx[8];

	z2_put_u16(tx, 0x1c73);
	z2_put_u32(tx + 2, addr);
	z2_put_u16(tx + 6, z2_sum(tx + 2, 4));
	z2_xfer(z2, &z2_normal, tx, NULL, 8, true);
	return z2_long_ack(z2);
}

static bool z2_write_register(struct zephyr2 *z2, u32 addr, u32 value, u32 mask)
{
	u8 tx[16];

	z2_put_u16(tx, 0x1e33);
	z2_put_u32(tx + 2, addr);
	z2_put_u32(tx + 6, mask);
	z2_put_u32(tx + 10, value);
	z2_put_u16(tx + 14, z2_sum(tx + 2, 12));
	z2_xfer(z2, &z2_normal, tx, NULL, 16, true);
	return z2_short_ack(z2) == Z2_REG_ACK_OK;
}

static bool z2_load_firmware(struct zephyr2 *z2)
{
	const struct firmware *fw = z2->fw;
	int try;
	u16 ack;

	for (try = 0; try < 5; try++) {
		struct spi_transfer t = {
			.tx_buf = fw->data,
			.len = fw->size,
			.speed_hz = z2_fast.hz,
			.delay = {
				.value = z2_fast.tx_delay_us,
				.unit = SPI_DELAY_UNIT_USECS,
			},
		};
		int ret = spi_sync_transfer(z2->spi, &t, 1);

		if (ret) {
			dev_err(&z2->spi->dev, "SPI firmware transfer failed: %d\n", ret);
			continue;
		}
		udelay(300);
		ack = z2_short_ack(z2);
		if (ack == Z2_ACK_OK)
			return true;
		dev_warn(&z2->spi->dev, "firmware upload ack 0x%04x\n", ack);
	}
	return false;
}

/* 18 E1, then a 0x3001 bootloader packet with "middle-endian" data. */
static int z2_make_packet(u8 *out, u32 addr, const u8 *data, int len)
{
	int i;

	out[0] = 0x18;
	out[1] = 0xe1;
	out[2] = 0x30;
	out[3] = 0x01;
	z2_put_u16(out + 4, len >> 2);
	z2_put_u32(out + 6, addr);
	z2_put_u16(out + 10, z2_sum(out + 4, 6));
	for (i = 0; i < len; i += 4) {
		out[12 + i + 0] = data[i + 1];
		out[12 + i + 1] = data[i + 0];
		out[12 + i + 2] = data[i + 3];
		out[12 + i + 3] = data[i + 2];
	}
	z2_put_u32(out + 12 + len, z2_sum(out + 12, len));
	return len + 0x10;
}

static bool z2_load_cal(struct zephyr2 *z2)
{
	const struct firmware *cal = z2->cal;
	size_t off = 0;
	u32 addr = Z2_CAL_ADDR;

	while (off < cal->size) {
		size_t raw = min_t(size_t, cal->size - off, 0x3f0);
		int n = (raw + 3) & ~3;
		int len, try;

		memset(z2->prx, 0, n);
		memcpy(z2->prx, cal->data + off, raw);
		len = z2_make_packet(z2->ptx, addr, z2->prx, n);
		for (try = 0; try < 5; try++) {
			if (z2_xfer(z2, &z2_fast, z2->ptx, NULL, len, false))
				continue;
			udelay(300);
			if (z2_short_ack(z2) == Z2_ACK_OK)
				break;
		}
		if (try == 5)
			return false;
		addr += n;
		off += raw;
	}
	return true;
}

static bool z2_calibrate(struct zephyr2 *z2)
{
	u32 version = z2_read_register(z2, 0x10008ffc);
	u8 tx[2] = { 0x1f, 0x01 };
	u32 ack;

	dev_info(&z2->spi->dev, "Zephyr2 version 0x%08x\n", version);
	if (version != 0x5a020028 && version != 0x5a02002a) {
		if (!z2_write_register(z2, 0x10001c04, 0x16e4, 0x1fff) ||
		    !z2_write_register(z2, 0x10001c08, 0x840000, 0xff0000) ||
		    !z2_write_register(z2, 0x1000300c, 0x05, 0x85) ||
		    !z2_write_register(z2, 0x1000304c, 0x20, 0xffffffff)) {
			dev_err(&z2->spi->dev, "register initialisation failed\n");
			return false;
		}
	}

	z2_xfer(z2, &z2_normal, tx, NULL, 2, true);
	msleep(65);
	ack = z2_long_ack(z2);
	dev_dbg(&z2->spi->dev, "calibration done: 0x%x\n", ack);
	return true;
}

static void z2_send_execute(struct zephyr2 *z2)
{
	u8 tx[12] = { 0x1d, 0x53, 0x18, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00 };

	z2_put_u16(tx + 10, z2_sum(tx + 2, 8));
	z2_xfer(z2, &z2_normal, tx, NULL, 12, true);
}

/* 16-byte command frame with the sum of bytes 0-13 in bytes 14-15. */
static void z2_cmd16(u8 *tx, u8 b0, u8 b1, u8 b2, u16 b34)
{
	u32 s;

	memset(tx, 0, 16);
	tx[0] = b0;
	tx[1] = b1;
	tx[2] = b2;
	tx[3] = b34;
	tx[4] = b34 >> 8;
	s = z2_sum(tx, 14);
	tx[14] = s;
	tx[15] = s >> 8;
}

static bool z2_rx16_ok(const u8 *rx, u8 expect)
{
	u32 s = z2_sum(rx, 14);

	return rx[0] == expect && (rx[14] | (rx[15] << 8)) == (s & 0xffff);
}

static bool z2_interface_version(struct zephyr2 *z2)
{
	u8 tx[16], rx[16];
	int try;

	z2_cmd16(tx, 0xe2, 0, 0, 0);
	/*
	 * The reply comes back in the next exchange: the first one returns
	 * the answer to the previous command (send once and discard). Then
	 * poll, with room for an SPI timeout or two.
	 */
	z2_xfer(z2, &z2_normal, tx, rx, 16, true);
	for (try = 0; try < 10; try++) {
		z2_xfer(z2, &z2_normal, tx, rx, 16, true);
		if (z2_rx16_ok(rx, 0xe2)) {
			z2->max_packet = (rx[4] << 8) | rx[3];
			dev_dbg(&z2->spi->dev, "interface version %d, max packet %d\n",
				rx[2], z2->max_packet);
			return true;
		}
		msleep(5);
	}
	z2->max_packet = 1000;
	return false;
}

static bool z2_get_report(struct zephyr2 *z2, int id, u8 *buf, int buf_size,
			  int *out_len)
{
	u8 *big_tx = z2->ptx, *big_rx = z2->prx;
	u8 tx[16], rx[16] = { 0 };
	int try, len = 0, copy;
	u32 s;

	for (try = 0; try < 4; try++) {
		z2_cmd16(tx, 0xe3, id, 0, 0);
		if (z2_xfer(z2, &z2_normal, tx, rx, 16, true))
			continue;
		if (z2_rx16_ok(rx, 0xe3) && !rx[2]) {
			len = (rx[4] << 8) | rx[3];
			break;
		}
	}
	if (try == 4 || len <= 0 || len > Z2_BUF_SIZE - 8)
		return false;

	memset(big_tx, 0, Z2_BUF_SIZE);
	copy = min(len, buf_size);

	for (try = 0; try < 4; try++) {
		if (len < 12) {
			z2_cmd16(big_tx, 0xe6, id, 0, len);
			if (z2_xfer(z2, &z2_normal, big_tx, NULL, 16, true))
				continue;
			udelay(25);
			/* Same frame with byte 2 = 1; preserve original checksum. */
			big_tx[2] = 1;
			if (z2_xfer(z2, &z2_normal, big_tx, big_rx, 16, true))
				continue;
			s = z2_sum(big_rx, 14);
			if ((big_rx[14] | (big_rx[15] << 8)) == (s & 0xffff)) {
				memcpy(buf, big_rx + 3, copy);
				*out_len = copy;
				return true;
			}
		} else {
			z2_cmd16(big_tx, 0xe7, id, 0, len);
			s = z2_sum(big_tx, 5);
			if (z2_xfer(z2, &z2_normal, big_tx, NULL, 16, true))
				continue;
			big_tx[2] = 1;
			big_tx[14] = 0;
			big_tx[15] = 0;
			big_tx[3 + len] = s;
			big_tx[3 + len + 1] = s >> 8;
			if (z2_xfer(z2, &z2_normal, big_tx, big_rx, len + 5, true))
				continue;
			s = z2_sum(big_rx, len + 3);
			if ((big_rx[3 + len] | (big_rx[3 + len + 1] << 8)) ==
			    (s & 0xffff)) {
				memcpy(buf, big_rx + 3, copy);
				*out_len = copy;
				return true;
			}
		}
	}
	return false;
}

/*
 * Multi-touch protocol B: a slot per finger id, released when the finger is
 * no longer in a frame (INPUT_MT_DROP_UNUSED); the single-touch axes and
 * BTN_TOUCH are the input core's emulation. (Protocol A, as before, sent a
 * finger's tracking id in every frame: SDL, without slots, took each frame
 * of a held finger as a new touch, so a drag reached applications as a row
 * of taps.)
 */
static void z2_report_frame(struct zephyr2 *z2, const u8 *data, int len)
{
	const struct z2_frame_header *hdr = (const void *)data;
	const struct z2_finger *f;
	int i;

	if (len < sizeof(*hdr) || hdr->header_len < 12)
		return;
	if (hdr->type != 0x44 && hdr->type != 0x43)
		dev_dbg(&z2->spi->dev, "unknown frame type 0x%x\n", hdr->type);

	f = (const void *)(data + hdr->header_len);
	for (i = 0; i < hdr->num_fingers; i++) {
		int fmaj, fmin, slot;

		if ((const u8 *)f + sizeof(*f) > data + len)
			break;
		fmaj = max(0, (int)f->force_major - (int)z2->min_pressure);
		fmin = max(0, (int)f->force_minor - (int)z2->min_pressure);
		slot = input_mt_get_slot_by_key(z2->input, f->id);
		if (fmaj && fmin && slot >= 0) {
			input_mt_slot(z2->input, slot);
			input_mt_report_slot_state(z2->input, MT_TOOL_FINGER, true);
			input_report_abs(z2->input, ABS_MT_TOUCH_MAJOR, fmaj);
			input_report_abs(z2->input, ABS_MT_TOUCH_MINOR, fmin);
			input_report_abs(z2->input, ABS_MT_WIDTH_MAJOR, f->size_major);
			input_report_abs(z2->input, ABS_MT_WIDTH_MINOR, f->size_minor);
			input_report_abs(z2->input, ABS_MT_ORIENTATION,
					 Z2_MAX_ORIENTATION - f->orientation);
			input_report_abs(z2->input, ABS_MT_POSITION_X, f->x);
			input_report_abs(z2->input, ABS_MT_POSITION_Y,
					 z2->sensor_height - f->y);
		}
		f = (const void *)((const u8 *)f + hdr->finger_data_len);
	}
	input_mt_sync_frame(z2->input);	/* (unused slots released; ABS_X/Y, BTN_TOUCH) */
	input_sync(z2->input);
}

static int z2_query_frame_len(struct zephyr2 *z2, const struct z2_speed *rd,
			      int *out_len)
{
	u8 tx[16], rx[16] = { 0 };
	int len, ret;

	/* Frame length query: checksum stored little-endian. */
	memset(tx, 0, sizeof(tx));
	tx[0] = z2->flip_nop ? 0xeb : 0xea;
	tx[1] = z2->cur_nop;
	z2_put_u16r(tx + 14, z2_sum(tx, 14));
	ret = z2_xfer(z2, rd, tx, rx, 16, true);
	if (ret)
		return ret;
	if ((rx[0] & 0xf0) != 0xe0 ||
	    (rx[14] | (rx[15] << 8)) != (z2_sum(rx, 14) & 0xffff)) {
		dev_warn_ratelimited(&z2->spi->dev, "bad frame length reply %16ph\n", rx);
		return -EIO;
	}
	len = rx[1] | (rx[2] << 8);
	if (len > z2->max_packet || len + 5 > Z2_BUF_SIZE) {
		dev_warn_ratelimited(&z2->spi->dev,
				     "oversized frame length %d (max %d)\n",
				     len, z2->max_packet);
		return -EIO;
	}
	*out_len = len;
	return 0;
}

static void z2_read_frame(struct zephyr2 *z2)
{
	const struct z2_speed rd = { Z2_READ_HZ, 0, 0 };
	u8 *ptx = z2->ptx, *prx = z2->prx;
	ktime_t t0 = ktime_get();
	int len, plen;
	u32 s, us;

	/*
	 * After a bad frame, issue a preliminary length query to resync the
	 * controller's SPI pipeline before reading the next frame.
	 */
	if (z2->flip_nop && !z2->last_frame_ok) {
		if (z2_query_frame_len(z2, &rd, &len)) {
			z2->last_frame_ok = false;
			z2->errors++;
			return;
		}
	}

	if (z2_query_frame_len(z2, &rd, &len)) {
		z2->last_frame_ok = false;
		z2->errors++;
		return;
	}
	if (!len) {
		z2->empty++;
		goto ok;
	}

	len += 5;
	memset(ptx, 0, len);
	ptx[0] = z2->flip_nop ? 0xeb : 0xea;
	ptx[1] = z2->cur_nop;
	ptx[2] = 1;
	s = z2_sum(ptx, 14);
	ptx[len - 2] = s;
	ptx[len - 1] = s >> 8;
	if (z2_xfer(z2, &rd, ptx, prx, len, true)) {
		z2->last_frame_ok = false;
		z2->errors++;
		return;
	}

	if (prx[0] != 0xea && !(z2->flip_nop && prx[0] == 0xeb)) {
		dev_warn_ratelimited(&z2->spi->dev, "bad frame header %16ph\n", prx);
		z2->last_frame_ok = false;
		z2->errors++;
		return;
	}
	if (z2_sum(prx, 5) & 0xff) {
		dev_warn_ratelimited(&z2->spi->dev, "bad frame head sum %16ph\n", prx);
		z2->last_frame_ok = false;
		z2->errors++;
		return;
	}
	plen = prx[2] | (prx[3] << 8);
	if (plen > 2) {
		if (5 + plen - 2 > len - 2) {
			dev_warn_ratelimited(&z2->spi->dev,
					     "bad frame payload len %d (buf %d)\n",
					     plen, len);
			z2->last_frame_ok = false;
			z2->errors++;
			return;
		}
		s = z2_sum(prx + 5, plen - 2);
		if ((prx[len - 2] | (prx[len - 1] << 8)) == (s & 0xffff)) {
			z2_report_frame(z2, prx + 5, plen - 2);
			z2->frames++;
		} else {
			dev_warn_ratelimited(&z2->spi->dev, "bad frame sum, len %d plen %d\n",
					     len, plen);
			z2->last_frame_ok = false;
			z2->errors++;
			return;
		}
	}
	us = ktime_us_delta(ktime_get(), t0);
	z2->read_us_last = us;
	z2->read_us_max = max(z2->read_us_max, us);
	z2->read_ns_total += (u64)us * 1000;
ok:
	z2->last_frame_ok = true;
	if (z2->flip_nop)
		z2->cur_nop = z2->cur_nop == 1 ? 2 : 1;
}

static irqreturn_t z2_irq_thread(int irq, void *data)
{
	struct zephyr2 *z2 = data;
	unsigned long errors = z2->errors;

	/*
	 * IRQ threads are SCHED_FIFO. SPI IRQ PIO avoids long CPU spins during
	 * normal reads, but after an SPI desync the controller asks again and
	 * again with bad frames: at real-time priority that kept user space
	 * (the UI) off the CPU for 100-300 ms. At normal priority the
	 * scheduler shares the CPU.
	 */
	if (unlikely(current->policy != SCHED_NORMAL))
		sched_set_normal(current, -5);
	z2_read_frame(z2);
	/* after a bad frame, give the controller a moment to settle */
	if (z2->errors != errors)
		usleep_range(2000, 3000);
	return IRQ_HANDLED;
}

static int z2_boot(struct zephyr2 *z2)
{
	struct device *dev = &z2->spi->dev;
	u8 buf[32];
	int i, len;

	for (i = 0; i < 4; i++) {
		static const u8 prime_tx[16] = {
			0x1a, 0xa1, 0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1,
			0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1, 0x18, 0xe1,
		};

		gpiod_set_value_cansleep(z2->reset, 1);
		msleep(200);
		/* Ensure SCLK is idle-high (mode 3) before reset deasserts. */
		z2_xfer(z2, &z2_normal, prime_tx, NULL, sizeof(prime_tx), false);
		gpiod_set_value_cansleep(z2->reset, 0);
		msleep(15);
		/* Consume the post-reset HBPP ready banner (0x1f01). */
		z2_xfer(z2, &z2_normal, prime_tx, NULL, sizeof(prime_tx), false);
		if (z2_load_firmware(z2))
			break;
	}
	if (i == 4) {
		dev_err(dev, "firmware upload failed\n");
		return -EIO;
	}
	dev_dbg(dev, "loaded %zu byte firmware\n", z2->fw->size);

	if (!z2_load_cal(z2)) {
		dev_err(dev, "calibration upload failed\n");
		return -EIO;
	}
	if (!z2_calibrate(z2))
		return -EIO;
	z2_send_execute(z2);
	msleep(1);

	if (!z2_interface_version(z2)) {
		dev_err(dev, "no interface version\n");
		return -EIO;
	}

	if (z2_get_report(z2, MT_INFO_FAMILYID, buf, sizeof(buf), &len) && len >= 1)
		dev_dbg(dev, "family 0x%x\n", buf[0]);
	if (z2_get_report(z2, MT_INFO_SENSORINFO, buf, sizeof(buf), &len) && len >= 5) {
		z2->bcd_version = (buf[3] << 8) | buf[4];
		dev_dbg(dev, "sensor %d rows x %d columns, bcd 0x%x\n",
			buf[1], buf[2], z2->bcd_version);
	}
	if (z2_get_report(z2, MT_INFO_SENSORDIM, buf, sizeof(buf), &len) && len >= 8) {
		z2->sensor_width = (9000 - (int)get_unaligned_le32(buf)) * 84 / 73;
		z2->sensor_height = (13850 - (int)get_unaligned_le32(buf + 4)) * 84 / 73;
	}
	if (z2->sensor_width <= 0 || z2->sensor_height <= 0) {
		/* iOS: surface 5000 x 7500 */
		z2->sensor_width = 5000;
		z2->sensor_height = 7500;
	}
	dev_dbg(dev, "surface %d x %d\n", z2->sensor_width, z2->sensor_height);

	z2->flip_nop = z2->bcd_version > 0x23;
	z2->cur_nop = 1;
	z2->last_frame_ok = true;
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int z2_stats_show(struct seq_file *m, void *unused)
{
	struct zephyr2 *z2 = m->private;
	unsigned long n = z2->frames ?: 1;

	seq_printf(m, "frames %lu, empty %lu, errors %lu\n", z2->frames, z2->empty, z2->errors);
	seq_printf(m, "read us: last %u, max %u, average %llu\n", z2->read_us_last,
		   z2->read_us_max, div_u64(z2->read_ns_total, n * 1000));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(z2_stats);

static void z2_debugfs_init(struct zephyr2 *z2)
{
	z2->debugfs = debugfs_create_dir("zephyr2", NULL);
	debugfs_create_file("stats", 0444, z2->debugfs, z2, &z2_stats_fops);
}
#else
static inline void z2_debugfs_init(struct zephyr2 *z2) { }
#endif

/* Lift every contact, so nothing stays pressed across sleep or a failed boot. */
static void z2_release_all(struct zephyr2 *z2)
{
	input_mt_sync_frame(z2->input);	/* no slot used: every slot released */
	input_sync(z2->input);
}

/*
 * Resume reloads the firmware and calibration (about 1.3 s, mostly fixed
 * delays), so it runs from a worker instead of holding up system resume.
 * An SPI transfer that times out at the wrong step (seen after suspend to
 * RAM, while other devices are still coming back) fails the whole boot, so
 * try again: every attempt starts from a reset.
 */
static void z2_resume_work(struct work_struct *work)
{
	struct zephyr2 *z2 = container_of(work, struct zephyr2, resume_work);
	int tries;

	for (tries = 1; z2_boot(z2); tries++) {
		if (tries == 3) {
			dev_err(&z2->spi->dev, "not restarted after sleep\n");
			gpiod_set_value_cansleep(z2->reset, 1);
			return;
		}
		dev_warn(&z2->spi->dev, "restart after sleep failed, trying again\n");
		msleep(100);
	}
	mutex_lock(&z2->state_lock);
	if (z2->closed) {
		gpiod_set_value_cansleep(z2->reset, 1);
		mutex_unlock(&z2->state_lock);
		return;
	}
	z2->asleep = false;
	z2_read_frame(z2);
	enable_irq(z2->spi->irq);
	mutex_unlock(&z2->state_lock);
}

static void z2_stop(struct zephyr2 *z2)
{
	cancel_work_sync(&z2->resume_work);
	mutex_lock(&z2->state_lock);
	if (z2->asleep) {
		mutex_unlock(&z2->state_lock);
		return;		/* (or the last restart failed: the interrupt is off) */
	}
	disable_irq(z2->spi->irq);
	z2_release_all(z2);
	/* reset held: the controller stops scanning */
	gpiod_set_value_cansleep(z2->reset, 1);
	z2->asleep = true;
	mutex_unlock(&z2->state_lock);
}

static int z2_suspend(struct device *dev)
{
	z2_stop(spi_get_drvdata(to_spi_device(dev)));
	return 0;
}

static int z2_resume(struct device *dev)
{
	struct zephyr2 *z2 = spi_get_drvdata(to_spi_device(dev));

	mutex_lock(&z2->state_lock);
	if (!z2->closed)	/* a closed one comes back when it is opened */
		schedule_work(&z2->resume_work);
	mutex_unlock(&z2->state_lock);
	return 0;
}

/*
 * Nobody reading touches (closed, or inhibited while the screen is off):
 * hold the controller in reset; it stops scanning the panel. Opened again,
 * it reboots like after a system sleep.
 */
static int z2_open(struct input_dev *input)
{
	struct zephyr2 *z2 = input_get_drvdata(input);

	mutex_lock(&z2->state_lock);
	z2->closed = false;
	if (z2->asleep)
		schedule_work(&z2->resume_work);
	mutex_unlock(&z2->state_lock);
	return 0;
}

static void z2_close(struct input_dev *input)
{
	struct zephyr2 *z2 = input_get_drvdata(input);

	mutex_lock(&z2->state_lock);
	z2->closed = true;
	mutex_unlock(&z2->state_lock);
	z2_stop(z2);
}

static void z2_cleanup(void *data)
{
	struct zephyr2 *z2 = data;

	cancel_work_sync(&z2->resume_work);
	debugfs_remove_recursive(z2->debugfs);
	release_firmware(z2->cal);
	release_firmware(z2->fw);
}

static DEFINE_SIMPLE_DEV_PM_OPS(z2_pm_ops, z2_suspend, z2_resume);

static int z2_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct zephyr2 *z2;
	int ret, i;

	z2 = devm_kzalloc(dev, sizeof(*z2), GFP_KERNEL);
	if (!z2)
		return -ENOMEM;
	z2->spi = spi;
	mutex_init(&z2->state_lock);
	INIT_WORK(&z2->resume_work, z2_resume_work);
	spi_set_drvdata(spi, z2);
	ret = devm_add_action_or_reset(dev, z2_cleanup, z2);
	if (ret)
		return ret;
	z2->tx = devm_kzalloc(dev, Z2_BUF_SIZE + 0x10, GFP_KERNEL);
	z2->rx = devm_kzalloc(dev, Z2_BUF_SIZE + 0x10, GFP_KERNEL);
	if (!z2->tx || !z2->rx)
		return -ENOMEM;
	of_property_read_u32(dev->of_node, "apple,min-pressure", &z2->min_pressure);

	z2->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(z2->reset))
		return dev_err_probe(dev, PTR_ERR(z2->reset), "no reset GPIO\n");

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "no vdd supply\n");

	spi->bits_per_word = 8;
	if (!spi->cs_setup.value) {
		spi->cs_setup.value = 10;
		spi->cs_setup.unit = SPI_DELAY_UNIT_USECS;
	}
	if (!spi->cs_hold.value) {
		spi->cs_hold.value = 10;
		spi->cs_hold.unit = SPI_DELAY_UNIT_USECS;
	}
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ret = request_firmware(&z2->fw, Z2_FW_NAME, dev);
	if (ret)
		return dev_err_probe(dev, ret, "no %s\n", Z2_FW_NAME);
	ret = request_firmware(&z2->cal, Z2_CAL_NAME, dev);
	if (ret)
		return dev_err_probe(dev, ret, "no %s\n", Z2_CAL_NAME);

	/* the same SPI timeouts hit the first boot too: retry like resume */
	for (i = 1; (ret = z2_boot(z2)) && ret != -EPROBE_DEFER && i < 3; i++) {
		dev_warn(dev, "start failed (%d), trying again\n", ret);
		msleep(100);
	}
	if (ret)
		return ret;

	z2->input = devm_input_allocate_device(dev);
	if (!z2->input)
		return -ENOMEM;
	z2->input->name = "Apple Zephyr2 multi-touch";
	z2->input->id.bustype = BUS_SPI;
	z2->input->open = z2_open;
	z2->input->close = z2_close;
	input_set_drvdata(z2->input, z2);
	input_set_abs_params(z2->input, ABS_MT_POSITION_X, 0, z2->sensor_width, 0, 0);
	input_set_abs_params(z2->input, ABS_MT_POSITION_Y, 0, z2->sensor_height, 0, 0);
	input_set_abs_params(z2->input, ABS_MT_TOUCH_MAJOR, 0,
			     max(z2->sensor_width, z2->sensor_height), 0, 0);
	input_set_abs_params(z2->input, ABS_MT_TOUCH_MINOR, 0,
			     max(z2->sensor_width, z2->sensor_height), 0, 0);
	input_set_abs_params(z2->input, ABS_MT_WIDTH_MAJOR, 0,
			     max(z2->sensor_width, z2->sensor_height), 0, 0);
	input_set_abs_params(z2->input, ABS_MT_WIDTH_MINOR, 0,
			     max(z2->sensor_width, z2->sensor_height), 0, 0);
	input_set_abs_params(z2->input, ABS_MT_ORIENTATION, -Z2_MAX_ORIENTATION,
			     Z2_MAX_ORIENTATION, 0, 0);
	/* (also: BTN_TOUCH, ABS_X/Y copied from the MT axes, INPUT_PROP_DIRECT) */
	ret = input_mt_init_slots(z2->input, Z2_MAX_CONTACTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;
	ret = input_register_device(z2->input);
	if (ret)
		return ret;

	z2_debugfs_init(z2);

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, z2_irq_thread,
					IRQF_ONESHOT, "zephyr2", z2);
	if (ret)
		return ret;

	/* Read one frame so the controller starts reporting. */
	disable_irq(spi->irq);
	z2_read_frame(z2);
	enable_irq(spi->irq);
	return 0;
}

static const struct of_device_id z2_of_match[] = {
	{ .compatible = "apple,n72-multitouch" },
	{ }
};
MODULE_DEVICE_TABLE(of, z2_of_match);

static struct spi_driver z2_driver = {
	.driver = {
		.name = "zephyr2",
		.of_match_table = z2_of_match,
		.pm = pm_sleep_ptr(&z2_pm_ops),
	},
	.probe = z2_probe,
};
module_spi_driver(z2_driver);

MODULE_DESCRIPTION("Apple Zephyr2 multi-touch (iPod touch 2G)");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(Z2_FW_NAME);
MODULE_FIRMWARE(Z2_CAL_NAME);
