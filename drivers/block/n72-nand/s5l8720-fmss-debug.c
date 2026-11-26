// SPDX-License-Identifier: GPL-2.0
/*
 * Sysfs debug/test interface and direct read-ID helpers for the Apple S5L8720
 * FMSS NAND controller.
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>

#include "s5l8720-fmss.h"

static int fmss_direct_readid(struct fmss *f, unsigned int ce, u8 id[8])
{
	u32 ctrl0 = FMCTRL0_IDTIMING | FMCTRL0_ON | FMCTRL0_CE(ce);
	u32 w0, w1, v;
	int ret;

	/* command prologue (minus CS-only registers) */
	fmss_wr(f, FMCTRL1, FMCTRL1_SEQCFG);
	fmss_wr(f, FMCSTAT, FMCSTAT_ALL);

	/* RESET this CE first */
	fmss_wr(f, FMCTRL0, ctrl0);
	fmss_wr(f, FMCMD, NAND_CMD_RESET);
	ret = fmss_fmc_wait(f, FMCSTAT_CMD);
	if (ret)
		goto out;
	usleep_range(1000, 2000);	/* tRST */

	fmss_wr(f, FMCMD, NAND_CMD_READID);
	ret = fmss_fmc_wait(f, FMCSTAT_CMD);
	if (ret)
		goto out;
	fmss_wr(f, FMANUM, 0);
	fmss_wr(f, FMADDR0, 0);
	fmss_wr(f, FMCTRL1, FMCTRL1_SENDADDR);
	ret = fmss_fmc_wait(f, FMCSTAT_ADDR);
	if (ret)
		goto out;
	fmss_wr(f, FMDNUM, 7);
	fmss_wr(f, FMUNIT, 0x10);
	fmss_wr(f, FMCTRL1, FMCTRL1_READFIFO);
	ret = fmss_fmc_wait(f, FMCSTAT_XFER);
	if (ret)
		goto out;
	fmss_wr(f, FMFIFOSTAT, 0x82);
	ret = readl_poll_timeout_atomic(f->base + FMFIFOSTAT, v, !(v & 2),
					FMSS_POLL_US, FMSS_POLL_TOTAL_US);
	if (ret)
		goto out;
	w0 = fmss_rd(f, FMRXDATA0);
	w1 = fmss_rd(f, FMRXDATA1);
	put_unaligned_le32(w0, id);
	put_unaligned_le32(w1, id + 4);
out:
	/* deselect: clear CE bits, keep timing and enable */
	fmss_wr(f, FMCTRL0, ctrl0 & ~FMCTRL0_CE_MASK);
	if (ret) {
		dev_warn(f->dev, "direct read ID CE%u timed out, FMCSTAT %#x\n",
			 ce, fmss_rd(f, FMCSTAT));
		fmss_hw_reset(f);
	}
	return ret;
}

/* reset program (#5) then read-ID program (#4) into idbuf[8][8] */
static int fmss_seq_readid(struct fmss *f)
{
	int ret;

	ret = fmss_run(f, &f->seq_reset, NULL);
	if (ret)
		return ret;
	memset(f->idbuf, 0, 0x40);
	wmb();
	fmss_wr(f, CSGENR(2), lower_32_bits(f->idbuf_dma));
	fmss_wr(f, CSGENR(3), FMSS_NCE);
	ret = fmss_run(f, &f->seq_readid, NULL);
	rmb();
	return ret;
}

/* "ce block": erase it (write_allow only) */
static ssize_t erase_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fmss *f = dev_get_drvdata(dev);
	u32 ce, block;
	int ret;

	if (sscanf(buf, "%u %u", &ce, &block) != 2)
		return -EINVAL;
	fmss_lock(f);
	ret = fmss_erase_block(f, ce, block, false);
	fmss_unlock(f);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(erase);

/*
 * "ce page seed": program a test pattern (byte i = (i * 7 + seed) & 0xff,
 * metadata all 0xff so no FTL page type is set). write_allow only.
 */
static ssize_t program_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct fmss *f = dev_get_drvdata(dev);
	u8 meta[FMSS_META_BYTES];
	u32 ce, page, seed, i;
	u8 *d;
	int ret;

	if (sscanf(buf, "%u %u %u", &ce, &page, &seed) != 3)
		return -EINVAL;
	memset(meta, 0xff, sizeof(meta));
	fmss_lock(f);
	d = f->data;
	for (i = 0; i < f->page_size; i++)
		d[i] = (i * 7 + seed) & 0xff;
	ret = fmss_program_page(f, ce, page, meta, false);
	fmss_unlock(f);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(program);

static ssize_t readid_direct_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct fmss *f = dev_get_drvdata(dev);
	ssize_t n = 0;
	unsigned int ce;

	fmss_lock(f);
	for (ce = 0; ce < FMSS_NCE; ce++) {
		u8 id[8] = {};
		int ret;

		if (!(f->ce_bitmap & BIT(ce)))
			continue;
		ret = fmss_direct_readid(f, ce, id);
		n += sysfs_emit_at(buf, n, "CE%u: %s %8phN id32=%#010x %s\n", ce,
				   ret ? "ERR" : "ok", id,
				   get_unaligned_le32(id),
				   get_unaligned_le32(id) == f->expected_id ?
				   "MATCH" : "MISMATCH");
		if (ret)
			break;
	}
	fmss_unlock(f);
	return n;
}
static DEVICE_ATTR_RO(readid_direct);

static ssize_t readid_seq_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct fmss *f = dev_get_drvdata(dev);
	ssize_t n = 0;
	unsigned int ce;
	int ret;

	fmss_lock(f);
	ret = fmss_seq_readid(f);
	if (ret) {
		n = sysfs_emit(buf, "ERR %d (R6 %#x)\n", ret, fmss_rd(f, CSSTATUS));
		goto out;
	}
	for (ce = 0; ce < FMSS_NCE; ce++) {
		const u8 *id = f->idbuf + 8 * ce;

		n += sysfs_emit_at(buf, n, "CE%u: %8phN id32=%#010x%s\n", ce, id,
				   get_unaligned_le32(id),
				   get_unaligned_le32(id) == f->expected_id ?
				   " MATCH" : "");
	}
out:
	fmss_unlock(f);
	return n;
}
static DEVICE_ATTR_RO(readid_seq);

static ssize_t page_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct fmss *f = dev_get_drvdata(dev);
	u32 ce, page;
	int ret;

	if (sscanf(buf, "%u %u", &ce, &page) != 2)
		return -EINVAL;
	fmss_lock(f);
	ret = fmss_read_page(f, ce, page);
	fmss_unlock(f);
	if (ret && ret != -ENODATA && ret != -EBADMSG)
		return ret;
	return count;
}

static ssize_t page_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct fmss *f = dev_get_drvdata(dev);
	u32 st;
	const char *what;
	ssize_t n;

	fmss_lock(f);
	st = f->last_status;
	switch (f->last_ret) {
	case 0:
		what = "ok";
		break;
	case -ENODATA:
		what = "blank";
		break;
	case -EBADMSG:
		what = "uncorrectable";
		break;
	case -EIO:
		what = "bad-media";
		break;
	case -ENOENT:
		what = "no-firmware";
		break;
	default:
		what = "error";
		break;
	}
	n = sysfs_emit(buf, "ce %u page %u ret %d %s status %#010x maxcorr %u meta %12phN\n",
		       f->last_ce, f->last_page, f->last_ret, what, st,
		       (st & 0xffff) ? fls(st & 0xffff) : 0, f->last_meta);
	fmss_unlock(f);
	return n;
}
static DEVICE_ATTR_RW(page);

static ssize_t page_data_read(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	struct fmss *f = dev_get_drvdata(kobj_to_dev(kobj));
	ssize_t n;

	fmss_lock(f);
	n = memory_read_from_buffer(buf, count, &off, f->data, f->page_size);
	fmss_unlock(f);
	return n;
}
static BIN_ATTR_RO(page_data, 0);

static ssize_t regs_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct fmss *f = dev_get_drvdata(dev);
	ssize_t n = 0;
	unsigned int i;

	fmss_lock(f);
	n += sysfs_emit_at(buf, n, "FMCTRL0 %08x FMCTRL1 %08x FMCMD %08x FMIRQMASK %08x FMCSTAT %08x\n",
			   fmss_rd(f, 0), fmss_rd(f, 4), fmss_rd(f, 8),
			   fmss_rd(f, 0x44), fmss_rd(f, FMCSTAT));
	n += sysfs_emit_at(buf, n, "CSCTRL %08x CSBASEADDR %08x CSSTAT %08x CSIRQ %08x CSIRQMASK %08x\n",
			   fmss_rd(f, CSCTRL), fmss_rd(f, CSBASEADDR),
			   fmss_rd(f, CSSTAT), fmss_rd(f, CSIRQ),
			   fmss_rd(f, CSIRQMASK));
	n += sysfs_emit_at(buf, n, "CSBUFRST %08x CSBUFCOND %08x ECC_CORE %08x\n",
			   fmss_rd(f, CSBUFRST), fmss_rd(f, CSBUFCOND),
			   fmss_rd(f, ECC_CORE));
	for (i = 0; i < 9; i++)
		n += sysfs_emit_at(buf, n, "CSINTNR%u %08x%s", i,
				   fmss_rd(f, CSINTNR(i)), i == 8 ? "\n" : " ");
	for (i = 0; i < 32; i++)
		n += sysfs_emit_at(buf, n, "GENR%02X %08x%s", i,
				   fmss_rd(f, CSGENR(i)), (i & 3) == 3 ? "\n" : " ");
	fmss_unlock(f);
	return n;
}
static DEVICE_ATTR_RO(regs);

static struct attribute *fmss_attrs[] = {
	&dev_attr_readid_direct.attr,
	&dev_attr_readid_seq.attr,
	&dev_attr_page.attr,
	&dev_attr_regs.attr,
	&dev_attr_erase.attr,
	&dev_attr_program.attr,
	NULL,
};

static struct bin_attribute *fmss_bin_attrs[] = {
	&bin_attr_page_data,
	NULL,
};

const struct attribute_group fmss_debug_group = {
	.attrs = fmss_attrs,
	.bin_attrs = fmss_bin_attrs,
};
