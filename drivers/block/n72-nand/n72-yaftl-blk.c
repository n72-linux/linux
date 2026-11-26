// SPDX-License-Identifier: GPL-2.0-only
/*
 * Block devices for the iPod touch 2G's NAND, through Apple's VSVFL +
 * YAFTL (n72-yaftl-core.c).
 *
 * One read-only disk per GPT partition of the FTL's logical disk
 * ("nandsystem", "nanddata"), 512-byte sectors. Nothing can write to
 * those: iOS's file systems stay iOS's.
 *
 * One writable disk, "nandlinux": a file that iOS created on its data
 * volume, described to this driver as a list of extents (the "image" file
 * in sysfs, filled in by n72-imgmap from the HFS+ volume mounted read-only).
 * Writes reach only the logical pages under those extents. They collect
 * in a small buffer and go to the flash on a flush, when the buffer is
 * full, or a few seconds after the last write.
 */

#include <linux/blk-mq.h>
#include <linux/ctype.h>
#include <linux/math64.h>
#include <linux/unaligned.h>
#include <linux/blkdev.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#include "n72-yaftl-core.h"
#include "s5l8720-fmss.h"

#define N72_LBA_SIZE	8192		/* the GPT's logical block: one FTL page */
#define N72_LBA_SECT	(N72_LBA_SIZE / SECTOR_SIZE)
#define N72_MAX_PARTS	4
#define N72_IMG_BLOCK	4096		/* HFS+ allocation block: what extents are made of */
#define N72_MAX_EXTENTS	8192
#define N72_WB_PAGES	128		/* 1 MiB of written pages waiting */
#define N72_WB_DELAY	(3 * HZ)

struct n72_nand;

struct n72_ext {
	u64 img;			/* first sector in the image */
	u64 disk;			/* first sector on the FTL's logical disk */
	u64 len;			/* sectors */
};

struct n72_part {
	struct gendisk *disk;
	struct blk_mq_tag_set tags;
	u64 first_lba, nr_lbas;		/* in 8 KiB LBAs (partitions) */
	bool image;			/* the image: mapped through nd->ext[] */
	struct n72_nand *nd;
};

struct n72_found {
	char name[24];
	u64 first, last;
};

struct n72_wb {
	u32 lpn;
	u16 have;			/* bit per 512-byte sector written */
	u8 *data;
};

struct n72_nand {
	struct device *dev;
	struct fmss *f;
	struct nand_ops ops;
	struct vfl vfl;
	struct yaftl ftl;
	struct mutex lock;		/* the FTL, the caches and the image map */
	u8 *page;			/* one cached logical page */
	u32 page_lpn;			/* its lpn, or U32_MAX */
	struct n72_part part[N72_MAX_PARTS];
	struct n72_found found[N72_MAX_PARTS];
	int nfound, nparts;
	struct work_struct restore_work;
	bool have_ftl;

	/* the image */
	struct n72_part img;
	struct n72_ext *ext;
	u32 n_ext;
	u64 img_sectors;
	u64 img_base, img_limit;	/* the partition it lives in, in sectors */
	bool img_open, img_rw, img_dead;
	struct n72_wb wb[N72_WB_PAGES];
	u8 *wb_mem, *wb_tmp;
	u32 wb_n;
	struct delayed_work wb_work;
	struct notifier_block reboot_nb;
	u64 st_wr_sect, st_rd_sect, st_flush, st_rmw;
};

/*
 * How many of YAFTL's index pages may be out of date on the flash. iOS
 * leaves up to ten after a crash; see "writing" in n72-yaftl-core.c.
 */
#define N72_IDX_STALE_MAX 7

static void *n72_alloc(size_t n)
{
	return kvzalloc(n, GFP_KERNEL);
}

static void n72_free(void *p)
{
	kvfree(p);
}

static void n72_log(const char *fmt, ...)
{
	char msg[256];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);
	pr_info("%s", msg);
}

static int n72_write_page(void *ctx, uint32_t ce, uint32_t page, const void *data,
			  const uint8_t meta[NAND_META_BYTES])
{
	struct n72_nand *nd = ctx;

	return fmss_ftl_program(nd->f, ce, page, data, meta) ? -1 : 0;
}

static int n72_erase_block(void *ctx, uint32_t ce, uint32_t block)
{
	struct n72_nand *nd = ctx;

	return fmss_ftl_erase(nd->f, ce, block) ? -1 : 0;
}

static int n72_read_page(void *ctx, uint32_t ce, uint32_t page, void *data,
			 uint8_t meta[NAND_META_BYTES])
{
	struct n72_nand *nd = ctx;
	int ret = fmss_read_page_copy(nd->f, ce, page, data, meta);

	if (!ret)
		return NAND_OK;
	if (ret == -ENODATA)
		return NAND_BLANK;
	return NAND_EIO;
}

/* --- the write buffer (caller holds nd->lock) --------------------------- */

static struct n72_wb *n72_wb_find(struct n72_nand *nd, u32 lpn)
{
	u32 i;

	for (i = 0; i < nd->wb_n; i++)
		if (nd->wb[i].lpn == lpn)
			return &nd->wb[i];
	return NULL;
}

/* Fill in, from the flash, the sectors of a buffered page not written yet. */
static int n72_wb_complete(struct n72_nand *nd, struct n72_wb *e)
{
	u32 s;

	if (e->have == 0xffff)
		return 0;
	if (yaftl_read(&nd->ftl, e->lpn, nd->wb_tmp) < 0)
		return -EIO;
	for (s = 0; s < N72_LBA_SECT; s++)
		if (!(e->have & BIT(s)))
			memcpy(e->data + s * SECTOR_SIZE, nd->wb_tmp + s * SECTOR_SIZE,
			       SECTOR_SIZE);
	e->have = 0xffff;
	nd->st_rmw++;
	return 0;
}

static int n72_wb_cmp(const void *a, const void *b)
{
	const struct n72_wb *x = a, *y = b;

	return x->lpn < y->lpn ? -1 : x->lpn > y->lpn;
}

/* Everything buffered goes to the flash, in the order of the logical pages. */
static int n72_wb_flush(struct n72_nand *nd)
{
	u32 i;
	int ret = 0;

	if (!nd->wb_n)
		return nd->img_dead ? -EIO : 0;
	sort(nd->wb, nd->wb_n, sizeof(nd->wb[0]), n72_wb_cmp, NULL);
	for (i = 0; i < nd->wb_n && !ret; i++) {
		struct n72_wb *e = &nd->wb[i];

		ret = n72_wb_complete(nd, e);
		if (!ret && yaftl_write(&nd->ftl, e->lpn, e->data))
			ret = -EIO;
		if (nd->page_lpn == e->lpn)
			nd->page_lpn = U32_MAX;
	}
	if (!ret && yaftl_sync(&nd->ftl))
		ret = -EIO;
	nd->st_flush++;
	nd->wb_n = 0;
	if (ret) {
		/* what was buffered is lost: the disk must not pretend otherwise */
		nd->img_dead = true;
		dev_err(nd->dev, "nandlinux: writing failed (free blocks %u%s): no more writes\n",
			nd->ftl.free_blocks, nd->ftl.failed ? ", FTL stopped" : "");
	}
	return ret;
}

static int n72_wb_put(struct n72_nand *nd, u32 lpn, u32 off, const void *src, u32 n)
{
	struct n72_wb *e = n72_wb_find(nd, lpn);
	u32 s;

	if (nd->img_dead)
		return -EIO;
	if (!e) {
		if (nd->wb_n == N72_WB_PAGES && n72_wb_flush(nd))
			return -EIO;
		e = &nd->wb[nd->wb_n];
		e->lpn = lpn;
		e->have = 0;
		e->data = nd->wb_mem + (size_t)nd->wb_n * N72_LBA_SIZE;
		nd->wb_n++;
	}
	memcpy(e->data + off, src, n);
	for (s = off / SECTOR_SIZE; s < (off + n) / SECTOR_SIZE; s++)
		e->have |= BIT(s);
	if (nd->page_lpn == lpn)
		nd->page_lpn = U32_MAX;
	return 0;
}

static void n72_wb_work(struct work_struct *w)
{
	struct n72_nand *nd = container_of(to_delayed_work(w), struct n72_nand, wb_work);

	mutex_lock(&nd->lock);
	n72_wb_flush(nd);
	mutex_unlock(&nd->lock);
}

static int n72_reboot(struct notifier_block *nb, unsigned long action, void *data)
{
	struct n72_nand *nd = container_of(nb, struct n72_nand, reboot_nb);

	cancel_delayed_work_sync(&nd->wb_work);
	mutex_lock(&nd->lock);
	n72_wb_flush(nd);
	nd->img_dead = true;		/* nothing after this */
	mutex_unlock(&nd->lock);
	return NOTIFY_DONE;
}

/* --- reading -------------------------------------------------------------- */

/* One logical page (8 KiB), newest contents; caller holds nd->lock. */
static const u8 *n72_get_lpn(struct n72_nand *nd, u32 lpn)
{
	struct n72_wb *e = n72_wb_find(nd, lpn);
	int ret;

	if (e)
		return n72_wb_complete(nd, e) ? NULL : e->data;
	if (nd->page_lpn == lpn)
		return nd->page;
	nd->page_lpn = U32_MAX;
	ret = yaftl_read(&nd->ftl, lpn, nd->page);
	if (ret < 0)
		return NULL;
	nd->page_lpn = lpn;	/* NAND_BLANK: yaftl_read zeroed it */
	return nd->page;
}

/*
 * Image sector -> sector on the FTL's logical disk, and how many sectors
 * follow in the same extent. ext[] is in image order.
 */
static int n72_img_map(struct n72_nand *nd, u64 sect, u64 *disk, u64 *run)
{
	u32 lo = 0, hi = nd->n_ext;

	while (lo < hi) {
		u32 mid = (lo + hi) / 2;

		if (nd->ext[mid].img + nd->ext[mid].len <= sect)
			lo = mid + 1;
		else
			hi = mid;
	}
	if (lo == nd->n_ext || nd->ext[lo].img > sect)
		return -EIO;
	*disk = nd->ext[lo].disk + (sect - nd->ext[lo].img);
	*run = nd->ext[lo].len - (sect - nd->ext[lo].img);
	return 0;
}

static blk_status_t n72_queue_rq(struct blk_mq_hw_ctx *hctx,
				 const struct blk_mq_queue_data *bd)
{
	struct request *rq = bd->rq;
	struct n72_part *p = rq->q->queuedata;
	struct n72_nand *nd = p->nd;
	u64 sect = blk_rq_pos(rq);
	blk_status_t st = BLK_STS_OK;
	struct req_iterator iter;
	struct bio_vec bv;
	bool write = false;

	blk_mq_start_request(rq);
	switch (req_op(rq)) {
	case REQ_OP_READ:
		break;
	case REQ_OP_WRITE:
		/* only the image, and only once it was made writable */
		if (!p->image || !nd->img_rw) {
			blk_mq_end_request(rq, BLK_STS_IOERR);
			return BLK_STS_OK;
		}
		write = true;
		break;
	case REQ_OP_FLUSH:
		mutex_lock(&nd->lock);
		if (p->image && n72_wb_flush(nd))
			st = BLK_STS_IOERR;
		mutex_unlock(&nd->lock);
		blk_mq_end_request(rq, st);
		return BLK_STS_OK;
	default:
		blk_mq_end_request(rq, BLK_STS_NOTSUPP);
		return BLK_STS_OK;
	}

	mutex_lock(&nd->lock);
	rq_for_each_segment(bv, rq, iter) {
		u32 done = 0;

		while (done < bv.bv_len) {
			u64 dsect, run;
			u32 lpn, off, n;
			void *buf;

			if (p->image) {
				if (n72_img_map(nd, sect, &dsect, &run)) {
					st = BLK_STS_IOERR;
					goto out;
				}
			} else {
				if (sect >= p->nr_lbas * N72_LBA_SECT) {
					st = BLK_STS_IOERR;
					goto out;
				}
				dsect = p->first_lba * N72_LBA_SECT + sect;
				run = p->nr_lbas * N72_LBA_SECT - sect;
			}
			lpn = div_u64(dsect, N72_LBA_SECT);
			off = (dsect - (u64)lpn * N72_LBA_SECT) * SECTOR_SIZE;
			n = min_t(u64, bv.bv_len - done, run * SECTOR_SIZE);
			n = min_t(u32, n, N72_LBA_SIZE - off);

			buf = bvec_kmap_local(&bv);
			if (write) {
				if (n72_wb_put(nd, lpn, off, buf + done, n))
					st = BLK_STS_IOERR;
			} else {
				const u8 *src = n72_get_lpn(nd, lpn);

				if (src)
					memcpy(buf + done, src + off, n);
				else
					st = BLK_STS_IOERR;
			}
			kunmap_local(buf);
			if (st)
				goto out;
			done += n;
			sect += n / SECTOR_SIZE;
		}
	}
	if (write) {
		nd->st_wr_sect += blk_rq_sectors(rq);
		if ((rq->cmd_flags & REQ_FUA) && n72_wb_flush(nd))
			st = BLK_STS_IOERR;
		else
			mod_delayed_work(system_wq, &nd->wb_work, N72_WB_DELAY);
	} else if (p->image) {
		nd->st_rd_sect += blk_rq_sectors(rq);
	}
out:
	mutex_unlock(&nd->lock);
	blk_mq_end_request(rq, st);
	return BLK_STS_OK;
}

static const struct blk_mq_ops n72_mq_ops = {
	.queue_rq = n72_queue_rq,
};

static const struct block_device_operations n72_bdops = {
	.owner = THIS_MODULE,
};

static int n72_add_disk(struct n72_nand *nd, struct n72_part *p, const char *name,
			u64 sectors, bool image)
{
	struct queue_limits lim = {
		.logical_block_size = image ? N72_IMG_BLOCK : SECTOR_SIZE,
		.physical_block_size = image ? N72_LBA_SIZE : PAGE_SIZE,
		.max_hw_sectors = 256,
		.features = image ? BLK_FEAT_WRITE_CACHE | BLK_FEAT_FUA : 0,
	};
	int ret;

	p->nd = nd;
	p->image = image;
	p->tags.ops = &n72_mq_ops;
	p->tags.nr_hw_queues = 1;
	p->tags.queue_depth = 4;
	p->tags.numa_node = NUMA_NO_NODE;
	p->tags.flags = BLK_MQ_F_BLOCKING;
	ret = blk_mq_alloc_tag_set(&p->tags);
	if (ret)
		return ret;

	p->disk = blk_mq_alloc_disk(&p->tags, &lim, p);
	if (IS_ERR(p->disk)) {
		ret = PTR_ERR(p->disk);
		blk_mq_free_tag_set(&p->tags);
		return ret;
	}
	p->disk->fops = &n72_bdops;
	p->disk->private_data = p;
	p->disk->flags |= GENHD_FL_NO_PART;
	snprintf(p->disk->disk_name, DISK_NAME_LEN, "nand%s", name);
	set_capacity(p->disk, sectors);
	set_disk_ro(p->disk, true);

	ret = add_disk(p->disk);
	if (ret) {
		put_disk(p->disk);
		blk_mq_free_tag_set(&p->tags);
		return ret;
	}
	return 0;
}

/* GPT at LBA 1 (= logical page 1), entries from LBA 2; 8 KiB LBAs. */
static int n72_scan_gpt(struct n72_nand *nd)
{
	u32 entries_lba, nentries, esize, i;
	const u8 *hdr;

	hdr = n72_get_lpn(nd, 1);
	if (!hdr)
		return -EIO;
	if (memcmp(hdr, "EFI PART", 8)) {
		dev_err(nd->dev, "no GPT on the FTL's logical disk\n");
		return -ENODEV;
	}
	entries_lba = get_unaligned_le64(hdr + 72);
	nentries = get_unaligned_le32(hdr + 80);
	esize = get_unaligned_le32(hdr + 84);
	if (esize < 128 || esize > N72_LBA_SIZE || nentries > 128)
		return -EINVAL;

	for (i = 0; i < nentries && nd->nfound < N72_MAX_PARTS; i++) {
		u64 byte = (u64)i * esize;
		u32 lpn = entries_lba + div_u64(byte, N72_LBA_SIZE);
		u32 off = byte - (u64)(lpn - entries_lba) * N72_LBA_SIZE;
		char name[24];
		u64 first, last;
		const u8 *e;
		int k;

		e = n72_get_lpn(nd, lpn);
		if (!e)
			return -EIO;
		e += off;
		if (!memchr_inv(e, 0, 16))	/* unused entry */
			continue;
		first = get_unaligned_le64(e + 32);
		last = get_unaligned_le64(e + 40);
		/* UTF-16LE name, ASCII part only, lower-cased */
		for (k = 0; k < (int)sizeof(name) - 1 && e[56 + 2 * k]; k++)
			name[k] = tolower(e[56 + 2 * k]);
		name[k] = 0;
		if (!k)
			snprintf(name, sizeof(name), "p%u", i + 1);
		if (last < first || last >= nd->ftl.total_pages)
			continue;
		strscpy(nd->found[nd->nfound].name, name);
		nd->found[nd->nfound].first = first;
		nd->found[nd->nfound].last = last;
		nd->nfound++;
	}
	return nd->nfound ? 0 : -ENODEV;
}

/* --- the image: sysfs ------------------------------------------------------- */

static struct n72_nand *n72_nd;

static int n72_ext_cmp(const void *a, const void *b)
{
	const struct n72_ext *x = a, *y = b;

	return x->disk < y->disk ? -1 : x->disk > y->disk;
}

/* The extents must lie in the partition and share nothing with each other. */
static int n72_img_check(struct n72_nand *nd)
{
	struct n72_ext *s;
	u32 i;
	int ret = 0;

	if (!nd->n_ext || nd->img_sectors < 2048)
		return -EINVAL;
	s = kvmalloc_array(nd->n_ext, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	memcpy(s, nd->ext, nd->n_ext * sizeof(*s));
	sort(s, nd->n_ext, sizeof(*s), n72_ext_cmp, NULL);
	for (i = 0; i < nd->n_ext && !ret; i++) {
		if (s[i].disk < nd->img_base || s[i].disk + s[i].len > nd->img_limit)
			ret = -ERANGE;
		else if (i && s[i - 1].disk + s[i - 1].len > s[i].disk)
			ret = -EEXIST;
	}
	kvfree(s);
	return ret;
}

/* Make the image writable: the map by scanning, the buffer, the limits. */
static int n72_img_rw(struct n72_nand *nd)
{
	int ret;

	if (nd->img_rw)
		return 0;
	if (!nd->img_open)
		return -ENODEV;
	if (!fmss_ftl_write_enabled()) {
		dev_err(nd->dev, "nandlinux: the ftl_write parameter is off\n");
		return -EPERM;
	}
	if (!nd->ftl.l2v) {
		/* opened from iOS's clean context: writing needs the scan */
		dev_dbg(nd->dev, "YAFTL: scanning for the write state\n");
		nd->page_lpn = U32_MAX;
		ret = yaftl_restore(&nd->ftl);
		if (ret)
			return -EIO;
	}
	if (!nd->wb_mem) {
		nd->wb_mem = vmalloc((size_t)N72_WB_PAGES * N72_LBA_SIZE);
		nd->wb_tmp = kmalloc(N72_LBA_SIZE, GFP_KERNEL);
		if (!nd->wb_mem || !nd->wb_tmp)
			return -ENOMEM;
	}
	nd->ftl.idx_dirty_max = N72_IDX_STALE_MAX;
	nd->ftl.idx_dirty_burst = N72_IDX_STALE_MAX;
	nd->ops.log = n72_log;
	if (yaftl_prepare_write(&nd->ftl))
		return -EIO;
	nd->reboot_nb.notifier_call = n72_reboot;
	register_reboot_notifier(&nd->reboot_nb);
	nd->img_rw = true;
	set_disk_ro(nd->img.disk, false);
	dev_dbg(nd->dev, "nandlinux: writable (%u free FTL blocks, %u stale index pages)\n",
		nd->ftl.free_blocks, nd->ftl.n_dirty);
	return 0;
}

/*
 * image: the file that is the "nandlinux" disk, one command per line:
 *   new PARTITION      start a list; extents are in this partition ("data")
 *   ext START COUNT    the file's next extent: 512-byte sectors in the
 *                      partition, whole 4 KiB blocks, in file order
 *   commit             check the list and create the disk, read-only
 *   rw                 allow writing (unless the fmss ftl_write parameter is 0)
 * Reading it shows the state.
 */
static ssize_t image_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct n72_nand *nd = n72_nd;
	char *copy, *cur, *line;
	int ret = 0;

	if (!nd)
		return -ENODEV;
	copy = kstrndup(buf, count, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	cur = copy;
	mutex_lock(&nd->lock);
	while (!ret && (line = strsep(&cur, "\n"))) {
		unsigned long long start, len;
		char name[24];
		int i;

		line = strim(line);
		if (!*line)
			continue;
		if (sscanf(line, "new %23s", name) == 1) {
			if (nd->img_open) {
				ret = -EBUSY;
				break;
			}
			ret = -ENOENT;
			for (i = 0; i < nd->nfound; i++) {
				if (strcmp(nd->found[i].name, name))
					continue;
				nd->img_base = nd->found[i].first * N72_LBA_SECT;
				nd->img_limit = (nd->found[i].last + 1) * N72_LBA_SECT;
				ret = 0;
			}
			if (!ret && !nd->ext) {
				nd->ext = kvmalloc_array(N72_MAX_EXTENTS, sizeof(*nd->ext),
							 GFP_KERNEL);
				if (!nd->ext)
					ret = -ENOMEM;
			}
			nd->n_ext = 0;
			nd->img_sectors = 0;
		} else if (sscanf(line, "ext %llu %llu", &start, &len) == 2) {
			u32 blk = N72_IMG_BLOCK / SECTOR_SIZE;

			if (nd->img_open || !nd->ext || !nd->img_limit)
				ret = -EBUSY;
			else if (nd->n_ext == N72_MAX_EXTENTS)
				ret = -E2BIG;
			else if (!len || start % blk || len % blk ||
				 start + len > nd->img_limit - nd->img_base)
				ret = -ERANGE;
			else {
				nd->ext[nd->n_ext].img = nd->img_sectors;
				nd->ext[nd->n_ext].disk = nd->img_base + start;
				nd->ext[nd->n_ext].len = len;
				nd->n_ext++;
				nd->img_sectors += len;
			}
		} else if (!strcmp(line, "commit")) {
			if (nd->img_open || !nd->ext) {
				ret = -EBUSY;
				break;
			}
			ret = n72_img_check(nd);
			if (ret)
				break;
			/*
			 * the lock is not needed by a disk nobody knows yet,
			 * and add_disk() reads through queue_rq
			 */
			nd->img_open = true;
			mutex_unlock(&nd->lock);
			ret = n72_add_disk(nd, &nd->img, "linux", nd->img_sectors, true);
			mutex_lock(&nd->lock);
			if (ret)
				nd->img_open = false;
			else
				dev_dbg(nd->dev, "nandlinux: %llu MiB in %u extents, read-only\n",
					nd->img_sectors >> 11, nd->n_ext);
		} else if (!strcmp(line, "rw")) {
			ret = n72_img_rw(nd);
		} else if (!strcmp(line, "flush")) {
			ret = n72_wb_flush(nd);
		} else {
			ret = -EINVAL;
		}
	}
	mutex_unlock(&nd->lock);
	kfree(copy);
	return ret ? ret : count;
}

static ssize_t image_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct n72_nand *nd = n72_nd;
	struct yaftl *y;
	ssize_t n;

	if (!nd)
		return -ENODEV;
	y = &nd->ftl;
	mutex_lock(&nd->lock);
	n = sysfs_emit(buf,
		       "state %s%s\nextents %u\nsectors %llu\nbuffered %u\n"
		       "read_sectors %llu\nwritten_sectors %llu\nflushes %llu\nmerged_pages %llu\n"
		       "ftl_free_blocks %u\nftl_user_blocks %u\nftl_index_blocks %u\n"
		       "ftl_stale_index %u\nftl_user_pages %u\nftl_gc_pages %u\nftl_gc_blocks %u\n"
		       "ftl_index_pages %u\nftl_erases %u\nftl_usn %#x\n",
		       !nd->img_open ? "none" : nd->img_rw ? "rw" : "ro",
		       nd->img_dead ? " failed" : "", nd->n_ext, nd->img_sectors, nd->wb_n,
		       nd->st_rd_sect, nd->st_wr_sect, nd->st_flush, nd->st_rmw,
		       y->free_blocks, y->user_blocks, y->index_blocks, y->n_dirty,
		       y->st_user, y->st_gc_pages, y->st_gc_blocks, y->st_index, y->st_erases,
		       y->max_usn);
	mutex_unlock(&nd->lock);
	return n;
}
static DEVICE_ATTR_RW(image);

#ifdef CONFIG_BLK_DEV_N72_NAND_DEBUG
/*
 * Test files, built with CONFIG_BLK_DEV_N72_NAND_DEBUG only (ftl_test
 * writes without the image's "rw").
 *
 * ftl_test: "lpn count seed" writes count (<= 4096) test pages at lpn..
 * through yaftl_write() and reads each back through the FTL map. Only
 * logical pages above what iOS exposes (pages_available..total_pages) are
 * accepted: no partition, file system or GPT lives there. Needs the fmss
 * ftl_write parameter. Reading it back gives the last result.
 */
static char ftl_test_result[200] = "none\n";

static u8 ftl_test_byte(u32 lpn, u32 seed, u32 j)
{
	return (j * 7 + seed + lpn * 13) & 0xff;
}

static ssize_t ftl_test_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct n72_nand *nd = n72_nd;
	u32 lpn, n, seed, i, j, bad = 0, blank = 0;
	u8 *data, *rd;
	int ret = 0;

	bool verify = buf[0] == 'v';	/* "v lpn count seed": only read and compare */

	if (!nd || sscanf(buf + verify, "%u %u %u", &lpn, &n, &seed) != 3)
		return -EINVAL;
	if (!n || n > 4096 || lpn < nd->ftl.pages_available || lpn + n > nd->ftl.total_pages)
		return -ERANGE;
	if (!verify && !fmss_ftl_write_enabled())
		return -EPERM;
	data = kmalloc(N72_LBA_SIZE, GFP_KERNEL);
	rd = kmalloc(N72_LBA_SIZE, GFP_KERNEL);
	if (!data || !rd) {
		kfree(data);
		kfree(rd);
		return -ENOMEM;
	}
	mutex_lock(&nd->lock);
	nd->page_lpn = U32_MAX;
	nd->ops.log = n72_log;
	if (!verify && !nd->ftl.l2v && yaftl_restore(&nd->ftl))
		ret = -EIO;
	if (!verify)
		nd->ftl.idx_dirty_max = nd->ftl.idx_dirty_burst = N72_IDX_STALE_MAX;
	for (i = 0; i < n && !ret && !verify; i++) {
		for (j = 0; j < N72_LBA_SIZE; j++)
			data[j] = ftl_test_byte(lpn + i, seed, j);
		ret = yaftl_write(&nd->ftl, lpn + i, data);
	}
	if (!ret && !verify)
		ret = yaftl_sync(&nd->ftl);
	if (!ret) {
		for (i = 0; i < n; i++) {
			int r = yaftl_read(&nd->ftl, lpn + i, rd);

			for (j = 0; j < N72_LBA_SIZE; j++)
				data[j] = ftl_test_byte(lpn + i, seed, j);
			if (r == NAND_BLANK)
				blank++;
			else if (r != NAND_OK || memcmp(rd, data, N72_LBA_SIZE))
				bad++;
		}
		scnprintf(ftl_test_result, sizeof(ftl_test_result),
			  "%slpn %u count %u: usn %#x, read back %u ok, %u bad, %u blank; free blocks %u, stale index %u\n",
			  verify ? "verify " : "", lpn, n, nd->ftl.max_usn, n - bad - blank, bad, blank,
			  nd->ftl.free_blocks, nd->ftl.n_dirty);
	} else {
		scnprintf(ftl_test_result, sizeof(ftl_test_result),
			  "lpn %u count %u: write failed %d after %u pages\n", lpn, n, ret, i);
		ret = -EIO;
	}
	mutex_unlock(&nd->lock);
	dev_info(nd->dev, "ftl_test: %s", ftl_test_result);
	kfree(data);
	kfree(rd);
	return ret ? ret : count;
}

static ssize_t ftl_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct n72_nand *nd = n72_nd;

	if (!nd)
		return -ENODEV;
	return sysfs_emit(buf, "available %u total %u free blocks %u max usn %#x %s; last: %s",
			  nd->ftl.pages_available, nd->ftl.total_pages, nd->ftl.free_blocks,
			  nd->ftl.max_usn, nd->ftl.clean ? "CLEAN context" : "rebuilt context",
			  ftl_test_result);
}
static DEVICE_ATTR_RW(ftl_test);

/*
 * ftl_check: what writing would start from, without writing anything: the
 * blocks left open, and how many of iOS's index pages disagree with the map
 * the scan built (iOS leaves at most ten; many more would mean the scan
 * reads the flash differently from iOS).
 */
static ssize_t ftl_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct n72_nand *nd = n72_nd;
	struct yaftl *y;
	ssize_t n = 0;
	u32 i, b, unclosed = 0, mapped = 0;

	if (!nd)
		return -ENODEV;
	y = &nd->ftl;
	mutex_lock(&nd->lock);
	if (!y->l2v) {
		n = sysfs_emit(buf, "no scan (clean context in use)\n");
		goto out;
	}
	nd->page_lpn = U32_MAX;
	for (b = 0; b < y->num_blocks; b++)
		if ((y->block_status[b] == YBS_USER || y->block_status[b] == YBS_INDEX) &&
		    !(y->blk_flags[b] & YBF_CLOSED))
			unclosed++;
	for (i = 0; i < y->total_pages; i++)
		if (y->l2v[i] != YAFTL_VPN_NONE)
			mapped++;
	n += sysfs_emit_at(buf, n, "blocks: %u user, %u index, %u free; %u without a TOC\n",
			   y->user_blocks, y->index_blocks, y->free_blocks, unclosed);
	for (i = 0; i < y->n_left; i++) {
		u32 blk = y->left[i].blk, used = 0, p;

		for (p = 0; p < y->stripe_pages - 1; p++)
			if (y->left[i].btoc[p] != YAFTL_VPN_NONE)
				used = p + 1;
		n += sysfs_emit_at(buf, n, "to close: %s block %u, usn %#x, %u pages, %u current\n",
				   y->block_status[blk] == YBS_INDEX ? "index" : "user", blk,
				   y->blk_usn[blk], used, y->valid[blk]);
	}
	if (y->iblk.blk != YAFTL_VPN_NONE)
		n += sysfs_emit_at(buf, n, "open index block %u, next page %u\n",
				   y->iblk.blk, y->iblk.used);
	n += sysfs_emit_at(buf, n, "mapped logical pages: %u of %u; max usn %#x; context %s\n",
			   mapped, y->total_pages, y->max_usn,
			   y->cxt_page != YAFTL_VPN_NONE ? "clean on the flash" : "already spoiled");
	if (y->prepared)
		n += sysfs_emit_at(buf, n, "writing: stale index pages %u\n", y->n_dirty);
	else
		n += sysfs_emit_at(buf, n, "index pages that differ from the map: %u of %u\n",
				   yaftl_index_compare(y, 0), y->toc_array_len);
	{
		u32 d[6];

		yaftl_index_diff(y, d);
		n += sysfs_emit_at(buf, n,
				   "entries: %u only in the map, %u only in the index, %u map newer, %u MAP OLDER, %u same block; %u ranges without index page\n",
				   d[0], d[1], d[2], d[3], d[4], d[5]);
	}
out:
	mutex_unlock(&nd->lock);
	return n;
}
static DEVICE_ATTR_RO(ftl_check);
#endif

static int n72_add_disks(struct n72_nand *nd)
{
	int ret;

	dev_info(nd->dev, "YAFTL: %u logical pages (%llu MiB), %s context usn %#x\n",
		 nd->ftl.total_pages,
		 ((u64)nd->ftl.total_pages * N72_LBA_SIZE) >> 20,
		 nd->ftl.clean ? "clean" : "rebuilt (stale)", nd->ftl.cxt_usn);
	if (!nd->ftl.clean)
		dev_info(nd->dev, "YAFTL: %u user, %u index, %u free blocks\n",
			 nd->ftl.user_blocks, nd->ftl.index_blocks, nd->ftl.free_blocks);
	mutex_lock(&nd->lock);
	ret = n72_scan_gpt(nd);
	mutex_unlock(&nd->lock);
	if (ret)
		return ret;
	n72_nd = nd;
#ifdef CONFIG_BLK_DEV_N72_NAND_DEBUG
	if (device_create_file(nd->dev, &dev_attr_ftl_test) ||
	    device_create_file(nd->dev, &dev_attr_ftl_check))
		dev_warn(nd->dev, "no ftl_test/ftl_check files\n");
#endif
	if (device_create_file(nd->dev, &dev_attr_image))
		dev_warn(nd->dev, "sysfs files missing\n");
	for (; nd->nparts < nd->nfound; nd->nparts++) {
		struct n72_found *fp = &nd->found[nd->nparts];
		struct n72_part *p = &nd->part[nd->nparts];

		p->first_lba = fp->first;
		p->nr_lbas = fp->last - fp->first + 1;
		ret = n72_add_disk(nd, p, fp->name, p->nr_lbas * N72_LBA_SECT, false);
		if (ret)
			return ret;
		dev_dbg(nd->dev, "%s: LBA %llu-%llu (%llu MiB), read-only\n",
			p->disk->disk_name, fp->first, fp->last,
			(p->nr_lbas * N72_LBA_SIZE) >> 20);
	}
	return 0;
}

static void n72_restore_work(struct work_struct *w)
{
	struct n72_nand *nd = container_of(w, struct n72_nand, restore_work);
	ktime_t t0 = ktime_get();
	int ret;

	mutex_lock(&nd->lock);
	ret = yaftl_open(&nd->ftl, &nd->vfl, 1);
	mutex_unlock(&nd->lock);
	if (ret) {
		dev_err(nd->dev, "YAFTL: map rebuild failed (%d)\n", ret);
		return;
	}
	dev_dbg(nd->dev, "YAFTL: map rebuilt by scanning in %lld ms\n",
		ktime_ms_delta(ktime_get(), t0));
	ret = n72_add_disks(nd);
	if (ret)
		dev_err(nd->dev, "YAFTL: no disks (%d)\n", ret);
}

int n72_yaftl_attach(struct device *dev, struct fmss *f)
{
	static const struct nand_geom geom = {
		.page_size = 8192, .pages_per_block = 128, .ce_blocks = 4148, .chip_enables = 2,
	};
	struct n72_nand *nd;
	int ret;

	nd = devm_kzalloc(dev, sizeof(*nd), GFP_KERNEL);
	if (!nd)
		return -ENOMEM;
	nd->dev = dev;
	nd->f = f;
	mutex_init(&nd->lock);
	INIT_DELAYED_WORK(&nd->wb_work, n72_wb_work);
	nd->page_lpn = U32_MAX;
	nd->page = devm_kmalloc(dev, N72_LBA_SIZE, GFP_KERNEL);
	if (!nd->page)
		return -ENOMEM;
	nd->ops.ctx = nd;
	nd->ops.read_page = n72_read_page;
	nd->ops.write_page = n72_write_page;
	nd->ops.erase_block = n72_erase_block;
	nd->ops.alloc = n72_alloc;
	nd->ops.free = n72_free;
	nd->ops.log = n72_log;

	ret = vfl_open(&nd->vfl, &nd->ops, &geom, 1);
	if (ret) {
		dev_err(dev, "VSVFL: no valid context (%d)\n", ret);
		return -ENODEV;
	}
	ret = yaftl_open(&nd->ftl, &nd->vfl, 0);	/* clean context only */
	if (!ret && nd->ftl.clean)
		return n72_add_disks(nd);
	if (ret >= 0)
		yaftl_close(&nd->ftl);
	dev_dbg(dev, "YAFTL: context stale (iOS not shut down cleanly): rebuilding the map in the background\n");
	INIT_WORK(&nd->restore_work, n72_restore_work);
	schedule_work(&nd->restore_work);
	return 0;
}
