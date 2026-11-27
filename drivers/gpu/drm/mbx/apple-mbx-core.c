// SPDX-License-Identifier: GPL-2.0-only
/*
 * PowerVR MBX Lite on the Apple S5L8720 (iPod touch 2G).
 *
 * Powers and resets the core, runs its MMU, and lets userspace allocate GPU
 * buffers and submit 2D, 3D-blit and TA jobs through a DRM render node;
 * the display (n72-clcd) takes scan-out buffers from it by PRIME.
 */

#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include "apple-mbx.h"

/* ------------------------------------------------------------------ MMU */

/*
 * Load the GPU's TLB entries for @npages pages from the page tables. Each
 * entry is written individually: GPU page to 0x1024, physical page to
 * 0x1028, then bit 8 of the control register; stale entries otherwise
 * survive a page-table change.
 */
int mbx_mmu_flush(struct apple_mbx *mbx, u32 gpu_addr, unsigned int npages)
{
	unsigned int i, page;
	u32 val;
	int ret;

	for (i = 0; i < npages; i++) {
		page = (gpu_addr >> PAGE_SHIFT) + i;
		mbx_write(mbx, MBX_MMU_TLB_VPAGE, page & 0x1fff);
		mbx_write(mbx, MBX_MMU_TLB_PPAGE,
			  mbx->pt[page / MBX_MMU_PT_ENTRIES][page % MBX_MMU_PT_ENTRIES] >>
			  PAGE_SHIFT);
		mbx_write(mbx, MBX_MMU_CTRL, mbx_read(mbx, MBX_MMU_CTRL) | MBX_MMU_CTRL_INVAL);
		ret = readl_poll_timeout(mbx->core + MBX_MMU_CTRL, val,
					 !(val & MBX_MMU_STAT_BUSY), 0, 10000);
		if (ret) {
			dev_err(mbx->dev, "MMU TLB update wedged\n");
			return ret;
		}
	}
	return 0;
}

void mbx_mmu_set(struct apple_mbx *mbx, u32 gpu_addr, phys_addr_t pa)
{
	unsigned int page = gpu_addr >> PAGE_SHIFT;

	mbx->pt[page / MBX_MMU_PT_ENTRIES][page % MBX_MMU_PT_ENTRIES] = pa;
}

static int mbx_mmu_init(struct apple_mbx *mbx)
{
	u32 val;
	int i, j;

	mbx->va_map = devm_bitmap_zalloc(mbx->dev, MBX_GPU_PAGES, GFP_KERNEL);
	if (!mbx->va_map)
		return -ENOMEM;
	/* Keep GPU address 0 unmapped so a zero address faults. */
	set_bit(0, mbx->va_map);

	for (i = 0; i < MBX_MMU_DIR_ENTRIES; i++) {
		mbx->pt[i] = dmam_alloc_coherent(mbx->dev, PAGE_SIZE,
						 &mbx->pt_dma[i], GFP_KERNEL);
		if (!mbx->pt[i])
			return -ENOMEM;
		for (j = 0; j < MBX_MMU_PT_ENTRIES; j++)
			mbx->pt[i][j] = MBX_PTE_INVALID;
		mbx_write(mbx, MBX_MMU_DIR(i), mbx->pt_dma[i]);
	}

	mbx_write(mbx, MBX_MMU_CTRL, MBX_MMU_CTRL_ENABLE);
	return readl_poll_timeout(mbx->core + MBX_MMU_CTRL, val,
				  val & MBX_MMU_STAT_ENABLED, 1, 10000);
}

/* Reserve @npages of GPU address space; returns the GPU address or 0. */
u32 mbx_va_alloc(struct apple_mbx *mbx, unsigned int npages)
{
	unsigned long start;

	start = bitmap_find_next_zero_area(mbx->va_map, MBX_GPU_PAGES, 0, npages, 0);
	if (start >= MBX_GPU_PAGES)
		return 0;
	bitmap_set(mbx->va_map, start, npages);
	return start << PAGE_SHIFT;
}

void mbx_va_free(struct apple_mbx *mbx, u32 gpu_addr, unsigned int npages)
{
	unsigned int i;

	for (i = 0; i < npages; i++)
		mbx_mmu_set(mbx, gpu_addr + i * PAGE_SIZE, MBX_PTE_INVALID);
	mbx_mmu_flush(mbx, gpu_addr, npages);
	bitmap_clear(mbx->va_map, gpu_addr >> PAGE_SHIFT, npages);
}

/* Map the display framebuffer (a physical range) into the GPU. */
static int mbx_map_fb(struct apple_mbx *mbx)
{
	struct device_node *np;
	struct resource res;
	unsigned int i, npages;
	u32 gpu, w = 0, h = 0, stride = 0;
	int ret;

	np = of_parse_phandle(mbx->dev->of_node, "apple,framebuffer", 0);
	if (!np)
		return 0;
	ret = of_address_to_resource(np, 0, &res);
	of_property_read_u32(np, "width", &w);
	of_property_read_u32(np, "height", &h);
	of_property_read_u32(np, "stride", &stride);
	of_node_put(np);
	if (ret)
		return ret;

	npages = PAGE_ALIGN(resource_size(&res)) >> PAGE_SHIFT;
	gpu = mbx_va_alloc(mbx, npages);
	if (!gpu)
		return -ENOMEM;
	for (i = 0; i < npages; i++)
		mbx_mmu_set(mbx, gpu + i * PAGE_SIZE, res.start + i * PAGE_SIZE);
	mbx_mmu_flush(mbx, gpu, npages);

	mbx->fb.gpu_addr = gpu;
	mbx->fb.width = w;
	mbx->fb.height = h;
	mbx->fb.stride = stride;
	mbx->fb.format = MBX_FMT_ARGB8888;
	return 0;
}

/* Header objects at the start of the parameter buffer. */
static void mbx_pb_header(u32 *p)
{
	int i;

	memset(p, 0, 0x37 * 4);
	p[0] = 0xe0000000; p[1] = 0xa8800000; p[2] = 0x0e000000;
	p[3] = 0xd6887610; p[4] = 0x22220e80;
	for (i = 0x0b; i <= 0x10; i++)
		p[i] = 0x3f800000;
	p[0x1a] = 0xe0000000; p[0x1b] = 0x22220e80;
	for (i = 0x22; i <= 0x27; i++)
		p[i] = 0x3f800000;
	p[0x28] = p[0x29] = p[0x2a] = 0xffff00ff;
	p[0x2b] = 0x22206f80;
	for (i = 0x2c; i <= 0x31; i++)
		p[i] = 0xbf800000;
	p[0x32] = p[0x33] = p[0x34] = 0x3f800000;
	p[0x35] = 0x4000002b;
	p[0x36] = 0xf0000000;
}

/* Wait (with the lock held) until the armed status bits arrive. */
bool mbx_wait_completion_short(struct completion *c, unsigned int ms)
{
	ktime_t end = ktime_add_us(ktime_get(), 100);

	do {
		if (try_wait_for_completion(c))
			return true;
		if (need_resched())
			break;
		udelay(2);
	} while (ktime_before(ktime_get(), end));
	return wait_for_completion_timeout(c, msecs_to_jiffies(ms)) != 0;
}

int mbx_wait_bits(struct apple_mbx *mbx, unsigned int ms)
{
	if (!mbx_wait_completion_short(&mbx->done3d, ms))
		return -ETIMEDOUT;
	return 0;
}

/*
 * Finish the asynchronous job, if any (lock held). @ms = 0 only looks:
 * -EBUSY if it is still running. A timeout is handled as for a
 * synchronous job: counted, and a 3D timeout resets the core.
 */
int mbx_idle_ms(struct apple_mbx *mbx, unsigned int ms)
{
	struct completion *c;
	int ret = 0;

	if (!mbx->pending)
		return 0;
	c = mbx->pending == MBX_JOB_2D ? &mbx->done : &mbx->done3d;
	if (!ms) {
		if (!completion_done(c))
			return -EBUSY;
	} else if (!wait_for_completion_timeout(c, msecs_to_jiffies(ms))) {
		ret = -ETIMEDOUT;
		if (mbx->pending == MBX_JOB_2D) {
			dev_warn_ratelimited(mbx->dev, "async 2D batch timed out (status %08x)\n",
					     mbx_read(mbx, MBX_STATUS));
		} else {
			mbx->render_timeouts++;
			mbx_recover(mbx);
		}
	}
	mbx->pending = 0;
	mbx->seq_done = mbx->seq_sub;
	return ret;
}

int mbx_idle(struct apple_mbx *mbx)
{
	return mbx_idle_ms(mbx, mbx->pending_ms ?: 500);
}

static int apple_mbx_power_on(struct apple_mbx *mbx);
static int apple_mbx_reset(struct apple_mbx *mbx);
static void mbx_mmu_reload(struct apple_mbx *mbx);
static int mbx_pb_init(struct apple_mbx *mbx);

/* Gate the core's clock (lock held; no job running). */
static void mbx_gate_locked(struct apple_mbx *mbx)
{
	if (mbx->gated)
		return;
	disable_irq(mbx->irq);
	mbx_write(mbx, MBX_IRQ_ENABLE, 0);
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_EVENTS);
	if (mbx->clk_on) {
		clk_disable_unprepare(mbx->clk);
		mbx->clk_on = false;
	}
	if (mbx->dmac_clk_on) {		/* (no TA frame is queued or running here) */
		clk_disable(mbx->dmac_clk);
		mbx->dmac_clk_on = false;
	}
	mbx->gated = true;
}

/* Ungate and rebuild the state, as resume from sleep did (lock held). */
int mbx_wake_locked(struct apple_mbx *mbx)
{
	int ret;

	ktime_t t0 = ktime_get(), t1, t2, t3;

	if (!mbx->gated)
		return 0;
	mbx->vgp_loaded = false;
	ret = apple_mbx_power_on(mbx);
	t1 = ktime_get();
	if (!ret)
		ret = apple_mbx_reset(mbx);
	t2 = ktime_get();
	if (ret) {
		dev_err(mbx->dev, "core did not come back: %d\n", ret);
		return ret;
	}
	/* the parameter buffer init needs the PB_DONE interrupt */
	enable_irq(mbx->irq);
	mbx->gated = false;
	mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);
	mbx_mmu_reload(mbx);
	t3 = ktime_get();
	if (mbx->pb.hdr && mbx_pb_init(mbx))
		dev_err(mbx->dev, "parameter buffer init failed after ungating\n");
	dev_dbg(mbx->dev, "ungate: power %lld reset %lld mmu %lld pb %lld us\n",
		ktime_us_delta(t1, t0), ktime_us_delta(t2, t1), ktime_us_delta(t3, t2),
		ktime_us_delta(ktime_get(), t3));
	return 0;
}

/*
 * No queued TA frame left (lock held; dropped while waiting, so that
 * ta_work, which takes it to start each frame, can go on).
 */
static void mbx_ta_drain(struct apple_mbx *mbx)
{
	while (!mbx_ta_empty(mbx)) {
		mutex_unlock(&mbx->lock);
		wait_for_completion_timeout(&mbx->ta_done, msecs_to_jiffies(1000));
		mutex_lock(&mbx->lock);
	}
}

/*
 * Every GPU operation starts here (lock held): awake, idle, timer re-armed.
 * @drain: also wait for every queued TA frame (MMU changes, buffer frees,
 * suspend, synchronous frames); else only for the one on the core, which
 * lets the compositor's 2D jobs go ahead of a game's queue (the
 * worker starts the next frame only with mbx->lock).
 */
static int mbx_begin_ex(struct apple_mbx *mbx, bool drain)
{
	int ret;

	if (drain)
		mbx_ta_drain(mbx);
	else if (READ_ONCE(mbx->ta_busy)) {
		atomic_inc(&mbx->ta_waiters);		/* (the pipeline gives the core up between frames) */
		wait_for_completion(&mbx->ta_jdone);	/* bounded: every wait in ta_work is */
		atomic_dec(&mbx->ta_waiters);
	}
	ret = mbx_wake_locked(mbx);
	mbx_idle(mbx);
	mbx->last_use = jiffies;
	mod_delayed_work(system_wq, &mbx->idle_work,
			 msecs_to_jiffies(MBX_IDLE_GATE_MS));
	return ret;
}

int mbx_begin(struct apple_mbx *mbx)
{
	return mbx_begin_ex(mbx, true);
}

/* For the compositor's jobs: ahead of queued TA frames. */
int mbx_begin_light(struct apple_mbx *mbx)
{
	return mbx_begin_ex(mbx, false);
}

static void mbx_idle_work(struct work_struct *work)
{
	struct apple_mbx *mbx = container_of(to_delayed_work(work),
					     struct apple_mbx, idle_work);
	unsigned long idle = msecs_to_jiffies(MBX_IDLE_GATE_MS);

	mutex_lock(&mbx->lock);
	if (!mbx->gated) {
		if (mbx->pending || !mbx_ta_empty(mbx) || time_before(jiffies, mbx->last_use + idle))
			mod_delayed_work(system_wq, &mbx->idle_work, idle);
		else
			mbx_gate_locked(mbx);
	}
	mutex_unlock(&mbx->lock);
}

void mbx_arm_wait(struct apple_mbx *mbx, u32 mask)
{
	unsigned long flags;

	reinit_completion(&mbx->done3d);
	spin_lock_irqsave(&mbx->irq_lock, flags);
	mbx->seen = 0;
	mbx->wait_mask = mask;
	spin_unlock_irqrestore(&mbx->irq_lock, flags);
}

/* Hand the parameter buffer to the core. */
static int mbx_pb_init(struct apple_mbx *mbx)
{
	u32 first = (mbx->pb.gpu >> PAGE_SHIFT) + 1;

	mbx_pb_header(mbx->pb.hdr);
	/* Ensure parameter buffer header writes reach RAM before PB_START. */
	wmb();
	mbx_arm_wait(mbx, MBX_STATUS_PB_DONE);
	mbx_write(mbx, MBX_PB_CTRL, mbx->ctrl_gpu);
	mbx_write(mbx, MBX_PB_FIRST, first);
	mbx_write(mbx, MBX_PB_LAST, first + ((mbx->pb.size - PAGE_SIZE) >> PAGE_SHIFT) - 1);
	mbx_write(mbx, MBX_PB_START, 1);
	mbx_write(mbx, MBX_PB_BASE, mbx->pb.gpu);
	return mbx_wait_bits(mbx, 100);
}

/* @gpu_unmap: also give its GPU range back (the core awake, lock held). */
static void mbx_pb_free(struct apple_mbx *mbx, struct mbx_pb *pb, bool gpu_unmap)
{
	unsigned int i, n = pb->size >> PAGE_SHIFT;

	if (gpu_unmap && pb->gpu)
		mbx_va_free(mbx, pb->gpu, n);
	for (i = 1; pb->pages && i < n; i++) {
		if (!pb->pages[i])
			break;
		dma_unmap_page(mbx->dev, pb->dma[i], PAGE_SIZE, DMA_BIDIRECTIONAL);
		__free_page(pb->pages[i]);
	}
	if (pb->hdr)
		dma_free_coherent(mbx->dev, PAGE_SIZE, pb->hdr, pb->hdr_dma);
	kfree(pb->pages);
	kfree(pb->dma);
	memset(pb, 0, sizeof(*pb));
}

/*
 * A parameter buffer of @size bytes, mapped into the GPU (lock held, the
 * core awake). The pool's pages are order 0: no contiguous run to find
 * after a long uptime.
 */
static int mbx_pb_alloc(struct apple_mbx *mbx, u32 size, struct mbx_pb *pb)
{
	unsigned int i, n = size >> PAGE_SHIFT;

	memset(pb, 0, sizeof(*pb));
	pb->size = size;
	pb->hdr = dma_alloc_coherent(mbx->dev, PAGE_SIZE, &pb->hdr_dma, GFP_KERNEL | __GFP_NOWARN);
	pb->pages = kcalloc(n, sizeof(*pb->pages), GFP_KERNEL | __GFP_NOWARN);
	pb->dma = kcalloc(n, sizeof(*pb->dma), GFP_KERNEL | __GFP_NOWARN);
	if (!pb->hdr || !pb->pages || !pb->dma)
		goto err;
	for (i = 1; i < n; i++) {
		struct page *page = alloc_page(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN);

		if (!page)
			goto err;
		/* (also pushes the zeroes out of the CPU cache; the CPU never touches it again) */
		pb->dma[i] = dma_map_page(mbx->dev, page, 0, PAGE_SIZE, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(mbx->dev, pb->dma[i])) {
			__free_page(page);
			goto err;
		}
		pb->pages[i] = page;
	}
	pb->gpu = mbx_va_alloc(mbx, n);
	if (!pb->gpu)
		goto err;
	mbx_mmu_set(mbx, pb->gpu, pb->hdr_dma);
	for (i = 1; i < n; i++)
		mbx_mmu_set(mbx, pb->gpu + i * PAGE_SIZE, pb->dma[i]);
	mbx_mmu_flush(mbx, pb->gpu, n);
	return 0;

err:
	mbx_pb_free(mbx, pb, false);
	return -ENOMEM;
}

static void mbx_pb_release(void *data)
{
	struct apple_mbx *mbx = data;

	/* (the MMU is off by now: nothing to unmap) */
	mbx_pb_free(mbx, &mbx->pb, false);
}

/*
 * Grow the parameter buffer to @pb_want at the TA start of a later frame:
 * allocate and map a new buffer, write its header, hand it to the core,
 * then free the old one. Lock held, the core awake and with nothing in the
 * parameter buffer (no TA, no render). Keeps the old buffer on failure.
 */
void mbx_pb_grow(struct apple_mbx *mbx)
{
	struct mbx_pb old = mbx->pb, nb;
	u32 want = mbx->pb_want;

	mbx->pb_want = 0;	/* (also when gated: the next full buffer asks again) */
	if (mbx->gated || !old.hdr || want <= old.size)
		return;
	if (mbx_pb_alloc(mbx, want, &nb)) {
		dev_warn_ratelimited(mbx->dev, "no memory for a %u KiB parameter buffer, keeping %u KiB\n",
				     want >> 10, old.size >> 10);
		return;
	}
	mbx->pb = nb;
	if (mbx_pb_init(mbx)) {
		dev_err(mbx->dev, "parameter buffer init failed with %u KiB, keeping %u KiB\n",
			want >> 10, old.size >> 10);
		mbx->pb = old;
		mbx_pb_free(mbx, &nb, true);
		mbx_recover(mbx);
		return;
	}
	mbx_pb_free(mbx, &old, true);
	mbx->pb_grows++;
	dev_dbg(mbx->dev, "parameter buffer: %u KiB\n", want >> 10);
}

/* Kernel-owned, physically contiguous memory, mapped into the GPU. */
static void *mbx_kalloc(struct apple_mbx *mbx, size_t size, dma_addr_t *dma, u32 *gpu)
{
	unsigned int i, npages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	void *p;

	p = dmam_alloc_coherent(mbx->dev, size, dma, GFP_KERNEL);
	if (!p)
		return NULL;
	*gpu = mbx_va_alloc(mbx, npages);
	if (!*gpu)
		return NULL;
	for (i = 0; i < npages; i++)
		mbx_mmu_set(mbx, *gpu + i * PAGE_SIZE, *dma + i * PAGE_SIZE);
	mbx_mmu_flush(mbx, *gpu, npages);
	return p;
}

static int mbx_3d_init(struct apple_mbx *mbx)
{
	void *ctrl;
	int ret;

	mbx->tmpl = mbx_kalloc(mbx, PAGE_SIZE, &mbx->tmpl_dma, &mbx->tmpl_gpu);
	mbx->tiles = mbx_kalloc(mbx, MBX_TILE_BUF_SIZE, &mbx->tiles_dma, &mbx->tiles_gpu);
	mbx->tiles0 = mbx_kalloc(mbx, MBX_TILE_BUF_SIZE, &mbx->tiles0_dma, &mbx->tiles0_gpu);
	ctrl = mbx_kalloc(mbx, MBX_PB_CTRL_SIZE, &mbx->ctrl_dma, &mbx->ctrl_gpu);
	mbx->tail = mbx_kalloc(mbx, MBX_TA_TAIL_SIZE, &mbx->tail_dma, &mbx->tail_gpu);
	mbx->tactrl = mbx_kalloc(mbx, MBX_TA_CTRL_SIZE, &mbx->tactrl_dma, &mbx->tactrl_gpu);
	mbx->ta_lli = dmam_alloc_coherent(mbx->dev,
					  (MBX_TA_QUEUE + 2) * MBX_TA_LLI_PER_SLOT * sizeof(*mbx->ta_lli),
					  &mbx->ta_lli_dma, GFP_KERNEL);
	if (!mbx->tmpl || !mbx->tiles || !mbx->tiles0 || !ctrl ||
	    !mbx->tail || !mbx->tactrl || !mbx->ta_lli)
		return -ENOMEM;
	ret = mbx_pb_alloc(mbx, MBX_PB_MIN, &mbx->pb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(mbx->dev, mbx_pb_release, mbx);
	if (ret)
		return ret;
	mbx_tmpl_reset(mbx);
	return mbx_pb_init(mbx);
}

/* Reload every GPU TLB entry, e.g. after a core reset. */
static void mbx_mmu_reload(struct apple_mbx *mbx)
{
	unsigned int i;
	u32 val;

	for (i = 0; i < MBX_MMU_DIR_ENTRIES; i++)
		mbx_write(mbx, MBX_MMU_DIR(i), mbx->pt_dma[i]);
	mbx_write(mbx, MBX_MMU_CTRL, MBX_MMU_CTRL_ENABLE);
	readl_poll_timeout(mbx->core + MBX_MMU_CTRL, val, val & MBX_MMU_STAT_ENABLED, 1, 10000);
	for (i = 0; i < MBX_GPU_PAGES; i++)
		if (test_bit(i, mbx->va_map))
			mbx_mmu_flush(mbx, i << PAGE_SHIFT, 1);
}

static int apple_mbx_reset(struct apple_mbx *mbx);

/* After a hung render: reset the core and bring the 3D state back. */
void mbx_recover(struct apple_mbx *mbx)
{
	dev_warn(mbx->dev, "resetting the core (status %08x)\n", mbx_read(mbx, MBX_STATUS));
	apple_mbx_reset(mbx);
	mbx->vgp_loaded = false;
	mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);
	mbx_mmu_reload(mbx);
	if (mbx_pb_init(mbx))
		dev_err(mbx->dev, "parameter buffer init failed after reset\n");
}

/* ---------------------------------------------------------------- core */

/*
 * Clock: iOS's device-clocks table gives the MBX (clock id 0x18) three gate
 * bits, 0x3c500068 bits 5-7, with no parent clocks. Ungating only some of them
 * hangs the SoC, so they are cleared together in one write. An unclocked core
 * returns stale bus data, so check two registers with distinct reset values.
 */
static int apple_mbx_power_on(struct apple_mbx *mbx)
{
	int ret;

	if (mbx->clk && !mbx->clk_on) {
		ret = clk_prepare_enable(mbx->clk);
		if (ret)
			return ret;
		mbx->clk_on = true;
	}

	if (mbx_read(mbx, MBX_ID_FF) != 0xff || mbx_read(mbx, MBX_ID_FFFF) != 0xffff)
		return -ENODEV;
	return 0;
}

static void mbx_clk_release(void *data)
{
	struct apple_mbx *mbx = data;

	if (mbx->clk_on) {
		clk_disable_unprepare(mbx->clk);
		mbx->clk_on = false;
	}
}

static int apple_mbx_reset(struct apple_mbx *mbx)
{
	u32 val;
	int ret;

	writel(1, mbx->wrap + WRAP_RESET);
	ret = readl_poll_timeout(mbx->wrap + WRAP_RESET, val, !(val & 1), 1,
				 10000);
	if (ret) {
		dev_err(mbx->dev, "wrapper reset timed out\n");
		return ret;
	}

	mbx_write(mbx, MBX_SOFT_RESET, MBX_SOFT_RESET_ALL);
	udelay(10);
	mbx_write(mbx, MBX_SOFT_RESET, 0);
	udelay(10);

	mbx_write(mbx, MBX_CONFIG, 0);
	mbx_write(mbx, MBX_IRQ_CLEAR, MBX_STATUS_EVENTS);
	return 0;
}

static irqreturn_t apple_mbx_irq(int irq, void *data)
{
	struct apple_mbx *mbx = data;
	u32 status = mbx_read(mbx, MBX_STATUS) & MBX_STATUS_EVENTS;
	struct dma_fence *rf = NULL;
	unsigned long bits = status, flags;
	int b;

	if (!status)
		return IRQ_NONE;
	spin_lock_irqsave(&mbx->irq_lock, flags);
	mbx_write(mbx, MBX_IRQ_CLEAR, status);
	mbx->irq_count++;
	for_each_set_bit(b, &bits, ARRAY_SIZE(mbx->event_count))
		mbx->event_count[b]++;
	if (status & MBX_STATUS_2D_DONE) {
		mbx->t_2d_irq = ktime_get();
		complete(&mbx->done);
	}
	if (mbx->ov) {
		/*
		 * The pipeline: its waiters look at what was cleared here. A
		 * reset in the pipeline (mbx_recover) waits for the parameter
		 * buffer's init by wait_mask too, and timed out every time when
		 * this branch did not complete it.
		 */
		if (mbx->wait_mask) {
			mbx->seen |= status;
			if ((mbx->seen & mbx->wait_mask) == mbx->wait_mask) {
				mbx->wait_mask = 0;
				complete(&mbx->done3d);
			}
		}
		mbx->ov_seen |= status;
		if (mbx->ov_rarmed) {
			mbx->ov_rseen |= status & MBX_STATUS_RENDER_DONE;
			if (mbx->ov_rseen == MBX_STATUS_RENDER_DONE) {
				rf = mbx->ov_rfence;

				mbx->ov_rarmed = false;
				mbx->ov_rfence = NULL;
				mbx->ov_rt_done = ktime_get();
				complete(&mbx->ta_rdone);
			}
		}
		if (++mbx->ov_irqs > 20000 && !mbx->ov_storm) {
			/* an event that does not clear: no more interrupts, the waits time out */
			mbx_write(mbx, MBX_IRQ_ENABLE, 0);
			mbx->ov_storm = true;
		}
		spin_unlock_irqrestore(&mbx->irq_lock, flags);
		if (rf)
			dma_fence_signal(rf);
		wake_up(&mbx->ov_wq);
		return IRQ_HANDLED;
	}
	if (mbx->wait_mask) {
		mbx->seen |= status;
		if ((mbx->seen & mbx->wait_mask) == mbx->wait_mask) {
			mbx->wait_mask = 0;
			complete(&mbx->done3d);
		}
	}
	spin_unlock_irqrestore(&mbx->irq_lock, flags);
	return IRQ_HANDLED;
}

static void mbx_dmac_release(void *data)
{
	struct apple_mbx *mbx = data;

	if (mbx->dmac_clk_on)
		clk_disable(mbx->dmac_clk);
	clk_unprepare(mbx->dmac_clk);
	clk_put(mbx->dmac_clk);
	iounmap(mbx->dmac);
}

static int apple_mbx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_mbx *mbx;
	int ret;

	mbx = devm_kzalloc(dev, sizeof(*mbx), GFP_KERNEL);
	if (!mbx)
		return -ENOMEM;
	mbx->dev = dev;
	mutex_init(&mbx->lock);
	init_completion(&mbx->done);
	init_completion(&mbx->done3d);
	init_completion(&mbx->ta_done);
	complete_all(&mbx->ta_done);	/* the queue is empty */
	init_completion(&mbx->ta_rdone);
	init_completion(&mbx->ta_jdone);
	complete_all(&mbx->ta_jdone);
	spin_lock_init(&mbx->ta_qlock);
	init_waitqueue_head(&mbx->ta_slot_wq);
	init_waitqueue_head(&mbx->ov_wq);
	mbx->ta_idx = 1;
	INIT_WORK(&mbx->ta_work, mbx_ta_work);
	spin_lock_init(&mbx->fence_lock);
	spin_lock_init(&mbx->irq_lock);
	mbx->fence_ctx = dma_fence_context_alloc(1);

	mbx->core = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(mbx->core))
		return PTR_ERR(mbx->core);
	mbx->core_phys = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core")->start;
	/*
	 * Optional: the DMA controller that feeds the TA port,
	 * "apple,feed-dma" = <&controller channel>. The controller's node is
	 * reserved for us (no dmaengine driver binds it): its registers and
	 * its clock are taken from there.
	 */
	{
		struct of_phandle_args a;

		if (!of_parse_phandle_with_fixed_args(pdev->dev.of_node, "apple,feed-dma", 1, 0, &a)) {
			mbx->dma_channel = a.args[0];
			mbx->dmac_clk = of_clk_get(a.np, 0);
			if (!IS_ERR(mbx->dmac_clk) && mbx->dma_channel < 8 && !clk_prepare(mbx->dmac_clk)) {
				mbx->dmac = of_iomap(a.np, 0);
				if (!mbx->dmac)
					clk_unprepare(mbx->dmac_clk);
			}
			of_node_put(a.np);
			if (mbx->dmac) {
				ret = devm_add_action_or_reset(dev, mbx_dmac_release, mbx);
				if (ret)
					return ret;
			} else {
				if (!IS_ERR(mbx->dmac_clk))
					clk_put(mbx->dmac_clk);
				mbx->dmac_clk = NULL;
				dev_warn(&pdev->dev, "apple,feed-dma unusable: TA fed through the port\n");
			}
		}
	}
	mbx->wrap = devm_platform_ioremap_resource_byname(pdev, "wrapper");
	if (IS_ERR(mbx->wrap))
		return PTR_ERR(mbx->wrap);
	mbx->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(mbx->clk))
		return dev_err_probe(dev, PTR_ERR(mbx->clk), "clock\n");
	if (mbx->clk) {
		ret = devm_add_action_or_reset(dev, mbx_clk_release, mbx);
		if (ret)
			return ret;
	}

	mbx->irq = platform_get_irq(pdev, 0);
	if (mbx->irq < 0)
		return mbx->irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = apple_mbx_power_on(mbx);
	if (ret)
		return dev_err_probe(dev, ret, "core does not respond\n");
	ret = apple_mbx_reset(mbx);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, mbx->irq, apple_mbx_irq, 0, dev_name(dev), mbx);
	if (ret)
		return ret;
	mbx_write(mbx, MBX_IRQ_ENABLE, MBX_IRQ_MASK);

	ret = mbx_mmu_init(mbx);
	if (ret)
		return dev_err_probe(dev, ret, "MMU did not enable\n");
	ret = mbx_3d_init(mbx);
	if (ret)
		dev_warn(dev, "3D core not initialised: %d\n", ret);
	ret = mbx_map_fb(mbx);
	if (ret)
		dev_warn(dev, "framebuffer not mapped: %d\n", ret);

	INIT_DELAYED_WORK(&mbx->idle_work, mbx_idle_work);
	mbx->last_use = jiffies;
	ret = mbx_drm_register(mbx);
	if (ret)
		return dev_err_probe(dev, ret, "DRM render node\n");
	/* (only now: a probe that failed above must not leave the work behind) */
	schedule_delayed_work(&mbx->idle_work, msecs_to_jiffies(MBX_IDLE_GATE_MS));

	mbx_debugfs_init(mbx);

	platform_set_drvdata(pdev, mbx);
	dev_info(dev, "MBX Lite up: revision %08x, MMU on, framebuffer at GPU %08x; render node %s\n",
		 mbx_read(mbx, MBX_REVISION), mbx->fb.gpu_addr, mbx->drm->render->kdev ?
		 dev_name(mbx->drm->render->kdev) : "?");
	return 0;
}

static void apple_mbx_remove(struct platform_device *pdev)
{
	struct apple_mbx *mbx = platform_get_drvdata(pdev);
	unsigned int i;

	drm_dev_unregister(mbx->drm);
	cancel_delayed_work_sync(&mbx->idle_work);
	flush_work(&mbx->ta_work);
	for (i = 0; i < ARRAY_SIZE(mbx->ta_buf); i++) {
		struct mbx_ta_buf *b = &mbx->ta_buf[i];

		if (b->dma)
			dma_free_coherent(mbx->dev, (size_t)(b->cap + MBX_TA_BUF_SLACK) * 4, b->words, b->dma);
		else
			vfree(b->words);
	}
	debugfs_remove_recursive(mbx->debugfs);
	mutex_lock(&mbx->lock);
	if (!mbx->gated) {
		mbx_write(mbx, MBX_IRQ_ENABLE, 0);
		mbx_write(mbx, MBX_MMU_CTRL, 0);
	}
	mutex_unlock(&mbx->lock);
	release_firmware(mbx->vgp_fw);
}

/*
 * System sleep: wait out any submission, then gate the core clock. Resume
 * ungates it and rebuilds the state the way a hung-render recovery does, so
 * nothing depends on what the core keeps while unclocked. Userspace is
 * frozen around this, so no ioctl can reach an ungated core.
 */
static int apple_mbx_suspend(struct device *dev)
{
	struct apple_mbx *mbx = dev_get_drvdata(dev);

	mutex_lock(&mbx->lock);
	mbx_ta_drain(mbx);
	mbx_idle(mbx);
	mbx_gate_locked(mbx);
	mutex_unlock(&mbx->lock);
	return 0;
}

static int apple_mbx_resume(struct device *dev)
{
	/*
	 * Nothing to do: the core stays gated and the next GPU operation
	 * brings it back (mbx_wake_locked), so resume does not pay for it.
	 */
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_mbx_pm_ops, apple_mbx_suspend, apple_mbx_resume);

static const struct of_device_id apple_mbx_of_match[] = {
	{ .compatible = "apple,s5l8720-mbx" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_mbx_of_match);

static struct platform_driver apple_mbx_driver = {
	.probe = apple_mbx_probe,
	.remove = apple_mbx_remove,
	.driver = {
		.name = "apple-mbx",
		.of_match_table = apple_mbx_of_match,
		.pm = pm_sleep_ptr(&apple_mbx_pm_ops),
	},
};
module_platform_driver(apple_mbx_driver);

MODULE_DESCRIPTION("PowerVR MBX Lite on Apple S5L8720");
MODULE_LICENSE("GPL");
