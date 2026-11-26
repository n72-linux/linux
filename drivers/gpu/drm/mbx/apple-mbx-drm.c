// SPDX-License-Identifier: GPL-2.0-only
/* PowerVR MBX Lite on the Apple S5L8720: GPU buffers, GEM objects and the DRM render node. */

#include <linux/dma-buf.h>
#include <linux/dma-map-ops.h>	/* dma_alloc_contiguous() */
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_prime.h>
#include <drm/drm_vma_manager.h>
#include <uapi/drm/apple_mbx_drm.h>
#include "apple-mbx.h"

/* ------------------------------------------------------------- buffers */

static void mbx_bo_free(struct mbx_bo *bo);

static void mbx_bo_release(struct kref *ref)
{
	mbx_bo_free(container_of(ref, struct mbx_bo, ref));
}

static void mbx_bo_put(struct mbx_bo *bo)
{
	kref_put(&bo->ref, mbx_bo_release);
}

static void mbx_bo_free(struct mbx_bo *bo)
{
	struct apple_mbx *mbx = bo->mbx;
	unsigned int i;

	if (bo->gpu_addr) {
		mutex_lock(&mbx->lock);
		mbx_begin(mbx);			/* awake; the GPU may still be using it */
		mbx_va_free(mbx, bo->gpu_addr, bo->npages);
		mutex_unlock(&mbx->lock);
	}
	for (i = 0; i < bo->npages && !bo->imported; i++) {
		dma_unmap_page(mbx->dev, bo->dma[i], PAGE_SIZE, DMA_BIDIRECTIONAL);
		if (!bo->cma)
			__free_page(bo->pages[i]);
	}
	if (bo->cma)
		dma_free_contiguous(mbx->dev, bo->cma, (size_t)bo->npages << PAGE_SHIFT);
	kvfree(bo->dma);
	kvfree(bo->pages);
	kfree(bo);
}

/* Give the buffer's pages an address range in the GPU's MMU. */
static int mbx_bo_map_gpu(struct mbx_bo *bo)
{
	struct apple_mbx *mbx = bo->mbx;
	unsigned int i;

	mutex_lock(&mbx->lock);
	mbx_begin(mbx);			/* awake; no MMU changes under a running job */
	bo->gpu_addr = mbx_va_alloc(mbx, bo->npages);
	if (bo->gpu_addr) {
		for (i = 0; i < bo->npages; i++)
			mbx_mmu_set(mbx, bo->gpu_addr + i * PAGE_SIZE,
				    bo->dma[i]);
		mbx_mmu_flush(mbx, bo->gpu_addr, bo->npages);
	}
	mutex_unlock(&mbx->lock);
	return bo->gpu_addr ? 0 : -ENOSPC;
}

static struct mbx_bo *mbx_bo_alloc(struct apple_mbx *mbx, size_t size, bool cached,
				    bool contig, bool no_gpu)
{
	struct mbx_bo *bo;
	unsigned int i, mapped = 0;
	void *va = NULL;

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return NULL;
	bo->mbx = mbx;
	kref_init(&bo->ref);
	bo->cached = cached;
	bo->contig = contig;
	bo->npages = PAGE_ALIGN(size) >> PAGE_SHIFT;
	bo->pages = kvcalloc(bo->npages, sizeof(*bo->pages), GFP_KERNEL);
	bo->dma = kvcalloc(bo->npages, sizeof(*bo->dma), GFP_KERNEL);
	if (!bo->pages || !bo->dma)
		goto err;

	if (contig) {
		/*
		 * From the CMA area: the kernel moves movable pages out of the
		 * way, so this works however fragmented memory has become
		 * after a long uptime (the buddy allocator's 1 MiB runs ran
		 * out: the launcher could not have its scan-out buffers). Else
		 * order-0 pages after a split, freed one by one like the others.
		 */
		bo->cma = dma_alloc_contiguous(mbx->dev, (size_t)bo->npages << PAGE_SHIFT,
					       GFP_KERNEL | __GFP_NOWARN);
		if (bo->cma) {
			va = page_address(bo->cma);
			memset(va, 0, (size_t)bo->npages << PAGE_SHIFT);
		} else {
			va = alloc_pages_exact(bo->npages << PAGE_SHIFT, GFP_KERNEL | __GFP_ZERO);
		}
		if (!va)
			goto err;
	}
	for (i = 0; i < bo->npages; i++) {
		bo->pages[i] = contig ? virt_to_page(va + i * PAGE_SIZE) :
			       alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!bo->pages[i])
			goto err;
		/* Also pushes the zeroes out of the CPU cache. */
		bo->dma[i] = dma_map_page(mbx->dev, bo->pages[i], 0, PAGE_SIZE,
					  DMA_BIDIRECTIONAL);
		if (dma_mapping_error(mbx->dev, bo->dma[i]))
			goto err;
		mapped++;
	}

	if (!no_gpu && mbx_bo_map_gpu(bo))
		goto err;
	return bo;

err:
	for (i = 0; i < mapped; i++)
		dma_unmap_page(mbx->dev, bo->dma[i], PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (bo->cma)
		dma_free_contiguous(mbx->dev, bo->cma, (size_t)bo->npages << PAGE_SHIFT);
	else if (va)
		free_pages_exact(va, (size_t)bo->npages << PAGE_SHIFT);
	else if (bo->pages)
		for (i = 0; i < bo->npages; i++)
			if (bo->pages[i])
				__free_page(bo->pages[i]);
	kvfree(bo->dma);
	kvfree(bo->pages);
	kfree(bo);
	return NULL;
}

/*
 * The ARM1176's data cache holds 16 KiB (512 lines). Syncing a range walks
 * every line address in it (twice for a flush): 19,200 for one 320x480
 * buffer, ~0.75 ms, to find at most 512 lines. From 32 KiB up, the
 * whole data cache is cleaned (and, for a flush, dropped) instead: a
 * superset of the range, in the time of a 16 KiB one. Interrupts are off
 * for it (it is not restartable part-way) and the write buffer is drained,
 * as the range functions do.
 */
#define MBX_SYNC_WHOLE_BYTES	SZ_32K

static bool mbx_sync_whole(u32 op, u32 size)
{
#if (defined(CONFIG_CPU_V6) || defined(CONFIG_CPU_V6K)) && !defined(CONFIG_SMP)
	unsigned long flags;

	if (size < MBX_SYNC_WHOLE_BYTES)
		return false;
	local_irq_save(flags);
	if (op == MBX_SYNC_FLUSH)
		asm volatile("mcr p15, 0, %0, c7, c14, 0" : : "r" (0));	/* clean + invalidate D */
	else
		asm volatile("mcr p15, 0, %0, c7, c10, 0" : : "r" (0));	/* clean D */
	asm volatile("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));		/* drain write buffer */
	local_irq_restore(flags);
	return true;
#else
	return false;
#endif
}

static void mbx_bo_sync(struct mbx_bo *bo, u32 op, u32 offset, u32 size)
{
	struct device *dev = bo->mbx->dev;
	u32 end = offset + size;

	if (mbx_sync_whole(op, size))
		return;
	while (offset < end) {
		unsigned int i = offset >> PAGE_SHIFT;
		u32 off = offset & ~PAGE_MASK;
		u32 len = min_t(u32, PAGE_SIZE - off, end - offset);

		/* Clean: write dirty lines back. Flush: also drop them. */
		dma_sync_single_for_device(dev, bo->dma[i] + off, len, DMA_TO_DEVICE);
		if (op == MBX_SYNC_FLUSH)
			dma_sync_single_for_cpu(dev, bo->dma[i] + off, len,
						DMA_FROM_DEVICE);
		offset += len;
	}
}

/* ------------------------------------------------------- DRM render node */

/*
 * The interface: a render-only DRM device (/dev/dri/renderD*, driver
 * "apple-mbx"). GEM objects wrap mbx_bo; PRIME export goes through DRM's
 * own dma-buf ops (get_sg_table), import maps any dma-buf's pages into
 * the GPU. Jobs take the <linux/apple-mbx.h> structures.
 */
struct mbx_drm {
	struct drm_device drm;
	struct apple_mbx *mbx;
};

struct mbx_gem {
	struct drm_gem_object base;
	struct mbx_bo *bo;
};

#define to_mbx_gem(o) container_of(o, struct mbx_gem, base)

static struct apple_mbx *drm_to_mbx(struct drm_device *drm)
{
	return container_of(drm, struct mbx_drm, drm)->mbx;
}

static void mbx_gem_free(struct drm_gem_object *obj)
{
	struct mbx_gem *g = to_mbx_gem(obj);

	if (g->bo)
		mbx_bo_put(g->bo);	/* the GPU mapping goes before the pages do */
	if (obj->import_attach)
		drm_prime_gem_destroy(obj, obj->import_attach->sgt);
	drm_gem_object_release(obj);
	kfree(g);
}

static int mbx_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct mbx_bo *bo = to_mbx_gem(obj)->bo;
	unsigned long i, npages = vma_pages(vma);
	int ret;

	if (bo->imported || npages > bo->npages)
		return -EINVAL;	/* imported: map it through its exporter */
	vma->vm_pgoff = 0;
	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
	if (!bo->cached)
		vma->vm_page_prot = pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	for (i = 0; i < npages; i++) {
		ret = vm_insert_page(vma, vma->vm_start + i * PAGE_SIZE, bo->pages[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static struct sg_table *mbx_gem_get_sg_table(struct drm_gem_object *obj)
{
	struct mbx_bo *bo = to_mbx_gem(obj)->bo;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);
	ret = sg_alloc_table_from_pages(sgt, bo->pages, bo->npages, 0,
					(size_t)bo->npages << PAGE_SHIFT, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}
	return sgt;
}

static const struct vm_operations_struct mbx_gem_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct drm_gem_object_funcs mbx_gem_funcs = {
	.free = mbx_gem_free,
	.mmap = mbx_gem_mmap,
	.get_sg_table = mbx_gem_get_sg_table,
	.vm_ops = &mbx_gem_vm_ops,
};

static struct mbx_gem *mbx_gem_new(struct drm_device *drm, size_t size)
{
	struct mbx_gem *g = kzalloc(sizeof(*g), GFP_KERNEL);

	if (!g)
		return NULL;
	g->base.funcs = &mbx_gem_funcs;
	drm_gem_private_object_init(drm, &g->base, size);
	return g;
}

static struct drm_gem_object *mbx_gem_import_sg_table(struct drm_device *drm,
						      struct dma_buf_attachment *at,
						      struct sg_table *sgt)
{
	struct apple_mbx *mbx = drm_to_mbx(drm);
	size_t size = PAGE_ALIGN(at->dmabuf->size);
	struct mbx_gem *g;
	struct mbx_bo *bo;
	int ret = -ENOMEM;

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return ERR_PTR(-ENOMEM);
	bo->mbx = mbx;
	kref_init(&bo->ref);
	bo->imported = true;
	bo->npages = size >> PAGE_SHIFT;
	bo->pages = kvcalloc(bo->npages, sizeof(*bo->pages), GFP_KERNEL);
	bo->dma = kvcalloc(bo->npages, sizeof(*bo->dma), GFP_KERNEL);
	if (!bo->pages || !bo->dma)
		goto err;
	ret = drm_prime_sg_to_page_array(sgt, bo->pages, bo->npages);
	if (!ret)
		ret = drm_prime_sg_to_dma_addr_array(sgt, bo->dma, bo->npages);
	if (!ret)
		ret = mbx_bo_map_gpu(bo);
	if (ret)
		goto err;
	g = mbx_gem_new(drm, size);
	if (!g) {
		mbx_bo_put(bo);
		return ERR_PTR(-ENOMEM);
	}
	g->bo = bo;
	return &g->base;
err:
	kvfree(bo->dma);
	kvfree(bo->pages);
	kfree(bo);
	return ERR_PTR(ret);
}

static struct mbx_bo *mbx_lookup_bo(struct drm_file *file, u32 handle,
				    struct drm_gem_object **objp)
{
	struct drm_gem_object *obj = drm_gem_object_lookup(file, handle);

	*objp = obj;
	return obj ? to_mbx_gem(obj)->bo : NULL;
}

static int mbx_drm_gem_create(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_apple_mbx_gem_create *c = data;
	struct mbx_gem *g;
	struct mbx_bo *bo;
	int ret;

	if (!c->size || c->size > SZ_16M || c->pad ||
	    (c->flags & ~(APPLE_MBX_BO_CACHED | APPLE_MBX_BO_CONTIG | APPLE_MBX_BO_NO_GPU)))
		return -EINVAL;
	bo = mbx_bo_alloc(drm_to_mbx(drm), c->size, c->flags & APPLE_MBX_BO_CACHED,
			  c->flags & APPLE_MBX_BO_CONTIG, c->flags & APPLE_MBX_BO_NO_GPU);
	if (!bo)
		return -ENOMEM;
	g = mbx_gem_new(drm, (size_t)bo->npages << PAGE_SHIFT);
	if (!g) {
		mbx_bo_put(bo);
		return -ENOMEM;
	}
	g->bo = bo;
	ret = drm_gem_handle_create(file, &g->base, &c->handle);
	drm_gem_object_put(&g->base);	/* the handle holds it now */
	if (ret)
		return ret;
	c->gpu_addr = bo->gpu_addr;
	return 0;
}

static int mbx_drm_gem_info(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_apple_mbx_gem_info *in = data;
	struct drm_gem_object *obj;
	struct mbx_bo *bo = mbx_lookup_bo(file, in->handle, &obj);
	int ret = 0;

	if (!bo) {
		if (obj)
			drm_gem_object_put(obj);
		return -ENOENT;
	}
	in->gpu_addr = bo->gpu_addr;
	in->size = obj->size;
	in->mmap_offset = 0;
	if (!bo->imported) {
		ret = drm_gem_create_mmap_offset(obj);
		if (!ret)
			in->mmap_offset = drm_vma_node_offset_addr(&obj->vma_node);
	}
	drm_gem_object_put(obj);
	return ret;
}

static int mbx_drm_gem_sync(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_apple_mbx_gem_sync *sy = data;
	struct drm_gem_object *obj;
	struct mbx_bo *bo = mbx_lookup_bo(file, sy->handle, &obj);
	int ret = 0;

	if (!bo) {
		if (obj)
			drm_gem_object_put(obj);
		return -ENOENT;
	}
	if ((sy->op != MBX_SYNC_CLEAN && sy->op != MBX_SYNC_FLUSH) ||
	    sy->offset > (u32)bo->npages << PAGE_SHIFT ||
	    sy->size > ((u32)bo->npages << PAGE_SHIFT) - sy->offset)
		ret = -EINVAL;
	else if (bo->cached && !bo->imported)
		mbx_bo_sync(bo, sy->op, sy->offset, sy->size);
	drm_gem_object_put(obj);
	return ret;
}

static int mbx_drm_submit(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_apple_mbx_submit *sb = data;
	u32 stack_words[64];
	u32 *words;
	int ret;

	if (!sb->count || sb->count > MBX_SUBMIT_MAX_WORDS || (sb->flags & ~APPLE_MBX_JOB_ASYNC))
		return -EINVAL;
	if (sb->count <= ARRAY_SIZE(stack_words)) {
		if (copy_from_user(stack_words, u64_to_user_ptr(sb->words),
				   sb->count * sizeof(u32)))
			return -EFAULT;
		words = stack_words;
	} else {
		words = vmemdup_array_user(u64_to_user_ptr(sb->words), sb->count, sizeof(u32));
		if (IS_ERR(words))
			return PTR_ERR(words);
	}
	ret = mbx_submit(drm_to_mbx(drm), words, sb->count, sb->timeout_ms,
			 sb->flags & APPLE_MBX_JOB_ASYNC);
	if (words != stack_words)
		kvfree(words);
	sb->seq = ret > 0 ? ret : 0;
	return ret > 0 ? 0 : ret;
}

static int mbx_drm_blit3d(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_apple_mbx_blit3d *b = data;
	int ret;

	if (b->flags & ~APPLE_MBX_JOB_ASYNC)
		return -EINVAL;
	ret = mbx_blit3d(drm_to_mbx(drm), &b->b, b->flags & APPLE_MBX_JOB_ASYNC);
	b->seq = ret > 0 ? ret : 0;
	return ret > 0 ? 0 : ret;
}

static int mbx_drm_ta_frame(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct apple_mbx *mbx = drm_to_mbx(drm);
	struct mbx_ta_frame *f = data;
	struct drm_gem_object *target = NULL, *stream_obj = NULL;
	struct mbx_bo *stream_bo = NULL;
	int ret;

	if (f->flags & MBX_TA_GEM_WORDS) {
		u32 last_off, last_pg, pg_off;

		if (!f->count || f->count > MBX_TA_MAX_WORDS || f->words > U32_MAX)
			return -EINVAL;
		stream_bo = mbx_lookup_bo(file, (u32)f->words, &stream_obj);
		if (!stream_obj || stream_obj->funcs != &mbx_gem_funcs || !stream_bo ||
		    stream_bo->imported || !stream_bo->contig ||
		    (size_t)f->count * 4 > ((size_t)stream_bo->npages << PAGE_SHIFT)) {
			if (stream_obj)
				drm_gem_object_put(stream_obj);
			return -EINVAL;
		}
		if (stream_bo->cached)
			mbx_bo_sync(stream_bo, MBX_SYNC_CLEAN, 0, f->count * 4);
		last_off = (f->count - 1) * 4;
		last_pg = last_off >> PAGE_SHIFT;
		pg_off = last_off & ~PAGE_MASK;
		if (!stream_bo->cached)
			dma_sync_single_for_cpu(mbx->dev, stream_bo->dma[last_pg] + pg_off, 4,
						DMA_FROM_DEVICE);
		if (*(u32 *)(page_address(stream_bo->pages[last_pg]) + pg_off) != 0xf0000000) {
			drm_gem_object_put(stream_obj);
			return -EINVAL;
		}
	}
	if (f->flags & MBX_TA_FENCE) {
		/* the target by its GPU address among this file's objects */
		struct drm_gem_object *obj;
		int id;

		spin_lock(&file->table_lock);
		idr_for_each_entry(&file->object_idr, obj, id) {
			struct mbx_gem *g = to_mbx_gem(obj);

			if (obj->funcs == &mbx_gem_funcs && g->bo && g->bo->gpu_addr &&
			    g->bo->gpu_addr == f->dst_gpu) {
				drm_gem_object_get(obj);
				target = obj;
				break;
			}
		}
		spin_unlock(&file->table_lock);
	}
	ret = mbx_ta_frame(mbx, f, u64_to_user_ptr(f->words), target ? target->resv : NULL,
			   stream_obj, stream_bo);
	if (target)
		drm_gem_object_put(target);
	if (stream_obj && (ret || !(f->flags & MBX_TA_ASYNC)))
		drm_gem_object_put(stream_obj);
	return ret;
}

static int mbx_drm_wait(struct drm_device *drm, void *data, struct drm_file *file)
{
	return mbx_do_wait(drm_to_mbx(drm), data);
}

static int mbx_drm_get_param(struct drm_device *drm, void *data, struct drm_file *file)
{
	struct drm_apple_mbx_get_param *gp = data;
	struct apple_mbx *mbx = drm_to_mbx(drm);

	switch (gp->param) {
	case APPLE_MBX_PARAM_REVISION:
		mutex_lock(&mbx->lock);
		mbx_begin(mbx);
		gp->value = mbx_read(mbx, MBX_REVISION);
		mutex_unlock(&mbx->lock);
		return 0;
	case APPLE_MBX_PARAM_FB_GPU_ADDR: gp->value = mbx->fb.gpu_addr; break;
	case APPLE_MBX_PARAM_FB_WIDTH: gp->value = mbx->fb.width; break;
	case APPLE_MBX_PARAM_FB_HEIGHT: gp->value = mbx->fb.height; break;
	case APPLE_MBX_PARAM_FB_STRIDE: gp->value = mbx->fb.stride; break;
	case APPLE_MBX_PARAM_FB_FORMAT: gp->value = mbx->fb.format; break;
	default:
		return -EINVAL;
	}
	return mbx->fb.gpu_addr ? 0 : -ENODEV;
}

static const struct drm_ioctl_desc mbx_drm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(APPLE_MBX_GEM_CREATE, mbx_drm_gem_create, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_MBX_GEM_INFO, mbx_drm_gem_info, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_MBX_GEM_SYNC, mbx_drm_gem_sync, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_MBX_SUBMIT, mbx_drm_submit, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_MBX_BLIT3D, mbx_drm_blit3d, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_MBX_TA_FRAME, mbx_drm_ta_frame, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_MBX_WAIT, mbx_drm_wait, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(APPLE_MBX_GET_PARAM, mbx_drm_get_param, DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_FOPS(mbx_drm_fops);

static const struct drm_driver mbx_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_RENDER,
	.gem_prime_import_sg_table = mbx_gem_import_sg_table,
	.ioctls = mbx_drm_ioctls,
	.num_ioctls = ARRAY_SIZE(mbx_drm_ioctls),
	.fops = &mbx_drm_fops,
	.name = "apple-mbx",
	.desc = "PowerVR MBX Lite (Apple S5L8720)",
	.major = 1,
	.minor = 0,
};

int mbx_drm_register(struct apple_mbx *mbx)
{
	struct mbx_drm *md;
	int ret;

	md = devm_drm_dev_alloc(mbx->dev, &mbx_drm_driver, struct mbx_drm, drm);
	if (IS_ERR(md))
		return PTR_ERR(md);
	md->mbx = mbx;
	ret = drm_dev_register(&md->drm, 0);
	if (ret)
		return ret;
	mbx->drm = &md->drm;
	return 0;
}
