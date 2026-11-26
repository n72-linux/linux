// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM/KMS for the S5L8720 display controller (CLCD) of the iPod touch 2G
 * (N72). The boot loader -- or n72-lcd after system sleep -- brings up the
 * CLCD, the MIPI DSI link and the panel; this driver scans out GEM DMA
 * buffers (dumb buffers, or PRIME-imported ones such as the MBX GPU's
 * contiguous buffers) and provides page flips with vblank events and an
 * fbdev console.
 *
 * The controller has two RGB layers and a video layer:
 *
 *   n72-clcd        layer 0, XRGB8888: what applications draw on, with the
 *                   fbdev console;
 *   n72-clcd-shell  layer 1, ARGB8888, blended over layer 0 with its own
 *                   alpha: for the launcher, which shows its status bar,
 *                   panels and pop-ups over whatever application has
 *                   layer 0.
 *
 * The video layer (layer 2: NV12, scaled and colour-converted by the
 * controller, between layer 0 and layer 1) is an overlay plane of the
 * first device, so that the application that has layer 0 can show video
 * on it.
 *
 * Two devices and not two planes of one, because a KMS device has one
 * master: this way an application is the plain master of the first device
 * and the launcher of the second, and neither has to be handed anything.
 * Both show the same 320x480 mode on the same panel and share the frame
 * interrupt.
 *
 * Registers used:
 *   0x004  layer enables: bit 4 layer 0, bit 5 layer 1 (bit 3 video)
 *   0x010  mode word rebuilt at every swap: 0x10 with layer 0, 0x300 with
 *          layer 1, 2 with video. The boot loader leaves 0x80 there and
 *          layer 0 shows with that; layer 1 needs 0x310 as a whole while on.
 *   0x020 + 0x20 * layer: the layer's block
 *     +0x00  configuration: 0x200700 for 32 bits a pixel; layer 1 uses
 *            0x110000 on top (CLCD_SHELL_CFG).
 *     +0x04  scan-out address, latched by the controller once per frame,
 *            at the end of the active lines
 *     +0x08  line length in pixels
 *     +0x0c  origin in the buffer, y | x << 16
 *     +0x10  size, height | width << 16
 *     +0x14  position on the panel, y | x << 16
 *     +0x1c  zeroed during swap
 *   0x1b10 VIDCON0, bit 0: scan-out on
 *   0x1b14 VIDCON1, bits 17:9: line being scanned (0-479); drops to 0 when
 *          the active lines end and a pending scan-out address latches
 *
 * Layer 1 lies on top and blends with premultiplied alpha: out = layer 1 +
 * (1 - alpha) * layer 0 (matching DRM's ARGB8888 convention).
 *
 * Vblank: frame interrupt on VIC0 line 13:
 *   0x1b28 video interrupt control: 4 latches the frame event
 *   0x1b2c video interrupt pending: write 0xf before 0x0c so 0x0c does not relatch
 *   0x0c   status (write 1 to clear): bit 0 frame (~16.667 ms), 0x1700 underruns
 *   0x08   enable, same bits
 * A 20 Hz safety timer fires when scan-out is stopped (blanked panel).
 */

#include <linux/aperture.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/soc/apple/n72-lcd.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "n72-clcd.h"

/* ---------------------------------------------------------------- vblank */

/* A vblank on one layer: count it and deliver a flip event that is due. */
static void n72_layer_vblank(struct n72_layer *l, bool events)
{
	struct drm_crtc *crtc = &l->pipe.crtc;
	struct drm_pending_vblank_event *ev = NULL;
	unsigned long flags;

	if (!READ_ONCE(l->vbl_on))
		return;
	drm_crtc_handle_vblank(crtc);
	if (!events)
		return;
	spin_lock_irqsave(&l->ev_lock, flags);
	if (l->event) {
		if (l->event_skip) {
			l->event_skip--;
		} else {
			ev = l->event;
			l->event = NULL;
		}
	}
	spin_unlock_irqrestore(&l->ev_lock, flags);
	if (ev) {
		spin_lock_irqsave(&l->drm.event_lock, flags);
		drm_crtc_send_vblank_event(crtc, ev);
		spin_unlock_irqrestore(&l->drm.event_lock, flags);
		drm_crtc_vblank_put(crtc);
	}
}

/* The one vblank of the panel, for every layer that listens. */
static void n72_vblank(struct n72_clcd *p, bool events)
{
	int i;

	for (i = 0; i < N72_LAYERS; i++)
		if (p->layer[i])
			n72_layer_vblank(p->layer[i], events);
}

static irqreturn_t n72_irq(int irq, void *data)
{
	struct n72_clcd *p = data;
	u32 st = readl(p->regs + CLCD_IRQ_ST);

	if (!st) {
		/* a pending bit with nothing in the status must not hold the line */
		writel(0xf, p->regs + CLCD_VIDINT_PEND);
		return IRQ_NONE;
	}
	writel(0xf, p->regs + CLCD_VIDINT_PEND);
	writel(st, p->regs + CLCD_IRQ_ST);
	if (st & CLCD_IRQ_FRAME) {
		WRITE_ONCE(p->last_irq, ktime_get());
		n72_vblank(p, true);
	}
	if (st & 0x1700)
		dev_warn_ratelimited(p->dev, "underrun %08x\n", st);
	return IRQ_HANDLED;
}

static enum hrtimer_restart n72_vbl_fn(struct hrtimer *t)
{
	struct n72_clcd *p = container_of(t, struct n72_clcd, vbl_timer);

	/* Keep a 20 Hz safety beat when scan-out stops (e.g. panel blanked). */
	if (ktime_ms_delta(ktime_get(), READ_ONCE(p->last_irq)) > 40)
		n72_vblank(p, true);
	hrtimer_forward_now(t, ns_to_ktime(3 * N72_FRAME_NS));
	return HRTIMER_RESTART;
}

/* Frame interrupt source on, pending and status cleared, enable */
static void n72_frame_irq_on(struct n72_clcd *p)
{
	writel(CLCD_VIDINT_FRAME, p->regs + CLCD_VIDINT_CTL);
	writel(0xf, p->regs + CLCD_VIDINT_PEND);
	writel(CLCD_IRQ_FRAME, p->regs + CLCD_IRQ_ST);
	writel(readl(p->regs + CLCD_IRQ_EN) | CLCD_IRQ_FRAME, p->regs + CLCD_IRQ_EN);
	WRITE_ONCE(p->last_irq, ktime_get());
}

/* The interrupt and the timer run while any layer wants vblanks. */
static int n72_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct n72_layer *l = to_layer(pipe->crtc.dev);
	struct n72_clcd *p = l->clcd;
	unsigned long flags;
	bool first;

	spin_lock_irqsave(&p->reg_lock, flags);
	WRITE_ONCE(l->vbl_on, true);
	first = !p->vbl_users++;
	if (first && n72_regs_ok(p))
		n72_frame_irq_on(p);
	n72_trace(p, "enable layer %u%s", l->index, first ? " first" : "");
	spin_unlock_irqrestore(&p->reg_lock, flags);
	if (first)
		hrtimer_start(&p->vbl_timer, ns_to_ktime(1000000), HRTIMER_MODE_REL);
	return 0;
}

static void n72_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct n72_layer *l = to_layer(pipe->crtc.dev);
	struct n72_clcd *p = l->clcd;
	unsigned long flags;
	bool last;

	spin_lock_irqsave(&p->reg_lock, flags);
	WRITE_ONCE(l->vbl_on, false);
	last = !--p->vbl_users;
	if (last && n72_regs_ok(p))
		writel(readl(p->regs + CLCD_IRQ_EN) & ~CLCD_IRQ_FRAME, p->regs + CLCD_IRQ_EN);
	n72_trace(p, "disable layer %u%s", l->index, last ? " last" : "");
	spin_unlock_irqrestore(&p->reg_lock, flags);
	if (last)
		hrtimer_cancel(&p->vbl_timer);
}

/* ------------------------------------------------------------ the pipes */

static enum drm_mode_status n72_mode_valid(struct drm_simple_display_pipe *pipe,
					   const struct drm_display_mode *mode)
{
	return mode->hdisplay == N72_W && mode->vdisplay == N72_H ? MODE_OK : MODE_BAD;
}

static int n72_check(struct drm_simple_display_pipe *pipe,
		     struct drm_plane_state *plane_state,
		     struct drm_crtc_state *crtc_state)
{
	struct drm_framebuffer *fb = plane_state->fb;

	bool turned = plane_state->rotation & (DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270);

	if (!fb)
		return 0;
	/* turned, the picture is landscape: the layer turns it onto the panel */
	if (fb->width < (turned ? N72_H : N72_W) || fb->height < (turned ? N72_W : N72_H) ||
	    fb->pitches[0] % 4 || plane_state->src_x || plane_state->src_y)
		return -EINVAL;
	return 0;
}

/*
 * Layer rotation code in config bits 23:22, with layer size matching the
 * framebuffer orientation; code 1 is DRM's 90 degrees counter-clockwise.
 */
static u32 n72_rot_code(unsigned int rotation)
{
	return rotation & DRM_MODE_ROTATE_90 ? 1 : rotation & DRM_MODE_ROTATE_270 ? 3 : 0;
}

static int n72_shell_plane_check(struct drm_plane *plane,
				 struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = ps->fb;
	struct drm_crtc_state *cs;
	bool turned = ps->rotation & (DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270);
	u32 sx, sy, sw, sh;
	int dx, dy, dw, dh, ret;

	if (!ps->crtc)
		return 0;
	cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	ret = drm_atomic_helper_check_plane_state(ps, cs, DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  true, false);
	if (ret || !ps->visible || !fb)
		return ret;
	if ((ps->src.x1 | ps->src.y1 | ps->src.x2 | ps->src.y2) & 0xffff ||
	    fb->pitches[0] % 4)
		return -EINVAL;
	sx = ps->src.x1 >> 16;
	sy = ps->src.y1 >> 16;
	sw = drm_rect_width(&ps->src) >> 16;
	sh = drm_rect_height(&ps->src) >> 16;
	dx = ps->dst.x1;
	dy = ps->dst.y1;
	dw = drm_rect_width(&ps->dst);
	dh = drm_rect_height(&ps->dst);
	if (turned) {
		if (dx != 0 || dy != 0 || dw != N72_W || dh != N72_H ||
		    sx != 0 || sy != 0 || fb->width < N72_H || fb->height < N72_W)
			return -EINVAL;
		return 0;
	}
	if (dx < 0 || dy < 0 || dx + dw > N72_W || dy + dh > N72_H ||
	    dw < 16 || dh < 1 || dw != sw || dh != sh ||
	    (sx % 4) != 0 || sx + sw > fb->width || sy + sh > fb->height)
		return -EINVAL;
	return 0;
}

/* reg_lock held, registers reachable */
static void n72_write_fb(struct n72_layer *l, struct drm_plane_state *ps, bool force)
{
	void __iomem *blk = l->clcd->regs + CLCD_L(l->index);
	u32 code = n72_rot_code(ps->rotation);
	u32 cfg = readl(blk + CLCD_L_CFG);

	if (!l->index) {
		if ((cfg >> 22 & 3) != code) {
			writel((cfg & ~(3U << 22)) | code << 22, blk + CLCD_L_CFG);
			writel(code ? N72_W | N72_H << 16 : N72_H | N72_W << 16,
			       blk + CLCD_L_SIZE);
		}
		writel(ps->fb->pitches[0] / 4, blk + CLCD_L_LINE);
		writel(lower_32_bits(drm_fb_dma_get_gem_addr(ps->fb, ps, 0)),
		       blk + CLCD_L_BASE);
		return;
	}

	{
		u32 sx = ps->src.x1 >> 16, sy = ps->src.y1 >> 16;
		u32 sw = drm_rect_width(&ps->src) >> 16;
		u32 sh = drm_rect_height(&ps->src) >> 16;
		u32 dx = ps->dst.x1, dy = ps->dst.y1;
		u32 size, pos;
		dma_addr_t addr;

		if (!sw || !sh) {
			dx = 0;
			dy = 0;
			sx = 0;
			sy = 0;
			sw = code ? N72_H : N72_W;
			sh = code ? N72_W : N72_H;
		}
		addr = drm_fb_dma_get_gem_obj(ps->fb, 0)->dma_addr +
		       ps->fb->offsets[0] + (dma_addr_t)sy * ps->fb->pitches[0] +
		       (dma_addr_t)sx * 4;
		size = sh | (sw << 16);
		pos = dy | (dx << 16);
		if ((cfg >> 22 & 3) != code)
			writel((cfg & ~(3U << 22)) | code << 22, blk + CLCD_L_CFG);
		if (force || readl(blk + CLCD_L_SIZE) != size)
			writel(size, blk + CLCD_L_SIZE);
		if (force || readl(blk + CLCD_L_POS) != pos)
			writel(pos, blk + CLCD_L_POS);
		if (force)
			writel(0, blk + CLCD_L_ORIGIN);
		writel(ps->fb->pitches[0] / 4, blk + CLCD_L_LINE);
		writel(lower_32_bits(addr), blk + CLCD_L_BASE);
	}
}

/*
 * Layer 1 on or off as a whole, the way iOS programs it at a swap: the
 * block, then its bits in 0x10, then its enable. Layer 0 is the boot
 * loader's and stays on. reg_lock held, registers reachable.
 */
static void n72_shell_set(struct n72_layer *l, struct drm_plane_state *ps)
{
	struct n72_clcd *p = l->clcd;
	void __iomem *blk = p->regs + CLCD_L(1);
	u32 mode = readl(p->regs + CLCD_LAYER_MODE);
	u32 en = readl(p->regs + CLCD_LAYER_EN) & ~BIT(5);

	if (ps && ps->fb) {
		if (!l->mode_saved) {
			l->boot_mode = mode;
			l->mode_saved = true;
		}
		writel(CLCD_SHELL_CFG, blk + CLCD_L_CFG);
		writel(0, blk + CLCD_L_1C);
		n72_write_fb(l, ps, true);	/* (and turn + active window, if any) */
		/*
		 * Rebuild the full 0x10 word with layer 1's bits (preserving
		 * bootloader bits beside layer 1 hides the layer).
		 */
		mode = CLCD_MODE_L0 | CLCD_SHELL_MODE;
		en |= BIT(5);
	} else if (l->mode_saved) {
		mode = l->boot_mode;
		l->mode_saved = false;
	}
	if (p->video_on)		/* the whole word is rebuilt: keep video's bit */
		mode |= CLCD_V_MODE;
	writel(mode, p->regs + CLCD_LAYER_MODE);
	writel(en, p->regs + CLCD_LAYER_EN);
}

/* Point the layer at @ps's buffer, and arm @event for the vblank that shows it. */
static void n72_scanout(struct n72_layer *l, struct drm_plane_state *ps,
			struct drm_pending_vblank_event *event, bool enable)
{
	struct n72_clcd *p = l->clcd;
	unsigned long flags;
	u32 line = 0;
	int skip = 0;

	if (ps->fb) {
		/*
		 * Written in the last few lines, the address may miss this
		 * frame's latch: then it shows a frame later, and so must the
		 * event (an early event lets user space draw into a buffer
		 * still on screen, which flickers). Display off: nothing to
		 * write now, the buffer goes up when it comes back (n72_lcd_on).
		 */
		spin_lock_irqsave(&p->reg_lock, flags);
		if (n72_regs_ok(p)) {
			if (n72_scanning(p)) {
				line = n72_line(p);
				skip = line >= N72_H - 6;
			}
			if (l->index && enable)
				n72_shell_set(l, ps);
			else
				n72_write_fb(l, ps, false);
		} else if (READ_ONCE(p->blanked)) {
			l->scanout_skipped = true;
		}
		spin_unlock_irqrestore(&p->reg_lock, flags);
	}
	n72_arm_event(l, event, skip);
}

/* Send @event at the vblank that shows what was just written (+@skip). */
void n72_arm_event(struct n72_layer *l, struct drm_pending_vblank_event *event,
		   int skip)
{
	struct drm_crtc *crtc = &l->pipe.crtc;
	unsigned long flags;

	if (!event)
		return;
	if (drm_crtc_vblank_get(crtc)) {
		spin_lock_irqsave(&l->drm.event_lock, flags);
		drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irqrestore(&l->drm.event_lock, flags);
		return;
	}
	spin_lock_irqsave(&l->ev_lock, flags);
	if (l->event) {			/* (not expected: one flip per vblank) */
		struct drm_pending_vblank_event *old = l->event;

		l->event = NULL;
		spin_unlock_irqrestore(&l->ev_lock, flags);
		spin_lock_irqsave(&l->drm.event_lock, flags);
		drm_crtc_send_vblank_event(crtc, old);
		spin_unlock_irqrestore(&l->drm.event_lock, flags);
		drm_crtc_vblank_put(crtc);
		spin_lock_irqsave(&l->ev_lock, flags);
	}
	l->event = event;
	l->event_skip = skip;
	spin_unlock_irqrestore(&l->ev_lock, flags);
}

static void n72_enable(struct drm_simple_display_pipe *pipe,
		       struct drm_crtc_state *crtc_state,
		       struct drm_plane_state *plane_state)
{
	struct n72_layer *l = to_layer(pipe->crtc.dev);
	struct drm_pending_vblank_event *ev = crtc_state->event;

	drm_crtc_vblank_on(&pipe->crtc);
	crtc_state->event = NULL;
	n72_scanout(l, plane_state, ev, true);
}

static void n72_disable(struct drm_simple_display_pipe *pipe)
{
	struct n72_layer *l = to_layer(pipe->crtc.dev);
	struct n72_clcd *p = l->clcd;
	unsigned long flags;

	/*
	 * Layer 0 and the panel stay as the boot loader set them up (n72-lcd
	 * owns the panel); layer 1 goes off. If the display is off, n72-lcd's
	 * saved registers still have it on: n72_lcd_event() sees to that.
	 */
	if (l->index) {
		spin_lock_irqsave(&p->reg_lock, flags);
		if (n72_regs_ok(p))
			n72_shell_set(l, NULL);
		spin_unlock_irqrestore(&p->reg_lock, flags);
	}
	drm_crtc_vblank_off(&pipe->crtc);
}

static void n72_update(struct drm_simple_display_pipe *pipe,
		       struct drm_plane_state *old_state)
{
	struct n72_layer *l = to_layer(pipe->crtc.dev);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_pending_vblank_event *ev = crtc->state->event;

	crtc->state->event = NULL;
	if (!crtc->state->active) {
		if (ev) {
			spin_lock_irq(&crtc->dev->event_lock);
			drm_crtc_send_vblank_event(crtc, ev);
			spin_unlock_irq(&crtc->dev->event_lock);
		}
		return;
	}
	n72_scanout(l, pipe->plane.state, ev, false);
}

static void n72_shell_plane_update(struct drm_plane *plane,
				   struct drm_atomic_state *state)
{
	struct n72_layer *l = to_layer(plane->dev);
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(state, plane);

	n72_update(&l->pipe, old_state);
}

static const struct drm_plane_helper_funcs n72_shell_plane_helper_funcs = {
	.prepare_fb = drm_gem_plane_helper_prepare_fb,
	.atomic_check = n72_shell_plane_check,
	.atomic_update = n72_shell_plane_update,
};

static const struct drm_simple_display_pipe_funcs n72_pipe_funcs = {
	.mode_valid = n72_mode_valid,
	.check = n72_check,
	.enable = n72_enable,
	.disable = n72_disable,
	.update = n72_update,
	.enable_vblank = n72_enable_vblank,
	.disable_vblank = n72_disable_vblank,
};

static const u32 n72_formats[] = { DRM_FORMAT_XRGB8888 };
/* blended with its alpha: nothing without one */
static const u32 n72_shell_formats[] = { DRM_FORMAT_ARGB8888 };

/* ------------------------------------------------------ the connectors */

/* 320x480 at 60 Hz; the porches are the boot loader's, not programmed here */
static const struct drm_display_mode n72_mode = {
	DRM_MODE_INIT(60, N72_W, N72_H, 50, 75),	/* panel ~50 x 75 mm */
};

static int n72_get_modes(struct drm_connector *conn)
{
	return drm_connector_helper_get_modes_fixed(conn, &n72_mode);
}

static const struct drm_connector_helper_funcs n72_conn_helper_funcs = {
	.get_modes = n72_get_modes,
};

static const struct drm_connector_funcs n72_conn_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void n72_commit_tail(struct drm_atomic_state *old_state)
{
	drm_atomic_helper_wait_for_fences(old_state->dev, old_state, false);
	drm_atomic_helper_wait_for_dependencies(old_state);
	drm_atomic_helper_commit_tail(old_state);
	drm_atomic_helper_commit_cleanup_done(old_state);
	drm_atomic_state_put(old_state);
}

static void n72_commit_work(struct work_struct *work)
{
	n72_commit_tail(container_of(work, struct drm_atomic_state, commit_work));
}

static int n72_atomic_commit(struct drm_device *dev, struct drm_atomic_state *state,
			     bool nonblock)
{
	struct n72_clcd *p = to_layer(dev)->clcd;
	int ret;

	ret = drm_atomic_helper_setup_commit(state, nonblock);
	if (ret)
		return ret;
	INIT_WORK(&state->commit_work, n72_commit_work);
	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		return ret;
	if (!nonblock) {
		ret = drm_atomic_helper_wait_for_fences(dev, state, true);
		if (ret)
			goto err;
	}
	ret = drm_atomic_helper_swap_state(state, true);
	if (ret)
		goto err;
	drm_atomic_state_get(state);
	if (nonblock)
		queue_work(p->commit_wq, &state->commit_work);
	else
		n72_commit_tail(state);
	return 0;
err:
	drm_atomic_helper_unprepare_planes(dev, state);
	return ret;
}

static const struct drm_mode_config_funcs n72_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = n72_atomic_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(n72_fops);

static const struct drm_driver n72_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &n72_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "n72-clcd",
	.desc = "S5L8720 CLCD (iPod touch 2G)",
	.major = 1,
	.minor = 0,
};

/* layer 1: the same, by another name so that it can be told apart */
static const struct drm_driver n72_shell_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &n72_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	.name = "n72-clcd-shell",
	.desc = "S5L8720 CLCD, the layer on top (iPod touch 2G)",
	.major = 1,
	.minor = 0,
};

/* ---------------------------------------------------------------- probe */

static void n72_unmap(void *regs)
{
	iounmap((void __iomem __force *)regs);
}

static void n72_destroy_wq(void *wq)
{
	destroy_workqueue(wq);
}

static int n72_lcd_event(struct notifier_block *nb, unsigned long event, void *data);
static void n72_lcd_unregister(void *nb);

static struct n72_layer *n72_layer_create(struct n72_clcd *p, unsigned int index)
{
	struct n72_layer *l;
	struct drm_device *drm;
	int ret;

	l = devm_drm_dev_alloc(p->dev, index ? &n72_shell_driver : &n72_driver,
			       struct n72_layer, drm);
	if (IS_ERR(l))
		return l;
	drm = &l->drm;
	l->clcd = p;
	l->index = index;
	spin_lock_init(&l->ev_lock);

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ERR_PTR(ret);
	/* (video frames are smaller: n72_check holds the RGB layers to the panel's size) */
	drm->mode_config.min_width = 16;
	drm->mode_config.max_width = 4096;
	drm->mode_config.min_height = 16;
	drm->mode_config.max_height = 4096;
	drm->mode_config.preferred_depth = index ? 32 : 24;
	drm->mode_config.funcs = &n72_mode_config_funcs;

	/* one panel: the second device's connector is not another one */
	drm_connector_helper_add(&l->conn, &n72_conn_helper_funcs);
	ret = drm_connector_init(drm, &l->conn, &n72_conn_funcs,
				 index ? DRM_MODE_CONNECTOR_VIRTUAL : DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ERR_PTR(ret);
	ret = drm_simple_display_pipe_init(drm, &l->pipe, &n72_pipe_funcs,
					   index ? n72_shell_formats : n72_formats, 1,
					   NULL, &l->conn);
	if (ret)
		return ERR_PTR(ret);
	if (index)
		drm_plane_helper_add(&l->pipe.plane, &n72_shell_plane_helper_funcs);
	/* the layer turns a landscape picture onto the panel (n72_write_fb) */
	ret = drm_plane_create_rotation_property(&l->pipe.plane, DRM_MODE_ROTATE_0,
						 DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
						 DRM_MODE_ROTATE_270);
	if (ret)
		return ERR_PTR(ret);
	if (!index) {
		ret = n72_v_plane_init(l);
		if (ret)
			return ERR_PTR(ret);
	}
	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ERR_PTR(ret);
	drm_mode_config_reset(drm);
	return l;
}

static int n72_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct resource fbres;
	struct n72_clcd *p;
	int i, ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;
	spin_lock_init(&p->reg_lock);
	hrtimer_init(&p->vbl_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->vbl_timer.function = n72_vbl_fn;

	np = of_parse_phandle(dev->of_node, "apple,clcd", 0);
	if (!np)
		return dev_err_probe(dev, -ENODEV, "no apple,clcd\n");
	/*
	 * Map the controller shared, never claim it: n72-lcd owns the range
	 * and holds the panel supply (LDO4). Claiming it here made n72-lcd
	 * fail with -EBUSY, and the regulator core then switched the
	 * unclaimed supply off ~30 s after boot: a white panel.
	 */
	p->regs = of_iomap(np, 0);
	of_node_put(np);
	if (!p->regs)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, n72_unmap, (void __force *)p->regs);
	if (ret)
		return ret;

	/* take over from simplefb on the boot framebuffer */
	np = of_parse_phandle(dev->of_node, "apple,framebuffer", 0);
	if (np && !of_address_to_resource(np, 0, &fbres))
		aperture_remove_conflicting_devices(fbres.start, resource_size(&fbres),
						    n72_driver.name);
	of_node_put(np);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	p->commit_wq = alloc_workqueue("n72-clcd", WQ_UNBOUND | WQ_HIGHPRI, 0);
	if (!p->commit_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, n72_destroy_wq, p->commit_wq);
	if (ret)
		return ret;
	for (i = 0; i < N72_LAYERS; i++) {
		struct n72_layer *l = n72_layer_create(p, i);

		if (IS_ERR(l))
			return PTR_ERR(l);
		p->layer[i] = l;
	}
	platform_set_drvdata(pdev, p);

	/* after the layers exist: the handler delivers vblanks to them */
	p->irq = platform_get_irq(pdev, 0);
	if (p->irq < 0)
		return p->irq;
	ret = devm_request_irq(dev, p->irq, n72_irq, 0, "n72-clcd", p);
	if (ret)
		return dev_err_probe(dev, ret, "frame IRQ\n");

	/* after the CRTCs exist: an N72_LCD_ON looks at their state */
	p->lcd_nb.notifier_call = n72_lcd_event;
	p->lcd_nb.priority = 1;		/* before the MBX flips on N72_LCD_ON */
	ret = n72_lcd_register_client(&p->lcd_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, n72_lcd_unregister, &p->lcd_nb);
	if (ret)
		return ret;

	/* the applications' device first: it is the one found first */
	for (i = 0; i < N72_LAYERS; i++) {
		ret = drm_dev_register(&p->layer[i]->drm, 0);
		if (ret) {
			while (i-- > 0)
				drm_dev_unregister(&p->layer[i]->drm);
			return ret;
		}
	}
	drm_client_setup(&p->layer[0]->drm, NULL);
	dev_info(dev, "%ux%u, layer 0 XRGB8888 and layer 1 ARGB8888 on top\n",
		 N72_W, N72_H);
	return 0;
}

/* n72-lcd powers the display down or up while the system runs */
static int n72_lcd_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct n72_clcd *p = container_of(nb, struct n72_clcd, lcd_nb);
	unsigned long flags;
	int i;

	n72_trace(p, "lcd event %s", event == N72_LCD_OFF ? "off" : event == N72_LCD_ON ? "on" :
		  event == N72_LCD_RESUMED ? "resumed" : "other");
	if (event == N72_LCD_OFF) {
		spin_lock_irqsave(&p->reg_lock, flags);
		if (!READ_ONCE(p->suspended))
			writel(readl(p->regs + CLCD_IRQ_EN) & ~CLCD_IRQ_FRAME,
			       p->regs + CLCD_IRQ_EN);
		WRITE_ONCE(p->blanked, true);
		i = p->vbl_users;
		spin_unlock_irqrestore(&p->reg_lock, flags);
		disable_irq(p->irq);	/* and wait for a running handler */
		/* a running timer may have read the flag before: wait for it */
		if (i) {
			hrtimer_cancel(&p->vbl_timer);
			hrtimer_start(&p->vbl_timer, ns_to_ktime(N72_FRAME_NS),
				      HRTIMER_MODE_REL);
		}
		return NOTIFY_OK;
	}
	if (event == N72_LCD_RESUMED) {
		/* a system sleep's bring-up reset the interrupt enables */
		spin_lock_irqsave(&p->reg_lock, flags);
		if (p->vbl_users && n72_regs_ok(p))
			n72_frame_irq_on(p);
		spin_unlock_irqrestore(&p->reg_lock, flags);
		return NOTIFY_OK;
	}
	if (event != N72_LCD_ON)
		return NOTIFY_DONE;

	/*
	 * n72-lcd put the layers back as they were at N72_LCD_OFF (layer 0
	 * may show an MBX flip's buffer): only a buffer committed since
	 * replaces layer 0's. Layer 1 is simply set to what its device has
	 * now, on or off.
	 */
	for (i = 0; i < N72_LAYERS; i++)
		drm_modeset_lock(&p->layer[i]->pipe.plane.mutex, NULL);
	drm_modeset_lock(&p->layer[0]->video.mutex, NULL);
	spin_lock_irqsave(&p->reg_lock, flags);
	WRITE_ONCE(p->blanked, false);
	if (!READ_ONCE(p->suspended)) {
		for (i = 0; i < N72_LAYERS; i++) {
			struct n72_layer *l = p->layer[i];
			struct drm_plane_state *ps = l->pipe.plane.state;
			bool shown = ps && ps->fb && ps->crtc && ps->crtc->state->active;

			if (i)
				n72_shell_set(l, shown ? ps : NULL);
			else if (l->scanout_skipped && shown)
				n72_write_fb(l, ps, false);
			l->scanout_skipped = false;
		}
		{
			/*
			 * The video layer: n72-lcd brought back its enables
			 * (0x04, 0x10) but not its block, which the power
			 * cycle reset. Written again, tables included.
			 */
			struct n72_layer *l = p->layer[0];
			struct drm_plane_state *vs = l->video.state;

			memset(l->vtab, 0xff, sizeof(l->vtab));
			if (vs && vs->fb && vs->visible && vs->crtc && vs->crtc->state->active)
				n72_v_write(l, vs);
			else
				n72_v_off(p);
		}
		if (p->vbl_users)
			n72_frame_irq_on(p);
	}
	spin_unlock_irqrestore(&p->reg_lock, flags);
	drm_modeset_unlock(&p->layer[0]->video.mutex);
	for (i = N72_LAYERS; i-- > 0;)
		drm_modeset_unlock(&p->layer[i]->pipe.plane.mutex);
	enable_irq(p->irq);
	return NOTIFY_OK;
}

static void n72_lcd_unregister(void *nb)
{
	n72_lcd_unregister_client(nb);
}

static void n72_remove(struct platform_device *pdev)
{
	struct n72_clcd *p = platform_get_drvdata(pdev);
	int i;

	for (i = N72_LAYERS; i-- > 0;) {
		drm_dev_unplug(&p->layer[i]->drm);
		drm_atomic_helper_shutdown(&p->layer[i]->drm);
	}
	hrtimer_cancel(&p->vbl_timer);
}

static void n72_shutdown(struct platform_device *pdev)
{
	struct n72_clcd *p = platform_get_drvdata(pdev);
	int i;

	for (i = N72_LAYERS; i-- > 0;)
		drm_atomic_helper_shutdown(&p->layer[i]->drm);
}

static int n72_pm_suspend(struct device *dev)
{
	struct n72_clcd *p = dev_get_drvdata(dev);
	int i, ret = 0, r;

	for (i = N72_LAYERS; i-- > 0;) {
		/* pipe off, vblank off */
		r = drm_mode_config_helper_suspend(&p->layer[i]->drm);
		if (r && !ret)
			ret = r;
	}
	n72_trace(p, "pm suspend");
	WRITE_ONCE(p->suspended, true);
	hrtimer_cancel(&p->vbl_timer);
	return ret;
}

static int n72_pm_resume(struct device *dev)
{
	struct n72_clcd *p = dev_get_drvdata(dev);
	int i, ret = 0, r;

	WRITE_ONCE(p->suspended, false);
	n72_trace(p, "pm resume");
	for (i = 0; i < N72_LAYERS; i++) {
		r = drm_mode_config_helper_resume(&p->layer[i]->drm);
		if (r && !ret)
			ret = r;
	}
	n72_trace(p, "pm resumed");
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(n72_pm_ops, n72_pm_suspend, n72_pm_resume);

static const struct of_device_id n72_of_match[] = {
	{ .compatible = "apple,n72-display" },
	{ }
};
MODULE_DEVICE_TABLE(of, n72_of_match);

static struct platform_driver n72_platform_driver = {
	.probe = n72_probe,
	.remove = n72_remove,
	.shutdown = n72_shutdown,
	.driver = {
		.name = "n72-clcd",
		.of_match_table = n72_of_match,
		.pm = pm_sleep_ptr(&n72_pm_ops),
	},
};
module_platform_driver(n72_platform_driver);

MODULE_DESCRIPTION("DRM/KMS for the S5L8720 CLCD (iPod touch 2G)");
MODULE_LICENSE("GPL");
