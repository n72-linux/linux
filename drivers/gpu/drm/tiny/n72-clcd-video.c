// SPDX-License-Identifier: GPL-2.0
/*
 * NV12 video overlay plane (layer 2) for the S5L8720 CLCD DRM driver.
 */

#include <linux/math64.h>
#include <linux/soc/apple/n72-lcd.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>

#include "n72-clcd.h"
#include "n72-clcd-coef.h"

/* Filter table selection for a 16.16 source-per-destination ratio */
static void n72_v_pick(u32 q, bool horiz, int *a, int *b)
{
	if ((horiz && q > 0x80000) || (!horiz && q > 0x40000)) {
		*a = 0;
		*b = 0;
	} else if ((horiz && q - 0x40001U < 0x40000) || q - 0x2aaabU <= 0x15555) {
		*a = 1;
		*b = 3;
	} else if (q - 0x20001U <= 0xaaa9) {
		*a = 2;
		*b = 5;
	} else {
		*b = 7;
		if (q - 0x1999aU <= 0x6666)
			*a = 3;
		else if (q - 0x15556U <= 0x4443)
			*a = 4;
		else if (q - 0x12493U <= 0x30c2)
			*a = 5;
		else if (q - 0x10001U <= 0x2491)
			*a = 6;
		else
			*a = 7;
	}
}

static const u32 n72_v_formats[] = { DRM_FORMAT_NV12 };

static int n72_v_check(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *cs;
	struct drm_framebuffer *fb = ps->fb;
	int ret;

	if (!fb || !ps->crtc)
		return 0;
	cs = drm_atomic_get_new_crtc_state(state, ps->crtc);
	/* down by 8 across and 4 down at most (the vertical limit for both) */
	ret = drm_atomic_helper_check_plane_state(ps, cs, 1, 4 << 16, true, true);
	if (ret || !ps->visible)
		return ret;
	if (fb->pitches[0] % 16 || fb->pitches[1] != fb->pitches[0] ||
	    fb->pitches[0] >= 1 << 15)
		return -EINVAL;
	if ((ps->src.x2 - ps->src.x1) >> 16 > N72_V_MAX_W ||
	    (ps->src.y2 - ps->src.y1) >> 16 > N72_V_MAX_H ||
	    (ps->src.x2 - ps->src.x1) >> 16 < 17 || (ps->src.y2 - ps->src.y1) >> 16 < 17)
		return -EINVAL;
	return 0;
}

/* Program the video layer for @ps (visible). reg_lock held, registers reachable. */
void n72_v_write(struct n72_layer *l, struct drm_plane_state *ps)
{
	struct n72_clcd *p = l->clcd;
	void __iomem *r = p->regs;
	struct drm_framebuffer *fb = ps->fb;
	u32 pitch = fb->pitches[0];
	u32 sx = ps->src.x1 >> 16, sy = ps->src.y1 >> 16;
	u32 sw = (ps->src.x2 - ps->src.x1) >> 16, sh = (ps->src.y2 - ps->src.y1) >> 16;
	u32 dx = ps->dst.x1, dy = ps->dst.y1;
	u32 dw = drm_rect_width(&ps->dst), dh = drm_rect_height(&ps->dst);
	/* the first line of the 16x16 block the source starts in; the rest by the origin */
	dma_addr_t y = drm_fb_dma_get_gem_obj(fb, 0)->dma_addr + fb->offsets[0] +
		       (sy & ~15) * pitch + (sx & ~15);
	dma_addr_t c = drm_fb_dma_get_gem_obj(fb, 1)->dma_addr + fb->offsets[1] +
		       (sy & ~15) / 2 * pitch + (sx & ~15);
	bool turned = ps->rotation & (DRM_MODE_ROTATE_90 | DRM_MODE_ROTATE_270);
	bool bt709 = ps->color_encoding == DRM_COLOR_YCBCR_BT709;
	bool full = ps->color_range == DRM_COLOR_YCBCR_FULL_RANGE;
	/* turned, the source's height runs across the panel, its width down */
	u32 qh = div_u64((u64)(turned ? ps->src.y2 - ps->src.y1 : ps->src.x2 - ps->src.x1), dw);
	u32 qv = div_u64((u64)(turned ? ps->src.x2 - ps->src.x1 : ps->src.y2 - ps->src.y1), dh);
	int t[4], i;
	static const u32 csc[2][2][9] = {
		{	/* BT.601: limited ('420v'), full ('420f') */
			{ 0x4a8, 0, 0x662, 0x4a8, 0x1191, 0x1340, 0x4a8, 0x812, 0 },
			{ 0x400, 0, 0x596, 0x400, 0x115f, 0x12d8, 0x400, 0x70f, 0 },
		},
		{	/* BT.709: limited ('420v'), full ('420f') */
			{ 0x4a8, 0, 0x72c, 0x4a8, 0x10da, 0x1222, 0x4a8, 0x873, 0 },
			{ 0x400, 0, 0x631, 0x400, 0x10c0, 0x11df, 0x400, 0x76c, 0 },
		},
	};
	const u32 *m = csc[bt709][full];
	u32 en;

	writel(!turned ? 0 : ps->rotation & DRM_MODE_ROTATE_90 ? CLCD_V_ROT90 : CLCD_V_ROT270,
	       r + CLCD_V_CONFIG);
	writel(pitch << 17 | 0x50, r + CLCD_V_PITCH_Y);
	writel(pitch << 17 | 0x50, r + CLCD_V_PITCH_C);
	writel(sh | sw << 16, r + CLCD_V_SRCSIZE);
	writel((sy & 15) | (sx & 15) << 16, r + CLCD_V_ORIGIN);
	writel(lower_32_bits(y), r + CLCD_V_LUMA);
	writel(lower_32_bits(c), r + CLCD_V_CHROMA);
	writel(lower_32_bits(y + pitch), r + CLCD_V_LUMA1);
	writel(lower_32_bits(c + pitch), r + CLCD_V_CHROMA1);
	writel(dy | dx << 16, r + CLCD_V_POS);
	writel(dh | dw << 16, r + CLCD_V_SIZE);
	for (i = 0; i < 9; i++)
		writel(m[i], r + CLCD_V_MATRIX + 4 * i);
	writel(qh, r + CLCD_V_RATIO_H);
	writel(qv, r + CLCD_V_RATIO_V);
	n72_v_pick(qh, true, &t[0], &t[1]);
	n72_v_pick(qv, false, &t[2], &t[3]);
	if (memcmp(t, l->vtab, sizeof(t))) {
		for (i = 0; i < 36; i++)
			writel(n72_v_h8[t[0]][i], r + CLCD_V_H8 + 4 * i);
		for (i = 0; i < 18; i++) {
			writel(n72_v_t4[t[1]][i], r + CLCD_V_T4_H + 4 * i);
			writel(n72_v_t4[t[2]][i], r + CLCD_V_T4_V + 4 * i);
			writel(n72_v_t4[t[3]][i], r + CLCD_V_T4_V2 + 4 * i);
		}
		memcpy(l->vtab, t, sizeof(t));
	}
	if (!p->video_on) {
		writel(readl(r + CLCD_LAYER_MODE) | CLCD_V_MODE, r + CLCD_LAYER_MODE);
		p->video_on = true;
	}
	en = (readl(r + CLCD_LAYER_EN) & ~CLCD_V_LIMITED) | CLCD_V_EN;
	if (!full)
		en |= CLCD_V_LIMITED;
	writel(en, r + CLCD_LAYER_EN);
}

/* reg_lock held, registers reachable */
void n72_v_off(struct n72_clcd *p)
{
	if (!p->video_on)
		return;
	writel(readl(p->regs + CLCD_LAYER_EN) & ~(CLCD_V_EN | CLCD_V_LIMITED),
	       p->regs + CLCD_LAYER_EN);
	writel(readl(p->regs + CLCD_LAYER_MODE) & ~CLCD_V_MODE, p->regs + CLCD_LAYER_MODE);
	p->video_on = false;
}

static void n72_v_update(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct n72_layer *l = to_layer(plane->dev);
	struct n72_clcd *p = l->clcd;
	struct drm_plane_state *ps = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc *crtc = &l->pipe.crtc;
	struct drm_pending_vblank_event *ev = crtc->state->event;
	unsigned long flags;
	int skip = 0;

	spin_lock_irqsave(&p->reg_lock, flags);
	if (n72_regs_ok(p)) {
		if (n72_scanning(p))
			skip = n72_line(p) >= N72_H - 6;
		if (ps->visible && ps->fb)
			n72_v_write(l, ps);
		else
			n72_v_off(p);
	} else if (READ_ONCE(p->blanked)) {
		l->scanout_skipped = true;	/* put up again at N72_LCD_ON */
	}
	spin_unlock_irqrestore(&p->reg_lock, flags);
	n72_mie_sync(p);
	/* a flip of this plane alone: the primary's update has not sent it */
	if (ev && crtc->state->active) {
		crtc->state->event = NULL;
		n72_arm_event(l, ev, skip);
	}
}

static void n72_v_disable(struct drm_plane *plane, struct drm_atomic_state *state)
{
	struct n72_layer *l = to_layer(plane->dev);
	struct n72_clcd *p = l->clcd;
	unsigned long flags;

	spin_lock_irqsave(&p->reg_lock, flags);
	if (n72_regs_ok(p))
		n72_v_off(p);
	spin_unlock_irqrestore(&p->reg_lock, flags);
	n72_mie_sync(p);
}

static const struct drm_plane_helper_funcs n72_v_helper_funcs = {
	.atomic_check = n72_v_check,
	.atomic_update = n72_v_update,
	.atomic_disable = n72_v_disable,
};

static const struct drm_plane_funcs n72_v_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

int n72_v_plane_init(struct n72_layer *l)
{
	struct drm_device *drm = &l->drm;
	int ret;

	/* the video layer, over this layer and under the launcher's */
	memset(l->vtab, 0xff, sizeof(l->vtab));
	drm_plane_helper_add(&l->video, &n72_v_helper_funcs);
	ret = drm_universal_plane_init(drm, &l->video, drm_crtc_mask(&l->pipe.crtc),
				       &n72_v_funcs, n72_v_formats,
				       ARRAY_SIZE(n72_v_formats), NULL,
				       DRM_PLANE_TYPE_OVERLAY, "video");
	if (ret)
		return ret;
	/*
	 * Hardware rotates the picture on readback via CONFIG codes 1/3,
	 * keeping panel size and scale ratios landscape-oriented.
	 */
	ret = drm_plane_create_rotation_property(&l->video, DRM_MODE_ROTATE_0,
						 DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
						 DRM_MODE_ROTATE_270);
	if (ret)
		return ret;
	return drm_plane_create_color_properties(&l->video,
						 BIT(DRM_COLOR_YCBCR_BT601) |
						 BIT(DRM_COLOR_YCBCR_BT709),
						 BIT(DRM_COLOR_YCBCR_LIMITED_RANGE) |
						 BIT(DRM_COLOR_YCBCR_FULL_RANGE),
						 DRM_COLOR_YCBCR_BT601,
						 DRM_COLOR_YCBCR_LIMITED_RANGE);
}
