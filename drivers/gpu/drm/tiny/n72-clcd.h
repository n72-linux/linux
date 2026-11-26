/* SPDX-License-Identifier: GPL-2.0-only */
/* Internal definitions for the S5L8720 N72 CLCD DRM driver. */
#ifndef N72_CLCD_H
#define N72_CLCD_H

#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_plane.h>
#include <drm/drm_simple_kms_helper.h>

#define CLCD_LAYER_EN	0x0004
#define CLCD_IRQ_EN	0x0008
#define CLCD_IRQ_ST	0x000c
#define CLCD_LAYER_MODE	0x0010
#define  CLCD_MODE_L0	0x010	/* layer 0 active */
#define CLCD_VIDINT_CTL	0x1b28
#define CLCD_VIDINT_PEND 0x1b2c
#define CLCD_VIDINT_FRAME 4
#define  CLCD_IRQ_FRAME	BIT(0)
#define CLCD_L(n)	(0x0020 + 0x20 * (n))
#define  CLCD_L_CFG	0x00
#define  CLCD_L_BASE	0x04
#define  CLCD_L_LINE	0x08
#define  CLCD_L_ORIGIN	0x0c
#define  CLCD_L_SIZE	0x10
#define  CLCD_L_POS	0x14
#define  CLCD_L_1C	0x1c
/* the video layer (layer 2) */
#define CLCD_V_EN	(BIT(3) | BIT(8))	/* in 0x004 */
#define CLCD_V_MODE	0x002			/* in 0x010 */
#define CLCD_V_CONFIG	0x118			/* the turn: 1 = 90 (counter-clockwise), 3 = 270 */
#define  CLCD_V_ROT90	1
#define  CLCD_V_ROT270	3
#define CLCD_V_LUMA	0x11c
#define CLCD_V_CHROMA	0x120
#define CLCD_V_LUMA1	0x124			/* + one line */
#define CLCD_V_CHROMA1	0x128
#define CLCD_V_ORIGIN	0x130			/* y & 15 | (x & 15) << 16 */
#define CLCD_V_SRCSIZE	0x134			/* height | width << 16 */
#define CLCD_V_POS	0x138			/* y | x << 16 on the panel */
#define CLCD_V_SIZE	0x13c			/* height | width << 16 */
#define CLCD_V_MATRIX	0x070			/* 9 words: rows R G B of Y Cb Cr */
#define CLCD_V_H8	0x140			/* 8-tap tables: 36 words */
#define CLCD_V_T4_V	0x1d0			/* 4-tap tables: 18 words each */
#define CLCD_V_T4_H	0x220
#define CLCD_V_T4_V2	0x270
#define CLCD_V_RATIO_H	0x2c0			/* 16.16 source per destination */
#define CLCD_V_RATIO_V	0x2c4
#define CLCD_V_PITCH_Y	0x2e0			/* pitch << 17 | 0x50 */
#define CLCD_V_PITCH_C	0x2e4
#define N72_V_MAX_W	720
#define N72_V_MAX_H	640
#define CLCD_VIDCON0	0x1b10
#define CLCD_VIDCON1	0x1b14

#define N72_W		320
#define N72_H		480
#define N72_FRAME_NS	16666667
#define N72_LAYERS	2
#define CLCD_SHELL_CFG	(0x200700 | 0x110000)
#define CLCD_SHELL_MODE	0x300

#define n72_trace(p, what, ...)							\
	dev_dbg((p)->dev, "vbl: " what " (users %u%s%s)\n",			\
		##__VA_ARGS__, (p)->vbl_users,					\
		READ_ONCE((p)->suspended) ? " suspended" : "",			\
		READ_ONCE((p)->blanked) ? " blanked" : "")

struct device;
struct drm_pending_vblank_event;
struct workqueue_struct;

/* One RGB layer: a DRM device with one CRTC, one plane and one connector. */
struct n72_layer {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector conn;
	struct n72_clcd *clcd;
	unsigned int index;
	bool vbl_on;			/* this device has vblanks enabled */
	bool scanout_skipped;		/* a buffer committed while blanked */
	u32 boot_mode;			/* layer 1: register 0x10 as it was while off */
	bool mode_saved;
	struct drm_plane video;		/* layer 0's device: the video layer */
	int vtab[4];			/* the filter tables loaded (-1: none) */
	/* a flip's completion event, sent at the vblank that shows it */
	spinlock_t ev_lock;
	struct drm_pending_vblank_event *event;
	int event_skip;			/* vblanks to let pass first */
};

struct n72_clcd {
	struct device *dev;
	void __iomem *regs;
	struct n72_layer *layer[N72_LAYERS];
	struct workqueue_struct *commit_wq;

	struct hrtimer vbl_timer;
	unsigned int vbl_users;		/* layers with vblanks enabled; reg_lock */
	int irq;			/* >= 0: frame interrupt in use */
	ktime_t last_irq;		/* the watchdog's beat starts when these stop */
	/*
	 * Set across system sleep: the CLCD's clock may be gated then, and
	 * reading a gated block hangs the bus, so the timer must not touch
	 * the registers (it is also cancelled).
	 */
	bool suspended;
	/*
	 * Set while n72-lcd has the display off and its clocks gated (the
	 * system runs): the same rule. Register accesses outside the timer
	 * and the interrupt take reg_lock and check both flags, so once
	 * blanked is set under it, no new access starts.
	 */
	bool blanked;
	bool video_on;			/* the video layer is enabled; reg_lock */
	spinlock_t reg_lock;
	struct notifier_block lcd_nb;
};

static inline struct n72_layer *to_layer(struct drm_device *drm)
{
	return container_of(drm, struct n72_layer, drm);
}

static inline bool n72_regs_ok(struct n72_clcd *p)
{
	return !READ_ONCE(p->suspended) && !READ_ONCE(p->blanked);
}

static inline u32 n72_line(struct n72_clcd *p)
{
	return (readl(p->regs + CLCD_VIDCON1) >> 9) & 0x1ff;
}

static inline bool n72_scanning(struct n72_clcd *p)
{
	return readl(p->regs + CLCD_VIDCON0) & 1;
}

void n72_arm_event(struct n72_layer *l, struct drm_pending_vblank_event *event,
		   int skip);
int n72_v_plane_init(struct n72_layer *l);
void n72_v_write(struct n72_layer *l, struct drm_plane_state *ps);
void n72_v_off(struct n72_clcd *p);

#endif
