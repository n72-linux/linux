// SPDX-License-Identifier: GPL-2.0-only
/*
 * Gamma LUT (CLCD+0x400..0x410) and Mobile Image Enhancement (MIE,
 * CLCD+0x1b30..0x1c0c) for the S5L8720 CLCD on N72.
 */

#include <linux/delay.h>
#include <linux/mfd/apple-n72-pmu.h>
#include <drm/drm_color_mgmt.h>

#include "n72-clcd.h"

/* Constant MIE tables from AppleM2DisplayDrivers / MIEDriver on N72 */
static const u32 n72_mie_engamma[225] = {
	0x00000, 0x00006, 0x0000d, 0x00013, 0x0001a, 0x00020, 0x00027, 0x0002d,
	0x00033, 0x00038, 0x0003d, 0x00042, 0x00047, 0x0004b, 0x0004f, 0x00053,
	0x00057, 0x0005a, 0x0005e, 0x00061, 0x00065, 0x00068, 0x0006b, 0x0006e,
	0x00071, 0x00074, 0x00077, 0x0007a, 0x0007c, 0x0007f, 0x00081, 0x00084,
	0x00087, 0x00089, 0x0008b, 0x0008e, 0x00090, 0x00092, 0x00095, 0x00097,
	0x00099, 0x0009b, 0x0009d, 0x000a0, 0x000a2, 0x000a4, 0x000a6, 0x000a8,
	0x000aa, 0x000ac, 0x000ae, 0x000af, 0x000b1, 0x000b3, 0x000b5, 0x000b7,
	0x000b9, 0x000ba, 0x000bc, 0x000be, 0x000c0, 0x000c1, 0x000c3, 0x000c5,
	0x000c6, 0x000ca, 0x000cd, 0x000d0, 0x000d3, 0x000d6, 0x000d9, 0x000dc,
	0x000df, 0x000e2, 0x000e5, 0x000e8, 0x000eb, 0x000ed, 0x000f0, 0x000f3,
	0x000f5, 0x000f8, 0x000fb, 0x000fd, 0x00100, 0x00102, 0x00105, 0x00107,
	0x00109, 0x0010c, 0x0010e, 0x00110, 0x00113, 0x00115, 0x00117, 0x0011a,
	0x0011c, 0x00120, 0x00124, 0x00129, 0x0012d, 0x00131, 0x00135, 0x00139,
	0x0013d, 0x00141, 0x00145, 0x00148, 0x0014c, 0x00150, 0x00153, 0x00157,
	0x0015a, 0x0015e, 0x00161, 0x00165, 0x00168, 0x0016b, 0x0016f, 0x00172,
	0x00175, 0x00178, 0x0017b, 0x0017e, 0x00182, 0x00185, 0x00188, 0x0018b,
	0x0018e, 0x00193, 0x00199, 0x0019f, 0x001a4, 0x001aa, 0x001af, 0x001b5,
	0x001ba, 0x001bf, 0x001c4, 0x001c9, 0x001ce, 0x001d3, 0x001d8, 0x001dc,
	0x001e1, 0x001e6, 0x001ea, 0x001ef, 0x001f3, 0x001f8, 0x001fc, 0x00201,
	0x00205, 0x00209, 0x0020d, 0x00211, 0x00216, 0x0021a, 0x0021e, 0x00222,
	0x00226, 0x0022d, 0x00235, 0x0023d, 0x00244, 0x0024b, 0x00253, 0x0025a,
	0x00261, 0x00267, 0x0026e, 0x00275, 0x0027c, 0x00282, 0x00288, 0x0028f,
	0x00295, 0x0029b, 0x002a1, 0x002a7, 0x002ad, 0x002b3, 0x002b9, 0x002bf,
	0x002c5, 0x002ca, 0x002d0, 0x002d5, 0x002db, 0x002e0, 0x002e6, 0x002eb,
	0x002f0, 0x002fb, 0x00305, 0x0030f, 0x00319, 0x00323, 0x0032c, 0x00336,
	0x0033f, 0x00348, 0x00351, 0x0035a, 0x00363, 0x0036c, 0x00374, 0x0037d,
	0x00385, 0x0038e, 0x00396, 0x0039e, 0x003a6, 0x003ae, 0x003b5, 0x003bd,
	0x003c5, 0x003cc, 0x003d4, 0x003db, 0x003e3, 0x003ea, 0x003f1, 0x003f8,
	0x003ff,
};

static const u32 n72_mie_degamma[256] = {
	0x00000, 0x00001, 0x00001, 0x00002, 0x00002, 0x00003, 0x00004, 0x00004,
	0x00005, 0x00006, 0x00006, 0x00007, 0x00008, 0x00008, 0x00009, 0x0000a,
	0x0000b, 0x0000b, 0x0000c, 0x0000d, 0x0000e, 0x0000f, 0x00010, 0x00012,
	0x00013, 0x00014, 0x00015, 0x00016, 0x00018, 0x00019, 0x0001b, 0x0001c,
	0x0001e, 0x0001f, 0x00021, 0x00022, 0x00024, 0x00026, 0x00028, 0x0002a,
	0x0002b, 0x0002d, 0x0002f, 0x00031, 0x00034, 0x00036, 0x00038, 0x0003a,
	0x0003d, 0x0003f, 0x00041, 0x00044, 0x00046, 0x00049, 0x0004c, 0x0004e,
	0x00051, 0x00054, 0x00057, 0x0005a, 0x0005c, 0x00060, 0x00063, 0x00066,
	0x00069, 0x0006c, 0x00070, 0x00073, 0x00076, 0x0007a, 0x0007d, 0x00081,
	0x00085, 0x00088, 0x0008c, 0x00090, 0x00094, 0x00098, 0x0009c, 0x000a0,
	0x000a4, 0x000a8, 0x000ad, 0x000b1, 0x000b5, 0x000ba, 0x000be, 0x000c3,
	0x000c8, 0x000cc, 0x000d1, 0x000d6, 0x000db, 0x000e0, 0x000e5, 0x000ea,
	0x000ef, 0x000f5, 0x000fa, 0x000ff, 0x00105, 0x0010a, 0x00110, 0x00116,
	0x0011b, 0x00121, 0x00127, 0x0012d, 0x00133, 0x00139, 0x0013f, 0x00145,
	0x0014c, 0x00152, 0x00158, 0x0015f, 0x00166, 0x0016c, 0x00173, 0x0017a,
	0x00180, 0x00187, 0x0018e, 0x00195, 0x0019d, 0x001a4, 0x001ab, 0x001b2,
	0x001ba, 0x001c1, 0x001c9, 0x001d1, 0x001d8, 0x001e0, 0x001e8, 0x001f0,
	0x001f8, 0x00200, 0x00208, 0x00211, 0x00219, 0x00221, 0x0022a, 0x00232,
	0x0023b, 0x00244, 0x0024c, 0x00255, 0x0025e, 0x00267, 0x00270, 0x00279,
	0x00283, 0x0028c, 0x00295, 0x0029f, 0x002a9, 0x002b2, 0x002bc, 0x002c6,
	0x002d0, 0x002da, 0x002e4, 0x002ee, 0x002f8, 0x00302, 0x0030d, 0x00317,
	0x00322, 0x0032c, 0x00337, 0x00342, 0x0034c, 0x00357, 0x00362, 0x0036e,
	0x00379, 0x00384, 0x0038f, 0x0039b, 0x003a6, 0x003b2, 0x003be, 0x003c9,
	0x003d5, 0x003e1, 0x003ed, 0x003f9, 0x00405, 0x00412, 0x0041e, 0x0042a,
	0x00437, 0x00444, 0x00450, 0x0045d, 0x0046a, 0x00477, 0x00484, 0x00491,
	0x0049e, 0x004ac, 0x004b9, 0x004c6, 0x004d4, 0x004e2, 0x004ef, 0x004fd,
	0x0050b, 0x00519, 0x00527, 0x00535, 0x00544, 0x00552, 0x00560, 0x0056f,
	0x0057e, 0x0058c, 0x0059b, 0x005aa, 0x005b9, 0x005c8, 0x005d7, 0x005e7,
	0x005f6, 0x00605, 0x00615, 0x00624, 0x00634, 0x00644, 0x00654, 0x00664,
	0x00674, 0x00684, 0x00694, 0x006a5, 0x006b5, 0x006c6, 0x006d6, 0x006e7,
	0x006f8, 0x00709, 0x0071a, 0x0072b, 0x0073c, 0x0074d, 0x0075e, 0x00770,
	0x00781, 0x00793, 0x007a5, 0x007b7, 0x007c9, 0x007db, 0x007ed, 0x007ff,
};

static const u32 n72_mie_matrix[11][2] = {
	{ 0x0a7ff5a5, 0xfff00000 },
	{ 0xf07ff5a5, 0xfff00000 },
	{ 0xff0ff7f7, 0xfff00000 },
	{ 0xaa50f5a5, 0xfff00000 },
	{ 0x55550353, 0xfff00000 },
	{ 0xfffff0fd, 0xfff00000 },
	{ 0xff7ff505, 0xfff00000 },
	{ 0xfffffdf0, 0xfff00000 },
	{ 0x33335333, 0x0ff00000 },
	{ 0x33333333, 0x70f00000 },
	{ 0xffffffff, 0xff000000 },
};

static const u32 n72_mie_bl_thresh[255] = {
	2185, 2243, 2299, 2353, 2414, 2468, 2522, 2580,
	2642, 2706, 2776, 2845, 2907, 2971, 3055, 3128,
	3213, 3298, 3382, 3461, 3552, 3632, 3710, 3797,
	3889, 3980, 4087, 4189, 4277, 4374, 4500, 4607,
	4733, 4860, 4983, 5102, 5238, 5356, 5473, 5604,
	5739, 5875, 6035, 6186, 6319, 6464, 6648, 6809,
	6981, 7170, 7352, 7532, 7730, 7904, 8079, 8270,
	8473, 8675, 8910, 9132, 9327, 9541, 9818, 10053,
	10305, 10588, 10855, 11114, 11412, 11670, 11923, 12211,
	12512, 12808, 13156, 13483, 13772, 14086, 14497, 14844,
	15231, 15643, 16049, 16431, 16863, 17251, 17628, 18047,
	18499, 18933, 19445, 19934, 20361, 20831, 21435, 21939,
	22467, 23071, 23656, 24227, 24873, 25444, 26003, 26628,
	27269, 27928, 28681, 29410, 30041, 30735, 31612, 32381,
	33148, 34061, 34933, 35754, 36717, 37546, 38386, 39293,
	40244, 41210, 42325, 43395, 44335, 45356, 46669, 47801,
	48989, 50334, 51653, 52885, 54288, 55535, 56780, 58145,
	59511, 61012, 62591, 64177, 65601, 67090, 69094, 70654,
	72286, 75679, 76221, 78010, 80245, 81976, 83784, 85811,
	87938, 90021, 92526, 94835, 96908, 99259, 101921, 104524,
	107480, 110150, 112957, 115910, 119021, 121745, 124596, 127584,
	130719, 133675, 137474, 141121, 144180, 147376, 152007, 155563,
	158814, 163700, 167831, 171621, 176167, 180348, 184731, 188662,
	194170, 198517, 203841, 208638, 213665, 218043, 224483, 230314,
	235410, 242934, 248610, 254558, 262082, 267350, 274240, 281495,
	287581, 293937, 302287, 311126, 316682, 324406, 334607, 343242,
	350017, 361922, 369462, 380018, 391195, 400019, 406126, 418918,
	429053, 439691, 450869, 462631, 470819, 483660, 497220, 511563,
	521594, 532026, 542883, 560027, 578289, 584644, 604575, 618634,
	633364, 648812, 665032, 682084, 700034, 709368, 738925, 760037,
	771052, 794068, 818501, 858106, 872173, 886709, 901738, 933378,
	950046, 985233, 1003822, 1023126, 1043188, 1064051, 1108387,
};

static u8 n72_mie_bl_scale(u32 stat)
{
	int i;

	if (stat < n72_mie_bl_thresh[0])
		return 255;
	for (i = 0; i < ARRAY_SIZE(n72_mie_bl_thresh) - 1; i++) {
		if (stat < n72_mie_bl_thresh[i + 1])
			return 255 - i;
	}
	return 1;
}

static void n72_lut_write_bank(void __iomem *regs, u32 idx_base, u32 bank,
			       const u32 *vals, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		writel(idx_base | CLCD_LUT_WRITE | bank | i,
		       regs + CLCD_LUT_IDX);
		writel(vals[i] & 0xfffff, regs + CLCD_LUT_WDATA);
	}
}

/* Write gamma and/or MIE LUT banks with CLCD_LUT_EN briefly bypassed. */
static void n72_clcd_write_lut_banks(struct n72_clcd *p, bool write_gamma,
				     bool write_mie)
{
	unsigned long flags;
	u32 base_idx, lut_en;

	spin_lock_irqsave(&p->reg_lock, flags);
	if (!n72_regs_ok(p)) {
		spin_unlock_irqrestore(&p->reg_lock, flags);
		return;
	}
	writel(0, p->regs + CLCD_LUT_EN);
	spin_unlock_irqrestore(&p->reg_lock, flags);

	/* iOS waits one frame (17 ms) after disabling LUT_EN before writing LUT RAM. */
	usleep_range(17000, 18000);

	spin_lock_irqsave(&p->reg_lock, flags);
	if (!n72_regs_ok(p)) {
		spin_unlock_irqrestore(&p->reg_lock, flags);
		return;
	}
	base_idx = readl(p->regs + CLCD_LUT_IDX) & 0xfffe0f00;
	if (write_gamma) {
		n72_lut_write_bank(p->regs, base_idx, CLCD_LUT_BANK_GAMMA_R,
				   p->gamma_lut[0], 256);
		n72_lut_write_bank(p->regs, base_idx, CLCD_LUT_BANK_GAMMA_G,
				   p->gamma_lut[1], 256);
		n72_lut_write_bank(p->regs, base_idx, CLCD_LUT_BANK_GAMMA_B,
				   p->gamma_lut[2], 256);
	}
	if (write_mie) {
		n72_lut_write_bank(p->regs, base_idx, CLCD_LUT_BANK_ENGAMMA,
				   n72_mie_engamma, ARRAY_SIZE(n72_mie_engamma));
		n72_lut_write_bank(p->regs, base_idx, CLCD_LUT_BANK_DEGAMMA_R,
				   n72_mie_degamma, ARRAY_SIZE(n72_mie_degamma));
		n72_lut_write_bank(p->regs, base_idx, CLCD_LUT_BANK_DEGAMMA_G,
				   n72_mie_degamma, ARRAY_SIZE(n72_mie_degamma));
		n72_lut_write_bank(p->regs, base_idx, CLCD_LUT_BANK_DEGAMMA_B,
				   n72_mie_degamma, ARRAY_SIZE(n72_mie_degamma));
	}
	lut_en = (p->gamma_enabled ? CLCD_LUT_GAMMA_EN : 0) |
		 (p->mie_active ? CLCD_LUT_MIE_EN : 0);
	writel(lut_en, p->regs + CLCD_LUT_EN);
	spin_unlock_irqrestore(&p->reg_lock, flags);
}

/* reg_lock held, registers reachable */
static void n72_mie_hw_init_locked(struct n72_clcd *p)
{
	void __iomem *r = p->regs;
	int i;

	/* ROI0/ROI1: +0 = (x_min << 16) | y_min, +4 = (x_max << 16) | y_max */
	writel(0, r + CLCD_MIE_ROI0);
	writel(((u32)(N72_W - 1) << 16) | (N72_H - 1), r + CLCD_MIE_ROI0 + 4);
	writel(0, r + CLCD_MIE_ROI1);
	writel(((u32)(N72_W - 1) << 16) | (N72_H - 1), r + CLCD_MIE_ROI1 + 4);

	writel((88U << 16) | 64U, r + CLCD_MIE_CURVEX);
	writel((105U << 16) | 56U, r + CLCD_MIE_CURVEX + 4);
	writel((80U << 16) | 128U, r + CLCD_MIE_CURVEX + 8);
	writel((80U << 16) | 128U, r + CLCD_MIE_CURVEX + 12);
	writel((96U << 16) | 96U, r + CLCD_MIE_CURVEX + 16);
	writel(0, r + CLCD_MIE_CURVEX + 20);

	writel(0x0c0d0bcc, r + CLCD_MIE_CURVEY);
	writel(0x0a110c26, r + CLCD_MIE_CURVEY + 4);
	writel(0x0e1f08f3, r + CLCD_MIE_CURVEY + 8);
	writel(0x0bbb08f3, r + CLCD_MIE_CURVEY + 12);
	writel(0x15b919b4, r + CLCD_MIE_CURVEY + 16);
	writel(0x08890000, r + CLCD_MIE_CURVEY + 20);

	for (i = 0; i < 11; i++) {
		writel(n72_mie_matrix[i][0], r + CLCD_MIE_MATRIX + i * 8);
		writel(n72_mie_matrix[i][1], r + CLCD_MIE_MATRIX + i * 8 + 4);
	}

	writel(readl(r + CLCD_MIE_CTRL) | BIT(5), r + CLCD_MIE_CTRL);
	writel((0x7ffU << 16) | 0x7ffU, r + CLCD_MIE_GAIN1);
	writel(0x7ffU << 16, r + CLCD_MIE_GAIN1 + 4);
	writel(0, r + CLCD_MIE_GAIN2);
	writel(0, r + CLCD_MIE_GAIN2 + 4);
	writel(0, r + CLCD_MIE_GAIN3);
	writel((readl(r + CLCD_MIE_CFG0) & ~0x1ffU) | 0x80U, r + CLCD_MIE_CFG0);
}

static void n72_mie_bl_work(struct work_struct *work)
{
	struct n72_clcd *p = container_of(work, struct n72_clcd, mie_bl_work);
	u8 scale = READ_ONCE(p->mie_pending_scale);

	WRITE_ONCE(p->mie_scale, scale);
	n72_pmu_bl_set_mie_scale(scale);
}

void n72_mie_vblank(struct n72_clcd *p)
{
	unsigned long flags;
	bool sched_bl = false;
	u32 stat;

	if (!READ_ONCE(p->mie_active))
		return;

	spin_lock_irqsave(&p->reg_lock, flags);
	if (!p->mie_active || !n72_regs_ok(p)) {
		spin_unlock_irqrestore(&p->reg_lock, flags);
		return;
	}
	stat = readl(p->regs + CLCD_MIE_STAT);
	if (!p->mie_stopping && (stat & 0xffff) != p->mie_last_stat) {
		u8 scale;

		p->mie_last_stat = stat & 0xffff;
		scale = n72_mie_bl_scale(p->mie_last_stat);
		if (scale != READ_ONCE(p->mie_pending_scale)) {
			WRITE_ONCE(p->mie_pending_scale, scale);
			sched_bl = true;
		}
	}
	if (p->mie_stopping) {
		p->mie_off_frames++;
		if (!(stat & BIT(20)) && p->mie_off_frames >= 2) {
			u32 ctrl = readl(p->regs + CLCD_MIE_CTRL) & ~(BIT(0) | BIT(1));

			writel(ctrl, p->regs + CLCD_MIE_CTRL);
			writel(p->gamma_enabled ? CLCD_LUT_GAMMA_EN : 0,
			       p->regs + CLCD_LUT_EN);
			p->mie_active = false;
			p->mie_stopping = false;
			p->mie_off_frames = 0;
			n72_vbl_hold_mie(p, false);
			WRITE_ONCE(p->mie_pending_scale, 255);
			sched_bl = true;
		}
	}
	spin_unlock_irqrestore(&p->reg_lock, flags);

	if (sched_bl)
		schedule_work(&p->mie_bl_work);
}

static void n72_mie_sync_locked(struct n72_clcd *p, bool force_reinit)
{
	bool want = p->mie_mode == 2 || (p->mie_mode == 1 && READ_ONCE(p->video_on));
	unsigned long flags;
	bool reset_bl = false;

	if (want && (!p->mie_active || p->mie_stopping || force_reinit)) {
		if (!p->mie_hw_inited || force_reinit)
			n72_clcd_write_lut_banks(p, p->gamma_enabled, true);

		spin_lock_irqsave(&p->reg_lock, flags);
		if (n72_regs_ok(p)) {
			u32 ctrl;

			if (!p->mie_hw_inited || force_reinit) {
				n72_mie_hw_init_locked(p);
				p->mie_hw_inited = true;
			}
			ctrl = readl(p->regs + CLCD_MIE_CTRL) & ~BIT(1);
			writel(ctrl, p->regs + CLCD_MIE_CTRL);
			writel(readl(p->regs + CLCD_MIE_CFG0) & ~BIT(8),
			       p->regs + CLCD_MIE_CFG0);
			writel((p->gamma_enabled ? CLCD_LUT_GAMMA_EN : 0) | CLCD_LUT_MIE_EN,
			       p->regs + CLCD_LUT_EN);
			writel(ctrl | BIT(0), p->regs + CLCD_MIE_CTRL);
			p->mie_active = true;
			p->mie_stopping = false;
			p->mie_off_frames = 0;
			p->mie_last_stat = 0x889;
			n72_vbl_hold_mie(p, true);
		}
		spin_unlock_irqrestore(&p->reg_lock, flags);
	} else if (!want && p->mie_active && !p->mie_stopping) {
		spin_lock_irqsave(&p->reg_lock, flags);
		if (n72_regs_ok(p)) {
			writel(readl(p->regs + CLCD_MIE_CTRL) | BIT(1),
			       p->regs + CLCD_MIE_CTRL);
			writel((readl(p->regs + CLCD_MIE_RAMP) & ~0xfU) | 10U,
			       p->regs + CLCD_MIE_RAMP);
			p->mie_stopping = true;
			p->mie_off_frames = 0;
		} else {
			p->mie_active = false;
			p->mie_stopping = false;
			n72_vbl_hold_mie(p, false);
			reset_bl = true;
		}
		spin_unlock_irqrestore(&p->reg_lock, flags);
	} else if (!want && !p->mie_active) {
		if (READ_ONCE(p->mie_scale) != 255)
			reset_bl = true;
	}

	if (reset_bl) {
		WRITE_ONCE(p->mie_pending_scale, 255);
		schedule_work(&p->mie_bl_work);
	}
}

void n72_mie_sync(struct n72_clcd *p)
{
	mutex_lock(&p->lut_lock);
	n72_mie_sync_locked(p, false);
	mutex_unlock(&p->lut_lock);
}

int n72_mie_set_mode(struct n72_clcd *p, u8 mode)
{
	if (mode > 2)
		return -EINVAL;

	mutex_lock(&p->lut_lock);
	p->mie_mode = mode;
	n72_mie_sync_locked(p, false);
	mutex_unlock(&p->lut_lock);
	return 0;
}

void n72_clcd_apply_gamma_state(struct n72_clcd *p,
				const struct drm_property_blob *gamma_lut)
{
	unsigned long flags;
	bool changed_to_on = false;

	mutex_lock(&p->lut_lock);
	if (gamma_lut && gamma_lut->data) {
		const struct drm_color_lut *lut = gamma_lut->data;
		int i;

		for (i = 0; i < 256; i++) {
			u32 r0 = lut[i].red >> 6;
			u32 g0 = lut[i].green >> 6;
			u32 b0 = lut[i].blue >> 6;
			u32 r1 = i < 255 ? ((r0 + (lut[i + 1].red >> 6)) >> 1) : r0;
			u32 g1 = i < 255 ? ((g0 + (lut[i + 1].green >> 6)) >> 1) : g0;
			u32 b1 = i < 255 ? ((b0 + (lut[i + 1].blue >> 6)) >> 1) : b0;

			p->gamma_lut[0][i] = (r0 & 0x3ff) | ((r1 & 0x3ff) << 10);
			p->gamma_lut[1][i] = (g0 & 0x3ff) | ((g1 & 0x3ff) << 10);
			p->gamma_lut[2][i] = (b0 & 0x3ff) | ((b1 & 0x3ff) << 10);
		}
		p->gamma_enabled = true;
		changed_to_on = true;
	} else {
		p->gamma_enabled = false;
	}

	if (changed_to_on) {
		n72_clcd_write_lut_banks(p, true, false);
	} else {
		spin_lock_irqsave(&p->reg_lock, flags);
		if (n72_regs_ok(p))
			writel(p->mie_active ? CLCD_LUT_MIE_EN : 0,
			       p->regs + CLCD_LUT_EN);
		spin_unlock_irqrestore(&p->reg_lock, flags);
	}
	mutex_unlock(&p->lut_lock);
}

static void n72_lut_restore_work(struct work_struct *work)
{
	struct n72_clcd *p = container_of(work, struct n72_clcd, lut_restore_work);

	n72_clcd_restore_color(p);
}

void n72_clcd_restore_color(struct n72_clcd *p)
{
	mutex_lock(&p->lut_lock);
	if (p->gamma_enabled)
		n72_clcd_write_lut_banks(p, true, false);
	p->mie_hw_inited = false;
	n72_mie_sync_locked(p, true);
	mutex_unlock(&p->lut_lock);
}

void n72_clcd_init_color(struct n72_clcd *p)
{
	mutex_init(&p->lut_lock);
	INIT_WORK(&p->lut_restore_work, n72_lut_restore_work);
	INIT_WORK(&p->mie_bl_work, n72_mie_bl_work);
	p->mie_scale = 255;
	p->mie_pending_scale = 255;
}

void n72_clcd_fini_color(struct n72_clcd *p)
{
	cancel_work_sync(&p->lut_restore_work);
	cancel_work_sync(&p->mie_bl_work);
	n72_pmu_bl_set_mie_scale(255);
}
