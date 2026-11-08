// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 */

#include <drm/drm_managed.h>

#include "dpu_hwio.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_lm.h"
#include "dpu_hw_dspp.h"
#include "dpu_kms.h"


/* DSPP_PCC */
#define PCC_EN BIT(0)
#define PCC_DIS 0
#define PCC_RED_R_OFF 0x10
#define PCC_RED_G_OFF 0x1C
#define PCC_RED_B_OFF 0x28
#define PCC_GREEN_R_OFF 0x14
#define PCC_GREEN_G_OFF 0x20
#define PCC_GREEN_B_OFF 0x2C
#define PCC_BLUE_R_OFF 0x18
#define PCC_BLUE_G_OFF 0x24
#define PCC_BLUE_B_OFF 0x30

/* DSPP_GC */
#define GC_EN BIT(0)
#define GC_DIS 0
#define GC_8B_ROUND_EN BIT(1)
#define GC_LUT_SWAP_OFF 0x1c
#define GC_C0_OFF 0x4
#define GC_C1_OFF 0xc
#define GC_C2_OFF 0x14
#define GC_C0_INDEX_OFF 0x8
#define GC_C1_INDEX_OFF 0x10
#define GC_C2_INDEX_OFF 0x18

/*
 * DSPP_GAMUT, the 3D LUT ahead of PCC. In its 13-point mode the LUT memory
 * holds two banks, so a new LUT can be written while the other one is shown
 * and the switch is a single opmode write, double buffered by the flush.
 *
 * Nodes are spread over four tables by node index modulo 4, which keeps the
 * four corners of an interpolation tetrahedron in different tables. Each
 * entry is two writes, c2 << 16 | c1 to the lower and c0 to the upper color
 * register, with c0 = R, c1 = B, c2 = G, 12 bits each. The table select
 * register picks the table and the start entry, and the writes advance it.
 * Bank B starts at entry 640 of each table. The tables cannot be read back.
 *
 * The 13-point mode needs the input map: per channel 16 segments of
 * slope << 16 | breakpoint, breakpoints in 12-bit input codes. An input in
 * segment j reaches grid position j + slope * (x - breakpoint_j) / 2^19.
 * Bank A uses map A, bank B map B.
 */
#define GAMUT_EN		BIT(0)
#define GAMUT_MAP_EN		BIT(1)
#define GAMUT_MODE_13A		(2 << 2)
#define GAMUT_MODE_13B		(3 << 2)
#define GAMUT_TABLE_SEL_OFF	0x4
#define GAMUT_UPPER_COLOR_OFF	0x8
#define GAMUT_LOWER_COLOR_OFF	0xc
#define GAMUT_MAP_A_OFF		0x10
#define GAMUT_MAP_B_OFF		0xe0
#define GAMUT_TABLE0_SEL	BIT(12)
#define GAMUT_TABLES		4
#define GAMUT_BANK_B_ENTRY	640
#define GAMUT_MAP_SEGMENTS	16
/* 2^19 per node spacing of 4096 / 12 input codes */
#define GAMUT_MAP_SLOPE		1536

static void dpu_setup_dspp_gamut_load(struct dpu_hw_dspp *ctx, int bank,
		const struct dpu_hw_gamut_lut *lut)
{
	u32 base = ctx->cap->sblk->gamut.base;
	u32 map = base + (bank ? GAMUT_MAP_B_OFF : GAMUT_MAP_A_OFF);
	int t, i, ch;

	for (t = 0; t < GAMUT_TABLES; t++) {
		DPU_REG_WRITE(&ctx->hw, base + GAMUT_TABLE_SEL_OFF,
			      GAMUT_TABLE0_SEL << t |
			      (bank ? GAMUT_BANK_B_ENTRY : 0));
		for (i = t; i < DPU_GAMUT_NODES; i += GAMUT_TABLES) {
			const u16 *rgb = lut->node[i];

			DPU_REG_WRITE(&ctx->hw, base + GAMUT_LOWER_COLOR_OFF,
				      rgb[1] << 16 | rgb[2]);
			DPU_REG_WRITE(&ctx->hw, base + GAMUT_UPPER_COLOR_OFF,
				      rgb[0]);
		}
	}

	/* uniform grid; segments past the last node stay at full scale */
	for (ch = 0; ch < 3; ch++) {
		for (i = 0; i < GAMUT_MAP_SEGMENTS; i++) {
			u32 bp = min_t(u32, (i * 4096 + 6) / (DPU_GAMUT_GRID - 1), 4095);

			DPU_REG_WRITE(&ctx->hw,
				      map + (ch * GAMUT_MAP_SEGMENTS + i) * 4,
				      GAMUT_MAP_SLOPE << 16 | bp);
		}
	}
}

static void dpu_setup_dspp_gamut_select(struct dpu_hw_dspp *ctx, int bank)
{
	u32 base = ctx->cap->sblk->gamut.base;

	if (bank < 0) {
		DPU_REG_WRITE(&ctx->hw, base, 0);
		return;
	}

	DPU_REG_WRITE(&ctx->hw, base, GAMUT_EN | GAMUT_MAP_EN |
		      (bank ? GAMUT_MODE_13B : GAMUT_MODE_13A));
}

static void dpu_setup_dspp_pcc(struct dpu_hw_dspp *ctx,
		struct dpu_hw_pcc_cfg *cfg)
{

	u32 base;

	if (!ctx) {
		DRM_ERROR("invalid ctx %p\n", ctx);
		return;
	}

	base = ctx->cap->sblk->pcc.base;

	if (!base) {
		DRM_ERROR("invalid ctx %p pcc base 0x%x\n", ctx, base);
		return;
	}

	if (!cfg) {
		DRM_DEBUG_DRIVER("disable pcc feature\n");
		DPU_REG_WRITE(&ctx->hw, base, PCC_DIS);
		return;
	}

	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_R_OFF, cfg->r.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_G_OFF, cfg->r.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_B_OFF, cfg->r.b);

	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_R_OFF, cfg->g.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_G_OFF, cfg->g.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_B_OFF, cfg->g.b);

	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_R_OFF, cfg->b.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_G_OFF, cfg->b.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_B_OFF, cfg->b.b);

	DPU_REG_WRITE(&ctx->hw, base, PCC_EN);
}

static void dpu_setup_dspp_gc(struct dpu_hw_dspp *ctx,
		struct dpu_hw_gc_lut *gc_lut)
{
	int i = 0;
	u32 base, reg;

	if (!ctx) {
		DRM_ERROR("invalid ctx\n");
		return;
	}

	base = ctx->cap->sblk->gc.base;

	if (!base) {
		DRM_ERROR("invalid ctx %p gc base\n", ctx);
		return;
	}

	if (!gc_lut) {
		DRM_DEBUG_DRIVER("disable gc feature\n");
		DPU_REG_WRITE(&ctx->hw, base, GC_DIS);
		return;
	}

	DPU_REG_WRITE(&ctx->hw, base + GC_C0_INDEX_OFF, 0);
	DPU_REG_WRITE(&ctx->hw, base + GC_C1_INDEX_OFF, 0);
	DPU_REG_WRITE(&ctx->hw, base + GC_C2_INDEX_OFF, 0);

	for (i = 0; i < PGC_TBL_LEN; i++) {
		DPU_REG_WRITE(&ctx->hw, base + GC_C0_OFF, gc_lut->c0[i]);
		DPU_REG_WRITE(&ctx->hw, base + GC_C1_OFF, gc_lut->c1[i]);
		DPU_REG_WRITE(&ctx->hw, base + GC_C2_OFF, gc_lut->c2[i]);
	}

	DPU_REG_WRITE(&ctx->hw, base + GC_LUT_SWAP_OFF, BIT(0));

	reg = GC_EN | ((gc_lut->flags & PGC_8B_ROUND) ? GC_8B_ROUND_EN : 0);
	DPU_REG_WRITE(&ctx->hw, base, reg);
}

/**
 * dpu_hw_dspp_init() - Initializes the DSPP hw driver object.
 * should be called once before accessing every DSPP.
 * @dev:  Corresponding device for devres management
 * @cfg:  DSPP catalog entry for which driver object is required
 * @addr: Mapped register io address of MDP
 * Return: pointer to structure or ERR_PTR
 */
struct dpu_hw_dspp *dpu_hw_dspp_init(struct drm_device *dev,
				     const struct dpu_dspp_cfg *cfg,
				     void __iomem *addr)
{
	struct dpu_hw_dspp *c;

	if (!addr)
		return ERR_PTR(-EINVAL);

	c = drmm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->hw.blk_addr = addr + cfg->base;
	c->hw.log_mask = DPU_DBG_MASK_DSPP;

	/* Assign ops */
	c->idx = cfg->id;
	c->cap = cfg;
	if (c->cap->sblk->pcc.base)
		c->ops.setup_pcc = dpu_setup_dspp_pcc;
	if (c->cap->sblk->gc.base)
		c->ops.setup_gc = dpu_setup_dspp_gc;
	if (c->cap->sblk->gamut.base) {
		c->ops.load_gamut = dpu_setup_dspp_gamut_load;
		c->ops.select_gamut = dpu_setup_dspp_gamut_select;
	}

	return c;
}
