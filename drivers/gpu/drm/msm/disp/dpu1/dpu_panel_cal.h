/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _DPU_PANEL_CAL_H
#define _DPU_PANEL_CAL_H

#include <linux/types.h>

struct dpu_hw_gamut_lut;

/* the LUT of each gamut, also the 3D LUT bank it is loaded into */
enum dpu_panel_cal_gamut {
	DPU_PANEL_CAL_SRGB,
	DPU_PANEL_CAL_P3,
	DPU_PANEL_CAL_GAMUTS,
};

int dpu_panel_cal_load(struct dpu_hw_gamut_lut *lut);

#endif /* _DPU_PANEL_CAL_H */
