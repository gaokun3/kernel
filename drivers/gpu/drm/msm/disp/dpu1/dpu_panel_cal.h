/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _DPU_PANEL_CAL_H
#define _DPU_PANEL_CAL_H

#include <linux/types.h>

/* Coefficients are Q15, row-major, in the order of the DRM CTM */
#define DPU_PANEL_CAL_ONE	(1 << 15)
#define DPU_PANEL_CAL_LEN	9

enum dpu_panel_cal_gamut {
	DPU_PANEL_CAL_SRGB,
	DPU_PANEL_CAL_P3,
	DPU_PANEL_CAL_GAMUTS,
};

int dpu_panel_cal_load(s32 (*cal)[DPU_PANEL_CAL_LEN]);

#endif /* _DPU_PANEL_CAL_H */
