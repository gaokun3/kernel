// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-170.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v170
 *
 * Copyright (C) 2020-2021 Linaro Ltd.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-17x-meta.h"

#define VFE_GLOBAL_RESET_CMD			(0x018)
#define		GLOBAL_RESET_CMD_CORE		BIT(0)
#define		GLOBAL_RESET_CMD_CAMIF		BIT(1)
#define		GLOBAL_RESET_CMD_BUS		BIT(2)
#define		GLOBAL_RESET_CMD_BUS_BDG	BIT(3)
#define		GLOBAL_RESET_CMD_REGISTER	BIT(4)
#define		GLOBAL_RESET_CMD_PM		BIT(5)
#define		GLOBAL_RESET_CMD_BUS_MISR	BIT(6)
#define		GLOBAL_RESET_CMD_TESTGEN	BIT(7)
#define		GLOBAL_RESET_CMD_DSP		BIT(8)
#define		GLOBAL_RESET_CMD_IDLE_CGC	BIT(9)
#define		GLOBAL_RESET_CMD_RDI0		BIT(10)
#define		GLOBAL_RESET_CMD_RDI1		BIT(11)
#define		GLOBAL_RESET_CMD_RDI2		BIT(12)
#define		GLOBAL_RESET_CMD_RDI3		BIT(13)
#define		GLOBAL_RESET_CMD_VFE_DOMAIN	BIT(30)
#define		GLOBAL_RESET_CMD_RESET_BYPASS	BIT(31)

/* Pixel pipeline module enables */
#define VFE_MODULE_LENS_EN			(0x040)
#define		MODULE_LENS_EN_DEMUX		BIT(2)
#define		MODULE_LENS_EN_DEMOSAIC		BIT(10)
#define		MODULE_LENS_EN_BLS		BIT(11)
#define VFE_MODULE_STATS_EN			(0x044)
#define VFE_MODULE_COLOR_EN			(0x048)
#define		MODULE_COLOR_EN_GAMMA		BIT(3)
#define VFE_MODULE_ZOOM_EN			(0x04c)
#define		MODULE_ZOOM_EN_CST		BIT(0)
#define		MODULE_ZOOM_EN_SCALE_FULL	BIT(3)
#define		MODULE_ZOOM_EN_CROP_FULL	BIT(4)

#define VFE_CORE_CFG				(0x050)
#define		CFG_PIXEL_PATTERN_YCBYCR	(0x4)
#define		CFG_PIXEL_PATTERN_YCRYCB	(0x5)
#define		CFG_PIXEL_PATTERN_CBYCRY	(0x6)
#define		CFG_PIXEL_PATTERN_CRYCBY	(0x7)
#define		CFG_COMPOSITE_REG_UPDATE_EN	BIT(4)
/* Set by the Windows SC8280XP ISP driver on every IFE start, unnamed */
#define		CFG_BIT24			BIT(24)

#define VFE_IRQ_CMD				(0x058)
#define		CMD_GLOBAL_CLEAR		BIT(0)

#define VFE_IRQ_MASK_0					(0x05c)
#define		MASK_0_CAMIF_SOF			BIT(0)
#define		MASK_0_CAMIF_EOF			BIT(1)
#define		MASK_0_RDI_REG_UPDATE(n)		BIT((n) + 5)
#define		MASK_0_IMAGE_MASTER_n_PING_PONG(n)	BIT((n) + 8)
#define		MASK_0_IMAGE_COMPOSITE_DONE_n(n)	BIT((n) + 25)
#define		MASK_0_RESET_ACK			BIT(31)

#define VFE_IRQ_MASK_1					(0x060)
#define		MASK_1_CAMIF_ERROR			BIT(0)
#define		MASK_1_VIOLATION			BIT(7)
#define		MASK_1_BUS_BDG_HALT_ACK			BIT(8)
#define		MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(n)	BIT((n) + 9)
#define		MASK_1_RDI_SOF(n)			BIT((n) + 29)

#define VFE_IRQ_CLEAR_0					(0x064)
#define VFE_IRQ_CLEAR_1					(0x068)

#define VFE_IRQ_STATUS_0				(0x06c)
#define		STATUS_0_CAMIF_SOF			BIT(0)
#define		STATUS_0_CAMIF_EPOCH0			BIT(2)
#define		STATUS_0_CAMIF_REG_UPDATE		BIT(4)
#define		STATUS_0_RDI_REG_UPDATE(n)		BIT((n) + 5)
#define		STATUS_0_IMAGE_MASTER_PING_PONG(n)	BIT((n) + 8)
#define		STATUS_0_IMAGE_COMPOSITE_DONE(n)	BIT((n) + 25)
#define		STATUS_0_RESET_ACK			BIT(31)

#define VFE_IRQ_STATUS_1				(0x070)
#define		STATUS_1_VIOLATION			BIT(7)
#define		STATUS_1_BUS_BDG_HALT_ACK		BIT(8)
#define		STATUS_1_RDI_SOF(n)			BIT((n) + 27)

#define VFE_VIOLATION_STATUS			(0x07c)

#define VFE_CAMIF_CMD				(0x478)
#define		CMD_CLEAR_CAMIF_STATUS		BIT(2)

#define VFE_CAMIF_CFG				(0x47c)
#define		CFG_VSYNC_SYNC_EDGE		(0)
#define			VSYNC_ACTIVE_HIGH	(0)
#define			VSYNC_ACTIVE_LOW	(1)
#define		CFG_HSYNC_SYNC_EDGE		(1)
#define			HSYNC_ACTIVE_HIGH	(0)
#define			HSYNC_ACTIVE_LOW	(1)
#define		CFG_VFE_SUBSAMPLE_ENABLE	BIT(4)
#define		CFG_BUS_SUBSAMPLE_ENABLE	BIT(5)
#define		CFG_VFE_OUTPUT_EN		BIT(6)
#define		CFG_BUS_OUTPUT_EN		BIT(7)
#define		CFG_BINNING_EN			BIT(9)
#define		CFG_FRAME_BASED_EN		BIT(10)
#define		CFG_RAW_CROP_EN			BIT(22)

#define VFE_REG_UPDATE_CMD			(0x4ac)
#define		REG_UPDATE_PIX			BIT(0)
#define		REG_UPDATE_RDI(n)		BIT(1 + (n))

/*
 * Pixel pipeline (CAMIF -> DEMUX -> DEMOSAIC/WB -> CST -> MNDS -> CROP ->
 * ROUND/CLAMP -> FULL Y/C write masters). Offsets and field layouts come
 * from the Titan 17x register packing in the SC8280XP Windows camera stack
 * (CamX rel 10.6, QcDeviceMFT8280.dll) and its ISP kernel driver.
 */
#define VFE_CAMIF_LINE_SKIP_PATTERN		(0x488)
#define VFE_CAMIF_PIXEL_SKIP_PATTERN		(0x48c)
#define VFE_CAMIF_SKIP_PERIOD			(0x490)
#define		CAMIF_SKIP_PERIOD_DEFAULT	0xf0f
#define VFE_CAMIF_IRQ_SUBSAMPLE_PATTERN		(0x49c)

#define VFE_CAMIF_RAW_CROP_WIDTH_CFG		(0xce4)
#define VFE_CAMIF_RAW_CROP_HEIGHT_CFG		(0xce8)
#define		RAW_CROP_FIRST			16
#define		RAW_CROP_LAST			0
#define		RAW_CROP_MASK			0x3fff

#define VFE_DEMUX_CFG				(0x560)
#define		DEMUX_CFG_PERIOD		0
#define VFE_DEMUX_GAIN(n)			(0x564 + (n) * 4)
#define		DEMUX_GAIN_CH0			0
#define		DEMUX_GAIN_CH1			16
#define		DEMUX_GAIN_UNITY		0x400	/* Q10 */
#define VFE_DEMUX_EVEN_CFG			(0x574)
#define VFE_DEMUX_ODD_CFG			(0x578)

#define VFE_DEMOSAIC_CFG			(0x6f8)
#define		DEMOSAIC_CFG_DIR_G_INTERP_DIS	BIT(0)
#define		DEMOSAIC_CFG_DIR_RB_INTERP_DIS	BIT(1)
#define VFE_DEMOSAIC_INTERP_COEFF_0		(0x71c)
#define VFE_DEMOSAIC_INTERP_COEFF_1		(0x720)

/* White balance gains, applied by the demosaic block */
#define VFE_WB_CFG_0				(0x6fc)
#define		WB_CFG_0_G_GAIN			0
#define		WB_CFG_0_B_GAIN			16
#define VFE_WB_CFG_1				(0x700)
#define		WB_CFG_1_R_GAIN			0
#define VFE_WB_OFFSET_CFG_0			(0x704)
#define VFE_WB_OFFSET_CFG_1			(0x708)
#define		WB_GAIN_UNITY			0x80	/* Q7 */
#define		WB_GAIN_MASK			0xfff

/*
 * Black level subtraction (BLS12), after the demux: out = (in - offset) * scale
 * with scale = 16383 / (16383 - offset) in Q11, so the remaining range is
 * stretched back to full scale. The pixel pipeline works on 14 bit values with
 * the sensor data MSB aligned, so a 10 bit black level of 64 is 1024 here.
 */
#define VFE_BLS_CFG				(0x6b0)
#define		BLS_CFG_OFFSET			0
#define		BLS_CFG_OFFSET_MASK		0x3fff
#define		BLS_CFG_SCALE			15
#define VFE_BLS_THRESHOLD_0			(0x6b4)
#define VFE_BLS_THRESHOLD_1			(0x6b8)
#define VFE_PIX_MAX				16383	/* 14 bit */

/* RGB to YCbCr colour space transform, one 4 register block per channel */
#define VFE_CST_CH_CFG(ch, n)			(0xf30 + (ch) * 0x10 + (n) * 4)
#define		CST_M0				0	/* reg 0, bits 12:0 */
#define		CST_M1				16	/* reg 0, bits 28:16 */
#define		CST_M2				0	/* reg 1, bits 12:0 */
#define		CST_OFFSET_IN			0	/* reg 2, bits 10:0 */
#define		CST_OFFSET_OUT			16	/* reg 2, bits 26:16 */
#define		CST_CLAMP_MIN			0	/* reg 3, bits 9:0 */
#define		CST_CLAMP_MAX			16	/* reg 3, bits 25:16 */
#define		CST_COEFF_MASK			0x1fff

/* MNDS16 scaler of the FULL output path, luma at 0xa3c, chroma at 0xa68 */
#define VFE_SCALE_FULL_Y_BASE			(0xa3c)
#define VFE_SCALE_FULL_C_BASE			(0xa68)
#define		SCALE_CFG			0x00
#define			SCALE_CFG_H_EN		BIT(0)
#define			SCALE_CFG_V_EN		BIT(1)
#define		SCALE_H_IMAGE_SIZE		0x04
#define		SCALE_H_PHASE			0x08
#define			SCALE_PHASE_MULT	0
#define			SCALE_PHASE_MULT_MASK	0x3fffff
#define			SCALE_PHASE_INTERP	28
#define		SCALE_H_STRIPE			0x0c
#define		SCALE_H_INIT_PHASE		0x10
#define		SCALE_H_PAD			0x14
#define		SCALE_V_IMAGE_SIZE		0x18
#define		SCALE_V_PHASE			0x1c
#define		SCALE_V_STRIPE			0x20
#define		SCALE_V_INIT_PHASE		0x24
#define		SCALE_V_PAD			0x28
#define		SCALE_SIZE_IN			0
#define		SCALE_SIZE_OUT			16
#define		SCALE_SIZE_MASK			0x3fff

/* Crop / round / clamp of the FULL output path */
#define VFE_FULL_Y_MODULE_CFG			(0xe0c)
#define VFE_FULL_C_MODULE_CFG			(0xe2c)
#define		FULL_MODULE_CFG_CROP_EN		BIT(9)
#define		FULL_MODULE_CFG_ROUND_EN	BIT(10)
#define		FULL_MODULE_CFG_CLAMP_EN	BIT(11)
#define VFE_FULL_Y_CROP_LINE_CFG		(0xe10)
#define VFE_FULL_Y_CROP_PIXEL_CFG		(0xe14)
#define VFE_FULL_Y_CLAMP_CFG			(0xe18)
#define VFE_FULL_Y_ROUND_CFG			(0xe1c)
#define VFE_FULL_C_CROP_LINE_CFG		(0xe30)
#define VFE_FULL_C_CROP_PIXEL_CFG		(0xe34)
#define VFE_FULL_C_CLAMP_CFG			(0xe38)
#define VFE_FULL_C_ROUND_CFG			(0xe3c)
#define		CROP_FIRST			16
#define		CROP_LAST			0
#define		CROP_LINE_MASK			0x3fff
#define		CROP_PIXEL_MASK			0x1fff
/* 10 bit pipeline clamped to 0..255 and rounded to 8 bit, as CamX does */
#define		CLAMP_CFG_8BIT			0x0ff00000
#define		ROUND_CFG_Y_8BIT		0x16
#define		ROUND_CFG_C_8BIT		0x17

/* Last register of the pixel pipeline IQ window */

#define VFE_BUS_IRQ_MASK(n)		(0x2044 + (n) * 4)
#define VFE_BUS_IRQ_CLEAR(n)		(0x2050 + (n) * 4)
#define VFE_BUS_IRQ_STATUS(n)		(0x205c + (n) * 4)
#define		STATUS0_COMP_RESET_DONE		BIT(0)
#define		STATUS0_COMP_REG_UPDATE0_DONE	BIT(1)
#define		STATUS0_COMP_REG_UPDATE1_DONE	BIT(2)
#define		STATUS0_COMP_REG_UPDATE2_DONE	BIT(3)
#define		STATUS0_COMP_REG_UPDATE3_DONE	BIT(4)
#define		STATUS0_COMP_REG_UPDATE_DONE(n)	BIT((n) + 1)
#define		STATUS0_COMP0_BUF_DONE		BIT(5)
#define		STATUS0_COMP1_BUF_DONE		BIT(6)
#define		STATUS0_COMP2_BUF_DONE		BIT(7)
#define		STATUS0_COMP3_BUF_DONE		BIT(8)
#define		STATUS0_COMP4_BUF_DONE		BIT(9)
#define		STATUS0_COMP5_BUF_DONE		BIT(10)
#define		STATUS0_COMP_BUF_DONE(n)	BIT((n) + 5)
#define		STATUS0_COMP_ERROR		BIT(11)
#define		STATUS0_COMP_OVERWRITE		BIT(12)
#define		STATUS0_OVERFLOW		BIT(13)
#define		STATUS0_VIOLATION		BIT(14)
/* WM_CLIENT_BUF_DONE defined for buffers 0:19 */
#define		STATUS1_WM_CLIENT_BUF_DONE(n)		BIT(n)
#define		STATUS1_EARLY_DONE			BIT(24)
#define		STATUS2_DUAL_COMP0_BUF_DONE		BIT(0)
#define		STATUS2_DUAL_COMP1_BUF_DONE		BIT(1)
#define		STATUS2_DUAL_COMP2_BUF_DONE		BIT(2)
#define		STATUS2_DUAL_COMP3_BUF_DONE		BIT(3)
#define		STATUS2_DUAL_COMP4_BUF_DONE		BIT(4)
#define		STATUS2_DUAL_COMP5_BUF_DONE		BIT(5)
#define		STATUS2_DUAL_COMP_BUF_DONE(n)		BIT(n)
#define		STATUS2_DUAL_COMP_ERROR			BIT(6)
#define		STATUS2_DUAL_COMP_OVERWRITE		BIT(7)

#define VFE_BUS_IRQ_CLEAR_GLOBAL		(0x2068)
#define VFE_BUS_COMP_ERROR_STATUS		(0x206c)
#define VFE_BUS_COMP_OVERWRITE_STATUS		(0x2070)
#define VFE_BUS_DEBUG_STATUS0			(0x2270)

/* The error interrupts, as the vendor driver for Titan 175 takes them */
#define VFE_ERR_MASK_0				0x0003fc00
#define VFE_ERR_MASK_1				0xefff7e80
#define VFE_BUS_ERR_MASK_0			(STATUS0_COMP_ERROR | \
						 STATUS0_COMP_OVERWRITE | \
						 STATUS0_OVERFLOW | \
						 STATUS0_VIOLATION)
#define VFE_BUS_ERR_MASK_2			(STATUS2_DUAL_COMP_ERROR | \
						 STATUS2_DUAL_COMP_OVERWRITE)
/* More error interrupts than this in VFE_ERR_WINDOW_NS turn them off */
#define VFE_ERR_STORM				16
#define VFE_ERR_WINDOW_NS			(100 * NSEC_PER_MSEC)

#define VFE_BUS_WM_DEBUG_STATUS_CFG		(0x226c)
#define		DEBUG_STATUS_CFG_STATUS0(n)	BIT(n)
#define		DEBUG_STATUS_CFG_STATUS1(n)	BIT(8 + (n))

#define VFE_BUS_WM_ADDR_SYNC_FRAME_HEADER	(0x2080)

#define VFE_BUS_WM_ADDR_SYNC_NO_SYNC		(0x2084)
#define		BUS_VER2_MAX_CLIENTS (24)
#define		WM_ADDR_NO_SYNC_DEFAULT_VAL \
				((1 << BUS_VER2_MAX_CLIENTS) - 1)

#define VFE_BUS_WM_CGC_OVERRIDE			(0x200c)
#define		WM_CGC_OVERRIDE_ALL		(0xFFFFF)

#define VFE_BUS_WM_TEST_BUS_CTRL		(0x211c)

#define VFE_BUS_WM_STATUS0(n)			(0x2200 + (n) * 0x100)
#define VFE_BUS_WM_STATUS1(n)			(0x2204 + (n) * 0x100)
#define VFE_BUS_WM_CFG(n)			(0x2208 + (n) * 0x100)
#define		WM_CFG_EN			(0)
#define		WM_CFG_MODE			(1)
#define			MODE_QCOM_PLAIN	(0)
#define			MODE_MIPI_RAW	(1)
#define		WM_CFG_VIRTUALFRAME		(2)
#define VFE_BUS_WM_HEADER_ADDR(n)		(0x220c + (n) * 0x100)
#define VFE_BUS_WM_HEADER_CFG(n)		(0x2210 + (n) * 0x100)
#define VFE_BUS_WM_IMAGE_ADDR(n)		(0x2214 + (n) * 0x100)
#define VFE_BUS_WM_IMAGE_ADDR_OFFSET(n)		(0x2218 + (n) * 0x100)
#define VFE_BUS_WM_BUFFER_WIDTH_CFG(n)		(0x221c + (n) * 0x100)
#define		WM_BUFFER_DEFAULT_WIDTH		(0xFF01)

#define VFE_BUS_WM_BUFFER_HEIGHT_CFG(n)		(0x2220 + (n) * 0x100)
#define VFE_BUS_WM_PACKER_CFG(n)		(0x2224 + (n) * 0x100)

#define VFE_BUS_WM_STRIDE(n)			(0x2228 + (n) * 0x100)
#define		WM_STRIDE_DEFAULT_STRIDE	(0xFF01)

#define VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(n)	(0x2248 + (n) * 0x100)
#define VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(n)	(0x224c + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PERIOD(n)		(0x2250 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PATTERN(n)		(0x2254 + (n) * 0x100)
#define VFE_BUS_WM_FRAME_INC(n)			(0x2258 + (n) * 0x100)
#define VFE_BUS_WM_BURST_LIMIT(n)		(0x225c + (n) * 0x100)

/* Composite group 0 completes when both FULL masters are done */
#define VFE_BUS_COMP_MASK(n)			(0x2010 + (n) * 4)
#define		PIX_COMP_GROUP			0

/* FULL output path write masters, fixed in hardware */
#define VFE_WM_PIX_FULL_Y			3
#define VFE_WM_PIX_FULL_C			4
#define		PACKER_FMT_PLAIN_8_LSB_MSB_10	0xe

/*
 * Clock gating overrides. The Windows camera stack (CamX Titan175IFE) sets all
 * of them before the IQ module setup of every request and never clears them.
 * Without the overrides, DMI writes do not reach the LUT memories: the Gamma16
 * banks keep their power-up contents. The names follow the downstream VFE
 * drivers; the Windows stack only uses the offsets.
 */
#define VFE_CGC_OVERRIDE_LENS			(0x02c)
#define VFE_CGC_OVERRIDE_STATS			(0x030)
#define VFE_CGC_OVERRIDE_COLOR			(0x034)
#define VFE_CGC_OVERRIDE_ZOOM			(0x038)
#define VFE_CGC_OVERRIDE_BUS			(0x03c)
#define VFE_CGC_OVERRIDE_ALL			(~0u)
#define VFE_CGC_OVERRIDE_DMI			(0xfac)

static void vfe_pix_cgc_config(struct vfe_device *vfe)
{
	writel_relaxed(VFE_CGC_OVERRIDE_ALL, vfe->base + VFE_CGC_OVERRIDE_LENS);
	writel_relaxed(VFE_CGC_OVERRIDE_ALL, vfe->base + VFE_CGC_OVERRIDE_STATS);
	writel_relaxed(VFE_CGC_OVERRIDE_ALL, vfe->base + VFE_CGC_OVERRIDE_COLOR);
	writel_relaxed(VFE_CGC_OVERRIDE_ALL, vfe->base + VFE_CGC_OVERRIDE_ZOOM);
	writel_relaxed(VFE_CGC_OVERRIDE_ALL, vfe->base + VFE_CGC_OVERRIDE_BUS);
	writel_relaxed(1, vfe->base + VFE_CGC_OVERRIDE_DMI);
}

/*
 * Gamma16: three curves of 64 linear segments, loaded through the DMI port.
 * The Windows driver writes one LUT bank per channel for every frame and
 * alternates between two banks per channel (26/27, 28/29, 30/31); the bank the
 * hardware reads is chosen by VFE_GAMMA_CFG. Fill all six banks with the same
 * curve and leave VFE_GAMMA_CFG at 0, so the bank choice does not matter.
 *
 * Entry i = node[i] | (node[i + 1] - node[i]) << 10, nodes 0..1023 and the
 * delta a signed 10 bit value. The curve is the gamma16_ife curve of the
 * Samsung S5K3L6 tuning data (all three channels use the same one).
 */
#define VFE_GAMMA_CFG				(0x798)
#define VFE_DMI_CFG				(0xc24)
#define		DMI_CFG_AUTO_INC		BIT(8)
#define VFE_DMI_ADDR				(0xc28)
/* 0xc2c takes the upper half of 64 bit entries, not needed for 32 bit LUTs */
#define VFE_DMI_DATA_LO				(0xc30)
#define VFE_GAMMA_BANK_FIRST			26
#define VFE_GAMMA_BANK_COUNT			6
#define VFE_GAMMA_LUT_ENTRIES			64

static const u32 vfe_gamma16_lut[VFE_GAMMA_LUT_ENTRIES] = {
	0x00010000, 0x0000b040, 0x0000906c, 0x00008090,
	0x000074b0, 0x00006ccd, 0x000068e8, 0x00006102,
	0x0000591a, 0x00005930, 0x00005546, 0x0000515b,
	0x00004d6f, 0x00004d82, 0x00004995, 0x000049a7,
	0x000045b9, 0x000045ca, 0x000041db, 0x000041eb,
	0x000041fb, 0x00003e0b, 0x00003e1a, 0x00003e29,
	0x00003e38, 0x00003a47, 0x00003a55, 0x00003a63,
	0x00003671, 0x0000367e, 0x00003a8b, 0x00003699,
	0x000032a6, 0x000036b2, 0x000032bf, 0x000036cb,
	0x000032d8, 0x000032e4, 0x000032f0, 0x00002efc,
	0x00003307, 0x00002f13, 0x0000331e, 0x00002f2a,
	0x00002f35, 0x00002f40, 0x00002f4b, 0x00002f56,
	0x00002b61, 0x00002f6b, 0x00002b76, 0x00002f80,
	0x00002b8b, 0x00002b95, 0x00002b9f, 0x00002ba9,
	0x00002bb3, 0x00002bbd, 0x00002bc7, 0x00002bd1,
	0x000027db, 0x00002be4, 0x000027ee, 0x000023f7,
};

static void vfe_pix_gamma_config(struct vfe_device *vfe)
{
	unsigned int bank, i;

	for (bank = VFE_GAMMA_BANK_FIRST;
	     bank < VFE_GAMMA_BANK_FIRST + VFE_GAMMA_BANK_COUNT; bank++) {
		writel_relaxed(bank | DMI_CFG_AUTO_INC, vfe->base + VFE_DMI_CFG);
		writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
		/* Zero entries have to be written too */
		for (i = 0; i < VFE_GAMMA_LUT_ENTRIES; i++)
			writel_relaxed(vfe_gamma16_lut[i], vfe->base + VFE_DMI_DATA_LO);
	}
	writel_relaxed(0, vfe->base + VFE_DMI_CFG);

	writel_relaxed(0, vfe->base + VFE_GAMMA_CFG);
}

/*
 * Image parameters of a stream start, until userspace sets its own through the
 * parameters node. The black level is in 14 bit units: 1024 is the usual 64 of
 * 10 bit sensors. The white balance gains (Q7, green = 1.0) were measured with
 * the Hynix Hi-846 of the Huawei MateBook E Go on a white target under indoor
 * lighting.
 */
#define VFE_PIX_BLACK_LEVEL			1024
#define VFE_PIX_WB_GAIN_R			302
#define VFE_PIX_WB_GAIN_B			194

static inline void vfe_reg_set(struct vfe_device *vfe, u32 reg, u32 set_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits | set_bits, vfe->base + reg);
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	u32 reset_bits = GLOBAL_RESET_CMD_CORE		|
			 GLOBAL_RESET_CMD_CAMIF		|
			 GLOBAL_RESET_CMD_BUS		|
			 GLOBAL_RESET_CMD_BUS_BDG	|
			 GLOBAL_RESET_CMD_REGISTER	|
			 GLOBAL_RESET_CMD_TESTGEN	|
			 GLOBAL_RESET_CMD_DSP		|
			 GLOBAL_RESET_CMD_IDLE_CGC	|
			 GLOBAL_RESET_CMD_RDI0		|
			 GLOBAL_RESET_CMD_RDI1		|
			 GLOBAL_RESET_CMD_RDI2		|
			 GLOBAL_RESET_CMD_RDI3;

	writel_relaxed(BIT(31), vfe->base + VFE_IRQ_MASK_0);

	/* Make sure IRQ mask has been written before resetting */
	wmb();

	writel_relaxed(reset_bits, vfe->base + VFE_GLOBAL_RESET_CMD);
}

static void vfe_bus_init(struct vfe_device *vfe)
{
	u32 val;

	/*Set Debug Registers*/
	val = DEBUG_STATUS_CFG_STATUS0(1) |
	      DEBUG_STATUS_CFG_STATUS0(7);
	writel_relaxed(val, vfe->base + VFE_BUS_WM_DEBUG_STATUS_CFG);

	/* BUS_WM_INPUT_IF_ADDR_SYNC_FRAME_HEADER */
	writel_relaxed(0, vfe->base + VFE_BUS_WM_ADDR_SYNC_FRAME_HEADER);

	/* no clock gating at bus input */
	val = WM_CGC_OVERRIDE_ALL;
	writel_relaxed(val, vfe->base + VFE_BUS_WM_CGC_OVERRIDE);

	writel_relaxed(0x0, vfe->base + VFE_BUS_WM_TEST_BUS_CTRL);

	/* if addr_no_sync has default value then config the addr no sync reg */
	val = WM_ADDR_NO_SYNC_DEFAULT_VAL;
	writel_relaxed(val, vfe->base + VFE_BUS_WM_ADDR_SYNC_NO_SYNC);
}

static void vfe_wm_start(struct vfe_device *vfe, u8 wm, struct vfe_line *line)
{
	u32 val;

	vfe_bus_init(vfe);

	writel_relaxed(0xf, vfe->base + VFE_BUS_WM_BURST_LIMIT(wm));

	val = WM_BUFFER_DEFAULT_WIDTH;
	writel_relaxed(val, vfe->base + VFE_BUS_WM_BUFFER_WIDTH_CFG(wm));

	val = 0;
	writel_relaxed(val, vfe->base + VFE_BUS_WM_BUFFER_HEIGHT_CFG(wm));

	val = 0;
	writel_relaxed(val, vfe->base + VFE_BUS_WM_PACKER_CFG(wm)); // XXX 1 for PLAIN8?

	/* Configure stride for RDIs */
	val = WM_STRIDE_DEFAULT_STRIDE;
	writel_relaxed(val, vfe->base + VFE_BUS_WM_STRIDE(wm));

	/* Enable WM */
	val = 1 << WM_CFG_EN |
	      MODE_MIPI_RAW << WM_CFG_MODE;
	writel_relaxed(val, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_wm_stop(struct vfe_device *vfe, u8 wm)
{
	/* Disable WM */
	writel_relaxed(0, vfe->base + VFE_BUS_WM_CFG(wm));
}

static u32 vfe_reg_update_bit(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	if (vfe_line_has_pix_pipeline(vfe, line_id))
		return REG_UPDATE_PIX;

	return REG_UPDATE_RDI(line_id);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	vfe->reg_update |= vfe_reg_update_bit(vfe, line_id);

	/* Enforce ordering between previous reg writes and reg update */
	wmb();

	writel_relaxed(vfe->reg_update, vfe->base + VFE_REG_UPDATE_CMD);

	/* Dual IFE: the slave latches together with the master */
	if (line_id == VFE_LINE_PIX && vfe->dual.master && vfe->dual.peer)
		writel_relaxed(REG_UPDATE_PIX, vfe->dual.peer->base + VFE_REG_UPDATE_CMD);

	/* Enforce ordering between reg update and subsequent reg writes */
	wmb();
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	vfe->reg_update &= ~vfe_reg_update_bit(vfe, line_id);
}

/*
 * Pixel pipeline of the full VFE (VFE_LINE_PIX): CSID IPP -> CAMIF -> DEMUX
 * -> BLS -> DEMOSAIC (+WB) -> GAMMA -> CST -> FULL path MNDS -> CROP ->
 * ROUND/CLAMP -> FULL Y and C write masters, producing NV12 or NV21.
 * Linearization, lens shading, colour correction, tone mapping and statistics
 * stay disabled: this is a fixed parameter pipeline without 3A.
 */

/* Bayer order as used by VFE_CORE_CFG and the demux channel tables */
enum vfe_bayer_pattern {
	VFE_BAYER_RGGB = 0,
	VFE_BAYER_GRBG = 1,
	VFE_BAYER_BGGR = 2,
	VFE_BAYER_GBRG = 3,
};

static int vfe_pix_bayer_pattern(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return VFE_BAYER_RGGB;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
		return VFE_BAYER_GRBG;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR14_1X14:
		return VFE_BAYER_BGGR;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
		return VFE_BAYER_GBRG;
	default:
		return -EINVAL;
	}
}

/* Demux channel select of even and odd lines, per Bayer pattern (CamX) */
static const u8 vfe_demux_line_cfg[4][2] = {
	[VFE_BAYER_RGGB] = { 0xc9, 0xac },
	[VFE_BAYER_GRBG] = { 0x9c, 0xca },
	[VFE_BAYER_BGGR] = { 0xca, 0x9c },
	[VFE_BAYER_GBRG] = { 0xac, 0xc9 },
};

/*
 * RGB to YCbCr, BT.601 full range, 10 bit, as in the CST12 block of the
 * S5K3L6 chromatix tuning. Matrix columns are in G, B, R order (Q10).
 */
static const struct {
	s16 m[3];
	u16 offset_out;
} vfe_cst_bt601_full[3] = {
	{ {  601,  117,  306 },   0 },	/* Y */
	{ { -338,  510, -172 }, 512 },	/* Cb */
	{ { -427,  -83,  510 }, 512 },	/* Cr */
};

static void vfe_pix_cst_config(struct vfe_device *vfe, bool crcb)
{
	unsigned int ch;

	for (ch = 0; ch < 3; ch++) {
		/* NV21 stores Cr first: swap the two chroma channels */
		unsigned int row = (crcb && ch) ? 3 - ch : ch;
		const s16 *m = vfe_cst_bt601_full[row].m;
		u32 val;

		val = ((u32)m[0] & CST_COEFF_MASK) << CST_M0;
		val |= ((u32)m[1] & CST_COEFF_MASK) << CST_M1;
		writel_relaxed(val, vfe->base + VFE_CST_CH_CFG(ch, 0));

		val = ((u32)m[2] & CST_COEFF_MASK) << CST_M2;
		writel_relaxed(val, vfe->base + VFE_CST_CH_CFG(ch, 1));

		val = (u32)vfe_cst_bt601_full[row].offset_out << CST_OFFSET_OUT;
		val |= 0 << CST_OFFSET_IN;
		writel_relaxed(val, vfe->base + VFE_CST_CH_CFG(ch, 2));

		val = 0 << CST_CLAMP_MIN;
		val |= 1023 << CST_CLAMP_MAX;
		writel_relaxed(val, vfe->base + VFE_CST_CH_CFG(ch, 3));
	}
}

static void vfe_pix_demux_demosaic_config(struct vfe_device *vfe,
					  enum vfe_bayer_pattern pattern)
{
	u32 gain = DEMUX_GAIN_UNITY << DEMUX_GAIN_CH0 |
		   DEMUX_GAIN_UNITY << DEMUX_GAIN_CH1;
	unsigned int i;

	/* One pixel pair per period, no black level change */
	writel_relaxed(1 << DEMUX_CFG_PERIOD, vfe->base + VFE_DEMUX_CFG);
	for (i = 0; i < 4; i++)
		writel_relaxed(gain, vfe->base + VFE_DEMUX_GAIN(i));
	writel_relaxed(vfe_demux_line_cfg[pattern][0], vfe->base + VFE_DEMUX_EVEN_CFG);
	writel_relaxed(vfe_demux_line_cfg[pattern][1], vfe->base + VFE_DEMUX_ODD_CFG);

	/*
	 * Directional interpolation needs sensor specific edge weights from
	 * the tuning data. Disable it, which leaves non-directional
	 * interpolation that does not depend on those weights.
	 */
	writel_relaxed(DEMOSAIC_CFG_DIR_G_INTERP_DIS |
		       DEMOSAIC_CFG_DIR_RB_INTERP_DIS,
		       vfe->base + VFE_DEMOSAIC_CFG);
	writel_relaxed(0, vfe->base + VFE_DEMOSAIC_INTERP_COEFF_0);
	writel_relaxed(0, vfe->base + VFE_DEMOSAIC_INTERP_COEFF_1);

	/* The WB gains have no enable bit, green stays at unity */
	writel_relaxed(WB_GAIN_UNITY << WB_CFG_0_G_GAIN |
		       VFE_PIX_WB_GAIN_B << WB_CFG_0_B_GAIN,
		       vfe->base + VFE_WB_CFG_0);
	writel_relaxed(VFE_PIX_WB_GAIN_R << WB_CFG_1_R_GAIN,
		       vfe->base + VFE_WB_CFG_1);
	writel_relaxed(0, vfe->base + VFE_WB_OFFSET_CFG_0);
	writel_relaxed(0, vfe->base + VFE_WB_OFFSET_CFG_1);
}

static void vfe_pix_bls_config(struct vfe_device *vfe)
{
	u32 offset = VFE_PIX_BLACK_LEVEL;
	u32 scale = (2048 * VFE_PIX_MAX) / (VFE_PIX_MAX - offset);

	writel_relaxed((offset & BLS_CFG_OFFSET_MASK) << BLS_CFG_OFFSET |
		       scale << BLS_CFG_SCALE, vfe->base + VFE_BLS_CFG);
	writel_relaxed(0, vfe->base + VFE_BLS_THRESHOLD_0);
	writel_relaxed(0, vfe->base + VFE_BLS_THRESHOLD_1);
}

static void vfe_pix_camif_config(struct vfe_device *vfe, u32 width, u32 height)
{
	u32 val;

	writel_relaxed(CMD_CLEAR_CAMIF_STATUS, vfe->base + VFE_CAMIF_CMD);

	/* No PDAF line/pixel skipping, interrupt on every frame */
	writel_relaxed(0, vfe->base + VFE_CAMIF_LINE_SKIP_PATTERN);
	writel_relaxed(0, vfe->base + VFE_CAMIF_PIXEL_SKIP_PATTERN);
	writel_relaxed(CAMIF_SKIP_PERIOD_DEFAULT, vfe->base + VFE_CAMIF_SKIP_PERIOD);
	writel_relaxed(1, vfe->base + VFE_CAMIF_IRQ_SUBSAMPLE_PATTERN);

	/* Raw crop window: the whole sensor frame */
	val = 0 << RAW_CROP_FIRST | ((width - 1) & RAW_CROP_MASK) << RAW_CROP_LAST;
	writel_relaxed(val, vfe->base + VFE_CAMIF_RAW_CROP_WIDTH_CFG);
	val = 0 << RAW_CROP_FIRST | ((height - 1) & RAW_CROP_MASK) << RAW_CROP_LAST;
	writel_relaxed(val, vfe->base + VFE_CAMIF_RAW_CROP_HEIGHT_CFG);

	writel_relaxed(CFG_VFE_OUTPUT_EN | CFG_BUS_OUTPUT_EN | CFG_RAW_CROP_EN,
		       vfe->base + VFE_CAMIF_CFG);
}

/* MNDS16 interpolation resolution for a downscale ratio (CamX) */
static u32 vfe_scale_interp_reso(u32 in, u32 out)
{
	u32 ratio = in / out;

	if (ratio < 4)
		return 3;
	if (ratio < 8)
		return 2;
	if (ratio < 16)
		return 1;
	return 0;
}

static void vfe_pix_scale_config(struct vfe_device *vfe, u32 base,
				 u32 in_w, u32 in_h, u32 out_w, u32 out_h)
{
	u32 reso, phase, val;

	writel_relaxed(SCALE_CFG_H_EN | SCALE_CFG_V_EN, vfe->base + base + SCALE_CFG);

	val = ((in_w - 1) & SCALE_SIZE_MASK) << SCALE_SIZE_IN;
	val |= ((out_w - 1) & SCALE_SIZE_MASK) << SCALE_SIZE_OUT;
	writel_relaxed(val, vfe->base + base + SCALE_H_IMAGE_SIZE);
	reso = vfe_scale_interp_reso(in_w, out_w);
	phase = (in_w << (reso + 14)) / out_w;
	val = (phase & SCALE_PHASE_MULT_MASK) << SCALE_PHASE_MULT;
	val |= reso << SCALE_PHASE_INTERP;
	writel_relaxed(val, vfe->base + base + SCALE_H_PHASE);
	writel_relaxed(0, vfe->base + base + SCALE_H_STRIPE);
	writel_relaxed(0, vfe->base + base + SCALE_H_INIT_PHASE);
	writel_relaxed((in_w - 1) & SCALE_SIZE_MASK, vfe->base + base + SCALE_H_PAD);

	val = ((in_h - 1) & SCALE_SIZE_MASK) << SCALE_SIZE_IN;
	val |= ((out_h - 1) & SCALE_SIZE_MASK) << SCALE_SIZE_OUT;
	writel_relaxed(val, vfe->base + base + SCALE_V_IMAGE_SIZE);
	reso = vfe_scale_interp_reso(in_h, out_h);
	phase = (in_h << (reso + 14)) / out_h;
	val = (phase & SCALE_PHASE_MULT_MASK) << SCALE_PHASE_MULT;
	val |= reso << SCALE_PHASE_INTERP;
	writel_relaxed(val, vfe->base + base + SCALE_V_PHASE);
	writel_relaxed(0, vfe->base + base + SCALE_V_STRIPE);
	writel_relaxed(0, vfe->base + base + SCALE_V_INIT_PHASE);
	writel_relaxed((in_h - 1) & SCALE_SIZE_MASK, vfe->base + base + SCALE_V_PAD);
}

/*
 * Crop the scaler output. The chroma plane has the luma width in samples
 * (interleaved CbCr) and half the lines.
 */
static void vfe_pix_crop_config(struct vfe_device *vfe, const struct v4l2_rect *crop)
{
	u32 first_pixel = crop->left;
	u32 last_pixel = crop->left + crop->width - 1;
	u32 val;

	val = (crop->top & CROP_LINE_MASK) << CROP_FIRST;
	val |= ((crop->top + crop->height - 1) & CROP_LINE_MASK) << CROP_LAST;
	writel_relaxed(val, vfe->base + VFE_FULL_Y_CROP_LINE_CFG);
	val = (first_pixel & CROP_PIXEL_MASK) << CROP_FIRST;
	val |= (last_pixel & CROP_PIXEL_MASK) << CROP_LAST;
	writel_relaxed(val, vfe->base + VFE_FULL_Y_CROP_PIXEL_CFG);

	val = ((crop->top / 2) & CROP_LINE_MASK) << CROP_FIRST;
	val |= (((crop->top + crop->height) / 2 - 1) & CROP_LINE_MASK) << CROP_LAST;
	writel_relaxed(val, vfe->base + VFE_FULL_C_CROP_LINE_CFG);
	val = (first_pixel & CROP_PIXEL_MASK) << CROP_FIRST;
	val |= (last_pixel & CROP_PIXEL_MASK) << CROP_LAST;
	writel_relaxed(val, vfe->base + VFE_FULL_C_CROP_PIXEL_CFG);

	writel_relaxed(CLAMP_CFG_8BIT, vfe->base + VFE_FULL_Y_CLAMP_CFG);
	writel_relaxed(ROUND_CFG_Y_8BIT, vfe->base + VFE_FULL_Y_ROUND_CFG);
	writel_relaxed(CLAMP_CFG_8BIT, vfe->base + VFE_FULL_C_CLAMP_CFG);
	writel_relaxed(ROUND_CFG_C_8BIT, vfe->base + VFE_FULL_C_ROUND_CFG);

	val = FULL_MODULE_CFG_CROP_EN | FULL_MODULE_CFG_ROUND_EN | FULL_MODULE_CFG_CLAMP_EN;
	writel_relaxed(val, vfe->base + VFE_FULL_Y_MODULE_CFG);
	writel_relaxed(val, vfe->base + VFE_FULL_C_MODULE_CFG);
}

/* Line based write master of the FULL path, left disabled */
static void vfe_pix_wm_config(struct vfe_device *vfe, u8 wm, u32 width,
			      u32 height, u32 stride)
{
	/* Keep every frame */
	writel_relaxed(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));

	writel_relaxed(width, vfe->base + VFE_BUS_WM_BUFFER_WIDTH_CFG(wm));
	writel_relaxed(height, vfe->base + VFE_BUS_WM_BUFFER_HEIGHT_CFG(wm));
	writel_relaxed(PACKER_FMT_PLAIN_8_LSB_MSB_10, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));
	writel_relaxed(stride, vfe->base + VFE_BUS_WM_STRIDE(wm));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_IMAGE_ADDR_OFFSET(wm));
	writel_relaxed(stride * height, vfe->base + VFE_BUS_WM_FRAME_INC(wm));
	writel_relaxed(0xf, vfe->base + VFE_BUS_WM_BURST_LIMIT(wm));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_CFG(wm));
}

static void vfe_pix_wm_enable(struct vfe_device *vfe)
{
	u32 val = 1 << WM_CFG_EN | MODE_QCOM_PLAIN << WM_CFG_MODE;

	writel_relaxed(val, vfe->base + VFE_BUS_WM_CFG(VFE_WM_PIX_FULL_Y));
	writel_relaxed(val, vfe->base + VFE_BUS_WM_CFG(VFE_WM_PIX_FULL_C));
}

/* Program both planes of a single buffer NV12/NV21 frame */
static void vfe_pix_wm_update(struct vfe_device *vfe, u32 addr,
			      struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;

	writel_relaxed(addr, vfe->base + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_PIX_FULL_Y));
	writel_relaxed(addr + stride * pix->height,
		       vfe->base + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_PIX_FULL_C));

	/* Dual IFE: the slave writes its stripe into the same buffer */
	if (vfe->dual.master && vfe->dual.peer) {
		void __iomem *s = vfe->dual.peer->base;
		u32 off = vfe->dual.x_offset;

		writel_relaxed(addr + off, s + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_PIX_FULL_Y));
		writel_relaxed(addr + stride * pix->height + off,
			       s + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_PIX_FULL_C));
	}
}

/*
 * Dual IFE: a pixel line whose output is also linked to the video node of
 * another VFE's pixel line writes its stripe into that node's buffers.
 */
static struct vfe_line *vfe_pix_dual_master(struct vfe_line *line)
{
	struct media_entity *entity = &line->subdev.entity;
	struct media_link *link;

	for_each_media_entity_data_link(entity, link) {
		struct media_entity *sink = link->sink->entity;
		struct camss_video *video;

		if (link->source->entity != entity ||
		    !(link->flags & MEDIA_LNK_FL_ENABLED) ||
		    !is_media_entity_v4l2_video_device(sink))
			continue;
		video = container_of(media_entity_to_video_device(sink),
				     struct camss_video, vdev);
		if (video != &line->video_out)
			return container_of(video, struct vfe_line, video_out);
	}

	return NULL;
}

/*
 * Master's output_lock held, the slave wrote a stripe at @ts: the held buffer
 * of that frame is complete, older ones lack the slave's stripe (it dropped
 * the frame). Pairing by time keeps the frames matched after a drop on
 * either side.
 */
static unsigned int vfe_pix_dual_collect(struct vfe_pix_dual *d, u64 ts,
					 struct camss_buffer **done, bool *error)
{
	/* error[] comes in as the error state of the slave's stripe */
	bool stripe_error = error[0];

	unsigned int i = 0, n = 0;

	while (i < d->nheld) {
		u64 t = d->held[i].ts;

		if (t + VFE_DUAL_MATCH_NS > ts) {
			if (t < ts + VFE_DUAL_MATCH_NS) {
				error[n] = d->held[i].error || stripe_error;
				done[n++] = d->held[i].buf;
			} else {
				i++;
				continue;
			}
		} else {
			error[n] = true;
			done[n++] = d->held[i].buf;
		}
		memmove(&d->held[i], &d->held[i + 1], (--d->nheld - i) * sizeof(d->held[0]));
	}

	return n;
}

/* Master's output_lock held: end the pairing, return what was waiting */
static unsigned int vfe_pix_dual_end(struct vfe_device *master,
				     struct camss_buffer **done)
{
	struct vfe_pix_dual *d = &master->dual;
	unsigned int i, n = d->nheld;

	for (i = 0; i < n; i++)
		done[i] = d->held[i].buf;
	memset(d, 0, sizeof(*d));

	return n;
}

/* The slave wrote its stripe of a frame */
static void vfe_pix_dual_slave_done(struct vfe_device *vfe)
{
	struct camss_buffer *done[VFE_DUAL_HELD];
	bool error[VFE_DUAL_HELD];
	struct vfe_device *master;
	unsigned long flags;
	unsigned int i, n = 0;
	u64 ts = ktime_get_ns();

	spin_lock_irqsave(&vfe->output_lock, flags);
	/* A corrupt stripe makes the master's buffer of that frame corrupt */
	error[0] = vfe->line[VFE_LINE_PIX].output.error;
	vfe->line[VFE_LINE_PIX].output.error = false;
	master = vfe->dual.peer;
	if (master) {
		spin_lock_nested(&master->output_lock, SINGLE_DEPTH_NESTING);
		if (master->dual.peer == vfe) {
			master->dual.slave_ts = ts;
			master->dual.slave_error = error[0];
			n = vfe_pix_dual_collect(&master->dual, ts, done, error);
		}
		spin_unlock(&master->output_lock);
	}
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	for (i = 0; i < n; i++) {
		if (error[i])
			dev_dbg_ratelimited(vfe->camss->dev,
					    "VFE%d: frame %u incomplete or corrupt\n", vfe->id,
					    done[i]->vb.sequence);
		vb2_buffer_done(&done[i]->vb.vb2_buf,
				error[i] ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	}
}

static int vfe_pix_configure(struct vfe_device *vfe, struct vfe_line *line)
{
	struct v4l2_mbus_framefmt *sink = &line->fmt[MSM_VFE_PAD_SINK];
	struct v4l2_mbus_framefmt *src = &line->fmt[MSM_VFE_PAD_SRC];
	struct v4l2_pix_format_mplane *pix = &line->video_out.active_fmt.fmt.pix_mp;
	const struct v4l2_rect *compose = &line->compose;
	const struct v4l2_rect *crop = &line->crop;
	u32 stride = pix->plane_fmt[0].bytesperline;
	bool crcb = src->code == MEDIA_BUS_FMT_YVYU8_1_5X8;
	int pattern;

	/* Dual IFE slave: the buffers are those of the master's video node */
	if (vfe->dual.peer && !vfe->dual.master)
		stride = vfe->dual.peer->line[VFE_LINE_PIX].video_out.active_fmt.fmt.pix_mp
				.plane_fmt[0].bytesperline;

	pattern = vfe_pix_bayer_pattern(sink->code);
	if (pattern < 0) {
		dev_err(vfe->camss->dev, "VFE%d: pixel path needs Bayer input, got 0x%04x\n",
			vfe->id, sink->code);
		return pattern;
	}

	vfe_bus_init(vfe);

	writel_relaxed(pattern | CFG_BIT24, vfe->base + VFE_CORE_CFG);

	vfe_pix_camif_config(vfe, sink->width, sink->height);
	vfe_pix_demux_demosaic_config(vfe, pattern);
	vfe_pix_bls_config(vfe);
	vfe_pix_cst_config(vfe, crcb);
	vfe_pix_cgc_config(vfe);
	vfe_pix_gamma_config(vfe);

	/* 4:4:4 in, luma at compose size, chroma at half size in both axes */
	vfe_pix_scale_config(vfe, VFE_SCALE_FULL_Y_BASE, sink->width, sink->height,
			     compose->width, compose->height);
	vfe_pix_scale_config(vfe, VFE_SCALE_FULL_C_BASE, sink->width, sink->height,
			     compose->width / 2, compose->height / 2);
	vfe_pix_crop_config(vfe, crop);

	writel_relaxed(MODULE_LENS_EN_DEMUX | MODULE_LENS_EN_DEMOSAIC |
		       MODULE_LENS_EN_BLS, vfe->base + VFE_MODULE_LENS_EN);
	writel_relaxed(0, vfe->base + VFE_MODULE_STATS_EN);
	writel_relaxed(MODULE_COLOR_EN_GAMMA, vfe->base + VFE_MODULE_COLOR_EN);
	writel_relaxed(MODULE_ZOOM_EN_CST | MODULE_ZOOM_EN_SCALE_FULL |
		       MODULE_ZOOM_EN_CROP_FULL, vfe->base + VFE_MODULE_ZOOM_EN);

	vfe_pix_wm_config(vfe, VFE_WM_PIX_FULL_Y, crop->width, crop->height, stride);
	vfe_pix_wm_config(vfe, VFE_WM_PIX_FULL_C, crop->width, crop->height / 2, stride);
	writel_relaxed(BIT(VFE_WM_PIX_FULL_Y) | BIT(VFE_WM_PIX_FULL_C),
		       vfe->base + VFE_BUS_COMP_MASK(PIX_COMP_GROUP));

	vfe17x_meta_start(vfe, sink->width, sink->height);

	return 0;
}

static int vfe_pix_enable_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = &line->output;
	struct vfe_line *mline;
	unsigned long flags;
	unsigned int i;
	int ret;

	spin_lock_irqsave(&vfe->output_lock, flags);

	vfe_reg_update_clear(vfe, line->id);

	if (output->state > VFE_OUTPUT_RESERVED) {
		dev_err(vfe->camss->dev, "Output is not in reserved state %d\n",
			output->state);
		ret = -EINVAL;
		goto out_unlock;
	}

	WARN_ON(output->gen2.active_num);

	/*
	 * Dual IFE slave: pair with the master before configuring, and leave
	 * the write masters off: the master gives them its buffers.
	 */
	mline = vfe_pix_dual_master(line);
	if (mline) {
		struct vfe_device *mvfe = to_vfe(mline);

		memset(&vfe->dual, 0, sizeof(vfe->dual));
		vfe->dual.peer = mvfe;
		spin_lock_nested(&mvfe->output_lock, SINGLE_DEPTH_NESTING);
		memset(&mvfe->dual, 0, sizeof(mvfe->dual));
		mvfe->dual.peer = vfe;
		mvfe->dual.master = true;
		mvfe->dual.x_offset = mline->crop.width;
		spin_unlock(&mvfe->output_lock);

		ret = vfe_pix_configure(vfe, line);
		if (ret < 0)
			goto out_unlock;
		output->state = VFE_OUTPUT_ON;
		output->armed_next = false;
		output->error = false;
		output->sequence = 0;
		vfe_reg_update(vfe, line->id);
		dev_dbg(vfe->camss->dev, "VFE%d: pixel path stripe of VFE%d at %u\n",
			vfe->id, mvfe->id, mvfe->dual.x_offset);
		goto out_unlock;
	}

	ret = vfe_pix_configure(vfe, line);
	if (ret < 0)
		goto out_unlock;

	output->state = VFE_OUTPUT_ON;
	output->armed_next = false;
	output->error = false;
	output->sequence = 0;
	output->wait_reg_update = 0;
	reinit_completion(&output->reg_update);

	for (i = 0; i < CAMSS_INIT_BUF_COUNT; i++) {
		output->buf[i] = vfe_buf_get_pending(output);
		if (!output->buf[i])
			break;
		output->gen2.active_num++;
		vfe_pix_wm_update(vfe, output->buf[i]->addr[0], line);
	}

	vfe_pix_wm_enable(vfe);
	if (vfe->dual.master && vfe->dual.peer)
		vfe_pix_wm_enable(vfe->dual.peer);

	/* Latch the pipeline configuration and the first buffers */
	vfe_reg_update(vfe, line->id);

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return ret;
}

static void vfe_wm_update(struct vfe_device *vfe, u8 wm, u32 addr,
			  struct vfe_line *line)
{
	struct v4l2_pix_format_mplane *pix =
		&line->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;

	if (vfe_line_has_pix_pipeline(vfe, line->id)) {
		vfe_pix_wm_update(vfe, addr, line);
		return;
	}

	writel_relaxed(addr, vfe->base + VFE_BUS_WM_IMAGE_ADDR(wm));
	writel_relaxed(stride * pix->height, vfe->base + VFE_BUS_WM_FRAME_INC(wm));
}

static void vfe_enable_irq_common(struct vfe_device *vfe)
{
	vfe_reg_set(vfe, VFE_IRQ_MASK_0, ~0u);
	vfe_reg_set(vfe, VFE_IRQ_MASK_1, ~0u);

	writel_relaxed(~0u, vfe->base + VFE_BUS_IRQ_MASK(0));
	writel_relaxed(~0u, vfe->base + VFE_BUS_IRQ_MASK(1));
	writel_relaxed(~0u, vfe->base + VFE_BUS_IRQ_MASK(2));
}

static void vfe_isr_halt_ack(struct vfe_device *vfe)
{
	complete(&vfe->halt_complete);
}

static void vfe_isr_read(struct vfe_device *vfe, u32 *status0, u32 *status1)
{
	*status0 = readl_relaxed(vfe->base + VFE_IRQ_STATUS_0);
	*status1 = readl_relaxed(vfe->base + VFE_IRQ_STATUS_1);

	writel_relaxed(*status0, vfe->base + VFE_IRQ_CLEAR_0);
	writel_relaxed(*status1, vfe->base + VFE_IRQ_CLEAR_1);

	/* Enforce ordering between IRQ Clear and Global IRQ Clear */
	wmb();
	writel_relaxed(CMD_GLOBAL_CLEAR, vfe->base + VFE_IRQ_CMD);
}

static void vfe_violation_read(struct vfe_device *vfe)
{
	u32 violation = readl_relaxed(vfe->base + VFE_VIOLATION_STATUS);

	pr_err_ratelimited("VFE: violation = 0x%08x\n", violation);
}

/*
 * The VFE is wedged, measured on SC8280XP: after one frame with a bus error
 * the pixel path writes nothing more until the stream restarts. Make every
 * queue it feeds fail, once per stream start, so that userspace restarts
 * them; with dual IFE, the master's queues hold both stripes.
 */
static void vfe17x_fatal_error(struct vfe_device *vfe)
{
	struct vfe_device *master = NULL;
	unsigned long flags;
	unsigned int i;
	bool first;

	spin_lock_irqsave(&vfe->output_lock, flags);
	first = !vfe->errors.fatal;
	vfe->errors.fatal = true;
	if (first) {
		for (i = 0; i < vfe->res->line_num; i++)
			if (vfe->line[i].output.state == VFE_OUTPUT_ON)
				vb2_queue_error(&vfe->line[i].video_out.vb2_q);
		if (vfe->dual.peer && !vfe->dual.master)
			master = vfe->dual.peer;
	}
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	if (!first)
		return;

	vfe17x_meta_fatal(vfe);
	dev_err(vfe->camss->dev, "VFE%d: wedged, the streams must restart\n", vfe->id);
	if (master)
		vfe17x_fatal_error(master);
}

/*
 * An error interrupt: the frames being written are corrupt and the VFE is
 * wedged (see vfe17x_fatal_error()). Every line streaming marks its buffer.
 * A storm of them turns the error interrupts off until the next stream
 * start.
 */
static void vfe_isr_error(struct vfe_device *vfe, u32 err0, u32 err1,
			  u32 bus0, u32 bus2)
{
	struct vfe_errors *e = &vfe->errors;
	u64 now = ktime_get_ns();
	unsigned int i;

	for (i = 0; i < vfe->res->line_num; i++)
		vfe_line_frame_error(&vfe->line[i]);
	vfe17x_fatal_error(vfe);

	e->count++;
	dev_err_ratelimited(vfe->camss->dev,
			    "VFE%d: error irq %08x %08x bus %08x %08x, violation %08x debug %08x comp error %08x overwrite %08x, wm %08x %08x\n",
			    vfe->id, err0, err1, bus0, bus2,
			    readl_relaxed(vfe->base + VFE_VIOLATION_STATUS),
			    readl_relaxed(vfe->base + VFE_BUS_DEBUG_STATUS0),
			    readl_relaxed(vfe->base + VFE_BUS_COMP_ERROR_STATUS),
			    readl_relaxed(vfe->base + VFE_BUS_COMP_OVERWRITE_STATUS),
			    readl_relaxed(vfe->base + VFE_BUS_WM_STATUS0(VFE_WM_PIX_FULL_Y)),
			    readl_relaxed(vfe->base + VFE_BUS_WM_STATUS1(VFE_WM_PIX_FULL_Y)));

	if (now - e->window > VFE_ERR_WINDOW_NS) {
		e->window = now;
		e->window_count = 0;
	}
	if (++e->window_count > VFE_ERR_STORM && !e->off) {
		e->off = true;
		writel_relaxed(readl_relaxed(vfe->base + VFE_IRQ_MASK_0) & ~VFE_ERR_MASK_0,
			       vfe->base + VFE_IRQ_MASK_0);
		writel_relaxed(readl_relaxed(vfe->base + VFE_IRQ_MASK_1) & ~VFE_ERR_MASK_1,
			       vfe->base + VFE_IRQ_MASK_1);
		writel_relaxed(readl_relaxed(vfe->base + VFE_BUS_IRQ_MASK(0)) & ~VFE_BUS_ERR_MASK_0,
			       vfe->base + VFE_BUS_IRQ_MASK(0));
		writel_relaxed(readl_relaxed(vfe->base + VFE_BUS_IRQ_MASK(2)) & ~VFE_BUS_ERR_MASK_2,
			       vfe->base + VFE_BUS_IRQ_MASK(2));
		dev_err(vfe->camss->dev,
			"VFE%d: %u error interrupts within %u ms, turned off until the next start (%u so far)\n",
			vfe->id, e->window_count, (u32)(VFE_ERR_WINDOW_NS / NSEC_PER_MSEC),
			e->count);
	}
}

/* Status 0 bit reporting a completed reg update of a line */
static u32 vfe_reg_update_done_bit(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	if (vfe_line_has_pix_pipeline(vfe, line_id))
		return STATUS_0_CAMIF_REG_UPDATE;

	return STATUS_0_RDI_REG_UPDATE(line_id);
}

/* The FULL masters complete through composite group 0, not per master */
static bool vfe_wm_is_pix(struct vfe_device *vfe, u8 wm)
{
	return (wm == VFE_WM_PIX_FULL_Y || wm == VFE_WM_PIX_FULL_C) &&
	       vfe_line_has_pix_pipeline(vfe, VFE_LINE_PIX);
}

/*
 * Give a line the buffer of the next frame, if it has none yet. Output lock
 * held.
 */
static bool vfe17x_arm_line(struct vfe_device *vfe, struct vfe_line *line)
{
	struct vfe_output *output = &line->output;
	struct camss_buffer *buf;

	if (output->armed_next || output->gen2.active_num >= 2)
		return false;
	buf = vfe_buf_get_pending(output);
	if (!buf)
		return false;

	output->buf[output->gen2.active_num++] = buf;
	vfe_wm_update(vfe, output->wm_idx[0], buf->addr[0], line);
	vfe_reg_update(vfe, line->id);
	output->armed_next = true;

	return true;
}

/*
 * The buffers of the next frame. The image path decides: the RDI lines of the
 * VFE get one only for a frame the image path writes, so that the RAW and
 * the processed image of a request are one frame. Output lock held. Returns
 * whether the image path writes the next frame.
 */
static bool vfe17x_arm_next(struct vfe_device *vfe)
{
	struct vfe_line *pix = &vfe->line[VFE_LINE_PIX];
	bool pix_next = false;
	unsigned int i;

	if (pix->output.state == VFE_OUTPUT_ON) {
		vfe17x_arm_line(vfe, pix);
		pix_next = pix->output.armed_next;
	}

	for (i = 0; i < vfe->res->line_num; i++) {
		struct vfe_line *line = &vfe->line[i];

		if (line->id != VFE_LINE_PIX && line->output.state == VFE_OUTPUT_ON &&
		    pix_next)
			vfe17x_arm_line(vfe, line);
	}

	return pix_next;
}

/*
 * Epoch k of the pixel path: settle the buffers of frame k + 1, as its
 * parameters go in now too. While the pixel path runs, buffers are queued
 * for a frame only here or, until the end of frame k, when they come
 * (vfe17x_queue_buffer()): a frame the image path skips for want of a buffer
 * then takes no request's buffers, and the parameters, statistics, image and
 * RDI buffers of a request stay on one frame. Returns whether the image path
 * writes frame k + 1.
 */
static bool vfe17x_epoch_refill(struct vfe_device *vfe)
{
	bool next_written;
	unsigned int i;

	spin_lock(&vfe->output_lock);
	/* A dual IFE slave writes the master's buffers */
	if (vfe->dual.peer && !vfe->dual.master) {
		spin_unlock(&vfe->output_lock);
		return false;
	}
	/* Frame k uses one buffer: a second one out is frame k + 1's */
	for (i = 0; i < vfe->res->line_num; i++)
		vfe->line[i].output.armed_next = vfe->line[i].output.gen2.active_num == 2;
	next_written = vfe17x_arm_next(vfe);
	spin_unlock(&vfe->output_lock);

	return next_written;
}

/*
 * While the pixel path runs, a buffer goes to its line for the next frame
 * when that frame still lacks one and can still take it (until the end of
 * the current frame), and otherwise at the next epoch.
 */
static int vfe17x_queue_buffer(struct camss_video *vid, struct camss_buffer *buf)
{
	struct vfe_line *line = container_of(vid, struct vfe_line, video_out);
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_line *pix = &vfe->line[VFE_LINE_PIX];
	unsigned long flags;

	if (!vfe17x_meta_running(vfe))
		return vfe_queue_buffer_v2(vid, buf);

	spin_lock_irqsave(&vfe->output_lock, flags);
	vfe_buf_add_pending(&line->output, buf);
	if (!(vfe->dual.peer && !vfe->dual.master)) {
		vfe17x_meta_lock(vfe);
		if (vfe17x_meta_join_open_locked(vfe)) {
			bool was_next = pix->output.armed_next;

			if (vfe17x_arm_next(vfe) && !was_next && vfe17x_meta_join_locked(vfe))
				vfe_reg_update(vfe, VFE_LINE_PIX);
		}
		vfe17x_meta_unlock(vfe);
	}
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

/*
 * vfe_isr - VFE module interrupt handler
 * @irq: Interrupt line
 * @dev: VFE device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t vfe_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 status0, status1, vfe_bus_status[3];
	int i, wm;

	status0 = readl_relaxed(vfe->base + VFE_IRQ_STATUS_0);
	status1 = readl_relaxed(vfe->base + VFE_IRQ_STATUS_1);

	writel_relaxed(status0, vfe->base + VFE_IRQ_CLEAR_0);
	writel_relaxed(status1, vfe->base + VFE_IRQ_CLEAR_1);

	/* The bus has three IRQ status registers, independent of line_num */
	for (i = 0; i < ARRAY_SIZE(vfe_bus_status); i++) {
		vfe_bus_status[i] = readl_relaxed(vfe->base + VFE_BUS_IRQ_STATUS(i));
		writel_relaxed(vfe_bus_status[i], vfe->base + VFE_BUS_IRQ_CLEAR(i));
	}

	/* Enforce ordering between IRQ reading and interpretation */
	wmb();

	writel_relaxed(CMD_GLOBAL_CLEAR, vfe->base + VFE_IRQ_CMD);
	writel_relaxed(1, vfe->base + VFE_BUS_IRQ_CLEAR_GLOBAL);

	if (status0 & STATUS_0_RESET_ACK)
		vfe->isr_ops.reset_ack(vfe);

	if ((status0 & VFE_ERR_MASK_0) || (status1 & VFE_ERR_MASK_1) ||
	    (vfe_bus_status[0] & VFE_BUS_ERR_MASK_0) ||
	    (vfe_bus_status[2] & VFE_BUS_ERR_MASK_2))
		vfe_isr_error(vfe, status0 & VFE_ERR_MASK_0, status1 & VFE_ERR_MASK_1,
			      vfe_bus_status[0] & VFE_BUS_ERR_MASK_0,
			      vfe_bus_status[2] & VFE_BUS_ERR_MASK_2);

	if (status0 & BIT(9))
		vfe17x_meta_bus(vfe, vfe_bus_status[1]);

	for (i = VFE_LINE_RDI0; i < vfe->res->line_num; i++)
		if (status0 & vfe_reg_update_done_bit(vfe, i))
			vfe->isr_ops.reg_update(vfe, i);

	for (i = VFE_LINE_RDI0; i < vfe->res->line_num; i++)
		if (status0 & STATUS_1_RDI_SOF(i))
			vfe->isr_ops.sof(vfe, i);

	for (i = 0; i < MSM_VFE_COMPOSITE_IRQ_NUM; i++)
		if (vfe_bus_status[0] & STATUS0_COMP_BUF_DONE(i))
			vfe->isr_ops.comp_done(vfe, i);

	for (wm = 0; wm < MSM_VFE_IMAGE_MASTERS_NUM; wm++)
		if (status0 & BIT(9))
			if ((vfe_bus_status[1] & STATUS1_WM_CLIENT_BUF_DONE(wm)) &&
			    !vfe_wm_is_pix(vfe, wm))
				vfe->isr_ops.wm_done(vfe, wm);

	/*
	 * Epoch 0 of the pixel path: program the next frame's parameters.
	 * After the buffers done in the same interrupt: those belong to an
	 * earlier frame and are numbered by the epochs before this one.
	 */
	if (status0 & STATUS_0_CAMIF_EPOCH0) {
		bool next_written = vfe17x_epoch_refill(vfe);

		if (vfe17x_meta_epoch(vfe, next_written)) {
			spin_lock(&vfe->output_lock);
			vfe_reg_update(vfe, VFE_LINE_PIX);
			spin_unlock(&vfe->output_lock);
		}
	}

	return IRQ_HANDLED;
}

/*
 * vfe_halt - Trigger halt on VFE module and wait to complete
 * @vfe: VFE device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_halt(struct vfe_device *vfe)
{
	/* rely on vfe_disable_output() to stop the VFE */
	return 0;
}

static int vfe_get_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output;
	unsigned long flags;
	int wm_idx;

	spin_lock_irqsave(&vfe->output_lock, flags);

	output = &line->output;
	if (output->state > VFE_OUTPUT_RESERVED) {
		dev_err(vfe->camss->dev, "Output is running\n");
		goto error;
	}

	output->wm_num = 1;

	if (vfe_line_has_pix_pipeline(vfe, line->id)) {
		/* NV12/NV21 use both FULL path masters, fixed in hardware */
		if (vfe->wm_output_map[VFE_WM_PIX_FULL_Y] != VFE_LINE_NONE ||
		    vfe->wm_output_map[VFE_WM_PIX_FULL_C] != VFE_LINE_NONE) {
			dev_err(vfe->camss->dev, "FULL path write masters are busy\n");
			goto error;
		}

		output->wm_num = 2;
		output->wm_idx[0] = VFE_WM_PIX_FULL_Y;
		output->wm_idx[1] = VFE_WM_PIX_FULL_C;
		vfe->wm_output_map[VFE_WM_PIX_FULL_Y] = line->id;
		vfe->wm_output_map[VFE_WM_PIX_FULL_C] = line->id;
		output->drop_update_idx = 0;

		spin_unlock_irqrestore(&vfe->output_lock, flags);

		return 0;
	}

	wm_idx = vfe_reserve_wm(vfe, line->id);
	if (wm_idx < 0) {
		dev_err(vfe->camss->dev, "Can not reserve wm\n");
		goto error_get_wm;
	}
	output->wm_idx[0] = wm_idx;

	output->drop_update_idx = 0;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;

error_get_wm:
	vfe_release_wm(vfe, output->wm_idx[0]);
	output->state = VFE_OUTPUT_OFF;
error:
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return -EINVAL;
}

/*
 * vfe_enable - Enable streaming on VFE line
 * @line: VFE line
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_enable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	int ret;

	mutex_lock(&vfe->stream_lock);

	if (!vfe->stream_count) {
		memset(&vfe->errors, 0, sizeof(vfe->errors));
		vfe_enable_irq_common(vfe);
	}

	vfe->stream_count++;

	mutex_unlock(&vfe->stream_lock);

	ret = vfe_get_output(line);
	if (ret < 0)
		goto error_get_output;

	if (vfe_line_has_pix_pipeline(vfe, line->id))
		ret = vfe_pix_enable_output(line);
	else
		ret = vfe_enable_output_v2(line);
	if (ret < 0)
		goto error_enable_output;

	vfe->was_streaming = 1;

	return 0;

error_enable_output:
	vfe_put_output(line);

error_get_output:
	mutex_lock(&vfe->stream_lock);

	vfe->stream_count--;

	mutex_unlock(&vfe->stream_lock);

	return ret;
}

/*
 * vfe_isr_sof - Process start of frame interrupt
 * @vfe: VFE Device
 * @line_id: VFE line
 */
static void vfe_isr_sof(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	/* nop */
}

/*
 * vfe_isr_reg_update - Process reg update interrupt
 * @vfe: VFE Device
 * @line_id: VFE line
 */
static void vfe_isr_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	struct vfe_output *output;
	unsigned long flags;

	spin_lock_irqsave(&vfe->output_lock, flags);
	vfe->res->hw_ops->reg_update_clear(vfe, line_id);

	output = &vfe->line[line_id].output;

	if (output->wait_reg_update) {
		output->wait_reg_update = 0;
		complete(&output->reg_update);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

/*
 * vfe_isr_wm_done - Process write master done interrupt
 * @vfe: VFE Device
 * @wm: Write master id
 */
static void vfe_isr_wm_done(struct vfe_device *vfe, u8 wm)
{
	struct vfe_line *line = &vfe->line[vfe->wm_output_map[wm]];
	struct camss_buffer *ready_buf, *done[VFE_DUAL_HELD + 1];
	bool error[VFE_DUAL_HELD + 1] = { };
	struct vfe_output *output;
	unsigned long flags;
	unsigned int i, ndone = 0;
	bool dual, bad;
	u32 index;
	u64 ts = ktime_get_ns();

	if (vfe->wm_output_map[wm] == VFE_LINE_PIX && vfe->dual.peer && !vfe->dual.master) {
		vfe_pix_dual_slave_done(vfe);
		return;
	}

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (vfe->wm_output_map[wm] == VFE_LINE_NONE) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Received wm done for unmapped index\n");
		goto out_unlock;
	}
	output = &vfe->line[vfe->wm_output_map[wm]].output;

	dual = line->id == VFE_LINE_PIX && vfe->dual.master && vfe->dual.peer;

	ready_buf = output->buf[0];
	if (!ready_buf) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Missing ready buf %d!\n", output->state);
		goto out_unlock;
	}

	/*
	 * Before the first epoch: a frame that began before the stream, not
	 * one of its frames. Queue the buffer's address again, after the one
	 * already queued, instead of numbering it as frame 0. The RDI lines
	 * of the VFE too, while its pixel path runs: they are numbered by its
	 * epochs.
	 */
	if (line->id == VFE_LINE_PIX ? !vfe17x_meta_frame_started(vfe) :
					vfe17x_meta_before_first_frame(vfe)) {
		output->buf[0] = output->buf[1];
		if (output->buf[0])
			output->buf[1] = ready_buf;
		else
			output->buf[0] = ready_buf;
		vfe_wm_update(vfe, output->wm_idx[0], ready_buf->addr[0], line);
		vfe_reg_update(vfe, line->id);
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		dev_dbg(vfe->camss->dev,
			"VFE%d: image before the first frame, buffer reused\n", vfe->id);
		return;
	}

	ready_buf->vb.vb2_buf.timestamp = ts;
	ready_buf->vb.sequence = output->sequence++;
	/*
	 * While the pixel path runs, the RDI lines of the same VFE number
	 * their frames by its epochs too, so that RAW and processed buffers of
	 * one sensor frame carry the same sequence.
	 */
	vfe17x_meta_sequence(vfe, &ready_buf->vb.sequence);
	bad = output->error;
	output->error = false;

	index = 0;
	output->buf[0] = output->buf[1];
	if (output->buf[0])
		index = 1;

	/* While the pixel path runs, the next buffers go in at its epochs */
	output->buf[index] = vfe17x_meta_running(vfe) ? NULL : vfe_buf_get_pending(output);

	if (output->buf[index]) {
		vfe_wm_update(vfe, output->wm_idx[0], output->buf[index]->addr[0], line);
		/*
		 * The pixel path latches its double buffered registers on a
		 * CAMIF reg update, issued for every frame like the vendor
		 * driver does.
		 */
		if (vfe_line_has_pix_pipeline(vfe, line->id))
			vfe_reg_update(vfe, line->id);
	} else {
		/*
		 * The write masters take one queued address per frame and skip
		 * the frames they have none for, without a done interrupt.
		 */
		output->gen2.active_num--;
	}

	/*
	 * Dual IFE master: the buffer is done once the slave wrote its stripe
	 * of the same frame, which completes within VFE_DUAL_MATCH_NS.
	 */
	if (dual) {
		struct vfe_pix_dual *d = &vfe->dual;

		if (d->slave_ts + VFE_DUAL_MATCH_NS > ts) {
			error[ndone] = bad || d->slave_error;
			done[ndone++] = ready_buf;
		} else {
			if (d->nheld == VFE_DUAL_HELD) {
				error[ndone] = true;
				done[ndone++] = d->held[0].buf;
				memmove(&d->held[0], &d->held[1],
					--d->nheld * sizeof(d->held[0]));
			}
			d->held[d->nheld].buf = ready_buf;
			d->held[d->nheld].ts = ts;
			d->held[d->nheld].error = bad;
			d->nheld++;
		}
	} else {
		error[ndone] = bad;
		done[ndone++] = ready_buf;
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	for (i = 0; i < ndone; i++)
		vb2_buffer_done(&done[i]->vb.vb2_buf,
				error[i] ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static const struct vfe_isr_ops vfe_isr_ops_170 = {
	.reset_ack = vfe_isr_reset_ack,
	.halt_ack = vfe_isr_halt_ack,
	.reg_update = vfe_isr_reg_update,
	.sof = vfe_isr_sof,
	.comp_done = vfe_isr_comp_done,
	.wm_done = vfe_isr_wm_done,
};

static const struct camss_video_ops vfe_video_ops_170 = {
	.queue_buffer = vfe17x_queue_buffer,
	.flush_buffers = vfe_flush_buffers,
};

static void vfe_subdev_init(struct device *dev, struct vfe_device *vfe)
{
	vfe->isr_ops = vfe_isr_ops_170;
	vfe->video_ops = vfe_video_ops_170;
}

/* Stop the pixel path statistics before the generic disable resets the VFE */
static int vfe17x_disable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct camss_buffer *done[VFE_DUAL_HELD];
	unsigned int i, n = 0;
	unsigned long flags;

	if (vfe_line_has_pix_pipeline(vfe, line->id)) {
		vfe17x_meta_stop(vfe);

		/*
		 * Dual IFE: end the pairing before this VFE stops or powers
		 * off, so the master no longer writes the slave's registers.
		 * Lock order slave, then master.
		 */
		spin_lock_irqsave(&vfe->output_lock, flags);
		if (vfe->dual.master) {
			n = vfe_pix_dual_end(vfe, done);
		} else if (vfe->dual.peer) {
			struct vfe_device *m = vfe->dual.peer;

			spin_lock_nested(&m->output_lock, SINGLE_DEPTH_NESTING);
			if (m->dual.peer == vfe)
				n = vfe_pix_dual_end(m, done);
			spin_unlock(&m->output_lock);
			memset(&vfe->dual, 0, sizeof(vfe->dual));
		}
		spin_unlock_irqrestore(&vfe->output_lock, flags);

		for (i = 0; i < n; i++)
			vb2_buffer_done(&done[i]->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	return vfe_disable(line);
}

const struct vfe_hw_ops vfe_ops_170 = {
	.global_reset = vfe_global_reset,
	.hw_version = vfe_hw_version,
	.isr_read = vfe_isr_read,
	.isr = vfe_isr,
	.pm_domain_off = vfe_pm_domain_off,
	.pm_domain_on = vfe_pm_domain_on,
	.reg_update_clear = vfe_reg_update_clear,
	.reg_update = vfe_reg_update,
	.subdev_init = vfe_subdev_init,
	.vfe_disable = vfe17x_disable,
	.vfe_enable = vfe_enable,
	.vfe_halt = vfe_halt,
	.violation_read = vfe_violation_read,
	.vfe_wm_start = vfe_wm_start,
	.vfe_wm_stop = vfe_wm_stop,
	.vfe_wm_update = vfe_wm_update,
	.meta_register = vfe17x_meta_register,
	.meta_unregister = vfe17x_meta_unregister,
	.fatal_error = vfe17x_fatal_error,
};
