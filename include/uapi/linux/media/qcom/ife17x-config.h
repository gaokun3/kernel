/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Qualcomm CAMSS Titan 17x IFE pixel pipeline - parameters and statistics
 *
 * The parameters buffer uses the V4L2 extensible ISP parameters format
 * (struct v4l2_isp_params_buffer): a sequence of the blocks below, each
 * starting with a struct v4l2_isp_params_block_header. Blocks not present in
 * a buffer keep their previous configuration.
 *
 * Register layouts were derived from the SC8280XP Windows camera stack and
 * measured on hardware; see the comments of each block.
 */

#ifndef __UAPI_QCOM_IFE17X_CONFIG_H
#define __UAPI_QCOM_IFE17X_CONFIG_H

#include <linux/types.h>
#include <linux/media/v4l2-isp.h>

/**
 * enum ife17x_params_block_type - IFE parameter block types
 *
 * @IFE17X_PARAMS_BLOCK_BLS: black level subtraction (BLS12)
 * @IFE17X_PARAMS_BLOCK_WB: white balance gains (WB12)
 * @IFE17X_PARAMS_BLOCK_CCM: colour correction matrix (CC12)
 * @IFE17X_PARAMS_BLOCK_GAMMA: gamma curves (Gamma16)
 * @IFE17X_PARAMS_BLOCK_AWB_BG: AWB Bayer grid statistics (AWBBGStats14)
 */
enum ife17x_params_block_type {
	IFE17X_PARAMS_BLOCK_BLS,
	IFE17X_PARAMS_BLOCK_WB,
	IFE17X_PARAMS_BLOCK_CCM,
	IFE17X_PARAMS_BLOCK_GAMMA,
	IFE17X_PARAMS_BLOCK_AWB_BG,
};

/**
 * struct ife17x_params_bls - black level subtraction
 * @header: block header, type IFE17X_PARAMS_BLOCK_BLS
 * @offset: black level in the 14 bit pipeline units (sensor data MSB
 *	    aligned: a 10 bit black level of 64 is 1024). The driver scales the
 *	    remaining range back to full scale.
 * @reserved: must be zero
 *
 * V4L2_ISP_PARAMS_FL_BLOCK_DISABLE bypasses the block.
 */
struct ife17x_params_bls {
	struct v4l2_isp_params_block_header header;
	__u16 offset;
	__u16 reserved[3];
} __attribute__((aligned(8)));

/**
 * struct ife17x_params_wb - white balance gains
 * @header: block header, type IFE17X_PARAMS_BLOCK_WB
 * @gain: red, green and blue gains, unsigned Q5.7 (1.0 = 128), at most 4095
 * @reserved: must be zero
 *
 * The gains apply after the AWB statistics tap.
 */
struct ife17x_params_wb {
	struct v4l2_isp_params_block_header header;
	__u16 gain[3];
	__u16 reserved;
} __attribute__((aligned(8)));

/**
 * struct ife17x_params_ccm - colour correction matrix
 * @header: block header, type IFE17X_PARAMS_BLOCK_CCM
 * @coeff: matrix, rows are the red, green and blue outputs, columns the red,
 *	   green and blue inputs; signed 12 bit, 1.0 = 128 << @shift
 * @offset: red, green and blue output offsets, signed 11 bit
 * @shift: fractional bits beyond 7, 0..3
 * @reserved: must be zero
 *
 * V4L2_ISP_PARAMS_FL_BLOCK_DISABLE bypasses the block.
 */
struct ife17x_params_ccm {
	struct v4l2_isp_params_block_header header;
	__s16 coeff[3][3];
	__s16 offset[3];
	__u8 shift;
	__u8 reserved[3];
} __attribute__((aligned(8)));

#define IFE17X_GAMMA_NODES		65

/**
 * struct ife17x_params_gamma - gamma curves
 * @header: block header, type IFE17X_PARAMS_BLOCK_GAMMA
 * @curve: red, green and blue curves, 65 nodes evenly spaced over the input
 *	   range, output 0..1023; the last node may be 1024
 * @reserved: must be zero
 *
 * The driver loads the curves into the inactive LUT banks and switches banks
 * with the next register update. V4L2_ISP_PARAMS_FL_BLOCK_DISABLE bypasses the
 * block.
 */
struct ife17x_params_gamma {
	struct v4l2_isp_params_block_header header;
	__u16 curve[3][IFE17X_GAMMA_NODES];
	__u16 reserved[3];
} __attribute__((aligned(8)));

#define IFE17X_AWB_BG_MAX_H		160
#define IFE17X_AWB_BG_MAX_V		90

/**
 * struct ife17x_params_awb_bg - AWB Bayer grid statistics configuration
 * @header: block header, type IFE17X_PARAMS_BLOCK_AWB_BG
 * @h_num: regions per row, 1..160
 * @v_num: region rows, 1..90
 * @threshold: per channel high thresholds in 14 bit units, in the order red,
 *	       blue, green on red rows, green on blue rows; pixels above are
 *	       not counted
 * @reserved: must be zero
 *
 * The grid covers the frame from its top left corner; regions are
 * (width / h_num) & ~1 by (height / v_num) & ~1 pixels, clamped to 6..390 by
 * 2..512. The statistics are taken after the black level subtraction and
 * before the white balance gains. V4L2_ISP_PARAMS_FL_BLOCK_DISABLE stops them.
 */
struct ife17x_params_awb_bg {
	struct v4l2_isp_params_block_header header;
	__u16 h_num;
	__u16 v_num;
	__u16 threshold[4];
	__u32 reserved;
} __attribute__((aligned(8)));

#define IFE17X_PARAMS_MAX_SIZE					\
	(sizeof(struct ife17x_params_bls) +			\
	 sizeof(struct ife17x_params_wb) +			\
	 sizeof(struct ife17x_params_ccm) +			\
	 sizeof(struct ife17x_params_gamma) +			\
	 sizeof(struct ife17x_params_awb_bg))

/**
 * struct ife17x_awb_bg_region - AWB Bayer grid record, as the hardware
 * writes it
 * @sum: sums of the counted pixels: red, blue, green on red rows, green on
 *	 blue rows
 * @count: counted pixels, same order
 */
struct ife17x_awb_bg_region {
	__u32 sum[4];
	__u16 count[4];
};

/**
 * struct ife17x_stats_buffer - statistics of one frame
 * @awb_bg: AWB grid, h_num * v_num records in row order; the rest of the
 *	    array is undefined
 *
 * The buffer sequence number is the frame sequence number of the image the
 * statistics describe.
 */
struct ife17x_stats_buffer {
	struct ife17x_awb_bg_region awb_bg[IFE17X_AWB_BG_MAX_H * IFE17X_AWB_BG_MAX_V];
};

#endif /* __UAPI_QCOM_IFE17X_CONFIG_H */
