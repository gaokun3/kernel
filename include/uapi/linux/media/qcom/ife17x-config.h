/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Qualcomm CAMSS Titan 17x IFE pixel pipeline - parameters and statistics
 *
 * The parameters buffer uses the V4L2 extensible ISP parameters format
 * (struct v4l2_isp_params_buffer): a sequence of the blocks below, each
 * starting with a struct v4l2_isp_params_block_header. Blocks not present in
 * a buffer keep their previous configuration while the stream runs. The IFE
 * is reset when streaming starts: the blocks are then back to the driver's
 * defaults until a buffer sets them, so the first buffer of a stream should
 * carry the full configuration. Out of range values and non-zero reserved
 * fields make the buffer invalid.
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
 * @IFE17X_PARAMS_BLOCK_PDPC: phase detection pixel correction (PDPC11)
 * @IFE17X_PARAMS_BLOCK_BF: focus statistics (BFStats23)
 */
enum ife17x_params_block_type {
	IFE17X_PARAMS_BLOCK_BLS,
	IFE17X_PARAMS_BLOCK_WB,
	IFE17X_PARAMS_BLOCK_CCM,
	IFE17X_PARAMS_BLOCK_GAMMA,
	IFE17X_PARAMS_BLOCK_AWB_BG,
	IFE17X_PARAMS_BLOCK_PDPC,
	IFE17X_PARAMS_BLOCK_BF,
};

/**
 * struct ife17x_params_bls - black level subtraction
 * @header: block header, type IFE17X_PARAMS_BLOCK_BLS
 * @offset: black level in the 14 bit pipeline units (sensor data MSB
 *	    aligned: a 10 bit black level of 64 is 1024), at most 8191. The
 *	    driver scales the remaining range back to full scale.
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
 * The gains apply after the AWB statistics tap. V4L2_ISP_PARAMS_FL_BLOCK_DISABLE
 * sets them to 1.0.
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
 *	   green and blue inputs; signed 12 bit (-2048..2047), 1.0 = 128 << @shift
 * @offset: red, green and blue output offsets, signed 11 bit (-1024..1023)
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
	__u8 reserved[7];
} __attribute__((aligned(8)));

#define IFE17X_GAMMA_NODES		65

/**
 * struct ife17x_params_gamma - gamma curves
 * @header: block header, type IFE17X_PARAMS_BLOCK_GAMMA
 * @curve: red, green and blue curves, 65 nodes evenly spaced over the input
 *	   range, output 0..1023; the last node may be 1024; a node differs
 *	   from the next one by -512..511
 * @reserved: must be zero
 *
 * The driver loads the curves into the inactive LUT banks and switches banks
 * with the next register update. V4L2_ISP_PARAMS_FL_BLOCK_DISABLE bypasses the
 * block.
 */
struct ife17x_params_gamma {
	struct v4l2_isp_params_block_header header;
	__u16 curve[3][IFE17X_GAMMA_NODES];
	__u16 reserved[5];
} __attribute__((aligned(8)));

#define IFE17X_AWB_BG_MAX_H		160
#define IFE17X_AWB_BG_MAX_V		90

/**
 * struct ife17x_params_awb_bg - AWB Bayer grid statistics configuration
 * @header: block header, type IFE17X_PARAMS_BLOCK_AWB_BG
 * @h_num: regions per row, 1..160
 * @v_num: region rows, 1..90
 * @threshold: per channel high thresholds in 14 bit units (at most 16383), in
 *	       the order red, blue, green on red rows, green on blue rows;
 *	       pixels above are not counted
 * @reserved: must be zero
 *
 * The grid covers the frame from its top left corner; regions are
 * (width / h_num) & ~1 by (height / v_num) & ~1 pixels, clamped to 6..390 by
 * 2..512. The statistics are taken after the black level subtraction and
 * before the white balance gains. V4L2_ISP_PARAMS_FL_BLOCK_DISABLE stops them.
 *
 * With two IFEs (frames wider than 3840 pixels, processed in a left and a
 * right stripe) each IFE covers its stripe: the left one with h_num / 2
 * regions per row, the right one with the rest, h_num 1 counting as 2. The
 * records of the left stripe come first, in row order, then those of the
 * right stripe.
 *
 * A grid queued while the stream is stopped is also applied at every later
 * stream start, until the parameters node stops streaming.
 */
struct ife17x_params_awb_bg {
	struct v4l2_isp_params_block_header header;
	__u16 h_num;
	__u16 v_num;
	__u16 threshold[4];
	__u32 reserved;
} __attribute__((aligned(8)));

#define IFE17X_PDPC_MASK_ROWS		64

/**
 * struct ife17x_params_pdpc - phase detection pixel correction
 * @header: block header, type IFE17X_PARAMS_BLOCK_PDPC
 * @black_level: black level of the input in the 14 bit pipeline units (a 10
 *		 bit level of 64 is 1024), at most 4095
 * @fmax: upper limit of a phase detection pixel against its neighbours,
 *	  unsigned Q6
 * @fmin: lower limit, unsigned Q6
 * @bp_offset: offsets for the green [0] and the red and blue [1] pixels, at
 *	       most 32767
 * @t2_bp_offset: second offsets, same order, at most 32767
 * @wb_ratio: white balance gain ratios R/G, B/G, G/R and G/B, unsigned Q12,
 *	      at most 131071
 * @mask: phase detection pixels of a 64x64 pixel tile repeated over the
 *	  frame from its top left corner: bit (31 - x / 2) of row y marks the
 *	  pixel pair at columns x & ~1 and x | 1 that holds one
 * @reserved: must be zero
 *
 * Corrects the pixels the mask marks over the whole frame. The field meanings
 * follow the calculation of the Windows driver; the tested values are those
 * of its ov13b10 tuning: fmax 80, fmin 32, offsets 128.
 * V4L2_ISP_PARAMS_FL_BLOCK_DISABLE bypasses the block.
 */
struct ife17x_params_pdpc {
	struct v4l2_isp_params_block_header header;
	__u16 black_level;
	__u8 fmax;
	__u8 fmin;
	__u16 bp_offset[2];
	__u16 t2_bp_offset[2];
	__u32 wb_ratio[4];
	__u32 mask[IFE17X_PDPC_MASK_ROWS];
	__u32 reserved;
} __attribute__((aligned(8)));

/* The ROI table has 180 entries, with an empty one before and after the regions */
#define IFE17X_BF_MAX_ROIS		178
/*
 * With two IFEs the right stripe's records start at a fixed slot, after the
 * left stripe's and the header the right stripe writes before its own.
 */
#define IFE17X_BF_DUAL_RIGHT		89
#define IFE17X_BF_DUAL_LEFT_MAX		(IFE17X_BF_DUAL_RIGHT - 1)
#define IFE17X_BF_DUAL_RIGHT_MAX	(IFE17X_BF_MAX_ROIS - IFE17X_BF_DUAL_RIGHT)
#define IFE17X_BF_IIR_COEFFS		10
#define IFE17X_BF_CORING_POINTS		17
#define IFE17X_BF_GAMMA_POINTS		32

/**
 * struct ife17x_bf_roi - focus statistics region
 * @left: first column, even, at least 76
 * @top: first row, even, at least 36
 * @width: columns, even, 6 to half the frame width
 * @height: rows, even, 8 to half the frame height
 *
 * left + width is at most the frame width - 12, top + height the frame height
 * - 4. Regions do not overlap and are listed in raster order of their bottom
 * right corners, which are at least 6 columns apart from one region to the
 * next; at most 21 start on one row. A grid of equal regions listed row by
 * row fits.
 */
struct ife17x_bf_roi {
	__u16 left;
	__u16 top;
	__u16 width;
	__u16 height;
};

/**
 * struct ife17x_bf_filter - one of the two focus filters
 * @iir: two cascaded biquads, b0 b1 b2 a1 a2 for the first then the second,
 *	 signed Q14, each H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 - a1 z^-1 - a2 z^-2)
 * @shift: right shift of the filter output, 0 to 15
 * @coring_gain: gain of the cored output, 0 to 1023 (16 in the tuning)
 * @coring_threshold: coring threshold, 0 to 131071
 * @coring: coring curve, 0 to 31
 * @reserved: must be zero
 */
struct ife17x_bf_filter {
	__s16 iir[IFE17X_BF_IIR_COEFFS];
	__u8 shift;
	__u8 reserved0;
	__u16 coring_gain;
	__u32 coring_threshold;
	__u8 coring[IFE17X_BF_CORING_POINTS];
	__u8 reserved[3];
};

/**
 * struct ife17x_params_bf - focus statistics configuration
 * @header: block header, type IFE17X_PARAMS_BLOCK_BF
 * @num_rois: regions in @rois, 1 to IFE17X_BF_MAX_ROIS
 * @green: green pixels the filters take, 0 or 1 (the tuning uses 0)
 * @gamma_enable: map the input through @gamma first
 * @num_left: with two IFEs, the regions the left stripe measures, from the
 *	      start of @rois; the right stripe measures the others. Zero with
 *	      one IFE
 * @reserved: must be zero
 * @gamma: gamma curve, 32 points of 14 bits; the slope after the last one
 *	   reaches 0x8000, as the Windows driver programs it
 * @h1: horizontal filter
 * @v: vertical filter
 * @rois: regions, in input pixels of the pixel path (the frame with two
 *	  IFEs); the record of each one carries its index in this array
 *
 * With two IFEs each stripe measures its own regions, which must lie inside
 * its input as the regions of one IFE lie inside the frame: the left stripe
 * runs from column 0, the right stripe from the split minus the padding of
 * the stripes (see the pipeline). The left stripe takes at most
 * IFE17X_BF_DUAL_LEFT_MAX regions and the right one at most
 * IFE17X_BF_DUAL_RIGHT_MAX; the regions of each part follow the order rules
 * on their own. V4L2_ISP_PARAMS_FL_BLOCK_DISABLE stops the statistics.
 *
 * The defaults of the Windows ov13b10 tuning: green 0, gamma on and linear,
 * both filters the IIR b {2926, 0, -2926}, a {28241, -13251}, b {5120, 0,
 * -5120}, a {18960, -9991}, shift 3, coring gain 16, threshold 400, curve
 * 0 0 0 0 0 0 1 1 2 3 4 5 7 9 11 13 16.
 */
struct ife17x_params_bf {
	struct v4l2_isp_params_block_header header;
	__u16 num_rois;
	__u8 green;
	__u8 gamma_enable;
	__u16 num_left;
	__u16 reserved;
	__u16 gamma[IFE17X_BF_GAMMA_POINTS];
	struct ife17x_bf_filter h1;
	struct ife17x_bf_filter v;
	struct ife17x_bf_roi rois[IFE17X_BF_MAX_ROIS];
} __attribute__((aligned(8)));

#define IFE17X_PARAMS_MAX_SIZE					\
	(sizeof(struct ife17x_params_bls) +			\
	 sizeof(struct ife17x_params_wb) +			\
	 sizeof(struct ife17x_params_ccm) +			\
	 sizeof(struct ife17x_params_gamma) +			\
	 sizeof(struct ife17x_params_awb_bg) +			\
	 sizeof(struct ife17x_params_pdpc) +			\
	 sizeof(struct ife17x_params_bf))

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
 * struct ife17x_bf_record - focus statistics of a region, as the hardware
 * writes it
 * @w: w[0] [36:0] H1 sum, [61:39] H1 count, [63] region type;
 *     w[1] [39:0] H1 sharpness, [61:54] region index (of the configured list);
 *     w[2] [36:0] V sum, [61:39] V count; w[3] [39:0] V sharpness
 */
struct ife17x_bf_record {
	__u64 w[4];
};

/**
 * struct ife17x_stats_buffer - statistics of one frame
 * @awb_bg: AWB grid, h_num * v_num records in row order (with two IFEs, the
 *	    left stripe's grid and then the right stripe's, see struct
 *	    ife17x_params_awb_bg); the rest of the array is undefined
 * @bf_header: written by the hardware before the focus records
 * @bf: focus records of the configured regions, in an order of their own:
 *	the region index tells which; undefined without focus statistics. With
 *	two IFEs the left stripe's records start at bf[0] and the right
 *	stripe's at bf[IFE17X_BF_DUAL_RIGHT]: the header the right stripe
 *	writes before its records ends bf[IFE17X_BF_DUAL_RIGHT - 1]
 *
 * The buffer sequence number is the frame sequence number of the image the
 * statistics describe.
 */
struct ife17x_stats_buffer {
	struct ife17x_awb_bg_region awb_bg[IFE17X_AWB_BG_MAX_H * IFE17X_AWB_BG_MAX_V];
	__u64 bf_header;
	struct ife17x_bf_record bf[IFE17X_BF_MAX_ROIS];
};

#endif /* __UAPI_QCOM_IFE17X_CONFIG_H */
