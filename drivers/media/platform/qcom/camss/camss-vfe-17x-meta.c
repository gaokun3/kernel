// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-17x-meta.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Titan 17x) pixel pipeline parameters
 * and statistics video nodes.
 *
 * The parameters node takes V4L2 extensible ISP parameters buffers (see
 * uapi/linux/media/qcom/ife17x-config.h). One buffer is applied at stream
 * start and then one per frame from the CAMIF epoch interrupt, early in the
 * frame; the registers are double buffered and latch with the next register
 * update. Gamma curves go to the inactive DMI banks first.
 *
 * The statistics node takes buffers the AWB Bayer grid write master writes
 * into directly. Without a queued buffer the master writes a driver scratch
 * buffer instead, so it never writes memory userspace got back.
 */

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp.h>
#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-vmalloc.h>

#include <uapi/linux/media/qcom/ife17x-config.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-17x-disp.h"
#include "camss-vfe-17x-meta.h"

/* Module enables, double buffered */
#define VFE_MODULE_LENS_EN		0x040
#define		LENS_EN_BPC		BIT(6)
#define		LENS_EN_ABF		BIT(7)
#define		LENS_EN_LSC		BIT(8)
#define		LENS_EN_BLS		BIT(11)
#define		LENS_EN_PDPC		BIT(13)
#define VFE_MODULE_STATS_EN		0x044
#define		STATS_EN_BF		BIT(2)
#define		STATS_EN_AWB_BG		BIT(3)
#define		STATS_EN_BHIST		BIT(4)
#define VFE_MODULE_COLOR_EN		0x048
#define		COLOR_EN_CC		BIT(1)
#define		COLOR_EN_GAMMA		BIT(3)
#define VFE_REG_UPDATE_CMD		0x4ac
#define		REG_UPDATE_PIX		BIT(0)
#define VFE_CAMIF_EPOCH_IRQ		0x4a0
/*
 * [31:16] epoch 0, [15:0] epoch 1. Epoch 0 at line 20 like the Windows driver
 * (19): frame start for the sensor controls, and where the next frame is
 * settled. Epoch 1 at three quarters of the frame closes that: the addresses
 * of the next frame are queued a quarter of a frame before it starts, ahead
 * of the frame end, which can come within half a millisecond of it.
 */
#define		CAMIF_EPOCH0_LINE	20
#define		CAMIF_EPOCH_CFG(height)	(CAMIF_EPOCH0_LINE << 16 | ((height) * 3 / 4))

/* BPCBCC50, from the Windows driver: 0x5c4..0x5e0 */
#define VFE_BPC_CFG			0x5c4	/* flags [2:0], black level [14:3] */
#define VFE_BPC_WB_RATIO(i)		(0x5c8 + (i) * 4)	/* R/G, B/G, G/R, G/B */
#define VFE_BPC_OFFSET			0x5d8	/* single [14:0], cluster [30:16] */
#define VFE_BPC_LIMITS			0x5dc	/* fmax [6:0], fmin [14:8], threshold [28:16] */
#define VFE_BPC_STATS			0x5e0

/*
 * ABF34, from the Windows driver: 0x5e8 configuration, 0x5f4..0x648 the rest,
 * the noise LUT in DMI bank 12 or 13 (0x5e8 bit 1 selects 13)
 */
#define VFE_ABF_CFG			0x5e8
#define		ABF_CFG_FILTER		BIT(0)
#define		ABF_CFG_LUT_BANK	BIT(1)
#define		ABF_CFG_RADIAL		BIT(2)
#define		ABF_CFG_BPC		BIT(3)
#define VFE_ABF_CURVE_OFFSET(c)		(0x5f4 + (c) * 4)
#define VFE_ABF_RADIAL_POS		0x604	/* x [13:0], y [29:16] */
#define VFE_ABF_RADIAL_R2_INIT		0x608
#define VFE_ABF_RADIAL_ANCHOR(i)	(0x60c + (i) * 4)	/* two per register */
#define VFE_ABF_RADIAL_SEG(c, s)	(0x614 + (c) * 0x10 + (s) * 4)
#define VFE_ABF_RADIAL_R2_SHIFT		0x634
#define VFE_ABF_BPC_0			0x638
#define VFE_ABF_BPC_1			0x63c
#define VFE_ABF_NOISE_PRSV_ANCHOR	0x640
#define VFE_ABF_NOISE_PRSV(c)		(0x644 + (c) * 4)
#define VFE_ABF_DMI_BANK		12
#define ABF_LUT_ENTRIES			(IFE17X_ABF_NOISE_NODES - 1)
/* The squared distance register is 28 bits wide */
#define ABF_R2_MAX			GENMASK(27, 0)

/*
 * LSC34, from the Windows driver: the grid of the left (_l) and the right
 * (_r) group, the gains in DMI banks 14 (R, Gr) and 15 (B, Gb), or 18 and 19
 * with 0x6bc bit 16. The Windows driver writes both groups alike.
 */
#define VFE_LSC_CFG			0x6bc	/* blocks across [23:18], down [29:24] */
#define		LSC_CFG_BANK		BIT(16)
#define VFE_LSC_GRID(g, i)		(0x6c0 + (g) * 0xc + (i) * 4)
#define VFE_LSC_START(g, i)		(0x6d8 + (g) * 8 + (i) * 4)
#define VFE_LSC_DMI_BANK		14

#define VFE_BLS_CFG			0x6b0
#define		BLS_CFG_SCALE		15
#define VFE_BLS_THRESHOLD_0		0x6b4
#define VFE_BLS_THRESHOLD_1		0x6b8

#define VFE_WB_CFG_0			0x6fc	/* G [11:0], B [27:16] */
#define VFE_WB_CFG_1			0x700	/* R [11:0] */

/* CC12: output rows G, B, R; inputs G, B, R inside a row (measured) */
#define VFE_CC_COEFF(row, col)		(0x760 + (row) * 0xc + (col) * 4)
#define VFE_CC_OFFSET(row)		(0x784 + (row) * 4)
#define VFE_CC_CFG			0x790

/* Gamma16: 0x798 bit0 G (banks 26/27), bit1 B (28/29), bit2 R (30/31) */
#define VFE_GAMMA_CFG			0x798
#define VFE_DMI_CFG			0xc24
#define		DMI_CFG_AUTO_INC	BIT(8)
#define VFE_DMI_ADDR			0xc28
#define VFE_DMI_DATA_HI			0xc2c
#define VFE_DMI_DATA_LO			0xc30
#define GAMMA_ENTRIES			64

/* PDPC11, from the Windows driver (Titan17x CreateCmdList and PackIQ) */
#define VFE_PDPC_CFG			0xcb4	/* [0] on, [1] HDR, [19:8] black */
#define VFE_PDPC_HDR_RATIO		0xcb8	/* [8:0] 1/ratio Q8, [30:16] ratio Q10 */
#define VFE_PDPC_T2_OFFSET		0xcbc	/* [14:0] red/blue, [30:16] green */
#define VFE_PDPC_LIMITS			0xcc0	/* [7:0] fmax, [15:8] fmin */
#define VFE_PDPC_OFFSET			0xcc4	/* [14:0] red/blue, [30:16] green */
#define VFE_PDPC_WB_RATIO(n)		(0xcc8 + 4 * (n))
#define VFE_PDPC_TILE_OFFSET		0xcd8	/* [13:0] x, [29:16] y */
#define VFE_PDPC_END			0xcdc	/* [13:0] x, [29:16] y */
#define VFE_PDPC_DMI_BANK		0x0b

#define VFE_STATS_CFG			0xaac
#define		STATS_CFG_BHIST_UNIFORM	BIT(4)
#define		STATS_CFG_FIXED		BIT(6)
#define		STATS_CFG_BHIST_CHANNEL	GENMASK(23, 21)
/* BHistStats14: [12:0] left, [29:16] top; [11:0] width / 2 - 1, [28:16] height / 2 - 1 */
#define VFE_BHIST_OFFSET		0xbe4
#define VFE_BHIST_SIZE			0xbe8
/* The histogram RAM, one 32-bit count per bin */
#define VFE_BHIST_DMI_BANK		0x3a
#define VFE_AWB_BG_OFFSET		0xbc8
#define VFE_AWB_BG_NUM			0xbcc
#define VFE_AWB_BG_SIZE			0xbd0
#define VFE_AWB_BG_HI_THR_0		0xbd4	/* R [13:0], Gr [29:16] */
#define VFE_AWB_BG_HI_THR_1		0xbd8	/* B [13:0], Gb [29:16] */
#define VFE_AWB_BG_LO_THR_0		0xbdc
#define VFE_AWB_BG_LO_THR_1		0xbe0
#define VFE_AWB_BG_LUMA_CFG		0xc14
#define VFE_AWB_BG_CFG			0xcb0

/* BFStats23, as the Windows driver programs it */
#define VFE_BF_CFG			0xae4
#define		BF_CFG_GREEN		BIT(0)
#define		BF_CFG_GAMMA		BIT(1)
#define		BF_CFG_GAMMA_BANK	BIT(2)
#define		BF_CFG_H1_IIR		BIT(9)
#define		BF_CFG_V_IIR		BIT(13)
#define		BF_CFG_ROI_BANK		BIT(16)
#define VFE_BF_LUMA(n)			(0xae8 + 4 * (n))	/* 2 */
#define VFE_BF_FIR(n)			(0xaf0 + 4 * (n))	/* 4 */
#define VFE_BF_H1_IIR(n)		(0xb00 + 4 * (n))	/* 5 */
#define VFE_BF_SHIFT			0xb4c	/* [3:0] H1, [19:16] V */
#define VFE_BF_H1_CORING_THR		0xb50
#define VFE_BF_H1_CORING(n)		(0xb54 + 4 * (n))	/* 4 */
#define VFE_BF_V_CORING_THR		0xb78
#define VFE_BF_V_CORING(n)		(0xb7c + 4 * (n))	/* 4 */
#define VFE_BF_H1_CORING_GAIN		0xb8c
#define VFE_BF_V_CORING_GAIN		0xb90
#define VFE_BF_SCALE(n)			(0xb94 + 4 * (n))	/* 6 */
#define VFE_BF_ACTIVE_START		0xca4
#define VFE_BF_ACTIVE_END		0xca8
#define VFE_BF_ROI_DMI_BANK		0x40
#define VFE_BF_GAMMA_DMI_BANK		0x42
/* The V filter's IIR is split in two register ranges */
static const u16 vfe_bf_v_iir[5] = { 0xb40, 0xb44, 0xb48, 0xc9c, 0xca0 };

/* Regions starting on one row, and the columns between the ends of two */
#define IFE_BF_ROIS_PER_ROW		21
#define IFE_BF_END_SPACING		6

/* Bus write masters of the focus statistics and the AWB grid, frame based, PLAIN64 */
#define VFE_WM_BF			14
#define VFE_WM_AWB_BG			15
#define VFE_WM_BHIST			16
#define IFE_STATS_WMS			(BIT(VFE_WM_BF) | BIT(VFE_WM_AWB_BG) | BIT(VFE_WM_BHIST))
#define VFE_BUS_WM_CFG(n)		(0x2208 + (n) * 0x100)
#define VFE_BUS_WM_IMAGE_ADDR(n)	(0x2214 + (n) * 0x100)
#define VFE_BUS_WM_BUFFER_WIDTH_CFG(n)	(0x221c + (n) * 0x100)
#define VFE_BUS_WM_BUFFER_HEIGHT_CFG(n)	(0x2220 + (n) * 0x100)
#define VFE_BUS_WM_PACKER_CFG(n)	(0x2224 + (n) * 0x100)
#define VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(n)	(0x2248 + (n) * 0x100)
#define VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(n)	(0x224c + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PERIOD(n)	(0x2250 + (n) * 0x100)
#define VFE_BUS_WM_FRAMEDROP_PATTERN(n)	(0x2254 + (n) * 0x100)
#define VFE_BUS_WM_FRAME_INC(n)		(0x2258 + (n) * 0x100)
#define VFE_BUS_WM_BURST_LIMIT(n)	(0x225c + (n) * 0x100)
#define		PACKER_FMT_PLAIN_64	0xa
#define		WM_CFG_FRAME_BASED	0x3

#define IFE_PARAMS_SIZE		v4l2_isp_params_buffer_size(IFE17X_PARAMS_MAX_SIZE)
#define IFE_STATS_SIZE		sizeof(struct ife17x_stats_buffer)
/*
 * Wait for the frames the statistics write masters have addresses of: three
 * frame periods and a margin, at least IFE_STATS_DRAIN_MIN_MS.
 */
#define IFE_STATS_DRAIN_MIN_MS	500
#define IFE_STATS_DRAIN_MAX_MS	5000

struct ife_params_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head queue;
	struct v4l2_isp_params_buffer *config;
	/* The outputs that write its frame (IFE17X_OUTPUT_*), 0 without the block */
	u32 outputs;
	/* The display output's window from its frame on, if it sets one */
	bool window_set;
	struct ife17x_params_display_window window;
};

struct ife_stats_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head queue;
	dma_addr_t addr;
};

struct vfe_pix_meta {
	struct vfe_device *vfe;

	struct video_device params_vdev;
	struct vb2_queue params_q;
	struct mutex params_lock;	/* params video node */
	struct media_pad params_pad;

	struct video_device stats_vdev;
	struct vb2_queue stats_q;
	struct mutex stats_lock;	/* stats video node */
	struct media_pad stats_pad;

	void *scratch;
	dma_addr_t scratch_addr;
	/* Statistics buffers completed or the pixel path stopped */
	wait_queue_head_t stats_wq;

	/* Everything below is protected by the lock */
	spinlock_t lock;
	struct list_head params_queue;
	struct list_head stats_queue;
	u32 stats_en;		/* STATS_EN bits of ours, as programmed */
	/*
	 * Statistics write masters done between epochs 0 of frame k and k + 1
	 * wrote frame k: the focus statistics when their last region ends, the
	 * AWB grid at the end of the frame. Frame k's buffer (stats_cur; none
	 * for the scratch buffer) completes once every write master the frame
	 * was set up with (cur_wms) is done (cur_got), and fails at the next
	 * epoch 0 otherwise. Frame k + 1 is settled (next_set) before epoch 1
	 * of frame k, its addresses queued then.
	 */
	struct ife_stats_buf *stats_cur;
	u32 cur_wms;
	u32 cur_got;
	struct ife_stats_buf *stats_next;
	u32 next_wms;
	u32 next_got;		/* done with frame k + 1 before its epoch 0 */
	bool next_set;
	bool running;
	/* The pixel path is wedged until the next start: no frame completes */
	bool wedged;
	/*
	 * The frame after the current one is written by the image path, so
	 * it belongs to a request: it takes the next parameters and
	 * statistics buffers. Decided at epoch 0.
	 */
	bool next_written;
	/*
	 * From epoch 0 to epoch 1, a buffer for the next frame may still join
	 * it, with its parameters (vfe17x_meta_join()).
	 */
	bool join_open;
	u32 width;
	u32 height;
	u32 frame;		/* epochs since the stream start */
	/*
	 * Frames are numbered from the first one the image path writes, as
	 * userspace expects of a stream: frame seq_base is frame 0. Frames
	 * before it (the display output not on yet) are not the stream's and
	 * have no frame start event. seq_based: that frame is known.
	 */
	u32 seq_base;
	bool seq_based;
	u64 epoch_ns;		/* time of the last epoch 0: the frame's start */
	u64 period_ns;		/* between the last two epochs 0 */
	u8 gamma_bank;		/* LUT banks in use: 0 even, 1 odd */
	u8 abf_bank;		/* ABF noise LUT bank in use */
	u8 lsc_bank;		/* LSC gain banks in use */
	bool awb_bg_pending;	/* AWB block seen before the stream start */
	struct ife17x_params_awb_bg awb_bg;
	bool pdpc_mask_valid;	/* the PDPC mask bank holds pdpc_mask */
	u32 pdpc_mask[IFE17X_PDPC_MASK_ROWS];
	u8 bf_bank;		/* BF ROI and gamma LUT banks in use */
	bool bf_valid;		/* the BF registers and banks hold bf */
	struct ife17x_params_bf bf;

	/*
	 * Dual IFE, under the master's lock: the master applies its parameters
	 * to the slave too, and the two grids share the statistics buffers.
	 */
	struct vfe_pix_meta *dual_slave;	/* master */
	struct vfe_pix_meta *dual_master;	/* slave */
	u32 stripe_x;		/* first input column the grid covers */
	u32 stripe_w;		/* input columns the grid covers */
	u32 frame_x;		/* frame column of input column 0 */
	u32 slave_stats_off;	/* master: byte offset of the slave's grid */
	u64 slave_stats_ts;	/* master: when the slave last wrote a grid */
	/* Slave, under the master's lock: write masters done, and when its grid was */
	u32 slave_got;
	u64 slave_grid_ts;
	unsigned int nheld;	/* master: buffers waiting for the slave's grid */
	struct {
		struct ife_stats_buf *buf;
		u64 ts;
	} held[4];
};

static inline struct ife_params_buf *to_params_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct ife_params_buf, vb);
}

static inline struct ife_stats_buf *to_stats_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct ife_stats_buf, vb);
}

static void vfe_rmw(struct vfe_device *vfe, u32 reg, u32 clear, u32 set)
{
	writel_relaxed((readl_relaxed(vfe->base + reg) & ~clear) | set,
		       vfe->base + reg);
}

/* Turn statistics on or off, from the next register update */
static void ife_stats_enable(struct vfe_pix_meta *meta, u32 bit, bool on)
{
	vfe_rmw(meta->vfe, VFE_MODULE_STATS_EN, bit, on ? bit : 0);
	if (on)
		meta->stats_en |= bit;
	else
		meta->stats_en &= ~bit;
}

/* The statistics write masters that write a frame with the programmed setup */
static u32 ife_stats_wms(struct vfe_pix_meta *meta)
{
	return (meta->stats_en & STATS_EN_AWB_BG ? BIT(VFE_WM_AWB_BG) : 0) |
	       (meta->stats_en & STATS_EN_BF ? BIT(VFE_WM_BF) : 0) |
	       (meta->stats_en & STATS_EN_BHIST ? BIT(VFE_WM_BHIST) : 0);
}

/* -----------------------------------------------------------------------------
 * Parameter blocks, called with the lock held and the VFE powered
 */

union ife_params_block {
	const struct v4l2_isp_params_block_header *header;
	const struct ife17x_params_bls *bls;
	const struct ife17x_params_wb *wb;
	const struct ife17x_params_ccm *ccm;
	const struct ife17x_params_gamma *gamma;
	const struct ife17x_params_awb_bg *awb_bg;
	const struct ife17x_params_pdpc *pdpc;
	const struct ife17x_params_bf *bf;
	const struct ife17x_params_abf *abf;
	const struct ife17x_params_bpc *bpc;
	const struct ife17x_params_lsc *lsc;
	const struct ife17x_params_outputs *outputs;
	const struct ife17x_params_display_window *window;
	const struct ife17x_params_bhist *bhist;
	const u8 *data;
};

typedef void (*ife_params_handler)(struct vfe_pix_meta *meta,
				   union ife_params_block block);

static bool ife_block_disabled(union ife_params_block block)
{
	return block.header->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE;
}

static void ife_params_bls(struct vfe_pix_meta *meta, union ife_params_block block)
{
	struct vfe_device *vfe = meta->vfe;
	u32 offset, scale;

	if (ife_block_disabled(block)) {
		vfe_rmw(vfe, VFE_MODULE_LENS_EN, LENS_EN_BLS, 0);
		return;
	}

	/* CamX: scale = 16384 / (16384 - offset) in Q11, at most 0xffff */
	offset = min_t(u32, block.bls->offset, 8191);
	scale = min_t(u32, (2048U * 16384U) / (16384U - offset), 0xffff);
	writel_relaxed(offset | scale << BLS_CFG_SCALE, vfe->base + VFE_BLS_CFG);
	writel_relaxed(0, vfe->base + VFE_BLS_THRESHOLD_0);
	writel_relaxed(0, vfe->base + VFE_BLS_THRESHOLD_1);
	vfe_rmw(vfe, VFE_MODULE_LENS_EN, 0, LENS_EN_BLS);
}

static void ife_params_wb(struct vfe_pix_meta *meta, union ife_params_block block)
{
	static const u16 unity[3] = { 128, 128, 128 };
	struct vfe_device *vfe = meta->vfe;
	/* The gains cannot be bypassed: disabled is 1.0 */
	const u16 *g = ife_block_disabled(block) ? unity : block.wb->gain;

	writel_relaxed((g[1] & 0xfff) | (g[2] & 0xfff) << 16,
		       vfe->base + VFE_WB_CFG_0);
	writel_relaxed(g[0] & 0xfff, vfe->base + VFE_WB_CFG_1);
}

static void ife_params_ccm(struct vfe_pix_meta *meta, union ife_params_block block)
{
	/* Hardware row/column index -> R(0), G(1), B(2) of the API */
	static const unsigned int hw_order[3] = { 1, 2, 0 };
	struct vfe_device *vfe = meta->vfe;
	const struct ife17x_params_ccm *ccm = block.ccm;
	unsigned int r, c;
	u32 v;

	if (ife_block_disabled(block)) {
		vfe_rmw(vfe, VFE_MODULE_COLOR_EN, COLOR_EN_CC, 0);
		return;
	}

	for (r = 0; r < 3; r++) {
		for (c = 0; c < 3; c++) {
			v = (u16)ccm->coeff[hw_order[r]][hw_order[c]] & 0xfff;
			writel_relaxed(v | v << 16, vfe->base + VFE_CC_COEFF(r, c));
		}
		v = (u16)ccm->offset[hw_order[r]] & 0x7ff;
		writel_relaxed(v | v << 16, vfe->base + VFE_CC_OFFSET(r));
	}
	v = min_t(u32, ccm->shift, 3);
	writel_relaxed(v | v << 16, vfe->base + VFE_CC_CFG);
	vfe_rmw(vfe, VFE_MODULE_COLOR_EN, 0, COLOR_EN_CC);
}

static void ife_params_gamma(struct vfe_pix_meta *meta, union ife_params_block block)
{
	/* 0x798 bit order G, B, R; banks 26/27, 28/29, 30/31; API order R, G, B */
	static const unsigned int curve_of_bit[3] = { 1, 2, 0 };
	struct vfe_device *vfe = meta->vfe;
	u8 bank = !meta->gamma_bank;
	unsigned int bit, i;

	if (ife_block_disabled(block)) {
		vfe_rmw(vfe, VFE_MODULE_COLOR_EN, COLOR_EN_GAMMA, 0);
		return;
	}

	for (bit = 0; bit < 3; bit++) {
		const u16 *t = block.gamma->curve[curve_of_bit[bit]];

		writel_relaxed((26 + 2 * bit + bank) | DMI_CFG_AUTO_INC,
			       vfe->base + VFE_DMI_CFG);
		writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
		for (i = 0; i < GAMMA_ENTRIES; i++) {
			int node = min_t(int, t[i], 1023);
			int next = min_t(int, t[i + 1], i < GAMMA_ENTRIES - 1 ? 1023 : 1024);
			int delta = clamp(next - node, -512, 511);

			writel_relaxed(node | ((u32)delta & 0x3ff) << 10,
				       vfe->base + VFE_DMI_DATA_LO);
		}
	}
	writel_relaxed(0, vfe->base + VFE_DMI_CFG);
	writel_relaxed(bank ? 0x7 : 0, vfe->base + VFE_GAMMA_CFG);
	meta->gamma_bank = bank;
	vfe_rmw(vfe, VFE_MODULE_COLOR_EN, 0, COLOR_EN_GAMMA);
}

static void ife_params_pdpc(struct vfe_pix_meta *meta, union ife_params_block block)
{
	const struct ife17x_params_pdpc *p = block.pdpc;
	struct vfe_device *vfe = meta->vfe;
	unsigned int i;

	if (ife_block_disabled(block)) {
		vfe_rmw(vfe, VFE_MODULE_LENS_EN, LENS_EN_PDPC, 0);
		return;
	}

	/* One mask bank: rewrite it only when it changes */
	if (!meta->pdpc_mask_valid || memcmp(meta->pdpc_mask, p->mask, sizeof(p->mask))) {
		writel_relaxed(VFE_PDPC_DMI_BANK | DMI_CFG_AUTO_INC, vfe->base + VFE_DMI_CFG);
		writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
		for (i = 0; i < IFE17X_PDPC_MASK_ROWS; i++)
			writel_relaxed(p->mask[i], vfe->base + VFE_DMI_DATA_LO);
		writel_relaxed(0, vfe->base + VFE_DMI_CFG);
		memcpy(meta->pdpc_mask, p->mask, sizeof(p->mask));
		meta->pdpc_mask_valid = true;
	}

	/* No HDR: exposure ratio 1.0 */
	writel_relaxed(BIT(0) | min_t(u32, p->black_level, 0xfff) << 8,
		       vfe->base + VFE_PDPC_CFG);
	writel_relaxed(0x400 << 16 | 0x100, vfe->base + VFE_PDPC_HDR_RATIO);
	writel_relaxed(min_t(u32, p->t2_bp_offset[1], 0x7fff) |
		       min_t(u32, p->t2_bp_offset[0], 0x7fff) << 16,
		       vfe->base + VFE_PDPC_T2_OFFSET);
	writel_relaxed(p->fmax | p->fmin << 8, vfe->base + VFE_PDPC_LIMITS);
	writel_relaxed(min_t(u32, p->bp_offset[1], 0x7fff) |
		       min_t(u32, p->bp_offset[0], 0x7fff) << 16,
		       vfe->base + VFE_PDPC_OFFSET);
	for (i = 0; i < 4; i++)
		writel_relaxed(min_t(u32, p->wb_ratio[i], 0x1ffff),
			       vfe->base + VFE_PDPC_WB_RATIO(i));
	/* The mask tile starts at the top left corner of the input */
	writel_relaxed(0, vfe->base + VFE_PDPC_TILE_OFFSET);
	writel_relaxed((meta->height - 1) << 16 | (meta->width - 1),
		       vfe->base + VFE_PDPC_END);
	vfe_rmw(vfe, VFE_MODULE_LENS_EN, 0, LENS_EN_PDPC);
}

static void ife_awb_bg_program(struct vfe_pix_meta *meta,
			       const struct ife17x_params_awb_bg *cfg)
{
	struct vfe_device *vfe = meta->vfe;
	u32 hn = clamp_t(u32, cfg->h_num, 1, IFE17X_AWB_BG_MAX_H);
	u32 vn = clamp_t(u32, cfg->v_num, 1, IFE17X_AWB_BG_MAX_V);
	u32 rw, rh = clamp_t(u32, (meta->height / vn) & ~1U, 2, 512);
	const u16 *t = cfg->threshold;

	/*
	 * Dual IFE: the master takes the left half of the grid columns over
	 * its stripe, the slave the right half over its own, and the slave's
	 * records follow the master's in the buffer. One column counts as two.
	 */
	if (meta->dual_slave || meta->dual_master)
		hn = max(hn, 2U);
	if (meta->dual_slave) {
		hn /= 2;
		meta->slave_stats_off = hn * vn * sizeof(struct ife17x_awb_bg_region);
	} else if (meta->dual_master) {
		hn -= hn / 2;
	}
	rw = clamp_t(u32, (meta->stripe_w / hn) & ~1U, 6, 390);

	writel_relaxed(meta->stripe_x & ~1U, vfe->base + VFE_AWB_BG_OFFSET);
	writel_relaxed((hn - 1) | (vn - 1) << 16, vfe->base + VFE_AWB_BG_NUM);
	writel_relaxed((rw - 1) | (rh - 1) << 16, vfe->base + VFE_AWB_BG_SIZE);
	writel_relaxed((t[0] & 0x3fff) | (t[2] & 0x3fff) << 16,
		       vfe->base + VFE_AWB_BG_HI_THR_0);
	writel_relaxed((t[1] & 0x3fff) | (t[3] & 0x3fff) << 16,
		       vfe->base + VFE_AWB_BG_HI_THR_1);
	writel_relaxed(0, vfe->base + VFE_AWB_BG_LO_THR_0);
	writel_relaxed(0, vfe->base + VFE_AWB_BG_LO_THR_1);
	writel_relaxed(0, vfe->base + VFE_AWB_BG_LUMA_CFG);
	writel_relaxed(0x00ffff00, vfe->base + VFE_AWB_BG_CFG);
	vfe_rmw(vfe, VFE_STATS_CFG, 0, STATS_CFG_FIXED);
	ife_stats_enable(meta, STATS_EN_AWB_BG, true);
}

static void ife_params_awb_bg(struct vfe_pix_meta *meta, union ife_params_block block)
{
	if (ife_block_disabled(block)) {
		ife_stats_enable(meta, STATS_EN_AWB_BG, false);
		return;
	}
	ife_awb_bg_program(meta, block.awb_bg);
}

/* A 64-bit DMI entry: the high half first, the low half writes the entry */
static void ife_dmi_write64(struct vfe_device *vfe, u64 v)
{
	writel_relaxed(upper_32_bits(v), vfe->base + VFE_DMI_DATA_HI);
	writel_relaxed(lower_32_bits(v), vfe->base + VFE_DMI_DATA_LO);
}

/* Biquad coefficients b10 b11 b12 a11 a12 b20 b21 b22 a21 a22 to registers */
static void ife_bf_iir(const s16 *c, u32 r[5])
{
	r[0] = (u16)c[0] | (u32)(u16)c[1] << 16;
	r[1] = (u16)c[2] | (u32)(u16)c[7] << 16;
	r[2] = (u16)c[3] | (u32)(u16)c[4] << 16;
	r[3] = (u16)c[5] | (u32)(u16)c[6] << 16;
	r[4] = (u16)c[8] | (u32)(u16)c[9] << 16;
}

/* 17 coring points of 5 bits, five to a register in 6-bit slots */
static void ife_bf_coring(struct vfe_device *vfe, u32 thr_reg, u32 reg0,
			  const struct ife17x_bf_filter *f)
{
	unsigned int i;
	u32 v[4] = {};

	for (i = 0; i < IFE17X_BF_CORING_POINTS; i++)
		v[i / 5] |= (u32)f->coring[i] << (6 * (i % 5));
	writel_relaxed(f->coring_threshold, vfe->base + thr_reg);
	for (i = 0; i < ARRAY_SIZE(v); i++)
		writel_relaxed(v[i], vfe->base + reg0 + 4 * i);
}

/*
 * Whether the regions [@first, @first + @count) fit the input of this IFE,
 * which starts at frame column @dx; the rest was checked in .buf_prepare()
 */
static bool ife_bf_fits(struct vfe_pix_meta *meta, const struct ife17x_params_bf *bf,
			unsigned int first, unsigned int count, u32 dx)
{
	unsigned int i;

	for (i = first; i < first + count; i++) {
		const struct ife17x_bf_roi *r = &bf->rois[i];

		if (r->left < dx + 76 || r->left - dx + r->width > meta->width - 12 ||
		    r->top + r->height > meta->height - 4 ||
		    r->width > meta->width / 2 || r->height > meta->height / 2)
			return false;
	}
	return true;
}

/*
 * Program the focus statistics. The ROI and gamma tables go to the banks not
 * in use, which the configuration register selects from the next register
 * update on.
 */
static void ife_bf_program(struct vfe_pix_meta *meta, const struct ife17x_params_bf *bf,
			   unsigned int first, unsigned int count, u32 dx)
{
	struct vfe_device *vfe = meta->vfe;
	u8 bank = !meta->bf_bank;
	u32 cfg = BF_CFG_H1_IIR | BF_CFG_V_IIR;
	unsigned int i;
	u32 r[5];

	if (bf->green)
		cfg |= BF_CFG_GREEN;
	if (bf->gamma_enable)
		cfg |= BF_CFG_GAMMA;
	if (bank)
		cfg |= BF_CFG_GAMMA_BANK | BF_CFG_ROI_BANK;

	/* No luma conversion, FIR or downscaler */
	for (i = 0; i < 2; i++)
		writel_relaxed(0, vfe->base + VFE_BF_LUMA(i));
	for (i = 0; i < 4; i++)
		writel_relaxed(0, vfe->base + VFE_BF_FIR(i));
	for (i = 0; i < 6; i++)
		writel_relaxed(0, vfe->base + VFE_BF_SCALE(i));

	ife_bf_iir(bf->h1.iir, r);
	for (i = 0; i < 5; i++)
		writel_relaxed(r[i], vfe->base + VFE_BF_H1_IIR(i));
	ife_bf_iir(bf->v.iir, r);
	for (i = 0; i < 5; i++)
		writel_relaxed(r[i], vfe->base + vfe_bf_v_iir[i]);
	writel_relaxed(bf->h1.shift | bf->v.shift << 16, vfe->base + VFE_BF_SHIFT);
	ife_bf_coring(vfe, VFE_BF_H1_CORING_THR, VFE_BF_H1_CORING(0), &bf->h1);
	ife_bf_coring(vfe, VFE_BF_V_CORING_THR, VFE_BF_V_CORING(0), &bf->v);
	writel_relaxed(bf->h1.coring_gain, vfe->base + VFE_BF_H1_CORING_GAIN);
	writel_relaxed(bf->v.coring_gain, vfe->base + VFE_BF_V_CORING_GAIN);
	writel_relaxed(32 << 16 | 64, vfe->base + VFE_BF_ACTIVE_START);
	writel_relaxed((meta->height - 32) << 16 | (meta->width - 64),
		       vfe->base + VFE_BF_ACTIVE_END);

	/*
	 * ROI table: an empty entry, the regions, an empty entry. A region:
	 * [12:0] height - 1, [25:14] width - 1, [40:27] top, [53:41] left in
	 * the input of this IFE, [62:54] its index in the whole list, which
	 * the record of the region carries.
	 */
	writel_relaxed((VFE_BF_ROI_DMI_BANK | bank) | DMI_CFG_AUTO_INC,
		       vfe->base + VFE_DMI_CFG);
	writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
	ife_dmi_write64(vfe, 0);
	for (i = first; i < first + count; i++) {
		const struct ife17x_bf_roi *roi = &bf->rois[i];

		ife_dmi_write64(vfe, (u64)(roi->height - 1) |
				     (u64)(roi->width - 1) << 14 |
				     (u64)roi->top << 27 |
				     (u64)(roi->left - dx) << 41 |
				     (u64)i << 54);
	}
	ife_dmi_write64(vfe, 0);

	/* Gamma: each point with the slope to the next, the last one to 0x8000 */
	if (bf->gamma_enable) {
		writel_relaxed((VFE_BF_GAMMA_DMI_BANK | bank) | DMI_CFG_AUTO_INC,
			       vfe->base + VFE_DMI_CFG);
		writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
		for (i = 0; i < IFE17X_BF_GAMMA_POINTS; i++) {
			int node = bf->gamma[i];
			int next = i < IFE17X_BF_GAMMA_POINTS - 1 ? bf->gamma[i + 1] : 0x8000;
			int slope = clamp(next - node, -0x2000, 0x1fff);

			writel_relaxed(node | ((u32)slope & 0x3fff) << 14,
				       vfe->base + VFE_DMI_DATA_LO);
		}
	}
	writel_relaxed(0, vfe->base + VFE_DMI_CFG);

	writel_relaxed(cfg, vfe->base + VFE_BF_CFG);
	meta->bf_bank = bank;
}

static void ife_params_bf(struct vfe_pix_meta *meta, union ife_params_block block)
{
	const struct ife17x_params_bf *bf = block.bf;
	unsigned int first = 0, count, max = IFE17X_BF_MAX_ROIS;
	u32 dx = 0;

	/* A disabled block may be its header alone */
	if (ife_block_disabled(block)) {
		ife_stats_enable(meta, STATS_EN_BF, false);
		return;
	}

	count = bf->num_rois;

	/*
	 * Dual IFE: the master measures the left stripe's part of the list,
	 * the slave the right stripe's in its own input columns.
	 */
	if (meta->dual_slave) {
		count = bf->num_left;
		max = IFE17X_BF_DUAL_LEFT_MAX;
	} else if (meta->dual_master) {
		first = bf->num_left;
		count = bf->num_rois - bf->num_left;
		max = IFE17X_BF_DUAL_RIGHT_MAX;
		dx = meta->frame_x;
	} else if (bf->num_left) {
		dev_warn_ratelimited(meta->vfe->camss->dev,
				     "VFE%d: focus regions split for two IFEs\n",
				     meta->vfe->id);
		ife_stats_enable(meta, STATS_EN_BF, false);
		return;
	}

	if (!count) {
		ife_stats_enable(meta, STATS_EN_BF, false);
		return;
	}

	if (count > max || !ife_bf_fits(meta, bf, first, count, dx)) {
		dev_warn_ratelimited(meta->vfe->camss->dev,
				     "VFE%d: focus regions outside the %ux%u input\n",
				     meta->vfe->id, meta->width, meta->height);
		ife_stats_enable(meta, STATS_EN_BF, false);
		return;
	}

	/* The tables take a few hundred writes: skip them when nothing changes */
	if (!meta->bf_valid || memcmp(&meta->bf, bf, sizeof(*bf))) {
		ife_bf_program(meta, bf, first, count, dx);
		meta->bf = *bf;
		meta->bf_valid = true;
	}
	ife_stats_enable(meta, STATS_EN_BF, true);
}

/*
 * Adaptive Bayer filter. The noise table goes to the inactive LUT bank, like
 * the gamma curves. Each LUT entry holds a node and the step down to the next.
 * The radial distances start from the IFE's own input column 0: with two IFEs
 * the right one starts at frame_x.
 */
static void ife_params_abf(struct vfe_pix_meta *meta, union ife_params_block block)
{
	const struct ife17x_params_abf *abf = block.abf;
	struct vfe_device *vfe = meta->vfe;
	u8 bank = !meta->abf_bank;
	s64 x, y, r2;
	unsigned int i, c;
	u32 cfg;

	if (ife_block_disabled(block)) {
		vfe_rmw(vfe, VFE_MODULE_LENS_EN, LENS_EN_ABF, 0);
		return;
	}

	writel_relaxed((VFE_ABF_DMI_BANK + bank) | DMI_CFG_AUTO_INC, vfe->base + VFE_DMI_CFG);
	writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
	for (i = 0; i < ABF_LUT_ENTRIES; i++)
		writel_relaxed(abf->noise[i] | (u32)(abf->noise[i] - abf->noise[i + 1]) << 9,
			       vfe->base + VFE_DMI_DATA_LO);
	writel_relaxed(0, vfe->base + VFE_DMI_CFG);

	for (c = 0; c < 4; c++)
		writel_relaxed(abf->curve_offset[c], vfe->base + VFE_ABF_CURVE_OFFSET(c));

	x = (s64)abf->radial_x + meta->frame_x;
	y = abf->radial_y;
	r2 = min_t(s64, x * x + y * y, ABF_R2_MAX);
	x = clamp_t(s64, x, -8192, 8191);
	writel_relaxed(((u32)x & 0x3fff) | ((u32)y & 0x3fff) << 16,
		       vfe->base + VFE_ABF_RADIAL_POS);
	writel_relaxed(r2, vfe->base + VFE_ABF_RADIAL_R2_INIT);
	for (i = 0; i < 2; i++)
		writel_relaxed(abf->radial_anchor[2 * i] | abf->radial_anchor[2 * i + 1] << 16,
			       vfe->base + VFE_ABF_RADIAL_ANCHOR(i));
	for (c = 0; c < 2; c++)
		for (i = 0; i < 4; i++)
			writel_relaxed(abf->radial_gain[c][i] | abf->radial_slope[c][i] << 8 |
				       abf->radial_shift[c][i] << 16,
				       vfe->base + VFE_ABF_RADIAL_SEG(c, i));
	writel_relaxed(abf->radial_r2_shift, vfe->base + VFE_ABF_RADIAL_R2_SHIFT);

	writel_relaxed(abf->bpc_fmax | abf->bpc_fmin << 8 | abf->bpc_offset << 16,
		       vfe->base + VFE_ABF_BPC_0);
	writel_relaxed(abf->bpc_min_shift | abf->bpc_max_shift << 4 | abf->black_level << 8,
		       vfe->base + VFE_ABF_BPC_1);
	writel_relaxed(abf->noise_prsv_anchor | abf->noise_prsv_gap << 16,
		       vfe->base + VFE_ABF_NOISE_PRSV_ANCHOR);
	for (c = 0; c < 2; c++)
		writel_relaxed(abf->noise_prsv_lo[c] |
			       ((u32)abf->noise_prsv_slope[c] & 0x3ff) << 12 |
			       abf->noise_prsv_shift[c] << 24,
			       vfe->base + VFE_ABF_NOISE_PRSV(c));

	cfg = bank ? ABF_CFG_LUT_BANK : 0;
	if (abf->flags & IFE17X_ABF_FL_FILTER)
		cfg |= ABF_CFG_FILTER;
	if (abf->flags & IFE17X_ABF_FL_RADIAL)
		cfg |= ABF_CFG_RADIAL;
	if (abf->flags & IFE17X_ABF_FL_BPC)
		cfg |= ABF_CFG_BPC;
	cfg |= abf->pixel_match[1] << 4 | abf->pixel_match[0] << 8;
	for (c = 0; c < 2; c++)
		cfg |= (abf->distance_ker[c][0] | abf->distance_ker[c][1] << 4 |
			abf->distance_ker[c][2] << 6) << (12 + 8 * c);
	writel_relaxed(cfg, vfe->base + VFE_ABF_CFG);
	meta->abf_bank = bank;
	vfe_rmw(vfe, VFE_MODULE_LENS_EN, 0, LENS_EN_ABF);
}

/* Bad pixel and bad cluster correction: the same values for both IFEs */
static void ife_params_bpc(struct vfe_pix_meta *meta, union ife_params_block block)
{
	const struct ife17x_params_bpc *bpc = block.bpc;
	struct vfe_device *vfe = meta->vfe;
	unsigned int i;

	if (ife_block_disabled(block)) {
		vfe_rmw(vfe, VFE_MODULE_LENS_EN, LENS_EN_BPC, 0);
		return;
	}

	writel_relaxed(bpc->flags | bpc->black_level << 3, vfe->base + VFE_BPC_CFG);
	for (i = 0; i < 4; i++)
		writel_relaxed(bpc->wb_ratio[i], vfe->base + VFE_BPC_WB_RATIO(i));
	writel_relaxed(bpc->bpc_offset | bpc->bcc_offset << 16, vfe->base + VFE_BPC_OFFSET);
	writel_relaxed(bpc->fmax | bpc->fmin << 8 | bpc->threshold << 16,
		       vfe->base + VFE_BPC_LIMITS);
	/* The Windows driver writes the statistics register with zero */
	writel_relaxed(0, vfe->base + VFE_BPC_STATS);
	vfe_rmw(vfe, VFE_MODULE_LENS_EN, 0, LENS_EN_BPC);
}

/*
 * Where an IFE's input starts in the LSC grid, x quads from the grid's left
 * edge and y from its top: the block, the subgrid inside it and the quad
 * inside that, and the vertical interpolation phase there.
 */
static void ife_lsc_start(const struct ife17x_params_lsc *lsc, u32 x, u32 y, u32 delta_y,
			  u32 *start0, u32 *start1)
{
	u32 sub_w = lsc->block_width >> lsc->subgrid_shift;
	u32 sub_h = lsc->block_height >> lsc->subgrid_shift;
	u32 bx = x % lsc->block_width, by = y % lsc->block_height;

	*start0 = (x / lsc->block_width) | (y / lsc->block_height) << 6 |
		  ((by % sub_h) * delta_y & 0xfffff) << 12;
	*start1 = (bx % sub_w) | (by % sub_h) << 12 | (bx / sub_w) << 24 |
		  (by / sub_h) << 28;
}

/*
 * Lens shading correction. The gains go to the inactive LUT banks, like the
 * gamma curves. Both IFEs take the frame's grid: the right one starts at its
 * first input column, frame_x.
 */
static void ife_params_lsc(struct vfe_pix_meta *meta, union ife_params_block block)
{
	const struct ife17x_params_lsc *lsc = block.lsc;
	struct vfe_device *vfe = meta->vfe;
	u32 nodes, sub_w, sub_h, delta_x, delta_y;
	u32 grid[3], start[2], x;
	u8 bank = !meta->lsc_bank;
	unsigned int i, t;

	/* A disabled block may be its header alone */
	if (ife_block_disabled(block)) {
		vfe_rmw(vfe, VFE_MODULE_LENS_EN, LENS_EN_LSC, 0);
		return;
	}

	nodes = (lsc->h_blocks + 1) * (lsc->v_blocks + 1);
	sub_w = lsc->block_width >> lsc->subgrid_shift;
	sub_h = lsc->block_height >> lsc->subgrid_shift;
	delta_x = 0x100000 / sub_w;
	delta_y = 0x100000 / sub_h;

	for (t = 0; t < 2; t++) {
		/* R and Gr, then B and Gb */
		const u16 *lo = lsc->gain[t ? 3 : 0], *hi = lsc->gain[t ? 2 : 1];

		writel_relaxed((VFE_LSC_DMI_BANK + 4 * bank + t) | DMI_CFG_AUTO_INC,
			       vfe->base + VFE_DMI_CFG);
		writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
		for (i = 0; i < nodes; i++)
			writel_relaxed(lo[i] | (u32)hi[i] << 13, vfe->base + VFE_DMI_DATA_LO);
	}
	writel_relaxed(0, vfe->base + VFE_DMI_CFG);

	grid[0] = (lsc->block_width - 1) | (lsc->block_height - 1) << 16;
	grid[1] = (sub_h - 1) | delta_y << 12 | lsc->subgrid_shift << 30;
	grid[2] = (sub_w - 1) | delta_x << 12;
	for (t = 0; t < 2; t++)
		for (i = 0; i < 3; i++)
			writel_relaxed(grid[i], vfe->base + VFE_LSC_GRID(t, i));

	/* The right group as the Windows driver writes it: the frame's start */
	ife_lsc_start(lsc, lsc->start_x, lsc->start_y, delta_y, &start[0], &start[1]);
	for (i = 0; i < 2; i++)
		writel_relaxed(start[i], vfe->base + VFE_LSC_START(1, i));

	x = lsc->start_x + meta->frame_x / 2;
	if (lsc->block_width - x % lsc->block_width <= 17 || sub_w - x % sub_w <= 8)
		dev_warn_once(vfe->camss->dev,
			      "VFE%d: LSC: input column %u starts too late in a block\n",
			      vfe->id, meta->frame_x);
	ife_lsc_start(lsc, x, lsc->start_y, delta_y, &start[0], &start[1]);
	for (i = 0; i < 2; i++)
		writel_relaxed(start[i], vfe->base + VFE_LSC_START(0, i));

	writel_relaxed((bank ? LSC_CFG_BANK : 0) | (lsc->h_blocks - 1) << 18 |
		       (lsc->v_blocks - 1) << 24, vfe->base + VFE_LSC_CFG);
	meta->lsc_bank = bank;
	vfe_rmw(vfe, VFE_MODULE_LENS_EN, 0, LENS_EN_LSC);
}

/* The outputs decide the frame's buffers (vfe17x_meta_next_outputs_locked()) */
static void ife_params_outputs(struct vfe_pix_meta *meta, union ife_params_block block)
{
}

/* Set with the frame's buffers too (vfe17x_meta_next_window_locked()) */
static void ife_params_display_window(struct vfe_pix_meta *meta, union ife_params_block block)
{
}

/*
 * Bayer histogram over a window of the frame. With two IFEs each counts the
 * part of the window in its stripe, in its own input columns (frame_x); one
 * whose stripe the window misses counts a 2 by 2 window at the start of the
 * stripe's last rows, so that both always write the buffer, at the end of the
 * frame like the grid. The write master completes when the window ends: a
 * window ending near the epoch 0 line could complete before the frame's
 * epoch, or be handled with it in one interrupt, as a frame earlier, so it is
 * stretched down to an eighth of the frame, milliseconds after the epoch.
 */
static void ife_params_bhist(struct vfe_pix_meta *meta, union ife_params_block block)
{
	const struct ife17x_params_bhist *h = block.bhist;
	struct vfe_device *vfe = meta->vfe;
	u32 sx = meta->stripe_x & ~1U, ex = (meta->stripe_x + meta->stripe_w) & ~1U;
	u32 x0, x1, y0, y1;

	if (ife_block_disabled(block)) {
		ife_stats_enable(meta, STATS_EN_BHIST, false);
		return;
	}
	if (h->channel > IFE17X_BHIST_CHANNEL_B) {
		dev_warn_ratelimited(vfe->camss->dev, "VFE%d: no histogram channel %u\n",
				     vfe->id, h->channel);
		ife_stats_enable(meta, STATS_EN_BHIST, false);
		return;
	}

	/* Frame columns to this IFE's input columns, inside its stripe */
	x0 = clamp_t(s64, (s64)(h->left & ~1U) - meta->frame_x, sx, ex);
	x1 = clamp_t(s64, (s64)((h->left + h->width) & ~1U) - meta->frame_x, sx, ex);
	y0 = min_t(u32, h->top & ~1U, meta->height & ~1U);
	y1 = min_t(u32, max_t(u32, (h->top + h->height) & ~1U, (meta->height / 8) & ~1U),
		   meta->height & ~1U);
	if (x1 < x0 + 2 || y1 < y0 + 2) {
		x0 = sx;
		x1 = sx + 2;
		y1 = meta->height & ~1U;
		y0 = y1 - 2;
	}

	writel_relaxed(x0 | y0 << 16, vfe->base + VFE_BHIST_OFFSET);
	writel_relaxed(((x1 - x0) / 2 - 1) | ((y1 - y0) / 2 - 1) << 16,
		       vfe->base + VFE_BHIST_SIZE);
	vfe_rmw(vfe, VFE_STATS_CFG, STATS_CFG_BHIST_CHANNEL,
		FIELD_PREP(STATS_CFG_BHIST_CHANNEL, h->channel) | STATS_CFG_BHIST_UNIFORM |
		STATS_CFG_FIXED);
	ife_stats_enable(meta, STATS_EN_BHIST, true);
}

static const ife_params_handler ife_params_handlers[] = {
	[IFE17X_PARAMS_BLOCK_BLS] = ife_params_bls,
	[IFE17X_PARAMS_BLOCK_WB] = ife_params_wb,
	[IFE17X_PARAMS_BLOCK_CCM] = ife_params_ccm,
	[IFE17X_PARAMS_BLOCK_GAMMA] = ife_params_gamma,
	[IFE17X_PARAMS_BLOCK_AWB_BG] = ife_params_awb_bg,
	[IFE17X_PARAMS_BLOCK_PDPC] = ife_params_pdpc,
	[IFE17X_PARAMS_BLOCK_BF] = ife_params_bf,
	[IFE17X_PARAMS_BLOCK_ABF] = ife_params_abf,
	[IFE17X_PARAMS_BLOCK_BPC] = ife_params_bpc,
	[IFE17X_PARAMS_BLOCK_LSC] = ife_params_lsc,
	[IFE17X_PARAMS_BLOCK_OUTPUTS] = ife_params_outputs,
	[IFE17X_PARAMS_BLOCK_DISPLAY_WINDOW] = ife_params_display_window,
	[IFE17X_PARAMS_BLOCK_BHIST] = ife_params_bhist,
};

static const struct v4l2_isp_params_block_type_info ife_params_block_types[] = {
	[IFE17X_PARAMS_BLOCK_BLS] = { .size = sizeof(struct ife17x_params_bls) },
	[IFE17X_PARAMS_BLOCK_WB] = { .size = sizeof(struct ife17x_params_wb) },
	[IFE17X_PARAMS_BLOCK_CCM] = { .size = sizeof(struct ife17x_params_ccm) },
	[IFE17X_PARAMS_BLOCK_GAMMA] = { .size = sizeof(struct ife17x_params_gamma) },
	[IFE17X_PARAMS_BLOCK_AWB_BG] = { .size = sizeof(struct ife17x_params_awb_bg) },
	[IFE17X_PARAMS_BLOCK_PDPC] = { .size = sizeof(struct ife17x_params_pdpc) },
	[IFE17X_PARAMS_BLOCK_BF] = { .size = sizeof(struct ife17x_params_bf) },
	[IFE17X_PARAMS_BLOCK_ABF] = { .size = sizeof(struct ife17x_params_abf) },
	[IFE17X_PARAMS_BLOCK_BPC] = { .size = sizeof(struct ife17x_params_bpc) },
	[IFE17X_PARAMS_BLOCK_LSC] = { .size = sizeof(struct ife17x_params_lsc) },
	[IFE17X_PARAMS_BLOCK_OUTPUTS] = { .size = sizeof(struct ife17x_params_outputs) },
	[IFE17X_PARAMS_BLOCK_DISPLAY_WINDOW] = {
		.size = sizeof(struct ife17x_params_display_window)
	},
	[IFE17X_PARAMS_BLOCK_BHIST] = { .size = sizeof(struct ife17x_params_bhist) },
};

static_assert(ARRAY_SIZE(ife_params_handlers) == ARRAY_SIZE(ife_params_block_types));

/*
 * The frame number of the epoch count @frame (struct vfe_pix_meta): 0 for the
 * first frame written. Lock held.
 */
static u32 ife_seq(const struct vfe_pix_meta *meta, u32 frame)
{
	return meta->seq_based && frame >= meta->seq_base ? frame - meta->seq_base : 0;
}

/* The next frame, @meta->frame, is the first one written. Lock held. */
static void ife_seq_first_written(struct vfe_pix_meta *meta)
{
	if (!meta->seq_based) {
		meta->seq_base = meta->frame;
		meta->seq_based = true;
	}
}

/* Apply the next queued parameters buffer; called with the lock held */
static bool ife_params_apply_next(struct vfe_pix_meta *meta, u32 sequence)
{
	struct v4l2_isp_params_buffer *config;
	struct ife_params_buf *buf;
	size_t off = 0;

	buf = list_first_entry_or_null(&meta->params_queue, struct ife_params_buf, queue);
	if (!buf)
		return false;
	list_del(&buf->queue);

	config = buf->config;
	while (off < config->data_size) {
		union ife_params_block block = { .data = &config->data[off] };

		/* Validated in .buf_prepare() */
		ife_params_handlers[block.header->type](meta, block);
		if (meta->dual_slave)
			ife_params_handlers[block.header->type](meta->dual_slave, block);
		off += block.header->size;
	}

	buf->vb.sequence = sequence;
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	return true;
}

/* Remember the AWB grid of a buffer applied before the frame size is known */
static void ife_params_scan_awb(struct vfe_pix_meta *meta,
				const struct v4l2_isp_params_buffer *config)
{
	size_t off = 0;

	while (off < config->data_size) {
		union ife_params_block block = { .data = &config->data[off] };

		if (block.header->type == IFE17X_PARAMS_BLOCK_AWB_BG &&
		    !ife_block_disabled(block)) {
			meta->awb_bg = *block.awb_bg;
			meta->awb_bg_pending = true;
		}
		off += block.header->size;
	}
}

/* -----------------------------------------------------------------------------
 * Statistics write master
 */

/* Point the write masters @wms at a statistics buffer for the next frame */
static void ife_stats_set_addr(struct vfe_pix_meta *meta, dma_addr_t addr, u32 wms)
{
	struct vfe_device *vfe = meta->vfe;

	/*
	 * Each address queues for one frame: a write master gets one only for
	 * a frame it writes.
	 */
	if (wms & BIT(VFE_WM_AWB_BG)) {
		writel_relaxed(lower_32_bits(addr),
			       vfe->base + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_AWB_BG));
		writel_relaxed(IFE_STATS_SIZE, vfe->base + VFE_BUS_WM_FRAME_INC(VFE_WM_AWB_BG));

		/*
		 * Dual IFE: the slave's grid goes after the master's, and its
		 * focus records to their fixed slot, after the header it
		 * writes before them.
		 */
		if (meta->dual_slave) {
			void __iomem *s = meta->dual_slave->vfe->base;

			writel_relaxed(lower_32_bits(addr) + meta->slave_stats_off,
				       s + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_AWB_BG));
			writel_relaxed(IFE_STATS_SIZE, s + VFE_BUS_WM_FRAME_INC(VFE_WM_AWB_BG));

			if (meta->dual_slave->stats_en & STATS_EN_BHIST) {
				writel_relaxed(lower_32_bits(addr) +
					       offsetof(struct ife17x_stats_buffer, bhist[1]),
					       s + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_BHIST));
				writel_relaxed(IFE_STATS_SIZE,
					       s + VFE_BUS_WM_FRAME_INC(VFE_WM_BHIST));
			}
			if (meta->dual_slave->stats_en & STATS_EN_BF) {
				writel_relaxed(lower_32_bits(addr) +
					       offsetof(struct ife17x_stats_buffer, bf_header) +
					       IFE17X_BF_DUAL_RIGHT *
					       sizeof(struct ife17x_bf_record),
					       s + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_BF));
				writel_relaxed(IFE_STATS_SIZE,
					       s + VFE_BUS_WM_FRAME_INC(VFE_WM_BF));
			}
		}
	}
	if (wms & BIT(VFE_WM_BF)) {
		writel_relaxed(lower_32_bits(addr) +
			       offsetof(struct ife17x_stats_buffer, bf_header),
			       vfe->base + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_BF));
		writel_relaxed(IFE_STATS_SIZE, vfe->base + VFE_BUS_WM_FRAME_INC(VFE_WM_BF));
	}
	if (wms & BIT(VFE_WM_BHIST)) {
		writel_relaxed(lower_32_bits(addr) +
			       offsetof(struct ife17x_stats_buffer, bhist[0]),
			       vfe->base + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_BHIST));
		writel_relaxed(IFE_STATS_SIZE, vfe->base + VFE_BUS_WM_FRAME_INC(VFE_WM_BHIST));
	}
}

/*
 * Master's lock held, the slave wrote a grid at @ts: the held buffer of that
 * frame is complete, older ones lack the slave's grid.
 */
static void ife_stats_dual_collect(struct vfe_pix_meta *meta, u64 ts)
{
	unsigned int i = 0;

	while (i < meta->nheld) {
		struct ife_stats_buf *buf = meta->held[i].buf;
		u64 t = meta->held[i].ts;
		bool ok = t + VFE_DUAL_MATCH_NS > ts && t < ts + VFE_DUAL_MATCH_NS;

		if (t + VFE_DUAL_MATCH_NS > ts && !ok) {
			i++;
			continue;
		}
		meta->nheld--;
		memmove(&meta->held[i], &meta->held[i + 1],
			(meta->nheld - i) * sizeof(meta->held[0]));
		vb2_buffer_done(&buf->vb.vb2_buf, ok ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);
		wake_up(&meta->stats_wq);
	}
}

/* Master's lock held: end the pairing, give waiting buffers back to the queue */
static void ife_dual_end(struct vfe_pix_meta *meta)
{
	while (meta->nheld)
		list_add(&meta->held[--meta->nheld].buf->queue, &meta->stats_queue);
	if (meta->dual_slave)
		meta->dual_slave->dual_master = NULL;
	meta->dual_slave = NULL;
	/* A statistics stop may wait for the held buffers */
	wake_up(&meta->stats_wq);
}

/*
 * Settle the statistics of the next frame: the next queued buffer if the frame
 * belongs to a request, else the scratch buffer, and queue the addresses of the
 * write masters its setup turns on. Once per frame, after its parameters; lock
 * held.
 */
static void ife_stats_settle(struct vfe_pix_meta *meta, bool written)
{
	struct ife_stats_buf *buf = NULL;

	if (meta->next_set)
		return;
	if (written) {
		buf = list_first_entry_or_null(&meta->stats_queue, struct ife_stats_buf, queue);
		if (buf)
			list_del(&buf->queue);
	}
	meta->stats_next = buf;
	meta->next_wms = ife_stats_wms(meta);
	meta->next_set = true;
	ife_stats_set_addr(meta, buf ? buf->addr : meta->scratch_addr, meta->next_wms);
}

/* A statistics buffer is complete, or failed; lock held */
static void ife_stats_finish(struct vfe_pix_meta *meta, struct ife_stats_buf *buf, bool ok)
{
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, IFE_STATS_SIZE);

	/* Dual IFE: wait for the slave's grid of the same frame */
	if (ok && meta->dual_slave &&
	    meta->slave_stats_ts + VFE_DUAL_MATCH_NS <= ktime_get_ns()) {
		if (meta->nheld == ARRAY_SIZE(meta->held)) {
			struct ife_stats_buf *old = meta->held[0].buf;

			meta->nheld--;
			memmove(&meta->held[0], &meta->held[1],
				meta->nheld * sizeof(meta->held[0]));
			vb2_buffer_done(&old->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
		/* Paired by completion time, not by the frame start time */
		meta->held[meta->nheld].buf = buf;
		meta->held[meta->nheld].ts = ktime_get_ns();

		meta->nheld++;
		wake_up(&meta->stats_wq);
		return;
	}

	vb2_buffer_done(&buf->vb.vb2_buf, ok ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);
	wake_up(&meta->stats_wq);
}

/* Give the buffers being written back to the queue, unused; lock held */
static void ife_stats_return(struct vfe_pix_meta *meta)
{
	/* Back to the head of the queue, the older one first */
	if (meta->stats_next) {
		list_add(&meta->stats_next->queue, &meta->stats_queue);
		meta->stats_next = NULL;
	}
	if (meta->stats_cur) {
		list_add(&meta->stats_cur->queue, &meta->stats_queue);
		meta->stats_cur = NULL;
	}
	meta->next_set = false;
}

static void ife_stats_wm_init(struct vfe_pix_meta *meta, bool written)
{
	static const u8 wms[] = { VFE_WM_BF, VFE_WM_AWB_BG, VFE_WM_BHIST };
	struct vfe_device *vfe = meta->vfe;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(wms); i++) {
		u8 wm = wms[i];

		writel_relaxed(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
		writel_relaxed(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));
		writel_relaxed(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
		writel_relaxed(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));
		writel_relaxed(0xf, vfe->base + VFE_BUS_WM_BURST_LIMIT(wm));
		writel_relaxed(0, vfe->base + VFE_BUS_WM_BUFFER_WIDTH_CFG(wm));
		writel_relaxed(0, vfe->base + VFE_BUS_WM_BUFFER_HEIGHT_CFG(wm));
		writel_relaxed(PACKER_FMT_PLAIN_64, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));
	}
	/* Frame 0 */
	meta->next_set = false;
	meta->next_got = 0;
	ife_stats_settle(meta, written);
	meta->stats_cur = meta->stats_next;
	meta->cur_wms = meta->next_wms;
	meta->cur_got = 0;
	meta->stats_next = NULL;
	meta->next_set = false;
	for (i = 0; i < ARRAY_SIZE(wms); i++)
		writel_relaxed(WM_CFG_FRAME_BASED, vfe->base + VFE_BUS_WM_CFG(wms[i]));
}

/* -----------------------------------------------------------------------------
 * Hooks from camss-vfe-17x.c
 */

/*
 * Stream start, from vfe_pix_configure() with the output lock held and
 * interrupts off, before the first register update. @written: the image path
 * writes frame 0, which then takes the first parameters and statistics
 * buffers; otherwise they wait for the first frame it writes.
 */
void vfe17x_meta_start(struct vfe_device *vfe, u32 width, u32 height, bool written)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	if (!meta)
		return;

	spin_lock(&meta->lock);
	meta->width = width;
	meta->height = height;
	meta->frame = 0;
	meta->seq_base = 0;
	meta->seq_based = written;
	meta->wedged = false;
	meta->next_written = written;
	meta->join_open = false;
	meta->stripe_x = 0;
	meta->stripe_w = width;
	meta->frame_x = 0;
	meta->slave_stats_off = 0;
	meta->slave_stats_ts = 0;
	meta->slave_got = 0;
	meta->slave_grid_ts = 0;
	meta->nheld = 0;
	meta->dual_slave = NULL;
	meta->dual_master = NULL;

	/*
	 * Dual IFE master (the slave started first): the master's grid and
	 * parameters cover the left stripe up to its output width, the slave's
	 * the right stripe from its crop.
	 */
	if (vfe->dual.master && vfe->dual.peer && vfe->dual.peer->pix_meta) {
		struct vfe_pix_meta *slave = vfe->dual.peer->pix_meta;
		const struct v4l2_rect *sc = &vfe->dual.peer->line[VFE_LINE_PIX].crop;

		meta->stripe_w = vfe->dual.x_offset;
		slave->stripe_x = sc->left;
		slave->stripe_w = sc->width;
		slave->frame_x = vfe->dual.x_offset - sc->left;
		if (slave->frame_x % IFE17X_PDPC_MASK_ROWS)
			dev_warn(vfe->camss->dev,
				 "VFE%d: dual stripe at column %u: PDPC tiles not aligned\n",
				 vfe->dual.peer->id, slave->frame_x);
		slave->dual_master = meta;
		meta->dual_slave = slave;
	}
	/* The default setup filled every gamma bank and selected bank 0 */
	meta->gamma_bank = 0;
	/* The VFE was reset: ABF is off and selects bank 12, LSC 14 and 15 */
	meta->abf_bank = 0;
	meta->lsc_bank = 0;
	/* The VFE was reset: the mask bank must be written again */
	meta->pdpc_mask_valid = false;
	/* Likewise the focus statistics, and no statistics are on */
	meta->bf_valid = false;
	meta->bf_bank = 0;
	meta->stats_en = 0;
	/*
	 * The histogram RAM keeps its counts over the reset, and from power up
	 * holds garbage: the first frame's histogram would add to it. Clear it,
	 * like the Windows driver does.
	 */
	writel_relaxed(VFE_BHIST_DMI_BANK | DMI_CFG_AUTO_INC, vfe->base + VFE_DMI_CFG);
	writel_relaxed(0, vfe->base + VFE_DMI_ADDR);
	for (unsigned int i = 0; i < IFE17X_BHIST_BINS; i++)
		writel_relaxed(0, vfe->base + VFE_DMI_DATA_LO);
	writel_relaxed(0, vfe->base + VFE_DMI_CFG);
	writel_relaxed(CAMIF_EPOCH_CFG(height), vfe->base + VFE_CAMIF_EPOCH_IRQ);

	if (meta->awb_bg_pending) {
		ife_awb_bg_program(meta, &meta->awb_bg);
		if (meta->dual_slave)
			ife_awb_bg_program(meta->dual_slave, &meta->awb_bg);
	}
	if (written)
		ife_params_apply_next(meta, 0);
	ife_stats_wm_init(meta, written);
	meta->running = true;
	spin_unlock(&meta->lock);
}

/* Stream stop, process context, before the VFE is reset */
void vfe17x_meta_stop(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	struct vfe_pix_meta *master;
	unsigned long flags;

	if (!meta)
		return;

	spin_lock_irqsave(&meta->lock, flags);
	meta->running = false;
	/*
	 * Without the reset, when another line keeps the VFE, the statistics
	 * would go on: stop them and their write masters before their
	 * buffers go back to the queue.
	 */
	ife_stats_enable(meta, STATS_EN_AWB_BG | STATS_EN_BF | STATS_EN_BHIST, false);
	writel_relaxed(REG_UPDATE_PIX, vfe->base + VFE_REG_UPDATE_CMD);
	writel_relaxed(0, vfe->base + VFE_BUS_WM_CFG(VFE_WM_BF));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_CFG(VFE_WM_AWB_BG));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_CFG(VFE_WM_BHIST));
	ife_stats_return(meta);
	ife_dual_end(meta);
	wake_up(&meta->stats_wq);
	spin_unlock_irqrestore(&meta->lock, flags);

	/* Dual IFE slave: leave the master's pairing */
	master = READ_ONCE(meta->dual_master);
	if (master) {
		spin_lock_irqsave(&master->lock, flags);
		if (master->dual_slave == meta)
			ife_dual_end(master);
		meta->dual_master = NULL;
		spin_unlock_irqrestore(&master->lock, flags);
	}
}

/* The pixel path is wedged: the parameters and statistics queues must restart */
void vfe17x_meta_fatal(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	unsigned long flags;

	if (!meta)
		return;

	/*
	 * Its frames will not end: a statistics stop need not wait for them,
	 * the pixel path stop resets the VFE.
	 */
	spin_lock_irqsave(&meta->lock, flags);
	meta->wedged = true;
	wake_up(&meta->stats_wq);
	spin_unlock_irqrestore(&meta->lock, flags);

	if (vb2_is_streaming(&meta->stats_q))
		vb2_queue_error(&meta->stats_q);
	if (vb2_is_streaming(&meta->params_q))
		vb2_queue_error(&meta->params_q);
}

/*
 * CAMIF epoch 0 interrupt: program the parameters of a later frame. Returns
 * true if registers changed and need a PIX register update.
 */
/* The pixel path of the VFE runs, with its parameters and statistics */
bool vfe17x_meta_running(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	unsigned long flags;
	bool running;

	if (!meta)
		return false;

	spin_lock_irqsave(&meta->lock, flags);
	running = meta->running;
	spin_unlock_irqrestore(&meta->lock, flags);

	return running;
}

/* Time between the last two epochs 0 of the pixel path, 0 if unknown */
/* The pixel path is wedged until the next start (vfe17x_meta_fatal()) */
bool vfe17x_meta_wedged(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	unsigned long flags;
	bool wedged;

	if (!meta)
		return false;

	spin_lock_irqsave(&meta->lock, flags);
	wedged = meta->wedged;
	spin_unlock_irqrestore(&meta->lock, flags);

	return wedged;
}

u64 vfe17x_meta_period_ns(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	unsigned long flags;
	u64 period;

	if (!meta)
		return 0;

	spin_lock_irqsave(&meta->lock, flags);
	period = meta->period_ns;
	spin_unlock_irqrestore(&meta->lock, flags);

	return period;
}

/* As vfe17x_meta_running(), with the lock held (vfe17x_meta_lock()) */
bool vfe17x_meta_running_locked(struct vfe_device *vfe)
{
	return vfe->pix_meta && vfe->pix_meta->running;
}

/*
 * A buffer joining the next frame after epoch 0 (vfe17x_queue_buffer())
 * decides under the meta lock, which epoch 1 takes to settle the frame: the
 * two do not interleave. Output lock held, interrupts off.
 */
void vfe17x_meta_lock(struct vfe_device *vfe)
{
	if (vfe->pix_meta)
		spin_lock(&vfe->pix_meta->lock);
}

void vfe17x_meta_unlock(struct vfe_device *vfe)
{
	if (vfe->pix_meta)
		spin_unlock(&vfe->pix_meta->lock);
}

/* From epoch 0 to epoch 1 of a frame, the next frame can still be joined */
bool vfe17x_meta_join_open_locked(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	return meta && meta->running && meta->join_open;
}

/*
 * The outputs the parameters buffer the next frame written takes names
 * (struct ife17x_params_outputs), 0 without the block or without a buffer.
 * Lock held (vfe17x_meta_lock()).
 */
u32 vfe17x_meta_next_outputs_locked(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	struct ife_params_buf *buf;

	if (!meta)
		return 0;

	buf = list_first_entry_or_null(&meta->params_queue, struct ife_params_buf, queue);

	return buf ? buf->outputs : 0;
}

/*
 * The display window the parameters buffer the next frame written takes sets
 * (struct ife17x_params_display_window), or NULL. Lock held.
 */
const struct ife17x_params_display_window *
vfe17x_meta_next_window_locked(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	struct ife_params_buf *buf;

	if (!meta)
		return NULL;

	buf = list_first_entry_or_null(&meta->params_queue, struct ife_params_buf, queue);

	return buf && buf->window_set ? &buf->window : NULL;
}

/*
 * The outputs the next frame needs got their buffers after epoch 0, before
 * epoch 1 of the current frame: the next frame belongs to a request after all.
 * Apply its parameters and queue its statistics buffer now, before the next
 * frame starts. Returns true if registers need a PIX register update.
 */
bool vfe17x_meta_join_locked(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	bool applied;

	if (!vfe17x_meta_join_open_locked(vfe) || meta->next_written)
		return false;

	meta->next_written = true;
	ife_seq_first_written(meta);
	applied = ife_params_apply_next(meta, ife_seq(meta, meta->frame));
	ife_stats_settle(meta, true);
	return applied;
}

/*
 * @next_written: the image path has a buffer for the next frame. A frame it
 * skips, for want of a buffer, belongs to no request: it takes neither the
 * next parameters nor a statistics buffer, which stay with the frame of
 * their request.
 */
/*
 * Epoch 0 of the pixel path, at time @now: the interrupt's entry, not this
 * call, which comes after the buffers done in the same interrupt.
 */
bool vfe17x_meta_epoch(struct vfe_device *vfe, bool next_written, u64 now)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	bool applied = false;

	if (!meta)
		return false;

	struct v4l2_event ev = { .type = V4L2_EVENT_FRAME_SYNC };
	bool running, event = false;

	spin_lock(&meta->lock);
	running = meta->running;
	/* A dual IFE slave's statistics go with the master's frames */
	if (running && meta->frame && !meta->dual_master) {
		/* The frame before this one is over, complete or not */
		if (meta->stats_cur) {
			dev_dbg(vfe->camss->dev,
				"VFE%d: frame %u statistics incomplete, write masters %#x missing\n",
				vfe->id, meta->frame - 1, meta->cur_wms & ~meta->cur_got);
			ife_stats_finish(meta, meta->stats_cur, false);
		}
		/*
		 * Not settled by epoch 1: the frame started without statistics
		 * addresses, and the write masters skip it. An address queued
		 * now would go to the frame after, and every later one too.
		 */
		if (!meta->next_set) {
			if (meta->stats_en && vb2_is_streaming(&meta->stats_q))
				dev_warn_ratelimited(vfe->camss->dev,
						     "VFE%d: frame %u statistics not settled in time\n",
						     vfe->id, meta->frame);
			meta->stats_next = NULL;
			meta->next_wms = 0;
		}
		meta->stats_cur = meta->stats_next;
		meta->cur_wms = meta->next_wms;
		meta->cur_got = meta->next_got;
		meta->stats_next = NULL;
		meta->next_got = 0;
		meta->next_set = false;
	}
	if (running) {
		u32 frame = meta->frame;

		if (meta->frame)
			meta->period_ns = now - meta->epoch_ns;
		meta->epoch_ns = now;
		meta->frame++;
		meta->next_written = next_written;
		meta->join_open = true;
		if (next_written) {
			ife_seq_first_written(meta);
			applied = ife_params_apply_next(meta, ife_seq(meta, meta->frame));
			if (!meta->dual_master)
				ife_stats_settle(meta, true);
		}
		/* Not before the first frame written: not one of the stream */
		event = meta->seq_based && frame >= meta->seq_base;
		ev.u.frame_sync.frame_sequence = ife_seq(meta, frame);
	}
	spin_unlock(&meta->lock);

	/* Frame start for the sensor control timing in userspace */
	if (event)
		v4l2_event_queue(vfe->line[VFE_LINE_PIX].subdev.devnode, &ev);

	return applied;
}

/*
 * Epoch 1, three quarters into the frame: the next frame is settled. No
 * buffer joins it any more; without one it writes its statistics to the
 * scratch buffer. @late: the next frame already started (its SOF is in the
 * same interrupt), so addresses queued now would go to the frame after it:
 * queue none, the write masters skip the frame.
 */
void vfe17x_meta_epoch1(struct vfe_device *vfe, bool late)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	if (!meta)
		return;

	spin_lock(&meta->lock);
	/* Before epoch 0 of the first frame, it is a frame before the stream */
	if (meta->running && meta->frame) {
		meta->join_open = false;
		if (!meta->dual_master && !late)
			ife_stats_settle(meta, false);
	}
	spin_unlock(&meta->lock);
}

/*
 * Whether the first frame of the stream started (its epoch came). A buffer
 * that completes before belongs to a frame that began before the stream:
 * not one of its frames.
 */
bool vfe17x_meta_frame_started(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	unsigned long flags;
	bool started;

	if (!meta)
		return true;

	spin_lock_irqsave(&meta->lock, flags);
	started = meta->running && meta->frame;
	spin_unlock_irqrestore(&meta->lock, flags);

	return started;
}

/* The pixel path runs and its first frame did not start yet */
bool vfe17x_meta_before_first_frame(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	unsigned long flags;
	bool before;

	if (!meta)
		return false;

	spin_lock_irqsave(&meta->lock, flags);
	before = meta->running && !meta->frame;
	spin_unlock_irqrestore(&meta->lock, flags);

	return before;
}

/*
 * Number a completed image of the pixel path, or of an RDI line of the same
 * VFE while the pixel path runs, by the epochs like the statistics, so images,
 * statistics and parameters keep the same frame numbers when the driver runs
 * out of image buffers. @timestamp: the start of the frame (epoch 0, line
 * CAMIF_EPOCH0_LINE), for the pixel path's buffers rather than their
 * completion: its outputs of a frame end at different times (the display
 * output ends early on a window that leaves out the bottom of the frame), and
 * all should carry the same time.
 */
void vfe17x_meta_frame(struct vfe_device *vfe, u32 *sequence, u64 *timestamp)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	unsigned long flags;

	if (!meta)
		return;

	spin_lock_irqsave(&meta->lock, flags);
	if (meta->running && meta->frame) {
		*sequence = ife_seq(meta, meta->frame - 1);
		*timestamp = meta->epoch_ns;
	}
	spin_unlock_irqrestore(&meta->lock, flags);
}

/* Bus interrupt: statistics write masters done (see struct vfe_pix_meta) */
void vfe17x_meta_bus(struct vfe_device *vfe, u32 bus_status1)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	u32 wms = bus_status1 & IFE_STATS_WMS;
	struct vfe_pix_meta *master;
	struct ife_stats_buf *done;

	if (!meta || !wms)
		return;

	/*
	 * Dual IFE slave: its grid, and its histogram when on, went into the
	 * master's buffer. The grid completes at the end of the frame, its
	 * focus records before; the histogram when its window ends, earlier
	 * in the frame or, with a window down to the bottom, just after the
	 * grid. A histogram VFE_DUAL_MATCH_NS or more after the grid is of a
	 * later frame: the grid's frame had none (the histogram turned on
	 * after it). The slave's part of a frame is written once both are
	 * done, and pairs with the master's by the time of its grid. What it
	 * waits for is the setup of the frame after the grid's, applied at
	 * the grid frame's epoch 0: the frame before the histogram turns on
	 * is dropped, and the one before it turns off may complete before the
	 * slave's histogram is written.
	 */
	master = READ_ONCE(meta->dual_master);
	if (master) {
		u64 now = ktime_get_ns();

		spin_lock(&master->lock);
		if (master->dual_slave == meta) {
			u32 expect = ife_stats_wms(meta) &
				     (BIT(VFE_WM_AWB_BG) | BIT(VFE_WM_BHIST));

			if (wms & BIT(VFE_WM_BHIST)) {
				if (meta->slave_got & BIT(VFE_WM_AWB_BG) &&
				    now >= meta->slave_grid_ts + VFE_DUAL_MATCH_NS)
					meta->slave_got = 0;
				meta->slave_got |= BIT(VFE_WM_BHIST);
			}
			if (wms & BIT(VFE_WM_AWB_BG)) {
				/* An earlier grid's frame never completed */
				if (meta->slave_got & BIT(VFE_WM_AWB_BG))
					meta->slave_got = 0;
				meta->slave_got |= BIT(VFE_WM_AWB_BG);
				meta->slave_grid_ts = now;
			}
			if (expect & BIT(VFE_WM_AWB_BG) &&
			    (meta->slave_got & expect) == expect) {
				meta->slave_got = 0;
				master->slave_stats_ts = meta->slave_grid_ts;
				ife_stats_dual_collect(master, meta->slave_grid_ts);
			}
		}
		spin_unlock(&master->lock);
		return;
	}

	spin_lock(&meta->lock);
	if (!meta->running) {
		spin_unlock(&meta->lock);
		return;
	}
	/*
	 * Before the first epoch: statistics of a frame that began before the
	 * stream, not one of its frames. Have those masters write the buffer
	 * again, by the first frame, rather than number them as frame 0.
	 */
	if (!meta->frame) {
		ife_stats_set_addr(meta, meta->stats_cur ? meta->stats_cur->addr :
				   meta->scratch_addr, wms);
		spin_unlock(&meta->lock);
		dev_dbg(vfe->camss->dev,
			"VFE%d: statistics before the first frame, buffer reused\n", vfe->id);
		return;
	}

	/*
	 * A write master already done with this frame is done with the next
	 * one, whose epoch 0 is in the same interrupt.
	 */
	meta->next_got |= wms & meta->cur_got;
	meta->cur_got |= wms;

	done = meta->stats_cur;
	if (done && (meta->cur_got & meta->cur_wms) == meta->cur_wms) {
		meta->stats_cur = NULL;
		/*
		 * Frame k's statistics complete between epochs 0 of frames k
		 * and k + 1: number them from the epochs, so they match the
		 * image even if they were turned on after the stream start,
		 * and stamp them with the start of the frame like the images
		 * (vfe17x_meta_frame()).
		 */
		done->vb.sequence = ife_seq(meta, meta->frame - 1);
		done->vb.vb2_buf.timestamp = meta->epoch_ns;
		ife_stats_finish(meta, done, true);
	}
	spin_unlock(&meta->lock);
}

/* -----------------------------------------------------------------------------
 * Video nodes
 */

static int ife_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "qcom-camss", sizeof(cap->driver));
	strscpy(cap->card, "Qualcomm Camera Subsystem", sizeof(cap->card));
	return 0;
}

static int ife_params_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_META_FMT_QCOM_IFE17X_PARAMS;
	return 0;
}

static int ife_params_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	f->fmt.meta.dataformat = V4L2_META_FMT_QCOM_IFE17X_PARAMS;
	f->fmt.meta.buffersize = IFE_PARAMS_SIZE;
	return 0;
}

static int ife_stats_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_META_FMT_QCOM_IFE17X_STATS;
	return 0;
}

static int ife_stats_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	f->fmt.meta.dataformat = V4L2_META_FMT_QCOM_IFE17X_STATS;
	f->fmt.meta.buffersize = IFE_STATS_SIZE;
	return 0;
}

static const struct v4l2_ioctl_ops ife_params_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enum_fmt_meta_out = ife_params_enum_fmt,
	.vidioc_g_fmt_meta_out = ife_params_g_fmt,
	.vidioc_s_fmt_meta_out = ife_params_g_fmt,
	.vidioc_try_fmt_meta_out = ife_params_g_fmt,
	.vidioc_querycap = ife_querycap,
};

static const struct v4l2_ioctl_ops ife_stats_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enum_fmt_meta_cap = ife_stats_enum_fmt,
	.vidioc_g_fmt_meta_cap = ife_stats_g_fmt,
	.vidioc_s_fmt_meta_cap = ife_stats_g_fmt,
	.vidioc_try_fmt_meta_cap = ife_stats_g_fmt,
	.vidioc_querycap = ife_querycap,
};

static const struct v4l2_file_operations ife_meta_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int ife_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			   unsigned int *num_planes, unsigned int sizes[],
			   struct device *alloc_devs[], size_t size)
{
	if (*num_planes) {
		if (*num_planes > 1 || sizes[0] < size)
			return -EINVAL;
		return 0;
	}
	*num_planes = 1;
	sizes[0] = size;
	return 0;
}

static int ife_params_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				  unsigned int *num_planes, unsigned int sizes[],
				  struct device *alloc_devs[])
{
	return ife_queue_setup(q, num_buffers, num_planes, sizes, alloc_devs,
			       IFE_PARAMS_SIZE);
}

static int ife_params_buf_init(struct vb2_buffer *vb)
{
	struct ife_params_buf *buf = to_params_buf(to_vb2_v4l2_buffer(vb));

	buf->config = kvmalloc(IFE_PARAMS_SIZE, GFP_KERNEL);
	return buf->config ? 0 : -ENOMEM;
}

static void ife_params_buf_cleanup(struct vb2_buffer *vb)
{
	struct ife_params_buf *buf = to_params_buf(to_vb2_v4l2_buffer(vb));

	kvfree(buf->config);
	buf->config = NULL;
}

static int ife_bf_check_filter(struct device *dev, const struct ife17x_bf_filter *f)
{
	unsigned int i;

	if (f->shift > 15 || f->coring_gain > 1023 || f->coring_threshold > 0x1ffff ||
	    f->reserved0 || memchr_inv(f->reserved, 0, sizeof(f->reserved)))
		return -EINVAL;
	for (i = 0; i < IFE17X_BF_CORING_POINTS; i++)
		if (f->coring[i] > 31)
			return -EINVAL;
	return 0;
}

/*
 * What the hardware takes, as the Windows driver checks it, short of the frame
 * size: that one is checked when the block is applied.
 */
static int ife_bf_check(struct device *dev, const struct ife17x_params_bf *bf)
{
	unsigned int i, j, n;

	if (!bf->num_rois || bf->num_rois > IFE17X_BF_MAX_ROIS || bf->num_left > bf->num_rois ||
	    bf->green > 1 ||
	    bf->gamma_enable > 1 || bf->reserved ||
	    ife_bf_check_filter(dev, &bf->h1) || ife_bf_check_filter(dev, &bf->v)) {
		dev_dbg(dev, "BF: invalid configuration\n");
		return -EINVAL;
	}
	for (i = 0; i < IFE17X_BF_GAMMA_POINTS; i++) {
		if (bf->gamma[i] > 0x3fff) {
			dev_dbg(dev, "BF: invalid gamma point %u\n", i);
			return -EINVAL;
		}
	}

	for (i = 0; i < bf->num_rois; i++) {
		const struct ife17x_bf_roi *r = &bf->rois[i];

		/* Even, inside the fields of the ROI table */
		if ((r->left | r->top | r->width | r->height) & 1 ||
		    r->left < 76 || r->top < 36 || r->width < 6 || r->height < 8 ||
		    r->width > 4096 || r->left + r->width > 8192 ||
		    r->top + r->height > 16384) {
			dev_dbg(dev, "BF: invalid region %u\n", i);
			return -EINVAL;
		}

		/* In raster order of their ends, which are apart, in each part */
		if (i && i != bf->num_left) {
			const struct ife17x_bf_roi *p = &bf->rois[i - 1];
			int pb = p->top + p->height, pr = p->left + p->width;
			int rb = r->top + r->height, rr = r->left + r->width;

			if (rb < pb || (rb == pb && rr <= pr) ||
			    abs(rr - pr) < IFE_BF_END_SPACING) {
				dev_dbg(dev, "BF: region %u out of order\n", i);
				return -EINVAL;
			}
		}

		for (j = 0, n = 0; j < bf->num_rois; j++) {
			const struct ife17x_bf_roi *o = &bf->rois[j];

			/* One IFE takes each part */
			if (o->top == r->top && (j < bf->num_left) == (i < bf->num_left))
				n++;
			if (j != i && o->left < r->left + r->width &&
			    r->left < o->left + o->width &&
			    o->top < r->top + r->height &&
			    r->top < o->top + o->height) {
				dev_dbg(dev, "BF: regions %u and %u overlap\n", i, j);
				return -EINVAL;
			}
		}
		if (n > IFE_BF_ROIS_PER_ROW) {
			dev_dbg(dev, "BF: more than %u regions start on row %u\n",
				IFE_BF_ROIS_PER_ROW, r->top);
			return -EINVAL;
		}
	}
	return 0;
}

/* Ranges and reserved fields of the blocks other than BF */
static int ife_abf_check(struct device *dev, const struct ife17x_params_abf *abf)
{
	static const u8 ker_min[3] = { 1, 0, 0 }, ker_max[3] = { 4, 2, 1 };
	unsigned int i, c;

	/* The LUT holds each step down to the next node in 9 bits */
	for (i = 0; i < IFE17X_ABF_NOISE_NODES; i++)
		if (abf->noise[i] > 511 || (i && abf->noise[i] > abf->noise[i - 1]))
			goto invalid;
	if (abf->flags & ~(IFE17X_ABF_FL_FILTER | IFE17X_ABF_FL_RADIAL | IFE17X_ABF_FL_BPC))
		goto invalid;
	for (c = 0; c < 2; c++) {
		if (abf->pixel_match[c] > 4 || abf->noise_prsv_lo[c] > 256 ||
		    abf->noise_prsv_slope[c] < -256 || abf->noise_prsv_slope[c] > 256 ||
		    abf->noise_prsv_shift[c] > 10)
			goto invalid;
		for (i = 0; i < 3; i++)
			if (abf->distance_ker[c][i] < ker_min[i] ||
			    abf->distance_ker[c][i] > ker_max[i])
				goto invalid;
		for (i = 0; i < 4; i++)
			if (abf->radial_shift[c][i] > 15)
				goto invalid;
	}
	for (c = 0; c < 4; c++)
		if (abf->curve_offset[c] > 64)
			goto invalid;
	for (i = 0; i < 4; i++)
		if (abf->radial_anchor[i] > 4095 ||
		    (i && abf->radial_anchor[i] < abf->radial_anchor[i - 1]))
			goto invalid;
	if (abf->bpc_fmax > 63 || abf->bpc_fmin > 63 || abf->bpc_max_shift > 14 ||
	    abf->bpc_min_shift > 14 || abf->bpc_offset > 4095 || abf->black_level > 4095 ||
	    abf->noise_prsv_anchor > 1023 || abf->noise_prsv_gap > 1023 ||
	    abf->radial_x < -8192 || abf->radial_x > 8191 ||
	    abf->radial_y < -8192 || abf->radial_y > 8191 || abf->radial_r2_shift > 15 ||
	    memchr_inv(abf->reserved, 0, sizeof(abf->reserved)))
		goto invalid;
	return 0;

invalid:
	dev_dbg(dev, "invalid adaptive Bayer filter parameters\n");
	return -EINVAL;
}

static int ife_lsc_check(struct device *dev, const struct ife17x_params_lsc *lsc)
{
	unsigned int nodes = (lsc->h_blocks + 1) * (lsc->v_blocks + 1);
	unsigned int c, i, f = lsc->subgrid_shift;

	if (!lsc->h_blocks || !lsc->v_blocks || lsc->h_blocks > IFE17X_LSC_MAX_H_BLOCKS ||
	    lsc->v_blocks > IFE17X_LSC_MAX_V_BLOCKS || nodes > IFE17X_LSC_MAX_NODES || f > 3 ||
	    lsc->block_width < 18 || lsc->block_width > 2048 ||
	    lsc->block_height < 9 || lsc->block_height > 1024 ||
	    lsc->block_width & ((1 << f) - 1) || lsc->block_height & ((1 << f) - 1) ||
	    lsc->block_width >> f < 9 || lsc->block_height >> f < 9 ||
	    lsc->start_x >= lsc->block_width * lsc->h_blocks ||
	    lsc->start_y >= lsc->block_height * lsc->v_blocks ||
	    lsc->reserved0 || lsc->reserved)
		goto invalid;
	for (c = 0; c < 4; c++)
		for (i = 0; i < nodes; i++)
			if (lsc->gain[c][i] > 8191)
				goto invalid;
	return 0;

invalid:
	dev_dbg(dev, "invalid lens shading correction parameters\n");
	return -EINVAL;
}

/*
 * A display window: even values and the smallest compose rectangle always;
 * the rest against the display subdev's sink and crop size once they are
 * fixed, while it streams. Else vfe17x_disp_next_window() checks it.
 */
static bool ife_display_window_valid(struct vfe_pix_meta *meta,
				     const struct ife17x_params_display_window *w)
{
	struct vfe_pix_disp *disp = meta->vfe->pix_disp;
	struct v4l2_rect compose = { 0, 0, w->compose_width, w->compose_height };
	struct v4l2_rect crop;

	if (w->reserved[0] || w->reserved[1] ||
	    (w->compose_width | w->compose_height | w->crop_left |
			    w->crop_top) & 1 ||
	    w->compose_width < VFE_DISP_MIN_WIDTH || w->compose_height < VFE_DISP_MIN_HEIGHT)
		return false;
	if (!disp || !READ_ONCE(disp->subdev.s_stream_enabled))
		return true;

	crop = (struct v4l2_rect){ w->crop_left, w->crop_top, disp->crop.width,
				   disp->crop.height };

	return vfe17x_disp_window_valid(&disp->fmt[MSM_VFE_PAD_SINK], &compose, &crop);
}

static int ife_params_check(struct vfe_pix_meta *meta, struct device *dev,
			    union ife_params_block block)
{
	unsigned int i, j;

	switch (block.header->type) {
	case IFE17X_PARAMS_BLOCK_BLS:
		if (block.bls->offset > 8191 ||
		    memchr_inv(block.bls->reserved, 0, sizeof(block.bls->reserved)))
			goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_WB:
		for (i = 0; i < 3; i++)
			if (block.wb->gain[i] > 4095)
				goto invalid;
		if (block.wb->reserved)
			goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_CCM:
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++)
				if (block.ccm->coeff[i][j] < -2048 || block.ccm->coeff[i][j] > 2047)
					goto invalid;
			if (block.ccm->offset[i] < -1024 || block.ccm->offset[i] > 1023)
				goto invalid;
		}
		if (block.ccm->shift > 3 ||
		    memchr_inv(block.ccm->reserved, 0, sizeof(block.ccm->reserved)))
			goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_GAMMA:
		for (i = 0; i < 3; i++) {
			const u16 *t = block.gamma->curve[i];

			for (j = 0; j < IFE17X_GAMMA_NODES; j++)
				if (t[j] > (j < IFE17X_GAMMA_NODES - 1 ? 1023 : 1024))
					goto invalid;
			/* The LUT holds each step to the next node in 10 signed bits */
			for (j = 0; j + 1 < IFE17X_GAMMA_NODES; j++)
				if ((int)t[j + 1] - t[j] < -512 || (int)t[j + 1] - t[j] > 511)
					goto invalid;
		}
		if (memchr_inv(block.gamma->reserved, 0, sizeof(block.gamma->reserved)))
			goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_AWB_BG:
		if (!block.awb_bg->h_num || block.awb_bg->h_num > IFE17X_AWB_BG_MAX_H ||
		    !block.awb_bg->v_num || block.awb_bg->v_num > IFE17X_AWB_BG_MAX_V ||
		    block.awb_bg->reserved)
			goto invalid;
		for (i = 0; i < 4; i++)
			if (block.awb_bg->threshold[i] > 0x3fff)
				goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_PDPC:
		if (block.pdpc->black_level > 4095 || block.pdpc->reserved)
			goto invalid;
		for (i = 0; i < 2; i++)
			if (block.pdpc->bp_offset[i] > 0x7fff ||
			    block.pdpc->t2_bp_offset[i] > 0x7fff)
				goto invalid;
		for (i = 0; i < 4; i++)
			if (block.pdpc->wb_ratio[i] > 0x1ffff)
				goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_BF:
		return ife_bf_check(dev, block.bf);
	case IFE17X_PARAMS_BLOCK_ABF:
		return ife_abf_check(dev, block.abf);
	case IFE17X_PARAMS_BLOCK_LSC:
		return ife_lsc_check(dev, block.lsc);
	case IFE17X_PARAMS_BLOCK_DISPLAY_WINDOW:
		if (!ife_display_window_valid(meta, block.window))
			goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_OUTPUTS:
		if (block.outputs->outputs & ~(IFE17X_OUTPUT_FULL | IFE17X_OUTPUT_DISPLAY |
					       IFE17X_OUTPUT_RDI) ||
		    !(block.outputs->outputs & (IFE17X_OUTPUT_FULL | IFE17X_OUTPUT_DISPLAY)) ||
		    block.outputs->reserved)
			goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_BHIST:
		if (block.bhist->channel > IFE17X_BHIST_CHANNEL_B ||
		    block.bhist->width < 2 || block.bhist->height < 2 ||
		    (block.bhist->left | block.bhist->top | block.bhist->width |
		     block.bhist->height) & 1 ||
		    block.bhist->reserved0 || block.bhist->reserved)
			goto invalid;
		break;
	case IFE17X_PARAMS_BLOCK_BPC:
		for (i = 0; i < 4; i++)
			if (block.bpc->wb_ratio[i] > 0x1ffff)
				goto invalid;
		if (block.bpc->flags & ~(IFE17X_BPC_FL_HOT_OFF | IFE17X_BPC_FL_COLD_OFF |
					 IFE17X_BPC_FL_SAME_CHANNEL) ||
		    block.bpc->black_level > 4095 || block.bpc->bpc_offset > 0x7fff ||
		    block.bpc->bcc_offset > 0x7fff || block.bpc->threshold > 8191 ||
		    block.bpc->fmax < 64 || block.bpc->fmax > 127 || block.bpc->fmin > 64 ||
		    block.bpc->reserved)
			goto invalid;
		break;
	}
	return 0;

invalid:
	dev_dbg(dev, "invalid parameters block of type %u\n", block.header->type);
	return -EINVAL;
}

static int ife_params_buf_prepare(struct vb2_buffer *vb)
{
	struct vfe_pix_meta *meta = vb2_get_drv_priv(vb->vb2_queue);
	struct ife_params_buf *buf = to_params_buf(to_vb2_v4l2_buffer(vb));
	struct device *dev = meta->vfe->camss->dev;
	size_t off = 0;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(dev, vb, IFE_PARAMS_SIZE);
	if (ret)
		return ret;

	/* Work on a copy userspace can no longer change */
	memcpy(buf->config, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	ret = v4l2_isp_params_validate_buffer(dev, vb, buf->config,
					      ife_params_block_types,
					      ARRAY_SIZE(ife_params_block_types));
	if (ret)
		return ret;

	/* The values of the enabled blocks; a disabled one may be a bare header */
	buf->outputs = 0;
	buf->window_set = false;
	while (off < buf->config->data_size) {
		union ife_params_block block = { .data = &buf->config->data[off] };

		if (!ife_block_disabled(block)) {
			ret = ife_params_check(meta, dev, block);
			if (ret)
				return ret;
			if (block.header->type == IFE17X_PARAMS_BLOCK_OUTPUTS)
				buf->outputs = block.outputs->outputs;
			if (block.header->type == IFE17X_PARAMS_BLOCK_DISPLAY_WINDOW) {
				buf->window = *block.window;
				buf->window_set = true;
			}
		}
		off += block.header->size;
	}
	return 0;
}

static void ife_params_buf_queue(struct vb2_buffer *vb)
{
	struct vfe_pix_meta *meta = vb2_get_drv_priv(vb->vb2_queue);
	struct ife_params_buf *buf = to_params_buf(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&meta->lock, flags);
	/* Before the stream starts the frame size is unknown: keep the grid */
	if (!meta->running)
		ife_params_scan_awb(meta, buf->config);
	/*
	 * Its outputs decide the next frame it is first in the queue for. A
	 * frame not yet written does not join now even if its image buffers
	 * are all there: it waits for the next epoch 0.
	 */
	list_add_tail(&buf->queue, &meta->params_queue);
	spin_unlock_irqrestore(&meta->lock, flags);
}

static int ife_meta_start_streaming(struct vb2_queue *q, unsigned int count)
{
	return 0;
}

static void ife_params_stop_streaming(struct vb2_queue *q)
{
	struct vfe_pix_meta *meta = vb2_get_drv_priv(q);
	struct ife_params_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&meta->lock, flags);
	list_for_each_entry_safe(buf, tmp, &meta->params_queue, queue) {
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	meta->awb_bg_pending = false;
	spin_unlock_irqrestore(&meta->lock, flags);
}

static const struct vb2_ops ife_params_vb2_ops = {
	.queue_setup = ife_params_queue_setup,
	.buf_init = ife_params_buf_init,
	.buf_cleanup = ife_params_buf_cleanup,
	.buf_prepare = ife_params_buf_prepare,
	.buf_queue = ife_params_buf_queue,
	.start_streaming = ife_meta_start_streaming,
	.stop_streaming = ife_params_stop_streaming,
};

static int ife_stats_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				 unsigned int *num_planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	return ife_queue_setup(q, num_buffers, num_planes, sizes, alloc_devs,
			       IFE_STATS_SIZE);
}

static int ife_stats_buf_init(struct vb2_buffer *vb)
{
	struct ife_stats_buf *buf = to_stats_buf(to_vb2_v4l2_buffer(vb));
	struct sg_table *sgt = vb2_dma_sg_plane_desc(vb, 0);

	/* The camss device sits behind an IOMMU: one contiguous IOVA range */
	buf->addr = sg_dma_address(sgt->sgl);
	return 0;
}

static void ife_stats_buf_queue(struct vb2_buffer *vb)
{
	struct vfe_pix_meta *meta = vb2_get_drv_priv(vb->vb2_queue);
	struct ife_stats_buf *buf = to_stats_buf(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&meta->lock, flags);
	list_add_tail(&buf->queue, &meta->stats_queue);
	spin_unlock_irqrestore(&meta->lock, flags);
}

/* Whether the write masters are done with the driver's statistics buffers */
static bool ife_stats_idle(struct vfe_pix_meta *meta)
{
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&meta->lock, flags);
	idle = !meta->running || meta->wedged ||
	       (!meta->stats_cur && !meta->stats_next && !meta->nheld);
	spin_unlock_irqrestore(&meta->lock, flags);

	return idle;
}

/* Lock held: give every buffer the driver has back, failed */
static void ife_stats_fail_all(struct vfe_pix_meta *meta)
{
	struct ife_stats_buf *buf, *tmp;

	list_for_each_entry_safe(buf, tmp, &meta->stats_queue, queue) {
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	while (meta->nheld) {
		buf = meta->held[--meta->nheld].buf;
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	/* The frames stay settled: their addresses are queued */
	if (meta->stats_next) {
		vb2_buffer_done(&meta->stats_next->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		meta->stats_next = NULL;
	}
	if (meta->stats_cur) {
		vb2_buffer_done(&meta->stats_cur->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		meta->stats_cur = NULL;
	}
}

static void ife_stats_stop_streaming(struct vb2_queue *q)
{
	struct vfe_pix_meta *meta = vb2_get_drv_priv(q);
	struct device *dev = meta->vfe->camss->dev;
	struct ife_stats_buf *buf, *tmp;
	unsigned long flags;
	u64 period_ns;
	unsigned int ms;

	/* Buffers the hardware has no address of go back at once */
	spin_lock_irqsave(&meta->lock, flags);
	list_for_each_entry_safe(buf, tmp, &meta->stats_queue, queue) {
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	period_ns = meta->period_ns;
	spin_unlock_irqrestore(&meta->lock, flags);
	ms = clamp_t(u64, 3 * div_u64(period_ns, NSEC_PER_MSEC) + 100,
		     IFE_STATS_DRAIN_MIN_MS, IFE_STATS_DRAIN_MAX_MS);

	/*
	 * While the pixel path runs, the write masters have the addresses of
	 * up to two frames, and a dual IFE master's buffers may wait for the
	 * slave's grid. Let those frames complete (later ones go to the
	 * scratch buffer) before the memory goes back to userspace.
	 */
	if (!wait_event_timeout(meta->stats_wq, ife_stats_idle(meta), msecs_to_jiffies(ms)))
		dev_warn(dev, "VFE%d: statistics buffers returned before their frames ended\n",
			 meta->vfe->id);

	spin_lock_irqsave(&meta->lock, flags);
	ife_stats_fail_all(meta);
	spin_unlock_irqrestore(&meta->lock, flags);
}

static const struct vb2_ops ife_stats_vb2_ops = {
	.queue_setup = ife_stats_queue_setup,
	.buf_init = ife_stats_buf_init,
	.buf_queue = ife_stats_buf_queue,
	.start_streaming = ife_meta_start_streaming,
	.stop_streaming = ife_stats_stop_streaming,
};

static int ife_register_node(struct vfe_pix_meta *meta, struct v4l2_device *v4l2_dev,
			     struct video_device *vdev, struct vb2_queue *q,
			     struct mutex *lock, struct media_pad *pad, bool output)
{
	struct vfe_device *vfe = meta->vfe;
	int ret;

	mutex_init(lock);
	pad->flags = output ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, pad);
	if (ret)
		goto err_mutex;

	q->type = output ? V4L2_BUF_TYPE_META_OUTPUT : V4L2_BUF_TYPE_META_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = meta;
	q->mem_ops = output ? &vb2_vmalloc_memops : &vb2_dma_sg_memops;
	q->ops = output ? &ife_params_vb2_ops : &ife_stats_vb2_ops;
	q->buf_struct_size = output ? sizeof(struct ife_params_buf) :
				      sizeof(struct ife_stats_buf);
	/* Statistics take the time of their frame's start, like the images */
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
			     (output ? 0 : V4L2_BUF_FLAG_TSTAMP_SRC_SOE);
	q->lock = lock;
	q->dev = vfe->camss->dev;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_entity;

	snprintf(vdev->name, sizeof(vdev->name), "msm_vfe%d_pix_%s",
		 vfe->id, output ? "params" : "stats");
	vdev->release = video_device_release_empty;
	vdev->fops = &ife_meta_fops;
	vdev->ioctl_ops = output ? &ife_params_ioctl_ops : &ife_stats_ioctl_ops;
	vdev->lock = lock;
	vdev->v4l2_dev = v4l2_dev;
	vdev->queue = q;
	vdev->device_caps = (output ? V4L2_CAP_META_OUTPUT : V4L2_CAP_META_CAPTURE) |
			    V4L2_CAP_STREAMING | V4L2_CAP_IO_MC;
	vdev->vfl_dir = output ? VFL_DIR_TX : VFL_DIR_RX;
	video_set_drvdata(vdev, meta);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_queue;
	return 0;

err_queue:
	vb2_queue_release(q);
err_entity:
	media_entity_cleanup(&vdev->entity);
err_mutex:
	mutex_destroy(lock);
	return ret;
}

static void ife_unregister_node(struct video_device *vdev, struct mutex *lock)
{
	if (!video_is_registered(vdev))
		return;
	vb2_video_unregister_device(vdev);
	media_entity_cleanup(&vdev->entity);
	mutex_destroy(lock);
}

int vfe17x_meta_register(struct vfe_device *vfe, struct v4l2_device *v4l2_dev)
{
	struct device *dev = vfe->camss->dev;
	struct vfe_pix_meta *meta;
	int ret;

	meta = kzalloc(sizeof(*meta), GFP_KERNEL);
	if (!meta)
		return -ENOMEM;
	meta->vfe = vfe;
	spin_lock_init(&meta->lock);
	init_waitqueue_head(&meta->stats_wq);
	INIT_LIST_HEAD(&meta->params_queue);
	INIT_LIST_HEAD(&meta->stats_queue);

	meta->scratch = dma_alloc_coherent(dev, IFE_STATS_SIZE, &meta->scratch_addr,
					   GFP_KERNEL);
	if (!meta->scratch) {
		ret = -ENOMEM;
		goto err_free;
	}

	ret = ife_register_node(meta, v4l2_dev, &meta->params_vdev, &meta->params_q,
				&meta->params_lock, &meta->params_pad, true);
	if (ret)
		goto err_scratch;
	ret = ife_register_node(meta, v4l2_dev, &meta->stats_vdev, &meta->stats_q,
				&meta->stats_lock, &meta->stats_pad, false);
	if (ret)
		goto err_params;

	/* Frame sync events, before the subdev nodes are created */
	vfe->line[VFE_LINE_PIX].subdev.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;

	vfe->pix_meta = meta;
	return 0;

err_params:
	ife_unregister_node(&meta->params_vdev, &meta->params_lock);
err_scratch:
	dma_free_coherent(dev, IFE_STATS_SIZE, meta->scratch, meta->scratch_addr);
err_free:
	kfree(meta);
	return ret;
}

void vfe17x_meta_unregister(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	if (!meta)
		return;
	vfe->pix_meta = NULL;
	ife_unregister_node(&meta->stats_vdev, &meta->stats_lock);
	ife_unregister_node(&meta->params_vdev, &meta->params_lock);
	dma_free_coherent(vfe->camss->dev, IFE_STATS_SIZE, meta->scratch,
			  meta->scratch_addr);
	kfree(meta);
}
