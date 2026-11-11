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
#include "camss-vfe-17x-meta.h"

/* Module enables, double buffered */
#define VFE_MODULE_LENS_EN		0x040
#define		LENS_EN_BLS		BIT(11)
#define		LENS_EN_PDPC		BIT(13)
#define VFE_MODULE_STATS_EN		0x044
#define		STATS_EN_AWB_BG		BIT(3)
#define VFE_MODULE_COLOR_EN		0x048
#define		COLOR_EN_CC		BIT(1)
#define		COLOR_EN_GAMMA		BIT(3)
#define VFE_REG_UPDATE_CMD		0x4ac
#define		REG_UPDATE_PIX		BIT(0)
#define VFE_CAMIF_EPOCH_IRQ		0x4a0
/* Epoch 0 at line 20 like the Windows driver (19), epoch 1 off */
#define		CAMIF_EPOCH_CFG		(20 << 16 | 0x3fff)

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
#define		STATS_CFG_FIXED		BIT(6)
#define VFE_AWB_BG_OFFSET		0xbc8
#define VFE_AWB_BG_NUM			0xbcc
#define VFE_AWB_BG_SIZE			0xbd0
#define VFE_AWB_BG_HI_THR_0		0xbd4	/* R [13:0], Gr [29:16] */
#define VFE_AWB_BG_HI_THR_1		0xbd8	/* B [13:0], Gb [29:16] */
#define VFE_AWB_BG_LO_THR_0		0xbdc
#define VFE_AWB_BG_LO_THR_1		0xbe0
#define VFE_AWB_BG_LUMA_CFG		0xc14
#define VFE_AWB_BG_CFG			0xcb0

/* Bus write master of the AWB grid, frame based, PLAIN64 */
#define VFE_WM_AWB_BG			15
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

struct ife_params_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head queue;
	struct v4l2_isp_params_buffer *config;
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

	/* Everything below is protected by the lock */
	spinlock_t lock;
	struct list_head params_queue;
	struct list_head stats_queue;
	struct ife_stats_buf *stats_cur;
	bool running;
	/*
	 * The frame after the current one is written by the image path, so
	 * it belongs to a request: it takes the next parameters and
	 * statistics buffers. Decided at each epoch.
	 */
	bool next_written;
	/*
	 * From an epoch to the end of that frame, a buffer for the next frame
	 * may still join it, with its parameters (vfe17x_meta_join()).
	 */
	bool join_open;
	u32 width;
	u32 height;
	u32 frame;		/* epochs since the stream start */
	u8 gamma_bank;		/* LUT banks in use: 0 even, 1 odd */
	bool awb_bg_pending;	/* AWB block seen before the stream start */
	struct ife17x_params_awb_bg awb_bg;
	bool pdpc_mask_valid;	/* the PDPC mask bank holds pdpc_mask */
	u32 pdpc_mask[IFE17X_PDPC_MASK_ROWS];

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
	struct vfe_device *vfe = meta->vfe;
	const u16 *g = block.wb->gain;

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
	 * records follow the master's in the buffer.
	 */
	if (meta->dual_slave && hn > 1) {
		hn /= 2;
		meta->slave_stats_off = hn * vn * sizeof(struct ife17x_awb_bg_region);
	} else if (meta->dual_master && hn > 1) {
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
	vfe_rmw(vfe, VFE_MODULE_STATS_EN, 0, STATS_EN_AWB_BG);
}

static void ife_params_awb_bg(struct vfe_pix_meta *meta, union ife_params_block block)
{
	if (ife_block_disabled(block)) {
		vfe_rmw(meta->vfe, VFE_MODULE_STATS_EN, STATS_EN_AWB_BG, 0);
		return;
	}
	ife_awb_bg_program(meta, block.awb_bg);
}

static const ife_params_handler ife_params_handlers[] = {
	[IFE17X_PARAMS_BLOCK_BLS] = ife_params_bls,
	[IFE17X_PARAMS_BLOCK_WB] = ife_params_wb,
	[IFE17X_PARAMS_BLOCK_CCM] = ife_params_ccm,
	[IFE17X_PARAMS_BLOCK_GAMMA] = ife_params_gamma,
	[IFE17X_PARAMS_BLOCK_AWB_BG] = ife_params_awb_bg,
	[IFE17X_PARAMS_BLOCK_PDPC] = ife_params_pdpc,
};

static const struct v4l2_isp_params_block_type_info ife_params_block_types[] = {
	[IFE17X_PARAMS_BLOCK_BLS] = { .size = sizeof(struct ife17x_params_bls) },
	[IFE17X_PARAMS_BLOCK_WB] = { .size = sizeof(struct ife17x_params_wb) },
	[IFE17X_PARAMS_BLOCK_CCM] = { .size = sizeof(struct ife17x_params_ccm) },
	[IFE17X_PARAMS_BLOCK_GAMMA] = { .size = sizeof(struct ife17x_params_gamma) },
	[IFE17X_PARAMS_BLOCK_AWB_BG] = { .size = sizeof(struct ife17x_params_awb_bg) },
	[IFE17X_PARAMS_BLOCK_PDPC] = { .size = sizeof(struct ife17x_params_pdpc) },
};

static_assert(ARRAY_SIZE(ife_params_handlers) == ARRAY_SIZE(ife_params_block_types));

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

static void ife_stats_set_addr(struct vfe_pix_meta *meta, dma_addr_t addr)
{
	struct vfe_device *vfe = meta->vfe;

	writel_relaxed(lower_32_bits(addr), vfe->base + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_AWB_BG));
	writel_relaxed(IFE_STATS_SIZE, vfe->base + VFE_BUS_WM_FRAME_INC(VFE_WM_AWB_BG));

	/* Dual IFE: the slave's grid goes after the master's */
	if (meta->dual_slave) {
		void __iomem *s = meta->dual_slave->vfe->base;

		writel_relaxed(lower_32_bits(addr) + meta->slave_stats_off,
			       s + VFE_BUS_WM_IMAGE_ADDR(VFE_WM_AWB_BG));
		writel_relaxed(IFE_STATS_SIZE, s + VFE_BUS_WM_FRAME_INC(VFE_WM_AWB_BG));
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
}

/* Take the next queued stats buffer as the write target; lock held */
static void ife_stats_next(struct vfe_pix_meta *meta)
{
	struct ife_stats_buf *buf;

	buf = list_first_entry_or_null(&meta->stats_queue, struct ife_stats_buf, queue);
	if (buf)
		list_del(&buf->queue);
	meta->stats_cur = buf;
	ife_stats_set_addr(meta, buf ? buf->addr : meta->scratch_addr);
}

static void ife_stats_wm_init(struct vfe_pix_meta *meta)
{
	struct vfe_device *vfe = meta->vfe;
	u8 wm = VFE_WM_AWB_BG;

	writel_relaxed(0, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PERIOD(wm));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_IRQ_SUBSAMPLE_PATTERN(wm));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_FRAMEDROP_PERIOD(wm));
	writel_relaxed(1, vfe->base + VFE_BUS_WM_FRAMEDROP_PATTERN(wm));
	writel_relaxed(0xf, vfe->base + VFE_BUS_WM_BURST_LIMIT(wm));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_BUFFER_WIDTH_CFG(wm));
	writel_relaxed(0, vfe->base + VFE_BUS_WM_BUFFER_HEIGHT_CFG(wm));
	writel_relaxed(PACKER_FMT_PLAIN_64, vfe->base + VFE_BUS_WM_PACKER_CFG(wm));
	ife_stats_next(meta);
	writel_relaxed(WM_CFG_FRAME_BASED, vfe->base + VFE_BUS_WM_CFG(wm));
}

/* -----------------------------------------------------------------------------
 * Hooks from camss-vfe-17x.c
 */

/*
 * Stream start, from vfe_pix_configure() with the output lock held and
 * interrupts off, before the first register update.
 */
void vfe17x_meta_start(struct vfe_device *vfe, u32 width, u32 height)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	if (!meta)
		return;

	spin_lock(&meta->lock);
	meta->width = width;
	meta->height = height;
	meta->frame = 0;
	meta->next_written = true;
	meta->join_open = false;
	meta->stripe_x = 0;
	meta->stripe_w = width;
	meta->frame_x = 0;
	meta->slave_stats_off = 0;
	meta->slave_stats_ts = 0;
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
	/* The VFE was reset: the mask bank must be written again */
	meta->pdpc_mask_valid = false;
	writel_relaxed(CAMIF_EPOCH_CFG, vfe->base + VFE_CAMIF_EPOCH_IRQ);

	if (meta->awb_bg_pending) {
		ife_awb_bg_program(meta, &meta->awb_bg);
		if (meta->dual_slave)
			ife_awb_bg_program(meta->dual_slave, &meta->awb_bg);
	}
	ife_params_apply_next(meta, 0);
	ife_stats_wm_init(meta);
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
	/* The master may still write the current buffer: give it back unused */
	if (meta->stats_cur) {
		list_add(&meta->stats_cur->queue, &meta->stats_queue);
		meta->stats_cur = NULL;
	}
	ife_dual_end(meta);
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

	if (!meta)
		return;

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
	bool running;

	if (!meta)
		return false;

	spin_lock(&meta->lock);
	running = meta->running;
	spin_unlock(&meta->lock);

	return running;
}

/*
 * A buffer joining the next frame after its epoch (vfe17x_queue_buffer())
 * decides under the meta lock, which the end of the frame takes to settle
 * the statistics: the two do not interleave. Output lock held, interrupts
 * off.
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

/* From an epoch until that frame ends, the next frame can still be joined */
bool vfe17x_meta_join_open_locked(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	return meta && meta->running && meta->join_open;
}

/*
 * The image path got a buffer for the next frame after its epoch, before the
 * end of the current frame: the next frame belongs to a request after all.
 * Apply its parameters now, before the next frame starts; its statistics
 * buffer is picked at the end of this frame. Returns true if registers need
 * a PIX register update.
 */
bool vfe17x_meta_join_locked(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	if (!vfe17x_meta_join_open_locked(vfe) || meta->next_written)
		return false;

	meta->next_written = true;
	return ife_params_apply_next(meta, meta->frame);
}

/*
 * @next_written: the image path has a buffer for the next frame. A frame it
 * skips, for want of a buffer, belongs to no request: it takes neither the
 * next parameters nor a statistics buffer, which stay with the frame of
 * their request.
 */
bool vfe17x_meta_epoch(struct vfe_device *vfe, bool next_written)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	bool applied = false;

	if (!meta)
		return false;

	struct v4l2_event ev = { .type = V4L2_EVENT_FRAME_SYNC };
	bool running;

	spin_lock(&meta->lock);
	running = meta->running;
	if (running) {
		ev.u.frame_sync.frame_sequence = meta->frame;
		meta->frame++;
		meta->next_written = next_written;
		meta->join_open = true;
		if (next_written)
			applied = ife_params_apply_next(meta, meta->frame);
	}
	spin_unlock(&meta->lock);

	/* Frame start for the sensor control timing in userspace */
	if (running)
		v4l2_event_queue(vfe->line[VFE_LINE_PIX].subdev.devnode, &ev);

	return applied;
}

/*
 * Number a completed image of the pixel path by the epochs, like the stats, so
 * images, statistics and parameters keep the same frame numbers when the
 * driver runs out of image buffers.
 */
/*
 * Whether the first frame of the stream started (its epoch came). A buffer
 * that completes before belongs to a frame that began before the stream:
 * not one of its frames.
 */
bool vfe17x_meta_frame_started(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	bool started;

	if (!meta)
		return true;

	spin_lock(&meta->lock);
	started = meta->running && meta->frame;
	spin_unlock(&meta->lock);

	return started;
}

/* The pixel path runs and its first frame did not start yet */
bool vfe17x_meta_before_first_frame(struct vfe_device *vfe)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	bool before;

	if (!meta)
		return false;

	spin_lock(&meta->lock);
	before = meta->running && !meta->frame;
	spin_unlock(&meta->lock);

	return before;
}

void vfe17x_meta_sequence(struct vfe_device *vfe, u32 *sequence)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;

	if (!meta)
		return;

	spin_lock(&meta->lock);
	if (meta->running && meta->frame)
		*sequence = meta->frame - 1;
	spin_unlock(&meta->lock);
}

/* Bus interrupt: complete the stats buffer of a finished frame */
void vfe17x_meta_bus(struct vfe_device *vfe, u32 bus_status1)
{
	struct vfe_pix_meta *meta = vfe->pix_meta;
	struct vfe_pix_meta *master;
	struct ife_stats_buf *done;

	if (!meta || !(bus_status1 & BIT(VFE_WM_AWB_BG)))
		return;

	/* Dual IFE slave: its grid went into the master's buffer */
	master = READ_ONCE(meta->dual_master);
	if (master) {
		spin_lock(&master->lock);
		if (master->dual_slave == meta) {
			u64 ts = ktime_get_ns();

			master->slave_stats_ts = ts;
			ife_stats_dual_collect(master, ts);
		}
		spin_unlock(&master->lock);
		return;
	}

	spin_lock(&meta->lock);
	if (!meta->running) {
		spin_unlock(&meta->lock);
		return;
	}
	done = meta->stats_cur;
	/* The frame ends: whether the next one belongs to a request is settled */
	meta->join_open = false;
	/*
	 * Before the first epoch: the grid of a frame that began before the
	 * stream, not one of its frames. Have the buffer written again, by
	 * the first frame, rather than number this grid as frame 0.
	 */
	if (done && !meta->frame) {
		ife_stats_set_addr(meta, done->addr);
		spin_unlock(&meta->lock);
		dev_dbg(vfe->camss->dev,
			"VFE%d: statistics before the first frame, buffer reused\n", vfe->id);
		return;
	}
	/* A frame the image path skips writes its grid to the scratch buffer */
	if (meta->next_written) {
		ife_stats_next(meta);
	} else {
		meta->stats_cur = NULL;
		ife_stats_set_addr(meta, meta->scratch_addr);
	}
	/* Dual IFE: wait for the slave's grid of the same frame */
	if (done && meta->dual_slave &&
	    meta->slave_stats_ts + VFE_DUAL_MATCH_NS <= ktime_get_ns()) {
		if (meta->nheld == ARRAY_SIZE(meta->held)) {
			struct ife_stats_buf *old = meta->held[0].buf;

			meta->nheld--;
			memmove(&meta->held[0], &meta->held[1],
				meta->nheld * sizeof(meta->held[0]));
			vb2_buffer_done(&old->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
		done->vb.sequence = meta->frame ? meta->frame - 1 : 0;
		done->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_set_plane_payload(&done->vb.vb2_buf, 0, IFE_STATS_SIZE);
		meta->held[meta->nheld].buf = done;
		meta->held[meta->nheld].ts = done->vb.vb2_buf.timestamp;
		meta->nheld++;
		done = NULL;
	}
	if (done) {
		/*
		 * The grid of frame k completes at its end, after the epoch of
		 * frame k and before that of frame k + 1: number it from the
		 * epochs, so it matches the image even if the grid was turned
		 * on after the stream start.
		 */
		done->vb.sequence = meta->frame ? meta->frame - 1 : 0;
		done->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_set_plane_payload(&done->vb.vb2_buf, 0, IFE_STATS_SIZE);
		vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);
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

static int ife_params_buf_prepare(struct vb2_buffer *vb)
{
	struct vfe_pix_meta *meta = vb2_get_drv_priv(vb->vb2_queue);
	struct ife_params_buf *buf = to_params_buf(to_vb2_v4l2_buffer(vb));
	struct device *dev = meta->vfe->camss->dev;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(dev, vb, IFE_PARAMS_SIZE);
	if (ret)
		return ret;

	/* Work on a copy userspace can no longer change */
	memcpy(buf->config, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	return v4l2_isp_params_validate_buffer(dev, vb, buf->config,
					       ife_params_block_types,
					       ARRAY_SIZE(ife_params_block_types));
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

static void ife_stats_stop_streaming(struct vb2_queue *q)
{
	struct vfe_pix_meta *meta = vb2_get_drv_priv(q);
	struct ife_stats_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&meta->lock, flags);
	if (meta->stats_cur) {
		/* Point the master at the scratch buffer before giving it back */
		if (meta->running)
			ife_stats_set_addr(meta, meta->scratch_addr);
		list_add(&meta->stats_cur->queue, &meta->stats_queue);
		meta->stats_cur = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &meta->stats_queue, queue) {
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
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
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
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
