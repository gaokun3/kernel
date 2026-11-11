/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-vfe-17x-disp.h
 *
 * Qualcomm MSM Camera Subsystem - VFE (Titan 17x) display output of the pixel
 * pipeline: a second, scaled output beside FULL.
 */
#ifndef QC_MSM_CAMSS_VFE_17X_DISP_H
#define QC_MSM_CAMSS_VFE_17X_DISP_H

#include <linux/wait.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>

#include "camss-vfe.h"
#include "camss-video.h"

/*
 * The display output path of a full VFE: it takes the pixel path's frame after
 * the colour space transform, at the CAMIF size, and scales it with its own
 * MNDS, crop and round/clamp into write masters 20 (Y) and 21 (CbCr), in the
 * NV12 or NV21 order of the pixel path. It writes only while the pixel path
 * runs; the pixel path's own (FULL) output decides which frames are written.
 *
 * Sink pad: the pixel path frame (CAMIF size, YUV code of the pixel path
 * source pad). Compose on the sink pad: the scaler output size. Crop on the
 * source pad: the part of the scaled frame written, which lets it match the
 * field of view of the FULL output after the FULL crop. Source pad: the crop
 * size.
 *
 * With dual IFE, the master's display output stands for the whole frame: its
 * sink is the frame (the master's output width plus the slave's), compose and
 * crop are the frame's, and the driver splits them between the two display
 * paths (struct vfe_disp_stripe). The slave's own display output stays off.
 */

/* Downscale limit per axis; the MNDS16 was measured up to about 13 */
#define VFE_DISP_RATIO_MAX		16
/* The smallest output, as for the FULL path */
#define VFE_DISP_MIN_WIDTH		16
#define VFE_DISP_MIN_HEIGHT		4

/* The VFEs that write a display frame with dual IFE */
#define VFE_DISP_MASTER			BIT(0)
#define VFE_DISP_SLAVE			BIT(1)

/*
 * The part one VFE's display path takes of the display output: its scaler
 * input and output columns, the crop of its scaler output, and the byte
 * offset of its columns in the buffer.
 */
struct vfe_disp_stripe {
	u32 in_w;
	u32 out_w;
	struct v4l2_rect crop;
	u32 offset;
};

struct vfe_pix_disp {
	struct vfe_device *vfe;
	struct v4l2_subdev subdev;
	struct media_pad pads[MSM_VFE_PADS_NUM];
	struct v4l2_mbus_framefmt fmt[MSM_VFE_PADS_NUM];
	struct v4l2_rect compose;
	struct v4l2_rect crop;
	struct camss_video video;
	/* Buffers and state, protected by the VFE's output_lock */
	struct vfe_output output;
	/*
	 * Its link was on when the pixel path started, which was not a dual
	 * IFE pair: without a list of outputs from the parameters, every
	 * frame the pixel path writes needs a display buffer too, and no
	 * frame is written while the display output is off; a list may name
	 * it. Links do not change while the pixel path streams.
	 */
	bool required;
	/* Started, the setup not latched yet: no buffer for the next frame */
	bool starting;
	/*
	 * The window of the frame last settled, as compose and crop
	 * rectangles of the whole frame: the subdev's from the start on, and
	 * then those the parameters set (struct ife17x_params_display_window).
	 * @stripe: the master's part of it, and with dual IFE the slave's;
	 * @writers: the VFEs that write it (VFE_DISP_MASTER, VFE_DISP_SLAVE),
	 * one only when the seam falls outside the crop. @slave: the slave
	 * VFE, until its pixel path stops. @pair: buffers the master wrote,
	 * waiting for the slave's stripe (only its slave_ts, slave_error,
	 * nheld and held are used). @buf_writers: the writers of the buffers
	 * out, as output.buf.
	 */
	struct v4l2_rect win_compose;
	struct v4l2_rect win_crop;
	struct vfe_disp_stripe stripe[2];
	u8 writers;
	bool dual;
	/*
	 * Dual IFE, set at the start: the stripes' input columns (master,
	 * slave), the frame column where the master's output ends, and the
	 * column of the slave's stripe where its output begins.
	 */
	struct {
		u32 wm;
		u32 ws;
		u32 split;
		u32 pad;
	} geo;
	struct vfe_device *slave;
	struct vfe_pix_dual pair;
	u8 buf_writers[2];
	/* A stop waits for the frames given buffers to be written */
	wait_queue_head_t drain_wq;
};

bool vfe17x_disp_window_valid(const struct v4l2_mbus_framefmt *sink,
			      const struct v4l2_rect *compose, const struct v4l2_rect *crop);
int vfe17x_disp_register(struct vfe_device *vfe, struct v4l2_device *v4l2_dev);
void vfe17x_disp_unregister(struct vfe_device *vfe);

/* camss-vfe-17x.c */
extern const struct camss_video_ops vfe17x_disp_video_ops;
int vfe17x_disp_start(struct vfe_pix_disp *disp);
void vfe17x_disp_stop(struct vfe_pix_disp *disp);

#endif /* QC_MSM_CAMSS_VFE_17X_DISP_H */
