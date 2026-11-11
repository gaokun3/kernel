// SPDX-License-Identifier: GPL-2.0
/*
 * camss-vfe-17x-disp.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Titan 17x) display output of the pixel
 * pipeline: its subdev and video node. The hardware side and the buffers are
 * in camss-vfe-17x.c.
 *
 * Media graph, per full VFE N:
 *
 *   msm_vfeN_pix:1 -> msm_vfeN_video3         (immutable, FULL output)
 *   msm_vfeN_pix:1 -> msm_vfeN_pix_disp:0     (off until enabled)
 *   msm_vfeN_pix_disp:1 -> msm_vfeN_video_disp (immutable)
 *
 * As on the pixel line, the sink compose rectangle is the scaler output and the
 * source crop rectangle the part of it written: the crop matches the field of
 * view of the FULL output, which crops after its own scaler.
 *
 * The FULL node starts the pixel path; the display node then starts only its
 * output (video_start_streaming() stops at the streaming pixel line). With the
 * link on at that start, every frame written needs a FULL and a display buffer
 * both, so no frame is written until the display node streams too, and the
 * two buffers of a request take the same frame; the parameters of a frame may
 * name its outputs instead (struct ife17x_params_outputs). Stopping the FULL
 * node stops the pixel path and with it the display output.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-17x-disp.h"

#define VFE_DISP_MAX_SIZE		8190

static struct v4l2_mbus_framefmt *
vfe_disp_format(struct vfe_pix_disp *disp, struct v4l2_subdev_state *state,
		unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_format(state, pad);

	return &disp->fmt[pad];
}

static struct v4l2_rect *
vfe_disp_compose(struct vfe_pix_disp *disp, struct v4l2_subdev_state *state,
		 enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_compose(state, MSM_VFE_PAD_SINK);

	return &disp->compose;
}

static struct v4l2_rect *
vfe_disp_crop(struct vfe_pix_disp *disp, struct v4l2_subdev_state *state,
	      enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_state_get_crop(state, MSM_VFE_PAD_SRC);

	return &disp->crop;
}

static bool vfe_disp_code_valid(struct vfe_pix_disp *disp, u32 code)
{
	const struct camss_formats *out = disp->vfe->res->formats_pix_out;
	unsigned int i;

	for (i = 0; i < out->nformats; i++)
		if (out->formats[i].code == code)
			return true;

	return false;
}

/*
 * Sink: the pixel path frame after CST, at the CAMIF size, in the YUV order of
 * the pixel path source pad (link validation checks both).
 */
static void vfe_disp_try_sink(struct vfe_pix_disp *disp, struct v4l2_mbus_framefmt *fmt)
{
	if (!vfe_disp_code_valid(disp, fmt->code))
		fmt->code = disp->vfe->res->formats_pix_out->formats[0].code;

	fmt->width = clamp_t(u32, fmt->width, VFE_DISP_MIN_WIDTH, VFE_DISP_MAX_SIZE) & ~1;
	fmt->height = clamp_t(u32, fmt->height, VFE_DISP_MIN_HEIGHT, VFE_DISP_MAX_SIZE) & ~1;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

/*
 * Compose on the sink: the scaled size. Even in both axes (NV12 chroma), no
 * larger than the sink and at most VFE_DISP_RATIO_MAX times smaller. The
 * aspect ratio is free.
 */
static void vfe_disp_try_compose(const struct v4l2_mbus_framefmt *sink, struct v4l2_rect *r)
{
	u32 min_w = ALIGN(max_t(u32, DIV_ROUND_UP(sink->width, VFE_DISP_RATIO_MAX),
				VFE_DISP_MIN_WIDTH), 2);
	u32 min_h = ALIGN(max_t(u32, DIV_ROUND_UP(sink->height, VFE_DISP_RATIO_MAX),
				VFE_DISP_MIN_HEIGHT), 2);

	r->left = 0;
	r->top = 0;
	r->width = clamp_t(u32, r->width & ~1, min_w, sink->width);
	r->height = clamp_t(u32, r->height & ~1, min_h, sink->height);
}

/*
 * Crop on the source: the part of the scaled frame the write masters take,
 * inside the compose rectangle. Even position and size (NV12 chroma), at
 * least VFE_DISP_MIN_WIDTH x VFE_DISP_MIN_HEIGHT.
 */
static void vfe_disp_try_crop(const struct v4l2_rect *compose, struct v4l2_rect *r)
{
	r->width = clamp_t(u32, r->width & ~1, VFE_DISP_MIN_WIDTH, compose->width);
	r->height = clamp_t(u32, r->height & ~1, VFE_DISP_MIN_HEIGHT, compose->height);
	r->left = clamp_t(s32, r->left & ~1, 0, compose->width - r->width);
	r->top = clamp_t(s32, r->top & ~1, 0, compose->height - r->height);
}

/*
 * Whether the scaler output @compose and the crop @crop of it are a window the
 * display output can take, with the sink @sink: the compose rectangle within
 * the limits of vfe_disp_try_compose(), the crop inside it, all even.
 */
bool vfe17x_disp_window_valid(const struct v4l2_mbus_framefmt *sink,
			      const struct v4l2_rect *compose, const struct v4l2_rect *crop)
{
	struct v4l2_rect r = *compose;

	vfe_disp_try_compose(sink, &r);
	if (r.width != compose->width || r.height != compose->height)
		return false;

	return crop->left >= 0 && crop->top >= 0 && !(crop->left & 1) && !(crop->top & 1) &&
	       !(crop->width & 1) && !(crop->height & 1) &&
	       crop->left + crop->width <= compose->width &&
	       crop->top + crop->height <= compose->height;
}

/* Source: the sink format at the crop size */
static void vfe_disp_update_source(struct vfe_pix_disp *disp, struct v4l2_subdev_state *state,
				   enum v4l2_subdev_format_whence which)
{
	struct v4l2_mbus_framefmt *sink = vfe_disp_format(disp, state, MSM_VFE_PAD_SINK, which);
	struct v4l2_mbus_framefmt *src = vfe_disp_format(disp, state, MSM_VFE_PAD_SRC, which);
	struct v4l2_rect *crop = vfe_disp_crop(disp, state, which);

	*src = *sink;
	src->width = crop->width;
	src->height = crop->height;
}

/* A new compose rectangle: the crop takes all of it again */
static void vfe_disp_reset_crop(struct vfe_pix_disp *disp, struct v4l2_subdev_state *state,
				enum v4l2_subdev_format_whence which)
{
	*vfe_disp_crop(disp, state, which) = *vfe_disp_compose(disp, state, which);
	vfe_disp_update_source(disp, state, which);
}

static int vfe_disp_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);
	const struct camss_formats *out = disp->vfe->res->formats_pix_out;

	if (code->pad == MSM_VFE_PAD_SINK) {
		if (code->index >= out->nformats)
			return -EINVAL;
		code->code = out->formats[code->index].code;
		return 0;
	}

	/* The source keeps the sink code: the CST orders the chroma for both paths */
	if (code->index)
		return -EINVAL;
	code->code = vfe_disp_format(disp, state, MSM_VFE_PAD_SINK, code->which)->code;

	return 0;
}

static int vfe_disp_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink;

	if (fse->index)
		return -EINVAL;

	if (fse->pad == MSM_VFE_PAD_SINK) {
		if (!vfe_disp_code_valid(disp, fse->code))
			return -EINVAL;
		fse->min_width = VFE_DISP_MIN_WIDTH;
		fse->min_height = VFE_DISP_MIN_HEIGHT;
		fse->max_width = VFE_DISP_MAX_SIZE;
		fse->max_height = VFE_DISP_MAX_SIZE;
		return 0;
	}

	/* From the smallest crop up to the sink, neither scaled nor cropped */
	sink = vfe_disp_format(disp, state, MSM_VFE_PAD_SINK, fse->which);
	if (fse->code != sink->code)
		return -EINVAL;
	fse->min_width = VFE_DISP_MIN_WIDTH;
	fse->min_height = VFE_DISP_MIN_HEIGHT;
	fse->max_width = sink->width;
	fse->max_height = sink->height;

	return 0;
}

static int vfe_disp_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *fmt)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);

	fmt->format = *vfe_disp_format(disp, state, fmt->pad, fmt->which);

	return 0;
}

static int vfe_disp_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *fmt)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);
	struct v4l2_rect *compose;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && sd->s_stream_enabled)
		return -EBUSY;

	/* The source follows the sink and the crop rectangle */
	if (fmt->pad == MSM_VFE_PAD_SRC) {
		fmt->format = *vfe_disp_format(disp, state, MSM_VFE_PAD_SRC, fmt->which);
		return 0;
	}

	vfe_disp_try_sink(disp, &fmt->format);
	*vfe_disp_format(disp, state, MSM_VFE_PAD_SINK, fmt->which) = fmt->format;

	/* No scaling and no crop until rectangles are set */
	compose = vfe_disp_compose(disp, state, fmt->which);
	compose->width = fmt->format.width;
	compose->height = fmt->format.height;
	vfe_disp_try_compose(&fmt->format, compose);
	vfe_disp_reset_crop(disp, state, fmt->which);

	return 0;
}

static int vfe_disp_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  struct v4l2_subdev_selection *sel)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *sink;

	if (sel->pad == MSM_VFE_PAD_SRC) {
		switch (sel->target) {
		case V4L2_SEL_TGT_CROP:
			sel->r = *vfe_disp_crop(disp, state, sel->which);
			return 0;
		case V4L2_SEL_TGT_CROP_BOUNDS:
		case V4L2_SEL_TGT_CROP_DEFAULT:
			sel->r = *vfe_disp_compose(disp, state, sel->which);
			return 0;
		default:
			return -EINVAL;
		}
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE:
		sel->r = *vfe_disp_compose(disp, state, sel->which);
		return 0;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		sink = vfe_disp_format(disp, state, MSM_VFE_PAD_SINK, sel->which);
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = sink->width;
		sel->r.height = sink->height;
		return 0;
	default:
		return -EINVAL;
	}
}

static int vfe_disp_set_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  struct v4l2_subdev_selection *sel)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE && sd->s_stream_enabled)
		return -EBUSY;

	if (sel->pad == MSM_VFE_PAD_SINK && sel->target == V4L2_SEL_TGT_COMPOSE) {
		vfe_disp_try_compose(vfe_disp_format(disp, state, MSM_VFE_PAD_SINK, sel->which),
				     &sel->r);
		*vfe_disp_compose(disp, state, sel->which) = sel->r;
		vfe_disp_reset_crop(disp, state, sel->which);
		return 0;
	}

	if (sel->pad == MSM_VFE_PAD_SRC && sel->target == V4L2_SEL_TGT_CROP) {
		vfe_disp_try_crop(vfe_disp_compose(disp, state, sel->which), &sel->r);
		*vfe_disp_crop(disp, state, sel->which) = sel->r;
		vfe_disp_update_source(disp, state, sel->which);
		return 0;
	}

	return -EINVAL;
}

/*
 * The display path takes the frame after CST, before the FULL scaler and crop:
 * its sink has the size of the pixel line sink (the CAMIF frame) and the code
 * of the pixel line source. With dual IFE it stands for the whole frame,
 * wider than the master's stripe; the pixel paths pair only at their start,
 * so the exact width is checked then (vfe17x_disp_start()).
 */
static int vfe_disp_link_validate(struct v4l2_subdev *sd, struct media_link *link,
				  struct v4l2_subdev_format *source_fmt,
				  struct v4l2_subdev_format *sink_fmt)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);
	struct vfe_line *pix = &disp->vfe->line[VFE_LINE_PIX];
	const struct v4l2_mbus_framefmt *in = &pix->fmt[MSM_VFE_PAD_SINK];

	if (link->source->entity != &pix->subdev.entity)
		return -EPIPE;

	if (sink_fmt->format.code != source_fmt->format.code ||
	    sink_fmt->format.width < in->width || sink_fmt->format.height != in->height) {
		dev_dbg(disp->vfe->camss->dev,
			"%s: sink %ux%u 0x%04x, the pixel path gives %ux%u 0x%04x\n",
			sd->name, sink_fmt->format.width, sink_fmt->format.height,
			sink_fmt->format.code, in->width, in->height, source_fmt->format.code);
		return -EPIPE;
	}

	return 0;
}

static int vfe_disp_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct vfe_pix_disp *disp = v4l2_get_subdevdata(sd);

	if (enable) {
		struct media_pad *remote;

		/* Its node can start without the link, when that is off */
		remote = media_pad_remote_pad_first(&disp->pads[MSM_VFE_PAD_SINK]);
		if (!remote || remote->entity != &disp->vfe->line[VFE_LINE_PIX].subdev.entity)
			return -ENOLINK;

		return vfe17x_disp_start(disp);
	}

	vfe17x_disp_stop(disp);

	return 0;
}

static int vfe_disp_init_formats(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_subdev_format format = {
		.pad = MSM_VFE_PAD_SINK,
		.which = fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE,
		.format = {
			.code = MEDIA_BUS_FMT_YUYV8_1_5X8,
			.width = 1920,
			.height = 1080,
		},
	};

	return vfe_disp_set_fmt(sd, fh ? fh->state : NULL, &format);
}

static const struct v4l2_subdev_video_ops vfe_disp_video_ops = {
	.s_stream = vfe_disp_s_stream,
};

static const struct v4l2_subdev_pad_ops vfe_disp_pad_ops = {
	.enum_mbus_code = vfe_disp_enum_mbus_code,
	.enum_frame_size = vfe_disp_enum_frame_size,
	.get_fmt = vfe_disp_get_fmt,
	.set_fmt = vfe_disp_set_fmt,
	.get_selection = vfe_disp_get_selection,
	.set_selection = vfe_disp_set_selection,
	.link_validate = vfe_disp_link_validate,
};

static const struct v4l2_subdev_ops vfe_disp_ops = {
	.video = &vfe_disp_video_ops,
	.pad = &vfe_disp_pad_ops,
};

static const struct v4l2_subdev_internal_ops vfe_disp_internal_ops = {
	.open = vfe_disp_init_formats,
};

static const struct media_entity_operations vfe_disp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

int vfe17x_disp_register(struct vfe_device *vfe, struct v4l2_device *v4l2_dev)
{
	struct device *dev = vfe->camss->dev;
	struct vfe_line *pix = &vfe->line[VFE_LINE_PIX];
	struct vfe_pix_disp *disp;
	struct camss_video *video;
	struct v4l2_subdev *sd;
	char name[32];
	int ret;

	/* Embeds a video node: lives as long as those of the lines */
	disp = devm_kzalloc(dev, sizeof(*disp), GFP_KERNEL);
	if (!disp)
		return -ENOMEM;
	disp->vfe = vfe;
	disp->output.state = VFE_OUTPUT_OFF;
	INIT_LIST_HEAD(&disp->output.pending_bufs);
	init_waitqueue_head(&disp->drain_wq);

	sd = &disp->subdev;
	v4l2_subdev_init(sd, &vfe_disp_ops);
	sd->internal_ops = &vfe_disp_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "msm_vfe%d_pix_disp", vfe->id);
	v4l2_set_subdevdata(sd, disp);

	ret = vfe_disp_init_formats(sd, NULL);
	if (ret < 0)
		goto err_free;

	disp->pads[MSM_VFE_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	disp->pads[MSM_VFE_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	sd->entity.ops = &vfe_disp_media_ops;
	ret = media_entity_pads_init(&sd->entity, MSM_VFE_PADS_NUM, disp->pads);
	if (ret < 0)
		goto err_free;

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0)
		goto err_entity;

	video = &disp->video;
	video->camss = vfe->camss;
	video->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	video->ops = &vfe17x_disp_video_ops;
	video->bpl_alignment = pix->video_out.bpl_alignment;
	video->line_based = 1;
	video->nformats = vfe->res->formats_pix_out->nformats;
	video->formats = vfe->res->formats_pix_out->formats;
	snprintf(name, sizeof(name), "msm_vfe%d_video_disp", vfe->id);
	ret = msm_video_register(video, v4l2_dev, name);
	if (ret < 0)
		goto err_subdev;
	/* Its buffers take the time of the frame start, like the pixel path's */
	video->vb2_q.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
				       V4L2_BUF_FLAG_TSTAMP_SRC_SOE;

	ret = media_create_pad_link(&sd->entity, MSM_VFE_PAD_SRC, &video->vdev.entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0)
		goto err_video;

	ret = media_create_pad_link(&pix->subdev.entity, MSM_VFE_PAD_SRC,
				    &sd->entity, MSM_VFE_PAD_SINK, 0);
	if (ret < 0)
		goto err_video;

	vfe->pix_disp = disp;

	return 0;

err_video:
	msm_video_unregister(video);
err_subdev:
	v4l2_device_unregister_subdev(sd);
err_entity:
	media_entity_cleanup(&sd->entity);
err_free:
	devm_kfree(dev, disp);
	return ret;
}

void vfe17x_disp_unregister(struct vfe_device *vfe)
{
	struct vfe_pix_disp *disp = vfe->pix_disp;

	if (!disp)
		return;

	vfe->pix_disp = NULL;
	msm_video_unregister(&disp->video);
	v4l2_device_unregister_subdev(&disp->subdev);
	media_entity_cleanup(&disp->subdev.entity);
}
