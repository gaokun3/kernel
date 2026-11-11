/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-vfe-17x-meta.h
 *
 * Qualcomm MSM Camera Subsystem - VFE (Titan 17x) pixel pipeline parameters
 * and statistics video nodes.
 */
#ifndef QC_MSM_CAMSS_VFE_17X_META_H
#define QC_MSM_CAMSS_VFE_17X_META_H

#include <linux/types.h>

struct ife17x_params_display_window;
struct v4l2_device;
struct vfe_device;

int vfe17x_meta_register(struct vfe_device *vfe, struct v4l2_device *v4l2_dev);
void vfe17x_meta_unregister(struct vfe_device *vfe);
void vfe17x_meta_start(struct vfe_device *vfe, u32 width, u32 height, bool written);
void vfe17x_meta_stop(struct vfe_device *vfe);
void vfe17x_meta_fatal(struct vfe_device *vfe);
bool vfe17x_meta_frame_started(struct vfe_device *vfe);
bool vfe17x_meta_before_first_frame(struct vfe_device *vfe);
bool vfe17x_meta_epoch(struct vfe_device *vfe, bool next_written);
bool vfe17x_meta_running(struct vfe_device *vfe);
bool vfe17x_meta_running_locked(struct vfe_device *vfe);
u64 vfe17x_meta_period_ns(struct vfe_device *vfe);
void vfe17x_meta_lock(struct vfe_device *vfe);
void vfe17x_meta_unlock(struct vfe_device *vfe);
bool vfe17x_meta_join_open_locked(struct vfe_device *vfe);
bool vfe17x_meta_join_locked(struct vfe_device *vfe);
u32 vfe17x_meta_next_outputs_locked(struct vfe_device *vfe);
const struct ife17x_params_display_window *
vfe17x_meta_next_window_locked(struct vfe_device *vfe);
void vfe17x_meta_bus(struct vfe_device *vfe, u32 bus_status1);
void vfe17x_meta_epoch1(struct vfe_device *vfe, bool late);
void vfe17x_meta_sequence(struct vfe_device *vfe, u32 *sequence);

#endif /* QC_MSM_CAMSS_VFE_17X_META_H */
