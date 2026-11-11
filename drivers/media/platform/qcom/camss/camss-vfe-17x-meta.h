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

struct v4l2_device;
struct vfe_device;

int vfe17x_meta_register(struct vfe_device *vfe, struct v4l2_device *v4l2_dev);
void vfe17x_meta_unregister(struct vfe_device *vfe);
void vfe17x_meta_start(struct vfe_device *vfe, u32 width, u32 height);
void vfe17x_meta_stop(struct vfe_device *vfe);
void vfe17x_meta_fatal(struct vfe_device *vfe);
bool vfe17x_meta_epoch(struct vfe_device *vfe);
void vfe17x_meta_bus(struct vfe_device *vfe, u32 bus_status1);
void vfe17x_meta_sequence(struct vfe_device *vfe, u32 *sequence);

#endif /* QC_MSM_CAMSS_VFE_17X_META_H */
