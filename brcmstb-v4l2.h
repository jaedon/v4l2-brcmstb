// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020,2021 Jaedon Shin <jaedon.shin@gmail.com>
 */

#ifndef __BRCMSTB_V4L2_H__
#define __BRCMSTB_V4L2_H__

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-memops.h>

struct v4l2_brcmstb_format {
	u32 pixfmt;
	u32 min_buffers;
	u32 max_buffers;
	u32 max_width;
	u32 max_height;
	u32 flags;

	u32 pixfmts_cap[4];
};

struct v4l2_brcmstb_ops;
struct v4l2_brcmstb_context;

struct v4l2_brcmstb_device {
	struct v4l2_device v4l2_dev;
	struct video_device *vdev_dec;
	const struct vb2_mem_ops *memops;
	struct device *dev;

	const struct v4l2_brcmstb_ops *ops;
	const struct v4l2_brcmstb_format *formats;
	u32 num_formats;

	struct v4l2_brcmstb_context *curr_context;
	struct mutex lock;

	void *priv;
};

struct v4l2_brcmstb_ops {
	void (*drain)(struct v4l2_brcmstb_context *);
	int (*streamon)(struct v4l2_brcmstb_context *);
	int (*streamoff)(struct v4l2_brcmstb_context *);
	u32 (*queuenum)(struct v4l2_brcmstb_context *);
	int (*queue)(struct v4l2_brcmstb_context *, const u8 *, u32, u64);
	int (*dequeuenum)(struct v4l2_brcmstb_context *);
	int (*dequeue)(struct v4l2_brcmstb_context *, dma_addr_t *, u64 *,
		       u32 *, u32 *, struct v4l2_timecode *);
};

enum v4l2_brcmstb_status {
	STATUS_STOPPED,
	STATUS_INIT,
	STATUS_RUNNING,
	STATUS_NEEDS_RESUME,
};

struct v4l2_brcmstb_context {
	struct v4l2_fh fh;
	struct v4l2_m2m_dev *m2m_dev;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *ctrl_min_buf_capture;
	struct mutex lock;

	struct v4l2_brcmstb_device *device;

	u32 pixfmt_cap;
	const struct v4l2_brcmstb_format *fmt_out;
	u32 width;
	u32 height;
	struct v4l2_fract pixelaspect;
	u32 src_buffer_size;

	u32 colorspace;
	u8 ycbcr_enc;
	u8 quantization;
	u8 xfer_func;

	atomic_t esparser_queued_bufs;
	struct work_struct esparser_queue_work;

	struct mutex recycle_lock;
	struct delayed_work recycle_work;

	unsigned int streamon_cap, streamon_out;
	unsigned int sequence_cap, sequence_out;
	unsigned int should_stop;
	unsigned int changed_format;
	unsigned int num_dst_bufs;
	enum v4l2_brcmstb_status status;
};

extern int v4l2_brcmstb_register_device(struct v4l2_brcmstb_device *);
extern int v4l2_brcmstb_unregister_device(struct v4l2_brcmstb_device *);

static inline dma_addr_t vb2_bmem_plane_dma_addr(struct vb2_buffer *vb,
						 unsigned int plane_no)
{
	dma_addr_t *addr = vb2_plane_cookie(vb, plane_no);

	return *addr;
}

#endif
