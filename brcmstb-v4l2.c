// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020,2021 Jaedon Shin <jaedon.shin@gmail.com>
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-vmalloc.h>

#include "brcmstb-v4l2.h"

static u32 get_output_size(u32 width, u32 height)
{
	return ALIGN(width * height, SZ_64K);
}

static int frames_ready(struct v4l2_brcmstb_context *context)
{
	struct vb2_v4l2_buffer *vbuf;
	dma_addr_t bufs[3];
	u64 timestamp = 0;
	u32 field = 0;
	u32 flags = 0;
	struct v4l2_timecode timecode;
	u32 output_size = get_output_size(context->width, context->height);
	int ret = 0;

	vbuf = v4l2_m2m_dst_buf_remove(context->m2m_ctx);
	if (!vbuf)
		return -ENOMEM;

	switch (context->pixfmt_cap) {
	case V4L2_PIX_FMT_NV12M:
		bufs[0] = vb2_bmem_plane_dma_addr(&vbuf->vb2_buf, 0);
		bufs[1] = vb2_bmem_plane_dma_addr(&vbuf->vb2_buf, 1);
		vbuf->vb2_buf.planes[0].bytesused = output_size;
		vbuf->vb2_buf.planes[1].bytesused = output_size / 2;
		break;
	case V4L2_PIX_FMT_YUV420M:
		bufs[0] = vb2_bmem_plane_dma_addr(&vbuf->vb2_buf, 0);
		bufs[1] = vb2_bmem_plane_dma_addr(&vbuf->vb2_buf, 1);
		bufs[2] = vb2_bmem_plane_dma_addr(&vbuf->vb2_buf, 2);
		vbuf->vb2_buf.planes[0].bytesused = output_size;
		vbuf->vb2_buf.planes[1].bytesused = output_size / 4;
		vbuf->vb2_buf.planes[2].bytesused = output_size / 4;
		break;
	}

	if (context->device->ops->dequeue)
		ret = context->device->ops->dequeue(context, bufs, &timestamp,
						    &field, &flags, &timecode);

	vbuf->vb2_buf.timestamp = timestamp;
	vbuf->field = field;
	vbuf->flags = flags;
	vbuf->sequence = context->sequence_cap++;

	if (context->should_stop &&
	    atomic_read(&context->esparser_queued_bufs) <= 1) {
		const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };

		v4l2_event_queue_fh(&context->fh, &ev);
		vbuf->flags |= V4L2_BUF_FLAG_LAST;
	} else if (context->status == STATUS_NEEDS_RESUME) {
		vbuf->flags |= V4L2_BUF_FLAG_LAST;
		context->sequence_cap = 0;
	}

	v4l2_m2m_buf_done(vbuf, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

	schedule_work(&context->esparser_queue_work);
	atomic_dec(&context->esparser_queued_bufs);

	return 0;
}

static void recycle_work(struct work_struct *work)
{
	struct v4l2_brcmstb_context *context = container_of(
		work, struct v4l2_brcmstb_context, recycle_work.work);
	int num_frames = 0, i;

	if (context->device->ops->dequeuenum)
		num_frames = context->device->ops->dequeuenum(context);

	for (i = 0; i < num_frames; i++) {
		if (frames_ready(context))
			break;
	}

	schedule_delayed_work(&context->recycle_work, msecs_to_jiffies(10));
}

static u32 esparser_get_free_space(struct v4l2_brcmstb_context *context)
{
	if (context->device->ops->queuenum)
		return context->device->ops->queuenum(context);

	return 0;
}

static int esparser_queue(struct v4l2_brcmstb_context *context,
			  struct vb2_v4l2_buffer *vbuf)
{
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	const u8 *buf = vb2_plane_vaddr(vb, 0);
	u32 len = vbuf->planes[0].bytesused;
	int ret = 0;

	if (esparser_get_free_space(context) < len)
		return -EAGAIN;

	v4l2_m2m_src_buf_remove_by_buf(context->m2m_ctx, vbuf);

	if (context->device->ops->queue)
		ret = context->device->ops->queue(context, buf, len,
						  vb->timestamp);

	vbuf->flags = 0;
	vbuf->field = V4L2_FIELD_NONE;
	vbuf->sequence = context->sequence_out++;

	if (ret <= 0) {
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);

		return 0;
	}

	atomic_inc(&context->esparser_queued_bufs);
	v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);

	return 0;
}

static void esparser_queue_all_src(struct work_struct *work)
{
	struct v4l2_m2m_buffer *buf, *n;
	struct v4l2_brcmstb_context *context = container_of(
		work, struct v4l2_brcmstb_context, esparser_queue_work);

	mutex_lock(&context->lock);
	v4l2_m2m_for_each_src_buf_safe(context->m2m_ctx, buf, n) {
		if (context->should_stop)
			break;

		if (esparser_queue(context, &buf->vb) < 0)
			break;
	}
	mutex_unlock(&context->lock);
}

static void device_run(void *priv)
{
	struct v4l2_brcmstb_context *context = priv;

	schedule_work(&context->esparser_queue_work);
}

static void job_abort(void *priv)
{
	struct v4l2_brcmstb_context *context = priv;

	v4l2_m2m_job_finish(context->m2m_dev, context->m2m_ctx);
}

static const struct v4l2_m2m_ops m2m_ops = {
	.device_run = device_run,
	.job_abort = job_abort,
};

static void process_num_buffers(struct vb2_queue *q,
				struct v4l2_brcmstb_context *context,
				unsigned int *num_buffers, bool is_reqbufs)
{
	const struct v4l2_brcmstb_format *fmt_out = context->fmt_out;
	unsigned int buffers_total = q->num_buffers + *num_buffers;
	u32 min_buf_capture = v4l2_ctrl_g_ctrl(context->ctrl_min_buf_capture);

	if (q->num_buffers + *num_buffers < min_buf_capture)
		*num_buffers = min_buf_capture - q->num_buffers;
	if (is_reqbufs && buffers_total < fmt_out->min_buffers)
		*num_buffers = fmt_out->min_buffers - q->num_buffers;
	if (buffers_total > fmt_out->max_buffers)
		*num_buffers = fmt_out->max_buffers - q->num_buffers;

	context->num_dst_bufs = q->num_buffers + *num_buffers;
	q->min_buffers_needed = max(fmt_out->min_buffers, context->num_dst_bufs);
}

static int queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
		       unsigned int *num_planes, unsigned int sizes[],
		       struct device *alloc_devs[])
{
	struct v4l2_brcmstb_context *context = vb2_get_drv_priv(q);
	u32 output_size = get_output_size(context->width, context->height);

	if (*num_planes) {
		switch (q->type) {
		case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
			if (*num_planes != 1 ||
			    sizes[0] < context->src_buffer_size)
				return -EINVAL;
			break;
		case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
			switch (context->pixfmt_cap) {
			case V4L2_PIX_FMT_NV12M:
				if (*num_planes != 2 ||
				    sizes[0] < output_size ||
				    sizes[1] < output_size / 2)
					return -EINVAL;
				break;
			case V4L2_PIX_FMT_YUV420M:
				if (*num_planes != 3 ||
				    sizes[0] < output_size ||
				    sizes[1] < output_size / 4 ||
				    sizes[2] < output_size / 4)
					return -EINVAL;
				break;
			default:
				return -EINVAL;
			}

			process_num_buffers(q, context, num_buffers, false);
			break;
		}

		return 0;
	}

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		sizes[0] = context->src_buffer_size;
		*num_planes = 1;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		switch (context->pixfmt_cap) {
		case V4L2_PIX_FMT_NV12M:
			sizes[0] = output_size;
			sizes[1] = output_size / 2;
			*num_planes = 2;
			break;
		case V4L2_PIX_FMT_YUV420M:
			sizes[0] = output_size;
			sizes[1] = output_size / 4;
			sizes[2] = output_size / 4;
			*num_planes = 3;
			break;
		default:
			return -EINVAL;
		}

		process_num_buffers(q, context, num_buffers, true);
		break;
	default:
		return -EINVAL;
	}

	context->changed_format = 1;
	return 0;
}

static void buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct v4l2_brcmstb_context *context = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m_ctx = context->m2m_ctx;

	v4l2_m2m_buf_queue(m2m_ctx, vbuf);

	if (!context->streamon_out)
		return;

	schedule_work(&context->esparser_queue_work);
}

static int buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct v4l2_brcmstb_context *context = vb2_get_drv_priv(q);

	if (context->device->curr_context &&
	    context->device->curr_context != context) {
		const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };
		v4l2_event_queue_fh(&context->device->curr_context->fh, &ev);
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		context->streamon_out = 1;
	else
		context->streamon_cap = 1;

	if (!context->streamon_out)
		return 0;

	if (context->status == STATUS_NEEDS_RESUME &&
	    q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    context->changed_format) {
		context->status = STATUS_RUNNING;
		return 0;
	}

	if (context->status == STATUS_RUNNING ||
	    context->status == STATUS_NEEDS_RESUME ||
	    context->status == STATUS_INIT)
		return 0;

	if (context->device->ops->streamon)
		context->device->ops->streamon(context);

	context->should_stop = 0;
	context->pixelaspect.numerator = 1;
	context->pixelaspect.denominator = 1;
	atomic_set(&context->esparser_queued_bufs, 0);
	v4l2_ctrl_s_ctrl(context->ctrl_min_buf_capture, 1);

	context->sequence_cap = 0;
	context->sequence_out = 0;
	schedule_delayed_work(&context->recycle_work, 0);

	context->status = STATUS_INIT;
	context->device->curr_context = context;
	schedule_work(&context->esparser_queue_work);
	return 0;
}

static void stop_streaming(struct vb2_queue *q)
{
	struct v4l2_brcmstb_context *context = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buf;

	if (context->status == STATUS_RUNNING ||
	    context->status == STATUS_INIT ||
	    (context->status == STATUS_NEEDS_RESUME &&
	     (!context->streamon_out || !context->streamon_cap))) {
		cancel_delayed_work_sync(&context->recycle_work);

		context->device->curr_context = NULL;
		context->status = STATUS_STOPPED;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (context->device->ops->streamoff)
			context->device->ops->streamoff(context);

		while ((buf = v4l2_m2m_src_buf_remove(context->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		context->streamon_out = 0;
	} else {
		if (context->status >= STATUS_RUNNING &&
		    context->device->ops->drain)
			context->device->ops->drain(context);

		while ((buf = v4l2_m2m_dst_buf_remove(context->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		context->streamon_cap = 0;
	}
}

static const struct vb2_ops vb2_ops = {
	.queue_setup = queue_setup,
	.buf_queue = buf_queue,
	.buf_prepare = buf_prepare,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, "brcmstb-decoder", sizeof(cap->driver));
	strscpy(cap->card, "Broadcom STB Decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:brcmstb-decoder",
		sizeof(cap->bus_info));

	return 0;
}

static const struct v4l2_brcmstb_format *
find_format(const struct v4l2_brcmstb_format *fmts, u32 size, u32 pixfmt)
{
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmts[i].pixfmt == pixfmt)
			return &fmts[i];
	}

	return NULL;
}

static unsigned int
supports_pixfmt_cap(const struct v4l2_brcmstb_format *fmt_out, u32 pixfmt_cap)
{
	int i;

	for (i = 0; fmt_out->pixfmts_cap[i]; i++)
		if (fmt_out->pixfmts_cap[i] == pixfmt_cap)
			return 1;

	return 0;
}

static const struct v4l2_brcmstb_format *
try_fmt_common(struct v4l2_brcmstb_context *context, u32 size,
	       struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = pixmp->plane_fmt;
	const struct v4l2_brcmstb_format *fmts = context->device->formats;
	const struct v4l2_brcmstb_format *fmt_out = NULL;
	u32 output_size = 0;

	memset(pfmt[0].reserved, 0, sizeof(pfmt[0].reserved));
	memset(pixmp->reserved, 0, sizeof(pixmp->reserved));

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		fmt_out = find_format(fmts, size, pixmp->pixelformat);
		if (!fmt_out) {
			pixmp->pixelformat = V4L2_PIX_FMT_H264;
			fmt_out = find_format(fmts, size, pixmp->pixelformat);
		}
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		fmt_out = context->fmt_out;
		break;
	default:
		return NULL;
	}

	pixmp->width = clamp(pixmp->width, (u32)256, fmt_out->max_width);
	pixmp->height = clamp(pixmp->height, (u32)144, fmt_out->max_height);
	output_size = get_output_size(pixmp->width, pixmp->height);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (!pfmt[0].sizeimage)
			pfmt[0].sizeimage = context->src_buffer_size;
		pfmt[0].bytesperline = 0;
		pixmp->num_planes = 1;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt_out = context->fmt_out;
		if (!supports_pixfmt_cap(fmt_out, pixmp->pixelformat))
			pixmp->pixelformat = fmt_out->pixfmts_cap[0];

		memset(pfmt[1].reserved, 0, sizeof(pfmt[1].reserved));
		if (pixmp->pixelformat == V4L2_PIX_FMT_NV12M) {
			pfmt[0].sizeimage = output_size;
			pfmt[0].bytesperline = ALIGN(pixmp->width, 32);

			pfmt[1].sizeimage = output_size / 2;
			pfmt[1].bytesperline = ALIGN(pixmp->width, 32);
			pixmp->num_planes = 2;
		} else if (pixmp->pixelformat == V4L2_PIX_FMT_YUV420M) {
			pfmt[0].bytesperline = ALIGN(pixmp->width, 16);
			pfmt[0].sizeimage = pfmt[0].bytesperline * pixmp->height;

			pfmt[1].bytesperline = ALIGN(pixmp->width / 2, 16);
			pfmt[1].sizeimage = pfmt[1].bytesperline * pixmp->height / 2;

			pfmt[2].bytesperline = ALIGN(pixmp->width / 2, 16);
			pfmt[1].sizeimage = pfmt[2].bytesperline * pixmp->height / 2;
			pixmp->num_planes = 3;
		}
	}

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	return fmt_out;
}

static int vidioc_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);

	try_fmt_common(context, context->device->num_formats, f);

	return 0;
}

static int vidioc_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		pixmp->pixelformat = context->pixfmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		pixmp->pixelformat = context->fmt_out->pixfmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixmp->width = context->width;
		pixmp->height = context->height;
		pixmp->colorspace = context->colorspace;
		pixmp->ycbcr_enc = context->ycbcr_enc;
		pixmp->quantization = context->quantization;
		pixmp->xfer_func = context->xfer_func;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixmp->width = context->width;
		pixmp->height = context->height;
	}

	try_fmt_common(context, context->device->num_formats, f);

	return 0;
}

static int vidioc_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	u32 num_formats = context->device->num_formats;
	const struct v4l2_brcmstb_format *fmt_out;
	struct v4l2_pix_format_mplane orig_pixmp;
	struct v4l2_format format;
	u32 pixfmt_out = 0, pixfmt_cap = 0;

	orig_pixmp = *pixmp;

	fmt_out = try_fmt_common(context, num_formats, f);
	if (!fmt_out)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixfmt_out = pixmp->pixelformat;
		pixfmt_cap = context->pixfmt_cap;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixfmt_cap = pixmp->pixelformat;
		pixfmt_out = context->fmt_out->pixfmt;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_out;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	try_fmt_common(context, num_formats, &format);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		context->width = format.fmt.pix_mp.width;
		context->height = format.fmt.pix_mp.height;
		context->colorspace = pixmp->colorspace;
		context->ycbcr_enc = pixmp->ycbcr_enc;
		context->quantization = pixmp->quantization;
		context->xfer_func = pixmp->xfer_func;
		context->src_buffer_size = pixmp->plane_fmt[0].sizeimage;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_cap;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	try_fmt_common(context, num_formats, &format);

	context->width = format.fmt.pix_mp.width;
	context->height = format.fmt.pix_mp.height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		context->fmt_out = fmt_out;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		context->pixfmt_cap = format.fmt.pix_mp.pixelformat;

	return 0;
}

static int vidioc_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);
	const struct v4l2_brcmstb_format *fmt_out;

	memset(f->reserved, 0, sizeof(f->reserved));

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (f->index >= context->device->num_formats)
			return -EINVAL;

		fmt_out = &context->device->formats[f->index];
		f->pixelformat = fmt_out->pixfmt;
		f->flags = fmt_out->flags;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt_out = context->fmt_out;
		if (f->index >= 4 || !fmt_out->pixfmts_cap[f->index])
			return -EINVAL;

		f->pixelformat = fmt_out->pixfmts_cap[f->index];
	} else {
		return -EINVAL;
	}

	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *fsize)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);
	const struct v4l2_brcmstb_format *formats = context->device->formats;
	const struct v4l2_brcmstb_format *fmt;
	u32 num_formats = context->device->num_formats;

	fmt = find_format(formats, num_formats, fsize->pixel_format);
	if (!fmt || fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;

	fsize->stepwise.min_width = 256;
	fsize->stepwise.max_width = fmt->max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 144;
	fsize->stepwise.max_height = fmt->max_height;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int vidioc_decoder_cmd(struct file *file, void *fh,
			      struct v4l2_decoder_cmd *cmd)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, cmd);
	if (ret)
		return ret;

	if (!(context->streamon_out & context->streamon_cap))
		return 0;

	if (cmd->cmd == V4L2_DEC_CMD_START) {
		v4l2_m2m_clear_state(context->m2m_ctx);
		context->should_stop = 0;
		return 0;
	}

	/* Should not happen */
	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

	context->should_stop = 1;

	v4l2_m2m_mark_stopped(context->m2m_ctx);

	if (context->device->ops->drain)
		context->device->ops->drain(context);

	return ret;
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static int vidioc_g_pixelaspect(struct file *file, void *fh, int type,
				struct v4l2_fract *f)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	*f = context->pixelaspect;
	return 0;
}

static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt,
	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_enum_framesizes = vidioc_enum_framesizes,
	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = vidioc_decoder_cmd,
	.vidioc_g_pixelaspect = vidioc_g_pixelaspect,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct v4l2_brcmstb_context *context = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vb2_ops;
	src_vq->mem_ops = &vb2_vmalloc_memops;
	src_vq->drv_priv = context;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->min_buffers_needed = 1;
	src_vq->dev = context->device->dev;
	src_vq->lock = &context->lock;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &vb2_ops;
	dst_vq->mem_ops = context->device->memops;
	dst_vq->drv_priv = context;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->min_buffers_needed = 1;
	dst_vq->dev = context->device->dev;
	dst_vq->lock = &context->lock;
	ret = vb2_queue_init(dst_vq);
	if (ret) {
		vb2_queue_release(src_vq);
		return ret;
	}

	return 0;
}

static int v4l2_brcmstb_init_ctrls(struct v4l2_brcmstb_context *context)
{
	struct v4l2_ctrl_handler *ctrl_handler = &context->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_handler, 1);
	if (ret)
		return ret;

	context->ctrl_min_buf_capture =
		v4l2_ctrl_new_std(ctrl_handler, NULL,
				  V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 32, 1,
				  1);

	ret = ctrl_handler->error;
	if (ret) {
		v4l2_ctrl_handler_free(ctrl_handler);
		return ret;
	}

	return 0;
}

static int v4l2_brcmstb_open(struct file *file)
{
	struct v4l2_brcmstb_device *device = video_drvdata(file);
	const struct v4l2_brcmstb_format *formats = device->formats;
	struct v4l2_brcmstb_context *context;
	int ret;

	context = kzalloc(sizeof(*context), GFP_KERNEL);
	if (!context)
		return -ENOMEM;

	context->device = device;

	context->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(context->m2m_dev)) {
		ret = PTR_ERR(context->m2m_dev);
		goto err_free_context;
	}

	context->m2m_ctx =
		v4l2_m2m_ctx_init(context->m2m_dev, context, queue_init);
	if (IS_ERR(context->m2m_ctx)) {
		ret = PTR_ERR(context->m2m_ctx);
		goto err_m2m_release;
	}

	ret = v4l2_brcmstb_init_ctrls(context);
	if (ret)
		goto err_m2m_release;

	context->pixfmt_cap = formats[0].pixfmts_cap[0];
	context->fmt_out = &formats[0];
	context->width = 1280;
	context->height = 720;
	context->pixelaspect.numerator = 1;
	context->pixelaspect.denominator = 1;
	context->src_buffer_size = SZ_1M;

	INIT_WORK(&context->esparser_queue_work, esparser_queue_all_src);
	mutex_init(&context->lock);
	mutex_init(&context->recycle_lock);
	INIT_DELAYED_WORK(&context->recycle_work, recycle_work);

	v4l2_fh_init(&context->fh, device->vdev_dec);
	context->fh.ctrl_handler = &context->ctrl_handler;
	v4l2_fh_add(&context->fh);
	context->fh.m2m_ctx = context->m2m_ctx;
	file->private_data = &context->fh;

	return 0;

err_m2m_release:
	v4l2_m2m_release(context->m2m_dev);
err_free_context:
	kfree(context);
	return ret;
}

static int v4l2_brcmstb_close(struct file *file)
{
	struct v4l2_brcmstb_context *context = container_of(
		file->private_data, struct v4l2_brcmstb_context, fh);

	v4l2_m2m_ctx_release(context->m2m_ctx);
	v4l2_m2m_release(context->m2m_dev);
	v4l2_fh_del(&context->fh);
	v4l2_fh_exit(&context->fh);

	mutex_destroy(&context->lock);
	mutex_destroy(&context->recycle_lock);

	kfree(context);

	return 0;
}

static const struct v4l2_file_operations fops = {
	.owner = THIS_MODULE,
	.open = v4l2_brcmstb_open,
	.release = v4l2_brcmstb_close,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

int v4l2_brcmstb_register_device(struct v4l2_brcmstb_device *device)
{
	struct video_device *vdev;
	int ret;

	strscpy(device->v4l2_dev.name, "brcmstb-decoder",
		sizeof(device->v4l2_dev.name));
	ret = v4l2_device_register(device->dev, &device->v4l2_dev);
	if (ret) {
		ret = -ENOMEM;
		goto err_fini;
	}

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_vdev_release;
	}

	device->vdev_dec = vdev;
	mutex_init(&device->lock);

	strscpy(vdev->name, "brcmstb-decoder", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &fops;
	vdev->ioctl_ops = &ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &device->v4l2_dev;
	vdev->lock = &device->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	video_set_drvdata(vdev, device);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_vdev_release;

	return 0;

err_vdev_release:
	video_device_release(vdev);
err_fini:
	return ret;
}
EXPORT_SYMBOL(v4l2_brcmstb_register_device);

int v4l2_brcmstb_unregister_device(struct v4l2_brcmstb_device *device)
{
	video_unregister_device(device->vdev_dec);

	return 0;
}
EXPORT_SYMBOL(v4l2_brcmstb_unregister_device);

MODULE_DESCRIPTION("Broadcom STB decoder driver for Video for Linux 2");
MODULE_AUTHOR("Jaedon Shin <jaedon.shin@gmail.com>");
MODULE_LICENSE("GPL v2");
