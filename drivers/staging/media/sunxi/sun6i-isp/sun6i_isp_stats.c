// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-v4l2.h>

#include "sun6i_isp.h"
#include "sun6i_isp_proc.h"
#include "sun6i_isp_reg.h"
#include "sun6i_isp_stats.h"
#include "uapi/sun6i-isp-config.h"

/*
 * Where each block sits in the statistics buffer the hardware fills. The
 * uapi structures are the hardware layouts, so each block is a straight
 * copy of its region.
 *
 * The region at 0x800 is left out. Nothing in the vendor driver reads it
 * and there is no evidence for what the hardware puts there, so there is
 * nothing to describe to userspace yet.
 */

#define SUN6I_ISP_STATS_HIST_OFFSET	0x0
#define SUN6I_ISP_STATS_AE_OFFSET	0x200
#define SUN6I_ISP_STATS_AF_OFFSET	0xa00
#define SUN6I_ISP_STATS_AFS_OFFSET	0xf00
#define SUN6I_ISP_STATS_AWB_OFFSET	0x1100

/*
 * The white balance region is allocated 0x1000, which would be 256 window
 * records, but the hardware only ever writes the first 0x200 of it. Copying
 * the whole region hands userspace 224 records of whatever the buffer held
 * before, and a reader that sums the array gets a number with no relation to
 * the frame. Measured on a solid white field: records 0 to 31 all satisfy
 * sum == 254 * count, and record 32 onwards does not, at every window
 * geometry tried.
 */
#define SUN6I_ISP_STATS_AWB_SIZE	0x200

/*
 * The same thing on the exposure side: the section is 0x600 but the
 * hardware writes one plane of 64 zones, 0x400, which is also where the
 * vendor's parser loop stops.
 */
#define SUN6I_ISP_STATS_AE_SIZE		0x400

/* Stats */

static void sun6i_isp_stats_buffer_fill(struct sun6i_isp_device *isp_dev,
					struct sun6i_isp_stats_buffer *stats,
					const void *base)
{
	u32 modules, blocks = 0;

	/*
	 * Which blocks ran is whatever the parameters last enabled, so read
	 * it back rather than tracking it separately.
	 */

	modules = sun6i_isp_load_read(isp_dev, SUN6I_ISP_MODULE_EN_REG);

	if (modules & SUN6I_ISP_MODULE_EN_HIST) {
		memcpy(&stats->hist, base + SUN6I_ISP_STATS_HIST_OFFSET,
		       sizeof(stats->hist));
		blocks |= SUN6I_ISP_STAT_HIST;
	}

	if (modules & SUN6I_ISP_MODULE_EN_AE) {
		memcpy(&stats->ae, base + SUN6I_ISP_STATS_AE_OFFSET,
		       SUN6I_ISP_STATS_AE_SIZE);
		blocks |= SUN6I_ISP_STAT_AE;
	}

	if (modules & SUN6I_ISP_MODULE_EN_AF) {
		memcpy(&stats->af, base + SUN6I_ISP_STATS_AF_OFFSET,
		       sizeof(stats->af));
		blocks |= SUN6I_ISP_STAT_AF;
	}

	if (modules & SUN6I_ISP_MODULE_EN_AFS) {
		memcpy(&stats->afs, base + SUN6I_ISP_STATS_AFS_OFFSET,
		       sizeof(stats->afs));
		blocks |= SUN6I_ISP_STAT_AFS;
	}

	if (modules & SUN6I_ISP_MODULE_EN_AWB) {
		memcpy(&stats->awb, base + SUN6I_ISP_STATS_AWB_OFFSET,
		       SUN6I_ISP_STATS_AWB_SIZE);
		blocks |= SUN6I_ISP_STAT_AWB;
	}

	/*
	 * The demosaic has no enable bit and always runs, and it reports by
	 * overwriting its own configuration in the save region rather than
	 * through the statistics DMA -- so this comes from there, not from
	 * `base`.
	 */
	stats->cfa.min_rgb =
		SUN6I_ISP_CFA_CFG0_MIN_RGB_GET(sun6i_isp_save_read(isp_dev,
						SUN6I_ISP_CFA_CFG0_REG));
	stats->cfa.texture = sun6i_isp_save_read(isp_dev,
						 SUN6I_ISP_CFA_TEX_REG);
	blocks |= SUN6I_ISP_STAT_CFA;

	stats->blocks_valid = blocks;
}

/*
 * Called from the frame finish interrupt, which is when the hardware is
 * done writing the buffer for this frame.
 */
void sun6i_isp_stats_finish(struct sun6i_isp_device *isp_dev,
			    const void *base)
{
	struct sun6i_isp_stats_state *state = &isp_dev->stats.state;
	struct sun6i_isp_stats_buffer *stats;
	struct sun6i_isp_buffer *isp_buffer;
	struct vb2_buffer *vb2_buffer;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	if (!state->streaming || list_empty(&state->queue))
		goto complete;

	isp_buffer = list_first_entry(&state->queue, struct sun6i_isp_buffer,
				      list);
	vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;

	stats = vb2_plane_vaddr(vb2_buffer, 0);
	if (!stats)
		goto complete;

	memset(stats, 0, sizeof(*stats));

	sun6i_isp_stats_buffer_fill(isp_dev, stats, base);

	stats->sequence =
		isp_dev->captures[SUN6I_ISP_CAPTURE_MAIN].state.sequence;

	list_del(&isp_buffer->list);

	vb2_buffer->timestamp = ktime_get_ns();
	isp_buffer->v4l2_buffer.sequence = stats->sequence;

	vb2_buffer_done(vb2_buffer, VB2_BUF_STATE_DONE);

complete:
	spin_unlock_irqrestore(&state->lock, flags);
}

static void sun6i_isp_stats_state_cleanup(struct sun6i_isp_device *isp_dev,
					  bool error)
{
	struct sun6i_isp_stats_state *state = &isp_dev->stats.state;
	struct sun6i_isp_buffer *isp_buffer;
	struct vb2_buffer *vb2_buffer;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	list_for_each_entry(isp_buffer, &state->queue, list) {
		vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;
		vb2_buffer_done(vb2_buffer, error ? VB2_BUF_STATE_ERROR :
				VB2_BUF_STATE_QUEUED);
	}

	INIT_LIST_HEAD(&state->queue);

	spin_unlock_irqrestore(&state->lock, flags);
}

/* Queue */

static int sun6i_isp_stats_queue_setup(struct vb2_queue *queue,
				       unsigned int *buffers_count,
				       unsigned int *planes_count,
				       unsigned int sizes[],
				       struct device *alloc_devs[])
{
	struct sun6i_isp_device *isp_dev = vb2_get_drv_priv(queue);
	unsigned int size = isp_dev->stats.format.fmt.meta.buffersize;

	if (*planes_count)
		return sizes[0] < size ? -EINVAL : 0;

	*planes_count = 1;
	sizes[0] = size;

	return 0;
}

static int sun6i_isp_stats_buffer_prepare(struct vb2_buffer *vb2_buffer)
{
	struct sun6i_isp_device *isp_dev =
		vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct v4l2_device *v4l2_dev = &isp_dev->v4l2.v4l2_dev;
	unsigned int size = isp_dev->stats.format.fmt.meta.buffersize;

	if (vb2_plane_size(vb2_buffer, 0) < size) {
		v4l2_err(v4l2_dev, "buffer too small (%lu < %u)\n",
			 vb2_plane_size(vb2_buffer, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb2_buffer, 0, size);

	return 0;
}

static void sun6i_isp_stats_buffer_queue(struct vb2_buffer *vb2_buffer)
{
	struct sun6i_isp_device *isp_dev =
		vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct sun6i_isp_stats_state *state = &isp_dev->stats.state;
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);
	struct sun6i_isp_buffer *isp_buffer =
		container_of(v4l2_buffer, struct sun6i_isp_buffer, v4l2_buffer);
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);
	list_add_tail(&isp_buffer->list, &state->queue);
	spin_unlock_irqrestore(&state->lock, flags);
}

static int sun6i_isp_stats_start_streaming(struct vb2_queue *queue,
					   unsigned int count)
{
	struct sun6i_isp_device *isp_dev = vb2_get_drv_priv(queue);

	isp_dev->stats.state.streaming = true;

	return 0;
}

static void sun6i_isp_stats_stop_streaming(struct vb2_queue *queue)
{
	struct sun6i_isp_device *isp_dev = vb2_get_drv_priv(queue);

	isp_dev->stats.state.streaming = false;
	sun6i_isp_stats_state_cleanup(isp_dev, true);
}

static const struct vb2_ops sun6i_isp_stats_queue_ops = {
	.queue_setup		= sun6i_isp_stats_queue_setup,
	.buf_prepare		= sun6i_isp_stats_buffer_prepare,
	.buf_queue		= sun6i_isp_stats_buffer_queue,
	.start_streaming	= sun6i_isp_stats_start_streaming,
	.stop_streaming		= sun6i_isp_stats_stop_streaming,
};

/* Video Device */

static int sun6i_isp_stats_querycap(struct file *file, void *private,
				    struct v4l2_capability *capability)
{
	struct sun6i_isp_device *isp_dev = video_drvdata(file);
	struct video_device *video_dev = &isp_dev->stats.video_dev;

	strscpy(capability->driver, SUN6I_ISP_NAME, sizeof(capability->driver));
	strscpy(capability->card, video_dev->name, sizeof(capability->card));
	snprintf(capability->bus_info, sizeof(capability->bus_info),
		 "platform:%s", dev_name(isp_dev->dev));

	return 0;
}

static int sun6i_isp_stats_enum_fmt(struct file *file, void *private,
				    struct v4l2_fmtdesc *fmtdesc)
{
	struct sun6i_isp_device *isp_dev = video_drvdata(file);
	struct v4l2_meta_format *params_format =
		&isp_dev->stats.format.fmt.meta;

	if (fmtdesc->index > 0)
		return -EINVAL;

	fmtdesc->pixelformat = params_format->dataformat;

	return 0;
}

static int sun6i_isp_stats_g_fmt(struct file *file, void *private,
				 struct v4l2_format *format)
{
	struct sun6i_isp_device *isp_dev = video_drvdata(file);

	*format = isp_dev->stats.format;

	return 0;
}

static const struct v4l2_ioctl_ops sun6i_isp_stats_ioctl_ops = {
	.vidioc_querycap		= sun6i_isp_stats_querycap,

	.vidioc_enum_fmt_meta_cap	= sun6i_isp_stats_enum_fmt,
	.vidioc_g_fmt_meta_cap		= sun6i_isp_stats_g_fmt,
	.vidioc_s_fmt_meta_cap		= sun6i_isp_stats_g_fmt,
	.vidioc_try_fmt_meta_cap	= sun6i_isp_stats_g_fmt,

	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations sun6i_isp_stats_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
};

/* Stats */

int sun6i_isp_stats_setup(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_stats *stats = &isp_dev->stats;
	struct sun6i_isp_stats_state *state = &stats->state;
	struct v4l2_device *v4l2_dev = &isp_dev->v4l2.v4l2_dev;
	struct v4l2_subdev *proc_subdev = &isp_dev->proc.subdev;
	struct video_device *video_dev = &stats->video_dev;
	struct vb2_queue *queue = &stats->queue;
	struct media_pad *pad = &stats->pad;
	struct v4l2_format *format = &stats->format;
	struct v4l2_meta_format *meta_format = &format->fmt.meta;
	int ret;

	/* State */

	INIT_LIST_HEAD(&state->queue);
	spin_lock_init(&state->lock);

	/* Media Pads */

	pad->flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;

	ret = media_entity_pads_init(&video_dev->entity, 1, pad);
	if (ret)
		goto error_mutex;

	/* Queue */

	mutex_init(&stats->lock);

	queue->type = V4L2_BUF_TYPE_META_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_DMABUF;
	queue->buf_struct_size = sizeof(struct sun6i_isp_buffer);
	queue->ops = &sun6i_isp_stats_queue_ops;
	queue->mem_ops = &vb2_vmalloc_memops;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock = &stats->lock;
	queue->dev = isp_dev->dev;
	queue->drv_priv = isp_dev;

	ret = vb2_queue_init(queue);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to initialize vb2 queue: %d\n", ret);
		goto error_media_entity;
	}

	/* V4L2 Format */

	format->type = queue->type;
	meta_format->dataformat = V4L2_META_FMT_SUN6I_ISP_STATS;
	meta_format->buffersize = sizeof(struct sun6i_isp_stats_buffer);

	/* Video Device */

	strscpy(video_dev->name, SUN6I_ISP_STATS_NAME,
		sizeof(video_dev->name));
	video_dev->device_caps = V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING;
	video_dev->vfl_dir = VFL_DIR_RX;
	video_dev->release = video_device_release_empty;
	video_dev->fops = &sun6i_isp_stats_fops;
	video_dev->ioctl_ops = &sun6i_isp_stats_ioctl_ops;
	video_dev->v4l2_dev = v4l2_dev;
	video_dev->queue = queue;
	video_dev->lock = &stats->lock;

	video_set_drvdata(video_dev, isp_dev);

	ret = video_register_device(video_dev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to register video device: %d\n",
			 ret);
		goto error_media_entity;
	}

	/* Media Pad Link */

	ret = media_create_pad_link(&proc_subdev->entity,
				    SUN6I_ISP_PROC_PAD_SOURCE,
				    &video_dev->entity, 0,
				    MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed to create %s:%u -> %s:%u link\n",
			 proc_subdev->entity.name, SUN6I_ISP_PROC_PAD_SOURCE,
			 video_dev->entity.name, 0);
		goto error_video_device;
	}

	return 0;

error_video_device:
	vb2_video_unregister_device(video_dev);

error_media_entity:
	media_entity_cleanup(&video_dev->entity);

error_mutex:
	mutex_destroy(&stats->lock);

	return ret;
}

void sun6i_isp_stats_cleanup(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_stats *stats = &isp_dev->stats;
	struct video_device *video_dev = &stats->video_dev;

	vb2_video_unregister_device(video_dev);
	media_entity_cleanup(&video_dev->entity);
	mutex_destroy(&stats->lock);
}
