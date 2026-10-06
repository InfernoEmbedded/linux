/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _SUN6I_ISP_STATS_H_
#define _SUN6I_ISP_STATS_H_

#include <media/v4l2-device.h>

#define SUN6I_ISP_STATS_NAME		"sun6i-isp-stats"

struct sun6i_isp_device;

struct sun6i_isp_stats_state {
	struct list_head		queue;
	spinlock_t			lock; /* Queue and buffers lock. */

	bool				streaming;
};

struct sun6i_isp_stats {
	struct sun6i_isp_stats_state	state;

	struct video_device		video_dev;
	struct vb2_queue		queue;
	struct mutex			lock; /* Queue lock. */
	struct media_pad		pad;

	struct v4l2_format		format;
};

/* Stats */

void sun6i_isp_stats_finish(struct sun6i_isp_device *isp_dev,
			    const void *base);

int sun6i_isp_stats_setup(struct sun6i_isp_device *isp_dev);
void sun6i_isp_stats_cleanup(struct sun6i_isp_device *isp_dev);

#endif
