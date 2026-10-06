/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _SUN6I_ISP_CAPTURE_H_
#define _SUN6I_ISP_CAPTURE_H_

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define SUN6I_ISP_CAPTURE_NAME		"sun6i-isp-capture"
#define SUN6I_ISP_CAPTURE_MAIN_NAME	"sun6i-isp-capture-main"
#define SUN6I_ISP_CAPTURE_SELF_NAME	"sun6i-isp-capture-self"

/*
 * The stride register holds a stride divided by four, so four is the least
 * it can express; the vendor aligns every plane to 16 and this follows it.
 */
#define SUN6I_ISP_CAPTURE_STRIDE_ALIGN	16

#define SUN6I_ISP_CAPTURE_WIDTH_MIN	16
#define SUN6I_ISP_CAPTURE_WIDTH_MAX	3264
#define SUN6I_ISP_CAPTURE_HEIGHT_MIN	16
#define SUN6I_ISP_CAPTURE_HEIGHT_MAX	2448

struct sun6i_isp_device;
struct v4l2_format_info;

struct sun6i_isp_capture_format {
	u32	pixelformat;
	u8	output_format;
};

/*
 * The hardware writes out through two independent channels, each with
 * its own size, scaler and buffer addresses. They are laid out
 * identically, so a channel is described by where its registers are.
 */
enum sun6i_isp_capture_id {
	SUN6I_ISP_CAPTURE_MAIN	= 0,
	SUN6I_ISP_CAPTURE_SELF	= 1,
	SUN6I_ISP_CAPTURE_COUNT	= 2,
};

struct sun6i_isp_capture_regs {
	u32	size_cfg;
	u32	scale_cfg;
	u32	cfg;
	u32	y_addr0;
	u32	u_addr0;
	u32	v_addr0;
};

struct sun6i_isp_capture_state {
	struct list_head		queue;
	spinlock_t			lock; /* Queue and buffers lock. */

	/* Staged in the load buffer, and latched by the hardware. */
	struct sun6i_isp_buffer		*pending;
	struct sun6i_isp_buffer		*active;

	unsigned int			sequence;
	bool				streaming;
};

struct sun6i_isp_capture {
	struct sun6i_isp_device		*isp_dev;
	enum sun6i_isp_capture_id	id;
	const struct sun6i_isp_capture_regs *regs;
	const char			*name;

	struct sun6i_isp_capture_state	state;

	struct video_device		video_dev;
	struct vb2_queue		queue;
	struct mutex			lock; /* Queue lock. */
	struct media_pad		pad;

	struct v4l2_ctrl_handler	ctrl_handler;

	/*
	 * The format is what userspace is handed, so on a quarter turn it is
	 * already the turned one. The geometry the channel itself writes is
	 * this with the two swapped back - see
	 * sun6i_isp_capture_source_dimensions().
	 */
	struct v4l2_format		format;
	bool				mirror;
	bool				flip;
	unsigned int			rotation;
};

/* Helpers */

void sun6i_isp_capture_dimensions(struct sun6i_isp_capture *capture,
				  unsigned int *width, unsigned int *height);
void sun6i_isp_capture_format(struct sun6i_isp_capture *capture,
			      u32 *pixelformat);

/* Format */

const struct sun6i_isp_capture_format *
sun6i_isp_capture_format_find(u32 pixelformat);

unsigned int sun6i_isp_capture_stride(const struct v4l2_format_info *info,
				      unsigned int plane, unsigned int width);
unsigned int sun6i_isp_capture_image_size(u32 pixelformat, unsigned int width,
					  unsigned int height);

/* Capture */

void sun6i_isp_capture_configure(struct sun6i_isp_device *isp_dev);
void sun6i_isp_capture_rewind(struct sun6i_isp_device *isp_dev);

/* State */

void sun6i_isp_capture_state_update(struct sun6i_isp_device *isp_dev,
				    bool *update);
void sun6i_isp_capture_state_complete(struct sun6i_isp_device *isp_dev);
void sun6i_isp_capture_finish(struct sun6i_isp_device *isp_dev);

/* There is one rotator, so at most one channel may be turning. */
bool sun6i_isp_capture_rotation_taken(struct sun6i_isp_device *isp_dev,
				      struct sun6i_isp_capture *except);

bool sun6i_isp_capture_streaming(struct sun6i_isp_device *isp_dev);

/* Capture */

int sun6i_isp_capture_setup(struct sun6i_isp_device *isp_dev);
void sun6i_isp_capture_cleanup(struct sun6i_isp_device *isp_dev);

#endif
