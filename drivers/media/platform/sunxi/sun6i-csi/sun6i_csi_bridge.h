/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _SUN6I_CSI_BRIDGE_H_
#define _SUN6I_CSI_BRIDGE_H_

#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define SUN6I_CSI_BRIDGE_NAME	"sun6i-csi-bridge"

enum sun6i_csi_bridge_pad {
	SUN6I_CSI_BRIDGE_PAD_SINK	= 0,
	SUN6I_CSI_BRIDGE_PAD_SOURCE	= 1,
	SUN6I_CSI_BRIDGE_PAD_COUNT	= 2,
};

struct sun6i_csi_device;

struct sun6i_csi_bridge_format {
	u32	mbus_code;
	u8	input_format;
	u8	input_yuv_seq;
	u8	input_yuv_seq_invert;
	/* Bytes the bus carries per pixel, which a pattern has to supply. */
	u8	bus_bpp;
};

struct sun6i_csi_bridge_source {
	struct v4l2_subdev		*subdev;
	struct v4l2_fwnode_endpoint	endpoint;
	bool				expected;
};

struct sun6i_csi_bridge_async_subdev {
	struct v4l2_async_connection	async_subdev;
	struct sun6i_csi_bridge_source	*source;
};

#define SUN6I_CSI_SOURCE_PARALLEL_MAX 2

struct sun6i_csi_bridge {
	struct v4l2_subdev		subdev;
	struct v4l2_async_notifier	notifier;
	struct media_pad		pads[2];

	struct sun6i_csi_bridge_source	source_parallel[SUN6I_CSI_SOURCE_PARALLEL_MAX];
	struct sun6i_csi_bridge_source	source_mipi_csi2;
	struct sun6i_csi_bridge_source	source_pattern;

	/*
	 * Whether the source was actually started, which is not the same as
	 * whether it should have been: a pattern leaves it alone. The v4l2
	 * core tracks each sub-device's streaming state and warns if it is
	 * stopped without having been started, and the knob that decides this
	 * can move while a stream is running, so remember what was done rather
	 * than work it out again on the way down.
	 */
	bool				source_streaming;
};

/* Format */

const struct sun6i_csi_bridge_format *
sun6i_csi_bridge_format_find(u32 mbus_code);

const struct sun6i_csi_bridge_format *
sun6i_csi_bridge_format_by_index(unsigned int index);

int sun6i_csi_bridge_frame_bytes(struct sun6i_csi_device *csi_dev, u32 *bytes);

/* Bridge */

int sun6i_csi_bridge_setup(struct sun6i_csi_device *csi_dev);
void sun6i_csi_bridge_cleanup(struct sun6i_csi_device *csi_dev);

#endif
