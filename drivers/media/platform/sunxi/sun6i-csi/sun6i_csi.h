/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2011-2018 Magewell Electronics Co., Ltd. (Nanjing)
 * Author: Yong Deng <yong.deng@magewell.com>
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _SUN6I_CSI_H_
#define _SUN6I_CSI_H_

#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>

#include "sun6i_csi_bridge.h"
#include "sun6i_csi_capture.h"

#define SUN6I_CSI_NAME		"sun6i-csi"
#define SUN6I_CSI_DESCRIPTION	"Allwinner A31 CSI Device"

enum sun6i_csi_port {
	SUN6I_CSI_PORT_PARALLEL		= 0,
	SUN6I_CSI_PORT_MIPI_CSI2	= 1,
	SUN6I_CSI_PORT_ISP		= 2,
};

struct sun6i_csi_buffer {
	struct vb2_v4l2_buffer		v4l2_buffer;
	struct list_head		list;
};

struct sun6i_csi_v4l2 {
	struct v4l2_device		v4l2_dev;
	struct media_device		media_dev;
};

/*
 * The pattern generator reads a DRAM buffer and drives it into the input mux
 * in place of an interface, synthesising the sync signals itself, so it is a
 * way to hand the CSI -- and the ISP behind it -- an image the driver chose.
 *
 * Nothing has ever programmed it, so what is here is deliberately a set of
 * knobs rather than a policy: every field below is reachable from debugfs so
 * that the unknowns can be walked without a kernel build each time.
 */
struct sun6i_csi_pattern {
	struct mutex			lock; /* Buffer and register state. */

	/*
	 * The generator is a source in its own right, so it is one in the
	 * graph too: link it to the bridge instead of a sensor and the
	 * pipeline runs with no sensor in it at all -- nothing to power up,
	 * nothing over i2c, and the sizes and codes on offer are the CSI's
	 * rather than whatever sensor happens to be fitted.
	 */
	struct v4l2_subdev		subdev;
	struct media_pad		pad;
	bool				registered;

	void				*buffer;
	dma_addr_t			address;
	size_t				size;

	u32				len;
	u8				cycle;
	u8				clk_div;
	u8				dly;
	bool				enabled;
	bool				keep_source;
};

struct sun6i_csi_device {
	struct device			*dev;
	struct v4l2_device		*v4l2_dev;
	struct media_device		*media_dev;

	struct sun6i_csi_v4l2		v4l2;
	struct sun6i_csi_bridge		bridge;
	struct sun6i_csi_capture	capture;
	struct sun6i_csi_pattern	pattern;

	struct regmap			*regmap;
	struct clk			*clock_mod;
	struct clk			*clock_ram;
	struct reset_control		*reset;
	struct dentry			*debugfs;

	bool				isp_available;
};

struct sun6i_csi_variant {
	unsigned long	clock_mod_rate;
};

/* ISP */

int sun6i_csi_isp_complete(struct sun6i_csi_device *csi_dev,
			   struct v4l2_device *v4l2_dev);

/* Pattern */

bool sun6i_csi_pattern_replaces_source(struct sun6i_csi_device *csi_dev);

#endif
