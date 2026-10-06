/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _SUN6I_ISP_H_
#define _SUN6I_ISP_H_

#include <linux/mutex.h>

#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>

#include "sun6i_isp_capture.h"
#include "sun6i_isp_params.h"
#include "sun6i_isp_proc.h"
#include "sun6i_isp_stats.h"

#define SUN6I_ISP_NAME			"sun6i-isp"
#define SUN6I_ISP_DESCRIPTION		"Allwinner A31 ISP Device"

enum sun6i_isp_port {
	SUN6I_ISP_PORT_CSI0	= 0,
	SUN6I_ISP_PORT_CSI1	= 1,
};

struct sun6i_isp_buffer {
	struct vb2_v4l2_buffer	v4l2_buffer;
	struct list_head	list;
};

struct sun6i_isp_v4l2 {
	struct v4l2_device		v4l2_dev;
	struct media_device		media_dev;
};

struct sun6i_isp_table {
	void		*data;
	dma_addr_t	address;
	unsigned int	size;
};

/*
 * The LUT and DRC buffers are each a concatenation of fixed-size
 * sub-tables. Both supported variants place them identically; the V3s
 * just has further tables after these, for modules it alone has.
 */

#define SUN6I_ISP_TABLE_LENS_OFFSET	0x0
#define SUN6I_ISP_TABLE_LENS_SIZE	0x600
#define SUN6I_ISP_TABLE_GAMMA_OFFSET	0x600
#define SUN6I_ISP_TABLE_GAMMA_SIZE	0x200

#define SUN6I_ISP_TABLE_DRC_OFFSET	0x0
#define SUN6I_ISP_TABLE_DRC_SIZE	0x200

struct sun6i_isp_tables {
	struct sun6i_isp_table	load;
	struct sun6i_isp_table	save;

	struct sun6i_isp_table	lut;
	struct sun6i_isp_table	drc;

	/*
	 * Two statistics buffers, so the copy out of one can be deferred to
	 * the interrupt thread while the hardware fills the other.
	 */
	struct sun6i_isp_table	stats[2];
	unsigned int		stats_index;
	int			stats_done;
};

struct sun6i_isp_device {
	struct device			*dev;

	struct sun6i_isp_tables		tables;

	struct sun6i_isp_v4l2		v4l2;
	struct sun6i_isp_proc		proc;
	struct sun6i_isp_capture	captures[SUN6I_ISP_CAPTURE_COUNT];
	struct sun6i_isp_params		params;
	struct sun6i_isp_stats		stats;

	struct regmap			*regmap;
	struct clk			*clock_mod;
	struct clk			*clock_ram;
	struct reset_control		*reset;
	int				irq;

	struct dentry			*debugfs;

	spinlock_t			state_lock; /* State helpers lock. */

	/*
	 * Serialises the channels against each other while the set of streaming
	 * channels changes: each change stops the pipeline, reprograms every
	 * channel and starts it again, which is neither atomic nor allowed to
	 * interleave with the same sequence from the other channel. The queue
	 * locks videobuf2 holds around this are per channel, so they do not.
	 */
	struct mutex			stream_lock; /* Channel start/stop lock. */
};

struct sun6i_isp_variant {
	unsigned int	table_load_save_size;
	unsigned int	table_lut_size;
	unsigned int	table_drc_size;
	unsigned int	table_stats_size;
};

/* Helpers */

u32 sun6i_isp_load_read(struct sun6i_isp_device *isp_dev, u32 offset);
u32 sun6i_isp_save_read(struct sun6i_isp_device *isp_dev, u32 offset);
void sun6i_isp_load_write(struct sun6i_isp_device *isp_dev, u32 offset,
			  u32 value);
u32 sun6i_isp_address_value(dma_addr_t address);

/* Pipeline */

int sun6i_isp_pipeline_start(struct sun6i_isp_device *isp_dev);
int sun6i_isp_pipeline_restart(struct sun6i_isp_device *isp_dev);

/* State */

void sun6i_isp_state_update(struct sun6i_isp_device *isp_dev, bool ready_hold);

/* Tables */

void sun6i_isp_tables_configure(struct sun6i_isp_device *isp_dev);

#endif
