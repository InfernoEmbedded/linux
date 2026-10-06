// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2011-2018 Magewell Electronics Co., Ltd. (Nanjing)
 * Author: Yong Deng <yong.deng@magewell.com>
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mc.h>

#include "sun6i_csi.h"
#include "sun6i_csi_bridge.h"
#include "sun6i_csi_capture.h"
#include "sun6i_csi_reg.h"

static int sun6i_csi_pattern_complete(struct sun6i_csi_device *csi_dev);

/* ISP */

int sun6i_csi_isp_complete(struct sun6i_csi_device *csi_dev,
			   struct v4l2_device *v4l2_dev)
{
	int ret;

	if (csi_dev->v4l2_dev && csi_dev->v4l2_dev != v4l2_dev)
		return -EINVAL;

	csi_dev->v4l2_dev = v4l2_dev;
	csi_dev->media_dev = v4l2_dev->mdev;

	ret = sun6i_csi_capture_setup(csi_dev);
	if (ret)
		return ret;

	return sun6i_csi_pattern_complete(csi_dev);
}

static int sun6i_csi_isp_detect(struct sun6i_csi_device *csi_dev)
{
	struct device *dev = csi_dev->dev;
	struct fwnode_handle *handle;

	/*
	 * ISP is not available if not connected via fwnode graph.
	 * This will also check that the remote parent node is available.
	 */
	handle = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev),
						 SUN6I_CSI_PORT_ISP, 0,
						 FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!handle)
		return 0;

	fwnode_handle_put(handle);

	if (!IS_ENABLED(CONFIG_VIDEO_SUN6I_ISP)) {
		dev_warn(dev,
			 "ISP link is detected but not enabled in kernel config!");
		return 0;
	}

	csi_dev->isp_available = true;

	return 0;
}

/* Media */

static const struct media_device_ops sun6i_csi_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

/* V4L2 */

static int sun6i_csi_v4l2_setup(struct sun6i_csi_device *csi_dev)
{
	struct sun6i_csi_v4l2 *v4l2 = &csi_dev->v4l2;
	struct media_device *media_dev = &v4l2->media_dev;
	struct v4l2_device *v4l2_dev = &v4l2->v4l2_dev;
	struct device *dev = csi_dev->dev;
	int ret;

	/* Media Device */

	strscpy(media_dev->model, SUN6I_CSI_DESCRIPTION,
		sizeof(media_dev->model));
	media_dev->hw_revision = 0;
	media_dev->ops = &sun6i_csi_media_ops;
	media_dev->dev = dev;

	media_device_init(media_dev);

	ret = media_device_register(media_dev);
	if (ret) {
		dev_err(dev, "failed to register media device: %d\n", ret);
		goto error_media;
	}

	/* V4L2 Device */

	v4l2_dev->mdev = media_dev;

	ret = v4l2_device_register(dev, v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register v4l2 device: %d\n", ret);
		goto error_media;
	}

	csi_dev->v4l2_dev = v4l2_dev;
	csi_dev->media_dev = media_dev;

	return 0;

error_media:
	media_device_unregister(media_dev);
	media_device_cleanup(media_dev);

	return ret;
}

static void sun6i_csi_v4l2_cleanup(struct sun6i_csi_device *csi_dev)
{
	struct sun6i_csi_v4l2 *v4l2 = &csi_dev->v4l2;

	media_device_unregister(&v4l2->media_dev);
	v4l2_device_unregister(&v4l2->v4l2_dev);
	media_device_cleanup(&v4l2->media_dev);
}

/* Debugfs */

/*
 * Unlike the ISP the CSI has no load and save buffers: a register holds what
 * the hardware is running, so one column says everything.
 *
 * This exists rather than regmap's own debugfs dump because that one is not
 * guarded against runtime PM, and reading the block while it is held in reset
 * takes the bus down.
 */
static const struct {
	const char	*name;
	unsigned int	offset;
} sun6i_csi_debugfs_registers[] = {
	{ "en",			SUN6I_CSI_EN_REG },
	{ "if_cfg",		SUN6I_CSI_IF_CFG_REG },
	{ "cap",		SUN6I_CSI_CAP_REG },
	{ "sync_cnt",		SUN6I_CSI_SYNC_CNT_REG },
	{ "fifo_thrs",		SUN6I_CSI_FIFO_THRS_REG },
	{ "bt656_head_cfg",	SUN6I_CSI_BT656_HEAD_CFG_REG },
	{ "ptn_len",		SUN6I_CSI_PTN_LEN_REG },
	{ "ptn_addr",		SUN6I_CSI_PTN_ADDR_REG },
	{ "ver",		SUN6I_CSI_VER_REG },
	{ "ch_cfg",		SUN6I_CSI_CH_CFG_REG },
	{ "ch_scale",		SUN6I_CSI_CH_SCALE_REG },
	{ "ch_fifo0_addr",	SUN6I_CSI_CH_FIFO0_ADDR_REG },
	{ "ch_fifo1_addr",	SUN6I_CSI_CH_FIFO1_ADDR_REG },
	{ "ch_fifo2_addr",	SUN6I_CSI_CH_FIFO2_ADDR_REG },
	{ "ch_sta",		SUN6I_CSI_CH_STA_REG },
	{ "ch_int_en",		SUN6I_CSI_CH_INT_EN_REG },
	{ "ch_int_sta",		SUN6I_CSI_CH_INT_STA_REG },
	{ "ch_fld1_vsize",	SUN6I_CSI_CH_FLD1_VSIZE_REG },
	{ "ch_hsize",		SUN6I_CSI_CH_HSIZE_REG },
	{ "ch_vsize",		SUN6I_CSI_CH_VSIZE_REG },
	{ "ch_buf_len",		SUN6I_CSI_CH_BUF_LEN_REG },
	{ "ch_flip_size",	SUN6I_CSI_CH_FLIP_SIZE_REG },
	{ "ch_frm_clk_cnt",	SUN6I_CSI_CH_FRM_CLK_CNT_REG },
	{ "ch_acc_itnl_clk_cnt", SUN6I_CSI_CH_ACC_ITNL_CLK_CNT_REG },
	{ "ch_fifo_stat",	SUN6I_CSI_CH_FIFO_STAT_REG },
	{ "ch_pclk_stat",	SUN6I_CSI_CH_PCLK_STAT_REG },
};

static int sun6i_csi_debugfs_registers_show(struct seq_file *seq, void *data)
{
	struct sun6i_csi_device *csi_dev = seq->private;
	unsigned int i;

	if (pm_runtime_get_if_in_use(csi_dev->dev) <= 0) {
		seq_puts(seq, "(device suspended)\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(sun6i_csi_debugfs_registers); i++) {
		u32 value = 0;

		regmap_read(csi_dev->regmap,
			    sun6i_csi_debugfs_registers[i].offset, &value);

		seq_printf(seq, "%-16s %08x\n",
			   sun6i_csi_debugfs_registers[i].name, value);
	}

	pm_runtime_put(csi_dev->dev);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sun6i_csi_debugfs_registers);

/*
 * Write a register directly: "offset value", both in any base strtoul takes.
 *
 * Several fields of this block were never identified -- the pattern generator
 * in particular has no driver support at all -- and finding out what one does
 * is otherwise a kernel build per bit.
 */
static ssize_t sun6i_csi_debugfs_poke_write(struct file *file,
					    const char __user *buffer,
					    size_t count, loff_t *ppos)
{
	struct sun6i_csi_device *csi_dev = file->private_data;
	unsigned int offset, value;
	char buf[64];

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	buf[count] = '\0';

	/*
	 * sscanf and not simple_strtoul: the latter does not skip leading
	 * whitespace the way its userspace namesake does, so parsing the value
	 * from just after the offset silently yields zero and every poke writes
	 * zero instead of what was asked for.
	 */
	if (sscanf(buf, "%i %i", &offset, &value) != 2)
		return -EINVAL;

	if (offset > SUN6I_CSI_CH_PCLK_STAT_REG || (offset & 3))
		return -EINVAL;

	if (pm_runtime_get_if_in_use(csi_dev->dev) <= 0)
		return -EAGAIN;

	regmap_write(csi_dev->regmap, offset, value);
	pm_runtime_put(csi_dev->dev);

	return count;
}

static const struct file_operations sun6i_csi_debugfs_poke_fops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= sun6i_csi_debugfs_poke_write,
	.llseek		= noop_llseek,
};

/* Pattern */

/*
 * A cap on what a single debugfs write can ask the CMA pool for. A full
 * 2592x1944 frame at two bytes a pixel is ten megabytes, so this leaves room
 * without letting a typo try to allocate the machine.
 */
#define SUN6I_CSI_PATTERN_SIZE_MAX	SZ_32M

bool sun6i_csi_pattern_replaces_source(struct sun6i_csi_device *csi_dev)
{
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	bool replaces;

	mutex_lock(&pattern->lock);
	replaces = pattern->enabled && !pattern->keep_source;
	mutex_unlock(&pattern->lock);

	return replaces;
}

/* Called with the pattern lock held and a runtime PM reference taken. */
static int sun6i_csi_pattern_arm(struct sun6i_csi_device *csi_dev)
{
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	struct regmap *regmap = csi_dev->regmap;
	u32 len = pattern->len;
	int ret;

	if (!len) {
		ret = sun6i_csi_bridge_frame_bytes(csi_dev, &len);
		if (ret)
			return ret;
	}

	if (len > pattern->size)
		return -EINVAL;

	/*
	 * Clearing CSI_EN resets the module, so it has to come out of reset
	 * before any of the below would stick. The bridge sets this bit too
	 * when it streams, and clears it when it stops -- so a pipeline that
	 * has run and stopped leaves the generator needing to be armed again.
	 */
	regmap_update_bits(regmap, SUN6I_CSI_EN_REG, SUN6I_CSI_EN_CSI_EN,
			   SUN6I_CSI_EN_CSI_EN);

	regmap_write(regmap, SUN6I_CSI_PTN_LEN_REG, len);

	/*
	 * Every other address register in this block holds the address shifted
	 * down by two and the manual describes those in the same words as this
	 * one, so that is the encoding to try first. If the generator turns out
	 * to want the address unshifted, poke it.
	 */
	regmap_write(regmap, SUN6I_CSI_PTN_ADDR_REG,
		     SUN6I_CSI_ADDR_VALUE(pattern->address));

	regmap_update_bits(regmap, SUN6I_CSI_FIFO_THRS_REG,
			   SUN6I_CSI_FIFO_THRS_PTN_GEN_CLK_DIV_MASK |
			   SUN6I_CSI_FIFO_THRS_PTN_GEN_DLY_MASK,
			   SUN6I_CSI_FIFO_THRS_PTN_GEN_CLK_DIV(pattern->clk_div) |
			   SUN6I_CSI_FIFO_THRS_PTN_GEN_DLY(pattern->dly));

	/*
	 * The generator reads through the CSI's SRAM, so make sure nothing has
	 * left it powered down.
	 */
	regmap_update_bits(regmap, SUN6I_CSI_EN_REG,
			   SUN6I_CSI_EN_PTN_CYCLE_MASK |
			   SUN6I_CSI_EN_SRAM_PWDN | SUN6I_CSI_EN_PTN_GEN_EN,
			   SUN6I_CSI_EN_PTN_CYCLE(pattern->cycle) |
			   SUN6I_CSI_EN_PTN_GEN_EN);

	return 0;
}

static int sun6i_csi_pattern_enable_get(void *data, u64 *value)
{
	struct sun6i_csi_device *csi_dev = data;

	*value = csi_dev->pattern.enabled;

	return 0;
}

/*
 * Enabling arms the generator and holds a runtime PM reference for as long as
 * it stays armed. Without that the device suspends whenever nothing streams,
 * and no register here could be read or written at all -- which would make the
 * first and most important experiment, whether the block does anything, need a
 * whole capture pipeline to perform.
 */
static int sun6i_csi_pattern_enable_set(void *data, u64 value)
{
	struct sun6i_csi_device *csi_dev = data;
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	struct device *dev = csi_dev->dev;
	int ret = 0;

	mutex_lock(&pattern->lock);

	if (!!value == pattern->enabled)
		goto complete;

	if (!value) {
		regmap_update_bits(csi_dev->regmap, SUN6I_CSI_EN_REG,
				   SUN6I_CSI_EN_PTN_GEN_EN, 0);

		pattern->enabled = false;
		pm_runtime_put(dev);

		goto complete;
	}

	if (!pattern->buffer) {
		ret = -ENODEV;
		goto complete;
	}

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto complete;

	ret = sun6i_csi_pattern_arm(csi_dev);
	if (ret) {
		pm_runtime_put(dev);
		goto complete;
	}

	pattern->enabled = true;

complete:
	mutex_unlock(&pattern->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(sun6i_csi_pattern_enable_fops,
			 sun6i_csi_pattern_enable_get,
			 sun6i_csi_pattern_enable_set, "%llu\n");

/*
 * Kicking the generator. The hardware clears this bit by itself when the
 * pattern has been generated, so reading the enable register back afterwards
 * is what says whether anything happened.
 */
static int sun6i_csi_pattern_start_set(void *data, u64 value)
{
	struct sun6i_csi_device *csi_dev = data;
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	int ret = 0;

	mutex_lock(&pattern->lock);

	if (pattern->enabled)
		regmap_update_bits(csi_dev->regmap, SUN6I_CSI_EN_REG,
				   SUN6I_CSI_EN_PTN_START,
				   value ? SUN6I_CSI_EN_PTN_START : 0);
	else
		ret = -EPERM;

	mutex_unlock(&pattern->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(sun6i_csi_pattern_start_fops, NULL,
			 sun6i_csi_pattern_start_set, "%llu\n");

static int sun6i_csi_pattern_size_get(void *data, u64 *value)
{
	struct sun6i_csi_device *csi_dev = data;

	*value = csi_dev->pattern.size;

	return 0;
}

static int sun6i_csi_pattern_size_set(void *data, u64 value)
{
	struct sun6i_csi_device *csi_dev = data;
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	struct device *dev = csi_dev->dev;
	int ret = 0;

	if (value > SUN6I_CSI_PATTERN_SIZE_MAX)
		return -EINVAL;

	mutex_lock(&pattern->lock);

	if (pattern->enabled) {
		ret = -EBUSY;
		goto complete;
	}

	if (pattern->buffer) {
		dma_free_coherent(dev, pattern->size, pattern->buffer,
				  pattern->address);

		pattern->buffer = NULL;
		pattern->size = 0;
	}

	if (value) {
		pattern->buffer = dma_alloc_coherent(dev, value,
						     &pattern->address,
						     GFP_KERNEL);
		if (!pattern->buffer) {
			ret = -ENOMEM;
			goto complete;
		}

		pattern->size = value;
	}

complete:
	mutex_unlock(&pattern->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(sun6i_csi_pattern_size_fops,
			 sun6i_csi_pattern_size_get,
			 sun6i_csi_pattern_size_set, "%llu\n");

/* Both encodings, because which one the generator wants is not known. */
static int sun6i_csi_pattern_address_show(struct seq_file *seq, void *data)
{
	struct sun6i_csi_device *csi_dev = seq->private;
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;

	mutex_lock(&pattern->lock);

	if (pattern->buffer) {
		seq_printf(seq, "address %pad\n", &pattern->address);
		seq_printf(seq, "shifted %08x\n",
			   (u32)SUN6I_CSI_ADDR_VALUE(pattern->address));
	}

	mutex_unlock(&pattern->lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sun6i_csi_pattern_address);

static ssize_t sun6i_csi_pattern_data_read(struct file *file,
					   char __user *buffer, size_t count,
					   loff_t *ppos)
{
	struct sun6i_csi_device *csi_dev = file->private_data;
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	ssize_t ret;

	mutex_lock(&pattern->lock);

	ret = pattern->buffer ?
	      simple_read_from_buffer(buffer, count, ppos, pattern->buffer,
				      pattern->size) : -ENODEV;

	mutex_unlock(&pattern->lock);

	return ret;
}

static ssize_t sun6i_csi_pattern_data_write(struct file *file,
					    const char __user *buffer,
					    size_t count, loff_t *ppos)
{
	struct sun6i_csi_device *csi_dev = file->private_data;
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	ssize_t ret;

	mutex_lock(&pattern->lock);

	ret = pattern->buffer ?
	      simple_write_to_buffer(pattern->buffer, pattern->size, ppos,
				     buffer, count) : -ENODEV;

	mutex_unlock(&pattern->lock);

	return ret;
}

static const struct file_operations sun6i_csi_pattern_data_fops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= sun6i_csi_pattern_data_read,
	.write		= sun6i_csi_pattern_data_write,
	.llseek		= default_llseek,
};

/* Pattern sub-device */

#define SUN6I_CSI_PATTERN_NAME		"sun6i-csi-pattern"

static inline struct sun6i_csi_device *
sun6i_csi_pattern_dev(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct sun6i_csi_device, pattern.subdev);
}

/*
 * The bus width is the one thing the format has to reach the bridge through,
 * because the generator is not on any wires and so has no device tree
 * endpoint saying how wide they are.
 */
static unsigned int sun6i_csi_pattern_bus_width(u32 mbus_code)
{
	switch (mbus_code) {
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return 10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return 12;
	default:
		return 8;
	}
}

static int sun6i_csi_pattern_s_stream(struct v4l2_subdev *subdev, int on)
{
	struct sun6i_csi_device *csi_dev = sun6i_csi_pattern_dev(subdev);
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(subdev);
	csi_dev->bridge.source_pattern.endpoint.bus.parallel.bus_width =
		sun6i_csi_pattern_bus_width(v4l2_subdev_state_get_format(state, 0)->code);
	v4l2_subdev_unlock_state(state);

	mutex_lock(&pattern->lock);

	if (!on) {
		regmap_update_bits(csi_dev->regmap, SUN6I_CSI_EN_REG,
				   SUN6I_CSI_EN_PTN_GEN_EN, 0);
		ret = 0;
		goto complete;
	}

	if (!pattern->buffer) {
		dev_err(csi_dev->dev, "no pattern buffer allocated\n");
		ret = -ENODEV;
		goto complete;
	}

	ret = sun6i_csi_pattern_arm(csi_dev);
	if (ret)
		goto complete;

	/*
	 * The generator stops after PTN_CYCLE+1 passes and the bit clears
	 * itself, so a stream longer than that needs re-kicking. Nothing here
	 * runs per frame, so start it at the maximum and let whoever wants
	 * more frames than that kick it again through debugfs.
	 */
	regmap_update_bits(csi_dev->regmap, SUN6I_CSI_EN_REG,
			   SUN6I_CSI_EN_PTN_START, SUN6I_CSI_EN_PTN_START);

complete:
	mutex_unlock(&pattern->lock);

	return ret;
}

static const struct v4l2_subdev_video_ops sun6i_csi_pattern_video_ops = {
	.s_stream	= sun6i_csi_pattern_s_stream,
};

static int
sun6i_csi_pattern_enum_mbus_code(struct v4l2_subdev *subdev,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code_enum)
{
	const struct sun6i_csi_bridge_format *format =
		sun6i_csi_bridge_format_by_index(code_enum->index);

	if (!format)
		return -EINVAL;

	code_enum->code = format->mbus_code;

	return 0;
}

/*
 * Anything the bridge can accept, at any size it can carry: the whole point
 * of the generator is that the input is not limited by what some sensor
 * happens to offer.
 */
static int sun6i_csi_pattern_set_fmt(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mbus_format =
		v4l2_subdev_state_get_format(state, 0);

	if (!sun6i_csi_bridge_format_find(format->format.code))
		mbus_format->code =
			sun6i_csi_bridge_format_by_index(0)->mbus_code;
	else
		mbus_format->code = format->format.code;

	mbus_format->width = clamp_t(u32, format->format.width,
				     SUN6I_CSI_CAPTURE_WIDTH_MIN,
				     SUN6I_CSI_CAPTURE_WIDTH_MAX);
	mbus_format->height = clamp_t(u32, format->format.height,
				      SUN6I_CSI_CAPTURE_HEIGHT_MIN,
				      SUN6I_CSI_CAPTURE_HEIGHT_MAX);

	mbus_format->field = V4L2_FIELD_NONE;
	mbus_format->colorspace = V4L2_COLORSPACE_RAW;
	mbus_format->quantization = V4L2_QUANTIZATION_DEFAULT;
	mbus_format->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	format->format = *mbus_format;

	return 0;
}

static int sun6i_csi_pattern_init_state(struct v4l2_subdev *subdev,
					struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *mbus_format =
		v4l2_subdev_state_get_format(state, 0);

	mbus_format->code = sun6i_csi_bridge_format_by_index(0)->mbus_code;
	mbus_format->width = 1280;
	mbus_format->height = 720;
	mbus_format->field = V4L2_FIELD_NONE;
	mbus_format->colorspace = V4L2_COLORSPACE_RAW;

	return 0;
}

static const struct v4l2_subdev_pad_ops sun6i_csi_pattern_pad_ops = {
	.enum_mbus_code	= sun6i_csi_pattern_enum_mbus_code,
	.get_fmt	= v4l2_subdev_get_fmt,
	.set_fmt	= sun6i_csi_pattern_set_fmt,
};

static const struct v4l2_subdev_ops sun6i_csi_pattern_subdev_ops = {
	.video	= &sun6i_csi_pattern_video_ops,
	.pad	= &sun6i_csi_pattern_pad_ops,
};

static const struct v4l2_subdev_internal_ops sun6i_csi_pattern_internal_ops = {
	.init_state	= sun6i_csi_pattern_init_state,
};

static const struct media_entity_operations sun6i_csi_pattern_entity_ops = {
	.link_validate	= v4l2_subdev_link_validate,
};

/*
 * Registered once the v4l2 device is known, which with an ISP attached is not
 * until the bridge's notifier binds something -- the same point the capture
 * device is set up.
 */
static int sun6i_csi_pattern_complete(struct sun6i_csi_device *csi_dev)
{
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	struct v4l2_subdev *subdev = &pattern->subdev;
	struct device *dev = csi_dev->dev;
	int ret;

	if (pattern->registered)
		return 0;

	v4l2_subdev_init(subdev, &sun6i_csi_pattern_subdev_ops);
	subdev->internal_ops = &sun6i_csi_pattern_internal_ops;
	strscpy(subdev->name, SUN6I_CSI_PATTERN_NAME, sizeof(subdev->name));
	subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	subdev->owner = THIS_MODULE;
	subdev->dev = dev;

	v4l2_set_subdevdata(subdev, csi_dev);

	subdev->entity.function = MEDIA_ENT_F_VID_MUX;
	subdev->entity.ops = &sun6i_csi_pattern_entity_ops;

	pattern->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&subdev->entity, 1, &pattern->pad);
	if (ret < 0)
		return ret;

	ret = v4l2_subdev_init_finalize(subdev);
	if (ret)
		goto error_media_entity;

	ret = v4l2_device_register_subdev(csi_dev->v4l2_dev, subdev);
	if (ret) {
		dev_err(dev, "failed to register pattern subdev: %d\n", ret);
		goto error_subdev;
	}

	ret = media_create_pad_link(&subdev->entity, 0,
				    &csi_dev->bridge.subdev.entity,
				    SUN6I_CSI_BRIDGE_PAD_SINK, 0);
	if (ret < 0) {
		dev_err(dev, "failed to link the pattern to the bridge\n");
		goto error_register;
	}

	/*
	 * Not on any wires, so there is no endpoint in the device tree to
	 * describe it. These are the polarities the bridge was measured
	 * working with, and the width follows the format at stream time.
	 */
	csi_dev->bridge.source_pattern.subdev = subdev;
	csi_dev->bridge.source_pattern.endpoint.bus_type = V4L2_MBUS_PARALLEL;
	csi_dev->bridge.source_pattern.endpoint.bus.parallel.bus_width = 8;
	csi_dev->bridge.source_pattern.endpoint.bus.parallel.flags =
		V4L2_MBUS_HSYNC_ACTIVE_LOW | V4L2_MBUS_VSYNC_ACTIVE_HIGH |
		V4L2_MBUS_PCLK_SAMPLE_FALLING;

	pattern->registered = true;

	return 0;

error_register:
	v4l2_device_unregister_subdev(subdev);

error_subdev:
	v4l2_subdev_cleanup(subdev);

error_media_entity:
	media_entity_cleanup(&subdev->entity);

	return ret;
}

static void sun6i_csi_pattern_setup(struct sun6i_csi_device *csi_dev)
{
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;
	struct dentry *debugfs;

	mutex_init(&pattern->lock);

	/* The reset values of the two fields that share the FIFO register. */
	pattern->clk_div = 0;
	pattern->dly = 0x0f;

	debugfs = debugfs_create_dir("pattern", csi_dev->debugfs);

	debugfs_create_file_unsafe("size", 0644, debugfs, csi_dev,
				   &sun6i_csi_pattern_size_fops);
	debugfs_create_file("address", 0444, debugfs, csi_dev,
			    &sun6i_csi_pattern_address_fops);
	debugfs_create_file("data", 0644, debugfs, csi_dev,
			    &sun6i_csi_pattern_data_fops);
	debugfs_create_file_unsafe("enable", 0644, debugfs, csi_dev,
				   &sun6i_csi_pattern_enable_fops);
	debugfs_create_file_unsafe("start", 0200, debugfs, csi_dev,
				   &sun6i_csi_pattern_start_fops);

	/*
	 * Plain storage, applied when the generator is armed: a length of zero
	 * means one frame of the bridge's current format, and the two timing
	 * fields are only worth moving if the defaults turn out not to work.
	 */
	debugfs_create_u32("len", 0644, debugfs, &pattern->len);
	debugfs_create_u8("cycle", 0644, debugfs, &pattern->cycle);
	debugfs_create_u8("clk_div", 0644, debugfs, &pattern->clk_div);
	debugfs_create_u8("dly", 0644, debugfs, &pattern->dly);
	debugfs_create_bool("keep_source", 0644, debugfs,
			    &pattern->keep_source);
}

static void sun6i_csi_pattern_cleanup(struct sun6i_csi_device *csi_dev)
{
	struct sun6i_csi_pattern *pattern = &csi_dev->pattern;

	if (pattern->registered) {
		v4l2_device_unregister_subdev(&pattern->subdev);
		v4l2_subdev_cleanup(&pattern->subdev);
		media_entity_cleanup(&pattern->subdev.entity);
		pattern->registered = false;
	}

	/* Enabling took a runtime PM reference that has to go back. */
	if (pattern->enabled) {
		pattern->enabled = false;
		pm_runtime_put(csi_dev->dev);
	}

	if (pattern->buffer)
		dma_free_coherent(csi_dev->dev, pattern->size, pattern->buffer,
				  pattern->address);

	mutex_destroy(&pattern->lock);
}

static void sun6i_csi_debugfs_setup(struct sun6i_csi_device *csi_dev)
{
	/*
	 * Named for the device and not the driver: variants other than the
	 * A83T have two CSI instances, which would collide on one directory.
	 */
	csi_dev->debugfs = debugfs_create_dir(dev_name(csi_dev->dev), NULL);

	debugfs_create_file("registers", 0444, csi_dev->debugfs, csi_dev,
			    &sun6i_csi_debugfs_registers_fops);
	debugfs_create_file("poke", 0200, csi_dev->debugfs, csi_dev,
			    &sun6i_csi_debugfs_poke_fops);

	sun6i_csi_pattern_setup(csi_dev);
}

static void sun6i_csi_debugfs_cleanup(struct sun6i_csi_device *csi_dev)
{
	debugfs_remove_recursive(csi_dev->debugfs);
	csi_dev->debugfs = NULL;

	sun6i_csi_pattern_cleanup(csi_dev);
}

/* Platform */

static irqreturn_t sun6i_csi_interrupt(int irq, void *private)
{
	struct sun6i_csi_device *csi_dev = private;
	bool capture_streaming = csi_dev->capture.state.streaming;
	struct regmap *regmap = csi_dev->regmap;
	u32 status = 0, enable = 0;

	regmap_read(regmap, SUN6I_CSI_CH_INT_STA_REG, &status);
	regmap_read(regmap, SUN6I_CSI_CH_INT_EN_REG, &enable);

	if (!status)
		return IRQ_NONE;
	else if (!(status & enable) || !capture_streaming)
		goto complete;

	if ((status & SUN6I_CSI_CH_INT_STA_FIFO0_OF) ||
	    (status & SUN6I_CSI_CH_INT_STA_FIFO1_OF) ||
	    (status & SUN6I_CSI_CH_INT_STA_FIFO2_OF) ||
	    (status & SUN6I_CSI_CH_INT_STA_HB_OF)) {
		regmap_write(regmap, SUN6I_CSI_CH_INT_STA_REG, status);

		regmap_update_bits(regmap, SUN6I_CSI_EN_REG,
				   SUN6I_CSI_EN_CSI_EN, 0);
		regmap_update_bits(regmap, SUN6I_CSI_EN_REG,
				   SUN6I_CSI_EN_CSI_EN, SUN6I_CSI_EN_CSI_EN);
		return IRQ_HANDLED;
	}

	if (status & SUN6I_CSI_CH_INT_STA_FD)
		sun6i_csi_capture_frame_done(csi_dev);

	if (status & SUN6I_CSI_CH_INT_STA_VS)
		sun6i_csi_capture_sync(csi_dev);

complete:
	regmap_write(regmap, SUN6I_CSI_CH_INT_STA_REG, status);

	return IRQ_HANDLED;
}

static int sun6i_csi_suspend(struct device *dev)
{
	struct sun6i_csi_device *csi_dev = dev_get_drvdata(dev);

	reset_control_assert(csi_dev->reset);
	clk_disable_unprepare(csi_dev->clock_ram);
	clk_disable_unprepare(csi_dev->clock_mod);

	return 0;
}

static int sun6i_csi_resume(struct device *dev)
{
	struct sun6i_csi_device *csi_dev = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(csi_dev->reset);
	if (ret) {
		dev_err(dev, "failed to deassert reset\n");
		return ret;
	}

	ret = clk_prepare_enable(csi_dev->clock_mod);
	if (ret) {
		dev_err(dev, "failed to enable module clock\n");
		goto error_reset;
	}

	ret = clk_prepare_enable(csi_dev->clock_ram);
	if (ret) {
		dev_err(dev, "failed to enable ram clock\n");
		goto error_clock_mod;
	}

	return 0;

error_clock_mod:
	clk_disable_unprepare(csi_dev->clock_mod);

error_reset:
	reset_control_assert(csi_dev->reset);

	return ret;
}

static const struct dev_pm_ops sun6i_csi_pm_ops = {
	.runtime_suspend	= sun6i_csi_suspend,
	.runtime_resume		= sun6i_csi_resume,
};

static const struct regmap_config sun6i_csi_regmap_config = {
	.reg_bits       = 32,
	.reg_stride     = 4,
	.val_bits       = 32,
	.max_register	= 0x9c,
};

static int sun6i_csi_resources_setup(struct sun6i_csi_device *csi_dev,
				     struct platform_device *platform_dev)
{
	struct device *dev = csi_dev->dev;
	const struct sun6i_csi_variant *variant;
	void __iomem *io_base;
	int ret;
	int irq;

	variant = of_device_get_match_data(dev);
	if (!variant)
		return -EINVAL;

	/* Registers */

	io_base = devm_platform_ioremap_resource(platform_dev, 0);
	if (IS_ERR(io_base))
		return PTR_ERR(io_base);

	csi_dev->regmap = devm_regmap_init_mmio_clk(dev, "bus", io_base,
						    &sun6i_csi_regmap_config);
	if (IS_ERR(csi_dev->regmap)) {
		dev_err(dev, "failed to init register map\n");
		return PTR_ERR(csi_dev->regmap);
	}

	/* Clocks */

	csi_dev->clock_mod = devm_clk_get(dev, "mod");
	if (IS_ERR(csi_dev->clock_mod)) {
		dev_err(dev, "failed to acquire module clock\n");
		return PTR_ERR(csi_dev->clock_mod);
	}

	csi_dev->clock_ram = devm_clk_get(dev, "ram");
	if (IS_ERR(csi_dev->clock_ram)) {
		dev_err(dev, "failed to acquire ram clock\n");
		return PTR_ERR(csi_dev->clock_ram);
	}

	ret = clk_set_rate_exclusive(csi_dev->clock_mod,
				     variant->clock_mod_rate);
	if (ret) {
		dev_err(dev, "failed to set mod clock rate\n");
		return ret;
	}

	/* Reset */

	csi_dev->reset = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(csi_dev->reset)) {
		dev_err(dev, "failed to acquire reset\n");
		ret = PTR_ERR(csi_dev->reset);
		goto error_clock_rate_exclusive;
	}

	/* Interrupt */

	irq = platform_get_irq(platform_dev, 0);
	if (irq < 0) {
		ret = -ENXIO;
		goto error_clock_rate_exclusive;
	}

	ret = devm_request_irq(dev, irq, sun6i_csi_interrupt, IRQF_SHARED,
			       SUN6I_CSI_NAME, csi_dev);
	if (ret) {
		dev_err(dev, "failed to request interrupt\n");
		goto error_clock_rate_exclusive;
	}

	/* Runtime PM */

	pm_runtime_enable(dev);

	return 0;

error_clock_rate_exclusive:
	clk_rate_exclusive_put(csi_dev->clock_mod);

	return ret;
}

static void sun6i_csi_resources_cleanup(struct sun6i_csi_device *csi_dev)
{
	pm_runtime_disable(csi_dev->dev);
	clk_rate_exclusive_put(csi_dev->clock_mod);
}

static int sun6i_csi_probe(struct platform_device *platform_dev)
{
	struct sun6i_csi_device *csi_dev;
	struct device *dev = &platform_dev->dev;
	int ret;

	csi_dev = devm_kzalloc(dev, sizeof(*csi_dev), GFP_KERNEL);
	if (!csi_dev)
		return -ENOMEM;

	csi_dev->dev = &platform_dev->dev;
	platform_set_drvdata(platform_dev, csi_dev);

	ret = sun6i_csi_resources_setup(csi_dev, platform_dev);
	if (ret)
		return ret;

	ret = sun6i_csi_isp_detect(csi_dev);
	if (ret)
		goto error_resources;

	sun6i_csi_debugfs_setup(csi_dev);

	/*
	 * Register our own v4l2 and media devices when there is no ISP around.
	 * Otherwise the ISP will use async subdev registration with our bridge,
	 * which will provide v4l2 and media devices that are used to register
	 * the video interface.
	 */
	if (!csi_dev->isp_available) {
		ret = sun6i_csi_v4l2_setup(csi_dev);
		if (ret)
			goto error_debugfs;
	}

	ret = sun6i_csi_bridge_setup(csi_dev);
	if (ret)
		goto error_v4l2;

	if (!csi_dev->isp_available) {
		ret = sun6i_csi_capture_setup(csi_dev);
		if (ret)
			goto error_bridge;

		ret = sun6i_csi_pattern_complete(csi_dev);
		if (ret)
			goto error_bridge;
	}

	return 0;

error_bridge:
	sun6i_csi_bridge_cleanup(csi_dev);

error_v4l2:
	if (!csi_dev->isp_available)
		sun6i_csi_v4l2_cleanup(csi_dev);

error_debugfs:
	sun6i_csi_debugfs_cleanup(csi_dev);

error_resources:
	sun6i_csi_resources_cleanup(csi_dev);

	return ret;
}

static void sun6i_csi_remove(struct platform_device *pdev)
{
	struct sun6i_csi_device *csi_dev = platform_get_drvdata(pdev);

	sun6i_csi_capture_cleanup(csi_dev);
	sun6i_csi_bridge_cleanup(csi_dev);

	if (!csi_dev->isp_available)
		sun6i_csi_v4l2_cleanup(csi_dev);

	sun6i_csi_debugfs_cleanup(csi_dev);
	sun6i_csi_resources_cleanup(csi_dev);
}

static const struct sun6i_csi_variant sun6i_a31_csi_variant = {
	.clock_mod_rate	= 297000000,
};

static const struct sun6i_csi_variant sun50i_a64_csi_variant = {
	.clock_mod_rate	= 300000000,
};

static const struct of_device_id sun6i_csi_of_match[] = {
	{
		.compatible	= "allwinner,sun6i-a31-csi",
		.data		= &sun6i_a31_csi_variant,
	},
	{
		.compatible	= "allwinner,sun8i-a83t-csi",
		.data		= &sun6i_a31_csi_variant,
	},
	{
		.compatible	= "allwinner,sun8i-h3-csi",
		.data		= &sun6i_a31_csi_variant,
	},
	{
		.compatible	= "allwinner,sun8i-v3s-csi",
		.data		= &sun6i_a31_csi_variant,
	},
	{
		.compatible	= "allwinner,sun50i-a64-csi",
		.data		= &sun50i_a64_csi_variant,
	},
	{},
};

MODULE_DEVICE_TABLE(of, sun6i_csi_of_match);

static struct platform_driver sun6i_csi_platform_driver = {
	.probe	= sun6i_csi_probe,
	.remove = sun6i_csi_remove,
	.driver	= {
		.name		= SUN6I_CSI_NAME,
		.of_match_table	= sun6i_csi_of_match,
		.pm		= &sun6i_csi_pm_ops,
	},
};

module_platform_driver(sun6i_csi_platform_driver);

MODULE_DESCRIPTION("Allwinner A31 Camera Sensor Interface driver");
MODULE_AUTHOR("Yong Deng <yong.deng@magewell.com>");
MODULE_AUTHOR("Paul Kocialkowski <paul.kocialkowski@bootlin.com>");
MODULE_LICENSE("GPL");
