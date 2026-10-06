// SPDX-License-Identifier: GPL-2.0+
/*
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

#include "sun6i_isp.h"
#include "sun6i_isp_capture.h"
#include "sun6i_isp_params.h"
#include "sun6i_isp_proc.h"
#include "sun6i_isp_reg.h"

/* Helpers */

u32 sun6i_isp_load_read(struct sun6i_isp_device *isp_dev, u32 offset)
{
	u32 *data = (u32 *)(isp_dev->tables.load.data + offset);

	return *data;
}

/*
 * The counterpart of the load buffer: the hardware writes the register file
 * it is running back here at each sync, so this is what it took, where the
 * load buffer is only what it was offered.
 */
u32 sun6i_isp_save_read(struct sun6i_isp_device *isp_dev, u32 offset)
{
	u32 *data = (u32 *)(isp_dev->tables.save.data + offset);

	return *data;
}

void sun6i_isp_load_write(struct sun6i_isp_device *isp_dev, u32 offset,
			  u32 value)
{
	u32 *data = (u32 *)(isp_dev->tables.load.data + offset);

	*data = value;
}

/* State */

/*
 * The ISP works with a load buffer, which gets copied to the actual registers
 * by the hardware before processing a frame when a specific flag is set.
 * This is represented by tracking the ISP state in the different parts of
 * the code with explicit sync points:
 * - state update: to update the load buffer for the next frame if necessary;
 * - state complete: to indicate that the state update was applied.
 */

/*
 * The request bits are edges, not levels. Nothing in the hardware clears
 * them once a request has been served, so a driver that only ever sets
 * them leaves them high and the next request never becomes a transition.
 * Clear them here, against the value about to be written, so that setting
 * them below is always a rising edge.
 */
static void sun6i_isp_state_arm(struct sun6i_isp_device *isp_dev)
{
	struct regmap *regmap = isp_dev->regmap;
	u32 value;

	regmap_read(regmap, SUN6I_ISP_FE_CTRL_REG, &value);
	value &= ~(SUN6I_ISP_FE_CTRL_PARA_READY |
		   SUN6I_ISP_FE_CTRL_LUT_UPDATE |
		   SUN6I_ISP_FE_CTRL_LENS_UPDATE |
		   SUN6I_ISP_FE_CTRL_GAMMA_UPDATE |
		   SUN6I_ISP_FE_CTRL_DRC_UPDATE |
		   SUN6I_ISP_FE_CTRL_DISC_UPDATE);
	regmap_write(regmap, SUN6I_ISP_FE_CTRL_REG, value);
}

static void sun6i_isp_state_ready(struct sun6i_isp_device *isp_dev)
{
	struct regmap *regmap = isp_dev->regmap;
	u32 value;

	sun6i_isp_state_arm(isp_dev);

	regmap_read(regmap, SUN6I_ISP_FE_CTRL_REG, &value);
	value |= SUN6I_ISP_FE_CTRL_PARA_READY;
	value |= sun6i_isp_params_table_update_take(isp_dev);
	regmap_write(regmap, SUN6I_ISP_FE_CTRL_REG, value);
}

static void sun6i_isp_state_complete(struct sun6i_isp_device *isp_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&isp_dev->state_lock, flags);

	sun6i_isp_capture_state_complete(isp_dev);
	sun6i_isp_params_state_complete(isp_dev);

	spin_unlock_irqrestore(&isp_dev->state_lock, flags);
}

void sun6i_isp_state_update(struct sun6i_isp_device *isp_dev, bool ready_hold)
{
	bool update = false;
	unsigned long flags;

	spin_lock_irqsave(&isp_dev->state_lock, flags);

	sun6i_isp_capture_state_update(isp_dev, &update);
	sun6i_isp_params_state_update(isp_dev, &update);

	if (update && !ready_hold)
		sun6i_isp_state_ready(isp_dev);

	spin_unlock_irqrestore(&isp_dev->state_lock, flags);
}

/* Pipeline */

/*
 * Program everything the channels that are streaming now describe, and start
 * the pipeline against it.
 *
 * The load buffer is filled first and handed over as a whole by the PARA_READY
 * that sun6i_isp_proc_start() sets, so the addresses and the configuration they
 * belong to reach the hardware together. state_update() therefore stages a
 * buffer for each channel but is told to leave PARA_READY alone.
 */
int sun6i_isp_pipeline_start(struct sun6i_isp_device *isp_dev)
{
	sun6i_isp_params_configure(isp_dev);
	sun6i_isp_proc_configure(isp_dev);
	sun6i_isp_capture_configure(isp_dev);

	sun6i_isp_state_update(isp_dev, true);

	return sun6i_isp_proc_start(isp_dev);
}

/*
 * Apply a change to the set of streaming channels.
 *
 * Which register block each channel drives, and the size, format and addresses
 * in it, are only honoured from the frontend start that latched them - see
 * sun6i_isp_proc_start(). So a channel joining or leaving is applied by taking
 * the pipeline down and bringing it back up around the new set.
 *
 * The buffers in flight go back to the head of their own queues first, so that
 * the frame the hardware was part way through is written again from the top
 * rather than handed to userspace torn.
 */
int sun6i_isp_pipeline_restart(struct sun6i_isp_device *isp_dev)
{
	sun6i_isp_proc_stop(isp_dev);

	sun6i_isp_capture_rewind(isp_dev);

	return sun6i_isp_pipeline_start(isp_dev);
}

/* Tables */

static int sun6i_isp_table_setup(struct sun6i_isp_device *isp_dev,
				 struct sun6i_isp_table *table)
{
	table->data = dma_alloc_coherent(isp_dev->dev, table->size,
					 &table->address, GFP_KERNEL);
	if (!table->data)
		return -ENOMEM;

	return 0;
}

static void sun6i_isp_table_cleanup(struct sun6i_isp_device *isp_dev,
				    struct sun6i_isp_table *table)
{
	dma_free_coherent(isp_dev->dev, table->size, table->data,
			  table->address);
}

void sun6i_isp_tables_configure(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_tables *tables = &isp_dev->tables;
	struct regmap *regmap = isp_dev->regmap;

	regmap_write(regmap, SUN6I_ISP_REG_LOAD_ADDR_REG,
		     SUN6I_ISP_ADDR_VALUE(isp_dev->tables.load.address));

	regmap_write(regmap, SUN6I_ISP_REG_SAVE_ADDR_REG,
		     SUN6I_ISP_ADDR_VALUE(isp_dev->tables.save.address));

	regmap_write(regmap, SUN6I_ISP_LUT_TABLE_ADDR_REG,
		     SUN6I_ISP_ADDR_VALUE(isp_dev->tables.lut.address));

	regmap_write(regmap, SUN6I_ISP_DRC_TABLE_ADDR_REG,
		     SUN6I_ISP_ADDR_VALUE(isp_dev->tables.drc.address));

	regmap_write(regmap, SUN6I_ISP_STATS_ADDR_REG,
		     SUN6I_ISP_ADDR_VALUE(tables->stats[tables->stats_index].address));
}

static int sun6i_isp_tables_setup(struct sun6i_isp_device *isp_dev,
				  const struct sun6i_isp_variant *variant)
{
	struct sun6i_isp_tables *tables = &isp_dev->tables;
	unsigned int i;
	int ret;

	tables->load.size = variant->table_load_save_size;
	ret = sun6i_isp_table_setup(isp_dev, &tables->load);
	if (ret)
		return ret;

	tables->save.size = variant->table_load_save_size;
	ret = sun6i_isp_table_setup(isp_dev, &tables->save);
	if (ret)
		return ret;

	tables->lut.size = variant->table_lut_size;
	ret = sun6i_isp_table_setup(isp_dev, &tables->lut);
	if (ret)
		return ret;

	tables->drc.size = variant->table_drc_size;
	ret = sun6i_isp_table_setup(isp_dev, &tables->drc);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(tables->stats); i++) {
		tables->stats[i].size = variant->table_stats_size;
		ret = sun6i_isp_table_setup(isp_dev, &tables->stats[i]);
		if (ret)
			return ret;
	}

	tables->stats_index = 0;
	tables->stats_done = -1;

	return 0;
}

static void sun6i_isp_tables_cleanup(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_tables *tables = &isp_dev->tables;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tables->stats); i++)
		sun6i_isp_table_cleanup(isp_dev, &tables->stats[i]);
	sun6i_isp_table_cleanup(isp_dev, &tables->drc);
	sun6i_isp_table_cleanup(isp_dev, &tables->lut);
	sun6i_isp_table_cleanup(isp_dev, &tables->save);
	sun6i_isp_table_cleanup(isp_dev, &tables->load);
}

/* Media */

static const struct media_device_ops sun6i_isp_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

/* V4L2 */

/* Debugfs */

/*
 * The hardware keeps its own account of itself: at every parameter sync it
 * writes the register file it is actually running to the save region, the
 * same layout as the load buffer the driver stages into. Reading the two
 * side by side is the only way to tell a register the driver got wrong
 * from one the hardware declined to take, which is a distinction guesswork
 * cannot make and which the whole of this block's behaviour turns on.
 */
static const struct {
	const char	*name;
	unsigned int	offset;
} sun6i_isp_debugfs_registers[] = {
	{ "module_en",	SUN6I_ISP_MODULE_EN_REG },
	{ "mode",	SUN6I_ISP_MODE_REG },
	{ "in_cfg",	SUN6I_ISP_IN_CFG_REG },
	{ "ob_size",	SUN6I_ISP_OB_SIZE_REG },
	{ "ob_valid",	SUN6I_ISP_OB_VALID_REG },
	{ "bayer_gain0", SUN6I_ISP_BAYER_GAIN0_REG },
	{ "bayer_gain1", SUN6I_ISP_BAYER_GAIN1_REG },
	{ "wb_gain0",	SUN6I_ISP_WB_GAIN0_REG },
	{ "wb_gain1",	SUN6I_ISP_WB_GAIN1_REG },
	{ "lsc_cfg",	SUN6I_ISP_LSC_CFG_REG },
	{ "cfa_cfg0",	SUN6I_ISP_CFA_CFG0_REG },
	{ "cfa_cfg1",	SUN6I_ISP_CFA_CFG1_REG },
	{ "sap_cfg",	SUN6I_ISP_SAP_CFG_REG },
	{ "contrast_cfg", SUN6I_ISP_CONTRAST_CFG_REG },
	{ "saturation_cfg", SUN6I_ISP_SATURATION_CFG_REG },
	{ "cnr",	SUN6I_ISP_CNR_REG },
	{ "mch_cfg",	SUN6I_ISP_MCH_CFG_REG },
	{ "sch_cfg",	SUN6I_ISP_SCH_CFG_REG },
};

static int sun6i_isp_debugfs_registers_show(struct seq_file *seq, void *data)
{
	struct sun6i_isp_device *isp_dev = seq->private;
	unsigned int i;
	bool live;
	u32 value;

	/*
	 * FE_CTRL is the one live register here; everything else is read out
	 * of the two DRAM buffers, which are just memory and always safe.
	 * Skip it rather than touch the bus while the device is suspended.
	 */
	if (pm_runtime_get_if_in_use(isp_dev->dev) > 0) {
		regmap_read(isp_dev->regmap, SUN6I_ISP_FE_CTRL_REG, &value);
		seq_printf(seq, "fe_ctrl %08x\n\n", value);
		pm_runtime_put(isp_dev->dev);
	} else {
		seq_puts(seq, "fe_ctrl (device suspended)\n\n");
	}

	/*
	 * Three views of the same register, and they answer different
	 * questions. "staged" is the load buffer, what the driver has
	 * offered. "running" is the save region, what the hardware wrote
	 * back at the last frame end -- which stays zero if no frame has ever
	 * completed, so it says nothing about a pipeline that has not run
	 * yet. "live" is the register itself, and it is the only one that
	 * shows whether a direct write to a normally staged register was
	 * kept or discarded.
	 */
	live = pm_runtime_get_if_in_use(isp_dev->dev) > 0;

	seq_printf(seq, "%-16s %8s %8s %8s\n", "register", "staged", "running",
		   live ? "live" : "");

	for (i = 0; i < ARRAY_SIZE(sun6i_isp_debugfs_registers); i++) {
		u32 offset = sun6i_isp_debugfs_registers[i].offset;

		seq_printf(seq, "%-16s %08x %08x",
			   sun6i_isp_debugfs_registers[i].name,
			   sun6i_isp_load_read(isp_dev, offset),
			   sun6i_isp_save_read(isp_dev, offset));

		if (live) {
			regmap_read(isp_dev->regmap, offset, &value);
			seq_printf(seq, " %08x", value);
		}

		seq_puts(seq, "\n");
	}

	if (live)
		pm_runtime_put(isp_dev->dev);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sun6i_isp_debugfs_registers);

/*
 * Write a frontend register directly: "offset value", both accepted in
 * any base strtoul takes.
 *
 * This exists because several fields of FE_CFG and FE_CTRL have no known
 * meaning -- the driver names six bits of one and nine of the other, and
 * the rest were never identified -- and finding out what they do a bit at
 * a time is otherwise a kernel build per bit.
 *
 * Any register in the block is allowed, not just the frontend ones below
 * 0x040. A value poked into a staged register survives only until the
 * next parameter load replaces it from the load region -- but the case
 * this is for is a pipeline that is not loading at all, where a poke is
 * the only way to change one.
 */
static ssize_t sun6i_isp_debugfs_poke_write(struct file *file,
					    const char __user *buffer,
					    size_t count, loff_t *ppos)
{
	struct sun6i_isp_device *isp_dev = file->private_data;
	unsigned int offset, value;
	char buf[64];

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	buf[count] = '\0';

	/*
	 * sscanf and not simple_strtoul: the latter does not skip leading
	 * whitespace the way its userspace namesake does, so parsing the
	 * value from just after the offset silently yields zero and every
	 * poke writes zero instead of what was asked for.
	 */
	if (sscanf(buf, "%i %i", &offset, &value) != 2)
		return -EINVAL;

	if (offset >= SZ_4K || (offset & 3))
		return -EINVAL;

	if (pm_runtime_get_if_in_use(isp_dev->dev) <= 0)
		return -EAGAIN;

	regmap_write(isp_dev->regmap, offset, value);
	pm_runtime_put(isp_dev->dev);

	return count;
}

/*
 * The same thing, into the load buffer instead of the register file.
 *
 * "poke" writes the live register, which the hardware overwrites from the
 * load region at the next parameter sync -- so for any register the
 * driver stages, a poke lasts at most one frame. That covers a pipeline
 * which is not loading at all, and nothing else.
 *
 * Two kinds of question need the other behaviour. A register the driver
 * writes once at stream start and never again, MODE being the one that
 * matters -- its sharpening mode select at bit 17 has never been anything
 * but 1. And the bits of a staged register that no config field reaches:
 * SAP_CFG [15:12] between level and min, which reads back as storage but
 * whose effect is unknown, and the same question for every block still to
 * be audited.
 *
 * Written here the value survives every sync until the driver itself
 * rewrites that register, which for an unnamed field is never.
 */
static ssize_t sun6i_isp_debugfs_load_poke_write(struct file *file,
						 const char __user *buffer,
						 size_t count, loff_t *ppos)
{
	struct sun6i_isp_device *isp_dev = file->private_data;
	unsigned int offset, value;
	char buf[64];

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	buf[count] = '\0';

	if (sscanf(buf, "%i %i", &offset, &value) != 2)
		return -EINVAL;

	if (offset >= SZ_4K || (offset & 3))
		return -EINVAL;

	/*
	 * No pm_runtime_get here: this is DRAM the driver owns, not the
	 * device, and it is readable and writable whether or not the
	 * hardware is powered.
	 */
	if (!isp_dev->tables.load.data)
		return -ENODEV;

	sun6i_isp_load_write(isp_dev, offset, value);

	return count;
}

static const struct file_operations sun6i_isp_debugfs_load_poke_fops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= sun6i_isp_debugfs_load_poke_write,
	.llseek		= noop_llseek,
};

static const struct file_operations sun6i_isp_debugfs_poke_fops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= sun6i_isp_debugfs_poke_write,
	.llseek		= noop_llseek,
};

/*
 * The whole register file, staged against running.
 *
 * The named dump above only shows what someone thought to name, which is
 * no use when the question is whether the hardware has written something
 * back that nobody expected -- a block that measures rather than acts
 * leaves its answer somewhere in here, and there is no way to find it by
 * guessing offsets one at a time.
 *
 * "staged" is the load buffer, what the driver offered. "running" is the
 * save region, which the hardware writes at every parameter sync, so a
 * difference between the columns is the hardware talking back.
 */
static int sun6i_isp_debugfs_raw_show(struct seq_file *seq, void *data)
{
	struct sun6i_isp_device *isp_dev = seq->private;
	unsigned int offset;

	seq_printf(seq, "%-8s %8s %8s\n", "offset", "staged", "running");

	for (offset = 0; offset < SZ_1K; offset += 4) {
		u32 load = sun6i_isp_load_read(isp_dev, offset);
		u32 save = sun6i_isp_save_read(isp_dev, offset);

		if (!load && !save)
			continue;

		seq_printf(seq, "%04x     %08x %08x%s\n", offset, load, save,
			   load != save ? "   <-- differs" : "");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sun6i_isp_debugfs_raw);

static void sun6i_isp_debugfs_setup(struct sun6i_isp_device *isp_dev)
{
	isp_dev->debugfs = debugfs_create_dir(SUN6I_ISP_NAME, NULL);

	debugfs_create_file("registers", 0444, isp_dev->debugfs, isp_dev,
			    &sun6i_isp_debugfs_registers_fops);
	debugfs_create_file("poke", 0200, isp_dev->debugfs, isp_dev,
			    &sun6i_isp_debugfs_poke_fops);
	debugfs_create_file("load_poke", 0200, isp_dev->debugfs, isp_dev,
			    &sun6i_isp_debugfs_load_poke_fops);
	debugfs_create_file("raw", 0444, isp_dev->debugfs, isp_dev,
			    &sun6i_isp_debugfs_raw_fops);
}

static void sun6i_isp_debugfs_cleanup(struct sun6i_isp_device *isp_dev)
{
	debugfs_remove_recursive(isp_dev->debugfs);
	isp_dev->debugfs = NULL;
}

static int sun6i_isp_v4l2_setup(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_v4l2 *v4l2 = &isp_dev->v4l2;
	struct v4l2_device *v4l2_dev = &v4l2->v4l2_dev;
	struct media_device *media_dev = &v4l2->media_dev;
	struct device *dev = isp_dev->dev;
	int ret;

	/* Media Device */

	strscpy(media_dev->model, SUN6I_ISP_DESCRIPTION,
		sizeof(media_dev->model));
	media_dev->ops = &sun6i_isp_media_ops;
	media_dev->hw_revision = 0;
	media_dev->dev = dev;

	media_device_init(media_dev);

	ret = media_device_register(media_dev);
	if (ret) {
		dev_err(dev, "failed to register media device\n");
		return ret;
	}

	/* V4L2 Device */

	v4l2_dev->mdev = media_dev;

	ret = v4l2_device_register(dev, v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register v4l2 device\n");
		goto error_media;
	}

	return 0;

error_media:
	media_device_unregister(media_dev);
	media_device_cleanup(media_dev);

	return ret;
}

static void sun6i_isp_v4l2_cleanup(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_v4l2 *v4l2 = &isp_dev->v4l2;

	media_device_unregister(&v4l2->media_dev);
	v4l2_device_unregister(&v4l2->v4l2_dev);
	media_device_cleanup(&v4l2->media_dev);
}

/* Platform */

static irqreturn_t sun6i_isp_interrupt(int irq, void *private)
{
	struct sun6i_isp_device *isp_dev = private;
	struct sun6i_isp_tables *tables = &isp_dev->tables;
	struct regmap *regmap = isp_dev->regmap;
	u32 status = 0, enable = 0;
	bool thread = false;
	dma_addr_t address;

	regmap_read(regmap, SUN6I_ISP_FE_INT_STA_REG, &status);
	regmap_read(regmap, SUN6I_ISP_FE_INT_EN_REG, &enable);

	if (!status)
		return IRQ_NONE;
	else if (!(status & enable))
		goto complete;

	/*
	 * The ISP working cycle starts with a params-load, which makes the
	 * state from the load buffer active. Then it starts processing the
	 * frame and gives a finish interrupt. Soon after that, the next state
	 * coming from the load buffer will be applied for the next frame,
	 * giving a params-load as well.
	 *
	 * Because both frame finish and params-load are received almost
	 * at the same time (one ISR call), handle them in chronology order.
	 */

	if (status & SUN6I_ISP_FE_INT_STA_FINISH) {
		sun6i_isp_capture_finish(isp_dev);

		/*
		 * Hand the buffer the hardware has just finished to the
		 * thread and point the hardware at the other one, so the
		 * copy out is not racing the next frame being written.
		 */

		tables->stats_done = tables->stats_index;
		tables->stats_index ^= 1;

		address = tables->stats[tables->stats_index].address;
		regmap_write(regmap, SUN6I_ISP_STATS_ADDR_REG,
			     SUN6I_ISP_ADDR_VALUE(address));

		thread = true;
	}

	if (status & SUN6I_ISP_FE_INT_STA_PARA_LOAD) {
		sun6i_isp_state_complete(isp_dev);
		sun6i_isp_state_update(isp_dev, false);
	}

complete:
	regmap_write(regmap, SUN6I_ISP_FE_INT_STA_REG, status);

	return thread ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

/*
 * The statistics copy is several kilobytes, which is more than belongs in
 * the hard handler -- especially here, where the line is shared with the
 * CSI on some variants.
 *
 * The line is not requested one-shot, so it is left unmasked for the
 * other user while this runs. The hard handler has already acked, and
 * has already swapped the hardware onto the other buffer, so a frame
 * arriving before this gets scheduled costs at worst one dropped set of
 * statistics rather than a torn one.
 */
static irqreturn_t sun6i_isp_interrupt_thread(int irq, void *private)
{
	struct sun6i_isp_device *isp_dev = private;
	struct sun6i_isp_tables *tables = &isp_dev->tables;
	int done = READ_ONCE(tables->stats_done);

	if (done < 0)
		return IRQ_HANDLED;

	sun6i_isp_stats_finish(isp_dev, tables->stats[done].data);

	return IRQ_HANDLED;
}

static int sun6i_isp_suspend(struct device *dev)
{
	struct sun6i_isp_device *isp_dev = dev_get_drvdata(dev);

	reset_control_assert(isp_dev->reset);
	clk_disable_unprepare(isp_dev->clock_ram);
	clk_disable_unprepare(isp_dev->clock_mod);

	return 0;
}

static int sun6i_isp_resume(struct device *dev)
{
	struct sun6i_isp_device *isp_dev = dev_get_drvdata(dev);
	int ret;

	ret = reset_control_deassert(isp_dev->reset);
	if (ret) {
		dev_err(dev, "failed to deassert reset\n");
		return ret;
	}

	ret = clk_prepare_enable(isp_dev->clock_mod);
	if (ret) {
		dev_err(dev, "failed to enable module clock\n");
		goto error_reset;
	}

	ret = clk_prepare_enable(isp_dev->clock_ram);
	if (ret) {
		dev_err(dev, "failed to enable ram clock\n");
		goto error_clock_mod;
	}

	return 0;

error_clock_mod:
	clk_disable_unprepare(isp_dev->clock_mod);

error_reset:
	reset_control_assert(isp_dev->reset);

	return ret;
}

static const struct dev_pm_ops sun6i_isp_pm_ops = {
	.runtime_suspend	= sun6i_isp_suspend,
	.runtime_resume		= sun6i_isp_resume,
};

static const struct regmap_config sun6i_isp_regmap_config = {
	.reg_bits       = 32,
	.reg_stride     = 4,
	.val_bits       = 32,
	.max_register	= 0x400,
};

static int sun6i_isp_resources_setup(struct sun6i_isp_device *isp_dev,
				     struct platform_device *platform_dev)
{
	struct device *dev = isp_dev->dev;
	void __iomem *io_base;
	int irq;
	int ret;

	/* Registers */

	io_base = devm_platform_ioremap_resource(platform_dev, 0);
	if (IS_ERR(io_base))
		return PTR_ERR(io_base);

	isp_dev->regmap = devm_regmap_init_mmio_clk(dev, "bus", io_base,
						    &sun6i_isp_regmap_config);
	if (IS_ERR(isp_dev->regmap)) {
		dev_err(dev, "failed to init register map\n");
		return PTR_ERR(isp_dev->regmap);
	}

	/* Clocks */

	isp_dev->clock_mod = devm_clk_get(dev, "mod");
	if (IS_ERR(isp_dev->clock_mod)) {
		dev_err(dev, "failed to acquire module clock\n");
		return PTR_ERR(isp_dev->clock_mod);
	}

	isp_dev->clock_ram = devm_clk_get(dev, "ram");
	if (IS_ERR(isp_dev->clock_ram)) {
		dev_err(dev, "failed to acquire ram clock\n");
		return PTR_ERR(isp_dev->clock_ram);
	}

	ret = clk_set_rate_exclusive(isp_dev->clock_mod, 297000000);
	if (ret) {
		dev_err(dev, "failed to set mod clock rate\n");
		return ret;
	}

	/* Reset */

	isp_dev->reset = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(isp_dev->reset)) {
		dev_err(dev, "failed to acquire reset\n");
		ret = PTR_ERR(isp_dev->reset);
		goto error_clock_rate_exclusive;
	}

	/* Interrupt */

	irq = platform_get_irq(platform_dev, 0);
	if (irq < 0) {
		ret = irq;
		goto error_clock_rate_exclusive;
	}

	ret = devm_request_threaded_irq(dev, irq, sun6i_isp_interrupt,
					sun6i_isp_interrupt_thread, IRQF_SHARED,
			       SUN6I_ISP_NAME, isp_dev);
	if (ret) {
		dev_err(dev, "failed to request interrupt\n");
		goto error_clock_rate_exclusive;
	}

	/* Kept for synchronize_irq() when the pipeline is taken down. */
	isp_dev->irq = irq;

	/* Runtime PM */

	pm_runtime_enable(dev);

	return 0;

error_clock_rate_exclusive:
	clk_rate_exclusive_put(isp_dev->clock_mod);

	return ret;
}

static void sun6i_isp_resources_cleanup(struct sun6i_isp_device *isp_dev)
{
	struct device *dev = isp_dev->dev;

	pm_runtime_disable(dev);
	clk_rate_exclusive_put(isp_dev->clock_mod);
}

static int sun6i_isp_probe(struct platform_device *platform_dev)
{
	struct sun6i_isp_device *isp_dev;
	struct device *dev = &platform_dev->dev;
	const struct sun6i_isp_variant *variant;
	int ret;

	variant = of_device_get_match_data(dev);
	if (!variant)
		return -EINVAL;

	isp_dev = devm_kzalloc(dev, sizeof(*isp_dev), GFP_KERNEL);
	if (!isp_dev)
		return -ENOMEM;

	isp_dev->dev = dev;
	platform_set_drvdata(platform_dev, isp_dev);

	spin_lock_init(&isp_dev->state_lock);
	mutex_init(&isp_dev->stream_lock);

	ret = sun6i_isp_resources_setup(isp_dev, platform_dev);
	if (ret)
		return ret;

	ret = sun6i_isp_tables_setup(isp_dev, variant);
	if (ret) {
		dev_err(dev, "failed to setup tables\n");
		goto error_resources;
	}

	ret = sun6i_isp_v4l2_setup(isp_dev);
	if (ret) {
		dev_err(dev, "failed to setup v4l2\n");
		goto error_tables;
	}

	ret = sun6i_isp_proc_setup(isp_dev);
	if (ret) {
		dev_err(dev, "failed to setup proc\n");
		goto error_v4l2;
	}

	ret = sun6i_isp_capture_setup(isp_dev);
	if (ret) {
		dev_err(dev, "failed to setup capture\n");
		goto error_proc;
	}

	ret = sun6i_isp_stats_setup(isp_dev);
	if (ret) {
		dev_err(dev, "failed to setup stats\n");
		goto error_capture;
	}

	ret = sun6i_isp_params_setup(isp_dev);
	if (ret) {
		dev_err(dev, "failed to setup params\n");
		goto error_stats;
	}

	sun6i_isp_debugfs_setup(isp_dev);

	return 0;

error_stats:
	sun6i_isp_stats_cleanup(isp_dev);

error_capture:
	sun6i_isp_capture_cleanup(isp_dev);

error_proc:
	sun6i_isp_proc_cleanup(isp_dev);

error_v4l2:
	sun6i_isp_v4l2_cleanup(isp_dev);

error_tables:
	sun6i_isp_tables_cleanup(isp_dev);

error_resources:
	sun6i_isp_resources_cleanup(isp_dev);

	mutex_destroy(&isp_dev->stream_lock);

	return ret;
}

static void sun6i_isp_remove(struct platform_device *platform_dev)
{
	struct sun6i_isp_device *isp_dev = platform_get_drvdata(platform_dev);

	sun6i_isp_debugfs_cleanup(isp_dev);
	sun6i_isp_params_cleanup(isp_dev);
	sun6i_isp_stats_cleanup(isp_dev);
	sun6i_isp_capture_cleanup(isp_dev);
	sun6i_isp_proc_cleanup(isp_dev);
	sun6i_isp_v4l2_cleanup(isp_dev);
	sun6i_isp_tables_cleanup(isp_dev);
	sun6i_isp_resources_cleanup(isp_dev);

	mutex_destroy(&isp_dev->stream_lock);
}

/*
 * History of sun6i-isp:
 * - sun4i-a10-isp: initial ISP tied to the CSI0 controller,
 *   apparently unused in software implementations;
 * - sun6i-a31-isp: separate ISP loosely based on sun4i-a10-isp,
 *   adding extra modules and features;
 * - sun9i-a80-isp: based on sun6i-a31-isp with some register offset changes
 *   and new modules like saturation and cnr;
 * - sun8i-a23-isp/sun8i-h3-isp: based on sun9i-a80-isp with most modules
 *   related to raw removed;
 * - sun8i-a83t-isp: based on sun9i-a80-isp with some register offset changes
 * - sun8i-v3s-isp: based on sun8i-a83t-isp with a new disc module;
 */

static const struct sun6i_isp_variant sun8i_a83t_isp_variant = {
	.table_load_save_size	= 0x1000,
	.table_lut_size		= 0x800,
	.table_drc_size		= 0x200,
	.table_stats_size	= 0x2100,
};

static const struct sun6i_isp_variant sun8i_v3s_isp_variant = {
	.table_load_save_size	= 0x1000,
	.table_lut_size		= 0xe00,
	.table_drc_size		= 0x600,
	.table_stats_size	= 0x2100,
};

static const struct of_device_id sun6i_isp_of_match[] = {
	{
		.compatible	= "allwinner,sun8i-a83t-isp",
		.data		= &sun8i_a83t_isp_variant,
	},
	{
		.compatible	= "allwinner,sun8i-v3s-isp",
		.data		= &sun8i_v3s_isp_variant,
	},
	{},
};

MODULE_DEVICE_TABLE(of, sun6i_isp_of_match);

static struct platform_driver sun6i_isp_platform_driver = {
	.probe	= sun6i_isp_probe,
	.remove = sun6i_isp_remove,
	.driver	= {
		.name		= SUN6I_ISP_NAME,
		.of_match_table	= sun6i_isp_of_match,
		.pm		= &sun6i_isp_pm_ops,
	},
};

module_platform_driver(sun6i_isp_platform_driver);

MODULE_DESCRIPTION("Allwinner A31 Image Signal Processor driver");
MODULE_AUTHOR("Paul Kocialkowski <paul.kocialkowski@bootlin.com>");
MODULE_LICENSE("GPL");
