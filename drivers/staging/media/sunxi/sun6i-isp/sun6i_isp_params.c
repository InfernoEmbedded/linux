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
#include "sun6i_isp_params.h"
#include "sun6i_isp_proc.h"
#include "sun6i_isp_reg.h"
#include "uapi/sun6i-isp-config.h"

/* Params */

static const struct sun6i_isp_params_config sun6i_isp_params_config_default = {
	.modules_used = SUN6I_ISP_MODULE_BAYER,

	.bayer = {
		/*
		 * Zero is the honest value: the luma coefficients sum to
		 * exactly 16384, so with no offset the matrix is unity and a
		 * uniform field should come out as it went in.
		 *
		 * It does not, quite. The hardware drops a level whenever the
		 * matrix accumulator lands exactly on a multiple of 2^14 --
		 * which is every level of a grey field, since the row sums to
		 * unity -- unless it also lands on a multiple of 2^20. So a
		 * grey ramp reads v - 1 at 189 of 192 levels and v at 0, 64
		 * and 128.
		 *
		 * An offset of 16 hides that, by shifting everything up a
		 * level before the matrix, and this was set to 16 for a while
		 * on that basis. It is left at 0 because the defect is the
		 * hardware's and not the driver's, and hiding it here means
		 * every measurement taken through this driver has to know
		 * that the default is not neutral. Userspace can set 16 if it
		 * wants the brighter result.
		 */
		.offset_r	= 0,
		.offset_gr	= 0,
		.offset_gb	= 0,
		.offset_b	= 0,

		.gain_r		= 256,
		.gain_gr	= 256,
		.gain_gb	= 256,
		.gain_b		= 256,

	},

	.bdnf = {
		.in_dis_min		= 8,
		.in_dis_max		= 16,

		.coefficients_g		= { 15, 4, 1 },
		.coefficients_rb	= { 15, 4 },
	},
};

static void sun6i_isp_params_configure_ob(struct sun6i_isp_device *isp_dev)
{
	unsigned int width, height;

	sun6i_isp_proc_dimensions(isp_dev, &width, &height);

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_SIZE_REG,
			     SUN6I_ISP_OB_SIZE_WIDTH(width) |
			     SUN6I_ISP_OB_SIZE_HEIGHT(height));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_VALID_REG,
			     SUN6I_ISP_OB_VALID_WIDTH(width) |
			     SUN6I_ISP_OB_VALID_HEIGHT(height));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_SRC0_VALID_START_REG,
			     SUN6I_ISP_OB_SRC0_VALID_START_HORZ(0) |
			     SUN6I_ISP_OB_SRC0_VALID_START_VERT(0));
}

static void sun6i_isp_params_configure_ae_default(struct sun6i_isp_device *isp_dev)
{
	/* These are default values that need to be set to get an output. */

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AE_CFG_REG,
			     SUN6I_ISP_AE_CFG_LOW_BRI_TH(0xff) |
			     SUN6I_ISP_AE_CFG_HORZ_NUM(8) |
			     SUN6I_ISP_AE_CFG_HIGH_BRI_TH(0xf00) |
			     SUN6I_ISP_AE_CFG_VERT_NUM(8));
}

static void
sun6i_isp_params_configure_bayer(struct sun6i_isp_device *isp_dev,
				 const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_bayer *bayer = &config->bayer;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_BAYER_OFFSET0_REG,
			     SUN6I_ISP_BAYER_OFFSET0_R(bayer->offset_r) |
			     SUN6I_ISP_BAYER_OFFSET0_GR(bayer->offset_gr));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_BAYER_OFFSET1_REG,
			     SUN6I_ISP_BAYER_OFFSET1_GB(bayer->offset_gb) |
			     SUN6I_ISP_BAYER_OFFSET1_B(bayer->offset_b));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_BAYER_GAIN0_REG,
			     SUN6I_ISP_BAYER_GAIN0_R(bayer->gain_r) |
			     SUN6I_ISP_BAYER_GAIN0_GR(bayer->gain_gr));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_BAYER_GAIN1_REG,
			     SUN6I_ISP_BAYER_GAIN1_GB(bayer->gain_gb) |
			     SUN6I_ISP_BAYER_GAIN1_B(bayer->gain_b));
}

static void
sun6i_isp_params_configure_yuv_gain_offset(struct sun6i_isp_device *isp_dev,
					   const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_yuv_gain_offset *yuv =
		&config->yuv_gain_offset;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_YUV_OFFSET0_REG,
			     SUN6I_ISP_YUV_OFFSET0_Y(yuv->offset_y) |
			     SUN6I_ISP_YUV_OFFSET0_U(yuv->offset_u));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_YUV_OFFSET1_REG,
			     SUN6I_ISP_YUV_OFFSET1_V(yuv->offset_v));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_YUV_GAIN0_REG,
			     SUN6I_ISP_YUV_GAIN0_Y(yuv->gain_y) |
			     SUN6I_ISP_YUV_GAIN0_U(yuv->gain_u));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_YUV_GAIN1_REG,
			     SUN6I_ISP_YUV_GAIN1_V(yuv->gain_v));
}

static void
sun6i_isp_params_configure_contrast(struct sun6i_isp_device *isp_dev,
				    const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_contrast *contrast =
		&config->contrast;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_CONTRAST_CFG_REG,
			     SUN6I_ISP_CONTRAST_CFG_LEVEL(contrast->level) |
			     SUN6I_ISP_CONTRAST_CFG_MIN(contrast->min) |
			     SUN6I_ISP_CONTRAST_CFG_MAX(contrast->max));
}

static void
sun6i_isp_params_configure_saturation(struct sun6i_isp_device *isp_dev,
				      const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_saturation *saturation =
		&config->saturation;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_SATURATION_CFG_REG,
			     SUN6I_ISP_SATURATION_CFG_R(saturation->saturation_r) |
			     SUN6I_ISP_SATURATION_CFG_G(saturation->saturation_g) |
			     SUN6I_ISP_SATURATION_CFG_B(saturation->saturation_b) |
			     SUN6I_ISP_SATURATION_CFG_GAIN(saturation->gain));
}

static void
sun6i_isp_params_configure_sharp(struct sun6i_isp_device *isp_dev,
				 const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_sharp *sharp = &config->sharp;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_SAP_CFG_REG,
			     SUN6I_ISP_SAP_CFG_LEVEL(sharp->level) |
			     SUN6I_ISP_SAP_CFG_MIN(sharp->min) |
			     SUN6I_ISP_SAP_CFG_MAX(sharp->max));
}

static void
sun6i_isp_params_configure_rgb2rgb(struct sun6i_isp_device *isp_dev,
				   const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_rgb2rgb *rgb2rgb =
		&config->rgb2rgb;
	const __u16 *matrix = &rgb2rgb->matrix[0][0];

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_RGB2RGB_GAIN0_REG,
			     SUN6I_ISP_RGB2RGB_GAIN_LOW(matrix[0]) |
			     SUN6I_ISP_RGB2RGB_GAIN_HIGH(matrix[1]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_RGB2RGB_GAIN1_REG,
			     SUN6I_ISP_RGB2RGB_GAIN_LOW(matrix[2]) |
			     SUN6I_ISP_RGB2RGB_GAIN_HIGH(matrix[3]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_RGB2RGB_GAIN2_REG,
			     SUN6I_ISP_RGB2RGB_GAIN_LOW(matrix[4]) |
			     SUN6I_ISP_RGB2RGB_GAIN_HIGH(matrix[5]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_RGB2RGB_GAIN3_REG,
			     SUN6I_ISP_RGB2RGB_GAIN_LOW(matrix[6]) |
			     SUN6I_ISP_RGB2RGB_GAIN_HIGH(matrix[7]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_RGB2RGB_GAIN4_REG,
			     SUN6I_ISP_RGB2RGB_GAIN4_MATRIX(matrix[8]) |
			     SUN6I_ISP_RGB2RGB_GAIN4_OFFSET_R(rgb2rgb->offset[0]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_RGB2RGB_OFFSET_REG,
			     SUN6I_ISP_RGB2RGB_OFFSET_G(rgb2rgb->offset[1]) |
			     SUN6I_ISP_RGB2RGB_OFFSET_B(rgb2rgb->offset[2]));
}

/*
 * Copy a sub-table into one of the DMA table buffers and note the
 * FE_CTRL bit that tells the hardware to reload it. The bit is only
 * applied at the next parameter sync point, so that a table swapped in
 * mid-stream takes effect together with the rest of the configuration.
 */
static void sun6i_isp_params_table_write(struct sun6i_isp_device *isp_dev,
					 struct sun6i_isp_table *table,
					 unsigned int offset, u32 update,
					 const void *data, unsigned int size)
{
	if (WARN_ON(offset + size > table->size))
		return;

	memcpy(table->data + offset, data, size);

	isp_dev->params.state.table_update |= update;
}

static void
sun6i_isp_params_configure_gamma(struct sun6i_isp_device *isp_dev,
				 const struct sun6i_isp_params_config *config)
{
	sun6i_isp_params_table_write(isp_dev, &isp_dev->tables.lut,
				     SUN6I_ISP_TABLE_GAMMA_OFFSET,
				     SUN6I_ISP_FE_CTRL_GAMMA_UPDATE,
				     config->gamma.table,
				     sizeof(config->gamma.table));
}

static void
sun6i_isp_params_configure_lsc(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_lsc *lsc = &config->lsc;
	unsigned int offset = SUN6I_ISP_TABLE_LENS_OFFSET;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_LSC_CFG_REG,
			     SUN6I_ISP_LSC_CFG_CENTER_X(lsc->center_x) |
			     SUN6I_ISP_LSC_CFG_CENTER_Y(lsc->center_y) |
			     SUN6I_ISP_LSC_CFG_SCALE(lsc->scale));

	/* The three curves sit back to back at the start of the buffer. */

	sun6i_isp_params_table_write(isp_dev, &isp_dev->tables.lut, offset,
				     SUN6I_ISP_FE_CTRL_LENS_UPDATE,
				     lsc->table_r, sizeof(lsc->table_r));
	offset += sizeof(lsc->table_r);

	sun6i_isp_params_table_write(isp_dev, &isp_dev->tables.lut, offset,
				     SUN6I_ISP_FE_CTRL_LENS_UPDATE,
				     lsc->table_g, sizeof(lsc->table_g));
	offset += sizeof(lsc->table_g);

	sun6i_isp_params_table_write(isp_dev, &isp_dev->tables.lut, offset,
				     SUN6I_ISP_FE_CTRL_LENS_UPDATE,
				     lsc->table_b, sizeof(lsc->table_b));
}

static void
sun6i_isp_params_configure_drc(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	sun6i_isp_params_table_write(isp_dev, &isp_dev->tables.drc,
				     SUN6I_ISP_TABLE_DRC_OFFSET,
				     SUN6I_ISP_FE_CTRL_DRC_UPDATE,
				     config->drc.table,
				     sizeof(config->drc.table));
}

static void
sun6i_isp_params_configure_ae(struct sun6i_isp_device *isp_dev,
			      const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_ae *ae = &config->ae;
	const struct sun6i_isp_params_config_h3a_window *win = &ae->window;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AE_CFG_REG,
			     SUN6I_ISP_AE_CFG_LOW_BRI_TH(ae->low_brightness_threshold) |
			     SUN6I_ISP_AE_CFG_HORZ_NUM(win->horz_num) |
			     SUN6I_ISP_AE_CFG_HIGH_BRI_TH(ae->high_brightness_threshold) |
			     SUN6I_ISP_AE_CFG_VERT_NUM(win->vert_num));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AE_SIZE_REG,
			     SUN6I_ISP_AE_SIZE_WIDTH(SUN6I_ISP_H3A_SIZE(win->width)) |
			     SUN6I_ISP_AE_SIZE_HEIGHT(SUN6I_ISP_H3A_SIZE(win->height)));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AE_POS_REG,
			     SUN6I_ISP_AE_POS_HORZ_START(SUN6I_ISP_H3A_POS(win->horz_start)) |
			     SUN6I_ISP_AE_POS_VERT_START(SUN6I_ISP_H3A_POS(win->vert_start)));
}

static void
sun6i_isp_params_configure_af(struct sun6i_isp_device *isp_dev,
			      const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_h3a_window *win = &config->af.window;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AF_CFG_REG,
			     SUN6I_ISP_AF_CFG_HORZ_NUM(win->horz_num) |
			     SUN6I_ISP_AF_CFG_VERT_NUM(win->vert_num));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AF_SIZE_REG,
			     SUN6I_ISP_AF_SIZE_WIDTH(SUN6I_ISP_H3A_SIZE(win->width)) |
			     SUN6I_ISP_AF_SIZE_HEIGHT(SUN6I_ISP_H3A_SIZE(win->height)));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AF_START_REG,
			     SUN6I_ISP_AF_START_HORZ(SUN6I_ISP_H3A_POS(win->horz_start)) |
			     SUN6I_ISP_AF_START_VERT(SUN6I_ISP_H3A_POS(win->vert_start)));
}

static void
sun6i_isp_params_configure_hist(struct sun6i_isp_device *isp_dev,
				const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_h3a_window *win =
		&config->hist.window;
	u32 value;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_HIST_SIZE_REG,
			     SUN6I_ISP_HIST_SIZE_WIDTH(SUN6I_ISP_H3A_SIZE(win->width)) |
			     SUN6I_ISP_HIST_SIZE_HEIGHT(SUN6I_ISP_H3A_SIZE(win->height)));

	/* Note the swapped halves compared to the other windows. */

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_HIST_POS_REG,
			     SUN6I_ISP_HIST_POS_HORZ(SUN6I_ISP_H3A_POS(win->horz_start)) |
			     SUN6I_ISP_HIST_POS_VERT(SUN6I_ISP_H3A_POS(win->vert_start)));

	value = sun6i_isp_load_read(isp_dev, SUN6I_ISP_MODE_REG);
	value &= ~SUN6I_ISP_MODE_HIST(3);
	value |= SUN6I_ISP_MODE_HIST(config->hist.mode);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_MODE_REG, value);
}

static void
sun6i_isp_params_configure_awb(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_awb *awb = &config->awb;
	const struct sun6i_isp_params_config_h3a_window *win = &awb->window;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AWB_CFG0_REG,
			     SUN6I_ISP_AWB_CFG0_SATURATION0(awb->saturation[0]) |
			     SUN6I_ISP_AWB_CFG0_SATURATION1(awb->saturation[1]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AWB_CFG1_REG,
			     SUN6I_ISP_AWB_CFG1_SATURATION1(awb->saturation[1]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AWB_CFG4_REG,
			     SUN6I_ISP_AWB_CFG4_WIDTH(SUN6I_ISP_H3A_SIZE(win->width)) |
			     SUN6I_ISP_AWB_CFG4_HEIGHT(SUN6I_ISP_H3A_SIZE(win->height)));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AWB_CFG5_REG,
			     SUN6I_ISP_AWB_CFG5_HORZ_START(SUN6I_ISP_H3A_POS(win->horz_start)) |
			     SUN6I_ISP_AWB_CFG5_VERT_START(SUN6I_ISP_H3A_POS(win->vert_start)));
}

static void
sun6i_isp_params_configure_afs(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_AFS_CFG_REG,
			     SUN6I_ISP_AFS_CFG_ANTI_FLICKER(config->afs.anti_flicker));
}

static void
sun6i_isp_params_configure_cnr(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_CNR_REG,
			     SUN6I_ISP_CNR_OFFSET(config->cnr.offset));
}

static void sun6i_isp_params_configure_wb(struct sun6i_isp_device *isp_dev)
{
	/* These are default values that need to be set to get an output. */

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_WB_GAIN0_REG,
			     SUN6I_ISP_WB_GAIN0_R(256) |
			     SUN6I_ISP_WB_GAIN0_GR(256));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_WB_GAIN1_REG,
			     SUN6I_ISP_WB_GAIN1_GB(256) |
			     SUN6I_ISP_WB_GAIN1_B(256));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_WB_CFG_REG,
			     SUN6I_ISP_WB_CFG_CLIP(0xfff));
}

static void sun6i_isp_params_configure_base(struct sun6i_isp_device *isp_dev)
{
	sun6i_isp_params_configure_ae_default(isp_dev);
	sun6i_isp_params_configure_ob(isp_dev);
	sun6i_isp_params_configure_wb(isp_dev);
}

/*
 * Defect pixel correction. The mode lives in MODE rather than beside the
 * thresholds, so that register is read back and only its one bit changed
 * -- the input format and the histogram mode share it.
 */
static void
sun6i_isp_params_configure_dpc(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_dpc *dpc = &config->dpc;
	u32 value;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OTF_DC_CFG_REG,
			     SUN6I_ISP_OTF_DC_CFG_TH_SLOPE(dpc->threshold_slope) |
			     SUN6I_ISP_OTF_DC_CFG_MIN_TH(dpc->threshold_min) |
			     SUN6I_ISP_OTF_DC_CFG_MAX_TH(dpc->threshold_max));

	value = sun6i_isp_load_read(isp_dev, SUN6I_ISP_MODE_REG);
	value &= ~SUN6I_ISP_MODE_OTF_DPC(1);
	value |= SUN6I_ISP_MODE_OTF_DPC(dpc->mode);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_MODE_REG, value);
}

/*
 * The demosaic. It has no enable bit of its own and runs whatever these
 * hold.
 *
 * The vendor does not program either of them, contrary to what this
 * comment used to claim. Its dispatch table has an entry for the
 * demosaic, but the feature bit that would reach it, ISP_FEATURES_CFA,
 * appears exactly once in the whole library -- in the enum defining it
 * -- so nothing ever sets it and the entry is never called. There is no
 * demosaic key in any of the tuning files either, and the one consumer
 * of what the block measures, the defogger, is not built into that
 * library at all. The vendor runs this block on its power-up values.
 *
 * Of the two, only the direction threshold does anything, and it is a
 * boolean rather than a threshold: zero interpolates isotropically, any
 * non-zero value interpolates directionally, and no value above zero
 * behaves differently from any other. The minimum is inert -- writing it
 * is ignored, and the field exists so the hardware can report the
 * minimum it saw.
 *
 * The default of zero is deliberate. The directional mode reconstructs
 * horizontal detail exactly and costs accuracy on vertical detail, so it
 * is a trade rather than an improvement, and userspace makes it.
 */
static void
sun6i_isp_params_configure_cfa(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_cfa *cfa = &config->cfa;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_CFA_CFG0_REG,
			     SUN6I_ISP_CFA_CFG0_DIR_TH(cfa->direction_threshold) |
			     SUN6I_ISP_CFA_CFG0_MIN_RGB(cfa->min_rgb));

	/*
	 * The other two copies of the minimum, which every caller so far has
	 * set to the same value as the first. They are separate here so that
	 * assumption can be tested rather than inherited.
	 */
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_CFA_CFG1_REG,
			     SUN6I_ISP_CFA_CFG1_MIN_RGB_LOW(cfa->min_rgb_low) |
			     SUN6I_ISP_CFA_CFG1_MIN_RGB_HIGH(cfa->min_rgb_high));

	/*
	 * A register the hardware writes its own measurement into. Writing it
	 * may do nothing at all; that is what this is for.
	 */
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_CFA_TEX_REG, cfa->texture);
}

/*
 * White balance correction. sun6i_isp_params_configure_wb() below sets the
 * unity gains that the pipeline needs to produce anything at all; this is
 * the module a parameters buffer can steer, and unlike that one it also
 * turns the block on.
 */
static void
sun6i_isp_params_configure_wb_gains(struct sun6i_isp_device *isp_dev,
				    const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_wb *wb = &config->wb;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_WB_GAIN0_REG,
			     SUN6I_ISP_WB_GAIN0_R(wb->gain_r) |
			     SUN6I_ISP_WB_GAIN0_GR(wb->gain_gr));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_WB_GAIN1_REG,
			     SUN6I_ISP_WB_GAIN1_GB(wb->gain_gb) |
			     SUN6I_ISP_WB_GAIN1_B(wb->gain_b));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_WB_CFG_REG,
			     SUN6I_ISP_WB_CFG_CLIP(wb->clip));
}

/*
 * Optical black clamp.
 *
 * The frame size stays as configure_ob() sets it, since that has to match
 * what the frontend is actually receiving. Everything else is the caller's,
 * because nothing here is understood well enough to derive: the block has
 * never been enabled on this variant, and its configuration, position and
 * fixed-offset registers have never been written by any driver.
 */
static void
sun6i_isp_params_configure_obc(struct sun6i_isp_device *isp_dev,
			       const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_obc *obc = &config->obc;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_VALID_REG,
			     SUN6I_ISP_OB_VALID_WIDTH(obc->valid_width) |
			     SUN6I_ISP_OB_VALID_HEIGHT(obc->valid_height));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_SRC0_VALID_START_REG,
			     SUN6I_ISP_OB_SRC0_VALID_START_HORZ(obc->valid_start_horz) |
			     SUN6I_ISP_OB_SRC0_VALID_START_VERT(obc->valid_start_vert));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_SPRITE_REG,
			     SUN6I_ISP_OB_SPRITE_WIDTH(obc->sprite_width) |
			     SUN6I_ISP_OB_SPRITE_HEIGHT(obc->sprite_height));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_SPRITE_START_REG,
			     SUN6I_ISP_OB_SPRITE_START_HORZ(obc->sprite_start_horz) |
			     SUN6I_ISP_OB_SPRITE_START_VERT(obc->sprite_start_vert));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_CFG_REG, obc->cfg);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_HORZ_POS_REG, obc->horz_pos);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_VERT_PARA_REG, obc->vert_para);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_OB_OFFSET_FIXED_REG,
			     obc->offset_fixed);

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_SRC0_OB_CLAMP_R_REG,
			     obc->clamp_r);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_SRC0_OB_CLAMP_GR_REG,
			     obc->clamp_gr);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_SRC0_OB_CLAMP_GB_REG,
			     obc->clamp_gb);
	sun6i_isp_load_write(isp_dev, SUN6I_ISP_SRC0_OB_CLAMP_B_REG,
			     obc->clamp_b);
}

static void
sun6i_isp_params_configure_bdnf(struct sun6i_isp_device *isp_dev,
				const struct sun6i_isp_params_config *config)
{
	const struct sun6i_isp_params_config_bdnf *bdnf = &config->bdnf;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_BDNF_CFG_REG,
			     SUN6I_ISP_BDNF_CFG_IN_DIS_MIN(bdnf->in_dis_min) |
			     SUN6I_ISP_BDNF_CFG_IN_DIS_MAX(bdnf->in_dis_max));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_BDNF_COEF_RB_REG,
			     SUN6I_ISP_BDNF_COEF_RB(0, bdnf->coefficients_rb[0]) |
			     SUN6I_ISP_BDNF_COEF_RB(1, bdnf->coefficients_rb[1]) |
			     SUN6I_ISP_BDNF_COEF_RB(2, bdnf->coefficients_rb[2]) |
			     SUN6I_ISP_BDNF_COEF_RB(3, bdnf->coefficients_rb[3]) |
			     SUN6I_ISP_BDNF_COEF_RB(4, bdnf->coefficients_rb[4]));

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_BDNF_COEF_G_REG,
			     SUN6I_ISP_BDNF_COEF_G(0, bdnf->coefficients_g[0]) |
			     SUN6I_ISP_BDNF_COEF_G(1, bdnf->coefficients_g[1]) |
			     SUN6I_ISP_BDNF_COEF_G(2, bdnf->coefficients_g[2]) |
			     SUN6I_ISP_BDNF_COEF_G(3, bdnf->coefficients_g[3]) |
			     SUN6I_ISP_BDNF_COEF_G(4, bdnf->coefficients_g[4]) |
			     SUN6I_ISP_BDNF_COEF_G(5, bdnf->coefficients_g[5]) |
			     SUN6I_ISP_BDNF_COEF_G(6, bdnf->coefficients_g[6]));
}

static void
sun6i_isp_params_configure_modules(struct sun6i_isp_device *isp_dev,
				   const struct sun6i_isp_params_config *config)
{
	bool bayer = sun6i_isp_proc_bayer(isp_dev);
	u32 modules = config->modules_used;
	u32 value;

	/*
	 * Some modules only exist on the way from bayer to YUV: the bayer
	 * gain/offset and the bayer denoise ahead of the demosaic, the
	 * colour correction matrix behind it. None of them has anything to
	 * act on when the input is already YUV, so leave them alone.
	 */

	if (!bayer)
		modules &= ~(SUN6I_ISP_MODULE_OBC |
			     SUN6I_ISP_MODULE_BDNF | SUN6I_ISP_MODULE_BAYER |
			     SUN6I_ISP_MODULE_RGB2RGB |
			     SUN6I_ISP_MODULE_CFA | SUN6I_ISP_MODULE_WB |
			     SUN6I_ISP_MODULE_DPC);

	/*
	 * The bayer and YUV gain/offset are the same registers behind two
	 * enable bits, so let the bayer one win rather than program the
	 * block twice and enable both interpretations of it.
	 */

	if (modules & SUN6I_ISP_MODULE_BAYER)
		modules &= ~SUN6I_ISP_MODULE_YUV_GAIN_OFFSET;

	if (modules & SUN6I_ISP_MODULE_OBC)
		sun6i_isp_params_configure_obc(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_BDNF)
		sun6i_isp_params_configure_bdnf(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_BAYER)
		sun6i_isp_params_configure_bayer(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_YUV_GAIN_OFFSET)
		sun6i_isp_params_configure_yuv_gain_offset(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_CNR)
		sun6i_isp_params_configure_cnr(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_CONTRAST)
		sun6i_isp_params_configure_contrast(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_SATURATION)
		sun6i_isp_params_configure_saturation(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_SHARP)
		sun6i_isp_params_configure_sharp(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_GAMMA)
		sun6i_isp_params_configure_gamma(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_LSC)
		sun6i_isp_params_configure_lsc(isp_dev, config);

	if (modules & (SUN6I_ISP_MODULE_DRC | SUN6I_ISP_MODULE_YUV_DRC))
		sun6i_isp_params_configure_drc(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_AE)
		sun6i_isp_params_configure_ae(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_AF)
		sun6i_isp_params_configure_af(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_HIST)
		sun6i_isp_params_configure_hist(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_AWB)
		sun6i_isp_params_configure_awb(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_AFS)
		sun6i_isp_params_configure_afs(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_RGB2RGB)
		sun6i_isp_params_configure_rgb2rgb(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_DPC)
		sun6i_isp_params_configure_dpc(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_CFA)
		sun6i_isp_params_configure_cfa(isp_dev, config);

	if (modules & SUN6I_ISP_MODULE_WB)
		sun6i_isp_params_configure_wb_gains(isp_dev, config);

	value = sun6i_isp_load_read(isp_dev, SUN6I_ISP_MODULE_EN_REG);
	/*
	 * Clear all modules but keep input configuration. The rotator enable
	 * lives in this register too and belongs to the capture side, which
	 * programmed it at stream start: it is not a tuning module and a
	 * params buffer arriving mid-stream must not turn it off.
	 */
	value &= SUN6I_ISP_MODULE_EN_SRC0 | SUN6I_ISP_MODULE_EN_SRC1 |
		 SUN6I_ISP_MODULE_EN_ROT;

	if (modules & SUN6I_ISP_MODULE_OBC)
		value |= SUN6I_ISP_MODULE_EN_OBC;

	if (modules & SUN6I_ISP_MODULE_BDNF)
		value |= SUN6I_ISP_MODULE_EN_BDNF;

	if (modules & SUN6I_ISP_MODULE_YUV_GAIN_OFFSET)
		value |= SUN6I_ISP_MODULE_EN_YUV_GAIN_OFFSET;

	if (modules & SUN6I_ISP_MODULE_CNR)
		value |= SUN6I_ISP_MODULE_EN_CNR;

	if (modules & SUN6I_ISP_MODULE_CONTRAST)
		value |= SUN6I_ISP_MODULE_EN_CONTRAST;

	if (modules & SUN6I_ISP_MODULE_SATURATION)
		value |= SUN6I_ISP_MODULE_EN_SATU;

	if (modules & SUN6I_ISP_MODULE_SHARP)
		value |= SUN6I_ISP_MODULE_EN_SAP;

	if (modules & SUN6I_ISP_MODULE_LSC)
		value |= SUN6I_ISP_MODULE_EN_LSC;

	if (modules & SUN6I_ISP_MODULE_DRC)
		value |= SUN6I_ISP_MODULE_EN_RGB_DRC;

	if (modules & SUN6I_ISP_MODULE_AE)
		value |= SUN6I_ISP_MODULE_EN_AE;

	if (modules & SUN6I_ISP_MODULE_AF)
		value |= SUN6I_ISP_MODULE_EN_AF;

	if (modules & SUN6I_ISP_MODULE_HIST)
		value |= SUN6I_ISP_MODULE_EN_HIST;

	if (modules & SUN6I_ISP_MODULE_AWB)
		value |= SUN6I_ISP_MODULE_EN_AWB;

	if (modules & SUN6I_ISP_MODULE_AFS)
		value |= SUN6I_ISP_MODULE_EN_AFS;

	if (modules & SUN6I_ISP_MODULE_RGB2RGB)
		value |= SUN6I_ISP_MODULE_EN_RGB2RGB;

	if (modules & SUN6I_ISP_MODULE_WB)
		value |= SUN6I_ISP_MODULE_EN_WB;

	if (modules & SUN6I_ISP_MODULE_DPC)
		value |= SUN6I_ISP_MODULE_EN_DPC_OTF;

	/*
	 * The second dynamic range compression, applied in the YUV domain
	 * rather than the RGB one. It reads the same table as the first.
	 */
	if (modules & SUN6I_ISP_MODULE_YUV_DRC)
		value |= SUN6I_ISP_MODULE_EN_YUV_DRC;

	/* The demosaic has no enable bit; it is configuration only. */

	/* Bayer stage is always enabled. */

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_MODULE_EN_REG, value);
}

u32 sun6i_isp_params_table_update_take(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_params_state *state = &isp_dev->params.state;
	unsigned long flags;
	u32 update;

	spin_lock_irqsave(&state->lock, flags);

	update = state->table_update;
	state->table_update = 0;

	spin_unlock_irqrestore(&state->lock, flags);

	return update;
}

void sun6i_isp_params_configure(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_params_state *state = &isp_dev->params.state;
	bool bayer = sun6i_isp_proc_bayer(isp_dev);
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	sun6i_isp_params_configure_base(isp_dev);

	/*
	 * The default configuration is applied at the first stream start, and
	 * again whenever the input changes between bayer and YUV, because
	 * which modules exist depends on that - sun6i_isp_params_configure_
	 * modules() drops the bayer domain ones on a YUV input. Applying it
	 * only once left those enables describing whatever the first stream
	 * happened to be, which for a driver that can switch between a bayer
	 * and a YUV sensor is a register set that no longer matches the
	 * hardware it is driving.
	 */
	if (state->configured && state->configured_bayer == bayer)
		goto complete;

	sun6i_isp_params_configure_modules(isp_dev,
					   &sun6i_isp_params_config_default);

	state->configured = true;
	state->configured_bayer = bayer;

complete:
	spin_unlock_irqrestore(&state->lock, flags);
}

/* State */

static void sun6i_isp_params_state_cleanup(struct sun6i_isp_device *isp_dev,
					   bool error)
{
	struct sun6i_isp_params_state *state = &isp_dev->params.state;
	struct sun6i_isp_buffer *isp_buffer;
	struct vb2_buffer *vb2_buffer;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	if (state->pending) {
		vb2_buffer = &state->pending->v4l2_buffer.vb2_buf;
		vb2_buffer_done(vb2_buffer, error ? VB2_BUF_STATE_ERROR :
				VB2_BUF_STATE_QUEUED);

		state->pending = NULL;
	}

	list_for_each_entry(isp_buffer, &state->queue, list) {
		vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;
		vb2_buffer_done(vb2_buffer, error ? VB2_BUF_STATE_ERROR :
				VB2_BUF_STATE_QUEUED);
	}

	INIT_LIST_HEAD(&state->queue);

	spin_unlock_irqrestore(&state->lock, flags);
}

void sun6i_isp_params_state_update(struct sun6i_isp_device *isp_dev,
				   bool *update)
{
	struct sun6i_isp_params_state *state = &isp_dev->params.state;
	struct sun6i_isp_buffer *isp_buffer;
	struct vb2_buffer *vb2_buffer;
	const struct sun6i_isp_params_config *config;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	if (list_empty(&state->queue))
		goto complete;

	if (state->pending)
		goto complete;

	isp_buffer = list_first_entry(&state->queue, struct sun6i_isp_buffer,
				      list);

	vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;
	config = vb2_plane_vaddr(vb2_buffer, 0);

	sun6i_isp_params_configure_modules(isp_dev, config);

	list_del(&isp_buffer->list);

	state->pending = isp_buffer;

	if (update)
		*update = true;

complete:
	spin_unlock_irqrestore(&state->lock, flags);
}

void sun6i_isp_params_state_complete(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_params_state *state = &isp_dev->params.state;
	struct sun6i_isp_buffer *isp_buffer;
	struct vb2_buffer *vb2_buffer;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	if (!state->pending)
		goto complete;

	isp_buffer = state->pending;
	vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;

	vb2_buffer->timestamp = ktime_get_ns();

	/* Parameters will be applied starting from the next frame. */
	isp_buffer->v4l2_buffer.sequence =
		isp_dev->captures[SUN6I_ISP_CAPTURE_MAIN].state.sequence + 1;

	vb2_buffer_done(vb2_buffer, VB2_BUF_STATE_DONE);

	state->pending = NULL;

complete:
	spin_unlock_irqrestore(&state->lock, flags);
}

/* Queue */

static int sun6i_isp_params_queue_setup(struct vb2_queue *queue,
					unsigned int *buffers_count,
					unsigned int *planes_count,
					unsigned int sizes[],
					struct device *alloc_devs[])
{
	struct sun6i_isp_device *isp_dev = vb2_get_drv_priv(queue);
	unsigned int size = isp_dev->params.format.fmt.meta.buffersize;

	if (*planes_count)
		return sizes[0] < size ? -EINVAL : 0;

	*planes_count = 1;
	sizes[0] = size;

	return 0;
}

static int sun6i_isp_params_buffer_prepare(struct vb2_buffer *vb2_buffer)
{
	struct sun6i_isp_device *isp_dev =
		vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct v4l2_device *v4l2_dev = &isp_dev->v4l2.v4l2_dev;
	unsigned int size = isp_dev->params.format.fmt.meta.buffersize;

	if (vb2_plane_size(vb2_buffer, 0) < size) {
		v4l2_err(v4l2_dev, "buffer too small (%lu < %u)\n",
			 vb2_plane_size(vb2_buffer, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb2_buffer, 0, size);

	return 0;
}

static void sun6i_isp_params_buffer_queue(struct vb2_buffer *vb2_buffer)
{
	struct sun6i_isp_device *isp_dev =
		vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct sun6i_isp_params_state *state = &isp_dev->params.state;
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);
	struct sun6i_isp_buffer *isp_buffer =
		container_of(v4l2_buffer, struct sun6i_isp_buffer, v4l2_buffer);
	bool capture_streaming = sun6i_isp_capture_streaming(isp_dev);
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);
	list_add_tail(&isp_buffer->list, &state->queue);
	spin_unlock_irqrestore(&state->lock, flags);

	if (state->streaming && capture_streaming)
		sun6i_isp_state_update(isp_dev, false);
}

static int sun6i_isp_params_start_streaming(struct vb2_queue *queue,
					    unsigned int count)
{
	struct sun6i_isp_device *isp_dev = vb2_get_drv_priv(queue);
	struct sun6i_isp_params_state *state = &isp_dev->params.state;
	bool capture_streaming = sun6i_isp_capture_streaming(isp_dev);

	state->streaming = true;

	/*
	 * Update the state as soon as possible if capture is streaming,
	 * otherwise it will be applied when capture starts streaming.
	 */

	if (capture_streaming)
		sun6i_isp_state_update(isp_dev, false);

	return 0;
}

static void sun6i_isp_params_stop_streaming(struct vb2_queue *queue)
{
	struct sun6i_isp_device *isp_dev = vb2_get_drv_priv(queue);
	struct sun6i_isp_params_state *state = &isp_dev->params.state;

	state->streaming = false;
	sun6i_isp_params_state_cleanup(isp_dev, true);
}

static const struct vb2_ops sun6i_isp_params_queue_ops = {
	.queue_setup		= sun6i_isp_params_queue_setup,
	.buf_prepare		= sun6i_isp_params_buffer_prepare,
	.buf_queue		= sun6i_isp_params_buffer_queue,
	.start_streaming	= sun6i_isp_params_start_streaming,
	.stop_streaming		= sun6i_isp_params_stop_streaming,
};

/* Video Device */

static int sun6i_isp_params_querycap(struct file *file, void *priv,
				     struct v4l2_capability *capability)
{
	struct sun6i_isp_device *isp_dev = video_drvdata(file);
	struct video_device *video_dev = &isp_dev->params.video_dev;

	strscpy(capability->driver, SUN6I_ISP_NAME, sizeof(capability->driver));
	strscpy(capability->card, video_dev->name, sizeof(capability->card));
	snprintf(capability->bus_info, sizeof(capability->bus_info),
		 "platform:%s", dev_name(isp_dev->dev));

	return 0;
}

static int sun6i_isp_params_enum_fmt(struct file *file, void *priv,
				     struct v4l2_fmtdesc *fmtdesc)
{
	struct sun6i_isp_device *isp_dev = video_drvdata(file);
	struct v4l2_meta_format *params_format =
		&isp_dev->params.format.fmt.meta;

	if (fmtdesc->index > 0)
		return -EINVAL;

	fmtdesc->pixelformat = params_format->dataformat;

	return 0;
}

static int sun6i_isp_params_g_fmt(struct file *file, void *priv,
				  struct v4l2_format *format)
{
	struct sun6i_isp_device *isp_dev = video_drvdata(file);

	*format = isp_dev->params.format;

	return 0;
}

static const struct v4l2_ioctl_ops sun6i_isp_params_ioctl_ops = {
	.vidioc_querycap		= sun6i_isp_params_querycap,

	.vidioc_enum_fmt_meta_out	= sun6i_isp_params_enum_fmt,
	.vidioc_g_fmt_meta_out		= sun6i_isp_params_g_fmt,
	.vidioc_s_fmt_meta_out		= sun6i_isp_params_g_fmt,
	.vidioc_try_fmt_meta_out	= sun6i_isp_params_g_fmt,

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

static const struct v4l2_file_operations sun6i_isp_params_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= video_ioctl2,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
};

/* Media Entity */

/*
 * The params node is a video output device feeding a sub-device sink pad, so
 * the pipeline asks it to validate its own link. It carries metadata with a
 * fixed format and no geometry, so there is nothing to disagree about --
 * but the operation has to exist, or v4l2_subdev_link_validate() calls the
 * driver a bug and skips the check.
 */
static int sun6i_isp_params_link_validate(struct media_link *link)
{
	return 0;
}

static const struct media_entity_operations sun6i_isp_params_entity_ops = {
	.link_validate	= sun6i_isp_params_link_validate,
};

/* Params */

int sun6i_isp_params_setup(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_params *params = &isp_dev->params;
	struct sun6i_isp_params_state *state = &params->state;
	struct v4l2_device *v4l2_dev = &isp_dev->v4l2.v4l2_dev;
	struct v4l2_subdev *proc_subdev = &isp_dev->proc.subdev;
	struct video_device *video_dev = &params->video_dev;
	struct vb2_queue *queue = &isp_dev->params.queue;
	struct media_pad *pad = &isp_dev->params.pad;
	struct v4l2_format *format = &isp_dev->params.format;
	struct v4l2_meta_format *params_format = &format->fmt.meta;
	int ret;

	/* State */

	INIT_LIST_HEAD(&state->queue);
	spin_lock_init(&state->lock);

	/* Media Pads */

	pad->flags = MEDIA_PAD_FL_SOURCE | MEDIA_PAD_FL_MUST_CONNECT;

	video_dev->entity.ops = &sun6i_isp_params_entity_ops;

	ret = media_entity_pads_init(&video_dev->entity, 1, pad);
	if (ret)
		goto error_mutex;

	/* Queue */

	mutex_init(&params->lock);

	queue->type = V4L2_BUF_TYPE_META_OUTPUT;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	queue->buf_struct_size = sizeof(struct sun6i_isp_buffer);
	queue->ops = &sun6i_isp_params_queue_ops;
	queue->mem_ops = &vb2_vmalloc_memops;
	queue->min_queued_buffers = 1;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock = &params->lock;
	queue->dev = isp_dev->dev;
	queue->drv_priv = isp_dev;

	ret = vb2_queue_init(queue);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to initialize vb2 queue: %d\n", ret);
		goto error_media_entity;
	}

	/* V4L2 Format */

	format->type = queue->type;
	params_format->dataformat = V4L2_META_FMT_SUN6I_ISP_PARAMS;
	params_format->buffersize = sizeof(struct sun6i_isp_params_config);

	/* Video Device */

	strscpy(video_dev->name, SUN6I_ISP_PARAMS_NAME,
		sizeof(video_dev->name));
	video_dev->device_caps = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING;
	video_dev->vfl_dir = VFL_DIR_TX;
	video_dev->release = video_device_release_empty;
	video_dev->fops = &sun6i_isp_params_fops;
	video_dev->ioctl_ops = &sun6i_isp_params_ioctl_ops;
	video_dev->v4l2_dev = v4l2_dev;
	video_dev->queue = queue;
	video_dev->lock = &params->lock;

	video_set_drvdata(video_dev, isp_dev);

	ret = video_register_device(video_dev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to register video device: %d\n",
			 ret);
		goto error_media_entity;
	}

	/* Media Pad Link */

	ret = media_create_pad_link(&video_dev->entity, 0,
				    &proc_subdev->entity,
				    SUN6I_ISP_PROC_PAD_SINK_PARAMS,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed to create %s:%u -> %s:%u link\n",
			 video_dev->entity.name, 0, proc_subdev->entity.name,
			 SUN6I_ISP_PROC_PAD_SINK_PARAMS);
		goto error_video_device;
	}

	return 0;

error_video_device:
	vb2_video_unregister_device(video_dev);

error_media_entity:
	media_entity_cleanup(&video_dev->entity);

error_mutex:
	mutex_destroy(&params->lock);

	return ret;
}

void sun6i_isp_params_cleanup(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_params *params = &isp_dev->params;
	struct video_device *video_dev = &params->video_dev;

	vb2_video_unregister_device(video_dev);
	media_entity_cleanup(&video_dev->entity);
	mutex_destroy(&params->lock);
}
