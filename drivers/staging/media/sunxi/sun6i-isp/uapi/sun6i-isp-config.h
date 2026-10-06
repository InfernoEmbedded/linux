/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT) */
/*
 * Allwinner A31 ISP Configuration
 */

#ifndef _UAPI_SUN6I_ISP_CONFIG_H
#define _UAPI_SUN6I_ISP_CONFIG_H

#include <linux/types.h>

#define V4L2_META_FMT_SUN6I_ISP_PARAMS v4l2_fourcc('S', '6', 'I', 'P') /* Allwinner A31 ISP Parameters */

#define SUN6I_ISP_MODULE_BAYER			(1U << 0)
#define SUN6I_ISP_MODULE_BDNF			(1U << 1)
#define SUN6I_ISP_MODULE_YUV_GAIN_OFFSET	(1U << 2)
#define SUN6I_ISP_MODULE_CNR			(1U << 3)
#define SUN6I_ISP_MODULE_CONTRAST		(1U << 4)
#define SUN6I_ISP_MODULE_SATURATION		(1U << 5)
#define SUN6I_ISP_MODULE_SHARP			(1U << 6)
#define SUN6I_ISP_MODULE_RGB2RGB		(1U << 7)
#define SUN6I_ISP_MODULE_GAMMA			(1U << 8)
#define SUN6I_ISP_MODULE_LSC			(1U << 9)
#define SUN6I_ISP_MODULE_DRC			(1U << 10)
#define SUN6I_ISP_MODULE_AE			(1U << 11)
#define SUN6I_ISP_MODULE_AF			(1U << 12)
#define SUN6I_ISP_MODULE_HIST			(1U << 13)
#define SUN6I_ISP_MODULE_AWB			(1U << 14)
#define SUN6I_ISP_MODULE_AFS			(1U << 15)
#define SUN6I_ISP_MODULE_CFA			(1U << 16)
#define SUN6I_ISP_MODULE_WB			(1U << 17)
#define SUN6I_ISP_MODULE_DPC			(1U << 18)
#define SUN6I_ISP_MODULE_YUV_DRC		(1U << 19)
#define SUN6I_ISP_MODULE_OBC			(1U << 20)

struct sun6i_isp_params_config_bayer {
	__u16	offset_r;
	__u16	offset_gr;
	__u16	offset_gb;
	__u16	offset_b;

	__u16	gain_r;
	__u16	gain_gr;
	__u16	gain_gb;
	__u16	gain_b;
};

/*
 * Optical black clamp. The hardware carries a full geometry for it -- a
 * frame size, a valid window inside that frame, and a sprite -- because
 * the block is meant to average a covered region of the sensor and
 * subtract it. Nothing here has ever been exercised on this variant, so
 * the fields are offered rather than interpreted.
 */
struct sun6i_isp_params_config_obc {
	__u16	valid_width;
	__u16	valid_height;
	__u16	valid_start_horz;
	__u16	valid_start_vert;

	__u16	sprite_width;
	__u16	sprite_height;
	__u16	sprite_start_horz;
	__u16	sprite_start_vert;

	__u32	cfg;
	__u32	horz_pos;
	__u32	vert_para;
	__u32	offset_fixed;

	/* Per-channel black level, in the mosaic's own order. */
	__u32	clamp_r;
	__u32	clamp_gr;
	__u32	clamp_gb;
	__u32	clamp_b;
};

struct sun6i_isp_params_config_bdnf {
	__u8	in_dis_min;
	__u8	in_dis_max;

	__u8	coefficients_g[7];
	__u8	coefficients_rb[5];
};

/*
 * Shares hardware registers with the bayer gain/offset, so BAYER and
 * YUV_GAIN_OFFSET are mutually exclusive in modules_used.
 */
struct sun6i_isp_params_config_yuv_gain_offset {
	__u16	gain_y;
	__u16	gain_u;
	__u16	gain_v;

	__u16	offset_y;
	__u16	offset_u;
	__u16	offset_v;
};

struct sun6i_isp_params_config_cnr {
	__u16	offset;
};

struct sun6i_isp_params_config_contrast {
	__u8	level;
	__u8	min;
	__u8	max;
};

struct sun6i_isp_params_config_saturation {
	__u8	saturation_r;
	__u8	saturation_g;
	__u8	saturation_b;

	__u16	gain;
};

struct sun6i_isp_params_config_sharp {
	__u16	level;

	__u8	min;
	__u8	max;
};

/* A 3x3 colour correction matrix and a per-channel offset. */
struct sun6i_isp_params_config_rgb2rgb {
	__u16	matrix[3][3];
	__u16	offset[3];
};

/* Gamma curve, one 16-bit entry per input level. */
struct sun6i_isp_params_config_gamma {
	__u16	table[256];
};

/*
 * Lens shading: the centre the falloff is measured from, a scale for how
 * fast it runs, and a per-channel correction curve along the radius.
 */
struct sun6i_isp_params_config_lsc {
	__u16	center_x;
	__u16	center_y;
	__u16	scale;

	__u16	table_r[256];
	__u16	table_g[256];
	__u16	table_b[256];
};

/* Dynamic range compression curve, one 16-bit entry per input level. */
struct sun6i_isp_params_config_drc {
	__u16	table[256];
};

/*
 * Geometry of the grid the statistics are gathered over. Sizes and
 * positions are in pixels; the hardware wants them halved, which the
 * driver does.
 *
 * width and height give the size of ONE ZONE, not of the whole area
 * measured. The area is the zone times the zone count, so a caller that
 * wants to cover a frame asks for frame_width / horz_num here. Passing
 * the frame size asks for a grid horz_num times too wide, which runs
 * off the sensor and quietly measures nothing useful.
 *
 * This is measured, not assumed. Holding the size and sweeping
 * horz_num over 1, 2, 4 and 8 leaves every exposure zone sum bit
 * identical -- a per-zone pixel count cannot depend on how many zones
 * there are unless the register is the zone. The vendor agrees:
 * HandleAeStat() divides each zone by the width times the height read
 * back out of the hardware's own saved AE_SIZE.
 *
 * A zone spans width by height pixels of the frame but only counts
 * width by height/2 of them, sampling every other line: the sum is
 * 254 * width * height / 2 to the pixel on a white field, for every
 * width from 64 to 384.
 *
 * The grid therefore reaches horz_num * width by vert_num * height, and
 * that has to fit the sensor. Tiling 1600 by 1200 with an 8 by 8 grid
 * means 200 by 150, which gives all 64 zones at 15000 pixels. Ask for
 * 200 by 300 instead and the grid wants 2400 lines, so only the four
 * rows that fit are written and the rest of the array keeps whatever it
 * held before -- the hardware reports no error for this.
 */
struct sun6i_isp_params_config_h3a_window {
	__u8	horz_num;
	__u8	vert_num;

	__u16	width;
	__u16	height;
	__u16	horz_start;
	__u16	vert_start;
};

struct sun6i_isp_params_config_ae {
	struct sun6i_isp_params_config_h3a_window	window;

	__u16	low_brightness_threshold;
	__u16	high_brightness_threshold;
};

struct sun6i_isp_params_config_af {
	struct sun6i_isp_params_config_h3a_window	window;
};

/* No zone grid: the histogram covers its window as a whole. */
struct sun6i_isp_params_config_hist {
	struct sun6i_isp_params_config_h3a_window	window;

	__u8	mode;
};

/*
 * White balance gathers over a window with no zone grid of its own, and
 * takes a pair of saturation limits. The zone counts in the window are
 * ignored here.
 */
struct sun6i_isp_params_config_awb {
	struct sun6i_isp_params_config_h3a_window	window;

	__u16	saturation[2];
};

/* Flicker avoidance, a single tuning value. */
struct sun6i_isp_params_config_afs {
	__u8	anti_flicker;
};

/*
 * Defect pixel correction, the on-the-fly kind that needs no defect list.
 * A pixel is corrected when it stands further from its neighbours than the
 * thresholds allow; the slope sets how the limit grows with the local
 * level, and the mode selects between the two comparisons the hardware
 * offers.
 */
struct sun6i_isp_params_config_dpc {
	__u16	threshold_slope;
	__u16	threshold_min;
	__u16	threshold_max;

	__u8	mode;
};

/*
 * Demosaic. The threshold decides how strongly the interpolation commits
 * to an edge direction; the minimum is a floor applied to the channels,
 * and the hardware writes its own measurement back over the same bits, so
 * reading these registers returns a level rather than what was written.
 *
 * The minimum occupies three separate 10-bit fields across two registers.
 * Every caller that has ever been seen -- the vendor library and this
 * driver before it -- writes the same value into all three, so whether
 * they do different things has never been tested. They are separate here
 * so that it can be. Set all three alike for the vendor's behaviour.
 *
 * texture goes to CFA_TEX. That register is a measurement the hardware
 * writes back; nothing is known to read it and nothing is known to write
 * it, so this exists to find out whether writing has any effect at all.
 */
struct sun6i_isp_params_config_cfa {
	__u16	direction_threshold;
	__u16	min_rgb;		/* CFA_CFG0, bits 25:16 */
	__u16	min_rgb_low;		/* CFA_CFG1, bits 9:0 */
	__u16	min_rgb_high;		/* CFA_CFG1, bits 25:16 */
	__u32	texture;		/* CFA_TEX */
};

/*
 * White balance correction, as opposed to the measurement in awb. Gains
 * are 1.8 fixed point, so 256 is unity, and the clip is the ceiling the
 * result is held to.
 */
struct sun6i_isp_params_config_wb {
	__u16	gain_r;
	__u16	gain_gr;
	__u16	gain_gb;
	__u16	gain_b;

	__u16	clip;
};

struct sun6i_isp_params_config {
	__u32					modules_used;

	struct sun6i_isp_params_config_bayer	bayer;
	struct sun6i_isp_params_config_bdnf	bdnf;
	struct sun6i_isp_params_config_yuv_gain_offset	yuv_gain_offset;
	struct sun6i_isp_params_config_cnr	cnr;
	struct sun6i_isp_params_config_contrast	contrast;
	struct sun6i_isp_params_config_saturation	saturation;
	struct sun6i_isp_params_config_sharp	sharp;
	struct sun6i_isp_params_config_rgb2rgb	rgb2rgb;
	struct sun6i_isp_params_config_gamma	gamma;
	struct sun6i_isp_params_config_lsc	lsc;
	struct sun6i_isp_params_config_drc	drc;
	struct sun6i_isp_params_config_ae	ae;
	struct sun6i_isp_params_config_af	af;
	struct sun6i_isp_params_config_hist	hist;
	struct sun6i_isp_params_config_awb	awb;
	struct sun6i_isp_params_config_afs	afs;
	struct sun6i_isp_params_config_cfa	cfa;
	struct sun6i_isp_params_config_wb	wb;
	struct sun6i_isp_params_config_dpc	dpc;
	struct sun6i_isp_params_config_obc	obc;
};

/*
 * Statistics
 *
 * Each block below is a straight copy of the region the hardware DMAs
 * into, so the layouts here are the hardware's, not a repacking.
 *
 * These are TENTATIVE. The record sizes and counts are read out of the
 * vendor parsers, but which sum is which colour is inferred from how the
 * vendor weights them, and the exposure block has an unresolved question
 * over it (see below). Expect the meanings, not the sizes, to be
 * corrected once this has been run against known input.
 */

#define V4L2_META_FMT_SUN6I_ISP_STATS v4l2_fourcc('S', '6', 'I', 'S') /* Allwinner A31 ISP Statistics */

#define SUN6I_ISP_STAT_HIST			(1U << 0)
#define SUN6I_ISP_STAT_AE			(1U << 1)
#define SUN6I_ISP_STAT_AF			(1U << 2)
#define SUN6I_ISP_STAT_AFS			(1U << 3)
#define SUN6I_ISP_STAT_AWB			(1U << 4)
#define SUN6I_ISP_STAT_CFA			(1U << 5)

/*
 * 128 bins over a 256 level range, so bin i covers luminance 2 * i + 1.
 * The vendor treats the top four bins as the clipped count.
 */
struct sun6i_isp_stats_hist {
	__u32	bins[128];
};

/*
 * Per-zone channel sums. The fourth word is not read by the vendor.
 *
 * The hardware writes one plane of 64 zones, which is the vendor's own
 * loop bound, and the array is sized for the region it is given rather
 * than for that. Only horz_num * vert_num of the 64 carry a
 * measurement, and fewer when the grid overruns the sensor; the rest
 * are zeroed. Measured on a white field with an 8 by 8 grid: zones 0 to
 * 63 all read 254 * width * height / 2 while the grid fits, and 64
 * onward were never written.
 *
 * The second and third planes the vendor points at zone 64 and beyond
 * the region entirely are still unexplained and are not exposed.
 */
struct sun6i_isp_stats_ae_zone {
	__u32	sum[3];
	__u32	unused;
};

struct sun6i_isp_stats_ae {
	struct sun6i_isp_stats_ae_zone	zones[96];
};

/*
 * The vendor pairs these as value[0] + value[2] and value[1] + value[3],
 * which are presumably two band energies, and never reads the header.
 */
struct sun6i_isp_stats_af_zone {
	__u32	value[4];
};

struct sun6i_isp_stats_af {
	__u8				header[256];
	struct sun6i_isp_stats_af_zone	zones[64];
};

/* Flicker detection samples. Element meaning not established. */
struct sun6i_isp_stats_afs {
	__u32	samples[128];
};

/*
 * Per-window channel sums and the pixel count they cover. The vendor
 * discards a window whose count is under 8 and otherwise normalises by
 * count >> 3. The origin depends on the bayer phase.
 *
 * The array is sized for the region the hardware is given, but only the
 * first 32 records are populated on the A83T; the rest are zeroed, so
 * the vendor's own count test rejects them. Those 32 are sixteen
 * columns by two rows.
 *
 * The window here is not the region measured. It gives the size of one
 * cell, and the grid of 16 by 12 cells is what spans the frame, so the
 * measured extent is four times the width by six times the height. A
 * window larger than a quarter of the frame width or a sixth of its
 * height therefore runs the grid off the frame and the counts stop
 * meaning anything -- 400 by 200 against a 1600 by 1200 sensor. Nothing
 * range-checks this.
 */
struct sun6i_isp_stats_awb_window {
	__u32	sum[3];
	__u32	count;
};

struct sun6i_isp_stats_awb {
	struct sun6i_isp_stats_awb_window	windows[256];
};

/*
 * blocks_valid says which of the following were enabled for the frame;
 * the rest are left zeroed. The sum ordering is believed to be red,
 * green, blue.
 */
/*
 * What the demosaic measured, taken from the save region rather than from
 * the statistics DMA -- the hardware reports these by writing back over
 * its own configuration registers.
 *
 * min_rgb is the minimum the block saw, which the vendor's 3A loop reads
 * every frame. texture is the whole of CFA_TEX; the vendor exports a
 * getter for it and never calls it, so what it counts is not known.
 *
 * The demosaic has no enable bit, so these are always reported.
 */
struct sun6i_isp_stats_cfa {
	__u32				min_rgb;
	__u32				texture;
};

struct sun6i_isp_stats_buffer {
	__u32				blocks_valid;
	__u32				sequence;

	struct sun6i_isp_stats_hist	hist;
	struct sun6i_isp_stats_ae	ae;
	struct sun6i_isp_stats_af	af;
	struct sun6i_isp_stats_afs	afs;
	struct sun6i_isp_stats_awb	awb;
	struct sun6i_isp_stats_cfa	cfa;
};

#endif /* _UAPI_SUN6I_ISP_CONFIG_H */
