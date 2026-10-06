/*
 * Galaxycore GC2145 driver.
 * Copyright (C) 2018 Ondřej Jirman <megi@xff.cz>.
 * Copyright (C) 2022, STMicroelectronics SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

//#define DEBUG

#include <asm/div64.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/*
 * GC2145
 * - 2M pixel
 * - 1600 x 1200, max frame rate: 720P, 30fps@96MHz
 * - Bayer RGB, RGB565, YCbCr 4:2:2
 * - AE, AWB
 * - PLL
 * - AVDD 2.7-3V, DVDD 1.7-1.9V, IOVDD 1.7-3V
 * - Power 180mW / 200uA standby
 * - Interpolation, denoise, gamma, edge enhance
 * I2C:
 * - write reg8
 * - read reg8
 * - write reg8 multi
 *
 * Power on:
 * MCLK on
 * PWDN, RESET low
 * IOVDD, AVDD, DVDD on in sequence
 * RESET high
 *
 * Power off:
 * PWDN, RESET low
 * RESET high
 * delay
 * PWDN high
 * RESET low
 * IOVDD, AVDD, DVDD off
 * PWDN low?
 * MCLK off
 *
 * Init:
 * - check chip id
 * - setup pll
 * - setup CSI interface / PAD drive strength
 * - setup resolution/fps
 * - enable postprocessing
 *   (ISP related chapter)
 *
 * Stream on:
 * - ???
 */

#define GC2145_FIRMWARE_PARAMETERS	"gc2145-init.bin"

#define GC2145_SENSOR_WIDTH_MIN		88u
#define GC2145_SENSOR_HEIGHT_MIN	72u

/*
 * The pixel array is 1616x1232, of which 1600x1200 carry a picture. The
 * dummy pixels around it, 8 per side horizontally and 16 vertically, are
 * read out with the rest and trimmed off by the ISP crop.
 */
#define GC2145_NATIVE_WIDTH		1616u
#define GC2145_NATIVE_HEIGHT		1232u
#define GC2145_ACTIVE_LEFT		8u
#define GC2145_ACTIVE_TOP		16u
#define GC2145_SENSOR_WIDTH_MAX		1600u
#define GC2145_SENSOR_HEIGHT_MAX	1200u

#define GC2145_DEFAULT_WIDTH		GC2145_SENSOR_WIDTH_MAX
#define GC2145_DEFAULT_HEIGHT		GC2145_SENSOR_HEIGHT_MAX
#define GC2145_DEFAULT_FRAMERATE	15

/*
 * One pixel clock period carries one byte out of the parallel port, so
 * a pixel takes two of them. 96 MHz is the highest rate the parallel
 * receivers this sensor is paired with are known to sample reliably,
 * and it divides the 24 MHz input clock exactly.
 */
#define GC2145_PCLK_RATE		96000000ul

/*
 * The GC2155 timing spreadsheet recommends keeping the horizontal
 * blanking above 0x1f0, but every vendor register table uses less than
 * that, and the frame rates they claim are only reachable below it.
 */
#define GC2145_HBLANK_REG_MIN		342ul
#define GC2145_VBLANK_REG_MIN		8ul
#define GC2145_VBLANK_REG_MAX		4092ul
#define GC2145_SH_DELAY			46ul

/* the exposure register counts array row slots and is 13 bits wide */
#define GC2145_EXPOSURE_REG_MAX		0x1fffu

/* {{{ Register definitions */

/* system registers */
#define GC2145_REG_CHIP_ID			0xf0
#define GC2145_REG_CHIP_ID_VALUE		0x2145

#define GC2145_REG_PAD_IO		0xf2
#define GC2145_REG_PLL_MODE1		0xf7
#define GC2145_REG_PLL_MODE2		0xf8
#define GC2145_REG_CM_MODE		0xf9
#define GC2145_REG_CLK_DIV_MODE		0xfa
#define GC2145_REG_ANALOG_PWC		0xfc
#define GC2145_REG_SCALER_MODE		0xfd
#define GC2145_REG_RESET		0xfe

#define GC2145_P0_EXPOSURE_HI		0x03
#define GC2145_P0_EXPOSURE_LO		0x04
#define GC2145_P0_HBLANK_DELAY_HI	0x05
#define GC2145_P0_HBLANK_DELAY_LO	0x06
#define GC2145_P0_VBLANK_DELAY_HI	0x07
#define GC2145_P0_VBLANK_DELAY_LO	0x08
#define GC2145_P0_ROW_START_HI		0x09
#define GC2145_P0_ROW_START_LO		0x0a
#define GC2145_P0_COL_START_HI		0x0b
#define GC2145_P0_COL_START_LO		0x0c
#define GC2145_P0_WIN_HEIGHT_HI		0x0d
#define GC2145_P0_WIN_HEIGHT_LO		0x0e
#define GC2145_P0_WIN_WIDTH_HI		0x0f
#define GC2145_P0_WIN_WIDTH_LO		0x10
#define GC2145_P0_SH_DELAY_HI		0x11
#define GC2145_P0_SH_DELAY_LO		0x12
#define GC2145_P0_START_TIME		0x13
#define GC2145_P0_END_TIME		0x14

#define GC2145_P0_ISP_BLK_ENABLE1	0x80
#define GC2145_ISP_BLK_ENABLE1_LSC		BIT(0)
#define GC2145_ISP_BLK_ENABLE1_DD		BIT(1)
#define GC2145_ISP_BLK_ENABLE1_DN		BIT(2)
#define GC2145_ISP_BLK_ENABLE1_INTERPOLATION	BIT(3)
#define GC2145_ISP_BLK_ENABLE1_EE		BIT(4)
#define GC2145_ISP_BLK_ENABLE1_CC		BIT(5)
#define GC2145_ISP_BLK_ENABLE1_GAMMA		BIT(6)
#define GC2145_P0_ISP_BLK_ENABLE2	0x81
#define GC2145_P0_ISP_BLK_ENABLE3	0x82
#define GC2145_ISP_BLK_ENABLE3_AWB		BIT(1)
#define GC2145_P0_ISP_SPECIAL_EFFECT	0x83
#define GC2145_SPECIAL_EFFECT_INVERT		BIT(0)
#define GC2145_SPECIAL_EFFECT_FIXED_CBCR	BIT(1)
#define GC2145_SPECIAL_EFFECT_EDGE_MAP1		BIT(2)
#define GC2145_SPECIAL_EFFECT_EDGE_MAP2		BIT(3)
#define GC2145_SPECIAL_EFFECT_SELECT		GENMASK(7, 4)
#define GC2145_P0_ISP_OUT_FORMAT	0x84
#define GC2145_P0_FRAME_START		0x85
#define GC2145_P0_SYNC_MODE		0x86
#define GC2145_SYNC_MODE_COL_SWITCH	BIT(4)
#define GC2145_SYNC_MODE_ROW_SWITCH	BIT(5)
#define GC2145_P0_ISP_BLK_ENABLE4	0x87
#define GC2145_P0_ISP_MODULE_GATING	0x88
#define GC2145_P0_ISP_BYPASS_MODE	0x89
#define GC2145_P0_DEBUG_MODE2		0x8c
#define GC2145_DEBUG_MODE2_TEST_IMAGE		BIT(2)
#define GC2145_DEBUG_MODE2_TEST_IMAGE_UXGA	BIT(3)
#define GC2145_DEBUG_MODE2_SKIN_MAP		BIT(4)
#define GC2145_P0_DEBUG_MODE3		0x8d
#define GC2145_DEBUG_MODE3_UPDATE_GAIN		BIT(0)
#define GC2145_DEBUG_MODE3_FIX_VALUE_MODE	BIT(3)

#define GC2145_P0_CROP_ENABLE		0x90
#define GC2145_P0_CROP_Y1_HI		0x91
#define GC2145_P0_CROP_Y1_LO		0x92
#define GC2145_P0_CROP_X1_HI		0x93
#define GC2145_P0_CROP_X1_LO		0x94
#define GC2145_P0_CROP_WIN_HEIGHT_HI	0x95
#define GC2145_P0_CROP_WIN_HEIGHT_LO	0x96
#define GC2145_P0_CROP_WIN_WIDTH_HI	0x97
#define GC2145_P0_CROP_WIN_WIDTH_LO	0x98

#define GC2145_P0_SUBSAMPLE_RATIO	0x99
#define GC2145_P0_SUBSAMPLE_MODE	0x9a
/* interpolate rather than drop, on both luma and chroma */
#define GC2145_SUBSAMPLE_MODE_SMOOTH		0x0e
#define GC2145_P0_SUB_ROW_N1		0x9b
#define GC2145_P0_SUB_ROW_N2		0x9c
#define GC2145_P0_SUB_ROW_N3		0x9d
#define GC2145_P0_SUB_ROW_N4		0x9e
#define GC2145_P0_SUB_COL_N1		0x9f
#define GC2145_P0_SUB_COL_N2		0xa0
#define GC2145_P0_SUB_COL_N3		0xa1
#define GC2145_P0_SUB_COL_N4		0xa2

/*
 * Gain stages, in the order the sensor applies them. Only the global
 * gain is ours; the datasheet marks the other two read-only and driven
 * by the AEC, writable only while the AEC is off. There is no separate
 * analogue gain register documented, and the global gain is the one the
 * vendor code drives, so that is what the analogue gain control gets.
 * Its scaling is documented as "float 4.4", but both its reset value
 * and the vendor code agree that 0x40 is unity.
 */
#define GC2145_P0_GLOBAL_GAIN		0xb0
#define GC2145_GLOBAL_GAIN_UNITY		0x40
#define GC2145_P0_AUTO_PREGAIN		0xb1
#define GC2145_PREGAIN_UNITY			0x20
#define GC2145_P0_AUTO_POSTGAIN		0xb2

/*
 * The global gain and the pre gain are multiplied together in one place
 * and the product saturates at 16 unities. Measured: the output stops
 * responding once global * pre reaches that, the knee sits where the
 * product predicts it at three different pre gains, and it does not move
 * when the exposure changes by two stops - so this is a clamp on the
 * gain rather than the picture clipping. The post gain is applied after
 * the clamp and does not count towards it.
 */
#define GC2145_TOTAL_GAIN_MAX \
	(16 * GC2145_GLOBAL_GAIN_UNITY * GC2145_PREGAIN_UNITY)

/*
 * The largest global gain that still leaves the pre gain its whole
 * register range under that clamp, which makes it the one value that
 * costs nothing to default to. It is also worth twice as much as unity
 * to the automatic exposure, which drives the pre gain and nothing else:
 * at unity the AEC tops out at 8x total, here at the full 16x.
 */
#define GC2145_GLOBAL_GAIN_DEFAULT	(GC2145_TOTAL_GAIN_MAX / 256)

#define GC2145_P0_AWB_R_GAIN		0xb3
#define GC2145_P0_AWB_G_GAIN		0xb4
#define GC2145_P0_AWB_B_GAIN		0xb5
#define GC2145_AWB_GAIN_UNITY			0x40

#define GC2145_P0_AEC_ENABLE		0xb6

/* YCP, page 2 */
#define GC2145_P2_GLOBAL_SATURATION	0x2d0
#define GC2145_P2_SATURATION_CB		0x2d1
#define GC2145_P2_SATURATION_CR		0x2d2
#define GC2145_SATURATION_UNITY			0x20
#define GC2145_P2_LUMA_CONTRAST		0x2d3
#define GC2145_CONTRAST_UNITY			0x40
#define GC2145_P2_LUMA_OFFSET		0x2d5
#define GC2145_P2_FIXED_CB		0x2da
#define GC2145_P2_FIXED_CR		0x2db

/* interpolation and edge enhancement, page 2 */
#define GC2145_P2_EDGE_EFFECT		0x297

/* AEC, page 1 */
#define GC2145_P1_AEC_MODE2		0x10b
#define GC2145_AEC_MODE2_EVERY_N_FRAMES		GENMASK(6, 4)
#define GC2145_P1_AEC_MODE3		0x10c
#define GC2145_AEC_MODE3_CENTRE_WEIGHT		GENMASK(6, 4)
#define GC2145_P1_AEC_ANTI_FLICKER_HI	0x125
#define GC2145_P1_AEC_EXP_LEVEL(n)	(0x127 + 2 * (n))
#define GC2145_AEC_EXP_LEVELS			7
#define GC2145_P1_AEC_MAX_PREGAIN	0x11f
#define GC2145_P1_AEC_MAX_POSTGAIN	0x120
#define GC2145_AEC_MAX_POSTGAIN			0xe0
#define GC2145_P1_AEC_MAX_EXP_LEVEL	0x13c
#define GC2145_AEC_MAX_EXP_LEVEL		GENMASK(6, 5)
#define GC2145_P1_AEC_LEVEL_MAX_GAIN(n)	(0x135 + (n))
#define GC2145_P0_OUT_BUF_ENABLE	0xc2

/* }}} */

struct gc2145_pixfmt {
	u32 code;
	u32 colorspace;
	u8 fmt_setup;
	/* row and column switch bits of the sync register, raw only */
	u8 sync_switch;
};

static const struct gc2145_pixfmt gc2145_formats[] = {
	{
		.code              = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.fmt_setup         = 0x00,
	},
	{
		.code              = MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.fmt_setup         = 0x01,
	},
	{
		.code              = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.fmt_setup         = 0x02,
	},
	{
		.code              = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.fmt_setup         = 0x03,
	},
	{
		.code              = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.fmt_setup         = 0x06,
	},
	/*
	 * The raw output mode has the sync register switch which of the odd
	 * and even rows and columns comes out first, which moves the Bayer
	 * phase. BGGR is what the sensor gives with neither switch set.
	 *
	 * The two switches are named for the wrong axis: measured against a
	 * solid primary, the bit the datasheet calls the column switch moves
	 * the phase down a row and the one it calls the row switch moves it
	 * along a column. The pairings below are what the sensor does, not
	 * what the names suggest.
	 */
	{
		.code              = MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace        = V4L2_COLORSPACE_RAW,
		.fmt_setup         = 0x17,
		.sync_switch       = 0,
	},
	{
		.code              = MEDIA_BUS_FMT_SGRBG8_1X8,
		.colorspace        = V4L2_COLORSPACE_RAW,
		.fmt_setup         = 0x17,
		.sync_switch       = GC2145_SYNC_MODE_COL_SWITCH,
	},
	{
		.code              = MEDIA_BUS_FMT_SGBRG8_1X8,
		.colorspace        = V4L2_COLORSPACE_RAW,
		.fmt_setup         = 0x17,
		.sync_switch       = GC2145_SYNC_MODE_ROW_SWITCH,
	},
	{
		.code              = MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace        = V4L2_COLORSPACE_RAW,
		.fmt_setup         = 0x17,
		.sync_switch       = GC2145_SYNC_MODE_COL_SWITCH |
				     GC2145_SYNC_MODE_ROW_SWITCH,
	},
};

static bool gc2145_format_is_raw(u32 code)
{
	return code == MEDIA_BUS_FMT_SBGGR8_1X8 ||
	       code == MEDIA_BUS_FMT_SGBRG8_1X8 ||
	       code == MEDIA_BUS_FMT_SGRBG8_1X8 ||
	       code == MEDIA_BUS_FMT_SRGGB8_1X8;
}

static const struct gc2145_pixfmt *gc2145_find_format(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gc2145_formats); i++)
		if (gc2145_formats[i].code == code)
			return &gc2145_formats[i];

	return NULL;
}

/* regulator supplies */
static const char * const gc2145_supply_name[] = {
	"IOVDD", /* Digital I/O (1.7-3V) suppply */
	"AVDD",  /* Analog (2.7-3V) supply */
	"DVDD",  /* Digital Core (1.7-1.9V) supply */
};

#define GC2145_NUM_SUPPLIES ARRAY_SIZE(gc2145_supply_name)

struct gc2145_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct {
		struct v4l2_ctrl *auto_exposure;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *d_gain;
	};
	struct v4l2_ctrl *a_gain;
	struct v4l2_ctrl *metering;
	struct v4l2_ctrl *exposure_bias;
	struct {
		struct v4l2_ctrl *wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct v4l2_ctrl *aaa_lock;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *pl_freq;
	struct v4l2_ctrl *colorfx;
	struct v4l2_ctrl *colorfx_cbcr;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *sharpness;
	struct v4l2_ctrl *test_pattern;
};

enum {
	TX_WRITE = 1,
	TX_WRITE16,
	TX_UPDATE_BITS,
};

#define GC2145_MAX_OPS 64

struct gc2145_tx_op {
	int op;
	u16 reg;
	u16 val;
	u16 mask;
};

struct gc2145_mode;

struct gc2145_sensor_params {
	const struct gc2145_mode *mode;
	/*
	 * Row skipping halves the time an array row slot takes, so the
	 * exposure register, which counts those slots, has to be scaled by
	 * it. Nothing else in the decimation chain changes the timing.
	 */
	unsigned int slots_per_line;
	/* readout window, in pixel array coordinates */
	unsigned long row_start;
	unsigned long col_start;
	unsigned long win_width;
	unsigned long win_height;
	/* timing */
	unsigned long sh_delay;
	unsigned long hb;
	unsigned long vb;
	unsigned long st;
	unsigned long et;
	/* size the sensor emits, what the blanking is relative to */
	u32 out_width;
	u32 out_height;
};

struct gc2145_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* external clock for GC2145 */

	struct regulator_bulk_data supplies[GC2145_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio; // nrst pin
	struct gpio_desc *enable_gpio; // ce pin

	/*
	 * The sub-device state lock, which is the control handler lock,
	 * protects all members below.
	 */
	struct v4l2_fract frame_interval;
	struct gc2145_ctrls ctrls;

	/* the timing the controls currently describe */
	struct gc2145_sensor_params params;

	/* mains frequency the row period is fitted to, in Hz */
	unsigned long power_line_freq;

	bool pending_mode_change;
	bool flips_grabbed;
	/* a fixed-value test pattern must not be metered, see s_ctrl */
	bool test_pattern_holds_aec;

	u8 current_bank;

	struct gc2145_tx_op ops[GC2145_MAX_OPS];
	int n_ops;
	int tx_started;
};

static inline struct gc2145_dev *to_gc2145_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc2145_dev, sd);
}

/* {{{ Register access helpers */

static int gc2145_write_regs(struct gc2145_dev *sensor, u8 addr,
			     u8 *data, int data_size)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[128 + 1];
	int ret;

	if (data_size > sizeof(buf) - 1) {
		v4l2_err(&sensor->sd, "%s: oversized transfer (size=%d)\n",
			 __func__, data_size);
		return -EINVAL;
	}

	buf[0] = addr;
	memcpy(buf + 1, data, data_size);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = data_size + 1;

	dev_dbg(&sensor->i2c_client->dev, "[wr %02x] <= %*ph\n",
		(u32)addr, data_size, data);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l2_err(&sensor->sd,
			 "%s: error %d: addr=%x, data=%*ph\n",
			 __func__, ret, (u32)addr, data_size, data);
		return ret;
	}

	return 0;
}

static int gc2145_read_regs(struct gc2145_dev *sensor, u8 addr,
			    u8 *data, int data_size)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = &addr;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = data_size;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		v4l2_err(&sensor->sd,
			 "%s: error %d: start_index=%x, data_size=%d\n",
			 __func__, ret, (u32)addr, data_size);
		return ret;
	}

	dev_dbg(&sensor->i2c_client->dev, "[rd %02x] => %*ph\n",
		(u32)addr, data_size, data);

	return 0;
}

static int gc2145_switch_bank(struct gc2145_dev *sensor, u16 reg)
{
	int ret;
	u8 bank = reg >> 8;

	if (bank & ~3u)
		return -ERANGE;

	if (sensor->current_bank != bank) {
		ret = gc2145_write_regs(sensor, GC2145_REG_RESET, &bank, 1);
		if (ret)
			return ret;

		sensor->current_bank = bank;
		dev_dbg(&sensor->i2c_client->dev, "bank switch: 0x%02x\n",
				(unsigned int)sensor->current_bank);
	}

	return 0;
}

static int gc2145_read(struct gc2145_dev *sensor, u16 reg, u8 *val)
{
	int ret;

	ret = gc2145_switch_bank(sensor, reg);
	if (ret)
		return ret;

	return gc2145_read_regs(sensor, reg, val, 1);
}

static int gc2145_write(struct gc2145_dev *sensor, u16 reg, u8 val)
{
	int ret;

	ret = gc2145_switch_bank(sensor, reg);
	if (ret)
		return ret;

	if ((reg & 0xffu) == GC2145_REG_RESET)
		sensor->current_bank = val & 3;

	return gc2145_write_regs(sensor, reg, &val, 1);
}

static int gc2145_update_bits(struct gc2145_dev *sensor, u16 reg, u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	ret = gc2145_read(sensor, reg, &tmp);
	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= val & mask;

	return gc2145_write(sensor, reg, tmp);
}

static int gc2145_read16(struct gc2145_dev *sensor, u16 reg, u16 *val)
{
	int ret;

	ret = gc2145_switch_bank(sensor, reg);
	if (ret)
		return ret;

	ret = gc2145_read_regs(sensor, reg, (u8 *)val, sizeof(*val));
	if (ret)
		return ret;

	*val = be16_to_cpu(*val);
	return 0;
}

static int gc2145_write16(struct gc2145_dev *sensor, u16 reg, u16 val)
{
	u16 tmp = cpu_to_be16(val);
	int ret;

	ret = gc2145_switch_bank(sensor, reg);
	if (ret)
		return ret;

	return gc2145_write_regs(sensor, reg, (u8 *)&tmp, sizeof(tmp));
}

static void gc2145_tx_start(struct gc2145_dev *sensor)
{
	if (sensor->tx_started++)
		dev_err(&sensor->i2c_client->dev,
				"tx_start called multiple times\n");

	sensor->n_ops = 0;
}

static void gc2145_tx_add(struct gc2145_dev *sensor, int kind,
			  u16 reg, u16 val, u16 mask)
{
	struct gc2145_tx_op *op;

	if (!sensor->tx_started) {
		dev_err(&sensor->i2c_client->dev,
				"op added without calling tx_start\n");
		return;
	}

	if (sensor->n_ops >= ARRAY_SIZE(sensor->ops)) {
		dev_err(&sensor->i2c_client->dev,
				"ops overflow, increase GC2145_MAX_OPS\n");
		return;
	}

	op = &sensor->ops[sensor->n_ops++];
	op->op = kind;
	op->reg = reg;
	op->val = val;
	op->mask = mask;
}

static void gc2145_tx_write8(struct gc2145_dev *sensor, u16 reg, u8 val)
{
	return gc2145_tx_add(sensor, TX_WRITE, reg, val, 0);
}

static void gc2145_tx_write16(struct gc2145_dev *sensor, u16 reg, u16 val)
{
	return gc2145_tx_add(sensor, TX_WRITE16, reg, val, 0);
}

static void gc2145_tx_update_bits(struct gc2145_dev *sensor, u16 reg,
				  u8 mask, u8 val)
{
	return gc2145_tx_add(sensor, TX_UPDATE_BITS, reg, val, mask);
}

static int gc2145_tx_commit(struct gc2145_dev *sensor)
{
	struct gc2145_tx_op* op;
	int i, ret, n_ops;

	if (!sensor->tx_started) {
		dev_err(&sensor->i2c_client->dev,
				"tx_commit called without tx_start\n");
		return 0;
	}

	n_ops = sensor->n_ops;
	sensor->tx_started = 0;
	sensor->n_ops = 0;

	for (i = 0; i < n_ops; i++) {
		op = &sensor->ops[i];

		switch (op->op) {
		case TX_WRITE:
			ret = gc2145_write(sensor, op->reg, op->val);
			break;
		case TX_WRITE16:
			ret = gc2145_write16(sensor, op->reg, op->val);
			break;
		case TX_UPDATE_BITS:
			ret = gc2145_update_bits(sensor, op->reg, op->mask, op->val);
			break;
		default:
			dev_err(&sensor->i2c_client->dev, "invalid op at %d\n", i);
			ret = -EINVAL;
		}

		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Efficiently write to a set of registers, using auto-increment
 * when possible. User must not use address 0xff. To switch banks,
 * use sequence: 0xfe, bank_no.
 */
static int gc2145_set_registers(struct gc2145_dev *sensor,
				const uint8_t* data, size_t data_len)
{
	int ret = 0, i = 0;
	u16 start, len;
	u8 buf[128];

	if (data_len % 2 != 0) {
		v4l2_err(&sensor->sd, "Register map has invalid size\n");
		return -EINVAL;
	}

	/* we speed up communication by using auto-increment functionality */
	while (i < data_len) {
		start = data[i];
		len = 0;

		while (i < data_len && data[i] == (start + len) &&
		       len < sizeof(buf)) {
			buf[len++] = data[i + 1];
			i += 2;
		}

		ret = gc2145_write_regs(sensor, start, buf, len);
		if (ret)
			return ret;
	}

	sensor->current_bank = 0xff;
	return 0;
}

/*
 * The firmware format:
 * <record 0>, ..., <record N - 1>
 * "record" is a 1-byte register address followed by 1-byte data
 */
static int gc2145_load_firmware(struct gc2145_dev *sensor, const char *name)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, name, sensor->sd.v4l2_dev->dev);
	if (ret) {
		v4l2_warn(&sensor->sd,
			  "Failed to read firmware %s, continuing anyway...\n",
			  name);
		return 1;
	}

	if (fw->size == 0) {
		release_firmware(fw);
		return 1;
	}

	ret = gc2145_set_registers(sensor, fw->data, fw->size);

	release_firmware(fw);
	return ret;
}

/* }}} */
/* {{{ Controls */

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct gc2145_dev,
			     ctrls.handler)->sd;
}


/* Image processing */

/*
 * The special effect register selects one of a set of canned effects in
 * its top nibble and turns on inversion, the edge maps and the fixed
 * chroma in the bottom one. The values are the ones the application
 * note for the previous generation lists, which the effect enumeration
 * in this sensor's own register list agrees with.
 */
static int gc2145_set_colorfx(struct gc2145_dev *sensor, s32 val)
{
	static const u8 effects[] = {
		[V4L2_COLORFX_NONE]		= 0x00,
		[V4L2_COLORFX_BW]		= 0x12,
		[V4L2_COLORFX_SEPIA]		= 0x82,
		[V4L2_COLORFX_NEGATIVE]		= 0x01,
		[V4L2_COLORFX_EMBOSS]		= 0x06,
		[V4L2_COLORFX_SKETCH]		= 0x08,
		[V4L2_COLORFX_SKY_BLUE]		= 0x62,
		[V4L2_COLORFX_GRASS_GREEN]	= 0x52,
		[V4L2_COLORFX_SOLARIZATION]	= 0x90,
		[V4L2_COLORFX_ANTIQUE]		= 0x82,
		[V4L2_COLORFX_SET_CBCR]		= GC2145_SPECIAL_EFFECT_FIXED_CBCR,
	};

	if (val < 0 || val >= ARRAY_SIZE(effects))
		return -EINVAL;

	return gc2145_write(sensor, GC2145_P0_ISP_SPECIAL_EFFECT, effects[val]);
}

/* Exposure and white balance */

/*
 * The AEC and the AWB own their result registers while they are running
 * and hand them back when they are switched off, keeping whatever they
 * last converged on. Switching the algorithm off is therefore both how
 * manual mode is entered and how 3A is locked, and it is what lets a
 * caller read out the exposure and gains a preview settled on and apply
 * the same ones to a still.
 */
static int gc2145_update_aec_enable(struct gc2145_dev *sensor)
{
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	bool run = ctrls->auto_exposure->val != V4L2_EXPOSURE_MANUAL &&
		   !(ctrls->aaa_lock->val & V4L2_LOCK_EXPOSURE) &&
		   !sensor->test_pattern_holds_aec;

	return gc2145_write(sensor, GC2145_P0_AEC_ENABLE, run ? 1 : 0);
}

static int gc2145_update_awb_enable(struct gc2145_dev *sensor)
{
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	bool run = ctrls->wb->val == V4L2_WHITE_BALANCE_AUTO &&
		   !(ctrls->aaa_lock->val & V4L2_LOCK_WHITE_BALANCE);

	return gc2145_update_bits(sensor, GC2145_P0_ISP_BLK_ENABLE3,
				  GC2145_ISP_BLK_ENABLE3_AWB,
				  run ? GC2145_ISP_BLK_ENABLE3_AWB : 0);
}

/*
 * The presets the vendor register tables carry. They are the same in
 * every GC2145 driver that has them, and are the gains the AWB settles
 * on under each illuminant.
 */
static const struct {
	u8 preset;
	u8 r, g, b;
} gc2145_wb_presets[] = {
	{ V4L2_WHITE_BALANCE_INCANDESCENT,	0x50, 0x40, 0xa8 },
	{ V4L2_WHITE_BALANCE_FLUORESCENT,	0x72, 0x40, 0x5b },
	{ V4L2_WHITE_BALANCE_HORIZON,		0xa0, 0x45, 0x40 },
	{ V4L2_WHITE_BALANCE_DAYLIGHT,		0x70, 0x40, 0x50 },
	{ V4L2_WHITE_BALANCE_CLOUDY,		0x58, 0x40, 0x50 },
};

static int gc2145_get_white_balance(struct gc2145_dev *sensor)
{
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	u8 r, b;
	int ret;

	ret = gc2145_read(sensor, GC2145_P0_AWB_R_GAIN, &r);
	if (ret)
		return ret;

	ret = gc2145_read(sensor, GC2145_P0_AWB_B_GAIN, &b);
	if (ret)
		return ret;

	ctrls->red_balance->val = r;
	ctrls->blue_balance->val = b;

	return 0;
}

static int gc2145_set_white_balance(struct gc2145_dev *sensor)
{
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	u8 r = ctrls->red_balance->val, g = GC2145_AWB_GAIN_UNITY;
	u8 b = ctrls->blue_balance->val;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(gc2145_wb_presets); i++) {
		if (gc2145_wb_presets[i].preset != ctrls->wb->val)
			continue;

		r = gc2145_wb_presets[i].r;
		g = gc2145_wb_presets[i].g;
		b = gc2145_wb_presets[i].b;
		break;
	}

	ret = gc2145_update_awb_enable(sensor);
	if (ret || ctrls->wb->val == V4L2_WHITE_BALANCE_AUTO)
		return ret;

	gc2145_tx_start(sensor);
	gc2145_tx_write8(sensor, GC2145_P0_AWB_R_GAIN, r);
	gc2145_tx_write8(sensor, GC2145_P0_AWB_G_GAIN, g);
	gc2145_tx_write8(sensor, GC2145_P0_AWB_B_GAIN, b);

	return gc2145_tx_commit(sensor);
}

/*
 * The exposure register counts array row slots, of which there are
 * slots_per_line in a V4L2 line, so the two differ whenever rows are
 * being skipped. Keeping the control in V4L2 lines is what makes
 * exposure * (width + hblank) / pixel_rate the exposure time in every
 * mode, and so what lets an exposure be carried between modes.
 */
static int gc2145_get_exposure(struct gc2145_dev *sensor)
{
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	u8 dgain;
	u16 exp;
	int ret;

	ret = gc2145_read(sensor, GC2145_P0_AUTO_PREGAIN, &dgain);
	if (ret)
		return ret;

	ret = gc2145_read16(sensor, GC2145_P0_EXPOSURE_HI, &exp);
	if (ret)
		return ret;

	ctrls->exposure->val = exp / sensor->params.slots_per_line;
	ctrls->d_gain->val = dgain;

	return 0;
}

#define AE_BIAS_MENU_DEFAULT_VALUE_INDEX 4
static const s64 ae_bias_menu_values[] = {
	-4000, -3000, -2000, -1000, 0, 1000, 2000, 3000, 4000
};

static const s8 ae_bias_menu_reg_values[] = {
	0x55, 0x60, 0x65, 0x70, 0x7b, 0x85, 0x90, 0x95, 0xa0
};

static int gc2145_set_exposure(struct gc2145_dev *sensor)
{
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	bool is_auto = (ctrls->auto_exposure->val != V4L2_EXPOSURE_MANUAL);
	int ret;

	ret = gc2145_update_aec_enable(sensor);
	if (ret)
		return ret;

	gc2145_tx_start(sensor);

	if (!is_auto && ctrls->exposure->is_new)
		gc2145_tx_write16(sensor, GC2145_P0_EXPOSURE_HI,
				  ctrls->exposure->val *
				  sensor->params.slots_per_line);

	if (!is_auto && ctrls->d_gain->is_new)
		gc2145_tx_write8(sensor, GC2145_P0_AUTO_PREGAIN,
				 ctrls->d_gain->val);

	return gc2145_tx_commit(sensor);
}

/*
 * Ways of making the output smaller than the analogue crop. All of them
 * keep the whole crop in the picture, so the field of view never depends
 * on the requested resolution:
 *
 *  - row skip drops every other row in the pixel array. It is the only
 *    one that shortens the frame, and it always comes paired with the
 *    column scaler so that the pixels stay square.
 *  - the column scaler halves the width in the ISP input stage.
 *  - the subsample block decimates by an arbitrary ratio afterwards.
 *
 * The ratios below are the ones that divide the active area exactly, so
 * that the ISP crop only ever has to trim the dummy pixels around the
 * readout window.
 */
struct gc2145_mode {
	/* row skip in the pixel array plus the ISP column scaler */
	bool half;
	/* subsample ratio applied after that */
	u8 sub_num;
	u8 sub_den;
	/* the two combined, as a fraction of the analogue crop */
	u8 num;
	u8 den;
};

static const struct gc2145_mode gc2145_modes[] = {
	{ .half = false, .sub_num = 1, .sub_den = 1, .num = 1, .den = 1 },
	{ .half = false, .sub_num = 4, .sub_den = 5, .num = 4, .den = 5 },
	{ .half = true,  .sub_num = 1, .sub_den = 1, .num = 1, .den = 2 },
	{ .half = true,  .sub_num = 4, .sub_den = 5, .num = 2, .den = 5 },
	{ .half = true,  .sub_num = 1, .sub_den = 2, .num = 1, .den = 4 },
	{ .half = true,  .sub_num = 2, .sub_den = 5, .num = 1, .den = 5 },
};

static void gc2145_mode_output_size(const struct gc2145_mode *mode,
				    const struct v4l2_rect *crop,
				    u32 *width, u32 *height)
{
	*width = round_down(crop->width * mode->num / mode->den, 2);
	*height = round_down(crop->height * mode->num / mode->den, 2);
}

/*
 * Only the undecimated mode keeps the Bayer phase.
 *
 * Every other mode runs the ISP column scaler, and the scaler mixes
 * neighbouring columns, which are neighbouring colours. Reading a raw
 * frame out of a decimated mode gives a mosaic whose phase changes along
 * the row: presenting the sensor with a solid primary and looking at the
 * four positions of each 2x2, the colour lands on one position at
 * 1600x1200 and on both column parities at 1280x960 and below. The row
 * phase does survive, because that half of the decimation is a skip in the
 * pixel array rather than a scale, but half a mosaic is no more usable
 * than none.
 *
 * So a raw format is only ever offered, and only ever set, at the full
 * size. The processed formats are unaffected - the ISP has already
 * demosaiced by the time the scaler sees the data.
 */
static const struct gc2145_mode *gc2145_raw_mode(void)
{
	return &gc2145_modes[0];
}

static const struct gc2145_mode *
gc2145_find_mode(const struct v4l2_rect *crop, u32 width, u32 height)
{
	const struct gc2145_mode *best = &gc2145_modes[0];
	unsigned int best_err = UINT_MAX, err, i;
	u32 w, h;

	for (i = 0; i < ARRAY_SIZE(gc2145_modes); i++) {
		gc2145_mode_output_size(&gc2145_modes[i], crop, &w, &h);
		if (w < GC2145_SENSOR_WIDTH_MIN || h < GC2145_SENSOR_HEIGHT_MIN)
			continue;

		err = abs((int)w - (int)width) + abs((int)h - (int)height);
		if (err < best_err) {
			best_err = err;
			best = &gc2145_modes[i];
		}
	}

	return best;
}

static void gc2145_params_init(struct gc2145_sensor_params *p,
			       const struct gc2145_mode *mode,
			       const struct v4l2_rect *crop)
{
	p->mode = mode;
	p->slots_per_line = mode->half ? 2 : 1;

	/*
	 * The readout window takes in the dummy pixels around the crop,
	 * which the ISP crop trims off again at the end of the pipeline.
	 */
	p->col_start = crop->left - GC2145_ACTIVE_LEFT;
	p->row_start = crop->top - GC2145_ACTIVE_TOP;
	p->win_width = crop->width + 2 * GC2145_ACTIVE_LEFT;
	p->win_height = crop->height + 2 * GC2145_ACTIVE_TOP;

	gc2145_mode_output_size(mode, crop, &p->out_width, &p->out_height);

	p->st = 2;
	p->et = 2;
	p->vb = GC2145_VBLANK_REG_MIN;
	p->hb = GC2145_HBLANK_REG_MIN;
	p->sh_delay = GC2145_SH_DELAY;
}

/*
 * The row period, in the sensor's own unit of two pixels. One unit takes
 * four periods of the pixel clock on the PCLK pin.
 */
static unsigned long
gc2145_params_row_period(struct gc2145_sensor_params *p)
{
	return p->hb + p->sh_delay + p->win_width / 2 + 4;
}

/* the number of array row slots in a frame */
static unsigned long
gc2145_params_frame_lines(struct gc2145_sensor_params *p)
{
	return p->vb + p->win_height;
}

static unsigned long
gc2145_params_frame_period(struct gc2145_sensor_params *p)
{
	return gc2145_params_row_period(p)
		* gc2145_params_frame_lines(p)
		/ p->slots_per_line;
}

/*
 * V4L2 expresses the frame timing as
 *
 *   frame interval = (width + hblank) * (height + vblank) / pixel_rate
 *
 * with the exposure counted in the same lines as vblank. The pixel rate
 * is the one in the pixel array, which is half the clock on the PCLK pin
 * because a pixel leaves the sensor as two bytes. A V4L2 line is one
 * output row, so the decimation shows up in hblank and vblank rather
 * than in the pixel rate, and the line period comes out the same in
 * every mode - which is what lets an exposure be carried from one mode
 * to another unchanged.
 */
static unsigned long
gc2145_params_pixel_rate(struct gc2145_sensor_params *p, unsigned long pclk)
{
	return pclk / 2;
}

static unsigned long
gc2145_params_hblank(struct gc2145_sensor_params *p)
{
	return 2 * gc2145_params_row_period(p) - p->out_width;
}

static unsigned long
gc2145_params_vblank(struct gc2145_sensor_params *p)
{
	return gc2145_params_frame_lines(p)
		/ p->slots_per_line - p->out_height;
}

/* the vertical blanking reachable with a given blanking register value */
static unsigned long
gc2145_params_vblank_for_vb(struct gc2145_sensor_params *p, unsigned long vb)
{
	return (vb + p->win_height) / p->slots_per_line
		- p->out_height;
}

/* and back, rounded down to what the register can express */
static unsigned long
gc2145_params_vb_for_vblank(struct gc2145_sensor_params *p,
			    unsigned long vblank)
{
	unsigned long lines = (p->out_height + vblank)
		* p->slots_per_line;

	if (lines < p->win_height)
		return GC2145_VBLANK_REG_MIN;

	return clamp(round_down(lines - p->win_height, 4),
		     GC2145_VBLANK_REG_MIN, GC2145_VBLANK_REG_MAX);
}

/*
 * The longest exposure that still fits in a frame. Beyond it the sensor
 * stretches the frame to make room, which the frame interval no longer
 * describes, so cap the control there.
 */
static unsigned long
gc2145_params_exposure_max(struct gc2145_sensor_params *p,
			   unsigned long vblank)
{
	unsigned int slots = p->slots_per_line;

	return min(p->out_height + vblank,
		   (unsigned long)GC2145_EXPOSURE_REG_MAX / slots);
}

/*
 * The largest pre gain the total gain clamp still lets through at a
 * given global gain. Asking for more only widens the gap between the
 * gain the control claims and the gain the picture got.
 */
static unsigned long gc2145_pregain_max(unsigned long global_gain)
{
	return min(255ul, GC2145_TOTAL_GAIN_MAX / global_gain);
}

/*
 * The exposure quantum that keeps lighting flicker out of the image: the
 * mains half period expressed in the units the exposure register counts.
 * A row slot takes 4 / pclk * row_period / slots_per_line seconds.
 */
static unsigned long
gc2145_params_flicker_step(struct gc2145_sensor_params *p,
			   unsigned long power_line_freq, unsigned long pclk)
{
	unsigned long rt = gc2145_params_row_period(p);

	if (!rt || !power_line_freq)
		return 0;

	return pclk / (2 * power_line_freq) * p->slots_per_line / (4 * rt);
}

static void gc2145_params_fit_hb(struct gc2145_sensor_params *p,
				 unsigned long power_line_freq,
				 unsigned long pclk)
{
	unsigned long rt, power_line_ratio;

	/*
	 * Nothing to fit the row period to, so leave it at its shortest -
	 * which is also the fastest the mode can be read out.
	 */
	if (!power_line_freq) {
		p->hb = GC2145_HBLANK_REG_MIN;
		return;
	}

	for (p->hb = GC2145_HBLANK_REG_MIN; p->hb < 4095; p->hb++) {
		rt = gc2145_params_row_period(p);

		// power_line_ratio is row_freq / power_line_freq * 1000
		power_line_ratio = pclk / 4 / power_line_freq * 1000 / rt;

		// if we're close enough, stop the search
		if (power_line_ratio % 1000 < 20)
			break;
	}

	// finding the optimal Hb is not critical
	if (p->hb == 4095)
		p->hb = GC2145_HBLANK_REG_MIN;
}

static void gc2145_params_fit_vb(struct gc2145_sensor_params *p,
				 unsigned long frame_period)
{
	unsigned long rt, fp;

	p->vb = GC2145_VBLANK_REG_MIN;
	rt = gc2145_params_row_period(p);
	fp = gc2145_params_frame_period(p);

	if (frame_period > fp)
		p->vb = frame_period
			* p->slots_per_line / rt
			- p->win_height;

	/* the vertical blanking register wants a multiple of four */
	p->vb = round_down(p->vb, 4);

	if (p->vb > GC2145_VBLANK_REG_MAX)
		p->vb = GC2145_VBLANK_REG_MAX;
}

static struct gc2145_sensor_params
gc2145_compute_params(struct gc2145_dev *sensor, const struct v4l2_rect *crop,
		      u32 width, u32 height, unsigned long framerate,
		      unsigned long *pclk)
{
	struct gc2145_sensor_params params = {0};
	unsigned long power_line_freq = sensor->power_line_freq;

	/*
	 * The pixel clock is the same for every mode. Keeping it fixed
	 * keeps the row period, and with it the exposure line, the same
	 * across a mode change.
	 */
	*pclk = GC2145_PCLK_RATE;

	gc2145_params_init(&params, gc2145_find_mode(crop, width, height),
			   crop);

	/*
	 * Stretch the row period so that it takes a whole number of mains
	 * periods, which is what keeps lighting flicker out of the image,
	 * then use the vertical blanking to reach the requested rate.
	 */
	gc2145_params_fit_hb(&params, power_line_freq, *pclk);

	if (framerate)
		gc2145_params_fit_vb(&params, *pclk / 4 / framerate);

	return params;
}

static struct gc2145_sensor_params
gc2145_get_sensor_params(struct gc2145_dev *sensor,
			 struct v4l2_subdev_state *state,
			 unsigned long framerate, unsigned long *pclk)
{
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);

	return gc2145_compute_params(sensor,
				     v4l2_subdev_state_get_crop(state, 0),
				     fmt->width, fmt->height, framerate, pclk);
}

/*
 * Refresh the controls that describe the timing to userspace. They are
 * what a caller needs to turn an exposure in lines into one in seconds,
 * so they have to follow every change of format, crop or frame rate.
 */
static void gc2145_update_timing_ctrls(struct gc2145_dev *sensor,
				       struct v4l2_subdev_state *state)
{
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	struct gc2145_sensor_params *p = &sensor->params;
	unsigned long pclk, vblank, vblank_min, vblank_max, hblank, exposure_max;

	*p = gc2145_get_sensor_params(sensor, state,
				      sensor->frame_interval.denominator, &pclk);

	hblank = gc2145_params_hblank(p);
	vblank = gc2145_params_vblank(p);
	vblank_min = gc2145_params_vblank_for_vb(p, GC2145_VBLANK_REG_MIN);
	vblank_max = gc2145_params_vblank_for_vb(p, GC2145_VBLANK_REG_MAX);

	__v4l2_ctrl_modify_range(ctrls->pixel_rate,
				 gc2145_params_pixel_rate(p, pclk),
				 gc2145_params_pixel_rate(p, pclk), 1,
				 gc2145_params_pixel_rate(p, pclk));
	__v4l2_ctrl_modify_range(ctrls->hblank, hblank, hblank, 1, hblank);
	__v4l2_ctrl_modify_range(ctrls->vblank, vblank_min, vblank_max, 1,
				 vblank);
	__v4l2_ctrl_s_ctrl(ctrls->vblank, vblank);

	exposure_max = gc2145_params_exposure_max(p, vblank);
	__v4l2_ctrl_modify_range(ctrls->exposure, 1, exposure_max, 1,
				 min(ctrls->exposure->val, (s32)exposure_max));
}

/*
 * Program the subsample block. Position zero of every group is always
 * emitted and the eight nibbles list the other positions to emit, so a
 * ratio of num/den needs num - 1 of them. Their placement within the
 * list does not appear to matter; they go at the end because that is
 * where the one vendor table that uses this block puts them.
 */
static void gc2145_tx_subsample(struct gc2145_dev *sensor, u8 num, u8 den)
{
	u8 regs[4] = {0, 0, 0, 0};
	unsigned int i, slot;

	for (i = 1; i < num; i++) {
		/* slot counts nibbles, the high one of each byte first */
		slot = ARRAY_SIZE(regs) * 2 - num + i;
		if (slot % 2)
			regs[slot / 2] |= (i * den / num) & 0xf;
		else
			regs[slot / 2] |= ((i * den / num) & 0xf) << 4;
	}

	gc2145_tx_write8(sensor, GC2145_P0_SUBSAMPLE_RATIO, den << 4 | den);
	gc2145_tx_write8(sensor, GC2145_P0_SUBSAMPLE_MODE,
			 GC2145_SUBSAMPLE_MODE_SMOOTH);

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		gc2145_tx_write8(sensor, GC2145_P0_SUB_ROW_N1 + i, regs[i]);
		gc2145_tx_write8(sensor, GC2145_P0_SUB_COL_N1 + i, regs[i]);
	}
}

static int gc2145_params_apply(struct gc2145_dev *sensor,
			       struct gc2145_sensor_params *p)
{
	const struct gc2145_mode *mode = p->mode;
	unsigned long isp_width, isp_height;

	/* the frame the ISP crop is taken out of */
	isp_width = p->win_width / (mode->half ? 2 : 1)
		* mode->sub_num / mode->sub_den;
	isp_height = p->win_height / (mode->half ? 2 : 1)
		* mode->sub_num / mode->sub_den;

	gc2145_tx_start(sensor);

	gc2145_tx_write8(sensor, GC2145_REG_SCALER_MODE,
			 mode->half ? BIT(0) | BIT(1) : 0);

	gc2145_tx_write8(sensor, 0x18, 0x0a | (mode->half ? BIT(6) : 0));

	gc2145_tx_write16(sensor, 0x09, p->row_start);
	gc2145_tx_write16(sensor, 0x0b, p->col_start);
	gc2145_tx_write16(sensor, 0x0d, p->win_height);
	gc2145_tx_write16(sensor, 0x0f, p->win_width);
	gc2145_tx_write16(sensor, 0x05, p->hb);
	gc2145_tx_write16(sensor, 0x07, p->vb);
	gc2145_tx_write16(sensor, 0x11, p->sh_delay);

	gc2145_tx_write8(sensor, 0x13, p->st);
	gc2145_tx_write8(sensor, 0x14, p->et);

	gc2145_tx_subsample(sensor, mode->sub_num, mode->sub_den);

	/*
	 * Trim what is left down to the requested size, centred. The
	 * origin has to stay even: an odd column would start the line on
	 * the other chroma sample of the pair and swap Cb with Cr, and an
	 * odd row would move the Bayer phase in the raw formats.
	 */
	gc2145_tx_write16(sensor, GC2145_P0_CROP_Y1_HI,
			  round_down((isp_height - p->out_height) / 2, 2));
	gc2145_tx_write16(sensor, GC2145_P0_CROP_X1_HI,
			  round_down((isp_width - p->out_width) / 2, 2));
	gc2145_tx_write16(sensor, GC2145_P0_CROP_WIN_HEIGHT_HI, p->out_height);
	gc2145_tx_write16(sensor, GC2145_P0_CROP_WIN_WIDTH_HI, p->out_width);
	gc2145_tx_write8(sensor, GC2145_P0_CROP_ENABLE, BIT(0));

	return gc2145_tx_commit(sensor);
}

/* Test patterns */

enum {
	GC2145_TEST_PATTERN_DISABLED,
	GC2145_TEST_PATTERN_VGA_COLOR_BARS,
	GC2145_TEST_PATTERN_UXGA_COLOR_BARS,
	GC2145_TEST_PATTERN_SKIN_MAP,
	GC2145_TEST_PATTERN_SOLID_COLOR,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"VGA color bars",
	"UXGA color bars",
	"Skin map",
	"Solid black color",
	"Solid light gray color",
	"Solid gray color",
	"Solid dark gray color",
	"Solid white color",
	"Solid red color",
	"Solid green color",
	"Solid blue color",
	"Solid yellow color",
	"Solid cyan color",
	"Solid magenta color",
};

static int gc2145_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	if (!pm_runtime_get_if_in_use(dev))
		return -EIO;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		ret = gc2145_get_exposure(sensor);
		break;

	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		ret = gc2145_get_white_balance(sensor);
		break;
	default:
		dev_err(dev, "getting unknown control %d\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int gc2145_setup_aec_range(struct gc2145_dev *sensor,
				  struct gc2145_sensor_params *params);

static int gc2145_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	s32 val = ctrl->val;
	unsigned long exposure_max, pregain_max;
	int ret;
	u8 test1, test2;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_POWER_LINE_FREQUENCY:
		/*
		 * Zero means there is no flicker to avoid, which leaves the
		 * row period free to be as short as it can and the exposure
		 * free to take any value. The row period is fitted when the
		 * mode is programmed, so this takes full effect then.
		 */
		switch (val) {
		case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
			sensor->power_line_freq = 0;
			break;
		case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
			sensor->power_line_freq = 60;
			break;
		default:
			sensor->power_line_freq = 50;
			break;
		}
		sensor->pending_mode_change = true;
		return 0;

	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_HBLANK:
		/*
		 * Read-only controls that only ever carry values computed
		 * from the mode. They still reach us through the internal
		 * s_ctrl helpers used to update them.
		 */
		return 0;

	case V4L2_CID_VBLANK:
		/*
		 * This is how the frame rate is set. Keep the longest
		 * exposure that still fits in a frame in step with it.
		 */
		sensor->params.vb =
			gc2145_params_vb_for_vblank(&sensor->params, val);

		exposure_max = gc2145_params_exposure_max(&sensor->params,
							  val);
		__v4l2_ctrl_modify_range(ctrls->exposure, 1, exposure_max, 1,
					 min(ctrls->exposure->val,
					     (s32)exposure_max));
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		/*
		 * The two gains are clamped as a product, so how far the
		 * digital one is worth moving depends on this one. Without
		 * this the top of its range would silently do nothing.
		 */
		pregain_max = gc2145_pregain_max(val);
		__v4l2_ctrl_modify_range(ctrls->d_gain, 1, pregain_max, 1,
					 min(ctrls->d_gain->val,
					     (s32)pregain_max));
		break;
	}

	/*
	 * Everything below touches registers. While the sensor is powered
	 * down the values are simply kept in the control handler, and the
	 * handler setup done when streaming starts applies them.
	 */
	if (!pm_runtime_get_if_in_use(&sensor->i2c_client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		ret = gc2145_set_exposure(sensor);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc2145_write(sensor, GC2145_P0_GLOBAL_GAIN, val);
		if (ret)
			break;

		/* the AEC has its own ceiling, under the same clamp */
		ret = gc2145_setup_aec_range(sensor, &sensor->params);
		break;

	case V4L2_CID_VBLANK:
		ret = gc2145_write16(sensor, GC2145_P0_VBLANK_DELAY_HI,
				     sensor->params.vb);
		if (ret)
			break;

		/*
		 * The frame just changed length and the AEC gears are
		 * measured against it. Userspace sets the frame rate after
		 * the mode is up, so this is not a rare path.
		 */
		ret = gc2145_setup_aec_range(sensor, &sensor->params);
		break;

	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		ret = gc2145_set_white_balance(sensor);
		break;

	case V4L2_CID_3A_LOCK:
		ret = gc2145_update_aec_enable(sensor);
		if (!ret)
			ret = gc2145_update_awb_enable(sensor);
		break;

	case V4L2_CID_BRIGHTNESS:
		/* a signed offset added to the luma */
		ret = gc2145_write(sensor, GC2145_P2_LUMA_OFFSET, val);
		break;

	case V4L2_CID_CONTRAST:
		ret = gc2145_write(sensor, GC2145_P2_LUMA_CONTRAST, val);
		break;

	case V4L2_CID_SATURATION:
		gc2145_tx_start(sensor);
		gc2145_tx_write8(sensor, GC2145_P2_SATURATION_CB, val);
		gc2145_tx_write8(sensor, GC2145_P2_SATURATION_CR, val);
		ret = gc2145_tx_commit(sensor);
		break;

	case V4L2_CID_SHARPNESS:
		/* the coarse and the fine edge effect share a register */
		ret = gc2145_write(sensor, GC2145_P2_EDGE_EFFECT,
				   val << 4 | val);
		break;

	case V4L2_CID_COLORFX:
		ret = gc2145_set_colorfx(sensor, val);
		break;

	case V4L2_CID_COLORFX_CBCR:
		gc2145_tx_start(sensor);
		gc2145_tx_write8(sensor, GC2145_P2_FIXED_CB, val >> 8);
		gc2145_tx_write8(sensor, GC2145_P2_FIXED_CR, val);
		ret = gc2145_tx_commit(sensor);
		break;

	case V4L2_CID_EXPOSURE_METERING:
		/*
		 * The AEC weights a centre window against the rest of the
		 * measure window. Turning the weight off is the closest
		 * this gets to a flat average.
		 */
		ret = gc2145_update_bits(sensor, GC2145_P1_AEC_MODE3,
					 GC2145_AEC_MODE3_CENTRE_WEIGHT,
					 val == V4L2_EXPOSURE_METERING_AVERAGE ?
					 0 : FIELD_PREP(GC2145_AEC_MODE3_CENTRE_WEIGHT, 4));
		break;

	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		if (val < 0 || val >= ARRAY_SIZE(ae_bias_menu_reg_values)) {
			dev_err(&sensor->i2c_client->dev, "ae bias out of range\n");
			ret = -EINVAL;
			break;
		}

		ret = gc2145_write(sensor, 0x113,
				   (u8)ae_bias_menu_reg_values[val]);
		break;

	case V4L2_CID_VFLIP:
		ret = gc2145_update_bits(sensor, 0x17, BIT(1),
					 val ? BIT(1) : 0);
		break;

	case V4L2_CID_HFLIP:
		ret = gc2145_update_bits(sensor, 0x17, BIT(0),
					 val ? BIT(0) : 0);
		break;

	case V4L2_CID_TEST_PATTERN:
		test1 = 0;
		test2 = GC2145_DEBUG_MODE3_UPDATE_GAIN;
		sensor->test_pattern_holds_aec = false;

		if (val == GC2145_TEST_PATTERN_VGA_COLOR_BARS)
			test1 = GC2145_DEBUG_MODE2_TEST_IMAGE;
		else if (val == GC2145_TEST_PATTERN_UXGA_COLOR_BARS)
			test1 = GC2145_DEBUG_MODE2_TEST_IMAGE |
				GC2145_DEBUG_MODE2_TEST_IMAGE_UXGA;
		else if (val == GC2145_TEST_PATTERN_SKIN_MAP)
			test1 = GC2145_DEBUG_MODE2_SKIN_MAP;
		else if (val >= GC2145_TEST_PATTERN_SOLID_COLOR) {
			/*
			 * The fixed value is a level, and both the update
			 * gain bit and the AEC would amplify it: metering a
			 * flat field drives the gain up until everything
			 * except the black level saturates to white.
			 */
			test1 = GC2145_DEBUG_MODE2_TEST_IMAGE;
			test2 = ((val - GC2145_TEST_PATTERN_SOLID_COLOR) << 4) |
				GC2145_DEBUG_MODE3_FIX_VALUE_MODE;
			sensor->test_pattern_holds_aec = true;
		} else if (val != GC2145_TEST_PATTERN_DISABLED) {
			dev_err(&sensor->i2c_client->dev, "test pattern out of range\n");
			ret = -EINVAL;
			break;
		}

		ret = gc2145_write(sensor, GC2145_P0_DEBUG_MODE2, test1);
		if (ret)
			break;

		ret = gc2145_write(sensor, GC2145_P0_DEBUG_MODE3, test2);
		if (ret)
			break;

		ret = gc2145_update_aec_enable(sensor);
		break;

	default:
		dev_err(&sensor->i2c_client->dev, "setting unknown control %d\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(&sensor->i2c_client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc2145_ctrl_ops = {
	.g_volatile_ctrl = gc2145_g_volatile_ctrl,
	.s_ctrl = gc2145_s_ctrl,
};

/* the pixel pitch, in nanometres */
static const struct v4l2_area gc2145_cell_size = {
	.width = 1750,
	.height = 1750,
};

static int gc2145_init_controls(struct gc2145_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &gc2145_ctrl_ops;
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct v4l2_fwnode_device_properties props;
	static const struct v4l2_rect default_crop = {
		.left = GC2145_ACTIVE_LEFT,
		.top = GC2145_ACTIVE_TOP,
		.width = GC2145_SENSOR_WIDTH_MAX,
		.height = GC2145_SENSOR_HEIGHT_MAX,
	};
	struct gc2145_sensor_params params;
	unsigned long pixel_rate;
	unsigned int i;
	u64 wb_mask, colorfx_mask;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	/* Clock related controls, for the mode .init_state will pick */
	params = gc2145_compute_params(sensor, &default_crop,
				       GC2145_DEFAULT_WIDTH,
				       GC2145_DEFAULT_HEIGHT,
				       sensor->frame_interval.denominator,
				       &pixel_rate);
	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
			0, INT_MAX, 1,
			gc2145_params_pixel_rate(&params, pixel_rate));

	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
			0, INT_MAX, 1,
			gc2145_params_hblank(&params));

	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
			0, INT_MAX, 1,
			gc2145_params_vblank(&params));

	/* Exposure controls */
	ctrls->auto_exposure = v4l2_ctrl_new_std_menu(hdl, ops,
						      V4L2_CID_EXPOSURE_AUTO,
						      V4L2_EXPOSURE_MANUAL, 0,
						      V4L2_EXPOSURE_AUTO);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    1, GC2145_EXPOSURE_REG_MAX, 1, 0x80);
	/*
	 * The pre digital gain is only ours while the AEC is off, so it
	 * belongs to the auto exposure cluster. Its scaling is not
	 * documented, so the value is the raw register one.
	 */
	ctrls->d_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN,
					  1, 255, 1, 0x20);
	/*
	 * The global gain is never touched by the AEC, so it stands on its
	 * own and applies in both exposure modes. Do not let it attenuate.
	 */
	ctrls->a_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					  GC2145_GLOBAL_GAIN_UNITY, 255, 1,
					  GC2145_GLOBAL_GAIN_DEFAULT);
	ctrls->exposure_bias =
		v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_AUTO_EXPOSURE_BIAS,
				       ARRAY_SIZE(ae_bias_menu_values) - 1,
				       AE_BIAS_MENU_DEFAULT_VALUE_INDEX,
				       ae_bias_menu_values);

	/*
	 * Flicker avoidance costs something, so it can be turned off: the
	 * row period is stretched to a whole number of mains periods, and
	 * the exposure the AEC picks is quantised to the mains half period.
	 * Under daylight or LED there is no flicker to avoid and both are
	 * pure loss - the quantum is 10 ms at 50 Hz, which in a 33 ms frame
	 * leaves the AEC three exposures to choose between.
	 *
	 * There is no detector, so this says what the light is rather than
	 * asking for it to be measured.
	 */
	ctrls->pl_freq = v4l2_ctrl_new_std_menu(hdl, ops,
						V4L2_CID_POWER_LINE_FREQUENCY,
						V4L2_CID_POWER_LINE_FREQUENCY_60HZ,
						0,
						V4L2_CID_POWER_LINE_FREQUENCY_50HZ);

	/*
	 * White balance. Only the manual mode takes the two gain controls;
	 * in the others the AWB owns them and they read back what it has
	 * converged on, which is what a caller needs to freeze a preview
	 * and reproduce it on a still.
	 */
	wb_mask = ~(BIT(V4L2_WHITE_BALANCE_MANUAL) |
		    BIT(V4L2_WHITE_BALANCE_AUTO));
	for (i = 0; i < ARRAY_SIZE(gc2145_wb_presets); i++)
		wb_mask &= ~BIT(gc2145_wb_presets[i].preset);

	ctrls->wb = v4l2_ctrl_new_std_menu(hdl, ops,
					   V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
					   V4L2_WHITE_BALANCE_SHADE, wb_mask,
					   V4L2_WHITE_BALANCE_AUTO);

	/*
	 * The white balance gains, like the global gain, are 2.6 fixed
	 * point with 0x40 for unity. The datasheet calls them 4.4, but the
	 * reset values, the gain limit registers and the vendor
	 * calibration procedure all agree that 0x40 is unity.
	 */
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 255, 1, GC2145_AWB_GAIN_UNITY);
	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 255, 1,
						GC2145_AWB_GAIN_UNITY);

	/*
	 * Switching an algorithm off leaves its results in place, so this
	 * is a real lock and not an approximation of one.
	 */
	ctrls->aaa_lock = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_3A_LOCK, 0,
					    V4L2_LOCK_EXPOSURE |
					    V4L2_LOCK_WHITE_BALANCE, 0, 0);

	ctrls->metering =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_METERING,
				       V4L2_EXPOSURE_METERING_CENTER_WEIGHTED,
				       ~(BIT(V4L2_EXPOSURE_METERING_AVERAGE) |
					 BIT(V4L2_EXPOSURE_METERING_CENTER_WEIGHTED)),
				       V4L2_EXPOSURE_METERING_CENTER_WEIGHTED);

	/*
	 * Image processing. The saturation and contrast registers are
	 * fixed point with the unity values below, the luma offset is a
	 * signed byte and the edge effect is a plain strength.
	 */
	ctrls->brightness = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BRIGHTNESS,
					      -128, 127, 1, 0);
	/*
	 * Both stop at twice unity rather than at the full register range.
	 * Unity is 0x40 of 255 for contrast and 0x20 of 255 for saturation,
	 * so over the whole range the useful part is squeezed into the
	 * bottom quarter and the bottom eighth, and the rest is chroma and
	 * luma that clipped long ago. Halving the span puts unity in the
	 * middle of the slider.
	 */
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, GC2145_CONTRAST_UNITY * 2, 1,
					    GC2145_CONTRAST_UNITY);
	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, GC2145_SATURATION_UNITY * 2, 1,
					      GC2145_SATURATION_UNITY);
	ctrls->sharpness = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SHARPNESS,
					     0, 15, 1, 4);

	colorfx_mask = ~(BIT(V4L2_COLORFX_NONE) |
			 BIT(V4L2_COLORFX_BW) |
			 BIT(V4L2_COLORFX_SEPIA) |
			 BIT(V4L2_COLORFX_NEGATIVE) |
			 BIT(V4L2_COLORFX_EMBOSS) |
			 BIT(V4L2_COLORFX_SKETCH) |
			 BIT(V4L2_COLORFX_SKY_BLUE) |
			 BIT(V4L2_COLORFX_GRASS_GREEN) |
			 BIT(V4L2_COLORFX_SOLARIZATION) |
			 BIT(V4L2_COLORFX_ANTIQUE) |
			 BIT(V4L2_COLORFX_SET_CBCR));

	ctrls->colorfx = v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_COLORFX,
						V4L2_COLORFX_SET_CBCR,
						colorfx_mask,
						V4L2_COLORFX_NONE);
	ctrls->colorfx_cbcr = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_COLORFX_CBCR,
						0, 0xffff, 1, 0);

	/*
	 * The pixel pitch, which is what turns the crop rectangle into a
	 * field of view.
	 */
	v4l2_ctrl_new_std_compound(hdl, NULL, V4L2_CID_UNIT_CELL_SIZE,
				   v4l2_ctrl_ptr_create((void *)&gc2145_cell_size),
				   v4l2_ctrl_ptr_create(NULL),
				   v4l2_ctrl_ptr_create(NULL));

	/* V/H flips */
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_VFLIP, 0, 1, 1, 0);

	ret = v4l2_fwnode_device_parse(&sensor->i2c_client->dev, &props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &gc2145_ctrl_ops,
					      &props);
	if (ret)
		goto free_ctrls;

	/* Test patterns */
	ctrls->test_pattern =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);
	/*
	 * The pixel rate is fixed and the horizontal blanking is picked to
	 * fit the mains period, so only the vertical blanking is left for
	 * a caller to set the frame rate with.
	 */
	ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_auto_cluster(3, &ctrls->auto_exposure, V4L2_EXPOSURE_MANUAL,
			       true);
	v4l2_ctrl_auto_cluster(3, &ctrls->wb, V4L2_WHITE_BALANCE_MANUAL,
			       true);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

/* }}} */
/* {{{ Video ops */

/*
 * Clock tree
 * ----------
 *
 *     MCLK pin
 *         |
 *  DIV2 (optional)        - Divide input MCLK by 2 when 0xf7[1] == 1
 *         |
 *   /- PLL mux -\         - PLL selected by 0xf8[7], otherwise fixed 32x mult
 *   |           |
 * PLL           |         - PLL multiplies by 0xf8[5:0]+1 * 4
 *   |      Fixed 32/48x   - Multiplies 32x when 0xf7[2] == 1 otherwise 48x
 *    \_________/
 *         |
 *       DOUBLE  (div by 4 or 8) based on 0xf7[3]
 *         |
 *      DVP MODE  (div by 4 more when 0xf7[7] == 1)
 *         |
 *     /-------\
 *     |       |
 * pclk_div  sclk_div     - sclk_div is 0xf7[6:4] + 1
 *     |       |
 *   2pclk    sclk        - sclk * 8 is the CSI-2 lane bit rate
 *
 * 2pclk is what leaves the PCLK pin, one byte per period. The internal
 * pixel clock, which the row period below is expressed in multiples of,
 * is half of it.
 */
__maybe_unused
static int gc2145_get_2pclk(struct gc2145_dev *sensor, unsigned long* pclk)
{
	u8 pll_mode1, pll_mode2, clk_div_mode;
	bool mclk_div2_en; // 0xf7[1]
	bool pll_en; // 0xf8[7]
	bool double_clk; // 0xf7[3]
	bool fixed_32x; // 0xf7[2]
	bool dvp_mode; // 0xf7[7]
	unsigned long pll_mult; // 0xf8[5:0] + 1
	unsigned long pclk_div; // 0xfa[7:4] + 1
	unsigned long int_clk;
	unsigned long mclk;
        int ret;

	ret = gc2145_read(sensor, 0xf7, &pll_mode1);
	if (ret)
		return ret;

	ret = gc2145_read(sensor, 0xf8, &pll_mode2);
	if (ret)
		return ret;

	ret = gc2145_read(sensor, 0xfa, &clk_div_mode);
	if (ret)
		return ret;

	mclk = clk_get_rate(sensor->xclk);
	if (mclk == 0)
		return -EINVAL;

	mclk_div2_en = pll_mode1 & BIT(1);
	pll_en = pll_mode2 & BIT(7);
	double_clk = pll_mode1 & BIT(3);
	fixed_32x = pll_mode1 & BIT(2);
	dvp_mode = pll_mode1 & BIT(7);
	pll_mult = (pll_mode2 & 0x3f) + 1;
	pclk_div = (clk_div_mode >> 4) + 1;

	int_clk = mclk / (mclk_div2_en ? 2 : 1);

	if (pll_en)
		int_clk *= pll_mult * 4;
	else
		int_clk *= fixed_32x ? 32 : 48;

	int_clk /= double_clk ? 4 : 8;

	if (dvp_mode)
		int_clk /= 4;

	if (pclk)
		*pclk = int_clk / pclk_div;

	return 0;
}

static int gc2145_set_2pclk(struct gc2145_dev *sensor,
			    unsigned long *freq, bool apply)
{
	unsigned long pll_mult, pll_mult_max, /*sclk_div,*/ pclk_div, pclk2,/* sclk,*/
		      mclk;
	unsigned long pll_mult_best = 0, pclk_div_best = 0, diff_best = ULONG_MAX, diff,
		      pclk2_best = 0;
	int mclk_div2_en; //, double_clk;
	int mclk_div2_en_best = 0; //, double_clk_best;

	mclk = clk_get_rate(sensor->xclk);
	if (mclk == 0)
		return -EINVAL;

        for (mclk_div2_en = 0; mclk_div2_en <= 1; mclk_div2_en++) {
		pll_mult_max = 768000000 / 4 / (mclk / (mclk_div2_en ? 2 : 1));
		if (pll_mult_max > 32)
			pll_mult_max = 32;

		for (pll_mult = 2; pll_mult <= pll_mult_max; pll_mult++) {
			for (pclk_div = 1; pclk_div <= 8; pclk_div++) {
				pclk2 = mclk / (mclk_div2_en ? 2 : 1) * pll_mult / pclk_div;

				if (pclk2 > *freq)
					continue;

				diff = *freq - pclk2;

				if (diff < diff_best) {
					diff_best = diff;
					pclk2_best = pclk2;

					pll_mult_best = pll_mult;
					pclk_div_best = pclk_div;
					mclk_div2_en_best = mclk_div2_en;
				}

				if (diff == 0)
					goto found;
			}
		}
	}

	if (diff_best == ULONG_MAX)
		return -1;

found:
	*freq = pclk2_best;
	if (!apply)
		return 0;

	gc2145_tx_start(sensor);

	/*
	 * The frequency computed above is the one reached with the CLK
	 * double bit set, which divides the PLL output by 4 rather than
	 * by 8. The serial clock divider in bits [6:4] only feeds the
	 * CSI-2 transmitter, so leave it alone.
	 */
	gc2145_tx_write8(sensor, 0xf7, BIT(3) /* clk_double */ |
			 (mclk_div2_en_best << 1) | BIT(0) /* pll_en */);
	gc2145_tx_write8(sensor, 0xf8, BIT(7) | (pll_mult_best - 1));

	/*
	 * The low nibble of the divider register is the number of high
	 * clocks per divided period. Half the divider gives the most
	 * symmetric duty cycle the hardware can produce.
	 */
	gc2145_tx_write8(sensor, 0xfa,
			 (pclk_div_best - 1) << 4 | (pclk_div_best / 2));

	return gc2145_tx_commit(sensor);
}

struct gc2145_reg {
	unsigned char address;
	unsigned char val;
};

static const struct gc2145_reg gc2145_awb_regs[] = {
	{0xfe, 0x01},
	{0x4f, 0x00}, {0x4f, 0x00}, {0x4b, 0x01}, {0x4f, 0x00},
	{0x4c, 0x01}, {0x4d, 0x71}, {0x4e, 0x01},
	{0x4c, 0x01}, {0x4d, 0x91}, {0x4e, 0x01},
	{0x4c, 0x01}, {0x4d, 0x70}, {0x4e, 0x01},
	{0x4c, 0x01}, {0x4d, 0x90}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0xb0}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0x8f}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0x6f}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0xaf}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0xd0}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0xf0}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0xcf}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0xef}, {0x4e, 0x02},
	{0x4c, 0x01}, {0x4d, 0x6e}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x8e}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xae}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xce}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x4d}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x6d}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x8d}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xad}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xcd}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x4c}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x6c}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x8c}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xac}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xcc}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xcb}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x4b}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x6b}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x8b}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0xab}, {0x4e, 0x03},
	{0x4c, 0x01}, {0x4d, 0x8a}, {0x4e, 0x04},
	{0x4c, 0x01}, {0x4d, 0xaa}, {0x4e, 0x04},
	{0x4c, 0x01}, {0x4d, 0xca}, {0x4e, 0x04},
	{0x4c, 0x01}, {0x4d, 0xca}, {0x4e, 0x04},
	{0x4c, 0x01}, {0x4d, 0xc9}, {0x4e, 0x04},
	{0x4c, 0x01}, {0x4d, 0x8a}, {0x4e, 0x04},
	{0x4c, 0x01}, {0x4d, 0x89}, {0x4e, 0x04},
	{0x4c, 0x01}, {0x4d, 0xa9}, {0x4e, 0x04},
	{0x4c, 0x02}, {0x4d, 0x0b}, {0x4e, 0x05},
	{0x4c, 0x02}, {0x4d, 0x0a}, {0x4e, 0x05},
	{0x4c, 0x01}, {0x4d, 0xeb}, {0x4e, 0x05},
	{0x4c, 0x01}, {0x4d, 0xea}, {0x4e, 0x05},
	{0x4c, 0x02}, {0x4d, 0x09}, {0x4e, 0x05},
	{0x4c, 0x02}, {0x4d, 0x29}, {0x4e, 0x05},
	{0x4c, 0x02}, {0x4d, 0x2a}, {0x4e, 0x05},
	{0x4c, 0x02}, {0x4d, 0x4a}, {0x4e, 0x05},
	{0x4c, 0x02}, {0x4d, 0x8a}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0x49}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0x69}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0x89}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0xa9}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0x48}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0x68}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0x69}, {0x4e, 0x06},
	{0x4c, 0x02}, {0x4d, 0xca}, {0x4e, 0x07},
	{0x4c, 0x02}, {0x4d, 0xc9}, {0x4e, 0x07},
	{0x4c, 0x02}, {0x4d, 0xe9}, {0x4e, 0x07},
	{0x4c, 0x03}, {0x4d, 0x09}, {0x4e, 0x07},
	{0x4c, 0x02}, {0x4d, 0xc8}, {0x4e, 0x07},
	{0x4c, 0x02}, {0x4d, 0xe8}, {0x4e, 0x07},
	{0x4c, 0x02}, {0x4d, 0xa7}, {0x4e, 0x07},
	{0x4c, 0x02}, {0x4d, 0xc7}, {0x4e, 0x07},
	{0x4c, 0x02}, {0x4d, 0xe7}, {0x4e, 0x07},
	{0x4c, 0x03}, {0x4d, 0x07}, {0x4e, 0x07},
	{0x4f, 0x01},
	{0x50, 0x80}, {0x51, 0xa8}, {0x52, 0x47}, {0x53, 0x38},
	{0x54, 0xc7}, {0x56, 0x0e}, {0x58, 0x08}, {0x5b, 0x00},
	{0x5c, 0x74}, {0x5d, 0x8b}, {0x61, 0xdb}, {0x62, 0xb8},
	{0x63, 0x86}, {0x64, 0xc0}, {0x65, 0x04}, {0x67, 0xa8},
	{0x68, 0xb0}, {0x69, 0x00}, {0x6a, 0xa8}, {0x6b, 0xb0},
	{0x6c, 0xaf}, {0x6d, 0x8b}, {0x6e, 0x50}, {0x6f, 0x18},
	{0x73, 0xf0}, {0x70, 0x0d}, {0x71, 0x60}, {0x72, 0x80},
	{0x74, 0x01}, {0x75, 0x01}, {0x7f, 0x0c}, {0x76, 0x70},
	{0x77, 0x58}, {0x78, 0xa0}, {0x79, 0x5e}, {0x7a, 0x54},
	{0x7b, 0x58},
	{0xfe, 0x00},
};

/*
 * The measure window the AWB averages over. The datasheet gives no units
 * for these four registers, but the reset defaults and every vendor mode
 * table only make sense with 16 output pixels per horizontal step and 8
 * output lines per vertical one. The GC2035 application note claims 4 for
 * all of them, which would place the window over a quarter of the frame.
 */
#define GC2145_AWB_WIN_X_RATIO		16
#define GC2145_AWB_WIN_Y_RATIO		8

static int gc2145_setup_awb(struct gc2145_dev *sensor,
			     u16 x1, u16 y1, u16 x2, u16 y2)
{
	// disable awb
	gc2145_update_bits(sensor, 0x82, BIT(1), 0);

	// load awb settings
	gc2145_set_registers(sensor, (void*)gc2145_awb_regs, sizeof(gc2145_awb_regs));

	// awb window (big window, page 0)
	gc2145_write(sensor, 0x0ec, x1 / GC2145_AWB_WIN_X_RATIO);
	gc2145_write(sensor, 0x0ed, y1 / GC2145_AWB_WIN_Y_RATIO);
	gc2145_write(sensor, 0x0ee, x2 / GC2145_AWB_WIN_X_RATIO);
	gc2145_write(sensor, 0x0ef, y2 / GC2145_AWB_WIN_Y_RATIO);

	// eanble awb
	gc2145_update_bits(sensor, 0x82, BIT(1), BIT(1));

	return 0;
}

/*
 * Unlike the AWB window above, both axes of the AEC window step in 8
 * output pixels. GC2035 uses 16 for the horizontal one, so its values
 * cannot be carried over.
 */
#define GC2145_AEC_WIN_RATIO		8

static int gc2145_setup_aec(struct gc2145_dev *sensor,
			     u16 x1, u16 y1, u16 x2, u16 y2,
			     u16 cx1, u16 cy1, u16 cx2, u16 cy2)
{
	gc2145_tx_start(sensor);

	// disable AEC
	gc2145_tx_write8(sensor, 0xb6, 0);

	// setup measure window
	gc2145_tx_write8(sensor, 0x101, x1 / GC2145_AEC_WIN_RATIO);
	gc2145_tx_write8(sensor, 0x102, x2 / GC2145_AEC_WIN_RATIO);
	gc2145_tx_write8(sensor, 0x103, y1 / GC2145_AEC_WIN_RATIO);
	gc2145_tx_write8(sensor, 0x104, y2 / GC2145_AEC_WIN_RATIO);

	// setup center
	gc2145_tx_write8(sensor, 0x105, cx1 / GC2145_AEC_WIN_RATIO);
	gc2145_tx_write8(sensor, 0x106, cx2 / GC2145_AEC_WIN_RATIO);
	gc2145_tx_write8(sensor, 0x107, cy1 / GC2145_AEC_WIN_RATIO);
	gc2145_tx_write8(sensor, 0x108, cy2 / GC2145_AEC_WIN_RATIO);

	/*
	 * Act on every frame rather than every second one, which is what
	 * the register comes up as. Both vendor drivers do this, and it is
	 * the whole of the difference between them and the reset state as
	 * far as the loop rate goes - the margins and step sizes at
	 * P1:0x18/0x19 and the 1.17x deadband at P1:0x1a are left alone by
	 * everyone, so they are left alone here too.
	 */
	gc2145_tx_update_bits(sensor, GC2145_P1_AEC_MODE2,
			      GC2145_AEC_MODE2_EVERY_N_FRAMES,
			      FIELD_PREP(GC2145_AEC_MODE2_EVERY_N_FRAMES, 1));

	// AEC_ASDE_select_luma_value AEC_low_light_exp_THD_max:
	//gc2145_tx_write8(sensor, 0x121, 0x15);

	// enable AEC again
	gc2145_tx_write8(sensor, 0xb6, 1);

	return gc2145_tx_commit(sensor);
}

/*
 * The envelope the AEC is allowed to work in.
 *
 * The seven exposure levels are the gears it drops through as the light
 * goes. Left to themselves they keep stretching the frame, and then the
 * frame interval and the exposure maximum this driver reports no longer
 * describe the sensor - the AEC was measured running a 800x600 preview
 * at 17.75fps while the controls said 30. So every gear is capped at
 * what fits in a frame. Userspace asks for a longer exposure by lowering
 * the frame rate through V4L2_CID_VBLANK, which lands back here.
 *
 * The gears only avoid flicker if they are whole multiples of the mains
 * period, so the cap is the last whole multiple that still fits.
 *
 * The gain ceilings are the AEC's own, separate from the gain controls,
 * and the pre gain one has to respect the same total gain clamp: asking
 * the AEC for gain the clamp swallows just leaves the picture dark.
 */
static int gc2145_setup_aec_range(struct gc2145_dev *sensor,
				  struct gc2145_sensor_params *params)
{
	unsigned long a_gain = sensor->ctrls.a_gain->val;
	unsigned long step, slots, level;
	int i;

	step = gc2145_params_flicker_step(params, sensor->power_line_freq,
					  GC2145_PCLK_RATE);

	/*
	 * The whole frame, not the last whole mains period inside it. The
	 * AEC quantises the exposure it picks against the anti-flicker
	 * register below, so rounding the ceiling down as well only stops
	 * the automatic exposure short of what the manual control offers -
	 * it reached 90% of the slider and then went to gain instead.
	 */
	slots = min_t(unsigned long, gc2145_params_frame_lines(params),
		      GC2145_EXPOSURE_REG_MAX);

	gc2145_tx_start(sensor);

	gc2145_tx_write16(sensor, GC2145_P1_AEC_ANTI_FLICKER_HI, step);

	for (i = 0; i < GC2145_AEC_EXP_LEVELS; i++) {
		level = step ? step * (i + 1) : slots;

		gc2145_tx_write16(sensor, GC2145_P1_AEC_EXP_LEVEL(i),
				  min(level, slots));
		gc2145_tx_write8(sensor, GC2145_P1_AEC_LEVEL_MAX_GAIN(i),
				 gc2145_pregain_max(a_gain));
	}

	/*
	 * Which gear the AEC may climb to. Only the first four levels are
	 * selectable, and they are all capped at a frame now, so let it
	 * have the highest one.
	 */
	gc2145_tx_update_bits(sensor, GC2145_P1_AEC_MAX_EXP_LEVEL,
			      GC2145_AEC_MAX_EXP_LEVEL,
			      GC2145_AEC_MAX_EXP_LEVEL);

	gc2145_tx_write8(sensor, GC2145_P1_AEC_MAX_PREGAIN,
			 gc2145_pregain_max(a_gain));
	gc2145_tx_write8(sensor, GC2145_P1_AEC_MAX_POSTGAIN,
			 GC2145_AEC_MAX_POSTGAIN);

	return gc2145_tx_commit(sensor);
}

/*
 * The sync register carries the bus signal polarities, which depend on the
 * board, and the row and column switches, which pick the Bayer phase and
 * so depend on the format. One write owns the register, so both have to be
 * composed here and written together rather than read-modify-written from
 * two places.
 */
static u8 gc2145_sync_mode(struct gc2145_dev *sensor,
			   const struct gc2145_pixfmt *pix_fmt)
{
	struct v4l2_mbus_config_parallel *bus = &sensor->ep.bus.parallel;
	u8 sync_mode = 0;

	if (bus->flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)
		sync_mode |= 0x01;

	if (bus->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)
		sync_mode |= 0x02;

	if (bus->flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)
		sync_mode |= 0x04;

	if (pix_fmt)
		sync_mode |= pix_fmt->sync_switch;

	return sync_mode;
}

static int gc2145_setup_mode(struct gc2145_dev *sensor,
			     struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	int ret, pad;
	struct gc2145_sensor_params params;
	unsigned long pclk2;
	unsigned long width = fmt->width;
	unsigned long height = fmt->height;
	const struct gc2145_pixfmt *pix_fmt;

	pix_fmt = gc2145_find_format(fmt->code);
	if (!pix_fmt) {
		dev_err(&sensor->i2c_client->dev,
			"pixel format not supported %u\n", fmt->code);
		return -EINVAL;
	}

	/*
	 * The timing was worked out when the format, the crop or the frame
	 * rate last changed, and the vertical blanking control may have
	 * moved since, so use what the controls describe rather than
	 * recomputing and undoing it.
	 */
	params = sensor->params;
	pclk2 = GC2145_PCLK_RATE;

	gc2145_params_apply(sensor, &params);

	ret = gc2145_set_2pclk(sensor, &pclk2, true);
	if (ret < 0)
		return ret;

	/* the measure windows are in output coordinates */
	width = params.out_width;
	height = params.out_height;

	pad = (width > 256 && height > 256) ? 32 : 16;

	ret = gc2145_setup_awb(sensor, pad, pad, width - pad * 2, height - pad * 2);
	if (ret)
		return ret;

	ret = gc2145_setup_aec(sensor,
				pad, pad, width - pad * 2, height - pad * 2,
				2 * pad, 2 * pad, width - pad * 4, height - pad * 4);
	if (ret)
		return ret;

	ret = gc2145_setup_aec_range(sensor, &params);
	if (ret)
		return ret;

	gc2145_tx_start(sensor);

	gc2145_tx_write8(sensor, GC2145_P0_ISP_OUT_FORMAT, pix_fmt->fmt_setup);

	/*
	 * Rewritten in full rather than updated, because the switch bits
	 * that pick the Bayer phase share the register with the bus
	 * polarities set at power-on.
	 */
	gc2145_tx_write8(sensor, GC2145_P0_SYNC_MODE,
			 gc2145_sync_mode(sensor, pix_fmt));

	/* drive strength, for the byte clock leaving the parallel port */
	gc2145_tx_write8(sensor, 0x24, pclk2 > 40000000 ? 0xff : 0x55);

	ret = gc2145_tx_commit(sensor);
	if (ret)
		return ret;

	sensor->pending_mode_change = false;
	return 0;
}

static int gc2145_set_stream(struct gc2145_dev *sensor, int enable)
{
	gc2145_tx_start(sensor);

	gc2145_tx_write8(sensor, GC2145_REG_PAD_IO, enable ? 0x0f : 0);

	//XXX: maybe disable cam module function blocks that are not used
	//and downclock the PLL/disable it when not streaming?

	return gc2145_tx_commit(sensor);
}

static int gc2145_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	ret = gc2145_setup_mode(sensor, state);
	if (ret)
		goto err_rpm_put;

	/*
	 * The mode setup rewrites the AEC and AWB blocks, so reapply the
	 * controls on top of it. The handler lock is our own mutex, which
	 * the caller already holds, hence the unlocked variant.
	 */
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	if (ret)
		goto err_rpm_put;

	ret = gc2145_set_stream(sensor, true);
	if (ret)
		goto err_rpm_put;

	/*
	 * A flip reorders the pixel array readout. The ISP compensates for
	 * that in the processed formats, but in the raw ones it moves the
	 * Bayer phase, which would no longer match the media bus code the
	 * format describes. Hold the flips still there only.
	 */
	if (gc2145_format_is_raw(fmt->code)) {
		__v4l2_ctrl_grab(sensor->ctrls.hflip, true);
		__v4l2_ctrl_grab(sensor->ctrls.vflip, true);
		sensor->flips_grabbed = true;
	}

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int gc2145_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	int ret;

	ret = gc2145_set_stream(sensor, false);

	if (sensor->flips_grabbed) {
		__v4l2_ctrl_grab(sensor->ctrls.hflip, false);
		__v4l2_ctrl_grab(sensor->ctrls.vflip, false);
		sensor->flips_grabbed = false;
	}

	pm_runtime_put_autosuspend(&sensor->i2c_client->dev);

	return ret;
}

/* }}} */
/* {{{ Pad ops */

static int gc2145_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct gc2145_ctrls *ctrls = &sensor->ctrls;
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	u64 lines;

	if (fi->pad != 0)
		return -EINVAL;

	/* whatever the blanking controls currently describe */
	lines = (u64)(fmt->width + ctrls->hblank->val)
		* (fmt->height + ctrls->vblank->val);

	fi->interval.numerator = 1;
	fi->interval.denominator = div64_u64(*ctrls->pixel_rate->p_cur.p_s64 + lines / 2,
					     lines);

	return 0;
}

static int gc2145_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	int fps;

	if (fi->pad != 0)
		return -EINVAL;

	/* user requested infinite frame rate */
	if (fi->interval.numerator == 0)
		fps = 60;
	else
		fps = DIV_ROUND_CLOSEST(fi->interval.denominator,
					fi->interval.numerator);

	fps = clamp(fps, 1, 60);

	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = fps;
	sensor->pending_mode_change = true;

	/*
	 * The frame rate lives in the vertical blanking control. This is
	 * only a way of setting it that predates that, so let it pick the
	 * blanking and report back what was actually reached.
	 */
	gc2145_update_timing_ctrls(sensor, state);
	gc2145_get_frame_interval(sd, state, fi);

	return 0;
}

static int gc2145_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0 || code->index >= ARRAY_SIZE(gc2145_formats))
		return -EINVAL;

	code->code = gc2145_formats[code->index].code;

	return 0;
}

/*
 * The sizes on offer are the analogue crop decimated by each of the
 * ratios the sensor can produce, so they follow the crop rectangle and
 * every one of them shows the same field of view.
 */
static int gc2145_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct gc2145_mode *mode;
	const struct v4l2_rect *crop;
	u32 width, height;

	if (fse->pad != 0 || !gc2145_find_format(fse->code))
		return -EINVAL;

	/* the raw formats come in one size, for the reason above */
	if (gc2145_format_is_raw(fse->code)) {
		if (fse->index > 0)
			return -EINVAL;
		mode = gc2145_raw_mode();
	} else {
		if (fse->index >= ARRAY_SIZE(gc2145_modes))
			return -EINVAL;
		mode = &gc2145_modes[fse->index];
	}

	crop = v4l2_subdev_state_get_crop(sd_state, 0);
	gc2145_mode_output_size(mode, crop, &width, &height);

	if (width < GC2145_SENSOR_WIDTH_MIN ||
	    height < GC2145_SENSOR_HEIGHT_MIN)
		return -EINVAL;

	fse->min_width = width;
	fse->max_width = width;
	fse->min_height = height;
	fse->max_height = height;

	return 0;
}

/*
 * The frame rate is continuous between the shortest frame the mode can
 * produce and the longest the blanking register can reach, so there is
 * nothing discrete to enumerate. Report the fastest one, and leave the
 * rest to the vertical blanking control.
 */
static int gc2145_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct gc2145_sensor_params params;
	unsigned long pclk;
	u64 lines;

	if (fie->pad != 0 || fie->index > 0)
		return -EINVAL;

	params = gc2145_compute_params(sensor,
				       v4l2_subdev_state_get_crop(sd_state, 0),
				       fie->width, fie->height, 0, &pclk);

	lines = (u64)(params.out_width + gc2145_params_hblank(&params))
		* (params.out_height + gc2145_params_vblank(&params));

	fie->interval.numerator = 1;
	fie->interval.denominator =
		div64_u64(gc2145_params_pixel_rate(&params, pclk) + lines / 2,
			  lines);

	return 0;
}

static int gc2145_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct gc2145_pixfmt *pixfmt;
	const struct gc2145_mode *mode;
	const struct v4l2_rect *crop;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && sd->enabled_pads)
		return -EBUSY;

	/* check if we support requested mbus fmt */
	pixfmt = gc2145_find_format(mf->code);
	if (!pixfmt)
		pixfmt = &gc2145_formats[0];

	mf->code = pixfmt->code;
	mf->colorspace = pixfmt->colorspace;
	mf->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	mf->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mf->quantization = V4L2_QUANTIZATION_DEFAULT;
	mf->field = V4L2_FIELD_NONE;

	/*
	 * Snap to the nearest size the decimation chain can produce out of
	 * the current analogue crop, rather than windowing the array down
	 * to the requested size and throwing away the field of view.
	 */
	crop = v4l2_subdev_state_get_crop(sd_state, 0);
	mode = gc2145_format_is_raw(mf->code) ? gc2145_raw_mode()
					      : gc2145_find_mode(crop, mf->width,
								 mf->height);
	gc2145_mode_output_size(mode, crop, &mf->width, &mf->height);

	*v4l2_subdev_state_get_format(sd_state, 0) = *mf;

	if (format->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	sensor->pending_mode_change = true;
	gc2145_update_timing_ctrls(sensor, sd_state);

	return 0;
}

static int gc2145_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->pad != 0)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = GC2145_NATIVE_WIDTH;
		sel->r.height = GC2145_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = GC2145_ACTIVE_LEFT;
		sel->r.top = GC2145_ACTIVE_TOP;
		sel->r.width = GC2145_SENSOR_WIDTH_MAX;
		sel->r.height = GC2145_SENSOR_HEIGHT_MAX;
		return 0;
	}

	return -EINVAL;
}

/*
 * The crop is the part of the pixel array that is read out, so it is
 * what decides the field of view. Narrowing it also shortens the row
 * and frame periods, which is the only way to go faster than the
 * decimation ratios allow.
 */
static int gc2145_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_rect *crop;
	const struct gc2145_mode *mode;

	if (sel->pad != 0 || sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE && sd->enabled_pads)
		return -EBUSY;

	/*
	 * The readout window has to stay even so that the Bayer phase of
	 * the crop matches the one the ISP is set up for.
	 */
	sel->r.width = clamp_t(u32, round_down(sel->r.width, 4),
			       GC2145_SENSOR_WIDTH_MIN,
			       GC2145_SENSOR_WIDTH_MAX);
	sel->r.height = clamp_t(u32, round_down(sel->r.height, 4),
				GC2145_SENSOR_HEIGHT_MIN,
				GC2145_SENSOR_HEIGHT_MAX);
	sel->r.left = clamp_t(u32, round_down(sel->r.left, 2),
			      GC2145_ACTIVE_LEFT,
			      GC2145_ACTIVE_LEFT + GC2145_SENSOR_WIDTH_MAX -
			      sel->r.width);
	sel->r.top = clamp_t(u32, round_down(sel->r.top, 2),
			     GC2145_ACTIVE_TOP,
			     GC2145_ACTIVE_TOP + GC2145_SENSOR_HEIGHT_MAX -
			     sel->r.height);

	crop = v4l2_subdev_state_get_crop(state, 0);
	*crop = sel->r;

	/* the sizes on offer moved with the crop, so refit the format */
	fmt = v4l2_subdev_state_get_format(state, 0);
	mode = gc2145_format_is_raw(fmt->code) ? gc2145_raw_mode()
					       : gc2145_find_mode(crop,
								  fmt->width,
								  fmt->height);
	gc2145_mode_output_size(mode, crop, &fmt->width, &fmt->height);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	sensor->pending_mode_change = true;
	gc2145_update_timing_ctrls(sensor, state);

	return 0;
}

static int gc2145_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = gc2145_formats[0].code,
			.width = GC2145_DEFAULT_WIDTH,
			.height = GC2145_DEFAULT_HEIGHT,
		},
	};

	*v4l2_subdev_state_get_crop(state, 0) = (struct v4l2_rect) {
		.left = GC2145_ACTIVE_LEFT,
		.top = GC2145_ACTIVE_TOP,
		.width = GC2145_SENSOR_WIDTH_MAX,
		.height = GC2145_SENSOR_HEIGHT_MAX,
	};

	return gc2145_set_fmt(sd, state, &fmt);
}

/* }}} */
/* {{{ Core Ops */

static int gc2145_identify(struct gc2145_dev *sensor)
{
	u16 chip_id;
	int ret;

	ret = gc2145_read16(sensor, GC2145_REG_CHIP_ID, &chip_id);
	if (ret)
		return ret;

	if (chip_id != GC2145_REG_CHIP_ID_VALUE) {
		dev_err(&sensor->i2c_client->dev,
			"unsupported device id: 0x%04x\n",
			(unsigned int)chip_id);
		return -ENODEV;
	}

	dev_dbg(&sensor->i2c_client->dev, "device id: 0x%04x\n",
		(unsigned int)chip_id);

	return 0;
}

static int gc2145_configure(struct gc2145_dev *sensor)
{
	u8 sync_mode = gc2145_sync_mode(sensor, NULL);
	int ret;

        // setup parallel bus

	gc2145_tx_start(sensor);

	// soft reset
	gc2145_tx_write8(sensor, GC2145_REG_RESET, 0xf0);

	// enable analog/digital parts
	gc2145_tx_write8(sensor, GC2145_REG_ANALOG_PWC, 0x06);

	// safe initial PLL setting
	gc2145_tx_write8(sensor, GC2145_REG_PLL_MODE1, 0x1d);
	gc2145_tx_write8(sensor, GC2145_REG_PLL_MODE2, 0x84);
	gc2145_tx_write8(sensor, GC2145_REG_CLK_DIV_MODE, 0x00);

	gc2145_tx_write8(sensor, GC2145_REG_CM_MODE, 0xfe);

	// disable pads
	gc2145_tx_write8(sensor, GC2145_REG_PAD_IO, 0);

	gc2145_tx_write8(sensor, 0x19, 0x0c); // set AD pipe number
	gc2145_tx_write8(sensor, 0x20, 0x01); // AD clk mode

	/*
	 * Edge enhancement is what the sharpness control drives and what
	 * the emboss and sketch effects are made of: both of those are
	 * nothing but the edge map bits, so with the block off they look
	 * like a plain greyscale and like nothing at all.
	 */
	/*
	 * One write owns this register. Setting a bit of it from somewhere
	 * else later only works if that somewhere always runs afterwards,
	 * and the gamma bit was being set that way and reaching the sensor
	 * clear: read back from a phone at a clean boot, this register was
	 * exactly the value below and the fully programmed gamma curve at
	 * P2:0x10-0x25 had never been switched on.
	 *
	 * Denoise stays off. It is perceptible but the strength knobs are
	 * not, and whether it is wanted depends on the light.
	 */
	gc2145_tx_write8(sensor, GC2145_P0_ISP_BLK_ENABLE1,
			 GC2145_ISP_BLK_ENABLE1_LSC |
			 GC2145_ISP_BLK_ENABLE1_DD |
			 GC2145_ISP_BLK_ENABLE1_INTERPOLATION |
			 GC2145_ISP_BLK_ENABLE1_EE |
			 GC2145_ISP_BLK_ENABLE1_GAMMA);

	gc2145_tx_write8(sensor, GC2145_P0_SYNC_MODE, sync_mode);

	ret = gc2145_tx_commit(sensor);
	if (ret)
		return ret;

	// load default register values from the firmware file
	ret = gc2145_load_firmware(sensor, GC2145_FIRMWARE_PARAMETERS);
	if (ret < 0)
		return ret;

	return 0;
}

static void gc2145_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc2145_dev *sensor = to_gc2145_dev(sd);

	clk_disable_unprepare(sensor->xclk);

	/*
	 * The supplies are shared with the other camera, so they may well
	 * stay up. Keep the sensor held in reset instead of letting the
	 * pins float, or it sits powered in an undefined state.
	 */
	gpiod_set_value(sensor->enable_gpio, 0);
	gpiod_set_value(sensor->reset_gpio, 1);

	regulator_bulk_disable(GC2145_NUM_SUPPLIES, sensor->supplies);
	msleep(100);

	/* the register cache in the bank tracker is gone with the power */
	sensor->current_bank = 0xff;
}

static int gc2145_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	int ret;

	/* the sensor is held in reset already, see the pin request */
	ret = regulator_bulk_enable(GC2145_NUM_SUPPLIES, sensor->supplies);
	if (ret)
		return ret;

	usleep_range(10000, 12000);

	ret = clk_set_rate(sensor->xclk, 24000000);
	if (ret)
		goto power_off;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret)
		goto power_off;

	usleep_range(10000, 12000);
	gpiod_set_value(sensor->enable_gpio, 1);
	usleep_range(10000, 12000);
	gpiod_set_value(sensor->reset_gpio, 0);
	usleep_range(40000, 50000);

	sensor->current_bank = 0xff;
	return 0;

power_off:
	gpiod_set_value(sensor->enable_gpio, 0);
	gpiod_set_value(sensor->reset_gpio, 1);
	regulator_bulk_disable(GC2145_NUM_SUPPLIES, sensor->supplies);
	return ret;
}

static int gc2145_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	int ret;

	ret = gc2145_power_on(dev);
	if (ret)
		return ret;

	ret = gc2145_configure(sensor);
	if (ret) {
		gc2145_power_off(dev);
		return ret;
	}

	/* the mode is applied when streaming starts */
	sensor->pending_mode_change = true;
	return 0;
}

static int gc2145_runtime_suspend(struct device *dev)
{
	gc2145_power_off(dev);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int gc2145_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct v4l2_subdev_state *state;
	int ret;
	u8 val = 0;

	if (reg->reg > 0xffff)
		return -EINVAL;

	reg->size = 1;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	ret = gc2145_read(sensor, reg->reg, &val);
	v4l2_subdev_unlock_state(state);
	if (ret)
		return -EIO;

	reg->val = val;
	return 0;
}

static int gc2145_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct v4l2_subdev_state *state;
	int ret;

	if (reg->reg > 0xffff || reg->val > 0xff)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	ret = gc2145_write(sensor, reg->reg, reg->val);
	v4l2_subdev_unlock_state(state);

	return ret;
}
#endif

/* }}} */

static const struct v4l2_subdev_core_ops gc2145_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = gc2145_g_register,
	.s_register = gc2145_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops gc2145_pad_ops = {
	.enum_mbus_code = gc2145_enum_mbus_code,
	.enum_frame_size = gc2145_enum_frame_size,
	.enum_frame_interval = gc2145_enum_frame_interval,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc2145_set_fmt,
	.get_selection = gc2145_get_selection,
	.set_selection = gc2145_set_selection,
	.get_frame_interval = gc2145_get_frame_interval,
	.set_frame_interval = gc2145_set_frame_interval,
	.enable_streams = gc2145_enable_streams,
	.disable_streams = gc2145_disable_streams,
};

static const struct v4l2_subdev_video_ops gc2145_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_ops gc2145_subdev_ops = {
	.core = &gc2145_core_ops,
	.pad = &gc2145_pad_ops,
	.video = &gc2145_video_ops,
};

static const struct v4l2_subdev_internal_ops gc2145_internal_ops = {
	.init_state = gc2145_init_state,
};

static int gc2145_get_regulators(struct gc2145_dev *sensor)
{
	int i;

	for (i = 0; i < GC2145_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = gc2145_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       GC2145_NUM_SUPPLIES,
				       sensor->supplies);
}

static int gc2145_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct gc2145_dev *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	v4l2_i2c_subdev_init(&sensor->sd, client, &gc2145_subdev_ops);
	sensor->sd.internal_ops = &gc2145_internal_ops;

	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = GC2145_DEFAULT_FRAMERATE;
	sensor->pending_mode_change = true;
	sensor->current_bank = 0xff;
	sensor->power_line_freq = 50;

	endpoint = fwnode_graph_get_next_endpoint(
		of_fwnode_handle(client->dev.of_node), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "could not parse endpoint\n");
		return ret;
	}

	if (sensor->ep.bus_type != V4L2_MBUS_PARALLEL) {
		dev_err(dev, "unsupported bus type %d\n", sensor->ep.bus_type);
		return -EINVAL;
	}

	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}

	/*
	 * Drive the pins from the start: the supplies are shared with the
	 * other camera on this bus, so this sensor can be powered long
	 * before the driver first powers it on, and it must spend that
	 * time held in reset rather than with its control pins floating.
	 */
	sensor->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						      GPIOD_OUT_LOW);
	if (IS_ERR(sensor->enable_gpio)) {
		dev_err(dev, "failed to get enable gpio\n");
		return PTR_ERR(sensor->enable_gpio);
	}

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio)) {
		dev_err(dev, "failed to get reset gpio\n");
		return PTR_ERR(sensor->reset_gpio);
	}

	if (!sensor->enable_gpio || !sensor->reset_gpio) {
		dev_err(dev, "enable and reset pins must be configured\n");
		return -EINVAL;
	}

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE |
			   V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = gc2145_get_regulators(sensor);
	if (ret)
		goto entity_cleanup;

	/* the sensor has to be powered for the chip id to be readable */
	ret = gc2145_power_on(dev);
	if (ret)
		goto entity_cleanup;

	ret = gc2145_identify(sensor);
	if (ret) {
		/*
		 * The camera is not reachable this early during boot on
		 * some boards - see the same comment in the hm5065 driver.
		 * Losing the probe here costs the whole camera stack, since
		 * the bridge waits for every sensor before it registers
		 * anything, so ask to be probed again later.
		 */
		if (ret != -ENODEV)
			ret = -EPROBE_DEFER;

		goto power_off;
	}

	ret = gc2145_init_controls(sensor);
	if (ret)
		goto power_off;

	/*
	 * Share one lock between the control handler and the sub-device
	 * state, so that a control handler callback can look at the active
	 * format without taking a second lock.
	 */
	sensor->sd.state_lock = sensor->ctrls.handler.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto free_ctrls;

	/*
	 * Hand the power we are holding over to runtime PM, then let it go
	 * idle once the sub-device is registered.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto rpm_disable;

	pm_runtime_idle(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	return 0;

rpm_disable:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&sensor->sd);
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
power_off:
	gc2145_power_off(dev);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static void gc2145_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2145_dev *sensor = to_gc2145_dev(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(&sensor->sd);
	v4l2_subdev_cleanup(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		gc2145_power_off(dev);
	pm_runtime_set_suspended(dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc2145_pm_ops, gc2145_runtime_suspend,
				 gc2145_runtime_resume, NULL);

static const struct i2c_device_id gc2145_id[] = {
	{"gc2145", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, gc2145_id);

static const struct of_device_id gc2145_dt_ids[] = {
	{ .compatible = "galaxycore,gc2145" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc2145_dt_ids);

static struct i2c_driver gc2145_i2c_driver = {
	.driver = {
		.name  = "gc2145",
		.of_match_table	= gc2145_dt_ids,
		.pm = pm_ptr(&gc2145_pm_ops),
	},
	.id_table = gc2145_id,
	.probe    = gc2145_probe,
	.remove   = gc2145_remove,
};

module_i2c_driver(gc2145_i2c_driver);

MODULE_AUTHOR("Ondrej Jirman <megi@xff.cz>");
MODULE_DESCRIPTION("GC2145 Camera Subdev Driver");
MODULE_LICENSE("GPL");
