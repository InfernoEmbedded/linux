/*
 * Himax HM5065 driver.
 * Copyright (C) 2017-2019 Ondřej Jirman <megi@xff.cz>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define HM5065_AF_FIRMWARE		"hm5065-af.bin"
#define HM5065_FIRMWARE_PARAMETERS	"hm5065-init.bin"

#define HM5065_SENSOR_WIDTH	2592u
#define HM5065_SENSOR_HEIGHT	1944u
#define HM5065_CAPTURE_WIDTH_MIN	88u
#define HM5065_CAPTURE_HEIGHT_MIN	72u

/* {{{ Register definitions */

/* the CCI register macros encode each register's width */

/* device parameters */
#define HM5065_REG_DEVICE_ID			CCI_REG16(0x0000)
#define HM5065_REG_DEVICE_ID_VALUE		0x039e
#define HM5065_REG_FIRMWARE_VSN			CCI_REG8(0x0002)
#define HM5065_REG_PATCH_VSN			CCI_REG8(0x0003)
#define HM5065_REG_EXCLOCKLUT			CCI_REG8(0x0009) /* standby */

#define HM5065_REG_INT_EVENT_FLAG		CCI_REG8(0x000a)
#define HM5065_REG_INT_EVENT_FLAG_OP_MODE	BIT(0)
#define HM5065_REG_INT_EVENT_FLAG_CAM_MODE	BIT(1)
#define HM5065_REG_INT_EVENT_FLAG_JPEG_STATUS	BIT(2)
#define HM5065_REG_INT_EVENT_FLAG_NUM_FRAMES	BIT(3)
#define HM5065_REG_INT_EVENT_FLAG_AF_LOCKED	BIT(4)

/* mode manager */
#define HM5065_REG_USER_COMMAND			CCI_REG8(0x0010)
#define HM5065_REG_USER_COMMAND_STOP		0x00
#define HM5065_REG_USER_COMMAND_RUN		0x01
#define HM5065_REG_USER_COMMAND_POWEROFF	0x02

#define HM5065_REG_STATE			CCI_REG8(0x0011)
#define HM5065_REG_STATE_RAW			0x10
#define HM5065_REG_STATE_IDLE			0x20
#define HM5065_REG_STATE_RUNNING		0x30

#define HM5065_REG_ACTIVE_PIPE_SETUP_BANK	CCI_REG8(0x0012)
#define HM5065_REG_ACTIVE_PIPE_SETUP_BANK_0	0x00
#define HM5065_REG_ACTIVE_PIPE_SETUP_BANK_1	0x01

#define HM5065_REG_NUMBER_OF_FRAMES_STREAMED	CCI_REG8(0x0014) /* ro */
#define HM5065_REG_REQUIRED_STREAM_LENGTH	CCI_REG8(0x0015)

#define HM5065_REG_CSI_ENABLE			CCI_REG8(0x0016) /* standby */
#define HM5065_REG_CSI_ENABLE_DISABLE		0x00
#define HM5065_REG_CSI_ENABLE_CSI2_1LANE	0x01
#define HM5065_REG_CSI_ENABLE_CSI2_2LANE	0x02

/* pipe setup bank 0 */
#define HM5065_REG_P0_SENSOR_MODE		CCI_REG8(0x0040)
#define HM5065_REG_SENSOR_MODE_FULLSIZE		0x00
#define HM5065_REG_SENSOR_MODE_BINNING_2X2	0x01
#define HM5065_REG_SENSOR_MODE_BINNING_4X4	0x02
#define HM5065_REG_SENSOR_MODE_SUBSAMPLING_2X2	0x03
#define HM5065_REG_SENSOR_MODE_SUBSAMPLING_4X4	0x04

#define HM5065_REG_P0_IMAGE_SIZE		CCI_REG8(0x0041)
#define HM5065_REG_IMAGE_SIZE_5MP		0x00
#define HM5065_REG_IMAGE_SIZE_UXGA		0x01
#define HM5065_REG_IMAGE_SIZE_SXGA		0x02
#define HM5065_REG_IMAGE_SIZE_SVGA		0x03
#define HM5065_REG_IMAGE_SIZE_VGA		0x04
#define HM5065_REG_IMAGE_SIZE_CIF		0x05
#define HM5065_REG_IMAGE_SIZE_QVGA		0x06
#define HM5065_REG_IMAGE_SIZE_QCIF		0x07
#define HM5065_REG_IMAGE_SIZE_QQVGA		0x08
#define HM5065_REG_IMAGE_SIZE_QQCIF		0x09
#define HM5065_REG_IMAGE_SIZE_MANUAL		0x0a

#define HM5065_REG_P0_MANUAL_HSIZE		CCI_REG16(0x0042)
#define HM5065_REG_P0_MANUAL_VSIZE		CCI_REG16(0x0044)

#define HM5065_REG_P0_DATA_FORMAT		CCI_REG8(0x0046)
#define HM5065_REG_DATA_FORMAT_YCBCR_JFIF       0x00
#define HM5065_REG_DATA_FORMAT_YCBCR_REC601     0x01
#define HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM     0x02
#define HM5065_REG_DATA_FORMAT_RGB_565          0x03
#define HM5065_REG_DATA_FORMAT_RGB_565_CUSTOM   0x04
#define HM5065_REG_DATA_FORMAT_RGB_444          0x05
#define HM5065_REG_DATA_FORMAT_RGB_555          0x06
#define HM5065_REG_DATA_FORMAT_RAW10ITU10       0x07
#define HM5065_REG_DATA_FORMAT_RAW10ITU8        0x08
#define HM5065_REG_DATA_FORMAT_JPEG             0x09

#define HM5065_REG_P0_GAMMA_GAIN		CCI_REG8(0x0049) /* 0-31 */
#define HM5065_REG_P0_GAMMA_INTERPOLATION	CCI_REG8(0x004a) /* 0-16 */
#define HM5065_REG_P0_PEAKING_GAIN		CCI_REG8(0x004c) /* 0-63 */

#define HM5065_REG_P0_JPEG_SQUEEZE_MODE		CCI_REG8(0x004d)
#define HM5065_REG_JPEG_SQUEEZE_MODE_USER	0x00
#define HM5065_REG_JPEG_SQUEEZE_MODE_AUTO	0x01

#define HM5065_REG_P0_JPEG_TARGET_FILE_SIZE	CCI_REG16(0x004e) /* kB */
#define HM5065_REG_P0_JPEG_IMAGE_QUALITY	CCI_REG8(0x0050)
#define HM5065_REG_JPEG_IMAGE_QUALITY_HIGH	0x00
#define HM5065_REG_JPEG_IMAGE_QUALITY_MEDIUM	0x01
#define HM5065_REG_JPEG_IMAGE_QUALITY_LOW	0x02

/* pipe setup bank 1 (only register indexes) */
#define HM5065_REG_P1_SENSOR_MODE		CCI_REG8(0x0060)
#define HM5065_REG_P1_IMAGE_SIZE		CCI_REG8(0x0061)
#define HM5065_REG_P1_MANUAL_HSIZE		CCI_REG16(0x0062)
#define HM5065_REG_P1_MANUAL_VSIZE		CCI_REG16(0x0064)
#define HM5065_REG_P1_DATA_FORMAT		CCI_REG8(0x0066)
#define HM5065_REG_P1_GAMMA_GAIN		CCI_REG8(0x0069) /* 0-31 */
#define HM5065_REG_P1_GAMMA_INTERPOLATION	CCI_REG8(0x006a) /* 0-16 */
#define HM5065_REG_P1_PEAKING_GAIN		CCI_REG8(0x006c) /* 0-63 */
#define HM5065_REG_P1_JPEG_SQUEEZE_MODE		CCI_REG8(0x006d)
#define HM5065_REG_P1_JPEG_TARGET_FILE_SIZE	CCI_REG16(0x006e) /* kB */
#define HM5065_REG_P1_JPEG_IMAGE_QUALITY	CCI_REG8(0x0070)

/* pipe setup - common registers */
#define HM5065_REG_CONTRAST			CCI_REG8(0x0080) /* 0-200 */
#define HM5065_REG_COLOR_SATURATION		CCI_REG8(0x0081) /* 0-200 */
#define HM5065_REG_BRIGHTNESS			CCI_REG8(0x0082) /* 0-200 */
#define HM5065_REG_HORIZONTAL_MIRROR		CCI_REG8(0x0083) /* 0,1 */
#define HM5065_REG_VERTICAL_FLIP		CCI_REG8(0x0084) /* 0,1 */

#define HM5065_REG_YCRCB_ORDER			CCI_REG8(0x0085)
#define HM5065_REG_YCRCB_ORDER_CB_Y_CR_Y	0x00
#define HM5065_REG_YCRCB_ORDER_CR_Y_CB_Y	0x01
#define HM5065_REG_YCRCB_ORDER_Y_CB_Y_CR	0x02
#define HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB	0x03

/* clock chain parameter inputs (floating point) */
#define HM5065_REG_EXTERNAL_CLOCK_FREQ_MHZ	CCI_REG16(0x00b0) /* fp16, 6-27, standby */
#define HM5065_REG_TARGET_PLL_OUTPUT	CCI_REG16(0x00b2) /* fp16, 450-1000, standby */

/* static frame rate control */
#define HM5065_REG_DESIRED_FRAME_RATE_NUM	CCI_REG16(0x00c8)
#define HM5065_REG_DESIRED_FRAME_RATE_DEN	CCI_REG8(0x00ca)

/* static frame rate status */
#define HM5065_REG_REQUESTED_FRAME_RATE_HZ	CCI_REG16(0x00d8) /* fp16 */
#define HM5065_REG_MAX_FRAME_RATE_HZ		CCI_REG16(0x00da) /* fp16 */
#define HM5065_REG_MIN_FRAME_RATE_HZ		CCI_REG16(0x00dc) /* fp16 */

/* automatic frame rate control (reverse engineered, not in the datasheet) */
#define HM5065_REG_AFR_ENABLE			CCI_REG8(0x00e8) /* 0,1 */
#define HM5065_REG_AFR_MIN_FPS			CCI_REG8(0x00ed)
#define HM5065_REG_AFR_MAX_FPS			CCI_REG8(0x00ee)

/* exposure controls */
#define HM5065_REG_EXPOSURE_MODE			CCI_REG8(0x0128)
#define HM5065_REG_EXPOSURE_MODE_AUTO			0x00
#define HM5065_REG_EXPOSURE_MODE_COMPILED_MANUAL	0x01
#define HM5065_REG_EXPOSURE_MODE_DIRECT_MANUAL		0x02

#define HM5065_REG_EXPOSURE_METERING		CCI_REG8(0x0129)
#define HM5065_REG_EXPOSURE_METERING_FLAT	0x00
#define HM5065_REG_EXPOSURE_METERING_BACKLIT	0x01
#define HM5065_REG_EXPOSURE_METERING_CENTERED	0x02

#define HM5065_REG_MANUAL_EXPOSURE_TIME_NUM	CCI_REG8(0x012a)
#define HM5065_REG_MANUAL_EXPOSURE_TIME_DEN	CCI_REG8(0x012b)
#define HM5065_REG_MANUAL_EXPOSURE_TIME_US	CCI_REG16(0x012c) /* fp16 */
#define HM5065_REG_COLD_START_DESIRED_TIME_US	CCI_REG16(0x012e) /* fp16, standby */
#define HM5065_REG_EXPOSURE_COMPENSATION	CCI_REG8(0x0130) /* s8, -7 - +7 */

#define HM5065_REG_DIRECT_MODE_COARSE_INTEGRATION_LINES	CCI_REG16(0x0132)
#define HM5065_REG_DIRECT_MODE_FINE_INTEGRATION_PIXELS	CCI_REG16(0x0134)
#define HM5065_REG_DIRECT_MODE_CODED_ANALOG_GAIN	CCI_REG16(0x0136)
#define HM5065_REG_DIRECT_MODE_DIGITAL_GAIN		CCI_REG16(0x0138) /* fp16 */
#define HM5065_REG_FREEZE_AUTO_EXPOSURE			CCI_REG8(0x0142) /* 0,1 */
#define HM5065_REG_USER_MAXIMUM_INTEGRATION_TIME_US	CCI_REG16(0x0143) /* fp16 */
#define HM5065_REG_ANTI_FLICKER_MODE			CCI_REG8(0x0148) /* 0,1 */

/* exposure algorithm controls */
#define HM5065_REG_DIGITAL_GAIN_FLOOR			CCI_REG16(0x015c) /* fp16 */
#define HM5065_REG_DIGITAL_GAIN_CEILING			CCI_REG16(0x015e) /* fp16 */
#define HM5065_REG_ANALOG_GAIN_FLOOR			CCI_REG16(0x02c0)
#define HM5065_REG_ANALOG_GAIN_CEILING			CCI_REG16(0x02c2)

/* exposure status */
#define HM5065_REG_COARSE_INTEGRATION			CCI_REG16(0x017c)
#define HM5065_REG_FINE_INTEGRATION_PENDING_PIXELS	CCI_REG16(0x017e)
#define HM5065_REG_ANALOG_GAIN_PENDING			CCI_REG16(0x0180) /* fp16 */
#define HM5065_REG_DIGITAL_GAIN_PENDING			CCI_REG16(0x0182) /* fp16 */
#define HM5065_REG_DESIRED_EXPOSURE_TIME_US		CCI_REG16(0x0184) /* fp16 */
#define HM5065_REG_COMPILED_EXPOSURE_TIME_US		CCI_REG16(0x0186) /* fp16 */
#define HM5065_REG_USER_MAXIMUM_INTEGRATION_LINES	CCI_REG16(0x0189)
#define HM5065_REG_TOTAL_INTEGRATION_TIME_PENDING_US	CCI_REG16(0x018b) /* fp16 */
#define HM5065_REG_CODED_ANALOG_GAIN_PENDING		CCI_REG16(0x018d)

/* flicker detect */
#define HM5065_REG_FD_ENABLE_DETECT			CCI_REG8(0x0190) /* 0,1 */
#define HM5065_REG_FD_DETECTION_START			CCI_REG8(0x0191) /* 0,1 */
#define HM5065_REG_FD_MAX_NUMBER_ATTEMP	CCI_REG8(0x0192) /* 0 = continuous */
#define HM5065_REG_FD_FLICKER_IDENTIFICATION_THRESHOLD	CCI_REG16(0x0193)
#define HM5065_REG_FD_WIN_TIMES				CCI_REG8(0x0195)
#define HM5065_REG_FD_FRAME_RATE_SHIFT_NUMBER		CCI_REG8(0x0196)
#define HM5065_REG_FD_MANUAL_FREF_ENABLE		CCI_REG8(0x0197) /* 0,1 */
#define HM5065_REG_FD_MANU_FREF_100			CCI_REG16(0x0198)
#define HM5065_REG_FD_MANU_FREF_120			CCI_REG16(0x019a)
#define HM5065_REG_FD_FLICKER_FREQUENCY			CCI_REG16(0x019c) /* fp16 */

/* white balance control */
#define HM5065_REG_WB_MODE			CCI_REG8(0x01a0)
#define HM5065_REG_WB_MODE_OFF			0x00
#define HM5065_REG_WB_MODE_AUTOMATIC		0x01
#define HM5065_REG_WB_MODE_AUTO_INSTANT		0x02
#define HM5065_REG_WB_MODE_MANUAL_RGB		0x03
#define HM5065_REG_WB_MODE_CLOUDY_PRESET	0x04
#define HM5065_REG_WB_MODE_SUNNY_PRESET		0x05
#define HM5065_REG_WB_MODE_LED_PRESET		0x06
#define HM5065_REG_WB_MODE_FLUORESCENT_PRESET	0x07
#define HM5065_REG_WB_MODE_TUNGSTEN_PRESET	0x08
#define HM5065_REG_WB_MODE_HORIZON_PRESET	0x09

#define HM5065_REG_WB_MANUAL_RED_GAIN		CCI_REG8(0x01a1)
#define HM5065_REG_WB_MANUAL_GREEN_GAIN		CCI_REG8(0x01a2)
#define HM5065_REG_WB_MANUAL_BLUE_GAIN		CCI_REG8(0x01a3)

#define HM5065_REG_WB_MISC_SETTINGS		CCI_REG8(0x01a4)
#define HM5065_REG_WB_MISC_SETTINGS_FREEZE_ALGO	BIT(2)

#define HM5065_REG_WB_HUE_R_BIAS		CCI_REG16(0x01a5) /* fp16 */
#define HM5065_REG_WB_HUE_B_BIAS		CCI_REG16(0x01a7) /* fp16 */

#define HM5065_REG_WB_STATUS			CCI_REG8(0x01c0)
#define HM5065_REG_WB_STATUS_STABLE		BIT(0)

#define HM5065_REG_WB_NORM_RED_GAIN		CCI_REG16(0x01c8) /* fp16 */
#define HM5065_REG_WB_PART_RED_GAIN		CCI_REG16(0x01e0) /* fp16 */
#define HM5065_REG_WB_PART_GREEN_GAIN		CCI_REG16(0x01e2) /* fp16 */
#define HM5065_REG_WB_PART_BLUE_GAIN		CCI_REG16(0x01e4) /* fp16 */

/* image stability status */
#define HM5065_REG_WHITE_BALANCE_STABLE		CCI_REG8(0x0291) /* 0,1 */
#define HM5065_REG_EXPOSURE_STABLE		CCI_REG8(0x0292) /* 0,1 */
#define HM5065_REG_STABLE			CCI_REG8(0x0294) /* 0,1 */

/* special effects */
#define HM5065_REG_EFFECTS_NEGATIVE		CCI_REG8(0x0380) /* 0,1 */
#define HM5065_REG_EFFECTS_SOLARISING		CCI_REG8(0x0381) /* 0,1 */
#define HM5065_REG_EFFECTS_SKECTH		CCI_REG8(0x0382) /* 0,1 */

#define HM5065_REG_EFFECTS_COLOR		CCI_REG8(0x0384)
#define HM5065_REG_EFFECTS_COLOR_NORMAL         0x00
#define HM5065_REG_EFFECTS_COLOR_RED_ONLY       0x01
#define HM5065_REG_EFFECTS_COLOR_YELLOW_ONLY    0x02
#define HM5065_REG_EFFECTS_COLOR_GREEN_ONLY     0x03
#define HM5065_REG_EFFECTS_COLOR_BLUE_ONLY      0x04
#define HM5065_REG_EFFECTS_COLOR_BLACK_WHITE    0x05
#define HM5065_REG_EFFECTS_COLOR_SEPIA          0x06
#define HM5065_REG_EFFECTS_COLOR_ANTIQUE        0x07
#define HM5065_REG_EFFECTS_COLOR_AQUA           0x08
#define HM5065_REG_EFFECTS_COLOR_MANUAL_MATRIX  0x09

/* anti-vignete, otp flash (skipped), page 79-89 */

/* flash control */
#define HM5065_REG_FLASH_MODE		CCI_REG8(0x02d0) /* 0,1 */
#define HM5065_REG_FLASH_RECOMMENDED	CCI_REG8(0x02d1) /* 0,1 */

/* test pattern */
#define HM5065_REG_ENABLE_TEST_PATTERN	CCI_REG8(0x05d8) /* 0,1 */

#define HM5065_REG_TEST_PATTERN				CCI_REG8(0x05d9)
#define HM5065_REG_TEST_PATTERN_NONE			0x00
#define HM5065_REG_TEST_PATTERN_HORIZONTAL_GREY_SCALE	0x01
#define HM5065_REG_TEST_PATTERN_VERTICAL_GREY_SCALE	0x02
#define HM5065_REG_TEST_PATTERN_DIAGONAL_GREY_SCALE	0x03
#define HM5065_REG_TEST_PATTERN_PN28			0x04
#define HM5065_REG_TEST_PATTERN_PN9			0x05
#define HM5065_REG_TEST_PATTERN_SOLID_COLOR		0x06
#define HM5065_REG_TEST_PATTERN_COLOR_BARS		0x07
#define HM5065_REG_TEST_PATTERN_GRADUATED_COLOR_BARS	0x08

#define HM5065_REG_TESTDATA_RED		CCI_REG16(0x4304) /* 0-1023 */
#define HM5065_REG_TESTDATA_GREEN_R	CCI_REG16(0x4308) /* 0-1023 */
#define HM5065_REG_TESTDATA_BLUE	CCI_REG16(0x430c) /* 0-1023 */
#define HM5065_REG_TESTDATA_GREEN_B	CCI_REG16(0x4310) /* 0-1023 */

/* contrast stretch */
#define HM5065_REG_CS_ENABLE			CCI_REG8(0x05e8) /* 0,1 */
#define HM5065_REG_CS_GAIN_CEILING		CCI_REG16(0x05e9) /* fp16 */
#define HM5065_REG_CS_BLACK_OFFSET_CEILING	CCI_REG8(0x05eb)
#define HM5065_REG_CS_WHITE_PIX_TARGET		CCI_REG16(0x05ec) /* fp16 */
#define HM5065_REG_CS_BLACK_PIX_TARGET		CCI_REG16(0x05ee) /* fp16 */
#define HM5065_REG_CS_ENABLED			CCI_REG8(0x05f8) /* 0,1 */
#define HM5065_REG_CS_TOTAL_PIXEL		CCI_REG16(0x05f9) /* fp16 */
#define HM5065_REG_CS_W_TARGET			CCI_REG32(0x05fb)
#define HM5065_REG_CS_B_TARGET			CCI_REG32(0x05ff)
#define HM5065_REG_CS_GAIN			CCI_REG16(0x0603) /* fp16 */
#define HM5065_REG_CS_BLACK_OFFSET		CCI_REG8(0x0605)
#define HM5065_REG_CS_WHITE_LIMIT		CCI_REG8(0x0606)

/* preset controls */
#define HM5065_REG_PRESET_LOADER_ENABLE		CCI_REG8(0x0638) /* 0,1, standby */

#define HM5065_REG_INDIVIDUAL_PRESET		CCI_REG8(0x0639) /* standby */
#define HM5065_REG_INDIVIDUAL_PRESET_ANTIVIGNETTE	BIT(0)
#define HM5065_REG_INDIVIDUAL_PRESET_WHITE_BALANCE	BIT(1)
#define HM5065_REG_INDIVIDUAL_PRESET_VCM		BIT(4)

/* jpeg control parameters*/
#define HM5065_REG_JPEG_STATUS			CCI_REG8(0x0649)
#define HM5065_REG_JPEG_RESTART			CCI_REG8(0x064a)
#define HM5065_REG_JPEG_HI_SQUEEZE_VALUE	CCI_REG8(0x064b) /* 5-255 (5 = max q.) */
#define HM5065_REG_JPEG_MED_SQUEEZE_VALUE	CCI_REG8(0x064c) /* 5-255 */
#define HM5065_REG_JPEG_LOW_SQUEEZE_VALUE	CCI_REG8(0x064d) /* 5-255 */
#define HM5065_REG_JPEG_LINE_LENGTH		CCI_REG16(0x064e) /* standby */
#define HM5065_REG_JPEG_CLOCK_RATIO		CCI_REG8(0x0650) /* 1-8, standby */
#define HM5065_REG_JPEG_THRES			CCI_REG16(0x0651) /* standby */
#define HM5065_REG_JPEG_BYTE_SENT		CCI_REG32(0x0653)

/* autofocus */

#define HM5065_REG_AF_WINDOWS_SYSTEM		CCI_REG8(0x065a)
#define HM5065_REG_AF_WINDOWS_SYSTEM_7_ZONES	0x00
#define HM5065_REG_AF_WINDOWS_SYSTEM_1_ZONE	0x01

#define HM5065_REG_AF_H_RATIO_NUM		CCI_REG8(0x065b)
#define HM5065_REG_AF_H_RATIO_DEN		CCI_REG8(0x065c)
#define HM5065_REG_AF_V_RATIO_NUM		CCI_REG8(0x065d)
#define HM5065_REG_AF_V_RATIO_DEN		CCI_REG8(0x065e)

#define HM5065_REG_AF_LENS_POSITION		CCI_REG16(0x06f0) /* ro, 0-1023 */
#define HM5065_REG_AF_LENS_IS_MOVING		CCI_REG8(0x06f2) /* ro 0,1 */
#define HM5065_REG_AF_TARGET_POSITION		CCI_REG16(0x0700) /* 0-1023 */

#define HM5065_REG_AF_RANGE			CCI_REG8(0x0709)
#define HM5065_REG_AF_RANGE_FULL		0x00
#define HM5065_REG_AF_RANGE_LANDSCAPE		0x01
#define HM5065_REG_AF_RANGE_MACRO		0x02

#define HM5065_REG_AF_MODE			CCI_REG8(0x070a)
#define HM5065_REG_AF_MODE_MANUAL		0x00
#define HM5065_REG_AF_MODE_CONTINUOUS		0x01
#define HM5065_REG_AF_MODE_SINGLE		0x03

#define HM5065_REG_AF_MODE_STATUS		CCI_REG8(0x0720)

#define HM5065_REG_AF_COMMAND			CCI_REG8(0x070b)
#define HM5065_REG_AF_COMMAND_NULL		0x00
#define HM5065_REG_AF_COMMAND_RELEASED_BUTTON	0x01
#define HM5065_REG_AF_COMMAND_HALF_BUTTON	0x02
#define HM5065_REG_AF_COMMAND_TAKE_SNAPSHOT	0x03
#define HM5065_REG_AF_COMMAND_REFOCUS		0x04

#define HM5065_REG_AF_LENS_COMMAND		CCI_REG8(0x070c)
#define HM5065_REG_AF_LENS_COMMAND_NULL				0x00
#define HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_INFINITY	0x01
#define HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_MACRO		0x02
#define HM5065_REG_AF_LENS_COMMAND_GOTO_INFINITY		0x03
#define HM5065_REG_AF_LENS_COMMAND_GOTO_MACRO			0x04
#define HM5065_REG_AF_LENS_COMMAND_GOTO_RECOVERY		0x05
#define HM5065_REG_AF_LENS_COMMAND_GOTO_TARGET_POSITION		0x07
#define HM5065_REG_AF_LENS_COMMAND_GOTO_HYPERFOCAL		0x0C

#define HM5065_REG_AF_MANUAL_STEP_SIZE		CCI_REG8(0x070d)
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE	CCI_REG8(0x0714)
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE_AF	BIT(0)
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE_AE	BIT(1)
#define HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE_AWB	BIT(2)
#define HM5065_REG_AF_FACE_LOCATION_X_START	CCI_REG16(0x0715)
#define HM5065_REG_AF_FACE_LOCATION_X_SIZE	CCI_REG16(0x0717)
#define HM5065_REG_AF_FACE_LOCATION_Y_START	CCI_REG16(0x0719)
#define HM5065_REG_AF_FACE_LOCATION_Y_SIZE	CCI_REG16(0x071b)

#define HM5065_REG_AF_IN_FOCUS			CCI_REG8(0x07ae) /* ro 0,1 */
#define HM5065_REG_AF_IS_STABLE			CCI_REG8(0x0725) /* ro 0,1 */

/* statistics zone weights; the default eye shape uses zones 0-6 */
#define HM5065_REG_AF_ZONE_WEIGHT(n)		CCI_REG8(0x0808 + (n))

/* reverse engineered registers */
#define HM5065_REG_BUS_DATA_FORMAT		CCI_REG8(0x7000)
#define HM5065_REG_COLORSPACE			CCI_REG8(0x5200)
#define HM5065_REG_BUS_CONFIG			CCI_REG8(0x7101)
#define HM5065_REG_BUS_CONFIG_BT656		0x24
#define HM5065_REG_BUS_CONFIG_PARALLEL_HH_VL	0x44

/*
 * Where the autofocus should look. The same private control the ov5640
 * driver has: the value packs the point of interest as x << 16 | y in
 * the frame's own pixels, and zero restores the default zones.
 */
#define V4L2_CID_HM5065_FOCUS_ZONE	(V4L2_CID_USER_BASE | 0x1001)

/* }}} */

struct reg_value {
	u16 addr;
	u8 value;
} __packed;

/*
 * Sensor has various pre-defined PLL configurations for a set of
 * external clock frequencies.
 */
struct hm5065_clk_lut {
	unsigned long clk_freq;
	u8 lut_id;
};

static const struct hm5065_clk_lut hm5065_clk_luts[] = {
	{ .clk_freq = 12000000, .lut_id = 0x10 },
	{ .clk_freq = 13000000, .lut_id = 0x11 },
	{ .clk_freq = 13500000, .lut_id = 0x12 },
	{ .clk_freq = 14400000, .lut_id = 0x13 },
	{ .clk_freq = 18000000, .lut_id = 0x14 },
	{ .clk_freq = 19200000, .lut_id = 0x15 },
	{ .clk_freq = 24000000, .lut_id = 0x16 },
	{ .clk_freq = 26000000, .lut_id = 0x17 },
	{ .clk_freq = 27000000, .lut_id = 0x18 },
};

static const struct hm5065_clk_lut *hm5065_find_clk_lut(unsigned long freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hm5065_clk_luts); i++)
		if (hm5065_clk_luts[i].clk_freq == freq)
			return &hm5065_clk_luts[i];

	return NULL;
}

struct hm5065_pixfmt {
	u32 code;
	u32 colorspace;
	u8 data_fmt;
	u8 ycbcr_order;
	u8 fmt_setup;
};

/*
 * The output pipe is JFIF based - the custom-order YCbCr formats span
 * the full [1:254] range with Rec. 601 encoding, and the RGB formats
 * are full range too.
 */
static const struct hm5065_pixfmt hm5065_formats[] = {
	{
		.code              = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_CB_Y_CR_Y,
		.fmt_setup         = 0x08
	},
	{
		.code              = MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_CR_Y_CB_Y,
		.fmt_setup         = 0x08
	},
	{
		.code              = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CB_Y_CR,
		.fmt_setup         = 0x08
	},
	{
		.code              = MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_YCBCR_CUSTOM,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB,
		.fmt_setup         = 0x08
	},
	{
		.code              = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_RGB_565,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB,
		.fmt_setup         = 0x02
	},
	{
		.code              = MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.colorspace        = V4L2_COLORSPACE_SRGB,
		.data_fmt          = HM5065_REG_DATA_FORMAT_RGB_555,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CR_Y_CB,
		.fmt_setup         = 0x02
	},
	/*
	 * Raw bayer, the sensor's own processing bypassed. The colour
	 * pipeline is what the ISP behind this is being characterised
	 * against, so handing it an unprocessed frame -- carrying a test
	 * pattern injected before that pipeline, so it arrives as a known
	 * mosaic -- is the point of this entry.
	 *
	 * The sensor is put in its ten bit mode, one pixel per clock, but
	 * the code is eight bit because that is what the receiver sees: the
	 * A711 camera module brings out eight data lines, D0 to D7, and a
	 * ten bit sensor on an eight bit bus is wired most significant bits
	 * first. So the CSI gets bits 9:2 and the bottom two are lost, which
	 * is a plain eight bit bayer frame.
	 *
	 * Not the ITU8 packing, which was tried first: it gives two bytes
	 * per pixel, ten bits of data in a sixteen bit container, so a 2592
	 * pixel line occupies 5184 bytes and nothing downstream can take it.
	 * All sixteen values of the byte order register were swept and every
	 * one of them does that -- the register only chooses which byte
	 * comes first. There is no eight bit raw mode on this sensor.
	 *
	 * The phase is a guess until it is measured off the CSI the way the
	 * gc2145 one was, and so is the bus setup value: the vendor never
	 * used raw here, its own raw format table being present and empty.
	 */
	{
		.code              = MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace        = V4L2_COLORSPACE_RAW,
		.data_fmt          = HM5065_REG_DATA_FORMAT_RAW10ITU10,
		.ycbcr_order       = HM5065_REG_YCRCB_ORDER_Y_CB_Y_CR,
		.fmt_setup         = 0x08
	},
};

/*
 * Register 0x7000 selects byte and component order and the vendor never
 * used it for raw, so the right value has to be found by trying them.
 *
 * It cannot be swept from userspace: writing 0x7000 while the sensor is
 * streaming stalls it, and outside streaming the chip is powered down and
 * will not answer. Hence a parameter, so that one build covers the search.
 * Negative means use the value from the format table.
 */
static int raw_bus_format = -1;
module_param(raw_bus_format, int, 0644);
MODULE_PARM_DESC(raw_bus_format,
		 "override register 0x7000 for raw formats (-1: table default)");

static bool hm5065_format_is_raw(const struct hm5065_pixfmt *fmt)
{
	return fmt->data_fmt == HM5065_REG_DATA_FORMAT_RAW10ITU10 ||
	       fmt->data_fmt == HM5065_REG_DATA_FORMAT_RAW10ITU8;
}

#define HM5065_NUM_FORMATS ARRAY_SIZE(hm5065_formats)

static const struct hm5065_pixfmt *hm5065_find_format(u32 code)
{
	int i;

	for (i = 0; i < HM5065_NUM_FORMATS; i++)
		if (hm5065_formats[i].code == code)
			return &hm5065_formats[i];

	return NULL;
}

/* regulator supplies */
static const char * const hm5065_supply_name[] = {
	"IOVDD", /* Digital I/O (2.8V) suppply */
	"AFVDD",  /* Autofocus (2.8V) supply */
	"DVDD",  /* Digital Core (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
};

#define HM5065_NUM_SUPPLIES ARRAY_SIZE(hm5065_supply_name)

struct hm5065_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exposure;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *d_gain;
		struct v4l2_ctrl *a_gain;
	};
	struct v4l2_ctrl *exposure_absolute;
	struct v4l2_ctrl *auto_priority;
	struct v4l2_ctrl *metering;
	struct v4l2_ctrl *exposure_bias;
	struct {
		struct v4l2_ctrl *wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *focus_auto;
		struct v4l2_ctrl *af_start;
		struct v4l2_ctrl *af_stop;
		struct v4l2_ctrl *af_status;
		struct v4l2_ctrl *af_distance;
		struct v4l2_ctrl *focus_relative;
		struct v4l2_ctrl *focus_absolute;
	};
	struct v4l2_ctrl *focus_zone;
	struct v4l2_ctrl *aaa_lock;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *pl_freq;
	struct v4l2_ctrl *colorfx;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *gamma;
	struct v4l2_ctrl *wdr;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *test_data[4];
};

struct hm5065_dev {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* external clock for HM5065 */

	struct regulator_bulk_data supplies[HM5065_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio; // nrst pin
	struct gpio_desc *enable_gpio; // ce pin

	/*
	 * The sub-device state lock, which is the control handler lock,
	 * protects all members below.
	 */
	struct v4l2_fract frame_interval;
	struct hm5065_ctrls ctrls;
};

static inline struct hm5065_dev *to_hm5065_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hm5065_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct hm5065_dev,
			     ctrls.handler)->sd;
}

/* {{{ Register access helpers */

/*
 * The firmware format:
 * <record 0>, ..., <record N - 1>
 * "record" is a 2-byte register address (big endian) followed by 1-byte data
 */
static int hm5065_load_firmware(struct hm5065_dev *sensor, const char *name)
{
	int ret = 0, i = 0, list_size;
	const struct firmware *fw;
	struct reg_value *list;
	u16 start, len;
	u8 buf[128];

	/*
	 * The firmware loads at runtime resume, which can run before the
	 * sub-device is registered with a v4l2_device, so the request goes
	 * against the I2C device.
	 */
	ret = request_firmware(&fw, name, &sensor->i2c_client->dev);
	if (ret) {
		v4l2_warn(&sensor->sd,
			  "Failed to read firmware %s, continuing anyway...\n",
			  name);
		return 1;
	}

	if (fw->size == 0) {
		ret = 1;
		goto err_release;
	}

	if (fw->size % 3 != 0) {
		v4l2_err(&sensor->sd, "Firmware image %s has invalid size\n",
			 name);
		ret = -EINVAL;
		goto err_release;
	}

	list_size = fw->size / 3;
	list = (struct reg_value *)fw->data;

	/* we speed up I2C communication via auto-increment functionality */
	while (i < list_size) {
		start = be16_to_cpu(list[i].addr);
		len = 0;

		while (i < list_size &&
		       be16_to_cpu(list[i].addr) == (start + len) &&
		       len < sizeof(buf))
			buf[len++] = list[i++].value;

		ret = regmap_bulk_write(sensor->regmap, start, buf, len);
		if (ret)
			goto err_release;
	}

err_release:
	release_firmware(fw);
	return ret;
}

/*
 * Sensor uses ST Float900 format to represent floating point numbers.
 * Binary floating point number: * (s ? -1 : 0) * 1.mmmmmmmmm * 2^eeeeee
 *
 * Following functions convert long value to and from the floating point format.
 *
 * Example:
 * mili variant: val = 123456 => fp_val = 123.456
 * micro variant: val = -12345678 => fp_val = -12.345678
 */
static s64 hm5065_mili_from_fp16(u16 fp_val)
{
	s64 val;
	s64 mantisa = fp_val & 0x1ff;
	int exp = (int)((fp_val >> 9) & 0x3f) - 31;

	val = (1000 * (mantisa | 0x200));
	if (exp > 0)
		val <<= exp;
	else if (exp < 0)
		val >>= -exp;
	val >>= 9;

	if (fp_val & 0x8000)
		val = -val;

	return val;
}

static u16 hm5065_mili_to_fp16(s32 val)
{
	int fls;
	u16 e, m, s = 0;
	u64 v, rem;

	if (val == 0)
		return 0;

	if (val < 0) {
		val = -val;
		s = 0x8000;
	}

	v = (u64)val * 1024;
	rem = do_div(v, 1000);
	if (rem >= 500)
		v++;

	fls = fls64(v) - 1;
	e = 31 + fls - 10;
	m = fls > 9 ? v >> (fls - 9) : v << (9 - fls);

	return s | (m & 0x1ff) | (e << 9);
}

/* }}} */
/* {{{ Controls */

static int hm5065_get_af_status(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	u64 in_focus, is_stable, mode, position;
	int ret = 0;

	cci_read(sensor->regmap, HM5065_REG_AF_MODE_STATUS, &mode, &ret);
	cci_read(sensor->regmap, HM5065_REG_AF_LENS_POSITION, &position, &ret);
	if (ret)
		return ret;

	ctrls->focus_absolute->val = position;

	if (mode == HM5065_REG_AF_MODE_MANUAL) {
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_IDLE;
		return 0;
	}

	/*
	 * The stability flag only signals that the lens stopped moving;
	 * whether a focused position was actually found is the algorithm's
	 * verdict in the in-focus flag.
	 */
	cci_read(sensor->regmap, HM5065_REG_AF_IN_FOCUS, &in_focus, &ret);
	cci_read(sensor->regmap, HM5065_REG_AF_IS_STABLE, &is_stable, &ret);
	if (ret)
		return ret;

	if (in_focus)
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_REACHED;
	else if (!is_stable || mode == HM5065_REG_AF_MODE_CONTINUOUS)
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_BUSY;
	else
		ctrls->af_status->val = V4L2_AUTO_FOCUS_STATUS_FAILED;

	return 0;
}

static int hm5065_get_exposure(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	u64 again, dgain, exp;
	int ret = 0;

	cci_read(sensor->regmap, HM5065_REG_CODED_ANALOG_GAIN_PENDING,
		 &again, &ret);
	cci_read(sensor->regmap, HM5065_REG_DIGITAL_GAIN_PENDING, &dgain,
		 &ret);
	cci_read(sensor->regmap, HM5065_REG_COARSE_INTEGRATION, &exp, &ret);
	if (ret)
		return ret;

	ctrls->exposure->val = exp;
	ctrls->d_gain->val = clamp(hm5065_mili_from_fp16(dgain), 1000ll,
				   4000ll);
	ctrls->a_gain->val = again;

	return 0;
}

static int hm5065_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	/* v4l2_ctrl_lock() locks the shared state lock */

	if (!pm_runtime_get_if_in_use(dev))
		return -EIO;

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_AUTO:
		ret = hm5065_get_af_status(sensor);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = hm5065_get_exposure(sensor);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(dev);
	return ret;
}

static const u8 hm5065_wb_opts[][2] = {
	{ V4L2_WHITE_BALANCE_MANUAL, HM5065_REG_WB_MODE_OFF },
	{ V4L2_WHITE_BALANCE_INCANDESCENT, HM5065_REG_WB_MODE_TUNGSTEN_PRESET },
	{ V4L2_WHITE_BALANCE_FLUORESCENT,
		HM5065_REG_WB_MODE_FLUORESCENT_PRESET },
	{ V4L2_WHITE_BALANCE_HORIZON, HM5065_REG_WB_MODE_HORIZON_PRESET },
	{ V4L2_WHITE_BALANCE_CLOUDY, HM5065_REG_WB_MODE_CLOUDY_PRESET },
	{ V4L2_WHITE_BALANCE_DAYLIGHT, HM5065_REG_WB_MODE_SUNNY_PRESET },
	{ V4L2_WHITE_BALANCE_AUTO, HM5065_REG_WB_MODE_AUTOMATIC },
};

static int hm5065_set_power_line_frequency(struct hm5065_dev *sensor, s32 val)
{
	struct regmap *map = sensor->regmap;
	int ret = 0;
	u16 freq;

	switch (val) {
	case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
		cci_write(map, HM5065_REG_ANTI_FLICKER_MODE, 0, &ret);
		cci_write(map, HM5065_REG_FD_ENABLE_DETECT, 0, &ret);
		return ret;
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		freq = (val == V4L2_CID_POWER_LINE_FREQUENCY_50HZ) ?
			0x4b20 : 0x4bc0;

		cci_write(map, HM5065_REG_ANTI_FLICKER_MODE, 1, &ret);
		cci_write(map, HM5065_REG_FD_ENABLE_DETECT, 0, &ret);
		cci_write(map, HM5065_REG_FD_FLICKER_FREQUENCY, freq, &ret);
		return ret;
	case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		cci_write(map, HM5065_REG_FD_ENABLE_DETECT, 1, &ret);
		cci_write(map, HM5065_REG_ANTI_FLICKER_MODE, 1, &ret);
		cci_write(map, HM5065_REG_FD_MAX_NUMBER_ATTEMP, 100, &ret);
		cci_write(map, HM5065_REG_FD_FLICKER_FREQUENCY, 0, &ret);
		cci_write(map, HM5065_REG_FD_DETECTION_START, 1, &ret);
		return ret;
	default:
		return -EINVAL;
	}
}

static int hm5065_set_colorfx(struct hm5065_dev *sensor, s32 val)
{
	struct regmap *map = sensor->regmap;
	int ret = 0;

	cci_write(map, HM5065_REG_EFFECTS_COLOR,
		  HM5065_REG_EFFECTS_COLOR_NORMAL, &ret);
	cci_write(map, HM5065_REG_EFFECTS_NEGATIVE, 0, &ret);
	cci_write(map, HM5065_REG_EFFECTS_SOLARISING, 0, &ret);
	cci_write(map, HM5065_REG_EFFECTS_SKECTH, 0, &ret);

	switch (val) {
	case V4L2_COLORFX_NONE:
		return ret;
	case V4L2_COLORFX_NEGATIVE:
		return cci_write(map, HM5065_REG_EFFECTS_NEGATIVE, 1, &ret);
	case V4L2_COLORFX_SOLARIZATION:
		return cci_write(map, HM5065_REG_EFFECTS_SOLARISING, 1, &ret);
	case V4L2_COLORFX_SKETCH:
		return cci_write(map, HM5065_REG_EFFECTS_SKECTH, 1, &ret);
	case V4L2_COLORFX_ANTIQUE:
		return cci_write(map, HM5065_REG_EFFECTS_COLOR,
				 HM5065_REG_EFFECTS_COLOR_ANTIQUE, &ret);
	case V4L2_COLORFX_SEPIA:
		return cci_write(map, HM5065_REG_EFFECTS_COLOR,
				 HM5065_REG_EFFECTS_COLOR_SEPIA, &ret);
	case V4L2_COLORFX_AQUA:
		return cci_write(map, HM5065_REG_EFFECTS_COLOR,
				 HM5065_REG_EFFECTS_COLOR_AQUA, &ret);
	case V4L2_COLORFX_BW:
		return cci_write(map, HM5065_REG_EFFECTS_COLOR,
				 HM5065_REG_EFFECTS_COLOR_BLACK_WHITE, &ret);
	default:
		return -EINVAL;
	}
}

#define AE_BIAS_MENU_DEFAULT_VALUE_INDEX 7
static const s64 ae_bias_menu_values[] = {
	-2100, -1800, -1500, -1200, -900, -600, -300,
	0, 300, 600, 900, 1200, 1500, 1800, 2100
};

static const s8 ae_bias_menu_reg_values[] = {
	-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7
};

static int hm5065_set_exposure(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	bool is_auto = (ctrls->auto_exposure->val != V4L2_EXPOSURE_MANUAL);
	int ret = 0;

	if (ctrls->auto_exposure->is_new) {
		u8 mode;

		switch (ctrls->auto_exposure->val) {
		case V4L2_EXPOSURE_AUTO:
			mode = HM5065_REG_EXPOSURE_MODE_AUTO;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			/* manual exposure time, the MCU compiles the rest */
			mode = HM5065_REG_EXPOSURE_MODE_COMPILED_MANUAL;
			break;
		default:
			mode = HM5065_REG_EXPOSURE_MODE_DIRECT_MANUAL;
			break;
		}

		ret = cci_write(sensor->regmap, HM5065_REG_EXPOSURE_MODE, mode,
				NULL);
		if (ret)
			return ret;

		if (ctrls->auto_exposure->cur.val != ctrls->auto_exposure->val &&
		    !is_auto) {
			/*
			 * Hack: At this point, there are current volatile
			 * values in val, but control framework will not
			 * update the cur values for our autocluster, as it
			 * should. I couldn't find the reason. This fixes
			 * it for our driver. Remove this after the kernel
			 * is fixed.
			 */
			ctrls->exposure->cur.val = ctrls->exposure->val;
			ctrls->d_gain->cur.val = ctrls->d_gain->val;
			ctrls->a_gain->cur.val = ctrls->a_gain->val;
		}
	}

	if (!is_auto && ctrls->exposure->is_new)
		cci_write(sensor->regmap,
			  HM5065_REG_DIRECT_MODE_COARSE_INTEGRATION_LINES,
			  ctrls->exposure->val, &ret);

	if (!is_auto && ctrls->d_gain->is_new)
		cci_write(sensor->regmap,
			  HM5065_REG_DIRECT_MODE_DIGITAL_GAIN,
			  hm5065_mili_to_fp16(ctrls->d_gain->val), &ret);

	if (!is_auto && ctrls->a_gain->is_new)
		cci_write(sensor->regmap,
			  HM5065_REG_DIRECT_MODE_CODED_ANALOG_GAIN,
			  ctrls->a_gain->val, &ret);

	return ret;
}

/*
 * With the automatic frame rate control the AE algorithm may drop the
 * frame rate below the desired rate to extend the integration time in
 * low light, down to a third of it.
 */
static int hm5065_set_afr(struct hm5065_dev *sensor, bool enable)
{
	int fps = sensor->frame_interval.denominator;
	int ret = 0;

	cci_write(sensor->regmap, HM5065_REG_AFR_ENABLE, enable ? 1 : 0, &ret);
	if (!enable)
		return ret;

	cci_write(sensor->regmap, HM5065_REG_AFR_MIN_FPS, max(fps / 3, 1),
		  &ret);
	cci_write(sensor->regmap, HM5065_REG_AFR_MAX_FPS, fps, &ret);

	return ret;
}

static int hm5065_3a_lock(struct hm5065_dev *sensor, struct v4l2_ctrl *ctrl)
{
	bool awb_lock = ctrl->val & V4L2_LOCK_WHITE_BALANCE;
	bool ae_lock = ctrl->val & V4L2_LOCK_EXPOSURE;
	bool af_lock = ctrl->val & V4L2_LOCK_FOCUS;
	int ret = 0;

	if ((ctrl->val ^ ctrl->cur.val) & V4L2_LOCK_EXPOSURE
	    && sensor->ctrls.auto_exposure->val == V4L2_EXPOSURE_AUTO)
		cci_write(sensor->regmap, HM5065_REG_FREEZE_AUTO_EXPOSURE,
			  ae_lock, &ret);

	if (((ctrl->val ^ ctrl->cur.val) & V4L2_LOCK_WHITE_BALANCE)
	    && sensor->ctrls.wb->val == V4L2_WHITE_BALANCE_AUTO)
		cci_write(sensor->regmap, HM5065_REG_WB_MISC_SETTINGS,
			  awb_lock ?
			  HM5065_REG_WB_MISC_SETTINGS_FREEZE_ALGO : 0, &ret);

	/*
	 * Parking the algorithm in the manual mode holds the lens at its
	 * current position; no lens command is sent, so nothing moves.
	 */
	if (((ctrl->val ^ ctrl->cur.val) & V4L2_LOCK_FOCUS)
	    && sensor->ctrls.focus_auto->val)
		cci_write(sensor->regmap, HM5065_REG_AF_MODE,
			  af_lock ? HM5065_REG_AF_MODE_MANUAL :
			  HM5065_REG_AF_MODE_CONTINUOUS, &ret);

	return ret;
}

static int hm5065_set_auto_focus(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	bool auto_focus = ctrls->focus_auto->val;
	struct regmap *map = sensor->regmap;
	int ret = 0;
	u8 range;

	if (auto_focus && ctrls->af_distance->is_new) {
		switch (ctrls->af_distance->val) {
		case V4L2_AUTO_FOCUS_RANGE_MACRO:
			range = HM5065_REG_AF_RANGE_MACRO;
			break;
		case V4L2_AUTO_FOCUS_RANGE_AUTO:
			range = HM5065_REG_AF_RANGE_FULL;
			break;
		case V4L2_AUTO_FOCUS_RANGE_INFINITY:
			range = HM5065_REG_AF_RANGE_LANDSCAPE;
			break;
		default:
			return -EINVAL;
		}

		cci_write(map, HM5065_REG_AF_RANGE, range, &ret);
	}

	if (ctrls->focus_auto->is_new) {
		v4l2_ctrl_activate(ctrls->af_start, !auto_focus);
		v4l2_ctrl_activate(ctrls->af_stop, !auto_focus);
		v4l2_ctrl_activate(ctrls->focus_relative, !auto_focus);
		v4l2_ctrl_activate(ctrls->focus_absolute, !auto_focus);

		cci_write(map, HM5065_REG_AF_MODE,
			  auto_focus ?
			  HM5065_REG_AF_MODE_CONTINUOUS :
			  HM5065_REG_AF_MODE_SINGLE, &ret);

		if (!auto_focus)
			cci_write(map, HM5065_REG_AF_COMMAND,
				  HM5065_REG_AF_COMMAND_RELEASED_BUTTON, &ret);
	}

	if (!auto_focus && ctrls->af_start->is_new) {
		cci_write(map, HM5065_REG_AF_MODE,
			  HM5065_REG_AF_MODE_SINGLE, &ret);
		cci_write(map, HM5065_REG_AF_COMMAND,
			  HM5065_REG_AF_COMMAND_RELEASED_BUTTON, &ret);
		if (ret)
			return ret;

		usleep_range(190000, 200000);

		cci_write(map, HM5065_REG_AF_COMMAND,
			  HM5065_REG_AF_COMMAND_HALF_BUTTON, &ret);
	}

	if (!auto_focus && ctrls->af_stop->is_new) {
		cci_write(map, HM5065_REG_AF_COMMAND,
			  HM5065_REG_AF_COMMAND_RELEASED_BUTTON, &ret);
		cci_write(map, HM5065_REG_AF_MODE,
			  HM5065_REG_AF_MODE_MANUAL, &ret);
	}

	if (!auto_focus && ctrls->focus_absolute->is_new) {
		cci_write(map, HM5065_REG_AF_MODE,
			  HM5065_REG_AF_MODE_MANUAL, &ret);
		cci_write(map, HM5065_REG_AF_TARGET_POSITION,
			  ctrls->focus_absolute->val, &ret);
		cci_write(map, HM5065_REG_AF_LENS_COMMAND,
			  HM5065_REG_AF_LENS_COMMAND_GOTO_TARGET_POSITION,
			  &ret);
	}

	if (!auto_focus && ctrls->focus_relative->is_new &&
	    ctrls->focus_relative->val) {
		s32 step = ctrls->focus_relative->val;

		ctrls->focus_relative->val = 0;

		cci_write(map, HM5065_REG_AF_MODE,
			  HM5065_REG_AF_MODE_MANUAL, &ret);
		cci_write(map, HM5065_REG_AF_MANUAL_STEP_SIZE, abs(step),
			  &ret);
		cci_write(map, HM5065_REG_AF_LENS_COMMAND,
			  step < 0 ?
			  HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_INFINITY :
			  HM5065_REG_AF_LENS_COMMAND_MOVE_STEP_TO_MACRO, &ret);
	}

	return ret;
}

/*
 * Point the autofocus at one place in the frame. The face location
 * window feeds the statistics engine instead of the default zones, and
 * zone 0 carries all the weight. Zero restores the default eye shaped
 * seven zone setup.
 */
static int hm5065_set_focus_zone(struct hm5065_dev *sensor, u32 packed)
{
	const struct v4l2_mbus_framefmt *fmt;
	struct regmap *map = sensor->regmap;
	struct v4l2_subdev_state *state;
	u32 x, y, x0, y0, w, h;
	int ret = 0, i;

	if (!packed) {
		for (i = 0; i < 7; i++)
			cci_write(map, HM5065_REG_AF_ZONE_WEIGHT(i), 1, &ret);
		cci_write(map, HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE, 0,
			  &ret);
		return ret;
	}

	/* the handler lock is the state lock, so the state is ours already */
	state = v4l2_subdev_get_locked_active_state(&sensor->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	/* a window sized like one default zone, centred on the point */
	w = fmt->width / 6;
	h = fmt->height / 9;
	x = min_t(u32, packed >> 16, fmt->width - 1);
	y = min_t(u32, packed & 0xffff, fmt->height - 1);
	x0 = x > w / 2 ? x - w / 2 : 0;
	if (x0 + w > fmt->width)
		x0 = fmt->width - w;
	y0 = y > h / 2 ? y - h / 2 : 0;
	if (y0 + h > fmt->height)
		y0 = fmt->height - h;

	for (i = 0; i < 7; i++)
		cci_write(map, HM5065_REG_AF_ZONE_WEIGHT(i), i == 0 ? 1 : 0,
			  &ret);
	cci_write(map, HM5065_REG_AF_FACE_LOCATION_X_START, x0, &ret);
	cci_write(map, HM5065_REG_AF_FACE_LOCATION_X_SIZE, w, &ret);
	cci_write(map, HM5065_REG_AF_FACE_LOCATION_Y_START, y0, &ret);
	cci_write(map, HM5065_REG_AF_FACE_LOCATION_Y_SIZE, h, &ret);
	cci_write(map, HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE,
		  HM5065_REG_AF_FACE_LOCATION_CTRL_ENABLE_AF, &ret);

	return ret;
}

static int hm5065_set_white_balance(struct hm5065_dev *sensor)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	bool manual_wb = ctrls->wb->val == V4L2_WHITE_BALANCE_MANUAL;
	int ret = 0, i;
	s32 val;

	if (ctrls->wb->is_new) {
		for (i = 0; i < ARRAY_SIZE(hm5065_wb_opts); i++) {
			if (hm5065_wb_opts[i][0] != ctrls->wb->val)
				continue;

			cci_write(sensor->regmap, HM5065_REG_WB_MODE,
				  hm5065_wb_opts[i][1], &ret);
			goto next;
		}

		return -EINVAL;
	}

next:
	if (ctrls->wb->is_new || ctrls->blue_balance->is_new) {
		val = manual_wb ? ctrls->blue_balance->val : 1000;
		cci_write(sensor->regmap, HM5065_REG_WB_HUE_B_BIAS,
			  hm5065_mili_to_fp16(val), &ret);
	}

	if (ctrls->wb->is_new || ctrls->red_balance->is_new) {
		val = manual_wb ? ctrls->red_balance->val : 1000;
		cci_write(sensor->regmap, HM5065_REG_WB_HUE_R_BIAS,
			  hm5065_mili_to_fp16(val), &ret);
	}

	return ret;
}

static int hm5065_s_ctrl_do(struct hm5065_dev *sensor,
			    struct v4l2_ctrl *ctrl)
{
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	s32 val = ctrl->val;
	unsigned int i;
	int ret;
	u8 reg;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
		return hm5065_set_exposure(sensor);

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/* the control unit is 100us, the register wants microseconds */
		return cci_write(sensor->regmap,
				 HM5065_REG_MANUAL_EXPOSURE_TIME_US,
				 hm5065_mili_to_fp16(val * 100000), NULL);

	case V4L2_CID_EXPOSURE_AUTO_PRIORITY:
		return hm5065_set_afr(sensor, val);

	case V4L2_CID_EXPOSURE_METERING:
		if (val == V4L2_EXPOSURE_METERING_AVERAGE)
			reg = HM5065_REG_EXPOSURE_METERING_FLAT;
		else if (val == V4L2_EXPOSURE_METERING_CENTER_WEIGHTED)
			reg = HM5065_REG_EXPOSURE_METERING_CENTERED;
		else
			return -EINVAL;

		return cci_write(sensor->regmap, HM5065_REG_EXPOSURE_METERING,
				 reg, NULL);

	case V4L2_CID_AUTO_EXPOSURE_BIAS:
		if (val < 0 || val >= ARRAY_SIZE(ae_bias_menu_reg_values))
			return -EINVAL;

		return cci_write(sensor->regmap,
				 HM5065_REG_EXPOSURE_COMPENSATION,
				 (u8)ae_bias_menu_reg_values[val], NULL);

	case V4L2_CID_FOCUS_AUTO:
		return hm5065_set_auto_focus(sensor);

	case V4L2_CID_HM5065_FOCUS_ZONE:
		return hm5065_set_focus_zone(sensor, val);

	case V4L2_CID_CONTRAST:
		return cci_write(sensor->regmap, HM5065_REG_CONTRAST, val,
				 NULL);

	case V4L2_CID_SATURATION:
		return cci_write(sensor->regmap, HM5065_REG_COLOR_SATURATION,
				 val, NULL);

	case V4L2_CID_BRIGHTNESS:
		return cci_write(sensor->regmap, HM5065_REG_BRIGHTNESS, val,
				 NULL);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return hm5065_set_power_line_frequency(sensor, val);

	case V4L2_CID_GAMMA:
		return cci_write(sensor->regmap, HM5065_REG_P0_GAMMA_GAIN, val,
				 NULL);

	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		return cci_write(sensor->regmap, HM5065_REG_CS_ENABLE,
				 val ? 1 : 0, NULL);

	case V4L2_CID_VFLIP:
		return cci_write(sensor->regmap, HM5065_REG_VERTICAL_FLIP,
				 val ? 1 : 0, NULL);

	case V4L2_CID_HFLIP:
		return cci_write(sensor->regmap, HM5065_REG_HORIZONTAL_MIRROR,
				 val ? 1 : 0, NULL);

	case V4L2_CID_COLORFX:
		return hm5065_set_colorfx(sensor, val);

	case V4L2_CID_3A_LOCK:
		return hm5065_3a_lock(sensor, ctrl);

	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		return hm5065_set_white_balance(sensor);

	case V4L2_CID_TEST_PATTERN_RED:
		return cci_write(sensor->regmap, HM5065_REG_TESTDATA_RED, val,
				 NULL);

	case V4L2_CID_TEST_PATTERN_GREENR:
		return cci_write(sensor->regmap, HM5065_REG_TESTDATA_GREEN_R,
				 val, NULL);

	case V4L2_CID_TEST_PATTERN_BLUE:
		return cci_write(sensor->regmap, HM5065_REG_TESTDATA_BLUE, val,
				 NULL);

	case V4L2_CID_TEST_PATTERN_GREENB:
		return cci_write(sensor->regmap, HM5065_REG_TESTDATA_GREEN_B,
				 val, NULL);

	case V4L2_CID_TEST_PATTERN:
		for (i = 0; i < ARRAY_SIZE(ctrls->test_data); i++)
			v4l2_ctrl_activate(ctrls->test_data[i],
					   val == 6); /* solid color */

		ret = 0;
		cci_write(sensor->regmap, HM5065_REG_ENABLE_TEST_PATTERN,
			  val == 0 ? 0 : 1, &ret);
		cci_write(sensor->regmap, HM5065_REG_TEST_PATTERN, val, &ret);
		return ret;

	default:
		return -EINVAL;
	}
}

static int hm5065_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	/* v4l2_ctrl_lock() locks the shared state lock */

	/*
	 * While the sensor is powered down the values are simply kept in
	 * the control handler, and the handler setup done when streaming
	 * starts applies them.
	 */
	if (!pm_runtime_get_if_in_use(dev))
		return 0;

	ret = hm5065_s_ctrl_do(sensor, ctrl);

	pm_runtime_put_autosuspend(dev);
	return ret;
}

static const struct v4l2_ctrl_ops hm5065_ctrl_ops = {
	.g_volatile_ctrl = hm5065_g_volatile_ctrl,
	.s_ctrl = hm5065_s_ctrl,
};

static const struct v4l2_ctrl_config hm5065_ctrl_focus_zone = {
	.ops = &hm5065_ctrl_ops,
	.id = V4L2_CID_HM5065_FOCUS_ZONE,
	.name = "Focus Zone",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 0x7fffffff,
	.step = 1,
	.def = 0,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"Horizontal gray scale",
	"Vertical gray scale",
	"Diagonal gray scale",
	"PN28",
	"PN9 (bus test)",
	"Solid color",
	"Color bars",
	"Graduated color bars",
};

static int hm5065_init_controls(struct hm5065_dev *sensor)
{
	const struct v4l2_ctrl_ops *ops = &hm5065_ctrl_ops;
	struct hm5065_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct v4l2_fwnode_device_properties props;
	u8 wb_max = 0;
	u64 wb_mask = 0;
	unsigned int i;
	int ret;

	v4l2_ctrl_handler_init(hdl, 32);

	ctrls->auto_exposure =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_AUTO,
				       V4L2_EXPOSURE_SHUTTER_PRIORITY, 0,
				       V4L2_EXPOSURE_AUTO);
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops,
					    V4L2_CID_EXPOSURE,
					    1, HM5065_SENSOR_HEIGHT, 1, 30);
	/* in 100us units, applied in the shutter priority mode */
	ctrls->exposure_absolute =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE_ABSOLUTE,
				  1, 10000, 1, 100);
	ctrls->auto_priority =
		v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE_AUTO_PRIORITY,
				  0, 1, 1, 0);
	ctrls->d_gain = v4l2_ctrl_new_std(hdl, ops,
					  V4L2_CID_DIGITAL_GAIN,
					  1000, 4000, 1, 1000);

	/*
	 * The gain coding is non-linear: each 0x10 step adds 0.56-2.5 dB
	 * up to 0xe0, continuing in steps of 4 up to 0xf0 = 24.1 dB (x16).
	 */
	ctrls->a_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					  0, 0xf0, 1, 0);

	ctrls->metering =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_EXPOSURE_METERING,
				       V4L2_EXPOSURE_METERING_CENTER_WEIGHTED,
				       0, V4L2_EXPOSURE_METERING_AVERAGE);
	ctrls->exposure_bias =
		v4l2_ctrl_new_int_menu(hdl, ops,
				       V4L2_CID_AUTO_EXPOSURE_BIAS,
				       ARRAY_SIZE(ae_bias_menu_values) - 1,
				       AE_BIAS_MENU_DEFAULT_VALUE_INDEX,
				       ae_bias_menu_values);

	for (i = 0; i < ARRAY_SIZE(hm5065_wb_opts); i++) {
		if (wb_max < hm5065_wb_opts[i][0])
			wb_max = hm5065_wb_opts[i][0];
		wb_mask |= BIT(hm5065_wb_opts[i][0]);
	}

	ctrls->wb = v4l2_ctrl_new_std_menu(hdl, ops,
			V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
			wb_max, ~wb_mask, V4L2_WHITE_BALANCE_AUTO);

	ctrls->blue_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
						0, 4000, 1, 1000);
	ctrls->red_balance = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					       0, 4000, 1, 1000);

	ctrls->gamma = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAMMA,
					 0, 31, 1, 20);

	/* the contrast stretch block */
	ctrls->wdr = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_WIDE_DYNAMIC_RANGE,
				       0, 1, 1, 0);

	ctrls->colorfx =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_COLORFX, 15,
				       ~(BIT(V4L2_COLORFX_NONE) |
					 BIT(V4L2_COLORFX_NEGATIVE) |
					 BIT(V4L2_COLORFX_SOLARIZATION) |
					 BIT(V4L2_COLORFX_SKETCH) |
					 BIT(V4L2_COLORFX_SEPIA) |
					 BIT(V4L2_COLORFX_ANTIQUE) |
					 BIT(V4L2_COLORFX_AQUA) |
					 BIT(V4L2_COLORFX_BW)),
				       V4L2_COLORFX_NONE);

	ctrls->pl_freq =
		v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_POWER_LINE_FREQUENCY,
				V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
				V4L2_CID_POWER_LINE_FREQUENCY_50HZ);

	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops,
					 V4L2_CID_VFLIP, 0, 1, 1, 0);

	ctrls->focus_auto = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_AUTO,
					      0, 1, 1, 1);

	ctrls->af_start = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTO_FOCUS_START,
					    0, 1, 1, 0);

	ctrls->af_stop = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTO_FOCUS_STOP,
					   0, 1, 1, 0);

	ctrls->af_status = v4l2_ctrl_new_std(hdl, ops,
					     V4L2_CID_AUTO_FOCUS_STATUS, 0,
					     (V4L2_AUTO_FOCUS_STATUS_BUSY |
					      V4L2_AUTO_FOCUS_STATUS_REACHED |
					      V4L2_AUTO_FOCUS_STATUS_FAILED),
					     0, V4L2_AUTO_FOCUS_STATUS_IDLE);

	ctrls->af_distance =
		v4l2_ctrl_new_std_menu(hdl, ops,
				       V4L2_CID_AUTO_FOCUS_RANGE,
				       V4L2_AUTO_FOCUS_RANGE_MACRO,
				       ~(BIT(V4L2_AUTO_FOCUS_RANGE_AUTO) |
					 BIT(V4L2_AUTO_FOCUS_RANGE_INFINITY) |
					 BIT(V4L2_AUTO_FOCUS_RANGE_MACRO)),
				       V4L2_AUTO_FOCUS_RANGE_AUTO);

	ctrls->focus_relative = v4l2_ctrl_new_std(hdl, ops,
						  V4L2_CID_FOCUS_RELATIVE,
						  -100, 100, 1, 0);

	/* the VCM DAC code; the default is the firmware's recovery position */
	ctrls->focus_absolute = v4l2_ctrl_new_std(hdl, ops,
						  V4L2_CID_FOCUS_ABSOLUTE,
						  0, 1023, 1, 508);

	ctrls->focus_zone = v4l2_ctrl_new_custom(hdl, &hm5065_ctrl_focus_zone,
						 NULL);

	ctrls->brightness = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BRIGHTNESS,
					      0, 200, 1, 90);
	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 200, 1, 110);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 200, 1, 108);

	ctrls->aaa_lock = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_3A_LOCK,
					    0, 0x7, 0, 0);

	ctrls->test_pattern =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);
	for (i = 0; i < ARRAY_SIZE(ctrls->test_data); i++)
		ctrls->test_data[i] =
			v4l2_ctrl_new_std(hdl, ops,
					  V4L2_CID_TEST_PATTERN_RED + i,
					  0, 1023, 1, 0);

	ret = v4l2_fwnode_device_parse(&sensor->i2c_client->dev, &props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);
	if (ret)
		goto free_ctrls;

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->af_status->flags |= V4L2_CTRL_FLAG_VOLATILE |
		V4L2_CTRL_FLAG_READ_ONLY;
	/* reads report the live lens position */
	ctrls->focus_absolute->flags |= V4L2_CTRL_FLAG_VOLATILE |
		V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;

	v4l2_ctrl_auto_cluster(3, &ctrls->wb, V4L2_WHITE_BALANCE_MANUAL, false);
	v4l2_ctrl_auto_cluster(4, &ctrls->auto_exposure, V4L2_EXPOSURE_MANUAL,
			       true);
	v4l2_ctrl_cluster(7, &ctrls->focus_auto);

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

/* }}} */
/* {{{ Video ops */

static int hm5065_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);

	if (fi->pad != 0)
		return -EINVAL;

	fi->interval = sensor->frame_interval;

	return 0;
}

static int hm5065_get_max_binning(int width, int height)
{
	if (width < HM5065_SENSOR_WIDTH / 4 &&
	    height < HM5065_SENSOR_HEIGHT / 4)
		return 4;
	else if (width < HM5065_SENSOR_WIDTH / 2 &&
		 height < HM5065_SENSOR_HEIGHT / 2)
		return 2;

	return 1;
}

static int hm5065_get_max_fps(int width, int height)
{
	int max_fps, bin_factor;

	// more bining allows for faster readouts
	bin_factor = hm5065_get_max_binning(width, height);
	max_fps = 25000000 / (width * height * 2) * bin_factor;

	return clamp(max_fps, 1, 60);
}

static int hm5065_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(sd_state, 0);
	int ret = 0, fps, max_fps;

	if (fi->pad != 0)
		return -EINVAL;

	max_fps = hm5065_get_max_fps(fmt->width, fmt->height);

	/* user requested infinite frame rate */
	if (fi->interval.numerator == 0)
		fps = max_fps;
	else
		fps = DIV_ROUND_CLOSEST(fi->interval.denominator,
					fi->interval.numerator);

	fps = clamp(fps, 1, max_fps);

	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = fps;
	fi->interval = sensor->frame_interval;

	if (sd->enabled_pads) {
		cci_write(sensor->regmap, HM5065_REG_DESIRED_FRAME_RATE_NUM,
			  fps, &ret);
		cci_write(sensor->regmap, HM5065_REG_DESIRED_FRAME_RATE_DEN,
			  1, &ret);
		if (!ret && sensor->ctrls.auto_priority->cur.val)
			ret = hm5065_set_afr(sensor, true);
	}

	return ret;
}

static int hm5065_setup_mode(struct hm5065_dev *sensor,
			     struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	const struct hm5065_pixfmt *pix_fmt;
	struct regmap *map = sensor->regmap;
	u8 sensor_mode;
	int ret = 0, fps;

	pix_fmt = hm5065_find_format(fmt->code);
	if (!pix_fmt) {
		dev_err(&sensor->i2c_client->dev,
			"pixel format not supported %u\n", fmt->code);
		return -EINVAL;
	}

	switch (hm5065_get_max_binning(fmt->width, fmt->height)) {
	case 4:
		sensor_mode = HM5065_REG_SENSOR_MODE_BINNING_4X4;
		break;
	case 2:
		sensor_mode = HM5065_REG_SENSOR_MODE_BINNING_2X2;
		break;
	default:
		sensor_mode = HM5065_REG_SENSOR_MODE_FULLSIZE;
	}

	fps = hm5065_get_max_fps(fmt->width, fmt->height);
	fps = clamp(fps, 1, (int)sensor->frame_interval.denominator);

	cci_write(map, HM5065_REG_USER_COMMAND,
		  HM5065_REG_USER_COMMAND_POWEROFF, &ret);
	cci_write(map, HM5065_REG_P0_SENSOR_MODE, sensor_mode, &ret);
	cci_write(map, HM5065_REG_P0_MANUAL_HSIZE, fmt->width, &ret);
	cci_write(map, HM5065_REG_P0_MANUAL_VSIZE, fmt->height, &ret);
	cci_write(map, HM5065_REG_P0_IMAGE_SIZE,
		  HM5065_REG_IMAGE_SIZE_MANUAL, &ret);
	cci_write(map, HM5065_REG_P0_DATA_FORMAT, pix_fmt->data_fmt, &ret);
	cci_write(map, HM5065_REG_YCRCB_ORDER, pix_fmt->ycbcr_order, &ret);
	/* without this, brightness, contrast and saturation will not work */
	cci_write(map, HM5065_REG_COLORSPACE, 9, &ret);
	cci_write(map, HM5065_REG_BUS_DATA_FORMAT,
		  hm5065_format_is_raw(pix_fmt) && raw_bus_format >= 0 ?
		  raw_bus_format : pix_fmt->fmt_setup, &ret);
	cci_write(map, HM5065_REG_DESIRED_FRAME_RATE_NUM, fps, &ret);
	cci_write(map, HM5065_REG_DESIRED_FRAME_RATE_DEN, 1, &ret);

	return ret;
}

static int hm5065_set_stream(struct hm5065_dev *sensor, int enable)
{
	return cci_write(sensor->regmap, HM5065_REG_USER_COMMAND, enable ?
			 HM5065_REG_USER_COMMAND_RUN :
			 HM5065_REG_USER_COMMAND_STOP, NULL);
}

/*
 * The firmware's timing generator publishes the frame rate range it can
 * actually achieve for the current mode. Check it once streaming starts
 * and pull the desired rate down when the mode setup asked for too much.
 */
static void hm5065_check_frame_rate(struct hm5065_dev *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	s64 max_mili, min_mili, max_hz, min_hz;
	s32 max_rem, min_rem;
	u64 max_fp, min_fp;
	int ret = 0, fps;

	cci_read(sensor->regmap, HM5065_REG_MAX_FRAME_RATE_HZ, &max_fp, &ret);
	cci_read(sensor->regmap, HM5065_REG_MIN_FRAME_RATE_HZ, &min_fp, &ret);
	if (ret || max_fp == 0)
		return;

	max_mili = hm5065_mili_from_fp16(max_fp);
	min_mili = hm5065_mili_from_fp16(min_fp);

	max_hz = div_s64_rem(max_mili, 1000, &max_rem);
	min_hz = div_s64_rem(min_mili, 1000, &min_rem);

	dev_dbg(dev, "firmware frame rate range %lld.%03d-%lld.%03d Hz\n",
		min_hz, min_rem, max_hz, max_rem);

	fps = sensor->frame_interval.denominator;
	if ((s64)fps * 1000 <= max_mili)
		return;

	fps = max_t(int, max_hz, 1);

	dev_warn(dev, "%d fps is beyond the mode's limit, dropping to %d\n",
		 sensor->frame_interval.denominator, fps);

	ret = 0;
	cci_write(sensor->regmap, HM5065_REG_DESIRED_FRAME_RATE_NUM, fps,
		  &ret);
	cci_write(sensor->regmap, HM5065_REG_DESIRED_FRAME_RATE_DEN, 1, &ret);
	if (!ret)
		sensor->frame_interval.denominator = fps;
}

static int hm5065_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	ret = hm5065_setup_mode(sensor, state);
	if (ret)
		goto err_rpm_put;

	/*
	 * The mode setup soft power cycles the camera MCU, and the sensor
	 * may just have been powered up with the controls only cached, so
	 * reapply them on top of it. The handler lock is the state lock,
	 * which the caller already holds.
	 */
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	if (ret)
		goto err_rpm_put;

	ret = hm5065_set_stream(sensor, true);
	if (ret)
		goto err_rpm_put;

	hm5065_check_frame_rate(sensor);

	if (sensor->ctrls.focus_auto->cur.val) {
		msleep(100);

		/* checking error here is not super important */
		cci_write(sensor->regmap, HM5065_REG_AF_MODE,
			  HM5065_REG_AF_MODE_CONTINUOUS, NULL);
	}

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int hm5065_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	int ret;

	ret = hm5065_set_stream(sensor, false);

	pm_runtime_put_autosuspend(&sensor->i2c_client->dev);

	return ret;
}

/* }}} */
/* {{{ Pad ops */

static int hm5065_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= HM5065_NUM_FORMATS)
		return -EINVAL;

	code->code = hm5065_formats[code->index].code;

	return 0;
}

static int hm5065_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct hm5065_pixfmt *pixfmt = hm5065_find_format(fse->code);

	if (fse->pad != 0)
		return -EINVAL;

	/*
	 * Raw is not scaled, so rather than a range it has exactly the
	 * three sizes the readout produces.
	 */
	if (pixfmt && hm5065_format_is_raw(pixfmt)) {
		static const unsigned int bin[] = { 1, 2, 4 };

		if (fse->index >= ARRAY_SIZE(bin))
			return -EINVAL;

		fse->min_width = HM5065_SENSOR_WIDTH / bin[fse->index];
		fse->max_width = fse->min_width;
		fse->min_height = HM5065_SENSOR_HEIGHT / bin[fse->index];
		fse->max_height = fse->min_height;

		return 0;
	}

	if (fse->index != 0)
		return -EINVAL;

	fse->min_width = HM5065_CAPTURE_WIDTH_MIN;
	fse->min_height = HM5065_CAPTURE_HEIGHT_MIN;

	fse->max_width = HM5065_SENSOR_WIDTH;
	fse->max_height = HM5065_SENSOR_HEIGHT;

	return 0;
}

static int hm5065_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_interval_enum
				      *fie)
{
	struct v4l2_fract tpf;
	u32 max_fps, width, height;

	if (fie->pad != 0)
		return -EINVAL;

	width = clamp(fie->width, HM5065_CAPTURE_WIDTH_MIN,
		      HM5065_SENSOR_WIDTH);
	height = clamp(fie->height, HM5065_CAPTURE_HEIGHT_MIN,
		       HM5065_SENSOR_HEIGHT);

	max_fps = hm5065_get_max_fps(width, height);

	if (fie->index + 1 > max_fps)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = fie->index + 1;
	fie->interval = tpf;

	return 0;
}

static int hm5065_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct hm5065_pixfmt *pixfmt;

	if (format->pad != 0)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && sd->enabled_pads)
		return -EBUSY;

	/* check if we support requested mbus fmt */
	pixfmt = hm5065_find_format(mf->code);
	if (!pixfmt)
		pixfmt = &hm5065_formats[0];

	mf->code = pixfmt->code;
	mf->colorspace = pixfmt->colorspace;
	mf->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	mf->ycbcr_enc = V4L2_YCBCR_ENC_601;
	mf->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	mf->field = V4L2_FIELD_NONE;

	mf->width = clamp(mf->width, HM5065_CAPTURE_WIDTH_MIN,
			  HM5065_SENSOR_WIDTH);
	mf->height = clamp(mf->height, HM5065_CAPTURE_HEIGHT_MIN,
			  HM5065_SENSOR_HEIGHT);

	/*
	 * Raw comes straight off the readout, so the only sizes available
	 * are the ones the readout itself produces: full, and the two
	 * binned modes. Anything else would need the scaler, which is part
	 * of the processing being bypassed.
	 */
	if (hm5065_format_is_raw(pixfmt)) {
		unsigned int bin = 1;

		/*
		 * Round the request up to a readout size rather than picking
		 * the most binning that fits strictly inside it, which is what
		 * hm5065_get_max_binning() is for and which would make two of
		 * the three sizes enum_frame_size() offers unreachable: asking
		 * for exactly 1296x972 would return the full 2592x1944.
		 */
		if (mf->width <= HM5065_SENSOR_WIDTH / 4 &&
		    mf->height <= HM5065_SENSOR_HEIGHT / 4)
			bin = 4;
		else if (mf->width <= HM5065_SENSOR_WIDTH / 2 &&
			 mf->height <= HM5065_SENSOR_HEIGHT / 2)
			bin = 2;

		mf->width = HM5065_SENSOR_WIDTH / bin;
		mf->height = HM5065_SENSOR_HEIGHT / bin;
	}

	*v4l2_subdev_state_get_format(sd_state, format->pad) = *mf;

	return 0;
}

/*
 * The firmware crops the field of view to the aspect ratio of the
 * requested output size all by itself - a 16:9 output really is the
 * middle 16:9 of the sensor, not the whole frame squashed - and there is
 * no way to ask for anything else: the zoom engine and the manual crop
 * registers this family documents do not do anything on this chip. So
 * the crop is a consequence of the format, and all we can do is report
 * the rectangle the sensor is actually reading out.
 */
static void hm5065_auto_crop(const struct v4l2_mbus_framefmt *fmt,
			     struct v4l2_rect *r)
{
	unsigned int w = HM5065_SENSOR_WIDTH;
	unsigned int h = HM5065_SENSOR_HEIGHT;

	if (!fmt->width || !fmt->height) {
		r->left = r->top = 0;
		r->width = w;
		r->height = h;
		return;
	}

	/* the largest centred window of the output's shape */
	if ((u64)w * fmt->height > (u64)h * fmt->width)
		w = h * fmt->width / fmt->height;
	else
		h = w * fmt->height / fmt->width;

	r->width = w;
	r->height = h;
	r->left = (HM5065_SENSOR_WIDTH - w) / 2;
	r->top = (HM5065_SENSOR_HEIGHT - h) / 2;
}

static int hm5065_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	if (sel->pad != 0)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		hm5065_auto_crop(v4l2_subdev_state_get_format(sd_state, 0),
				 &sel->r);
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = HM5065_SENSOR_WIDTH;
		sel->r.height = HM5065_SENSOR_HEIGHT;
		return 0;
	default:
		return -EINVAL;
	}
}

static int hm5065_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = hm5065_formats[0].code,
			.width = 1280,
			.height = 720,
		},
	};

	return hm5065_set_fmt(sd, state, &fmt);
}

/* }}} */
/* {{{ Core Ops */

static void hm5065_chip_enable(struct hm5065_dev *sensor, bool enable)
{
	gpiod_set_value(sensor->enable_gpio, enable ? 1 : 0);
	gpiod_set_value(sensor->reset_gpio, enable ? 0 : 1);
}

static int hm5065_identify(struct hm5065_dev *sensor)
{
	u64 device_id;
	int ret;

	ret = cci_read(sensor->regmap, HM5065_REG_DEVICE_ID, &device_id, NULL);
	if (ret)
		return ret;

	if (device_id != HM5065_REG_DEVICE_ID_VALUE) {
		dev_err(&sensor->i2c_client->dev,
			"unsupported device id: 0x%04x\n",
			(unsigned int)device_id);
		return -ENODEV;
	}

	return 0;
}

static int hm5065_configure(struct hm5065_dev *sensor)
{
	const struct hm5065_clk_lut *lut;
	unsigned long xclk_freq;
	int ret;

	xclk_freq = clk_get_rate(sensor->xclk);
	lut = hm5065_find_clk_lut(xclk_freq);
	if (!lut) {
		dev_err(&sensor->i2c_client->dev,
			"xclk frequency out of range: %lu Hz\n", xclk_freq);
		return -EINVAL;
	}

	ret = cci_write(sensor->regmap, HM5065_REG_EXCLOCKLUT, lut->lut_id,
			NULL);
	if (ret)
		return ret;

	ret = hm5065_load_firmware(sensor, HM5065_AF_FIRMWARE);
	if (ret < 0)
		return ret;

	if (ret == 0) /* ret == 1 means firmware file missing */
		mdelay(200);

	ret = hm5065_load_firmware(sensor, HM5065_FIRMWARE_PARAMETERS);
	if (ret < 0)
		return ret;

	return cci_write(sensor->regmap, HM5065_REG_BUS_CONFIG,
			 sensor->ep.bus_type == V4L2_MBUS_BT656 ?
			 HM5065_REG_BUS_CONFIG_BT656 :
			 HM5065_REG_BUS_CONFIG_PARALLEL_HH_VL, NULL);
}

static void hm5065_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);

	clk_disable_unprepare(sensor->xclk);
	hm5065_chip_enable(sensor, false);
	regulator_bulk_disable(HM5065_NUM_SUPPLIES, sensor->supplies);
	msleep(100);
}

static int hm5065_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	int ret;

	/* the chip is held in reset already, see the pin request */
	ret = regulator_bulk_enable(HM5065_NUM_SUPPLIES, sensor->supplies);
	if (ret)
		return ret;

	/* the supplies are shared with the other camera, let them settle */
	usleep_range(10000, 12000);

	/* pick the rate before the clock starts, not while it runs */
	ret = clk_set_rate(sensor->xclk, 24000000);
	if (ret)
		goto power_off;

	ret = clk_prepare_enable(sensor->xclk);
	if (ret)
		goto power_off;

	usleep_range(1000, 2000);
	hm5065_chip_enable(sensor, false);
	usleep_range(1000, 2000);
	hm5065_chip_enable(sensor, true);
	usleep_range(50000, 70000);

	return 0;

power_off:
	hm5065_chip_enable(sensor, false);
	regulator_bulk_disable(HM5065_NUM_SUPPLIES, sensor->supplies);
	return ret;
}

static int hm5065_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	int ret;

	ret = hm5065_power_on(dev);
	if (ret)
		return ret;

	ret = hm5065_configure(sensor);
	if (ret) {
		hm5065_power_off(dev);
		return ret;
	}

	/* the mode is applied when streaming starts */
	return 0;
}

static int hm5065_runtime_suspend(struct device *dev)
{
	hm5065_power_off(dev);

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int hm5065_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct v4l2_subdev_state *state;
	int ret;
	u64 val = 0;

	if (reg->reg > 0xffff)
		return -EINVAL;

	reg->size = 1;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	ret = cci_read(sensor->regmap, CCI_REG8(reg->reg), &val, NULL);
	v4l2_subdev_unlock_state(state);
	if (ret)
		return -EIO;

	reg->val = val;
	return 0;
}

static int hm5065_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct v4l2_subdev_state *state;
	int ret;

	if (reg->reg > 0xffff || reg->val > 0xff)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	ret = cci_write(sensor->regmap, CCI_REG8(reg->reg), reg->val, NULL);
	v4l2_subdev_unlock_state(state);

	return ret;
}
#endif

/* }}} */

static const struct v4l2_subdev_core_ops hm5065_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = hm5065_g_register,
	.s_register = hm5065_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops hm5065_pad_ops = {
	.enum_mbus_code = hm5065_enum_mbus_code,
	.enum_frame_size = hm5065_enum_frame_size,
	.enum_frame_interval = hm5065_enum_frame_interval,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = hm5065_set_fmt,
	.get_selection = hm5065_get_selection,
	.get_frame_interval = hm5065_get_frame_interval,
	.set_frame_interval = hm5065_set_frame_interval,
	.enable_streams = hm5065_enable_streams,
	.disable_streams = hm5065_disable_streams,
};

static const struct v4l2_subdev_video_ops hm5065_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_ops hm5065_subdev_ops = {
	.core = &hm5065_core_ops,
	.pad = &hm5065_pad_ops,
	.video = &hm5065_video_ops,
};

static const struct v4l2_subdev_internal_ops hm5065_internal_ops = {
	.init_state = hm5065_init_state,
};

static int hm5065_get_regulators(struct hm5065_dev *sensor)
{
	int i;

	for (i = 0; i < HM5065_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = hm5065_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       HM5065_NUM_SUPPLIES,
				       sensor->supplies);
}

#define HM5065_PARALLEL_SUPPORT_FLAGS \
	(V4L2_MBUS_HSYNC_ACTIVE_LOW | V4L2_MBUS_VSYNC_ACTIVE_HIGH | \
	 V4L2_MBUS_PCLK_SAMPLE_FALLING | V4L2_MBUS_DATA_ACTIVE_HIGH)

static int hm5065_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct hm5065_dev *sensor;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = 15;

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

	/*
	 * We don't know how to configure the camera for any other parallel
	 * mode, yet.
	 */
	if (sensor->ep.bus_type != V4L2_MBUS_BT656 &&
	    !(sensor->ep.bus_type == V4L2_MBUS_PARALLEL &&
	      (sensor->ep.bus.parallel.flags & HM5065_PARALLEL_SUPPORT_FLAGS) ==
	      HM5065_PARALLEL_SUPPORT_FLAGS)) {
		dev_err(dev, "unsupported bus configuration %d/%08x\n",
			sensor->ep.bus_type, sensor->ep.bus.parallel.flags);
		return -EINVAL;
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}

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

	if (!sensor->enable_gpio && !sensor->reset_gpio) {
		dev_err(dev,
			"either chip enable or reset pin must be configured\n");
		return -EINVAL;
	}

	ret = hm5065_get_regulators(sensor);
	if (ret)
		return ret;

	v4l2_i2c_subdev_init(&sensor->sd, client, &hm5065_subdev_ops);
	sensor->sd.internal_ops = &hm5065_internal_ops;

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE |
			   V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	/* the sensor has to be powered for the device id to be readable */
	ret = hm5065_power_on(dev);
	if (ret)
		goto entity_cleanup;

	ret = hm5065_identify(sensor);
	if (ret) {
		/*
		 * The camera is not reachable this early during boot on
		 * some boards - it answers nothing at all, however long it
		 * is given and however many times it is power cycled, and
		 * only starts talking once the machine is up. Before the
		 * identification moved into probe nothing noticed, because
		 * the first transfer happened when the device was opened.
		 * Ask to be probed again rather than losing the camera for
		 * the rest of the boot.
		 */
		if (ret != -ENODEV)
			ret = -EPROBE_DEFER;

		goto power_off;
	}

	ret = hm5065_init_controls(sensor);
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
	hm5065_power_off(dev);
entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static void hm5065_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm5065_dev *sensor = to_hm5065_dev(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(&sensor->sd);
	v4l2_subdev_cleanup(&sensor->sd);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		hm5065_power_off(dev);
	pm_runtime_set_suspended(dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(hm5065_pm_ops, hm5065_runtime_suspend,
				 hm5065_runtime_resume, NULL);

static const struct i2c_device_id hm5065_id[] = {
	{"hm5065", 0},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, hm5065_id);

static const struct of_device_id hm5065_dt_ids[] = {
	{ .compatible = "himax,hm5065" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hm5065_dt_ids);

static struct i2c_driver hm5065_i2c_driver = {
	.driver = {
		.name  = "hm5065",
		.of_match_table	= hm5065_dt_ids,
		.pm = pm_ptr(&hm5065_pm_ops),
	},
	.id_table = hm5065_id,
	.probe    = hm5065_probe,
	.remove   = hm5065_remove,
};

module_i2c_driver(hm5065_i2c_driver);

MODULE_AUTHOR("Ondrej Jirman <megi@xff.cz>");
MODULE_DESCRIPTION("HM5065 Camera Subdev Driver");
MODULE_LICENSE("GPL");
