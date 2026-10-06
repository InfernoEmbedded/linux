/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#ifndef _SUN6I_ISP_REG_H_
#define _SUN6I_ISP_REG_H_

#include <linux/kernel.h>

#define SUN6I_ISP_ADDR_VALUE(a)			((a) >> 2)

/* Frontend */

#define SUN6I_ISP_SRC_MODE_DRAM			0
#define SUN6I_ISP_SRC_MODE_CSI(n)		(1 + (n))

#define SUN6I_ISP_FE_CFG_REG			0x0
#define SUN6I_ISP_FE_CFG_EN			BIT(0)
#define SUN6I_ISP_FE_CFG_SRC0_MODE(v)		(((v) << 8) & GENMASK(9, 8))
#define SUN6I_ISP_FE_CFG_SRC1_MODE(v)		(((v) << 16) & GENMASK(17, 16))

#define SUN6I_ISP_FE_CTRL_REG			0x4
#define SUN6I_ISP_FE_CTRL_SCAP_EN		BIT(0)
#define SUN6I_ISP_FE_CTRL_VCAP_EN		BIT(1)
#define SUN6I_ISP_FE_CTRL_PARA_READY		BIT(2)
#define SUN6I_ISP_FE_CTRL_LUT_UPDATE		BIT(3)
#define SUN6I_ISP_FE_CTRL_LENS_UPDATE		BIT(4)
#define SUN6I_ISP_FE_CTRL_GAMMA_UPDATE		BIT(5)
#define SUN6I_ISP_FE_CTRL_DRC_UPDATE		BIT(6)
#define SUN6I_ISP_FE_CTRL_DISC_UPDATE		BIT(7)
#define SUN6I_ISP_FE_CTRL_OUTPUT_SPEED_CTRL(v)	(((v) << 16) & GENMASK(17, 16))
#define SUN6I_ISP_FE_CTRL_VCAP_READ_START	BIT(31)

#define SUN6I_ISP_FE_INT_EN_REG			0x8
#define SUN6I_ISP_FE_INT_EN_FINISH		BIT(0)
#define SUN6I_ISP_FE_INT_EN_START		BIT(1)
#define SUN6I_ISP_FE_INT_EN_PARA_SAVE		BIT(2)
#define SUN6I_ISP_FE_INT_EN_PARA_LOAD		BIT(3)
#define SUN6I_ISP_FE_INT_EN_SRC0_FIFO		BIT(4)
#define SUN6I_ISP_FE_INT_EN_SRC1_FIFO		BIT(5)
#define SUN6I_ISP_FE_INT_EN_ROT_FINISH		BIT(6)
#define SUN6I_ISP_FE_INT_EN_LINE_NUM_START	BIT(7)

#define SUN6I_ISP_FE_INT_STA_REG		0xc
#define SUN6I_ISP_FE_INT_STA_CLEAR		0xff
#define SUN6I_ISP_FE_INT_STA_FINISH		BIT(0)
#define SUN6I_ISP_FE_INT_STA_START		BIT(1)
#define SUN6I_ISP_FE_INT_STA_PARA_SAVE		BIT(2)
#define SUN6I_ISP_FE_INT_STA_PARA_LOAD		BIT(3)
#define SUN6I_ISP_FE_INT_STA_SRC0_FIFO		BIT(4)
#define SUN6I_ISP_FE_INT_STA_SRC1_FIFO		BIT(5)
#define SUN6I_ISP_FE_INT_STA_ROT_FINISH		BIT(6)
#define SUN6I_ISP_FE_INT_STA_LINE_NUM_START	BIT(7)

/* Only since sun9i-a80-isp. */
#define SUN6I_ISP_FE_INT_LINE_NUM_REG		0x18
#define SUN6I_ISP_FE_ROT_OF_CFG_REG		0x1c

/* Buffers/tables */

#define SUN6I_ISP_REG_LOAD_ADDR_REG		0x20
#define SUN6I_ISP_REG_SAVE_ADDR_REG		0x24

#define SUN6I_ISP_LUT_TABLE_ADDR_REG		0x28
#define SUN6I_ISP_DRC_TABLE_ADDR_REG		0x2c
#define SUN6I_ISP_STATS_ADDR_REG		0x30

/* SRAM */

#define SUN6I_ISP_SRAM_RW_OFFSET_REG		0x38
#define SUN6I_ISP_SRAM_RW_DATA_REG		0x3c

/* Global */

#define SUN6I_ISP_MODULE_EN_REG			0x40
#define SUN6I_ISP_MODULE_EN_AE			BIT(0)
#define SUN6I_ISP_MODULE_EN_OBC			BIT(1)
#define SUN6I_ISP_MODULE_EN_DPC_LUT		BIT(2)
#define SUN6I_ISP_MODULE_EN_DPC_OTF		BIT(3)
#define SUN6I_ISP_MODULE_EN_BDNF		BIT(4)
#define SUN6I_ISP_MODULE_EN_AWB			BIT(6)
#define SUN6I_ISP_MODULE_EN_WB			BIT(7)
#define SUN6I_ISP_MODULE_EN_LSC			BIT(8)
#define SUN6I_ISP_MODULE_EN_BGC			BIT(9)
#define SUN6I_ISP_MODULE_EN_SAP			BIT(10)
#define SUN6I_ISP_MODULE_EN_AF			BIT(11)
#define SUN6I_ISP_MODULE_EN_RGB2RGB		BIT(12)
#define SUN6I_ISP_MODULE_EN_RGB_DRC		BIT(13)
#define SUN6I_ISP_MODULE_EN_TDNF		BIT(15)
#define SUN6I_ISP_MODULE_EN_AFS			BIT(16)
#define SUN6I_ISP_MODULE_EN_HIST		BIT(17)
#define SUN6I_ISP_MODULE_EN_YUV_GAIN_OFFSET	BIT(18)
#define SUN6I_ISP_MODULE_EN_YUV_DRC		BIT(19)
#define SUN6I_ISP_MODULE_EN_TG			BIT(20)
#define SUN6I_ISP_MODULE_EN_ROT			BIT(21)
#define SUN6I_ISP_MODULE_EN_CONTRAST		BIT(22)
#define SUN6I_ISP_MODULE_EN_CNR			BIT(23)
#define SUN6I_ISP_MODULE_EN_SATU		BIT(24)
/* Only on sun8i-v3s-isp, which is the only variant with a disc module. */
#define SUN6I_ISP_MODULE_EN_DISC		BIT(25)
#define SUN6I_ISP_MODULE_EN_SRC1		BIT(30)
#define SUN6I_ISP_MODULE_EN_SRC0		BIT(31)

#define SUN6I_ISP_MODE_REG			0x44
#define SUN6I_ISP_MODE_INPUT_FMT(v)		((v) & GENMASK(2, 0))
#define SUN6I_ISP_MODE_INPUT_YUV_SEQ(v)		(((v) << 3) & GENMASK(4, 3))
#define SUN6I_ISP_MODE_OTF_DPC(v)		(((v) << 16) & BIT(16))
#define SUN6I_ISP_MODE_SHARP(v)			(((v) << 17) & BIT(17))
#define SUN6I_ISP_MODE_HIST(v)			(((v) << 20) & GENMASK(21, 20))

#define SUN6I_ISP_INPUT_FMT_YUV420		0
#define SUN6I_ISP_INPUT_FMT_YUV422		1
#define SUN6I_ISP_INPUT_FMT_RAW_BGGR		4
#define SUN6I_ISP_INPUT_FMT_RAW_RGGB		5
#define SUN6I_ISP_INPUT_FMT_RAW_GBRG		6
#define SUN6I_ISP_INPUT_FMT_RAW_GRBG		7

#define SUN6I_ISP_INPUT_YUV_SEQ_YUYV		0
#define SUN6I_ISP_INPUT_YUV_SEQ_YVYU		1
#define SUN6I_ISP_INPUT_YUV_SEQ_UYVY		2
#define SUN6I_ISP_INPUT_YUV_SEQ_VYUY		3

#define SUN6I_ISP_IN_CFG_REG			0x48
#define SUN6I_ISP_IN_CFG_STRIDE_DIV16(v)	((v) & GENMASK(10, 0))

#define SUN6I_ISP_IN_LUMA_RGB_ADDR0_REG		0x4c
#define SUN6I_ISP_IN_CHROMA_ADDR0_REG		0x50
#define SUN6I_ISP_IN_LUMA_RGB_ADDR1_REG		0x54
#define SUN6I_ISP_IN_CHROMA_ADDR1_REG		0x58

/* AE */

#define SUN6I_ISP_AE_CFG_REG			0x60
#define SUN6I_ISP_AE_CFG_LOW_BRI_TH(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_AE_CFG_HORZ_NUM(v)		(((v) << 12) & GENMASK(15, 12))
#define SUN6I_ISP_AE_CFG_HIGH_BRI_TH(v)		(((v) << 16) & GENMASK(27, 16))
#define SUN6I_ISP_AE_CFG_VERT_NUM(v)		(((v) << 28) & GENMASK(31, 28))

#define SUN6I_ISP_AE_SIZE_REG			0x64
#define SUN6I_ISP_AE_SIZE_WIDTH(v)		((v) & GENMASK(10, 0))
#define SUN6I_ISP_AE_SIZE_HEIGHT(v)		(((v) << 16) & GENMASK(26, 16))

#define SUN6I_ISP_AE_POS_REG			0x68
#define SUN6I_ISP_AE_POS_HORZ_START(v)		((v) & GENMASK(10, 0))
#define SUN6I_ISP_AE_POS_VERT_START(v)		(((v) << 16) & GENMASK(26, 16))

/*
 * The statistics windows all store their geometry the same odd way: a
 * size is halved and then one less, a position is just halved. The
 * histogram is the exception to the field order, putting its horizontal
 * start in the upper half where the others put the vertical one.
 */

#define SUN6I_ISP_H3A_SIZE(v)			(((v) >> 1) - 1)
#define SUN6I_ISP_H3A_POS(v)			((v) >> 1)

/* AF */

#define SUN6I_ISP_AF_CFG_REG			0x170
#define SUN6I_ISP_AF_CFG_HORZ_NUM(v)		((v) & GENMASK(3, 0))
#define SUN6I_ISP_AF_CFG_VERT_NUM(v)		(((v) << 8) & GENMASK(11, 8))

#define SUN6I_ISP_AF_SIZE_REG			0x174
#define SUN6I_ISP_AF_SIZE_WIDTH(v)		((v) & GENMASK(10, 0))
#define SUN6I_ISP_AF_SIZE_HEIGHT(v)		(((v) << 16) & GENMASK(26, 16))

#define SUN6I_ISP_AF_START_REG			0x178
#define SUN6I_ISP_AF_START_HORZ(v)		((v) & GENMASK(10, 0))
#define SUN6I_ISP_AF_START_VERT(v)		(((v) << 16) & GENMASK(26, 16))

/* AWB */

/*
 * The saturation limit is programmed twice: the second one lands both in
 * the upper half of the first register and in the low half of the
 * second. The sum and difference thresholds that would use awb_cfg2 and
 * awb_cfg3 have no setters on either supported variant -- both are bare
 * returns -- so those two registers are left alone.
 */

#define SUN6I_ISP_AWB_CFG0_REG			0xf8
#define SUN6I_ISP_AWB_CFG0_SATURATION0(v)	((v) & GENMASK(11, 0))
#define SUN6I_ISP_AWB_CFG0_SATURATION1(v)	(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_AWB_CFG1_REG			0xfc
#define SUN6I_ISP_AWB_CFG1_SATURATION1(v)	((v) & GENMASK(11, 0))

#define SUN6I_ISP_AWB_CFG4_REG			0x108
#define SUN6I_ISP_AWB_CFG4_WIDTH(v)		((v) & GENMASK(8, 0))
#define SUN6I_ISP_AWB_CFG4_HEIGHT(v)		(((v) << 16) & GENMASK(24, 16))

#define SUN6I_ISP_AWB_CFG5_REG			0x10c
#define SUN6I_ISP_AWB_CFG5_HORZ_START(v)	((v) & GENMASK(10, 0))
#define SUN6I_ISP_AWB_CFG5_VERT_START(v)	(((v) << 16) & GENMASK(26, 16))

/* AFS */

#define SUN6I_ISP_AFS_CFG_REG			0x1b8
#define SUN6I_ISP_AFS_CFG_ANTI_FLICKER(v)	((v) & GENMASK(5, 0))

/* Histogram */

#define SUN6I_ISP_HIST_SIZE_REG			0x110
#define SUN6I_ISP_HIST_SIZE_WIDTH(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_HIST_SIZE_HEIGHT(v)		(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_HIST_POS_REG			0x114
#define SUN6I_ISP_HIST_POS_VERT(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_HIST_POS_HORZ(v)		(((v) << 16) & GENMASK(27, 16))

/* OB */

#define SUN6I_ISP_OB_SIZE_REG			0x78
#define SUN6I_ISP_OB_SIZE_WIDTH(v)		((v) & GENMASK(13, 0))
#define SUN6I_ISP_OB_SIZE_HEIGHT(v)		(((v) << 16) & GENMASK(29, 16))

#define SUN6I_ISP_OB_VALID_REG			0x7c
#define SUN6I_ISP_OB_VALID_WIDTH(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_OB_VALID_HEIGHT(v)		(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_OB_SRC0_VALID_START_REG	0x80
#define SUN6I_ISP_OB_SRC0_VALID_START_HORZ(v)	((v) & GENMASK(11, 0))
#define SUN6I_ISP_OB_SRC0_VALID_START_VERT(v)	(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_OB_SRC1_VALID_START_REG	0x84
#define SUN6I_ISP_OB_SRC1_VALID_START_HORZ(v)	((v) & GENMASK(11, 0))
#define SUN6I_ISP_OB_SRC1_VALID_START_VERT(v)	(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_OB_SPRITE_REG			0x88
#define SUN6I_ISP_OB_SPRITE_WIDTH(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_OB_SPRITE_HEIGHT(v)		(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_OB_SPRITE_START_REG		0x8c
#define SUN6I_ISP_OB_SPRITE_START_HORZ(v)	((v) & GENMASK(11, 0))
#define SUN6I_ISP_OB_SPRITE_START_VERT(v)	(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_OB_CFG_REG			0x90
#define SUN6I_ISP_OB_HORZ_POS_REG		0x94
#define SUN6I_ISP_OB_VERT_PARA_REG		0x98
#define SUN6I_ISP_OB_OFFSET_FIXED_REG		0x9c

/*
 * Per-channel black level. Named from the vendor's register map, which
 * maps all four and writes none of them, on any of its six variants.
 */
#define SUN6I_ISP_SRC0_OB_CLAMP_R_REG		0xa0
#define SUN6I_ISP_SRC0_OB_CLAMP_GR_REG		0xa4
#define SUN6I_ISP_SRC0_OB_CLAMP_GB_REG		0xa8
#define SUN6I_ISP_SRC0_OB_CLAMP_B_REG		0xac

/* Defect pixel correction */

/*
 * The on-the-fly corrector, which needs no table. The table-driven one at
 * 0xc0 is not programmed: it wants a defect list in the LUT region, and
 * every block that reads from a table is inert on this part.
 */
#define SUN6I_ISP_OTF_DC_CFG_REG		0xc4
#define SUN6I_ISP_OTF_DC_CFG_TH_SLOPE(v)	((v) & GENMASK(3, 0))
#define SUN6I_ISP_OTF_DC_CFG_MIN_TH(v)		(((v) << 4) & GENMASK(15, 4))
#define SUN6I_ISP_OTF_DC_CFG_MAX_TH(v)		(((v) << 16) & GENMASK(27, 16))

/* BDNF */

#define SUN6I_ISP_BDNF_CFG_REG			0xcc
#define SUN6I_ISP_BDNF_CFG_IN_DIS_MIN(v)	((v) & GENMASK(7, 0))
#define SUN6I_ISP_BDNF_CFG_IN_DIS_MAX(v)	(((v) << 16) & GENMASK(23, 16))

#define SUN6I_ISP_BDNF_COEF_RB_REG		0xd0
#define SUN6I_ISP_BDNF_COEF_RB(i, v)		(((v) << (4 * (i))) & \
						 GENMASK(4 * (i) + 3, 4 * (i)))

#define SUN6I_ISP_BDNF_COEF_G_REG		0xd4
#define SUN6I_ISP_BDNF_COEF_G(i, v)		(((v) << (4 * (i))) & \
						 GENMASK(4 * (i) + 3, 4 * (i)))

/* Bayer */

#define SUN6I_ISP_BAYER_OFFSET0_REG		0xe0
#define SUN6I_ISP_BAYER_OFFSET0_R(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_BAYER_OFFSET0_GR(v)		(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_BAYER_OFFSET1_REG		0xe4
#define SUN6I_ISP_BAYER_OFFSET1_GB(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_BAYER_OFFSET1_B(v)		(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_BAYER_GAIN0_REG		0xe8
#define SUN6I_ISP_BAYER_GAIN0_R(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_BAYER_GAIN0_GR(v)		(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_BAYER_GAIN1_REG		0xec
#define SUN6I_ISP_BAYER_GAIN1_GB(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_BAYER_GAIN1_B(v)		(((v) << 16) & GENMASK(27, 16))

/*
 * YUV gain/offset
 *
 * This is the same register block as the bayer gain/offset above, laid
 * out for three components instead of four and with narrower offsets.
 * MODULE_EN picks which of the two the hardware applies, so only one of
 * them can be in use at a time.
 */

#define SUN6I_ISP_YUV_OFFSET0_REG		0xe0
#define SUN6I_ISP_YUV_OFFSET0_Y(v)		((v) & GENMASK(10, 0))
#define SUN6I_ISP_YUV_OFFSET0_U(v)		(((v) << 16) & GENMASK(26, 16))

#define SUN6I_ISP_YUV_OFFSET1_REG		0xe4
#define SUN6I_ISP_YUV_OFFSET1_V(v)		((v) & GENMASK(10, 0))

#define SUN6I_ISP_YUV_GAIN0_REG			0xe8
#define SUN6I_ISP_YUV_GAIN0_Y(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_YUV_GAIN0_U(v)		(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_YUV_GAIN1_REG			0xec
#define SUN6I_ISP_YUV_GAIN1_V(v)		((v) & GENMASK(11, 0))

/* WB */

#define SUN6I_ISP_WB_GAIN0_REG			0x140
#define SUN6I_ISP_WB_GAIN0_R(v)			((v) & GENMASK(11, 0))
#define SUN6I_ISP_WB_GAIN0_GR(v)		(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_WB_GAIN1_REG			0x144
#define SUN6I_ISP_WB_GAIN1_GB(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_WB_GAIN1_B(v)			(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_WB_CFG_REG			0x148
#define SUN6I_ISP_WB_CFG_CLIP(v)		((v) & GENMASK(11, 0))

/* Sharpness */

#define SUN6I_ISP_SAP_CFG_REG			0x160
#define SUN6I_ISP_SAP_CFG_LEVEL(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_SAP_CFG_MIN(v)		(((v) << 16) & GENMASK(23, 16))
#define SUN6I_ISP_SAP_CFG_MAX(v)		(((v) << 24) & GENMASK(31, 24))

/* RGB2RGB */

/*
 * A 3x3 colour correction matrix followed by a per-channel offset. The
 * twelve values are packed two to a register, except that the last
 * matrix coefficient shares a register with the first offset.
 */

#define SUN6I_ISP_RGB2RGB_GAIN0_REG		0x188
#define SUN6I_ISP_RGB2RGB_GAIN1_REG		0x18c
#define SUN6I_ISP_RGB2RGB_GAIN2_REG		0x190
#define SUN6I_ISP_RGB2RGB_GAIN3_REG		0x194
#define SUN6I_ISP_RGB2RGB_GAIN_LOW(v)		((v) & GENMASK(11, 0))
#define SUN6I_ISP_RGB2RGB_GAIN_HIGH(v)		(((v) << 16) & GENMASK(27, 16))

#define SUN6I_ISP_RGB2RGB_GAIN4_REG		0x198
#define SUN6I_ISP_RGB2RGB_GAIN4_MATRIX(v)	((v) & GENMASK(11, 0))
#define SUN6I_ISP_RGB2RGB_GAIN4_OFFSET_R(v)	(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_RGB2RGB_OFFSET_REG		0x19c
#define SUN6I_ISP_RGB2RGB_OFFSET_G(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_RGB2RGB_OFFSET_B(v)		(((v) << 16) & GENMASK(28, 16))

/* Contrast */

#define SUN6I_ISP_CONTRAST_CFG_REG		0x164
#define SUN6I_ISP_CONTRAST_CFG_LEVEL(v)		((v) & GENMASK(3, 0))
#define SUN6I_ISP_CONTRAST_CFG_MIN(v)		(((v) << 8) & GENMASK(15, 8))
#define SUN6I_ISP_CONTRAST_CFG_MAX(v)		(((v) << 16) & GENMASK(23, 16))

/* Saturation */

#define SUN6I_ISP_SATURATION_CFG_REG		0x168
#define SUN6I_ISP_SATURATION_CFG_R(v)		((v) & GENMASK(3, 0))
#define SUN6I_ISP_SATURATION_CFG_G(v)		(((v) << 4) & GENMASK(7, 4))
#define SUN6I_ISP_SATURATION_CFG_B(v)		(((v) << 8) & GENMASK(11, 8))
#define SUN6I_ISP_SATURATION_CFG_GAIN(v)	(((v) << 16) & GENMASK(24, 16))

/* LSC */

#define SUN6I_ISP_LSC_CFG_REG			0x150
#define SUN6I_ISP_LSC_CFG_CENTER_X(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_LSC_CFG_CENTER_Y(v)		(((v) << 13) & GENMASK(25, 13))
#define SUN6I_ISP_LSC_CFG_SCALE(v)		(((v) << 26) & GENMASK(30, 26))

/* Demosaic */

#define SUN6I_ISP_CFA_CFG0_REG			0x154
#define SUN6I_ISP_CFA_CFG0_DIR_TH(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_CFA_CFG0_MIN_RGB(v)		(((v) << 16) & GENMASK(25, 16))
#define SUN6I_ISP_CFA_CFG0_MIN_RGB_GET(v)	(((v) & GENMASK(25, 16)) >> 16)

/*
 * The minimum goes into three fields across two registers, the same value
 * in each. The hardware writes its own measurement back over the same
 * bits, so the save region reads a level rather than what was staged.
 */
#define SUN6I_ISP_CFA_CFG1_REG			0x158
#define SUN6I_ISP_CFA_CFG1_MIN_RGB_LOW(v)	((v) & GENMASK(9, 0))
#define SUN6I_ISP_CFA_CFG1_MIN_RGB_HIGH(v)	(((v) << 16) & GENMASK(25, 16))

#define SUN6I_ISP_CFA_TEX_REG			0x15c

/* CNR */

#define SUN6I_ISP_CNR_REG			0x14c
#define SUN6I_ISP_CNR_OFFSET(v)			((v) & GENMASK(11, 0))
/*
 * The upper half of this register is not configuration: the save block
 * reports the measured chroma noise level back in GENMASK(31, 16).
 */

/* Global */

#define SUN6I_ISP_MCH_SIZE_CFG_REG		0x1e0
#define SUN6I_ISP_MCH_SIZE_CFG_WIDTH(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_MCH_SIZE_CFG_HEIGHT(v)	(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_MCH_SCALE_CFG_REG		0x1e4
#define SUN6I_ISP_MCH_SCALE_CFG_X_RATIO(v)	((v) & GENMASK(11, 0))
#define SUN6I_ISP_MCH_SCALE_CFG_Y_RATIO(v)	(((v) << 16) & GENMASK(27, 16))
#define SUN6I_ISP_MCH_SCALE_CFG_WEIGHT_SHIFT(v)	(((v) << 28) & GENMASK(31, 28))

#define SUN6I_ISP_SCH_SIZE_CFG_REG		0x1e8
#define SUN6I_ISP_SCH_SIZE_CFG_WIDTH(v)		((v) & GENMASK(12, 0))
#define SUN6I_ISP_SCH_SIZE_CFG_HEIGHT(v)	(((v) << 16) & GENMASK(28, 16))

#define SUN6I_ISP_SCH_SCALE_CFG_REG		0x1ec
#define SUN6I_ISP_SCH_SCALE_CFG_X_RATIO(v)	((v) & GENMASK(11, 0))
#define SUN6I_ISP_SCH_SCALE_CFG_Y_RATIO(v)	(((v) << 16) & GENMASK(27, 16))
#define SUN6I_ISP_SCH_SCALE_CFG_WEIGHT_SHIFT(v)	(((v) << 28) & GENMASK(31, 28))

#define SUN6I_ISP_MCH_CFG_REG			0x1f0
#define SUN6I_ISP_MCH_CFG_EN			BIT(0)
#define SUN6I_ISP_MCH_CFG_SCALE_EN		BIT(1)
#define SUN6I_ISP_MCH_CFG_OUTPUT_FMT(v)		(((v) << 2) & GENMASK(4, 2))
#define SUN6I_ISP_MCH_CFG_MIRROR_EN		BIT(5)
#define SUN6I_ISP_MCH_CFG_FLIP_EN		BIT(6)
#define SUN6I_ISP_MCH_CFG_STRIDE_Y_DIV4(v)	(((v) << 8) & GENMASK(18, 8))
#define SUN6I_ISP_MCH_CFG_STRIDE_UV_DIV4(v)	(((v) << 20) & GENMASK(30, 20))

#define SUN6I_ISP_OUTPUT_FMT_YUV420SP		0
#define SUN6I_ISP_OUTPUT_FMT_YUV422SP		1
#define SUN6I_ISP_OUTPUT_FMT_YVU420SP		2
#define SUN6I_ISP_OUTPUT_FMT_YVU422SP		3
#define SUN6I_ISP_OUTPUT_FMT_YUV420P		4
#define SUN6I_ISP_OUTPUT_FMT_YUV422P		5
#define SUN6I_ISP_OUTPUT_FMT_YVU420P		6
#define SUN6I_ISP_OUTPUT_FMT_YVU422P		7

#define SUN6I_ISP_SCH_CFG_REG			0x1f4

#define SUN6I_ISP_MCH_Y_ADDR0_REG		0x1f8
#define SUN6I_ISP_MCH_U_ADDR0_REG		0x1fc
#define SUN6I_ISP_MCH_V_ADDR0_REG		0x200
#define SUN6I_ISP_MCH_Y_ADDR1_REG		0x204
#define SUN6I_ISP_MCH_U_ADDR1_REG		0x208
#define SUN6I_ISP_MCH_V_ADDR1_REG		0x20c
#define SUN6I_ISP_SCH_Y_ADDR0_REG		0x210
#define SUN6I_ISP_SCH_U_ADDR0_REG		0x214
#define SUN6I_ISP_SCH_V_ADDR0_REG		0x218

#define SUN6I_ISP_ROT_CFG_REG			0x230
#define SUN6I_ISP_ROT_CFG_SRC_SCH		BIT(0)
#define SUN6I_ISP_ROT_CFG_OUTPUT_FMT(v)		(((v) << 2) & GENMASK(4, 2))
#define SUN6I_ISP_ROT_CFG_ANGLE(v)		(((v) << 5) & GENMASK(6, 5))
#define SUN6I_ISP_ROT_CFG_STRIDE_Y_DIV4(v)	(((v) << 8) & GENMASK(18, 8))
#define SUN6I_ISP_ROT_CFG_STRIDE_UV_DIV4(v)	(((v) << 20) & GENMASK(30, 20))

#define SUN6I_ISP_ROT_ANGLE_0			0
#define SUN6I_ISP_ROT_ANGLE_90			1
#define SUN6I_ISP_ROT_ANGLE_180			2
#define SUN6I_ISP_ROT_ANGLE_270			3

#define SUN6I_ISP_ROT_Y_ADDR0_REG		0x234
#define SUN6I_ISP_ROT_U_ADDR0_REG		0x238
#define SUN6I_ISP_ROT_V_ADDR0_REG		0x23c
#define SUN6I_ISP_SCH_Y_ADDR1_REG		0x21c
#define SUN6I_ISP_SCH_U_ADDR1_REG		0x220
#define SUN6I_ISP_SCH_V_ADDR1_REG		0x224

#endif
