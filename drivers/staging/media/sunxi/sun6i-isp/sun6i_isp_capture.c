// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2022 Bootlin
 * Author: Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 */

#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "sun6i_isp.h"
#include "sun6i_isp_capture.h"
#include "sun6i_isp_proc.h"
#include "sun6i_isp_reg.h"

/* Channels */

static const struct sun6i_isp_capture_regs sun6i_isp_capture_regs_main = {
	.size_cfg	= SUN6I_ISP_MCH_SIZE_CFG_REG,
	.scale_cfg	= SUN6I_ISP_MCH_SCALE_CFG_REG,
	.cfg		= SUN6I_ISP_MCH_CFG_REG,
	.y_addr0	= SUN6I_ISP_MCH_Y_ADDR0_REG,
	.u_addr0	= SUN6I_ISP_MCH_U_ADDR0_REG,
	.v_addr0	= SUN6I_ISP_MCH_V_ADDR0_REG,
};

static const struct sun6i_isp_capture_regs sun6i_isp_capture_regs_self = {
	.size_cfg	= SUN6I_ISP_SCH_SIZE_CFG_REG,
	.scale_cfg	= SUN6I_ISP_SCH_SCALE_CFG_REG,
	.cfg		= SUN6I_ISP_SCH_CFG_REG,
	.y_addr0	= SUN6I_ISP_SCH_Y_ADDR0_REG,
	.u_addr0	= SUN6I_ISP_SCH_U_ADDR0_REG,
	.v_addr0	= SUN6I_ISP_SCH_V_ADDR0_REG,
};

/*
 * There is one rotator and it is not a channel: it reads back what one of
 * the two channels wrote and lays a turned copy through its own addresses.
 * Which channel it reads is bit 0 of its config register, so it is a mode
 * of a channel rather than an output of its own.
 */
static const struct sun6i_isp_capture_regs sun6i_isp_capture_regs_rot = {
	.cfg		= SUN6I_ISP_ROT_CFG_REG,
	.y_addr0	= SUN6I_ISP_ROT_Y_ADDR0_REG,
	.u_addr0	= SUN6I_ISP_ROT_U_ADDR0_REG,
	.v_addr0	= SUN6I_ISP_ROT_V_ADDR0_REG,
};

/*
 * Hand out the register blocks to the channels that are streaming.
 *
 * The self channel does not honour its own size, scale and output format
 * registers unless the main channel is enabled alongside it. Left to itself it
 * ignores them and lays down a full sized 4:2:2 frame - 1843200 bytes at 720p
 * whatever was asked for, at every size and format tried - which overruns any
 * buffer smaller than that. The main channel through the same code is right
 * every time, and the two together are right as well. The vendor's
 * bsp_isp_set_fmt() has the same shape: it only programs the self channel's
 * format when a second channel is in play, never on its own.
 *
 * The two blocks are laid out identically and take the same formats, so the
 * answer is to give the main block to whichever channel is streaming and only
 * use the self block when both are. Channels are visited in order and the main
 * channel comes first, so handing the blocks out in order does exactly that.
 *
 * Which block a channel drives can only change here, and this only runs while
 * the pipeline is stopped - see sun6i_isp_pipeline_restart().
 */
static void sun6i_isp_capture_blocks_assign(struct sun6i_isp_device *isp_dev)
{
	static const struct sun6i_isp_capture_regs * const blocks[] = {
		&sun6i_isp_capture_regs_main,
		&sun6i_isp_capture_regs_self,
	};
	unsigned int next = 0;
	unsigned int i;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++) {
		struct sun6i_isp_capture *capture = &isp_dev->captures[i];

		capture->regs = capture->state.streaming ? blocks[next++] : NULL;
	}
}

/*
 * Only a quarter turn needs the rotator. A half turn is a mirror and a flip
 * at once, which the channel does by itself for free - no second image in
 * the buffer for the rotator to read back, and no rotator to go wrong. It is
 * also the only way to get 180 out of this hardware: the rotator produces
 * nothing at all at that angle.
 */
static bool sun6i_isp_capture_quarter_turn(struct sun6i_isp_capture *capture)
{
	return capture->rotation == 90 || capture->rotation == 270;
}

static bool sun6i_isp_capture_half_turn(struct sun6i_isp_capture *capture)
{
	return capture->rotation == 180;
}

bool sun6i_isp_capture_rotation_taken(struct sun6i_isp_device *isp_dev,
				      struct sun6i_isp_capture *except)
{
	unsigned int i;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++) {
		struct sun6i_isp_capture *capture = &isp_dev->captures[i];

		if (capture != except &&
		    sun6i_isp_capture_quarter_turn(capture))
			return true;
	}

	return false;
}

/*
 * What the channel itself writes, as opposed to what comes out. They differ
 * on a quarter turn, where the rotated copy userspace is handed has the two
 * dimensions the other way round from the one the channel produced.
 */
static void sun6i_isp_capture_source_dimensions(struct sun6i_isp_capture *capture,
						unsigned int *width,
						unsigned int *height)
{
	sun6i_isp_capture_dimensions(capture, width, height);

	if (sun6i_isp_capture_quarter_turn(capture))
		swap(*width, *height);
}

/* Helpers */

void sun6i_isp_capture_dimensions(struct sun6i_isp_capture *capture,
				  unsigned int *width, unsigned int *height)
{
	if (width)
		*width = capture->format.fmt.pix.width;
	if (height)
		*height = capture->format.fmt.pix.height;
}

void sun6i_isp_capture_format(struct sun6i_isp_capture *capture,
			      u32 *pixelformat)
{
	if (pixelformat)
		*pixelformat = capture->format.fmt.pix.pixelformat;
}

/* Format */

static const struct sun6i_isp_capture_format sun6i_isp_capture_formats[] = {
	{
		.pixelformat		= V4L2_PIX_FMT_NV12,
		.output_format		= SUN6I_ISP_OUTPUT_FMT_YUV420SP,
	},
	{
		.pixelformat		= V4L2_PIX_FMT_NV21,
		.output_format		= SUN6I_ISP_OUTPUT_FMT_YVU420SP,
	},
	{
		.pixelformat		= V4L2_PIX_FMT_NV16,
		.output_format		= SUN6I_ISP_OUTPUT_FMT_YUV422SP,
	},
	{
		.pixelformat		= V4L2_PIX_FMT_NV61,
		.output_format		= SUN6I_ISP_OUTPUT_FMT_YVU422SP,
	},
	{
		.pixelformat		= V4L2_PIX_FMT_YUV420,
		.output_format		= SUN6I_ISP_OUTPUT_FMT_YUV420P,
	},
	{
		.pixelformat		= V4L2_PIX_FMT_YVU420,
		.output_format		= SUN6I_ISP_OUTPUT_FMT_YVU420P,
	},
	{
		.pixelformat		= V4L2_PIX_FMT_YUV422P,
		.output_format		= SUN6I_ISP_OUTPUT_FMT_YUV422P,
	},
};

const struct sun6i_isp_capture_format *
sun6i_isp_capture_format_find(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sun6i_isp_capture_formats); i++)
		if (sun6i_isp_capture_formats[i].pixelformat == pixelformat)
			return &sun6i_isp_capture_formats[i];

	return NULL;
}

/* Capture */

/*
 * Stride of one plane.
 *
 * The hardware holds strides divided by four, so a stride has to be a
 * multiple of four for the register to be able to express it at all, and
 * the vendor goes further and aligns every plane to 16 - ALIGN(width, 16)
 * for luma and ALIGN(width / 2, 16) for the chroma of a planar format, in
 * its sub_0(). Following that keeps what the register says and what the
 * buffer was laid out for the same thing, which they were not: bytesperline
 * was aligned to 2 while the register rounded up to 4, so a width that is
 * not a multiple of four had the hardware stepping further per row than the
 * buffer allowed and writing off the end of it.
 */
unsigned int sun6i_isp_capture_stride(const struct v4l2_format_info *info,
				      unsigned int plane, unsigned int width)
{
	unsigned int hdiv = (plane == 0) ? 1 : info->hdiv;

	return ALIGN(info->bpp[plane] * DIV_ROUND_UP(width, hdiv),
		     SUN6I_ISP_CAPTURE_STRIDE_ALIGN);
}

/*
 * Program one image's plane addresses, starting at base, into the given
 * register block. Returns the number of bytes the image occupies.
 */
static unsigned int
sun6i_isp_capture_planes_configure(struct sun6i_isp_device *isp_dev,
				   const struct sun6i_isp_capture_regs *regs,
				   dma_addr_t base, u32 pixelformat,
				   unsigned int width, unsigned int height,
				   bool flip)
{
	const u32 plane_regs[] = { regs->y_addr0, regs->u_addr0, regs->v_addr0 };
	const struct v4l2_format_info *info = v4l2_format_info(pixelformat);
	dma_addr_t address = base;
	unsigned int i;

	if (WARN_ON(!info))
		return 0;

	for (i = 0; i < info->comp_planes && i < ARRAY_SIZE(plane_regs); i++) {
		unsigned int vdiv = (i == 0) ? 1 : info->vdiv;
		unsigned int stride = sun6i_isp_capture_stride(info, i, width);
		unsigned int lines = DIV_ROUND_UP(height, vdiv);
		dma_addr_t plane = address;

		/*
		 * Vertical flip is done by walking each plane backwards, so
		 * the hardware has to be given its last line rather than its
		 * first - this is what the vendor's bsp_isp_set_output_addr()
		 * does under its flip flag. Handing it the first line instead
		 * leaves it running off the front of the buffer and into
		 * whatever happens to live below it, which on this board is
		 * the framebuffer.
		 */
		if (flip)
			plane += (lines - 1) * stride;

		sun6i_isp_load_write(isp_dev, plane_regs[i],
				     SUN6I_ISP_ADDR_VALUE(plane));

		address += stride * lines;
	}

	return address - base;
}

unsigned int sun6i_isp_capture_image_size(u32 pixelformat, unsigned int width,
					  unsigned int height)
{
	const struct v4l2_format_info *info = v4l2_format_info(pixelformat);
	unsigned int size = 0;
	unsigned int i;

	if (WARN_ON(!info))
		return 0;

	for (i = 0; i < info->comp_planes; i++) {
		unsigned int vdiv = (i == 0) ? 1 : info->vdiv;

		size += sun6i_isp_capture_stride(info, i, width) *
			DIV_ROUND_UP(height, vdiv);
	}

	return size;
}

/*
 * Where the channel's own output goes. Zero without rotation, where the
 * channel writes the whole buffer; otherwise just past the rotated image,
 * page aligned the way the vendor lays its regions out.
 */
static unsigned int
sun6i_isp_capture_source_offset(struct sun6i_isp_capture *capture)
{
	unsigned int width, height;
	u32 pixelformat;

	if (!sun6i_isp_capture_quarter_turn(capture))
		return 0;

	sun6i_isp_capture_dimensions(capture, &width, &height);
	sun6i_isp_capture_format(capture, &pixelformat);

	return ALIGN(sun6i_isp_capture_image_size(pixelformat, width, height),
		     PAGE_SIZE);
}

/*
 * Lay a buffer out for the hardware.
 *
 * Without rotation the channel writes the whole buffer and that is all there
 * is to it. With rotation the buffer holds two images: the channel writes its
 * own unrotated output into the scratch region at the far end, the rotator
 * reads that back and lays the turned copy at the start - which is where
 * userspace expects to find the frame. This is the vendor's arrangement,
 * where one buffer carries every output at its own offset, with the regions
 * ordered so that the one that matters is the one at offset zero.
 */
static void
sun6i_isp_capture_buffer_configure(struct sun6i_isp_capture *capture,
				   struct sun6i_isp_buffer *isp_buffer)
{
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	struct vb2_buffer *vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;
	dma_addr_t base = vb2_dma_contig_plane_dma_addr(vb2_buffer, 0);
	unsigned int width, height;
	u32 pixelformat;

	sun6i_isp_capture_format(capture, &pixelformat);
	sun6i_isp_capture_source_dimensions(capture, &width, &height);

	sun6i_isp_capture_planes_configure(isp_dev, capture->regs,
					   base + sun6i_isp_capture_source_offset(capture),
					   pixelformat, width, height,
					   capture->flip !=
					   sun6i_isp_capture_half_turn(capture));

	if (!sun6i_isp_capture_quarter_turn(capture))
		return;

	sun6i_isp_capture_dimensions(capture, &width, &height);

	/* The rotator has no flip of its own. */
	sun6i_isp_capture_planes_configure(isp_dev, &sun6i_isp_capture_regs_rot,
					   base, pixelformat, width, height,
					   false);
}

/*
 * The scaler needs a weight shift alongside the two ratios, picked so
 * that the accumulated filter weights stay in range. This is the
 * vendor's calculation: sum the products of the horizontal and vertical
 * tap weights over the whole filter footprint, then take the position of
 * the top bit of that sum, biased by 8.
 *
 * The sum is accumulated in 64 bits rather than the vendor's 32, which
 * only differs for ratios large enough to overflow, where the vendor
 * result would be meaningless anyway.
 */
static int sun6i_isp_capture_weight_shift(u32 x_ratio, u32 y_ratio)
{
	int center_x = ((x_ratio >> 8) + 1) << 7;
	int center_y = ((y_ratio >> 8) + 1) << 7;
	unsigned int taps_x = (x_ratio >> 8) + 2;
	unsigned int taps_y = (y_ratio >> 8) + 2;
	unsigned int i, j;
	int shift = -8;
	s64 sum = 0;

	for (i = 0; i < taps_x; i++) {
		int weight_x = x_ratio - abs((int)(i << 8) - center_x);

		for (j = 0; j < taps_y; j++)
			sum += (s64)weight_x *
			       (y_ratio - abs((int)(j << 8) - center_y));
	}

	for (sum >>= 8; sum; sum >>= 1)
		shift++;

	return shift;
}

static void sun6i_isp_capture_configure_scaler(struct sun6i_isp_capture *capture,
					       unsigned int width,
					       unsigned int height)
{
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	unsigned int input_width, input_height;
	u32 x_ratio, y_ratio;
	int shift;

	sun6i_isp_proc_dimensions(isp_dev, &input_width, &input_height);

	x_ratio = (input_width << 8) / width;
	y_ratio = (input_height << 8) / height;

	shift = sun6i_isp_capture_weight_shift(x_ratio, y_ratio);

	sun6i_isp_load_write(isp_dev, capture->regs->scale_cfg,
			     SUN6I_ISP_MCH_SCALE_CFG_X_RATIO(x_ratio) |
			     SUN6I_ISP_MCH_SCALE_CFG_Y_RATIO(y_ratio) |
			     SUN6I_ISP_MCH_SCALE_CFG_WEIGHT_SHIFT(shift));
}

/*
 * The rotator's own config: which channel it reads back, the angle, and the
 * geometry of what it lays down - which is the channel's, turned.
 */
static void sun6i_isp_capture_rotator_cfg(struct sun6i_isp_capture *capture)
{
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	unsigned int stride_luma_div4, stride_chroma_div4 = 0;
	const struct sun6i_isp_capture_format *format;
	const struct v4l2_format_info *info;
	unsigned int width, height;
	u32 pixelformat, value;
	u32 angle;

	sun6i_isp_capture_dimensions(capture, &width, &height);
	sun6i_isp_capture_format(capture, &pixelformat);

	format = sun6i_isp_capture_format_find(pixelformat);
	info = v4l2_format_info(pixelformat);
	if (WARN_ON(!format || !info))
		return;

	switch (capture->rotation) {
	case 90:
		angle = SUN6I_ISP_ROT_ANGLE_90;
		break;
	case 180:
		angle = SUN6I_ISP_ROT_ANGLE_180;
		break;
	case 270:
		angle = SUN6I_ISP_ROT_ANGLE_270;
		break;
	default:
		angle = SUN6I_ISP_ROT_ANGLE_0;
		break;
	}

	stride_luma_div4 = sun6i_isp_capture_stride(info, 0, width) / 4;

	if (info->comp_planes > 1)
		stride_chroma_div4 =
			sun6i_isp_capture_stride(info, 1, width) / 4;

	value = SUN6I_ISP_ROT_CFG_ANGLE(angle) |
		SUN6I_ISP_ROT_CFG_OUTPUT_FMT(format->output_format) |
		SUN6I_ISP_ROT_CFG_STRIDE_Y_DIV4(stride_luma_div4) |
		SUN6I_ISP_ROT_CFG_STRIDE_UV_DIV4(stride_chroma_div4);

	/*
	 * Bit 0 is the source select and not an enable: clear picks the main
	 * block, set picks the self one. The vendor's isp_rot_src_ch_sel()
	 * sets it for its channel 0, which its isp_channel_enable() shows is
	 * the self channel.
	 *
	 * It follows the block the channel drives rather than which channel it
	 * is, so a self channel holding the main block has the rotator read the
	 * main block back.
	 */
	if (capture->regs == &sun6i_isp_capture_regs_self)
		value |= SUN6I_ISP_ROT_CFG_SRC_SCH;

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_ROT_CFG_REG, value);
}

/*
 * Program the rotator for whichever channel is turning, and turn it off when
 * none is. At most one channel can be, since V4L2_CID_ROTATE refuses to take
 * the rotator away from the other one.
 *
 * This is driven from here rather than by the channel that wants it because
 * the load buffer keeps its contents across streams: a run that leaves the
 * enable set has the rotator carrying on against the address it was last
 * given, which by then is a freed buffer whose pages have gone back to CMA and
 * been handed to somebody else.
 */
static void
sun6i_isp_capture_rotator_configure(struct sun6i_isp_device *isp_dev)
{
	struct sun6i_isp_capture *rotating = NULL;
	unsigned int i;
	u32 value;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++) {
		struct sun6i_isp_capture *capture = &isp_dev->captures[i];

		if (capture->regs && sun6i_isp_capture_quarter_turn(capture))
			rotating = capture;
	}

	value = sun6i_isp_load_read(isp_dev, SUN6I_ISP_MODULE_EN_REG);

	if (rotating) {
		sun6i_isp_capture_rotator_cfg(rotating);
		value |= SUN6I_ISP_MODULE_EN_ROT;
	} else {
		value &= ~SUN6I_ISP_MODULE_EN_ROT;
	}

	sun6i_isp_load_write(isp_dev, SUN6I_ISP_MODULE_EN_REG, value);
}

static void sun6i_isp_capture_configure_one(struct sun6i_isp_capture *capture)
{
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	unsigned int input_width, input_height;
	unsigned int width, height;
	u32 cfg = 0;
	unsigned int stride_luma, stride_chroma;
	unsigned int stride_luma_div4, stride_chroma_div4 = 0;
	const struct sun6i_isp_capture_format *format;
	const struct v4l2_format_info *info;
	u32 pixelformat;

	/*
	 * Everything below is the channel's own output, which with rotation on
	 * is the scratch image the rotator reads back rather than the one
	 * userspace sees - so it is the source geometry throughout.
	 */
	sun6i_isp_capture_source_dimensions(capture, &width, &height);
	sun6i_isp_capture_format(capture, &pixelformat);

	format = sun6i_isp_capture_format_find(pixelformat);
	if (WARN_ON(!format))
		return;

	sun6i_isp_load_write(isp_dev, capture->regs->size_cfg,
			     SUN6I_ISP_MCH_SIZE_CFG_WIDTH(width) |
			     SUN6I_ISP_MCH_SIZE_CFG_HEIGHT(height));

	/*
	 * The scaler is always brought up, even at 1:1 where its ratios come
	 * out 0x100 and it is a pass-through. That is what the vendor does -
	 * channel_enable(), scale_enable(), set_output_size(), scale_cfg(),
	 * in that order, with no test on the size - and leaving it out is not
	 * the harmless optimisation it looks like: the ratio registers are in
	 * the load buffer, which keeps its contents across streams, so a
	 * channel captured at the input resolution runs on whatever ratios
	 * some earlier stream left behind. The self channel writes past the
	 * end of its buffer when that happens, which is why the fault looked
	 * stable within a boot and different across one.
	 */

	sun6i_isp_proc_dimensions(isp_dev, &input_width, &input_height);

	sun6i_isp_capture_configure_scaler(capture, width, height);
	cfg |= SUN6I_ISP_MCH_CFG_SCALE_EN;

	/*
	 * A half turn is mirror and flip together, so it composes with the
	 * two controls rather than overriding them: asking for 180 and a
	 * vertical flip leaves a plain mirror, which is what the two of them
	 * add up to.
	 */
	if (capture->mirror != sun6i_isp_capture_half_turn(capture))
		cfg |= SUN6I_ISP_MCH_CFG_MIRROR_EN;

	if (capture->flip != sun6i_isp_capture_half_turn(capture))
		cfg |= SUN6I_ISP_MCH_CFG_FLIP_EN;

	info = v4l2_format_info(pixelformat);
	if (WARN_ON(!info))
		return;

	stride_luma = sun6i_isp_capture_stride(info, 0, width);
	stride_luma_div4 = stride_luma / 4;

	if (info->comp_planes > 1) {
		stride_chroma = sun6i_isp_capture_stride(info, 1, width);
		stride_chroma_div4 = stride_chroma / 4;
	}

	sun6i_isp_load_write(isp_dev, capture->regs->cfg,
			     cfg | SUN6I_ISP_MCH_CFG_EN |
			     SUN6I_ISP_MCH_CFG_OUTPUT_FMT(format->output_format) |
			     SUN6I_ISP_MCH_CFG_STRIDE_Y_DIV4(stride_luma_div4) |
			     SUN6I_ISP_MCH_CFG_STRIDE_UV_DIV4(stride_chroma_div4));
}

void sun6i_isp_capture_configure(struct sun6i_isp_device *isp_dev)
{
	unsigned int i;

	sun6i_isp_capture_blocks_assign(isp_dev);

	/*
	 * Both blocks are written on every pass. A block with no channel behind
	 * it is written disabled rather than left describing an earlier stream,
	 * with only its enable bit standing between the hardware and a buffer
	 * that by then belongs to nobody.
	 */
	sun6i_isp_load_write(isp_dev, sun6i_isp_capture_regs_main.cfg, 0);
	sun6i_isp_load_write(isp_dev, sun6i_isp_capture_regs_self.cfg, 0);

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++)
		if (isp_dev->captures[i].regs)
			sun6i_isp_capture_configure_one(&isp_dev->captures[i]);

	sun6i_isp_capture_rotator_configure(isp_dev);
}

/*
 * Give a channel's buffers back to the head of its own queue.
 *
 * These are the buffer the hardware was part way through and the one staged
 * behind it. Both are put back in the order they were taken, so that the torn
 * frame is written again from the top rather than handed to userspace as a
 * frame, and the configuration that follows stages them afresh through
 * whichever register block the channel now drives.
 *
 * Nothing is handed to videobuf2 here: this is the driver's own queue, and
 * the buffers never stop belonging to the driver.
 */
static void sun6i_isp_capture_rewind_one(struct sun6i_isp_capture *capture)
{
	struct sun6i_isp_capture_state *state = &capture->state;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	if (state->pending) {
		list_add(&state->pending->list, &state->queue);
		state->pending = NULL;
	}

	if (state->active) {
		list_add(&state->active->list, &state->queue);
		state->active = NULL;
	}

	spin_unlock_irqrestore(&state->lock, flags);
}

/*
 * Give every channel's buffers back to the head of its own queue, ahead of a
 * pipeline restart. See sun6i_isp_pipeline_restart().
 */
void sun6i_isp_capture_rewind(struct sun6i_isp_device *isp_dev)
{
	unsigned int i;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++)
		sun6i_isp_capture_rewind_one(&isp_dev->captures[i]);
}

/* State */

static void sun6i_isp_capture_state_cleanup(struct sun6i_isp_capture *capture,
					    bool error)
{
	struct sun6i_isp_capture_state *state = &capture->state;
	struct sun6i_isp_buffer **isp_buffer_states[] = {
		&state->pending, &state->active,
	};
	struct sun6i_isp_buffer *isp_buffer;
	struct vb2_buffer *vb2_buffer;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&state->lock, flags);

	for (i = 0; i < ARRAY_SIZE(isp_buffer_states); i++) {
		isp_buffer = *isp_buffer_states[i];
		if (!isp_buffer)
			continue;

		vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;
		vb2_buffer_done(vb2_buffer, error ? VB2_BUF_STATE_ERROR :
				VB2_BUF_STATE_QUEUED);

		*isp_buffer_states[i] = NULL;
	}

	list_for_each_entry(isp_buffer, &state->queue, list) {
		vb2_buffer = &isp_buffer->v4l2_buffer.vb2_buf;
		vb2_buffer_done(vb2_buffer, error ? VB2_BUF_STATE_ERROR :
				VB2_BUF_STATE_QUEUED);
	}

	INIT_LIST_HEAD(&state->queue);

	spin_unlock_irqrestore(&state->lock, flags);
}

static void sun6i_isp_capture_state_update_one(struct sun6i_isp_capture *capture,
					       bool *update)
{
	struct sun6i_isp_capture_state *state = &capture->state;
	struct sun6i_isp_buffer *isp_buffer;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	/*
	 * A channel on its way out has its buffers back on this queue, put
	 * there by the rewind, and they are about to go back to userspace.
	 * Its enable bit is clear so the hardware would not write to them, but
	 * there is no reason to hand it an address it must not use.
	 */
	if (!state->streaming)
		goto complete;

	if (list_empty(&state->queue))
		goto complete;

	if (state->pending)
		goto complete;

	isp_buffer = list_first_entry(&state->queue, struct sun6i_isp_buffer,
				      list);

	sun6i_isp_capture_buffer_configure(capture, isp_buffer);

	list_del(&isp_buffer->list);

	state->pending = isp_buffer;

	if (update)
		*update = true;

complete:
	spin_unlock_irqrestore(&state->lock, flags);
}

static void
sun6i_isp_capture_state_complete_one(struct sun6i_isp_capture *capture)
{
	struct sun6i_isp_capture_state *state = &capture->state;
	struct sun6i_isp_buffer *done;
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);

	if (!state->pending)
		goto complete;

	/*
	 * The hardware has taken the staged buffer, so the one it was writing
	 * before is finished and goes back to userspace.
	 */
	done = state->active;
	state->active = state->pending;
	state->pending = NULL;

	if (done) {
		struct vb2_buffer *vb2_buffer = &done->v4l2_buffer.vb2_buf;

		vb2_buffer->timestamp = ktime_get_ns();
		done->v4l2_buffer.sequence = state->sequence;

		vb2_buffer_done(vb2_buffer, VB2_BUF_STATE_DONE);
	}

complete:
	spin_unlock_irqrestore(&state->lock, flags);
}

static void sun6i_isp_capture_finish_one(struct sun6i_isp_capture *capture)
{
	struct sun6i_isp_capture_state *state = &capture->state;
	unsigned long flags;

	if (!state->streaming)
		return;

	spin_lock_irqsave(&state->lock, flags);
	state->sequence++;
	spin_unlock_irqrestore(&state->lock, flags);
}

/* True while any channel is streaming. */
bool sun6i_isp_capture_streaming(struct sun6i_isp_device *isp_dev)
{
	unsigned int i;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++)
		if (isp_dev->captures[i].state.streaming)
			return true;

	return false;
}

void sun6i_isp_capture_state_update(struct sun6i_isp_device *isp_dev,
				    bool *update)
{
	unsigned int i;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++)
		sun6i_isp_capture_state_update_one(&isp_dev->captures[i],
						   update);
}

void sun6i_isp_capture_state_complete(struct sun6i_isp_device *isp_dev)
{
	unsigned int i;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++)
		sun6i_isp_capture_state_complete_one(&isp_dev->captures[i]);
}

/* The two channels share the frame finish interrupt. */
void sun6i_isp_capture_finish(struct sun6i_isp_device *isp_dev)
{
	sun6i_isp_capture_finish_one(&isp_dev->captures[SUN6I_ISP_CAPTURE_MAIN]);
	sun6i_isp_capture_finish_one(&isp_dev->captures[SUN6I_ISP_CAPTURE_SELF]);
}

/*
 * The rotator runs after the channel it reads back, so a rotating channel's
 * buffer is only complete once the rotator has finished with it.
 *
 * Nothing waits for that. Buffers are handed back a whole frame later, at the
 * parameter load that replaces them, by which point the rotator has long since
 * finished - so the existing one-frame-behind completion already covers its
 * extra latency, and hanging completion off ROT_FINISH instead would stall
 * capture outright whenever the rotator missed a frame. The interrupt is left
 * masked for that reason.
 */

/* Queue */

static int sun6i_isp_capture_queue_setup(struct vb2_queue *queue,
					 unsigned int *buffers_count,
					 unsigned int *planes_count,
					 unsigned int sizes[],
					 struct device *alloc_devs[])
{
	struct sun6i_isp_capture *capture = vb2_get_drv_priv(queue);
	unsigned int size = capture->format.fmt.pix.sizeimage;

	if (*planes_count)
		return sizes[0] < size ? -EINVAL : 0;

	*planes_count = 1;
	sizes[0] = size;

	return 0;
}

static int sun6i_isp_capture_buffer_prepare(struct vb2_buffer *vb2_buffer)
{
	struct sun6i_isp_capture *capture =
		vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct v4l2_device *v4l2_dev = &capture->isp_dev->v4l2.v4l2_dev;
	unsigned int size = capture->format.fmt.pix.sizeimage;
	unsigned int width, height;
	u32 pixelformat;

	if (vb2_plane_size(vb2_buffer, 0) < size) {
		v4l2_err(v4l2_dev, "buffer too small (%lu < %u)\n",
			 vb2_plane_size(vb2_buffer, 0), size);
		return -EINVAL;
	}

	/*
	 * The scratch region a rotating channel needs is part of the
	 * allocation but not part of the frame, so the payload is the image
	 * userspace is meant to read, which starts at offset zero.
	 */
	sun6i_isp_capture_dimensions(capture, &width, &height);
	sun6i_isp_capture_format(capture, &pixelformat);

	vb2_set_plane_payload(vb2_buffer, 0,
			      sun6i_isp_capture_image_size(pixelformat, width,
							   height));

	return 0;
}

static void sun6i_isp_capture_buffer_queue(struct vb2_buffer *vb2_buffer)
{
	struct sun6i_isp_capture *capture =
		vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct sun6i_isp_capture_state *state = &capture->state;
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);
	struct sun6i_isp_buffer *isp_buffer =
		container_of(v4l2_buffer, struct sun6i_isp_buffer, v4l2_buffer);
	unsigned long flags;

	spin_lock_irqsave(&state->lock, flags);
	list_add_tail(&isp_buffer->list, &state->queue);
	spin_unlock_irqrestore(&state->lock, flags);

	/* Update the state to schedule our buffer as soon as possible. */
	if (state->streaming)
		sun6i_isp_state_update(capture->isp_dev, false);
}

/* Controls */

static void sun6i_isp_capture_format_prepare(struct sun6i_isp_capture *capture,
					     struct v4l2_format *format);

static int sun6i_isp_capture_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sun6i_isp_capture *capture =
		container_of(ctrl->handler, struct sun6i_isp_capture,
			     ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		capture->mirror = ctrl->val;
		return 0;
	case V4L2_CID_VFLIP:
		capture->flip = ctrl->val;
		return 0;
	case V4L2_CID_ROTATE:
		/*
		 * Unlike the flips, this changes the geometry and the size of
		 * the buffer needed, so it cannot be moved under a queue that
		 * has already been allocated and started. And there is one
		 * rotator between the two channels, so taking it from the
		 * other one is refused rather than done silently, which would
		 * leave that channel's buffers laid out for a rotation that
		 * is no longer happening.
		 */
		if (capture->state.streaming)
			return -EBUSY;

		if ((ctrl->val == 90 || ctrl->val == 270) &&
		    sun6i_isp_capture_rotation_taken(capture->isp_dev, capture))
			return -EBUSY;

		if (capture->rotation == ctrl->val)
			return 0;

		capture->rotation = ctrl->val;

		/*
		 * A quarter turn transposes the delivered image and rotation
		 * changes how much buffer is needed, so the format has to be
		 * recomputed rather than left describing the old arrangement.
		 */
		sun6i_isp_capture_format_prepare(capture, &capture->format);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops sun6i_isp_capture_ctrl_ops = {
	.s_ctrl		= sun6i_isp_capture_s_ctrl,
};

static int sun6i_isp_capture_start_streaming(struct vb2_queue *queue,
					     unsigned int count)
{
	struct sun6i_isp_capture *capture = vb2_get_drv_priv(queue);
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	struct sun6i_isp_capture_state *state = &capture->state;
	struct video_device *video_dev = &capture->video_dev;
	struct v4l2_subdev *subdev = &isp_dev->proc.subdev;
	bool first;
	int ret;

	state->sequence = 0;

	ret = video_device_pipeline_alloc_start(video_dev);
	if (ret < 0)
		goto error_state;

	mutex_lock(&isp_dev->stream_lock);

	/*
	 * All the channels share one proc sub-device, so only the first one to
	 * start may start it: .s_stream() must never be called to start an
	 * already started sub-device. Asked before this channel is marked
	 * streaming, so it means "is any other channel already running".
	 */
	first = !sun6i_isp_capture_streaming(isp_dev);

	state->streaming = true;

	if (first) {
		ret = v4l2_subdev_call(subdev, video, s_stream, 1);
		if (ret && ret != -ENOIOCTLCMD)
			goto error_streaming;
	} else {
		/*
		 * The pipeline is already running, so this channel missed the
		 * configuration pass proc's .s_stream() makes - and the channel
		 * that is already streaming may have to move to the other
		 * register block now that this one is here. Both are settled by
		 * taking the pipeline through a stop and a start.
		 */
		ret = sun6i_isp_pipeline_restart(isp_dev);
		if (ret) {
			/*
			 * The pipeline is down now, and the channels that were
			 * streaming before this one turned up would be left
			 * with nothing feeding them. Take this channel back out
			 * and bring the pipeline up around the set that was
			 * working; if even that fails there is nothing further
			 * to be done here, and they will find out when they
			 * stop.
			 */
			state->streaming = false;
			sun6i_isp_pipeline_restart(isp_dev);

			goto error_unlock;
		}
	}

	mutex_unlock(&isp_dev->stream_lock);

	return 0;

error_streaming:
	state->streaming = false;

error_unlock:
	mutex_unlock(&isp_dev->stream_lock);

	video_device_pipeline_stop(video_dev);

error_state:
	sun6i_isp_capture_state_cleanup(capture, false);

	return ret;
}

static void sun6i_isp_capture_stop_streaming(struct vb2_queue *queue)
{
	struct sun6i_isp_capture *capture = vb2_get_drv_priv(queue);
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	struct sun6i_isp_capture_state *state = &capture->state;
	struct video_device *video_dev = &capture->video_dev;
	struct v4l2_subdev *subdev = &isp_dev->proc.subdev;

	mutex_lock(&isp_dev->stream_lock);

	state->streaming = false;

	/*
	 * Mirror of the start path: only the last channel out stops the shared
	 * proc sub-device. Stopping it on the first would cut the frames off
	 * under whichever channels remain, so those get the pipeline taken down
	 * and brought back up around the new set instead - which they need
	 * anyway, since a self channel left on its own has to move onto the
	 * main block now that it is free.
	 *
	 * Either way the frontend goes down before this returns, so the channel
	 * is out of the hardware's hands by the time its buffers are handed
	 * back below. Leaving that to the load buffer would mean the hardware
	 * finishing a frame into memory that has already gone back to the
	 * allocator, which surfaces much later as a partial frame written
	 * across whatever those pages were handed to next.
	 */
	if (sun6i_isp_capture_streaming(isp_dev)) {
		if (sun6i_isp_pipeline_restart(isp_dev))
			dev_err(isp_dev->dev,
				"failed to restart the pipeline for the remaining channels\n");
	} else {
		v4l2_subdev_call(subdev, video, s_stream, 0);
	}

	mutex_unlock(&isp_dev->stream_lock);

	video_device_pipeline_stop(video_dev);

	sun6i_isp_capture_state_cleanup(capture, true);
}

static const struct vb2_ops sun6i_isp_capture_queue_ops = {
	.queue_setup		= sun6i_isp_capture_queue_setup,
	.buf_prepare		= sun6i_isp_capture_buffer_prepare,
	.buf_queue		= sun6i_isp_capture_buffer_queue,
	.start_streaming	= sun6i_isp_capture_start_streaming,
	.stop_streaming		= sun6i_isp_capture_stop_streaming,
};

/* Video Device */

static void sun6i_isp_capture_format_prepare(struct sun6i_isp_capture *capture,
					     struct v4l2_format *format)
{
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	unsigned int input_width, input_height;
	const struct v4l2_format_info *info;
	unsigned int source_width, source_height;

	if (!sun6i_isp_capture_format_find(pix_format->pixelformat))
		pix_format->pixelformat =
			sun6i_isp_capture_formats[0].pixelformat;

	/*
	 * The format describes what comes out, so on a quarter turn it is the
	 * turned image. What the scaler has to fit inside the proc output is
	 * the untuned one the channel writes, so bound that and turn the
	 * result back.
	 */

	source_width = pix_format->width;
	source_height = pix_format->height;

	if (sun6i_isp_capture_quarter_turn(capture))
		swap(source_width, source_height);

	/* The scaler only goes down, so the input bounds what can be written. */
	sun6i_isp_proc_dimensions(isp_dev, &input_width, &input_height);

	v4l_bound_align_image(&source_width, SUN6I_ISP_CAPTURE_WIDTH_MIN,
			      min(input_width, (unsigned int)SUN6I_ISP_CAPTURE_WIDTH_MAX), 1,
			      &source_height, SUN6I_ISP_CAPTURE_HEIGHT_MIN,
			      min(input_height, (unsigned int)SUN6I_ISP_CAPTURE_HEIGHT_MAX), 1, 0);

	/*
	 * The self channel is the vendor's thumbnail channel, and its scaler
	 * does not take a free ratio the way the main one does: it has three
	 * settings, full size, half and quarter. The vendor picks between them
	 * from the ratio that was asked for and then recomputes the output
	 * size from the setting it chose, rather than from the request -
	 * bsp_isp_old_set_size_internal(), which does
	 *
	 *   ratio = (requested << 16) / input;
	 *   if      (ratio <= 0x4000) mode = RATIO_1_4, scale = 0x4000;
	 *   else if (ratio <= 0x8000) mode = RATIO_1_2, scale = 0x8000;
	 *   else                      mode = ORIGINAL,  scale = 0x10000;
	 *   size = input * scale >> 16;
	 *
	 * Anything else and the channel writes at a size the buffer was not
	 * allocated for, which is the driver handing the hardware memory it
	 * does not own. Snap here instead, so that what comes back from
	 * S_FMT is a size the channel can actually produce.
	 */
	if (capture->id == SUN6I_ISP_CAPTURE_SELF) {
		unsigned int ratio = (source_width << 16) / input_width;
		unsigned int shift;

		if (ratio <= 0x4000)
			shift = 2;
		else if (ratio <= 0x8000)
			shift = 1;
		else
			shift = 0;

		source_width = input_width >> shift;
		source_height = input_height >> shift;
	}

	pix_format->width = source_width;
	pix_format->height = source_height;

	if (sun6i_isp_capture_quarter_turn(capture))
		swap(pix_format->width, pix_format->height);

	info = v4l2_format_info(pix_format->pixelformat);
	if (WARN_ON(!info))
		return;

	pix_format->bytesperline = sun6i_isp_capture_stride(info, 0,
							    pix_format->width);

	pix_format->sizeimage =
		sun6i_isp_capture_image_size(pix_format->pixelformat,
					     pix_format->width,
					     pix_format->height);

	/*
	 * With rotation on the buffer has to carry the channel's own output
	 * as well, for the rotator to read back. It goes after the rotated
	 * image, page aligned, the way the vendor lays its regions out; the
	 * two are not the same size when the width and height pad differently.
	 */
	if (sun6i_isp_capture_quarter_turn(capture))
		pix_format->sizeimage = ALIGN(pix_format->sizeimage, PAGE_SIZE) +
			sun6i_isp_capture_image_size(pix_format->pixelformat,
						     source_width,
						     source_height);

	pix_format->field = V4L2_FIELD_NONE;

	pix_format->colorspace = V4L2_COLORSPACE_RAW;
	pix_format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix_format->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix_format->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int sun6i_isp_capture_querycap(struct file *file, void *priv,
				      struct v4l2_capability *capability)
{
	struct sun6i_isp_capture *capture = video_drvdata(file);
	struct video_device *video_dev = &capture->video_dev;
	struct sun6i_isp_device *isp_dev = capture->isp_dev;

	strscpy(capability->driver, SUN6I_ISP_NAME, sizeof(capability->driver));
	strscpy(capability->card, video_dev->name, sizeof(capability->card));
	snprintf(capability->bus_info, sizeof(capability->bus_info),
		 "platform:%s", dev_name(isp_dev->dev));

	return 0;
}

static int sun6i_isp_capture_enum_fmt(struct file *file, void *priv,
				      struct v4l2_fmtdesc *fmtdesc)
{
	u32 index = fmtdesc->index;

	if (index >= ARRAY_SIZE(sun6i_isp_capture_formats))
		return -EINVAL;

	fmtdesc->pixelformat = sun6i_isp_capture_formats[index].pixelformat;

	return 0;
}

static int sun6i_isp_capture_g_fmt(struct file *file, void *priv,
				   struct v4l2_format *format)
{
	struct sun6i_isp_capture *capture = video_drvdata(file);

	*format = capture->format;

	return 0;
}

static int sun6i_isp_capture_s_fmt(struct file *file, void *priv,
				   struct v4l2_format *format)
{
	struct sun6i_isp_capture *capture = video_drvdata(file);

	if (vb2_is_busy(&capture->queue))
		return -EBUSY;

	sun6i_isp_capture_format_prepare(capture, format);

	capture->format = *format;

	return 0;
}

static int sun6i_isp_capture_try_fmt(struct file *file, void *priv,
				     struct v4l2_format *format)
{
	struct sun6i_isp_capture *capture = video_drvdata(file);

	sun6i_isp_capture_format_prepare(capture, format);

	return 0;
}

static int sun6i_isp_capture_enum_input(struct file *file, void *priv,
					struct v4l2_input *input)
{
	if (input->index != 0)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(input->name, "Camera", sizeof(input->name));

	return 0;
}

static int sun6i_isp_capture_g_input(struct file *file, void *priv,
				     unsigned int *index)
{
	*index = 0;

	return 0;
}

static int sun6i_isp_capture_s_input(struct file *file, void *priv,
				     unsigned int index)
{
	if (index != 0)
		return -EINVAL;

	return 0;
}

static const struct v4l2_ioctl_ops sun6i_isp_capture_ioctl_ops = {
	.vidioc_querycap		= sun6i_isp_capture_querycap,

	.vidioc_enum_fmt_vid_cap	= sun6i_isp_capture_enum_fmt,
	.vidioc_g_fmt_vid_cap		= sun6i_isp_capture_g_fmt,
	.vidioc_s_fmt_vid_cap		= sun6i_isp_capture_s_fmt,
	.vidioc_try_fmt_vid_cap		= sun6i_isp_capture_try_fmt,

	.vidioc_enum_input		= sun6i_isp_capture_enum_input,
	.vidioc_g_input			= sun6i_isp_capture_g_input,
	.vidioc_s_input			= sun6i_isp_capture_s_input,

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

static int sun6i_isp_capture_open(struct file *file)
{
	struct sun6i_isp_capture *capture = video_drvdata(file);
	struct video_device *video_dev = &capture->video_dev;
	struct mutex *lock = &capture->lock;
	int ret;

	if (mutex_lock_interruptible(lock))
		return -ERESTARTSYS;

	ret = v4l2_pipeline_pm_get(&video_dev->entity);
	if (ret)
		goto error_mutex;

	ret = v4l2_fh_open(file);
	if (ret)
		goto error_pipeline;

	mutex_unlock(lock);

	return 0;

error_pipeline:
	v4l2_pipeline_pm_put(&video_dev->entity);

error_mutex:
	mutex_unlock(lock);

	return ret;
}

static int sun6i_isp_capture_release(struct file *file)
{
	struct sun6i_isp_capture *capture = video_drvdata(file);
	struct video_device *video_dev = &capture->video_dev;
	struct mutex *lock = &capture->lock;

	mutex_lock(lock);

	_vb2_fop_release(file, NULL);
	v4l2_pipeline_pm_put(&video_dev->entity);

	mutex_unlock(lock);

	return 0;
}

static const struct v4l2_file_operations sun6i_isp_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= sun6i_isp_capture_open,
	.release	= sun6i_isp_capture_release,
	.unlocked_ioctl	= video_ioctl2,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
};

/* Media Entity */

static int sun6i_isp_capture_link_validate(struct media_link *link)
{
	struct video_device *video_dev =
		media_entity_to_video_device(link->sink->entity);
	struct sun6i_isp_capture *capture = video_get_drvdata(video_dev);
	struct sun6i_isp_device *isp_dev = capture->isp_dev;
	struct v4l2_device *v4l2_dev = &isp_dev->v4l2.v4l2_dev;
	unsigned int capture_width, capture_height;
	unsigned int proc_width, proc_height;

	/*
	 * What has to fit inside the proc output is what the channel writes,
	 * not what comes out of the rotator: on a quarter turn the delivered
	 * image is legitimately taller than the proc output, and checking
	 * that instead is what used to make STREAMON fail at 90 and 270.
	 */
	sun6i_isp_capture_source_dimensions(capture, &capture_width,
					    &capture_height);
	sun6i_isp_proc_dimensions(isp_dev, &proc_width, &proc_height);

	/* Each channel scales down independently, so it may only shrink. */
	if (capture_width > proc_width || capture_height > proc_height) {
		v4l2_err(v4l2_dev,
			 "invalid input/output dimensions: %ux%u/%ux%u\n",
			 proc_width, proc_height, capture_width,
			 capture_height);
		return -EINVAL;
	}

	return 0;
}

static const struct media_entity_operations sun6i_isp_capture_entity_ops = {
	.link_validate	= sun6i_isp_capture_link_validate,
};

/* Capture */

static int sun6i_isp_capture_setup_one(struct sun6i_isp_device *isp_dev,
				       enum sun6i_isp_capture_id id)
{
	struct sun6i_isp_capture *capture = &isp_dev->captures[id];
	struct sun6i_isp_capture_state *state = &capture->state;
	struct v4l2_device *v4l2_dev = &isp_dev->v4l2.v4l2_dev;
	struct v4l2_subdev *proc_subdev = &isp_dev->proc.subdev;
	struct video_device *video_dev = &capture->video_dev;
	struct vb2_queue *queue = &capture->queue;
	struct media_pad *pad = &capture->pad;
	struct v4l2_format *format = &capture->format;
	struct v4l2_pix_format *pix_format = &format->fmt.pix;
	int ret;

	/* Channel */

	capture->isp_dev = isp_dev;
	capture->id = id;

	/*
	 * No register block yet: one is handed out for as long as the channel
	 * streams, by sun6i_isp_capture_blocks_assign().
	 */
	if (id == SUN6I_ISP_CAPTURE_SELF)
		capture->name = SUN6I_ISP_CAPTURE_SELF_NAME;
	else
		capture->name = SUN6I_ISP_CAPTURE_MAIN_NAME;

	/* State */

	INIT_LIST_HEAD(&state->queue);
	spin_lock_init(&state->lock);

	/* Media Entity */

	video_dev->entity.ops = &sun6i_isp_capture_entity_ops;

	/* Media Pads */

	pad->flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;

	ret = media_entity_pads_init(&video_dev->entity, 1, pad);
	if (ret)
		goto error_mutex;

	/* Queue */

	mutex_init(&capture->lock);

	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_DMABUF;
	queue->buf_struct_size = sizeof(struct sun6i_isp_buffer);
	queue->ops = &sun6i_isp_capture_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;
	queue->min_queued_buffers = 2;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	queue->lock = &capture->lock;
	queue->dev = isp_dev->dev;
	queue->drv_priv = capture;

	ret = vb2_queue_init(queue);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to initialize vb2 queue: %d\n", ret);
		goto error_media_entity;
	}

	/* V4L2 Format */

	format->type = queue->type;
	pix_format->pixelformat = sun6i_isp_capture_formats[0].pixelformat;
	pix_format->width = 1280;
	pix_format->height = 720;

	sun6i_isp_capture_format_prepare(capture, format);

	/* V4L2 Controls */

	v4l2_ctrl_handler_init(&capture->ctrl_handler, 3);

	v4l2_ctrl_new_std(&capture->ctrl_handler, &sun6i_isp_capture_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&capture->ctrl_handler, &sun6i_isp_capture_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	/*
	 * Both channels offer this, but there is only one rotator, so setting
	 * it on one while the other is already turning is refused - see
	 * sun6i_isp_capture_s_ctrl().
	 */
	v4l2_ctrl_new_std(&capture->ctrl_handler, &sun6i_isp_capture_ctrl_ops,
			  V4L2_CID_ROTATE, 0, 270, 90, 0);

	ret = capture->ctrl_handler.error;
	if (ret) {
		v4l2_err(v4l2_dev, "failed to create controls: %d\n", ret);
		goto error_ctrl_handler;
	}

	/* Video Device */

	strscpy(video_dev->name, capture->name, sizeof(video_dev->name));
	video_dev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	video_dev->vfl_dir = VFL_DIR_RX;
	video_dev->release = video_device_release_empty;
	video_dev->fops = &sun6i_isp_capture_fops;
	video_dev->ioctl_ops = &sun6i_isp_capture_ioctl_ops;
	video_dev->v4l2_dev = v4l2_dev;
	video_dev->queue = queue;
	video_dev->lock = &capture->lock;
	video_dev->ctrl_handler = &capture->ctrl_handler;

	video_set_drvdata(video_dev, capture);

	ret = video_register_device(video_dev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_err(v4l2_dev, "failed to register video device: %d\n",
			 ret);
		goto error_media_entity;
	}

	/*
	 * Media Pad Link
	 *
	 * Enabled by default but left mutable, so that userspace can drop a
	 * channel it has no use for instead of having it validated as part
	 * of every pipeline. The sink pad stays MUST_CONNECT, which is what
	 * makes streaming a channel whose link was disabled fail cleanly.
	 */

	ret = media_create_pad_link(&proc_subdev->entity,
				    SUN6I_ISP_PROC_PAD_SOURCE,
				    &video_dev->entity, 0,
				    MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "failed to create %s:%u -> %s:%u link\n",
			 proc_subdev->entity.name, SUN6I_ISP_PROC_PAD_SOURCE,
			 video_dev->entity.name, 0);
		goto error_video_device;
	}

	return 0;

error_video_device:
	vb2_video_unregister_device(video_dev);

error_ctrl_handler:
	v4l2_ctrl_handler_free(&capture->ctrl_handler);

error_media_entity:
	media_entity_cleanup(&video_dev->entity);

error_mutex:
	mutex_destroy(&capture->lock);

	return ret;
}

static void sun6i_isp_capture_cleanup_one(struct sun6i_isp_capture *capture)
{
	struct video_device *video_dev = &capture->video_dev;

	vb2_video_unregister_device(video_dev);
	v4l2_ctrl_handler_free(&capture->ctrl_handler);
	media_entity_cleanup(&video_dev->entity);
	mutex_destroy(&capture->lock);
}

int sun6i_isp_capture_setup(struct sun6i_isp_device *isp_dev)
{
	unsigned int i;
	int ret;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++) {
		ret = sun6i_isp_capture_setup_one(isp_dev, i);
		if (ret) {
			while (i--)
				sun6i_isp_capture_cleanup_one(&isp_dev->captures[i]);

			return ret;
		}
	}

	return 0;
}

void sun6i_isp_capture_cleanup(struct sun6i_isp_device *isp_dev)
{
	unsigned int i;

	for (i = 0; i < SUN6I_ISP_CAPTURE_COUNT; i++)
		sun6i_isp_capture_cleanup_one(&isp_dev->captures[i]);
}
