/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * sun5i-eink V4L2 output device - private ioctls
 *
 * The device scans out 8bpp palette-indexed frames to an eInk panel
 * through the A13 DEBE+TCON0. Every byte of a queued buffer is
 * resolved through a 256-entry table to a 32-bit word whose bits
 * drive the panel pins directly; buffer content (including all
 * control-signal rows/columns) is composed entirely in userspace.
 */
#ifndef _UAPI_SUN5I_EINK_H
#define _UAPI_SUN5I_EINK_H

#include <linux/types.h>
#include <linux/videodev2.h>

/*
 * Pixel formats: one buffer element per dot clock.
 *   EPD8: one byte, resolved to a 32-bit pin word through the
 *         palette uploaded with SUN5I_EINK_S_PALETTE
 *   EPDW: one 32-bit pin word directly (bit N drives LCD pin N)
 */
#define SUN5I_EINK_FMT_IDX8	v4l2_fourcc('E', 'P', 'D', '8')
#define SUN5I_EINK_FMT_PIN32	v4l2_fourcc('E', 'P', 'D', 'W')

/*
 * Every queued buffer starts with this header; the frame itself
 * (bytesperline * height) follows at SUN5I_EINK_HDR_SIZE. A buffer
 * with a bad magic or version is rejected at QBUF time.
 */
#define SUN5I_EINK_HDR_MAGIC	0x46445045	/* "EPDF" */
#define SUN5I_EINK_HDR_SIZE	64

struct sun5i_eink_frame_hdr {
	__u32 magic;		/* SUN5I_EINK_HDR_MAGIC */
	__u32 version;		/* 0 */
	__s32 vcom_uv;		/* VCOM magnitude while this frame scans;
				   0 = keep the current value */
	__u32 repeat;		/* scan this frame 1 + repeat times */
	__u32 flags;		/* must be 0 */
	__u32 reserved[11];	/* pads to SUN5I_EINK_HDR_SIZE */
};

struct sun5i_eink_palette {
	__u32 lut[256];		/* index -> DEBE output word (pin states) */
};

struct sun5i_eink_timing {
	__u32 sclk_hz;		/* TCON sclk rate; 0 = keep current (120 MHz) */
	__u32 dclk_div;		/* dot clock = sclk / dclk_div; 1..127, 0 = keep */
	__u32 hbp;		/* dclks from hsync start to data (incl. sync) */
	__u32 vbp;		/* lines from vsync start to data (incl. sync) */
	__u32 hsync;		/* hsync width in dclks */
	__u32 vsync;		/* vsync width in lines */
	__u32 clk_delay;	/* TCON0 data/clock delay, 0..30 */
};

#define SUN5I_EINK_S_PALETTE	_IOW('V', BASE_VIDIOC_PRIVATE + 0, \
				     struct sun5i_eink_palette)
#define SUN5I_EINK_S_TIMING	_IOW('V', BASE_VIDIOC_PRIVATE + 1, \
				     struct sun5i_eink_timing)

#endif /* _UAPI_SUN5I_EINK_H */
