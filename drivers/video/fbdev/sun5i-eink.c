/*
 * Copyright Ondrej Jirman <megi@xff.cz>
 *
 * eInk panel scanout driver for the Allwinner A13 (PocketBook Touch
 * Lux 3). The panel hangs off the LCD interface pins; DEBE scans an
 * 8bpp palette-indexed buffer and the palette translates every byte
 * into a 32-bit word of literal pin states, so the buffer content
 * fully determines all panel signals with one-dot-clock resolution.
 *
 * This driver only scans out: userspace composes complete frames
 * (control template + waveform data), uploads the palette and the
 * timing parameters, and queues frames through a V4L2 output device.
 * Queued frames are displayed exactly once each, back-to-back with
 * no gaps while the queue is non-empty; on an empty queue scanout
 * pauses at a frame boundary and resumes on the next queued buffer
 * (every frame is self-contained, so pausing is electrically safe).
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/sun5i-eink.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

// {{{ Registry defines

#define SUN4I_TCON_GCTL_REG			0x0
#define SUN4I_TCON_GCTL_TCON_ENABLE			BIT(31)
#define SUN4I_TCON_GCTL_IOMAP_MASK			BIT(0)
#define SUN4I_TCON_GCTL_IOMAP_TCON1			(1 << 0)
#define SUN4I_TCON_GCTL_IOMAP_TCON0			(0 << 0)

#define SUN4I_TCON_GINT0_REG			0x4
#define SUN4I_TCON_GINT0_VBLANK_ENABLE(pipe)		BIT(31 - (pipe))
#define SUN4I_TCON_GINT0_VBLANK_INT(pipe)		BIT(15 - (pipe))
#define SUN4I_TCON_GINT0_LINE_ENABLE(pipe)		BIT(29 - (pipe))
#define SUN4I_TCON_GINT0_LINE_INT(pipe)			BIT(13 - (pipe))

#define SUN4I_TCON_GINT1_REG			0x8

#define SUN4I_TCON0_CTL_REG			0x40
#define SUN4I_TCON0_CTL_TCON_ENABLE			BIT(31)
#define SUN4I_TCON0_CTL_IF_MASK				GENMASK(25, 24)
#define SUN4I_TCON0_CTL_IF_8080				(1 << 24)
#define SUN4I_TCON0_CTL_CLK_DELAY_MASK			GENMASK(8, 4)
#define SUN4I_TCON0_CTL_CLK_DELAY(delay)		((delay << 4) & SUN4I_TCON0_CTL_CLK_DELAY_MASK)
#define SUN4I_TCON0_CTL_SRC_SEL_MASK			GENMASK(2, 0)

#define SUN4I_TCON0_DCLK_REG			0x44
#define SUN4I_TCON0_DCLK_GATE_BIT			(31)
#define SUN4I_TCON0_DCLK_DIV_SHIFT			(0)
#define SUN4I_TCON0_DCLK_DIV_WIDTH			(7)

#define SUN4I_TCON0_BASIC0_REG			0x48
#define SUN4I_TCON0_BASIC0_X(width)			((((width) - 1) & 0xfff) << 16)
#define SUN4I_TCON0_BASIC0_Y(height)			(((height) - 1) & 0xfff)

#define SUN4I_TCON0_BASIC1_REG			0x4c
#define SUN4I_TCON0_BASIC1_H_TOTAL(total)		((((total) - 1) & 0x1fff) << 16)
#define SUN4I_TCON0_BASIC1_H_BACKPORCH(bp)		(((bp) - 1) & 0xfff)

#define SUN4I_TCON0_BASIC2_REG			0x50
#define SUN4I_TCON0_BASIC2_V_TOTAL(total)		(((total) & 0x1fff) << 16)
#define SUN4I_TCON0_BASIC2_V_BACKPORCH(bp)		(((bp) - 1) & 0xfff)

#define SUN4I_TCON0_BASIC3_REG			0x54
#define SUN4I_TCON0_BASIC3_H_SYNC(width)		((((width) - 1) & 0x7ff) << 16)
#define SUN4I_TCON0_BASIC3_V_SYNC(height)		(((height) - 1) & 0x7ff)

#define SUN4I_TCON0_HV_IF_REG			0x58

#define SUN4I_TCON0_IO_POL_REG			0x88
#define SUN4I_TCON0_IO_POL_DCLK_PHASE(phase)		((phase & 3) << 28)
#define SUN4I_TCON0_IO_POL_DE_NEGATIVE			BIT(27)
#define SUN4I_TCON0_IO_POL_DCLK_NEGATIVE		BIT(26)
#define SUN4I_TCON0_IO_POL_HSYNC_POSITIVE		BIT(25)
#define SUN4I_TCON0_IO_POL_VSYNC_POSITIVE		BIT(24)

#define SUN4I_TCON0_IO_TRI_REG			0x8c

#define SUN4I_BACKEND_MODCTL_REG		0x800
#define SUN4I_BACKEND_MODCTL_OUT_SEL			GENMASK(22, 20)
#define SUN4I_BACKEND_MODCTL_OUT_LCD0				(0 << 20)
#define SUN4I_BACKEND_MODCTL_LAY_EN(l)			BIT(8 + l)
#define SUN4I_BACKEND_MODCTL_START_CTL			BIT(1)
#define SUN4I_BACKEND_MODCTL_DEBE_EN			BIT(0)

#define SUN4I_BACKEND_BACKCOLOR_REG		0x804

#define SUN4I_BACKEND_DISSIZE_REG		0x808
#define SUN4I_BACKEND_DISSIZE(w, h)			(((((h) - 1) & 0xffff) << 16) | \
							 (((w) - 1) & 0xffff))

#define SUN4I_BACKEND_LAYSIZE_REG(l)		(0x810 + (0x4 * (l)))
#define SUN4I_BACKEND_LAYSIZE(w, h)			(((((h) - 1) & 0x1fff) << 16) | \
							 (((w) - 1) & 0x1fff))

#define SUN4I_BACKEND_LAYCOOR_REG(l)		(0x820 + (0x4 * (l)))
#define SUN4I_BACKEND_LAYCOOR(x, y)			((((u32)(y) & 0xffff) << 16) | \
							 ((u32)(x) & 0xffff))

#define SUN4I_BACKEND_LAYLINEWIDTH_REG(l)	(0x840 + (0x4 * (l)))

#define SUN4I_BACKEND_LAYFB_L32ADD_REG(l)	(0x850 + (0x4 * (l)))

#define SUN4I_BACKEND_LAYFB_H4ADD_REG		0x860
#define SUN4I_BACKEND_LAYFB_H4ADD_MSK(l)		GENMASK(3 + ((l) * 8), (l) * 8)
#define SUN4I_BACKEND_LAYFB_H4ADD(l, val)		((val) << ((l) * 8))

#define SUN4I_BACKEND_REGBUFFCTL_REG		0x870
#define SUN4I_BACKEND_REGBUFFCTL_AUTOLOAD_DIS		BIT(1)
#define SUN4I_BACKEND_REGBUFFCTL_LOADCTL		BIT(0)

#define SUN4I_BACKEND_ATTCTL_REG0(l)		(0x890 + (0x4 * (l)))
#define SUN4I_BACKEND_ATTCTL_REG0_LAY_PIPESEL(x)		((x) << 15)
#define SUN4I_BACKEND_ATTCTL_REG0_LAY_PRISEL(x)			((x) << 10)
#define SUN4I_BACKEND_ATTCTL_REG0_LAY_WORKMOD(x)		((x) << 22)

#define SUN4I_BACKEND_ATTCTL_REG1(l)		(0x8a0 + (0x4 * (l)))
#define SUN4I_BACKEND_LAY_FBFMT_8BPP				(3 << 8)
#define SUN4I_BACKEND_LAY_FBFMT_XRGB8888			(9 << 8)

/*
 * 256 x u32 pipe0 palette table, used by layers in the Index working
 * mode. NOT at 0x4400 - that's the gamma table; the layer palette
 * tables are per pipe: pipe0 0x5000-0x53ff, pipe1 0x5400-0x57ff.
 */
#define SUN4I_BACKEND_PALETTE_REG		0x5000

// }}}

enum eink_state {
	/* not streaming */
	EINK_STREAM_OFF = 0,
	/* TCON scanning, buffers flip in the frame interrupt */
	EINK_RUNNING,
	/* streaming, queue ran dry, TCON stopped at a frame boundary */
	EINK_PAUSED,
	/* a buffer arrived while paused; restart work is queued */
	EINK_RESTARTING,
	/* streamoff waits for the frame interrupt to stop the TCON */
	EINK_STOPPING,
};

struct eink_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	/* from the frame header, latched at buffer preparation */
	s32 vcom_uv;
	u32 repeat;
};

struct eink_dev {
	struct device *dev;

	// io
	struct pinctrl* pinctrl;
	struct pinctrl_state *pinctrl_active;
	struct pinctrl_state *pinctrl_idle;
	struct gpio_descs* gpios;

	// power
	struct regulator *vdd_supply;	/* panel logic (TPS65185 V3P3) */
	struct regulator *drive_supply;	/* source/gate rails (VPOS/VNEG..) */
	struct regulator *vcom_supply;
	u32 vcom_uv;			/* panel nominal VCOM magnitude */
	struct timer_list powerdown_timer;
	struct work_struct powerdown_work;
	bool powered;

	// tcon0/debe
	struct clk_bulk_data *clks;
	int num_clks;

	struct reset_control *rstc;
	struct regmap *tcon_regs;
	struct regmap *be_regs;
	int irq;

	// v4l2
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue queue;
	struct mutex fop_lock;
	struct v4l2_pix_format pix;

	// userspace-provided scanout configuration
	u32 palette[256];
	struct sun5i_eink_timing timing;

	// scanout state
	spinlock_t lock;		/* buf_list + state, taken in irq */
	struct list_head buf_list;
	enum eink_state state;
	unsigned int sequence;
	unsigned int repeats_left;	/* extra scans of the current buffer */
	wait_queue_head_t waitqueue;
	struct workqueue_struct *wq;	/* ordered */
	struct work_struct idle_work;
	struct work_struct restart_work;

	// per-frame vcom
	int vcom_cur_uv;		/* last value applied */
	int vcom_want_uv;		/* requested from the irq, eink->lock */
	struct work_struct vcom_work;
};

#define EINK_DEF_WIDTH		304	/* 1024/4 sources + 40 lead + 8 trail */
#define EINK_DEF_HEIGHT		767	/* 758 gates + 9 control rows */
#define EINK_MAX_WIDTH		4092	/* multiple of 4 */
#define EINK_MAX_HEIGHT		4095

static const struct sun5i_eink_timing eink_default_timing = {
	.sclk_hz	= 120000000,
	.dclk_div	= 6,		/* 20 MHz dot clock */
	.hbp		= 40,
	.vbp		= 1,
	.hsync		= 1,
	.vsync		= 1,
	.clk_delay	= 3,
};

// {{{ Power

/*
 * Panel power sequencing: logic supply first, then the drive rails
 * (the TPS65185 sequences the individual rails internally and waits
 * for power good), VCOM last; power down in reverse order.
 */
static int eink_set_power(struct eink_dev* eink, bool en)
{
	int ret;

	if (!en == !eink->powered)
		return 0;

	dev_dbg(eink->dev, "%sable power supplies", en ? "en" : "dis");

	if (en) {
		ret = regulator_set_voltage(eink->vcom_supply, eink->vcom_uv,
					    eink->vcom_uv);
		if (ret)
			goto err;
		eink->vcom_cur_uv = eink->vcom_uv;
		ret = regulator_enable(eink->vdd_supply);
		if (ret)
			goto err;
		ret = regulator_enable(eink->drive_supply);
		if (ret)
			goto err_vdd;
		ret = regulator_enable(eink->vcom_supply);
		if (ret)
			goto err_drive;
	} else {
		regulator_disable(eink->vcom_supply);
		regulator_disable(eink->drive_supply);
		regulator_disable(eink->vdd_supply);
	}

	eink->powered = en;
	return 0;

err_drive:
	regulator_disable(eink->drive_supply);
err_vdd:
	regulator_disable(eink->vdd_supply);
err:
	dev_err(eink->dev, "can't enable power supplies (%d)", ret);
	return ret;
}

static void eink_powerdown_work(struct work_struct *work)
{
	struct eink_dev *eink = container_of(work, struct eink_dev,
					     powerdown_work);

	mutex_lock(&eink->fop_lock);
	if (!vb2_is_streaming(&eink->queue))
		eink_set_power(eink, 0);
	mutex_unlock(&eink->fop_lock);
}

static void eink_supply_powerdown_timer(struct timer_list *t)
{
	struct eink_dev *eink = timer_container_of(eink, t, powerdown_timer);

	queue_work(eink->wq, &eink->powerdown_work);
}

// }}}
// {{{ Scanout engine

static void eink_debe_set_address(struct eink_dev *eink, dma_addr_t paddr)
{
	regmap_write(eink->be_regs, SUN4I_BACKEND_LAYFB_L32ADD_REG(0),
		     paddr << 3);
	regmap_update_bits(eink->be_regs, SUN4I_BACKEND_LAYFB_H4ADD_REG,
			   SUN4I_BACKEND_LAYFB_H4ADD_MSK(0),
			   SUN4I_BACKEND_LAYFB_H4ADD(0, paddr >> 29));
	/* transfer the write-buffered registers at the next frame edge */
	regmap_write(eink->be_regs, SUN4I_BACKEND_REGBUFFCTL_REG,
		     SUN4I_BACKEND_REGBUFFCTL_AUTOLOAD_DIS |
		     SUN4I_BACKEND_REGBUFFCTL_LOADCTL);
}

/* scanout starts past the frame header */
static dma_addr_t eink_buf_addr(struct eink_buffer *buf)
{
	return vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0) +
	       SUN5I_EINK_HDR_SIZE;
}

/*
 * Apply a frame's VCOM. Sleepable contexts only; the frame interrupt
 * defers to vcom_work instead (the value then settles within the
 * first millisecond of the ~13ms frame scan).
 */
static void eink_apply_vcom(struct eink_dev *eink, int uv)
{
	int ret;

	if (!uv || uv == eink->vcom_cur_uv)
		return;

	ret = regulator_set_voltage(eink->vcom_supply, uv, uv);
	if (ret)
		dev_err(eink->dev, "can't set vcom %duV (%d)\n", uv, ret);
	else
		eink->vcom_cur_uv = uv;
}

static void eink_vcom_work(struct work_struct *work)
{
	struct eink_dev *eink = container_of(work, struct eink_dev,
					     vcom_work);
	unsigned long flags;
	int uv;

	spin_lock_irqsave(&eink->lock, flags);
	uv = eink->vcom_want_uv;
	spin_unlock_irqrestore(&eink->lock, flags);

	eink_apply_vcom(eink, uv);
}

/*
 * Program the DEBE for the current format + palette.
 *
 * The ordering replicates the vendor epdc_hw_open() exactly: registers
 * zeroed and the palette RAM written while the backend is completely
 * disabled and register autoloading is in its enabled reset state; only
 * then backend enable, autoload disable, layer configuration, and the
 * layer enable bit dead last. CPU writes to the palette RAM in any
 * other state don't reach the table the scan pipeline resolves
 * through (they even read back differently depending on the autoload
 * state).
 */
static void eink_debe_setup(struct eink_dev *eink, struct eink_buffer *buf)
{
	struct regmap *be = eink->be_regs;
	bool pin32 = eink->pix.pixelformat == SUN5I_EINK_FMT_PIN32;
	u32 w = eink->pix.width, h = eink->pix.height;
	dma_addr_t paddr = eink_buf_addr(buf);
	unsigned int i;

	for (i = 0x800; i < 0xa00; i += 4)
		regmap_write(be, i, 0);

	if (!pin32)
		regmap_bulk_write(be, SUN4I_BACKEND_PALETTE_REG,
				  eink->palette, 256);

	regmap_write(be, SUN4I_BACKEND_MODCTL_REG,
		     SUN4I_BACKEND_MODCTL_START_CTL);
	regmap_write(be, SUN4I_BACKEND_MODCTL_REG,
		     SUN4I_BACKEND_MODCTL_START_CTL |
		     SUN4I_BACKEND_MODCTL_DEBE_EN);
	regmap_write(be, SUN4I_BACKEND_REGBUFFCTL_REG,
		     SUN4I_BACKEND_REGBUFFCTL_AUTOLOAD_DIS);

	regmap_write(be, SUN4I_BACKEND_DISSIZE_REG,
		     SUN4I_BACKEND_DISSIZE(w, h));
	regmap_write(be, SUN4I_BACKEND_LAYSIZE_REG(0),
		     SUN4I_BACKEND_LAYSIZE(w, h));
	regmap_write(be, SUN4I_BACKEND_LAYCOOR_REG(0),
		     SUN4I_BACKEND_LAYCOOR(0, 0));
	/* line width register is in bits */
	regmap_write(be, SUN4I_BACKEND_LAYLINEWIDTH_REG(0),
		     eink->pix.bytesperline * 8);

	/* For the palette format the vendor epdc value, verbatim: global
	 * alpha 0xff + enable is REQUIRED - the palette RAM stores only
	 * 24 bits, so per-pixel alpha reads as 0 and without the global
	 * override the layer blends fully transparent over the
	 * backcolor: every pin stays low and the panel never reacts.
	 * For the direct pin-word format, the original driver's proven
	 * attribute value. */
	regmap_write(be, SUN4I_BACKEND_ATTCTL_REG0(0),
		     pin32 ? SUN4I_BACKEND_ATTCTL_REG0_LAY_WORKMOD(0) |
			     SUN4I_BACKEND_ATTCTL_REG0_LAY_PIPESEL(0) |
			     SUN4I_BACKEND_ATTCTL_REG0_LAY_PRISEL(3) :
			     0xff400301);
	regmap_write(be, SUN4I_BACKEND_ATTCTL_REG1(0),
		     pin32 ? SUN4I_BACKEND_LAY_FBFMT_XRGB8888 :
			     SUN4I_BACKEND_LAY_FBFMT_8BPP);

	regmap_write(be, SUN4I_BACKEND_LAYFB_L32ADD_REG(0), paddr << 3);
	regmap_write(be, SUN4I_BACKEND_LAYFB_H4ADD_REG,
		     SUN4I_BACKEND_LAYFB_H4ADD(0, paddr >> 29));

	regmap_write(be, SUN4I_BACKEND_MODCTL_REG,
		     SUN4I_BACKEND_MODCTL_START_CTL |
		     SUN4I_BACKEND_MODCTL_DEBE_EN |
		     SUN4I_BACKEND_MODCTL_OUT_LCD0 |
		     SUN4I_BACKEND_MODCTL_LAY_EN(0));

	/* transfer everything to the active register copy */
	regmap_write(be, SUN4I_BACKEND_REGBUFFCTL_REG,
		     SUN4I_BACKEND_REGBUFFCTL_AUTOLOAD_DIS |
		     SUN4I_BACKEND_REGBUFFCTL_LOADCTL);
}

/*
 * Start the TCON scanning. The line interrupt is placed just past the
 * active area; the last rows of a well-formed frame are inert, so all
 * panel-visible data has been shifted out when it fires.
 */
static void eink_tcon_start(struct eink_dev *eink)
{
	struct regmap *tcon = eink->tcon_regs;
	const struct sun5i_eink_timing *t = &eink->timing;
	u32 w = eink->pix.width, h = eink->pix.height;
	int ret;

	regmap_write(tcon, SUN4I_TCON_GCTL_REG,
		     SUN4I_TCON_GCTL_TCON_ENABLE |
		     SUN4I_TCON_GCTL_IOMAP_TCON0);

	regmap_write(tcon, SUN4I_TCON0_BASIC0_REG,
		     SUN4I_TCON0_BASIC0_X(w) |
		     SUN4I_TCON0_BASIC0_Y(h));
	regmap_write(tcon, SUN4I_TCON0_BASIC1_REG,
		     SUN4I_TCON0_BASIC1_H_TOTAL(w + t->hbp + 2) |
		     SUN4I_TCON0_BASIC1_H_BACKPORCH(t->hbp));
	/* vt is in half-lines */
	regmap_write(tcon, SUN4I_TCON0_BASIC2_REG,
		     SUN4I_TCON0_BASIC2_V_TOTAL((h + t->vbp + 2) * 2) |
		     SUN4I_TCON0_BASIC2_V_BACKPORCH(t->vbp));
	regmap_write(tcon, SUN4I_TCON0_BASIC3_REG,
		     SUN4I_TCON0_BASIC3_H_SYNC(t->hsync) |
		     SUN4I_TCON0_BASIC3_V_SYNC(t->vsync));

	regmap_write(tcon, SUN4I_TCON0_HV_IF_REG, 0); // 24bit parallel mode
	regmap_write(tcon, SUN4I_TCON0_IO_POL_REG,
		     SUN4I_TCON0_IO_POL_DCLK_NEGATIVE);
	regmap_write(tcon, SUN4I_TCON0_IO_TRI_REG, 0);

	ret = pinctrl_select_state(eink->pinctrl, eink->pinctrl_active);
	if (ret)
		dev_err(eink->dev, "can't switch to active pinctrl state (%d)",
			ret);

	/* Interrupt once per frame, just past the active area */
	regmap_write(tcon, SUN4I_TCON_GINT0_REG, 0);
	regmap_write(tcon, SUN4I_TCON_GINT1_REG, (h + t->vbp + 2) << 16);
	regmap_write(tcon, SUN4I_TCON_GINT0_REG,
		     SUN4I_TCON_GINT0_LINE_ENABLE(0));

	regmap_write(tcon, SUN4I_TCON0_DCLK_REG,
		     BIT(SUN4I_TCON0_DCLK_GATE_BIT) |
		     (t->dclk_div << SUN4I_TCON0_DCLK_DIV_SHIFT));

	regmap_write(tcon, SUN4I_TCON0_CTL_REG,
		     SUN4I_TCON0_CTL_CLK_DELAY(t->clk_delay));
	regmap_update_bits(tcon, SUN4I_TCON0_CTL_REG,
			   SUN4I_TCON0_CTL_TCON_ENABLE,
			   SUN4I_TCON0_CTL_TCON_ENABLE);
}

/* Stop scanning. Safe in hard-irq context (regmap-mmio is fast_io). */
static void eink_tcon_stop(struct eink_dev *eink)
{
	struct regmap *tcon = eink->tcon_regs;

	regmap_update_bits(tcon, SUN4I_TCON0_CTL_REG,
			   SUN4I_TCON0_CTL_TCON_ENABLE, 0);
	regmap_write(tcon, SUN4I_TCON_GINT0_REG, 0);
	regmap_write(tcon, SUN4I_TCON_GINT1_REG, 0);
}

/* Park the pins; pinctrl_select_state may sleep, hence a work item. */
static void eink_idle_work(struct work_struct *work)
{
	struct eink_dev *eink = container_of(work, struct eink_dev, idle_work);
	int ret;

	ret = pinctrl_select_state(eink->pinctrl, eink->pinctrl_idle);
	if (ret)
		dev_err(eink->dev, "can't switch to idle pinctrl state (%d)",
			ret);

	regmap_write(eink->tcon_regs, SUN4I_TCON0_IO_TRI_REG, ~0);
}

/* A buffer was queued while scanout was paused: start a new frame. */
static void eink_restart_work(struct work_struct *work)
{
	struct eink_dev *eink = container_of(work, struct eink_dev,
					     restart_work);
	struct eink_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&eink->lock, flags);
	if (eink->state != EINK_RESTARTING || list_empty(&eink->buf_list)) {
		spin_unlock_irqrestore(&eink->lock, flags);
		return;
	}
	buf = list_first_entry(&eink->buf_list, struct eink_buffer, list);
	eink->state = EINK_RUNNING;
	eink->repeats_left = buf->repeat;
	spin_unlock_irqrestore(&eink->lock, flags);

	/* a batch start can apply its VCOM exactly, before scanning */
	eink_apply_vcom(eink, buf->vcom_uv);
	eink_debe_set_address(eink, eink_buf_addr(buf));
	eink_tcon_start(eink);
}

static irqreturn_t eink_tcon0_irq_handler(int irq, void *private)
{
	struct eink_dev *eink = private;
	struct eink_buffer *buf, *next;
	unsigned int status;

	regmap_read(eink->tcon_regs, SUN4I_TCON_GINT0_REG, &status);
	if (!(status & (SUN4I_TCON_GINT0_VBLANK_INT(0) |
			SUN4I_TCON_GINT0_LINE_INT(0))))
		return IRQ_NONE;

	regmap_update_bits(eink->tcon_regs, SUN4I_TCON_GINT0_REG,
			   SUN4I_TCON_GINT0_VBLANK_INT(0) |
			   SUN4I_TCON_GINT0_LINE_INT(0), 0);

	spin_lock(&eink->lock);

	if (eink->state == EINK_STOPPING) {
		eink_tcon_stop(eink);
		eink->state = EINK_PAUSED;
		spin_unlock(&eink->lock);
		wake_up(&eink->waitqueue);
		return IRQ_HANDLED;
	}

	if (eink->state != EINK_RUNNING || list_empty(&eink->buf_list)) {
		/* spurious - e.g. interrupt raced a stop */
		spin_unlock(&eink->lock);
		return IRQ_HANDLED;
	}

	if (eink->repeats_left) {
		/* let the current buffer scan out again */
		eink->repeats_left--;
		spin_unlock(&eink->lock);
		return IRQ_HANDLED;
	}

	buf = list_first_entry(&eink->buf_list, struct eink_buffer, list);
	list_del(&buf->list);

	if (!list_empty(&eink->buf_list)) {
		/* keep free-running into the next queued frame */
		next = list_first_entry(&eink->buf_list, struct eink_buffer,
					list);
		eink->repeats_left = next->repeat;
		if (next->vcom_uv && next->vcom_uv != eink->vcom_cur_uv) {
			eink->vcom_want_uv = next->vcom_uv;
			queue_work(eink->wq, &eink->vcom_work);
		}
		eink_debe_set_address(eink, eink_buf_addr(next));
	} else {
		/* nothing to show - stop cleanly at the frame boundary */
		eink_tcon_stop(eink);
		eink->state = EINK_PAUSED;
		queue_work(eink->wq, &eink->idle_work);
	}

	buf->vb.sequence = eink->sequence++;
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	spin_unlock(&eink->lock);
	wake_up(&eink->waitqueue);
	return IRQ_HANDLED;
}

// }}}
// {{{ vb2 queue operations

static int eink_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct eink_dev *eink = vb2_get_drv_priv(q);

	if (*nplanes)
		return sizes[0] < eink->pix.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = eink->pix.sizeimage;
	return 0;
}

static int eink_buf_prepare(struct vb2_buffer *vb)
{
	struct eink_dev *eink = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct eink_buffer *buf = container_of(vbuf, struct eink_buffer, vb);
	const struct sun5i_eink_frame_hdr *hdr;

	if (vb2_plane_size(vb, 0) < eink->pix.sizeimage) {
		dev_dbg(eink->dev, "buffer too small (%lu < %u)\n",
			vb2_plane_size(vb, 0), eink->pix.sizeimage);
		return -EINVAL;
	}

	hdr = vb2_plane_vaddr(vb, 0);
	if (!hdr) {
		dev_dbg(eink->dev, "buffer not CPU-mappable\n");
		return -EINVAL;
	}

	if (hdr->magic != SUN5I_EINK_HDR_MAGIC || hdr->version != 0 ||
	    hdr->flags != 0) {
		dev_dbg(eink->dev, "bad frame header (%08x/%u/%x)\n",
			hdr->magic, hdr->version, hdr->flags);
		return -EINVAL;
	}

	buf->vcom_uv = hdr->vcom_uv;
	buf->repeat = hdr->repeat;

	vb2_set_plane_payload(vb, 0, eink->pix.sizeimage);
	return 0;
}

static void eink_buf_queue(struct vb2_buffer *vb)
{
	struct eink_dev *eink = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct eink_buffer *buf = container_of(vbuf, struct eink_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&eink->lock, flags);
	list_add_tail(&buf->list, &eink->buf_list);
	if (eink->state == EINK_PAUSED) {
		eink->state = EINK_RESTARTING;
		queue_work(eink->wq, &eink->restart_work);
	}
	spin_unlock_irqrestore(&eink->lock, flags);
}

static void eink_return_buffers(struct eink_dev *eink,
				enum vb2_buffer_state state)
{
	struct eink_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&eink->lock, flags);
	list_for_each_entry_safe(buf, tmp, &eink->buf_list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&eink->lock, flags);
}

static int eink_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct eink_dev *eink = vb2_get_drv_priv(q);
	struct eink_buffer *buf;
	unsigned long flags;
	int ret;

	timer_delete_sync(&eink->powerdown_timer);
	ret = eink_set_power(eink, 1);
	if (ret)
		goto err_return;

	spin_lock_irqsave(&eink->lock, flags);
	buf = list_first_entry(&eink->buf_list, struct eink_buffer, list);
	eink->state = EINK_RUNNING;
	eink->sequence = 0;
	eink->repeats_left = buf->repeat;
	spin_unlock_irqrestore(&eink->lock, flags);

	eink_apply_vcom(eink, buf->vcom_uv);
	eink_debe_setup(eink, buf);
	eink_tcon_start(eink);
	return 0;

err_return:
	eink_return_buffers(eink, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void eink_stop_streaming(struct vb2_queue *q)
{
	struct eink_dev *eink = vb2_get_drv_priv(q);
	unsigned long flags;
	bool running;
	int ret;

	spin_lock_irqsave(&eink->lock, flags);
	running = eink->state == EINK_RUNNING;
	if (running)
		eink->state = EINK_STOPPING;
	spin_unlock_irqrestore(&eink->lock, flags);

	if (running) {
		/* let the current frame finish; ~13ms at defaults */
		ret = wait_event_timeout(eink->waitqueue,
					 eink->state != EINK_STOPPING,
					 msecs_to_jiffies(100));
		if (ret == 0) {
			dev_warn(eink->dev, "timeout stopping scanout\n");
			eink_tcon_stop(eink);
		}
	}

	spin_lock_irqsave(&eink->lock, flags);
	eink->state = EINK_STREAM_OFF;
	spin_unlock_irqrestore(&eink->lock, flags);

	ret = pinctrl_select_state(eink->pinctrl, eink->pinctrl_idle);
	if (ret)
		dev_err(eink->dev, "can't switch to idle pinctrl state (%d)",
			ret);
	regmap_write(eink->tcon_regs, SUN4I_TCON0_IO_TRI_REG, ~0);

	/* park the backend, as the vendor close does; the next stream
	 * start reprograms it from scratch */
	regmap_write(eink->be_regs, SUN4I_BACKEND_MODCTL_REG, 0);

	flush_workqueue(eink->wq);
	eink_return_buffers(eink, VB2_BUF_STATE_ERROR);

	mod_timer(&eink->powerdown_timer, jiffies + msecs_to_jiffies(5000));
}

static const struct vb2_ops eink_qops = {
	.queue_setup		= eink_queue_setup,
	.buf_prepare		= eink_buf_prepare,
	.buf_queue		= eink_buf_queue,
	.start_streaming	= eink_start_streaming,
	.stop_streaming		= eink_stop_streaming,
};

// }}}
// {{{ V4L2 ioctls

static int eink_querycap(struct file *file, void *fh,
			 struct v4l2_capability *cap)
{
	strscpy(cap->driver, "sun5i-eink", sizeof(cap->driver));
	strscpy(cap->card, "eInk panel", sizeof(cap->card));
	return 0;
}

static int eink_enum_fmt_vid_out(struct file *file, void *fh,
				 struct v4l2_fmtdesc *f)
{
	switch (f->index) {
	case 0:
		f->pixelformat = SUN5I_EINK_FMT_IDX8;
		return 0;
	case 1:
		f->pixelformat = SUN5I_EINK_FMT_PIN32;
		return 0;
	}
	return -EINVAL;
}

static void eink_fixup_fmt(struct v4l2_pix_format *pix)
{
	if (pix->pixelformat != SUN5I_EINK_FMT_PIN32)
		pix->pixelformat = SUN5I_EINK_FMT_IDX8;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_RAW;

	/* DEBE fetches 32-bit words; keep the row stride word-aligned */
	pix->width = clamp(round_up(pix->width, 4), 8u, (u32)EINK_MAX_WIDTH);
	pix->height = clamp(pix->height, 1u, (u32)EINK_MAX_HEIGHT);
	pix->bytesperline = pix->width *
		(pix->pixelformat == SUN5I_EINK_FMT_PIN32 ? 4 : 1);
	pix->sizeimage = SUN5I_EINK_HDR_SIZE +
			 pix->bytesperline * pix->height;
}

static int eink_try_fmt_vid_out(struct file *file, void *fh,
				struct v4l2_format *f)
{
	eink_fixup_fmt(&f->fmt.pix);
	return 0;
}

static int eink_s_fmt_vid_out(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct eink_dev *eink = video_drvdata(file);

	if (vb2_is_busy(&eink->queue))
		return -EBUSY;

	eink_fixup_fmt(&f->fmt.pix);
	eink->pix = f->fmt.pix;
	return 0;
}

static int eink_g_fmt_vid_out(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct eink_dev *eink = video_drvdata(file);

	f->fmt.pix = eink->pix;
	return 0;
}

static long eink_vidioc_default(struct file *file, void *fh, bool valid_prio,
				unsigned int cmd, void *arg)
{
	struct eink_dev *eink = video_drvdata(file);
	const struct sun5i_eink_timing *t = arg;

	switch (cmd) {
	case SUN5I_EINK_S_PALETTE:
		if (vb2_is_streaming(&eink->queue))
			return -EBUSY;
		memcpy(eink->palette, arg, sizeof(eink->palette));
		return 0;

	case SUN5I_EINK_S_TIMING:
		if (vb2_is_streaming(&eink->queue))
			return -EBUSY;
		if (t->dclk_div > 127 || t->clk_delay > 30 ||
		    t->hbp < 1 || t->hbp > 0xfff ||
		    t->vbp < 1 || t->vbp > 0xfff ||
		    t->hsync < 1 || t->hsync > 0x7ff ||
		    t->vsync < 1 || t->vsync > 0x7ff)
			return -EINVAL;
		if (t->sclk_hz) {
			int ret = clk_set_rate(eink->clks[1].clk, t->sclk_hz);
			if (ret)
				return ret;
			eink->timing.sclk_hz = t->sclk_hz;
		}
		if (t->dclk_div)
			eink->timing.dclk_div = t->dclk_div;
		eink->timing.hbp = t->hbp;
		eink->timing.vbp = t->vbp;
		eink->timing.hsync = t->hsync;
		eink->timing.vsync = t->vsync;
		eink->timing.clk_delay = t->clk_delay;
		return 0;
	}

	return -ENOTTY;
}

static const struct v4l2_ioctl_ops eink_ioctl_ops = {
	.vidioc_querycap		= eink_querycap,
	.vidioc_enum_fmt_vid_out	= eink_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out		= eink_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out		= eink_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out		= eink_try_fmt_vid_out,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_default			= eink_vidioc_default,
};

static const struct v4l2_file_operations eink_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
};

// }}}
// {{{ Probe

static struct regmap_config eink_tcon_regmap_config = {
	.name		= "tcon",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x7ff,
	.cache_type	= REGCACHE_NONE,
	.fast_io	= true,
};

static struct regmap_config eink_be_regmap_config = {
	.name		= "be",
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x57ff,
	.cache_type	= REGCACHE_NONE,
	.fast_io	= true,
};

static const struct clk_bulk_data eink_clocks[] = {
	{ .id = "tcon_bus" },
	{ .id = "tcon_mod" },
	{ .id = "be_bus" },
	{ .id = "be_mod" },
	{ .id = "be_ram" },
};

static int eink_probe(struct platform_device *pdev)
{
	struct eink_dev *eink;
	struct device *dev = &pdev->dev;
	void __iomem *tcon_regs, *be_regs;
	struct vb2_queue *q;
	int ret, i;

	eink = devm_kzalloc(dev, sizeof(*eink), GFP_KERNEL);
	if (!eink)
		return -ENOMEM;

	eink->dev = dev;
	platform_set_drvdata(pdev, eink);
	init_waitqueue_head(&eink->waitqueue);
	spin_lock_init(&eink->lock);
	mutex_init(&eink->fop_lock);
	INIT_LIST_HEAD(&eink->buf_list);
	INIT_WORK(&eink->idle_work, eink_idle_work);
	INIT_WORK(&eink->restart_work, eink_restart_work);
	INIT_WORK(&eink->powerdown_work, eink_powerdown_work);
	INIT_WORK(&eink->vcom_work, eink_vcom_work);
	timer_setup(&eink->powerdown_timer, eink_supply_powerdown_timer, 0);

	eink->timing = eink_default_timing;
	/* identity palette; userspace must upload a real pin-state table */
	for (i = 0; i < 256; i++)
		eink->palette[i] = i;

	eink->pix.width = EINK_DEF_WIDTH;
	eink->pix.height = EINK_DEF_HEIGHT;
	eink_fixup_fmt(&eink->pix);

	ret = of_dma_configure(dev, dev->of_node, true);
	if (ret) {
		dev_err(dev, "failed to configure dma (%d)\n", ret);
		return ret;
	}

	tcon_regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tcon_regs))
		return PTR_ERR(tcon_regs);

	be_regs = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(be_regs))
		return PTR_ERR(be_regs);

	eink->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(eink->pinctrl)) {
		ret = PTR_ERR(eink->pinctrl);
		dev_err(dev, "can't get pinctrl (%d)\n", ret);
		return ret;
	}

	eink->pinctrl_active = pinctrl_lookup_state(eink->pinctrl, "active");
	if (!eink->pinctrl_active) {
		dev_err(dev, "missing active pinctrl state");
		return -EINVAL;
	}

	eink->pinctrl_idle = pinctrl_lookup_state(eink->pinctrl, "idle");
	if (!eink->pinctrl_idle) {
		dev_err(dev, "missing idle pinctrl state");
		return -EINVAL;
	}

	ret = pinctrl_select_state(eink->pinctrl, eink->pinctrl_idle);
	if (ret) {
		dev_err(dev, "can't switch to idle pinctrl state (%d)", ret);
		return ret;
	}

	eink->gpios = devm_gpiod_get_array(dev, "all", GPIOD_OUT_LOW);
	if (IS_ERR(eink->gpios)) {
		ret = PTR_ERR(eink->gpios);
		dev_err(dev, "can't get all gpios (%d)\n", ret);
		return ret;
	}

	eink->irq = platform_get_irq(pdev, 0);
	if (eink->irq < 0) {
		dev_err(dev, "Couldn't retrieve the TCON interrupt\n");
		return eink->irq;
	}

	eink->vdd_supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(eink->vdd_supply)) {
		ret = PTR_ERR(eink->vdd_supply);
		dev_err(dev, "can't get vdd supply (%d)\n", ret);
		return ret;
	}

	eink->drive_supply = devm_regulator_get(dev, "drive");
	if (IS_ERR(eink->drive_supply)) {
		ret = PTR_ERR(eink->drive_supply);
		dev_err(dev, "can't get drive supply (%d)\n", ret);
		return ret;
	}

	eink->vcom_supply = devm_regulator_get(dev, "vcom");
	if (IS_ERR(eink->vcom_supply)) {
		ret = PTR_ERR(eink->vcom_supply);
		dev_err(dev, "can't get vcom supply (%d)\n", ret);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "vcom-microvolt",
				   &eink->vcom_uv);
	if (ret) {
		dev_err(dev, "missing vcom-microvolt property (%d)\n", ret);
		return ret;
	}

	eink->clks = devm_kmemdup(dev, eink_clocks, sizeof(eink_clocks),
				  GFP_KERNEL);
	if (!eink->clks)
		return -ENOMEM;

	eink->num_clks = ARRAY_SIZE(eink_clocks);
	ret = devm_clk_bulk_get(dev, eink->num_clks, eink->clks);
	if (ret)
		return ret;

	eink->rstc = devm_reset_control_array_get(dev, RESET_CONTROL_EXCLUSIVE);
	if (IS_ERR(eink->rstc)) {
		ret = PTR_ERR(eink->rstc);
		dev_err(dev, "Couldn't get our reset line (%d)\n", ret);
		return ret;
	}

	eink->tcon_regs = devm_regmap_init_mmio(dev, tcon_regs,
						&eink_tcon_regmap_config);
	if (IS_ERR(eink->tcon_regs)) {
		ret = PTR_ERR(eink->tcon_regs);
		dev_err(dev, "Couldn't create the TCON regmap (%d)\n", ret);
		return ret;
	}

	eink->be_regs = devm_regmap_init_mmio(dev, be_regs,
					      &eink_be_regmap_config);
	if (IS_ERR(eink->be_regs)) {
		ret = PTR_ERR(eink->be_regs);
		dev_err(dev, "Couldn't create the BE regmap (%d)\n", ret);
		return ret;
	}

	// init the actual hardware
	ret = reset_control_deassert(eink->rstc);
	if (ret) {
		dev_err(dev, "Couldn't deassert our reset line (%d)\n", ret);
		return ret;
	}

	// possible to set 27-381MHz in 3MHz steps
	ret = clk_set_rate(eink->clks[1].clk, eink->timing.sclk_hz);
	if (ret) {
		dev_err(dev, "Couldn't set tcon0 sclk rate (%d)\n", ret);
		goto err_reset;
	}

	ret = clk_bulk_prepare_enable(eink->num_clks, eink->clks);
	if (ret) {
		dev_err(dev, "Couldn't enable clocks (%d)\n", ret);
		goto err_reset;
	}

	/* Clear DEBE registers; the backend stays disabled with register
	 * autoloading in its enabled reset state until stream start */
	for (i = 0x800; i < 0x1000; i += 4)
		regmap_write(eink->be_regs, i, 0);

	/* Make sure the TCON is disabled and all IRQs are off */
	regmap_write(eink->tcon_regs, SUN4I_TCON_GCTL_REG, 0);
	regmap_write(eink->tcon_regs, SUN4I_TCON_GINT0_REG, 0);
	regmap_write(eink->tcon_regs, SUN4I_TCON_GINT1_REG, 0);

	/* Disable IO lines and set them to tristate */
	regmap_write(eink->tcon_regs, SUN4I_TCON0_IO_TRI_REG, ~0);

	ret = devm_request_irq(dev, eink->irq, eink_tcon0_irq_handler, 0,
			       dev_name(dev), eink);
	if (ret) {
		dev_err(dev, "Couldn't request the IRQ\n");
		goto err_disable_hw;
	}

	eink->wq = alloc_ordered_workqueue("eink", WQ_HIGHPRI);
	if (!eink->wq) {
		ret = -ENOMEM;
		goto err_disable_hw;
	}

	// v4l2 device
	ret = v4l2_device_register(dev, &eink->v4l2_dev);
	if (ret)
		goto err_free_wq;

	q = &eink->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = eink;
	q->buf_struct_size = sizeof(struct eink_buffer);
	q->ops = &eink_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &eink->fop_lock;
	q->dev = dev;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_unreg_v4l2;

	strscpy(eink->vdev.name, "sun5i-eink", sizeof(eink->vdev.name));
	eink->vdev.fops = &eink_fops;
	eink->vdev.ioctl_ops = &eink_ioctl_ops;
	eink->vdev.lock = &eink->fop_lock;
	eink->vdev.release = video_device_release_empty;
	eink->vdev.v4l2_dev = &eink->v4l2_dev;
	eink->vdev.vfl_dir = VFL_DIR_TX;
	eink->vdev.device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
	eink->vdev.queue = q;
	video_set_drvdata(&eink->vdev, eink);

	ret = video_register_device(&eink->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "can't register video device (%d)\n", ret);
		goto err_unreg_v4l2;
	}

	dev_info(dev, "eink-panel scanout driver ready (%s)\n",
		 video_device_node_name(&eink->vdev));
	return 0;

err_unreg_v4l2:
	v4l2_device_unregister(&eink->v4l2_dev);
err_free_wq:
	destroy_workqueue(eink->wq);
err_disable_hw:
	clk_bulk_disable_unprepare(eink->num_clks, eink->clks);
err_reset:
	reset_control_assert(eink->rstc);
	return ret;
}

static void eink_remove(struct platform_device *pdev)
{
	struct eink_dev *eink = platform_get_drvdata(pdev);

	video_unregister_device(&eink->vdev);
	v4l2_device_unregister(&eink->v4l2_dev);

	timer_delete_sync(&eink->powerdown_timer);
	destroy_workqueue(eink->wq);
	eink_set_power(eink, 0);

	clk_bulk_disable_unprepare(eink->num_clks, eink->clks);
	reset_control_assert(eink->rstc);
}

static const struct of_device_id eink_of_match[] = {
	{ .compatible = "custom,pocketbook-touch-lux-3-tcon0-ed060xd4-display" },
	{},
};
MODULE_DEVICE_TABLE(of, eink_of_match);

static struct platform_driver eink_platform_driver = {
	.probe = eink_probe,
	.remove = eink_remove,
	.driver = {
		.name = "eink_tcon0",
		.of_match_table = eink_of_match,
	},
};

module_platform_driver(eink_platform_driver);

MODULE_VERSION("2.0.0");
MODULE_DESCRIPTION("eInk display Allwinner TCON0 palette scanout driver");
MODULE_AUTHOR("Ondrej Jirman <megi@xff.cz>");
MODULE_LICENSE("GPL v2");

// }}}
