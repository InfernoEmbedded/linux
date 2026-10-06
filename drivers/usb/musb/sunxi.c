// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner sun4i MUSB Glue Layer
 *
 * Copyright (C) 2015 Hans de Goede <hdegoede@redhat.com>
 *
 * Based on code from
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 */

#include <linux/clk.h>
#include <linux/dma/sun4i-dma.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/extcon.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pfn.h>
#include <linux/phy/phy-sun4i-usb.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/soc/sunxi/sunxi_sram.h>
#include <linux/usb/musb.h>
#include <linux/usb/of.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/workqueue.h>
#include "musb_core.h"
#include "musb_dma.h"

/*
 * Register offsets, note sunxi musb has a different layout then most
 * musb implementations, we translate the layout in musb_readb & friends.
 */
#define SUNXI_MUSB_POWER			0x0040
#define SUNXI_MUSB_DEVCTL			0x0041
#define SUNXI_MUSB_INDEX			0x0042
#define SUNXI_MUSB_VEND0			0x0043
#define SUNXI_MUSB_INTRTX			0x0044
#define SUNXI_MUSB_INTRRX			0x0046
#define SUNXI_MUSB_INTRTXE			0x0048
#define SUNXI_MUSB_INTRRXE			0x004a
#define SUNXI_MUSB_INTRUSB			0x004c
#define SUNXI_MUSB_INTRUSBE			0x0050
#define SUNXI_MUSB_FRAME			0x0054
#define SUNXI_MUSB_TXFIFOSZ			0x0090
#define SUNXI_MUSB_TXFIFOADD			0x0092
#define SUNXI_MUSB_RXFIFOSZ			0x0094
#define SUNXI_MUSB_RXFIFOADD			0x0096
#define SUNXI_MUSB_FADDR			0x0098
#define SUNXI_MUSB_TXFUNCADDR			0x0098
#define SUNXI_MUSB_TXHUBADDR			0x009a
#define SUNXI_MUSB_TXHUBPORT			0x009b
#define SUNXI_MUSB_RXFUNCADDR			0x009c
#define SUNXI_MUSB_RXHUBADDR			0x009e
#define SUNXI_MUSB_RXHUBPORT			0x009f
#define SUNXI_MUSB_CONFIGDATA			0x00c0

/* VEND0 bits */
#define SUNXI_MUSB_VEND0_PIO_MODE		0
#define SUNXI_MUSB_VEND0_DMA_MODE		BIT(0)

/*
 * On SoCs without the internal DMA engine (A13), VEND0 additionally
 * multiplexes which endpoint FIFO is wired to the SoC DMA engine's single
 * USB DRQ line, so only one endpoint at a time can use DMA there.
 */
#define SUNXI_MUSB_VEND0_DRQ_SEL(sel)		((sel) << 1)
#define SUNXI_MUSB_VEND0_DRQ_TX(ep)		(((ep) - 1) * 2)
#define SUNXI_MUSB_VEND0_DRQ_RX(ep)		((ep) * 2 - 1)

/* flags */
#define SUNXI_MUSB_FL_ENABLED			0
#define SUNXI_MUSB_FL_HOSTMODE			1
#define SUNXI_MUSB_FL_HOSTMODE_PEND		2
#define SUNXI_MUSB_FL_VBUS_ON			3
#define SUNXI_MUSB_FL_PHY_ON			4
#define SUNXI_MUSB_FL_HAS_SRAM			5
#define SUNXI_MUSB_FL_HAS_RESET			6
#define SUNXI_MUSB_FL_NO_CONFIGDATA		7
#define SUNXI_MUSB_FL_PHY_MODE_PEND		8
#define SUNXI_MUSB_FL_HAS_IDMA			9
#define SUNXI_MUSB_FL_HAS_DDMA			10

struct sunxi_musb_cfg {
	const struct musb_hdrc_config *hdrc_config;
	bool has_sram;
	bool has_reset;
	bool no_configdata;
	bool has_idma;
	bool has_ddma;
};

/* Our read/write methods need access and do not get passed in a musb ref :| */
static struct musb *sunxi_musb;

struct sunxi_glue {
	struct device		*dev;
	struct musb		*musb;
	struct platform_device	*musb_pdev;
	struct musb_platform_ops ops;
	void __iomem		*idma_base;
	struct dma_chan		*ddma_chan;
	struct clk		*clk;
	struct reset_control	*rst;
	struct phy		*phy;
	struct platform_device	*usb_phy;
	struct usb_phy		*xceiv;
	enum phy_mode		phy_mode;
	unsigned long		flags;
	struct work_struct	work;
	struct extcon_dev	*extcon;
	struct notifier_block	host_nb;
};

/* phy_power_on / off may sleep, so we use a workqueue  */
static void sunxi_musb_work(struct work_struct *work)
{
	struct sunxi_glue *glue = container_of(work, struct sunxi_glue, work);
	bool vbus_on, phy_on;

	if (!test_bit(SUNXI_MUSB_FL_ENABLED, &glue->flags))
		return;

	if (test_and_clear_bit(SUNXI_MUSB_FL_HOSTMODE_PEND, &glue->flags)) {
		struct musb *musb = glue->musb;
		unsigned long flags;
		u8 devctl;

		spin_lock_irqsave(&musb->lock, flags);

		devctl = readb(musb->mregs + SUNXI_MUSB_DEVCTL);
		if (test_bit(SUNXI_MUSB_FL_HOSTMODE, &glue->flags)) {
			set_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
			MUSB_HST_MODE(musb);
			devctl |= MUSB_DEVCTL_SESSION;
		} else {
			clear_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
			musb->xceiv->otg->state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
			devctl &= ~MUSB_DEVCTL_SESSION;
		}
		writeb(devctl, musb->mregs + SUNXI_MUSB_DEVCTL);

		spin_unlock_irqrestore(&musb->lock, flags);
	}

	vbus_on = test_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
	phy_on = test_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags);

	if (phy_on != vbus_on) {
		if (vbus_on) {
			phy_power_on(glue->phy);
			set_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags);
		} else {
			phy_power_off(glue->phy);
			clear_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags);
		}
	}

	if (test_and_clear_bit(SUNXI_MUSB_FL_PHY_MODE_PEND, &glue->flags))
		phy_set_mode(glue->phy, glue->phy_mode);
}

static void sunxi_musb_set_vbus(struct musb *musb, int is_on)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	if (is_on) {
		set_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
		musb->xceiv->otg->state = OTG_STATE_A_WAIT_VRISE;
	} else {
		clear_bit(SUNXI_MUSB_FL_VBUS_ON, &glue->flags);
	}

	schedule_work(&glue->work);
}

static void sunxi_musb_pre_root_reset_end(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	sun4i_usb_phy_set_squelch_detect(glue->phy, false);
}

static void sunxi_musb_post_root_reset_end(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	sun4i_usb_phy_set_squelch_detect(glue->phy, true);
}

static void sunxi_musb_dma_irq(struct musb *musb);
static void sunxi_musb_idma_reset(struct musb *musb);

static irqreturn_t sunxi_musb_interrupt(int irq, void *__hci)
{
	struct musb *musb = __hci;
	unsigned long flags;

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = readb(musb->mregs + SUNXI_MUSB_INTRUSB);
	if (musb->int_usb)
		writeb(musb->int_usb, musb->mregs + SUNXI_MUSB_INTRUSB);
	musb->int_usb &= ~MUSB_INTR_SOF;

	if ((musb->int_usb & MUSB_INTR_RESET) && !is_host_active(musb)) {
		/* ep0 FADDR must be 0 when (re)entering peripheral mode */
		musb_ep_select(musb->mregs, 0);
		musb_writeb(musb->mregs, MUSB_FADDR, 0);
	}

	musb->int_tx = readw(musb->mregs + SUNXI_MUSB_INTRTX);
	if (musb->int_tx)
		writew(musb->int_tx, musb->mregs + SUNXI_MUSB_INTRTX);

	musb->int_rx = readw(musb->mregs + SUNXI_MUSB_INTRRX);
	if (musb->int_rx)
		writew(musb->int_rx, musb->mregs + SUNXI_MUSB_INTRRX);

	sunxi_musb_dma_irq(musb);

	musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return IRQ_HANDLED;
}

static int sunxi_musb_host_notifier(struct notifier_block *nb,
				    unsigned long event, void *ptr)
{
	struct sunxi_glue *glue = container_of(nb, struct sunxi_glue, host_nb);

	if (event)
		set_bit(SUNXI_MUSB_FL_HOSTMODE, &glue->flags);
	else
		clear_bit(SUNXI_MUSB_FL_HOSTMODE, &glue->flags);

	set_bit(SUNXI_MUSB_FL_HOSTMODE_PEND, &glue->flags);
	schedule_work(&glue->work);

	return NOTIFY_DONE;
}

static int sunxi_musb_init(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	int ret;

	sunxi_musb = musb;
	musb->phy = glue->phy;
	musb->xceiv = glue->xceiv;

	if (test_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags)) {
		ret = sunxi_sram_claim(musb->controller->parent);
		if (ret)
			return ret;
	}

	ret = clk_prepare_enable(glue->clk);
	if (ret)
		goto error_sram_release;

	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags)) {
		ret = reset_control_deassert(glue->rst);
		if (ret)
			goto error_clk_disable;
	}

	sunxi_musb_idma_reset(musb);

	writeb(SUNXI_MUSB_VEND0_PIO_MODE, musb->mregs + SUNXI_MUSB_VEND0);

	/* Register notifier before calling phy_init() */
	ret = devm_extcon_register_notifier(glue->dev, glue->extcon,
					EXTCON_USB_HOST, &glue->host_nb);
	if (ret)
		goto error_reset_assert;

	ret = phy_init(glue->phy);
	if (ret)
		goto error_reset_assert;

	musb->isr = sunxi_musb_interrupt;

	/* Stop the musb-core from doing runtime pm (not supported on sunxi) */
	pm_runtime_get(musb->controller);

	return 0;

error_reset_assert:
	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags))
		reset_control_assert(glue->rst);
error_clk_disable:
	clk_disable_unprepare(glue->clk);
error_sram_release:
	if (test_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags))
		sunxi_sram_release(musb->controller->parent);
	return ret;
}

static int sunxi_musb_exit(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	pm_runtime_put(musb->controller);

	cancel_work_sync(&glue->work);
	if (test_bit(SUNXI_MUSB_FL_PHY_ON, &glue->flags))
		phy_power_off(glue->phy);

	phy_exit(glue->phy);

	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags))
		reset_control_assert(glue->rst);

	clk_disable_unprepare(glue->clk);
	if (test_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags))
		sunxi_sram_release(musb->controller->parent);

	return 0;
}

static void sunxi_musb_enable(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	glue->musb = musb;

	/* musb_core does not call us in a balanced manner */
	if (test_and_set_bit(SUNXI_MUSB_FL_ENABLED, &glue->flags))
		return;

	schedule_work(&glue->work);
}

static void sunxi_musb_disable(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	clear_bit(SUNXI_MUSB_FL_ENABLED, &glue->flags);
}

#if IS_ENABLED(CONFIG_USB_SUNXI_IDMA)
/* ------------------------------------------------------------------ *
 * Allwinner USB-OTG inner-DMA (idma) controller
 *
 * Unlike the Mentor HS DMA block (musbhsdma.c), this is a vendor
 * extension integrated into the USB controller's own register window,
 * with its own interrupt status that is delivered through the shared USB
 * interrupt. A channel is pointed at an endpoint's FIFO and an SDRAM
 * address + byte count, and moves whole max-packet-sized packets
 * autonomously. (Vendor "SW_UDC_DMA_INNER".)
 *
 * The code below is modeled on musbhsdma.c, completing from the shared
 * USB IRQ (sunxi_musb_dma_irq, below) since the idma has no separate irq
 * line. The gadget core sets the standard RX/TX CSR DMA bits (DMAENAB
 * etc.) around channel_program(); we only drive the idma engine and
 * report completion via musb_dma_completion().
 *
 * The A83T, H3/H5 and A64 have this engine;
 * sunxi_musb_dma_controller_create() refuses to build a controller
 * unless the matched SoC declares it, so every other sunxi SoC stays on
 * PIO with a NULL dma_controller.
 * ------------------------------------------------------------------ */

/* Offsets into the "dma" register region, not the core's. */
#define SUNXI_MUSB_DMA_INTE			0x00	/* per-channel IRQ enable */
#define SUNXI_MUSB_DMA_INTS			0x04	/* per-channel status (W1C) */
#define SUNXI_MUSB_DMA_CHAN_CFG(n)		(0x40 + 0x10 * (n))
#define SUNXI_MUSB_DMA_SDRAM_ADDR(n)		(0x44 + 0x10 * (n))
#define SUNXI_MUSB_DMA_BC(n)			(0x48 + 0x10 * (n)) /* residual on read */

/* DMA_CHAN_CFG fields */
#define SUNXI_DMA_CFG_EP(n)			((n) & 0xf)	/* bits 0-3 */
#define SUNXI_DMA_CFG_DIR_RX			BIT(4)		/* FIFO -> SDRAM */
#define SUNXI_DMA_CFG_BST(len)			(((len) & 0x7ff) << 16)
#define SUNXI_DMA_CFG_START			BIT(31)

#define SUNXI_MUSB_DMA_CHANNELS			5

/*
 * The idma byte counter is narrow; a single ~58 KiB transfer works but
 * 1 MiB hangs the engine, so cap each programmed transfer well under that
 * (a multiple of the 512-byte max packet) and let the gadget re-issue.
 */
#define SUNXI_MUSB_DMA_MAX_LEN			0x8000

/*
 * Return the idma to the state musb_platform_init() expects, discarding
 * anything the bootloader left running. No-op unless we have the engine
 * and a region to reach it through.
 */
static void sunxi_musb_idma_reset(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	if (!test_bit(SUNXI_MUSB_FL_HAS_IDMA, &glue->flags))
		return;

	writel(0, glue->idma_base + SUNXI_MUSB_DMA_INTE);
	writel(0x1f, glue->idma_base + SUNXI_MUSB_DMA_INTS);
}

struct sunxi_dma_channel {
	struct dma_channel		channel;
	struct sunxi_dma_controller	*controller;
	dma_addr_t			start_addr;
	u32				len;
	u16				max_packet_sz;
	u8				idx;		/* idma hw channel */
	u8				epnum;
	u8				transmit;
};

struct sunxi_dma_controller {
	struct dma_controller		controller;
	struct sunxi_dma_channel	channel[SUNXI_MUSB_DMA_CHANNELS];
	struct musb			*musb;
	void __iomem			*base;		/* musb core regs */
	void __iomem			*idma;		/* idma regs */
	u8				used_channels;	/* bitmask */
};

static struct dma_channel *
sunxi_dma_channel_allocate(struct dma_controller *c, struct musb_hw_ep *hw_ep,
			   u8 transmit)
{
	struct sunxi_dma_controller *controller =
		container_of(c, struct sunxi_dma_controller, controller);
	struct sunxi_dma_channel *sunxi_channel;
	u8 bit;

	for (bit = 0; bit < SUNXI_MUSB_DMA_CHANNELS; bit++) {
		if (controller->used_channels & (1 << bit))
			continue;

		controller->used_channels |= (1 << bit);
		sunxi_channel = &controller->channel[bit];
		sunxi_channel->controller = controller;
		sunxi_channel->idx = bit;
		sunxi_channel->epnum = hw_ep->epnum;
		sunxi_channel->transmit = transmit;
		sunxi_channel->channel.private_data = sunxi_channel;
		sunxi_channel->channel.status = MUSB_DMA_STATUS_FREE;
		sunxi_channel->channel.max_len = SUNXI_MUSB_DMA_MAX_LEN;
		sunxi_channel->channel.desired_mode = transmit;
		sunxi_channel->channel.actual_len = 0;

		return &sunxi_channel->channel;
	}

	return NULL;
}

static void sunxi_dma_channel_release(struct dma_channel *channel)
{
	struct sunxi_dma_channel *sunxi_channel = channel->private_data;

	channel->actual_len = 0;
	sunxi_channel->start_addr = 0;
	sunxi_channel->len = 0;
	sunxi_channel->controller->used_channels &= ~(1 << sunxi_channel->idx);
	channel->status = MUSB_DMA_STATUS_UNKNOWN;
}

static int sunxi_dma_channel_program(struct dma_channel *channel,
				     u16 packet_sz, u8 mode,
				     dma_addr_t dma_addr, u32 len)
{
	struct sunxi_dma_channel *sunxi_channel = channel->private_data;
	struct sunxi_dma_controller *controller = sunxi_channel->controller;
	void __iomem *base = controller->base;
	void __iomem *idma = controller->idma;
	u8 idx = sunxi_channel->idx;
	u32 cfg;

	BUG_ON(channel->status == MUSB_DMA_STATUS_UNKNOWN ||
	       channel->status == MUSB_DMA_STATUS_BUSY);

	/* The idma moves whole packets only; a short tail is left to PIO. */
	len -= len % packet_sz;
	if (!len)
		return false;

	channel->actual_len = 0;
	sunxi_channel->start_addr = dma_addr;
	sunxi_channel->len = len;
	sunxi_channel->max_packet_sz = packet_sz;
	channel->status = MUSB_DMA_STATUS_BUSY;

	cfg = SUNXI_DMA_CFG_EP(sunxi_channel->epnum) |
	      SUNXI_DMA_CFG_BST(packet_sz);
	if (!sunxi_channel->transmit)
		cfg |= SUNXI_DMA_CFG_DIR_RX;

	dev_dbg(controller->musb->controller,
		"idma%u ep%u %s start %u bytes @%pad pkt %u mode %u\n",
		idx, sunxi_channel->epnum,
		sunxi_channel->transmit ? "tx" : "rx",
		len, &dma_addr, packet_sz, mode);

	writeb(SUNXI_MUSB_VEND0_DMA_MODE, base + SUNXI_MUSB_VEND0);

	/* enable this channel's completion status bit */
	writel(readl(idma + SUNXI_MUSB_DMA_INTE) | (1 << idx),
	       idma + SUNXI_MUSB_DMA_INTE);
	writel(1 << idx, idma + SUNXI_MUSB_DMA_INTS);	/* clear stale */

	writel(cfg, idma + SUNXI_MUSB_DMA_CHAN_CFG(idx));
	writel(lower_32_bits(dma_addr), idma + SUNXI_MUSB_DMA_SDRAM_ADDR(idx));
	writel(len, idma + SUNXI_MUSB_DMA_BC(idx));

	/*
	 * For TX the idma START bit kicks off the transfer immediately,
	 * so the TXCSR DMA bits must already be set or the engine sees no
	 * FIFO DMA request and completes 0 bytes.  txstate also writes
	 * these bits *after* channel_program returns, but that is far too
	 * late (and would only matter once the idma fills the first
	 * packet, which is microseconds away).  Program the full vendor
	 * Tx config up front: AUTOSET | MODE | DMAENAB (+ DMAMODE for the
	 * multi-packet mode-1 path, which is the only one we DMA -- a
	 * sub-packet Tx is rejected by is_compatible and stays PIO).
	 */
	if (sunxi_channel->transmit) {
		struct musb *musb = controller->musb;
		void __iomem *epio;
		u16 txcsr;

		musb->io.ep_select(base, sunxi_channel->epnum);
		epio = base + musb->io.ep_offset(sunxi_channel->epnum, 0);
		txcsr = musb_readw(epio, MUSB_TXCSR);
		txcsr |= MUSB_TXCSR_DMAENAB | MUSB_TXCSR_MODE;
		if (mode)
			txcsr |= MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAMODE;
		musb_writew(epio, MUSB_TXCSR, txcsr);
	}

	writel(cfg | SUNXI_DMA_CFG_START, idma + SUNXI_MUSB_DMA_CHAN_CFG(idx));

	return true;
}

/*
 * Reject anything we won't actually DMA, so the gadget core keeps it
 * unmapped and on PIO and never toggles the endpoint's DMA CSR bits.
 *
 * The sunxi idma handles whole packets only and cannot detect short
 * packets (e.g. a 31-byte CBW delivered into a 512-byte buffer).  Such
 * short completions must stay on PIO to avoid a deadlock where the idma
 * waits for the remaining bytes while the host waits for the CSW.
 *
 * Only transfers larger than one maxpacket are DMA-worthy; single-packet
 * requests (CBWs, small control transfers, short SCSI status) use PIO.
 */
static int sunxi_dma_is_compatible(struct dma_channel *channel, u16 maxpacket,
				   void *buf, u32 length)
{
	if (length <= maxpacket)
		return false;
	return true;
}

static int sunxi_dma_channel_abort(struct dma_channel *channel)
{
	struct sunxi_dma_channel *sunxi_channel = channel->private_data;
	struct sunxi_dma_controller *controller = sunxi_channel->controller;
	void __iomem *base = controller->base;
	void __iomem *idma = controller->idma;
	struct musb *musb = controller->musb;
	u8 idx = sunxi_channel->idx;
	void __iomem *epio;
	u16 csr;

	if (channel->status == MUSB_DMA_STATUS_BUSY) {
		/*
		 * Halt the engine before touching the endpoint's DMA CSR
		 * bits; clearing DMAENAB under a mid-burst engine desyncs
		 * the FIFO datapath for good.
		 */
		writel(0, idma + SUNXI_MUSB_DMA_CHAN_CFG(idx));
		writel(readl(idma + SUNXI_MUSB_DMA_INTE) & ~(1 << idx),
		       idma + SUNXI_MUSB_DMA_INTE);
		writel(1 << idx, idma + SUNXI_MUSB_DMA_INTS);	/* W1C any pending */

		musb_ep_select(base, sunxi_channel->epnum);
		epio = base + musb->io.ep_offset(sunxi_channel->epnum, 0);

		if (sunxi_channel->transmit) {
			csr = musb_readw(epio, MUSB_TXCSR);
			csr &= ~(MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAENAB);
			musb_writew(epio, MUSB_TXCSR, csr);
			csr &= ~MUSB_TXCSR_DMAMODE;
			musb_writew(epio, MUSB_TXCSR, csr);
		} else {
			csr = musb_readw(epio, MUSB_RXCSR);
			csr &= ~(MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAENAB |
				 MUSB_RXCSR_DMAMODE);
			musb_writew(epio, MUSB_RXCSR, csr);
		}

		/*
		 * The engine has no readable progress (BC and SDRAM_ADDR
		 * read back their programmed values); aborts only happen on
		 * teardown, where the moved count no longer matters.
		 */
		channel->actual_len = 0;
		channel->status = MUSB_DMA_STATUS_FREE;

		/*
		 * The completion IRQ normally restores the global VEND0
		 * PIO/DMA select; do it here too, or aborting the only busy
		 * channel leaves all PIO FIFO access (including ep0)
		 * broken until another DMA transfer completes.
		 */
		for (idx = 0; idx < SUNXI_MUSB_DMA_CHANNELS; idx++)
			if (controller->channel[idx].channel.status ==
			    MUSB_DMA_STATUS_BUSY)
				return 0;

		writeb(SUNXI_MUSB_VEND0_PIO_MODE, base + SUNXI_MUSB_VEND0);
	}

	return 0;
}

/*
 * Called from the USB interrupt (sunxi_musb_interrupt) with musb->lock
 * held. Completes any idma channels whose status bit is set.
 */
static void sunxi_musb_dma_irq(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	struct dma_controller *c = musb->dma_controller;
	struct sunxi_dma_controller *controller;
	void __iomem *base = musb->mregs;
	void __iomem *idma;
	u32 ints;
	u8 idx;

	/* The ddma engine completes through its own callback instead. */
	if (!c || !test_bit(SUNXI_MUSB_FL_HAS_IDMA, &glue->flags))
		return;

	controller = container_of(c, struct sunxi_dma_controller, controller);
	idma = controller->idma;

	ints = readl(idma + SUNXI_MUSB_DMA_INTS);
	if (!ints)
		return;
	writel(ints, idma + SUNXI_MUSB_DMA_INTS);	/* W1C */

	for (idx = 0; idx < SUNXI_MUSB_DMA_CHANNELS; idx++) {
		struct sunxi_dma_channel *sunxi_channel;
		struct dma_channel *channel;

		if (!(ints & (1 << idx)))
			continue;

		sunxi_channel = &controller->channel[idx];
		channel = &sunxi_channel->channel;
		if (channel->status != MUSB_DMA_STATUS_BUSY)
			continue;

		/*
		 * The idma raises its completion interrupt only once the
		 * whole programmed (whole-packet) transfer is done, so the
		 * full programmed length was moved.  DMA_BC is NOT a residual
		 * counter on this controller -- after completion it reads back
		 * the programmed byte count, so "len - residual" yields 0 and
		 * wedges the transfer.  Match the vendor driver and p-boot:
		 * report the programmed length.  (The short, sub-packet tail
		 * is rounded off in channel_program and handled by PIO.)
		 */
		channel->actual_len = sunxi_channel->len;
		channel->status = MUSB_DMA_STATUS_FREE;
		writel(0, idma + SUNXI_MUSB_DMA_CHAN_CFG(idx));

		dev_dbg(musb->controller,
			"idma%u ep%u %s done %zu bytes (BC reads %u)\n",
			idx, sunxi_channel->epnum,
			sunxi_channel->transmit ? "tx" : "rx",
			channel->actual_len,
			readl(idma + SUNXI_MUSB_DMA_BC(idx)));

		/*
		 * Tx: the idma raises completion as soon as the last packet
		 * has been written *into* the Tx FIFO -- the host may not have
		 * collected it yet.  musb_g_tx() clears TxPktRdy on completion,
		 * which would drop that in-flight packet, so wait for the FIFO
		 * to drain first (TxPktRdy and FIFO-not-empty both clear), as
		 * the vendor driver does.  Normally the host has already pulled
		 * it and this falls straight through; the bound stops a stalled
		 * or aborted host from wedging the IRQ.
		 *
		 * AUTOSET set TxPktRdy for every whole packet (including the
		 * last, since channel_program rounds to whole packets), so no
		 * manual last-packet hand-off is needed; a sub-packet tail is
		 * left to PIO by txstate.
		 */
		if (sunxi_channel->transmit) {
			void __iomem *txcsr;
			u16 v;
			int ret;

			/* point the indexed EP window at this endpoint */
			musb_ep_select(base, sunxi_channel->epnum);
			txcsr = base +
				musb->io.ep_offset(sunxi_channel->epnum, 0) +
				MUSB_TXCSR;

			/*
			 * Poll TXCSR until the last packet has actually left
			 * the FIFO.  Time-bounded (1 us poll, 1 ms cap) so a
			 * stalled host can't spin the hardirq; a healthy host
			 * has already drained it and this returns immediately.
			 */
			ret = readw_poll_timeout_atomic(txcsr, v,
				!(v & (MUSB_TXCSR_TXPKTRDY |
				       MUSB_TXCSR_FIFONOTEMPTY)),
				1, 1000);
			if (ret)
				dev_warn(musb->controller,
					 "idma ch%d ep%d Tx FIFO drain timeout\n",
					 idx, sunxi_channel->epnum);
		}

		musb_dma_completion(musb, sunxi_channel->epnum,
				    sunxi_channel->transmit);
	}

	/*
	 * VEND0 is the global PIO/DMA select for the FIFO data path. Hand it
	 * back to PIO once nothing is in flight, so interleaved PIO traffic
	 * (short packets, mass-storage CBW/CSW, ep0) works. musb_dma_completion
	 * above may have re-armed a channel (multi-chunk transfer), so only
	 * restore when no channel is still busy.
	 */
	for (idx = 0; idx < SUNXI_MUSB_DMA_CHANNELS; idx++)
		if (controller->channel[idx].channel.status ==
		    MUSB_DMA_STATUS_BUSY)
			return;

	writeb(SUNXI_MUSB_VEND0_PIO_MODE, base + SUNXI_MUSB_VEND0);
}

static struct dma_controller *
sunxi_idma_controller_create(struct musb *musb, void __iomem *base)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	struct sunxi_dma_controller *controller;

	controller = kzalloc(sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return NULL;

	controller->musb = musb;
	controller->base = base;
	controller->idma = glue->idma_base;
	controller->controller.musb = musb;
	controller->controller.channel_alloc = sunxi_dma_channel_allocate;
	controller->controller.channel_release = sunxi_dma_channel_release;
	controller->controller.channel_program = sunxi_dma_channel_program;
	controller->controller.channel_abort = sunxi_dma_channel_abort;
	controller->controller.is_compatible = sunxi_dma_is_compatible;

	return &controller->controller;
}

static void sunxi_idma_controller_destroy(struct dma_controller *c)
{
	struct sunxi_dma_controller *controller;

	controller = container_of(c, struct sunxi_dma_controller, controller);
	writel(0, controller->idma + SUNXI_MUSB_DMA_INTE);
	kfree(controller);
}

#else /* !CONFIG_USB_SUNXI_IDMA: idma compiled out */

static void sunxi_musb_idma_reset(struct musb *musb) { }
static void sunxi_musb_dma_irq(struct musb *musb) { }

static struct dma_controller *
sunxi_idma_controller_create(struct musb *musb, void __iomem *base)
{
	return NULL;
}

static void sunxi_idma_controller_destroy(struct dma_controller *c) { }

#endif /* CONFIG_USB_SUNXI_IDMA */

#if IS_ENABLED(CONFIG_USB_SUNXI_DDMA)
/* ------------------------------------------------------------------ *
 * Allwinner USB-OTG system DMA (ddma) support
 *
 * SoCs predating the idma (A13) have no DMA engine in the USB block at
 * all; transfers go through the SoC's dedicated DMA engine
 * (drivers/dma/sun4i-dma.c) instead, which handshakes with the USB
 * controller over a single USB DRQ line.  On these SoCs VEND0 both
 * selects the DMA FIFO bus and muxes which endpoint's FIFO is wired to
 * that DRQ line, so unlike the idma only one transfer can be in flight
 * at a time; a concurrent channel_program() is refused and that
 * transfer falls back to PIO.
 *
 * The gadget/host core sets the standard RX/TX CSR DMA bits around
 * channel_program() through the same software-mode-select paths the
 * idma uses (mode 1 for multi-packet TX and exact-length RX, per-packet
 * mode 0 otherwise), and the engine moves whole packets only, with
 * short tails left to PIO.  Completion arrives through the dmaengine
 * descriptor callback in tasklet context, outside the USB interrupt, so
 * musb->lock must be taken explicitly (modeled on ux500_dma.c).
 * ------------------------------------------------------------------ */

#define SUNXI_MUSB_DDMA_CHANNELS	5

struct sunxi_ddma_controller;

struct sunxi_ddma_channel {
	struct dma_channel		channel;
	struct sunxi_ddma_controller	*controller;
	struct sun4i_dma_chan_config	para;
	u32				len;
	u8				epnum;
	u8				transmit;
	bool				allocated;
};

struct sunxi_ddma_controller {
	struct dma_controller		controller;
	struct sunxi_ddma_channel	channel[2][SUNXI_MUSB_DDMA_CHANNELS];
	struct sunxi_ddma_channel	*active;	/* owns the VEND0 mux */
	struct musb			*musb;
	struct dma_chan			*chan;
	void __iomem			*base;		/* musb core regs */
	dma_addr_t			fifo_base;	/* the same, physical */
};

/*
 * Logical per-endpoint/direction channels; the single engine channel is
 * only claimed transfer-by-transfer in channel_program(), since the core
 * holds allocated channels for as long as an endpoint is in use and
 * refusing here would deny DMA to all but one endpoint permanently.
 */
static struct dma_channel *
sunxi_ddma_channel_allocate(struct dma_controller *c, struct musb_hw_ep *hw_ep,
			    u8 transmit)
{
	struct sunxi_ddma_controller *controller =
		container_of(c, struct sunxi_ddma_controller, controller);
	struct sunxi_ddma_channel *sunxi_channel;

	if (hw_ep->epnum == 0 || hw_ep->epnum > SUNXI_MUSB_DDMA_CHANNELS)
		return NULL;

	sunxi_channel = &controller->channel[!!transmit][hw_ep->epnum - 1];
	if (sunxi_channel->allocated)
		return NULL;

	sunxi_channel->allocated = true;
	sunxi_channel->controller = controller;
	sunxi_channel->epnum = hw_ep->epnum;
	sunxi_channel->transmit = transmit;
	sunxi_channel->channel.private_data = sunxi_channel;
	sunxi_channel->channel.status = MUSB_DMA_STATUS_FREE;
	sunxi_channel->channel.max_len = SZ_128K;
	sunxi_channel->channel.desired_mode = transmit;
	sunxi_channel->channel.actual_len = 0;

	return &sunxi_channel->channel;
}

static void sunxi_ddma_channel_release(struct dma_channel *channel)
{
	struct sunxi_ddma_channel *sunxi_channel = channel->private_data;

	channel->actual_len = 0;
	sunxi_channel->len = 0;
	sunxi_channel->allocated = false;
	channel->status = MUSB_DMA_STATUS_UNKNOWN;
}

/* Same policy as the idma: single-packet and sub-packet transfers stay PIO. */
static int sunxi_ddma_is_compatible(struct dma_channel *channel, u16 maxpacket,
				    void *buf, u32 length)
{
	return length > maxpacket;
}

static void sunxi_ddma_callback(void *param)
{
	struct sunxi_ddma_channel *sunxi_channel = param;
	struct sunxi_ddma_controller *controller = sunxi_channel->controller;
	struct dma_channel *channel = &sunxi_channel->channel;
	struct musb *musb = controller->musb;
	void __iomem *base = controller->base;
	unsigned long flags;

	spin_lock_irqsave(&musb->lock, flags);

	/* Raced with channel_abort(), which did the cleanup already. */
	if (controller->active != sunxi_channel ||
	    channel->status != MUSB_DMA_STATUS_BUSY) {
		spin_unlock_irqrestore(&musb->lock, flags);
		return;
	}

	controller->active = NULL;

	/* Hand the FIFO bus back to PIO before the core touches the FIFOs. */
	writeb(SUNXI_MUSB_VEND0_PIO_MODE, base + SUNXI_MUSB_VEND0);

	/*
	 * Tx: the engine is done once the last packet is written *into* the
	 * FIFO; wait for the host to collect it before completing, or
	 * musb_g_tx() clearing TxPktRdy would drop it (see the idma
	 * completion path for the full story).  Time-bounded so a stalled
	 * host can't wedge the tasklet.
	 */
	if (sunxi_channel->transmit) {
		void __iomem *txcsr;
		u16 v;
		int ret;

		musb->io.ep_select(base, sunxi_channel->epnum);
		txcsr = base + musb->io.ep_offset(sunxi_channel->epnum, 0) +
			MUSB_TXCSR;

		ret = readw_poll_timeout_atomic(txcsr, v,
			!(v & (MUSB_TXCSR_TXPKTRDY |
			       MUSB_TXCSR_FIFONOTEMPTY)),
			1, 1000);
		if (ret)
			dev_warn(musb->controller,
				 "ddma ep%d Tx FIFO drain timeout\n",
				 sunxi_channel->epnum);
	}

	/*
	 * The armed span is whole packets and the engine has consumed the
	 * whole byte count once it signals completion, so the programmed
	 * length was moved.
	 */
	channel->actual_len = sunxi_channel->len;
	channel->status = MUSB_DMA_STATUS_FREE;

	dev_dbg(musb->controller, "ddma ep%u %s done %zu bytes\n",
		sunxi_channel->epnum, sunxi_channel->transmit ? "tx" : "rx",
		channel->actual_len);

	musb_dma_completion(musb, sunxi_channel->epnum,
			    sunxi_channel->transmit);

	spin_unlock_irqrestore(&musb->lock, flags);
}

static int sunxi_ddma_channel_program(struct dma_channel *channel,
				      u16 packet_sz, u8 mode,
				      dma_addr_t dma_addr, u32 len)
{
	struct sunxi_ddma_channel *sunxi_channel = channel->private_data;
	struct sunxi_ddma_controller *controller = sunxi_channel->controller;
	struct dma_async_tx_descriptor *desc;
	struct dma_slave_config cfg = { };
	enum dma_transfer_direction dir;
	enum dma_slave_buswidth mem_width;
	dma_addr_t fifo_addr;
	struct scatterlist sg;
	u32 mem_burst;
	u8 drq_sel;

	BUG_ON(channel->status == MUSB_DMA_STATUS_UNKNOWN ||
	       channel->status == MUSB_DMA_STATUS_BUSY);

	/* A single DRQ line: whoever comes second falls back to PIO. */
	if (controller->active)
		return false;

	/* The engine moves whole packets only; a short tail is left to PIO. */
	len -= len % packet_sz;
	if (!len)
		return false;

	/* The FIFO side always runs word-sized; odd packet sizes can't. */
	if (packet_sz & 3)
		return false;

	fifo_addr = controller->fifo_base +
		    controller->musb->io.fifo_offset(sunxi_channel->epnum);

	/*
	 * len is whole packets of a word-multiple size, so only the buffer
	 * address can misalign; an unaligned memory side drops to byte
	 * accesses (the FIFO side stays word bursts), as the vendor driver
	 * does.
	 */
	if (dma_addr & 3) {
		mem_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		mem_burst = 1;
	} else {
		mem_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		mem_burst = 4;
	}

	/*
	 * One packet per DRQ handshake on the USB side, with the wait
	 * states the vendor driver uses (16 USB-side, minimum SDRAM-side).
	 */
	if (sunxi_channel->transmit) {
		dir = DMA_MEM_TO_DEV;
		cfg.dst_addr = fifo_addr;
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		cfg.dst_maxburst = 4;
		cfg.src_addr_width = mem_width;
		cfg.src_maxburst = mem_burst;
		sunxi_channel->para.para =
			SUN4I_DDMA_PARA_DST_DATA_BLK_SIZE(packet_sz / 4) |
			SUN4I_DDMA_PARA_DST_WAIT_CYCLES(16) |
			SUN4I_DDMA_PARA_SRC_DATA_BLK_SIZE(packet_sz / 4) |
			SUN4I_DDMA_PARA_SRC_WAIT_CYCLES(1);
	} else {
		dir = DMA_DEV_TO_MEM;
		cfg.src_addr = fifo_addr;
		cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		cfg.src_maxburst = 4;
		cfg.dst_addr_width = mem_width;
		cfg.dst_maxburst = mem_burst;
		sunxi_channel->para.para =
			SUN4I_DDMA_PARA_SRC_DATA_BLK_SIZE(packet_sz / 4) |
			SUN4I_DDMA_PARA_SRC_WAIT_CYCLES(16) |
			SUN4I_DDMA_PARA_DST_DATA_BLK_SIZE(packet_sz / 4) |
			SUN4I_DDMA_PARA_DST_WAIT_CYCLES(1);
	}
	cfg.peripheral_config = &sunxi_channel->para;
	cfg.peripheral_size = sizeof(sunxi_channel->para);

	if (dmaengine_slave_config(controller->chan, &cfg))
		return false;

	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(dma_addr)), len,
		    offset_in_page(dma_addr));
	sg_dma_address(&sg) = dma_addr;
	sg_dma_len(&sg) = len;

	desc = dmaengine_prep_slave_sg(controller->chan, &sg, 1, dir,
				       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return false;

	desc->callback = sunxi_ddma_callback;
	desc->callback_param = sunxi_channel;

	channel->actual_len = 0;
	sunxi_channel->len = len;
	channel->status = MUSB_DMA_STATUS_BUSY;
	controller->active = sunxi_channel;

	dev_dbg(controller->musb->controller,
		"ddma ep%u %s start %u bytes @%pad pkt %u mode %u\n",
		sunxi_channel->epnum, sunxi_channel->transmit ? "tx" : "rx",
		len, &dma_addr, packet_sz, mode);

	dmaengine_submit(desc);
	dma_async_issue_pending(controller->chan);

	/*
	 * Wire this endpoint's FIFO to the DRQ line and switch the FIFO bus
	 * to DMA.  The engine then waits for the DRQ handshake, which the
	 * controller raises once the core sets the endpoint's CSR DMA
	 * request bits (Rx already did, Tx does right after we return).
	 */
	drq_sel = sunxi_channel->transmit ?
		SUNXI_MUSB_VEND0_DRQ_TX(sunxi_channel->epnum) :
		SUNXI_MUSB_VEND0_DRQ_RX(sunxi_channel->epnum);
	writeb(SUNXI_MUSB_VEND0_DMA_MODE | SUNXI_MUSB_VEND0_DRQ_SEL(drq_sel),
	       controller->base + SUNXI_MUSB_VEND0);

	return true;
}

static int sunxi_ddma_channel_abort(struct dma_channel *channel)
{
	struct sunxi_ddma_channel *sunxi_channel = channel->private_data;
	struct sunxi_ddma_controller *controller = sunxi_channel->controller;
	struct musb *musb = controller->musb;
	void __iomem *base = controller->base;
	void __iomem *epio;
	u16 csr;

	if (channel->status == MUSB_DMA_STATUS_BUSY) {
		/*
		 * Halt the engine before touching the endpoint's DMA CSR
		 * bits, as with the idma.  This must be the non-syncing
		 * terminate: the completion callback takes musb->lock,
		 * which our caller holds, so waiting for callbacks here
		 * would deadlock.  sun4i-dma frees the descriptors without
		 * running their callbacks, and the busy-status check in the
		 * callback covers one that already left the queue.
		 */
		dmaengine_terminate_all(controller->chan);

		musb->io.ep_select(base, sunxi_channel->epnum);
		epio = base + musb->io.ep_offset(sunxi_channel->epnum, 0);

		if (sunxi_channel->transmit) {
			csr = musb_readw(epio, MUSB_TXCSR);
			csr &= ~(MUSB_TXCSR_AUTOSET | MUSB_TXCSR_DMAENAB);
			musb_writew(epio, MUSB_TXCSR, csr);
			/* DMAENAB and DMAMODE can't clear in the same cycle */
			csr &= ~MUSB_TXCSR_DMAMODE;
			musb_writew(epio, MUSB_TXCSR, csr);
		} else {
			csr = musb_readw(epio, MUSB_RXCSR);
			csr &= ~(MUSB_RXCSR_AUTOCLEAR | MUSB_RXCSR_DMAENAB |
				 MUSB_RXCSR_DMAMODE);
			musb_writew(epio, MUSB_RXCSR, csr);
		}

		channel->actual_len = 0;
		channel->status = MUSB_DMA_STATUS_FREE;

		if (controller->active == sunxi_channel) {
			controller->active = NULL;
			writeb(SUNXI_MUSB_VEND0_PIO_MODE,
			       base + SUNXI_MUSB_VEND0);
		}
	}

	return 0;
}

static struct dma_controller *
sunxi_ddma_controller_create(struct musb *musb, void __iomem *base)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	struct platform_device *pdev = to_platform_device(musb->controller);
	struct sunxi_ddma_controller *controller;
	struct resource *iomem;

	/* The engine needs the FIFOs' physical address, not the mapping. */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem)
		return NULL;

	controller = kzalloc_obj(*controller, GFP_KERNEL);
	if (!controller)
		return NULL;

	controller->musb = musb;
	controller->base = base;
	controller->chan = glue->ddma_chan;
	controller->fifo_base = iomem->start;
	controller->controller.musb = musb;
	controller->controller.channel_alloc = sunxi_ddma_channel_allocate;
	controller->controller.channel_release = sunxi_ddma_channel_release;
	controller->controller.channel_program = sunxi_ddma_channel_program;
	controller->controller.channel_abort = sunxi_ddma_channel_abort;
	controller->controller.is_compatible = sunxi_ddma_is_compatible;

	return &controller->controller;
}

static void sunxi_ddma_controller_destroy(struct dma_controller *c)
{
	struct sunxi_ddma_controller *controller =
		container_of(c, struct sunxi_ddma_controller, controller);

	kfree(controller);
}

#else /* !CONFIG_USB_SUNXI_DDMA: ddma compiled out */

static struct dma_controller *
sunxi_ddma_controller_create(struct musb *musb, void __iomem *base)
{
	return NULL;
}

static void sunxi_ddma_controller_destroy(struct dma_controller *c) { }

#endif /* CONFIG_USB_SUNXI_DDMA */

/*
 * musb_init_controller() rejects a glue without dma_init/dma_exit unless
 * the whole DMA framework is compiled out, so these exist even with both
 * engines compiled out and just decline to provide a controller.
 */
static struct dma_controller *
sunxi_musb_dma_controller_create(struct musb *musb, void __iomem *base)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	if (test_bit(SUNXI_MUSB_FL_HAS_IDMA, &glue->flags))
		return sunxi_idma_controller_create(musb, base);

	if (test_bit(SUNXI_MUSB_FL_HAS_DDMA, &glue->flags))
		return sunxi_ddma_controller_create(musb, base);

	return NULL;
}

static void sunxi_musb_dma_controller_destroy(struct dma_controller *c)
{
	struct sunxi_glue *glue;

	if (!c)
		return;

	glue = dev_get_drvdata(c->musb->controller->parent);

	if (test_bit(SUNXI_MUSB_FL_HAS_IDMA, &glue->flags))
		sunxi_idma_controller_destroy(c);
	else if (test_bit(SUNXI_MUSB_FL_HAS_DDMA, &glue->flags))
		sunxi_ddma_controller_destroy(c);
}

static int sunxi_musb_set_mode(struct musb *musb, u8 mode)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);
	enum phy_mode new_mode;

	switch (mode) {
	case MUSB_HOST:
		new_mode = PHY_MODE_USB_HOST;
		break;
	case MUSB_PERIPHERAL:
		new_mode = PHY_MODE_USB_DEVICE;
		break;
	case MUSB_OTG:
		new_mode = PHY_MODE_USB_OTG;
		break;
	default:
		dev_err(musb->controller->parent,
			"Error requested mode not supported by this kernel\n");
		return -EINVAL;
	}

	if (glue->phy_mode == new_mode)
		return 0;

	if (musb->port1_status & USB_PORT_STAT_ENABLE)
		musb_root_disconnect(musb);

	/*
	 * phy_set_mode may sleep, and we're called with a spinlock held,
	 * so let sunxi_musb_work deal with it.
	 */
	glue->phy_mode = new_mode;
	set_bit(SUNXI_MUSB_FL_PHY_MODE_PEND, &glue->flags);
	schedule_work(&glue->work);

	return 0;
}

static int sunxi_musb_recover(struct musb *musb)
{
	struct sunxi_glue *glue = dev_get_drvdata(musb->controller->parent);

	/*
	 * Schedule a phy_set_mode with the current glue->phy_mode value,
	 * this will force end the current session.
	 */
	set_bit(SUNXI_MUSB_FL_PHY_MODE_PEND, &glue->flags);
	schedule_work(&glue->work);

	return 0;
}

/*
 * sunxi musb register layout
 * 0x00 - 0x17	fifo regs, 1 long per fifo
 * 0x40 - 0x57	generic control regs (power - frame)
 * 0x80 - 0x8f	ep control regs (addressed through hw_ep->regs, indexed)
 * 0x90 - 0x97	fifo control regs (indexed)
 * 0x98 - 0x9f	multipoint / busctl regs (indexed)
 * 0xc0		configdata reg
 */

static u32 sunxi_musb_fifo_offset(u8 epnum)
{
	return (epnum * 4);
}

static u32 sunxi_musb_ep_offset(u8 epnum, u16 offset)
{
	WARN_ONCE(offset != 0,
		  "sunxi_musb_ep_offset called with non 0 offset\n");

	return 0x80; /* indexed, so ignore epnum */
}

static u32 sunxi_musb_busctl_offset(u8 epnum, u16 offset)
{
	return SUNXI_MUSB_TXFUNCADDR + offset;
}

static u8 sunxi_musb_readb(void __iomem *addr, u32 offset)
{
	struct sunxi_glue *glue;

	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_FADDR:
			return readb(addr + SUNXI_MUSB_FADDR);
		case MUSB_POWER:
			return readb(addr + SUNXI_MUSB_POWER);
		case MUSB_INTRUSB:
			return readb(addr + SUNXI_MUSB_INTRUSB);
		case MUSB_INTRUSBE:
			return readb(addr + SUNXI_MUSB_INTRUSBE);
		case MUSB_INDEX:
			return readb(addr + SUNXI_MUSB_INDEX);
		case MUSB_TESTMODE:
			return 0; /* No testmode on sunxi */
		case MUSB_DEVCTL:
			return readb(addr + SUNXI_MUSB_DEVCTL);
		case MUSB_TXFIFOSZ:
			return readb(addr + SUNXI_MUSB_TXFIFOSZ);
		case MUSB_RXFIFOSZ:
			return readb(addr + SUNXI_MUSB_RXFIFOSZ);
		case MUSB_CONFIGDATA + 0x10: /* See musb_read_configdata() */
			glue = dev_get_drvdata(sunxi_musb->controller->parent);
			/* A33 saves a reg, and we get to hardcode this */
			if (test_bit(SUNXI_MUSB_FL_NO_CONFIGDATA,
				     &glue->flags))
				return 0xde;

			return readb(addr + SUNXI_MUSB_CONFIGDATA);
		case MUSB_ULPI_BUSCONTROL:
			dev_warn(sunxi_musb->controller->parent,
				"sunxi-musb does not have ULPI bus control register\n");
			return 0;
		/* Offset for these is fixed by sunxi_musb_busctl_offset() */
		case SUNXI_MUSB_TXFUNCADDR:
		case SUNXI_MUSB_TXHUBADDR:
		case SUNXI_MUSB_TXHUBPORT:
		case SUNXI_MUSB_RXFUNCADDR:
		case SUNXI_MUSB_RXHUBADDR:
		case SUNXI_MUSB_RXHUBPORT:
			/* multipoint / busctl reg access */
			return readb(addr + offset);
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown readb offset %u\n", offset);
			return 0;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		/* sunxi has a 2 byte hole before the txtype register */
		if (offset >= MUSB_TXTYPE)
			offset += 2;
		return readb(addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown readb at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
	return 0;
}

static void sunxi_musb_writeb(void __iomem *addr, unsigned offset, u8 data)
{
	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_FADDR:
			return writeb(data, addr + SUNXI_MUSB_FADDR);
		case MUSB_POWER:
			return writeb(data, addr + SUNXI_MUSB_POWER);
		case MUSB_INTRUSB:
			return writeb(data, addr + SUNXI_MUSB_INTRUSB);
		case MUSB_INTRUSBE:
			return writeb(data, addr + SUNXI_MUSB_INTRUSBE);
		case MUSB_INDEX:
			return writeb(data, addr + SUNXI_MUSB_INDEX);
		case MUSB_TESTMODE:
			if (data)
				dev_warn(sunxi_musb->controller->parent,
					"sunxi-musb does not have testmode\n");
			return;
		case MUSB_DEVCTL:
			return writeb(data, addr + SUNXI_MUSB_DEVCTL);
		case MUSB_TXFIFOSZ:
			return writeb(data, addr + SUNXI_MUSB_TXFIFOSZ);
		case MUSB_RXFIFOSZ:
			return writeb(data, addr + SUNXI_MUSB_RXFIFOSZ);
		case MUSB_ULPI_BUSCONTROL:
			dev_warn(sunxi_musb->controller->parent,
				"sunxi-musb does not have ULPI bus control register\n");
			return;
		/* Offset for these is fixed by sunxi_musb_busctl_offset() */
		case SUNXI_MUSB_TXFUNCADDR:
		case SUNXI_MUSB_TXHUBADDR:
		case SUNXI_MUSB_TXHUBPORT:
		case SUNXI_MUSB_RXFUNCADDR:
		case SUNXI_MUSB_RXHUBADDR:
		case SUNXI_MUSB_RXHUBPORT:
			/* multipoint / busctl reg access */
			return writeb(data, addr + offset);
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown writeb offset %u\n", offset);
			return;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		if (offset >= MUSB_TXTYPE)
			offset += 2;
		return writeb(data, addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown writeb at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
}

static u16 sunxi_musb_readw(void __iomem *addr, u32 offset)
{
	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_INTRTX:
			return readw(addr + SUNXI_MUSB_INTRTX);
		case MUSB_INTRRX:
			return readw(addr + SUNXI_MUSB_INTRRX);
		case MUSB_INTRTXE:
			return readw(addr + SUNXI_MUSB_INTRTXE);
		case MUSB_INTRRXE:
			return readw(addr + SUNXI_MUSB_INTRRXE);
		case MUSB_FRAME:
			return readw(addr + SUNXI_MUSB_FRAME);
		case MUSB_TXFIFOADD:
			return readw(addr + SUNXI_MUSB_TXFIFOADD);
		case MUSB_RXFIFOADD:
			return readw(addr + SUNXI_MUSB_RXFIFOADD);
		case MUSB_HWVERS:
			return 0; /* sunxi musb version is not known */
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown readw offset %u\n", offset);
			return 0;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		return readw(addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown readw at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
	return 0;
}

static void sunxi_musb_writew(void __iomem *addr, unsigned offset, u16 data)
{
	if (addr == sunxi_musb->mregs) {
		/* generic control or fifo control reg access */
		switch (offset) {
		case MUSB_INTRTX:
			return writew(data, addr + SUNXI_MUSB_INTRTX);
		case MUSB_INTRRX:
			return writew(data, addr + SUNXI_MUSB_INTRRX);
		case MUSB_INTRTXE:
			return writew(data, addr + SUNXI_MUSB_INTRTXE);
		case MUSB_INTRRXE:
			return writew(data, addr + SUNXI_MUSB_INTRRXE);
		case MUSB_FRAME:
			return writew(data, addr + SUNXI_MUSB_FRAME);
		case MUSB_TXFIFOADD:
			return writew(data, addr + SUNXI_MUSB_TXFIFOADD);
		case MUSB_RXFIFOADD:
			return writew(data, addr + SUNXI_MUSB_RXFIFOADD);
		default:
			dev_err(sunxi_musb->controller->parent,
				"Error unknown writew offset %u\n", offset);
			return;
		}
	} else if (addr == (sunxi_musb->mregs + 0x80)) {
		/* ep control reg access */
		return writew(data, addr + offset);
	}

	dev_err(sunxi_musb->controller->parent,
		"Error unknown writew at 0x%x bytes offset\n",
		(int)(addr - sunxi_musb->mregs));
}

/*
 * Template only: sunxi_musb_probe() copies this into glue->ops and adds the
 * idma quirks there, since whether we have an engine is a per-SoC property
 * and quirks are read straight out of musb->ops.
 */
static const struct musb_platform_ops sunxi_musb_ops = {
	.quirks		= MUSB_INDEXED_EP,
	.init		= sunxi_musb_init,
	.exit		= sunxi_musb_exit,
	.enable		= sunxi_musb_enable,
	.disable	= sunxi_musb_disable,
	.fifo_offset	= sunxi_musb_fifo_offset,
	.ep_offset	= sunxi_musb_ep_offset,
	.busctl_offset	= sunxi_musb_busctl_offset,
	.readb		= sunxi_musb_readb,
	.writeb		= sunxi_musb_writeb,
	.readw		= sunxi_musb_readw,
	.writew		= sunxi_musb_writew,
	.dma_init	= sunxi_musb_dma_controller_create,
	.dma_exit	= sunxi_musb_dma_controller_destroy,
	.set_mode	= sunxi_musb_set_mode,
	.recover	= sunxi_musb_recover,
	.set_vbus	= sunxi_musb_set_vbus,
	.pre_root_reset_end = sunxi_musb_pre_root_reset_end,
	.post_root_reset_end = sunxi_musb_post_root_reset_end,
};

#define SUNXI_MUSB_RAM_BITS	11

/* Allwinner OTG supports up to 5 endpoints */
static const struct musb_fifo_cfg sunxi_musb_mode_cfg_5eps[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_RX, 512),
};

/* Allwinner OTG supports up to 5 endpoints */
static const struct musb_fifo_cfg sunxi_musb_mode_cfg_5eps_fast[] = {
	/*
	 * EP1 is the bulk hot path and the only endpoint the idma drives.
	 * Double-buffer both halves (2x512) so the controller can accept /
	 * present the next packet while the idma drains / fills the current
	 * one, removing the per-packet bubble (matches the p-boot setup).
	 * Only EP1 is doubled: 8 KiB FIFO RAM (ram_bits 11) fits ep0(64) +
	 * ep1(2x1024) + the remaining single-buffered eps, but not all.
	 */
	MUSB_EP_FIFO_DOUBLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_DOUBLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_DOUBLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_DOUBLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(5, FIFO_RX, 512),
};

/* H3/V3s OTG supports only 4 endpoints */
static const struct musb_fifo_cfg sunxi_musb_mode_cfg_4eps[] = {
	MUSB_EP_FIFO_SINGLE(1, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(1, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(2, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(3, FIFO_RX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_TX, 512),
	MUSB_EP_FIFO_SINGLE(4, FIFO_RX, 512),
};

static const struct musb_hdrc_config sunxi_musb_hdrc_config_5eps = {
	.fifo_cfg       = sunxi_musb_mode_cfg_5eps,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg_5eps),
	.multipoint	= true,
	.dyn_fifo	= true,
	/* Two FIFOs per endpoint, plus ep_0. */
	.num_eps	= (ARRAY_SIZE(sunxi_musb_mode_cfg_5eps) / 2) + 1,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
};

static const struct musb_hdrc_config sunxi_musb_hdrc_config_5eps_fast = {
	.fifo_cfg       = sunxi_musb_mode_cfg_5eps_fast,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg_5eps_fast),
	.multipoint	= true,
	.dyn_fifo	= true,
	/* Two FIFOs per endpoint, plus ep_0. */
	.num_eps	= (ARRAY_SIZE(sunxi_musb_mode_cfg_5eps_fast) / 2) + 1,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
};

static const struct musb_hdrc_config sunxi_musb_hdrc_config_4eps = {
	.fifo_cfg       = sunxi_musb_mode_cfg_4eps,
	.fifo_cfg_size  = ARRAY_SIZE(sunxi_musb_mode_cfg_4eps),
	.multipoint	= true,
	.dyn_fifo	= true,
	/* Two FIFOs per endpoint, plus ep_0. */
	.num_eps	= (ARRAY_SIZE(sunxi_musb_mode_cfg_4eps) / 2) + 1,
	.ram_bits	= SUNXI_MUSB_RAM_BITS,
};

static const char * const sunxi_musb_host_rerouted_phys[] = {
	"allwinner,sun8i-h3-usb-phy",
	"allwinner,sun8i-r40-usb-phy",
	"allwinner,sun8i-v3s-usb-phy",
	"allwinner,sun50i-a64-usb-phy",
	"allwinner,sun50i-h6-usb-phy",
	NULL,
};

static int sunxi_musb_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	pdata;
	struct platform_device_info	pinfo;
	struct sunxi_glue		*glue;
	const struct sunxi_musb_cfg	*cfg;
	struct device_node		*np = pdev->dev.of_node, *phy_np;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Error no device tree node found\n");
		return -EINVAL;
	}

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	memset(&pdata, 0, sizeof(pdata));
	switch (usb_get_dr_mode(&pdev->dev)) {
#if defined CONFIG_USB_MUSB_DUAL_ROLE || defined CONFIG_USB_MUSB_HOST
	case USB_DR_MODE_HOST:
		pdata.mode = MUSB_HOST;
		glue->phy_mode = PHY_MODE_USB_HOST;
		break;
#endif
#if defined CONFIG_USB_MUSB_DUAL_ROLE || defined CONFIG_USB_MUSB_GADGET
	case USB_DR_MODE_PERIPHERAL:
		pdata.mode = MUSB_PERIPHERAL;
		glue->phy_mode = PHY_MODE_USB_DEVICE;
		break;
#endif
#ifdef CONFIG_USB_MUSB_DUAL_ROLE
	case USB_DR_MODE_OTG:
		pdata.mode = MUSB_OTG;
		glue->phy_mode = PHY_MODE_USB_OTG;
		break;
#endif
	default:
		dev_err(&pdev->dev, "Invalid or missing 'dr_mode' property\n");
		return -EINVAL;
	}
	cfg = of_device_get_match_data(&pdev->dev);
	if (!cfg)
		return -EINVAL;

	pdata.config = cfg->hdrc_config;

	glue->dev = &pdev->dev;
	INIT_WORK(&glue->work, sunxi_musb_work);
	glue->host_nb.notifier_call = sunxi_musb_host_notifier;

	if (cfg->has_sram)
		set_bit(SUNXI_MUSB_FL_HAS_SRAM, &glue->flags);

	if (cfg->has_reset)
		set_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags);

	if (cfg->no_configdata)
		set_bit(SUNXI_MUSB_FL_NO_CONFIGDATA, &glue->flags);

	/*
	 * The idma registers sit above the USB PHY rather than inside the
	 * core's own window, so they come from a second region.  DTs predating
	 * it describe only the core; run those on PIO instead of poking
	 * addresses we were never given.
	 */
	if (IS_ENABLED(CONFIG_USB_SUNXI_IDMA) && cfg->has_idma) {
		glue->idma_base =
			devm_platform_ioremap_resource_byname(pdev, "dma");
		if (IS_ERR(glue->idma_base))
			dev_warn(&pdev->dev,
				 "no idma register region, using PIO: %pe\n",
				 glue->idma_base);
		else
			set_bit(SUNXI_MUSB_FL_HAS_IDMA, &glue->flags);
	}

	/*
	 * SoCs without any DMA engine in the USB block use the SoC DMA
	 * engine through a channel described in DT.  A missing description
	 * means PIO, like above; a missing provider is a deferral rather
	 * than an error, since the DT says the channel exists.
	 */
	if (IS_ENABLED(CONFIG_USB_SUNXI_DDMA) && cfg->has_ddma) {
		struct dma_chan *chan;

		chan = devm_dma_request_chan(&pdev->dev, "rxtx");
		if (IS_ERR(chan)) {
			if (PTR_ERR(chan) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_warn(&pdev->dev,
				 "no DMA channel, using PIO: %pe\n", chan);
		} else {
			glue->ddma_chan = chan;
			set_bit(SUNXI_MUSB_FL_HAS_DDMA, &glue->flags);
		}
	}

	/*
	 * MUSB_DMA_SW_MODE_SELECT makes the gadget core drive our engine via
	 * the standard software-mode-select RX/TX DMA paths (channel_program +
	 * the matching RX/TXCSR DMA bits).  MUSB_DMA_RX_MODE1_ALWAYS forces
	 * the Mode 1 (AUTOCLEAR) RXCSR setup the A64 idma requires for RX; the
	 * ddma works with the standard mode selection and must not declare it.
	 * Only declare them once we know we can actually drive the engine:
	 * they describe how the DMA paths must behave, and generic code is
	 * entitled to read them without first checking for a dma_controller.
	 */
	glue->ops = sunxi_musb_ops;
	if (test_bit(SUNXI_MUSB_FL_HAS_IDMA, &glue->flags))
		glue->ops.quirks |= MUSB_DMA_SW_MODE_SELECT |
				    MUSB_DMA_RX_MODE1_ALWAYS;
	else if (test_bit(SUNXI_MUSB_FL_HAS_DDMA, &glue->flags))
		glue->ops.quirks |= MUSB_DMA_SW_MODE_SELECT;

	pdata.platform_ops = &glue->ops;

	glue->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(glue->clk)) {
		dev_err(&pdev->dev, "Error getting clock: %ld\n",
			PTR_ERR(glue->clk));
		return PTR_ERR(glue->clk);
	}

	if (test_bit(SUNXI_MUSB_FL_HAS_RESET, &glue->flags)) {
		glue->rst = devm_reset_control_get(&pdev->dev, NULL);
		if (IS_ERR(glue->rst))
			return dev_err_probe(&pdev->dev, PTR_ERR(glue->rst),
					     "Error getting reset\n");
	}

	glue->extcon = extcon_get_edev_by_phandle(&pdev->dev, 0);
	if (IS_ERR(glue->extcon))
		return dev_err_probe(&pdev->dev, PTR_ERR(glue->extcon),
				     "Invalid or missing extcon\n");

	glue->phy = devm_phy_get(&pdev->dev, "usb");
	if (IS_ERR(glue->phy))
		return dev_err_probe(&pdev->dev, PTR_ERR(glue->phy),
				     "Error getting phy\n");

	/*
	 * Host mode is handled outside of the musb driver on some allwinner
	 * SoCs. We don't need musb host side code to be enabled at all.
	 * In fact it causes occasional issues with suspend to ram, when
	 * the host side code is enabled and unused (due to phy being re-routed
	 * to a different *HCI controller).
	 */
	phy_np = glue->phy->dev.of_node;
	if (of_device_compatible_match(phy_np, sunxi_musb_host_rerouted_phys)) {
		dev_info(&pdev->dev, "Disabling musb host side code due to re-routed phy\n");
		pdata.mode = MUSB_PERIPHERAL;
	}

	glue->usb_phy = usb_phy_generic_register();
	if (IS_ERR(glue->usb_phy)) {
		dev_err(&pdev->dev, "Error registering usb-phy %ld\n",
			PTR_ERR(glue->usb_phy));
		return PTR_ERR(glue->usb_phy);
	}

	glue->xceiv = devm_usb_get_phy(&pdev->dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(glue->xceiv)) {
		ret = PTR_ERR(glue->xceiv);
		dev_err(&pdev->dev, "Error getting usb-phy %d\n", ret);
		goto err_unregister_usb_phy;
	}

	platform_set_drvdata(pdev, glue);

	memset(&pinfo, 0, sizeof(pinfo));
	pinfo.name	 = "musb-hdrc";
	pinfo.id	= PLATFORM_DEVID_AUTO;
	pinfo.parent	= &pdev->dev;
	pinfo.fwnode	= of_fwnode_handle(pdev->dev.of_node);
	pinfo.of_node_reused = true;
	pinfo.res	= pdev->resource;
	pinfo.num_res	= pdev->num_resources;
	pinfo.data	= &pdata;
	pinfo.size_data = sizeof(pdata);

	glue->musb_pdev = platform_device_register_full(&pinfo);
	if (IS_ERR(glue->musb_pdev)) {
		ret = PTR_ERR(glue->musb_pdev);
		dev_err(&pdev->dev, "Error registering musb dev: %d\n", ret);
		goto err_unregister_usb_phy;
	}

	return 0;

err_unregister_usb_phy:
	usb_phy_generic_unregister(glue->usb_phy);
	return ret;
}

static void sunxi_musb_remove(struct platform_device *pdev)
{
	struct sunxi_glue *glue = platform_get_drvdata(pdev);
	struct platform_device *usb_phy = glue->usb_phy;

	platform_device_unregister(glue->musb_pdev);
	usb_phy_generic_unregister(usb_phy);
}

static const struct sunxi_musb_cfg sun4i_a10_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_sram = true,
};

static const struct sunxi_musb_cfg sun5i_a13_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_sram = true,
	.has_ddma = true,
};

static const struct sunxi_musb_cfg sun6i_a31_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_reset = true,
};

static const struct sunxi_musb_cfg sun8i_a33_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_reset = true,
	.no_configdata = true,
};

static const struct sunxi_musb_cfg sun8i_a83t_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_reset = true,
	.no_configdata = true,
	.has_idma = true,
};

static const struct sunxi_musb_cfg sun8i_h3_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_4eps,
	.has_reset = true,
	.no_configdata = true,
	.has_idma = true,
};

static const struct sunxi_musb_cfg sun50i_a64_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps_fast,
	.has_reset = true,
	.no_configdata = true,
	.has_idma = true,
};

static const struct sunxi_musb_cfg suniv_f1c100s_musb_cfg = {
	.hdrc_config = &sunxi_musb_hdrc_config_5eps,
	.has_sram = true,
	.has_reset = true,
	.no_configdata = true,
};

static const struct of_device_id sunxi_musb_match[] = {
	{ .compatible = "allwinner,sun4i-a10-musb",
	  .data = &sun4i_a10_musb_cfg, },
	{ .compatible = "allwinner,sun5i-a13-musb",
	  .data = &sun5i_a13_musb_cfg, },
	{ .compatible = "allwinner,sun6i-a31-musb",
	  .data = &sun6i_a31_musb_cfg, },
	{ .compatible = "allwinner,sun8i-a33-musb",
	  .data = &sun8i_a33_musb_cfg, },
	{ .compatible = "allwinner,sun8i-a83t-musb",
	  .data = &sun8i_a83t_musb_cfg, },
	{ .compatible = "allwinner,sun8i-h3-musb",
	  .data = &sun8i_h3_musb_cfg, },
	{ .compatible = "allwinner,sun50i-a64-musb",
	  .data = &sun50i_a64_musb_cfg, },
	{ .compatible = "allwinner,suniv-f1c100s-musb",
	  .data = &suniv_f1c100s_musb_cfg, },
	{}
};
MODULE_DEVICE_TABLE(of, sunxi_musb_match);

static struct platform_driver sunxi_musb_driver = {
	.probe = sunxi_musb_probe,
	.remove = sunxi_musb_remove,
	.driver = {
		.name = "musb-sunxi",
		.of_match_table = sunxi_musb_match,
	},
};
module_platform_driver(sunxi_musb_driver);

MODULE_DESCRIPTION("Allwinner sunxi MUSB Glue Layer");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL v2");
