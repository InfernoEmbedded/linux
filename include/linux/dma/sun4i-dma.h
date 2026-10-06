/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Allwinner sun4i DMA controller client interface
 */
#ifndef _LINUX_DMA_SUN4I_DMA_H
#define _LINUX_DMA_SUN4I_DMA_H

#include <linux/types.h>

/* Dedicated DMA parameter register layout */
#define SUN4I_DDMA_PARA_DST_DATA_BLK_SIZE(n)	(((n) - 1) << 24)
#define SUN4I_DDMA_PARA_DST_WAIT_CYCLES(n)	(((n) - 1) << 16)
#define SUN4I_DDMA_PARA_SRC_DATA_BLK_SIZE(n)	(((n) - 1) << 8)
#define SUN4I_DDMA_PARA_SRC_WAIT_CYCLES(n)	(((n) - 1) << 0)

/*
 * Peripheral-specific configuration, passed through the peripheral_config
 * member of struct dma_slave_config.  Without it, dedicated DMA slave
 * transfers use timing parameters that are known to work for SPI.
 *
 * para is the raw value for the dedicated DMA parameter register, composed
 * from the SUN4I_DDMA_PARA_* fields above.  It sets the DRQ handshake
 * granularity (data block size) and the wait states inserted between
 * blocks.  It has no effect on normal DMA channels.
 */
struct sun4i_dma_chan_config {
	u32 para;
};

#endif /* _LINUX_DMA_SUN4I_DMA_H */
