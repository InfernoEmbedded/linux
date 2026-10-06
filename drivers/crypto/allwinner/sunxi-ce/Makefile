# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_CRYPTO_DEV_SUNXI_CE) += sunxi-ce.o

sunxi-ce-y := sunxi_ce.o sunxi_ce_proc_comm.o sunxi_hwrng.o v5/sunxi_ce_reg.o v5/sunxi_ce_proc.o

ccflags-y += -DCONFIG_ARCH_SUN60IW2 -DCONFIG_AW_HWRNG_DRIVER=1 -DSS_SUPPORT_CE_V5 \
	     -DSS_GCM_MODE_ENABLE -DSS_SCATTER_ENABLE -DTASK_DMA_POOL \
	     -DSS_SHA_SWAP_PRE_ENABLE -DCE_BYTE_ADDR_ENABLE -DSS_TOTAL_DATALEN_ENABLE \
	     -DSS_SHA224_ENABLE -DSS_SHA256_ENABLE -DSS_SHA384_ENABLE -DSS_SHA512_ENABLE \
	     -DSS_HMAC_ENABLE -I$(src) -I$(src)/v5
