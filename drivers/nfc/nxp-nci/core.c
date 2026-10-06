// SPDX-License-Identifier: GPL-2.0-only
/*
 * Generic driver for NXP NCI NFC chips
 *
 * Copyright (C) 2014  NXP Semiconductors  All rights reserved.
 *
 * Authors: Clément Perrochaud <clement.perrochaud@nxp.com>
 *
 * Derived from PN544 device driver:
 * Copyright (C) 2012  Intel Corporation. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/nfc.h>

#include <net/nfc/nci_core.h>

#include "nxp-nci.h"

#define NXP_NCI_HDR_LEN	4

/*
 * MIFARE Classic support. The generic NCI core maps only ISO-DEP and
 * NFC-DEP to dedicated RF interfaces and leaves everything else on the
 * frame interface, where the controller relays bytes to the card
 * without running Crypto1 - so a Classic can be typed but never read.
 * NXP's own stack instead adds one RF_DISCOVER_MAP entry mapping the
 * proprietary MIFARE protocol to the proprietary MIFARE RF interface;
 * with it the controller activates a Classic on that interface and
 * performs the authentication itself, after which an 0x40 authenticate
 * frame and 0x30 reads sent over an AF_NFC raw socket come back
 * decrypted. We append that one mapping to RF_DISCOVER_MAP on the way
 * out, and teach the core to recognise the proprietary protocol id the
 * controller then reports back at activation (.get_rfprotocol).
 */
#define NXP_NCI_RF_PROTOCOL_MIFARE	0x80
#define NXP_NCI_RF_INTERFACE_MIFARE	0x80
#define NXP_NCI_DISC_MAP_MODE_POLL	0x01

#define NXP_NCI_NFC_PROTOCOLS (NFC_PROTO_JEWEL_MASK | \
			       NFC_PROTO_MIFARE_MASK | \
			       NFC_PROTO_FELICA_MASK | \
			       NFC_PROTO_ISO14443_MASK | \
			       NFC_PROTO_ISO14443_B_MASK | \
			       NFC_PROTO_ISO15693_MASK | \
			       NFC_PROTO_NFC_DEP_MASK)

#define NXP_NCI_RF_PLL_UNLOCKED_NTF nci_opcode_pack(NCI_GID_RF_MGMT, 0x21)
#define NXP_NCI_RF_TXLDO_ERROR_NTF nci_opcode_pack(NCI_GID_RF_MGMT, 0x23)

static int nxp_nci_open(struct nci_dev *ndev)
{
	struct nxp_nci_info *info = nci_get_drvdata(ndev);
	int r = 0;

	mutex_lock(&info->info_lock);

	if (info->mode != NXP_NCI_MODE_COLD) {
		r = -EBUSY;
		goto open_exit;
	}

	if (info->phy_ops->set_mode)
		r = info->phy_ops->set_mode(info->phy_id, NXP_NCI_MODE_NCI);

	info->mode = NXP_NCI_MODE_NCI;

open_exit:
	mutex_unlock(&info->info_lock);
	return r;
}

static int nxp_nci_close(struct nci_dev *ndev)
{
	struct nxp_nci_info *info = nci_get_drvdata(ndev);
	int r = 0;

	mutex_lock(&info->info_lock);

	if (info->phy_ops->set_mode)
		r = info->phy_ops->set_mode(info->phy_id, NXP_NCI_MODE_COLD);

	info->mode = NXP_NCI_MODE_COLD;

	mutex_unlock(&info->info_lock);
	return r;
}

static __u32 nxp_nci_get_rfprotocol(struct nci_dev *ndev, __u8 rf_protocol)
{
	return rf_protocol == NXP_NCI_RF_PROTOCOL_MIFARE ?
	       NFC_PROTO_MIFARE_MASK : 0;
}

/*
 * Append the MIFARE protocol->interface mapping to an outgoing
 * RF_DISCOVER_MAP command. Returns the buffer to send, which may be a
 * reallocated one; the original is consumed in that case. On any
 * problem the command is sent unchanged - MIFARE just stays unmapped.
 */
static struct sk_buff *nxp_nci_add_mifare_map(struct sk_buff *skb)
{
	u8 *hdr = skb->data;
	u8 *ent;

	/* octet2 is the parameter length, octet3 the number of mapping
	 * entries; guard that both are present before touching them */
	if (skb->len < NCI_CTRL_HDR_SIZE + 1 ||
	    nci_mt(hdr) != NCI_MT_CMD_PKT ||
	    nci_opcode(hdr) != NCI_OP_RF_DISCOVER_MAP_CMD)
		return skb;

	if (skb_tailroom(skb) < 3) {
		struct sk_buff *nskb;

		nskb = skb_copy_expand(skb, skb_headroom(skb), 3, GFP_KERNEL);
		if (!nskb)
			return skb;
		consume_skb(skb);
		skb = nskb;
		hdr = skb->data;
	}

	ent = skb_put(skb, 3);
	ent[0] = NXP_NCI_RF_PROTOCOL_MIFARE;
	ent[1] = NXP_NCI_DISC_MAP_MODE_POLL;
	ent[2] = NXP_NCI_RF_INTERFACE_MIFARE;
	hdr[2] += 3;		/* parameter length */
	hdr[3] += 1;		/* number of mapping configurations */

	return skb;
}

/*
 * NXP proprietary configuration, needed to read MIFARE Classic. With a
 * Classic activated on the proprietary MIFARE RF interface the CRYPTO1
 * authenticate succeeds, but the block reads that follow it come back
 * malformed, because the generic NCI bring-up never sends the vendor
 * configuration NXP's own stack does.
 *
 * Send it from .post_setup, which runs after CORE_INIT (a config command
 * before CORE_INIT is a protocol error and aborts bring-up): first
 * NXP_ACT_PROP_EXTN, which turns on the proprietary command set and is
 * required before any 0xA0 parameter is accepted, then the proprietary
 * extension configuration and the standard RF configuration. The values
 * are NXP's for the PN547C2.
 *
 * Best effort: a controller that rejects a command is logged and the
 * bring-up continues, so a configuration hiccup cannot take NFC down
 * altogether - it just leaves MIFARE Classic unreadable, as before.
 */

/* NXP_CORE_CONF_EXTN - CORE_SET_CONFIG payload: count, then 0xA0 TLVs */
static const __u8 nxp_nci_core_conf_extn[] = {
	0x04,
	0xA0, 0x5E, 0x01, 0x01,
	0xA0, 0x40, 0x01, 0x01,
	0xA0, 0x41, 0x01, 0x04,
	0xA0, 0x43, 0x01, 0x00,
};

/* NXP_CORE_CONF - CORE_SET_CONFIG payload: count, then standard NCI TLVs */
static const __u8 nxp_nci_core_conf[] = {
	0x0D,
	0x28, 0x01, 0x00,
	0x21, 0x01, 0x00,
	0x30, 0x01, 0x08,
	0x31, 0x01, 0x03,
	0x33, 0x04, 0x04, 0x03, 0x02, 0x01,
	0x54, 0x01, 0x06,
	0x50, 0x01, 0x02,
	0x5B, 0x01, 0x00,
	0x60, 0x01, 0x0E,
	0x80, 0x01, 0x01,
	0x81, 0x01, 0x01,
	0x82, 0x01, 0x0E,
	0x18, 0x01, 0x01,
};

/* NXP_NFC_PROFILE_EXTN - CORE_SET_CONFIG payload: select NFC Forum profile */
static const __u8 nxp_nci_nfc_profile_extn[] = {
	0x01,
	0xA0, 0x44, 0x01, 0x00,
};

static int nxp_nci_post_setup(struct nci_dev *ndev)
{
	int r;

	/* NXP_ACT_PROP_EXTN (2F 02 00) */
	r = nci_prop_cmd(ndev, 0x02, 0, NULL);
	if (r < 0)
		nfc_err(&ndev->nfc_dev->dev,
			"failed to enable NXP proprietary extensions: %d\n", r);

	/* NXP_NFC_PROFILE_EXTN */
	r = nci_core_cmd(ndev, NCI_OP_CORE_SET_CONFIG_CMD,
			 sizeof(nxp_nci_nfc_profile_extn),
			 nxp_nci_nfc_profile_extn);
	if (r < 0)
		nfc_err(&ndev->nfc_dev->dev,
			"failed to apply NXP NFC profile: %d\n", r);

	/* NXP_CORE_CONF_EXTN */
	r = nci_core_cmd(ndev, NCI_OP_CORE_SET_CONFIG_CMD,
			 sizeof(nxp_nci_core_conf_extn),
			 nxp_nci_core_conf_extn);
	if (r < 0)
		nfc_err(&ndev->nfc_dev->dev,
			"failed to apply NXP core configuration: %d\n", r);

	/* NXP_CORE_CONF - standard NCI RF configuration */
	r = nci_core_cmd(ndev, NCI_OP_CORE_SET_CONFIG_CMD,
			 sizeof(nxp_nci_core_conf),
			 nxp_nci_core_conf);
	if (r < 0)
		nfc_err(&ndev->nfc_dev->dev,
			"failed to apply NXP RF configuration: %d\n", r);

	return 0;
}

static int nxp_nci_send(struct nci_dev *ndev, struct sk_buff *skb)
{
	struct nxp_nci_info *info = nci_get_drvdata(ndev);
	int r;

	if (!info->phy_ops->write) {
		kfree_skb(skb);
		return -EOPNOTSUPP;
	}

	if (info->mode != NXP_NCI_MODE_NCI) {
		kfree_skb(skb);
		return -EINVAL;
	}

	skb = nxp_nci_add_mifare_map(skb);

	r = info->phy_ops->write(info->phy_id, skb);
	if (r < 0) {
		kfree_skb(skb);
		return r;
	}

	consume_skb(skb);
	return 0;
}

static int nxp_nci_rf_pll_unlocked_ntf(struct nci_dev *ndev,
				       struct sk_buff *skb)
{
	nfc_err(&ndev->nfc_dev->dev,
		"PLL didn't lock. Missing or unstable clock?\n");

	return 0;
}

static int nxp_nci_rf_txldo_error_ntf(struct nci_dev *ndev,
				      struct sk_buff *skb)
{
	nfc_err(&ndev->nfc_dev->dev,
		"RF transmitter couldn't start. Bad power and/or configuration?\n");

	return 0;
}

static const struct nci_driver_ops nxp_nci_core_ops[] = {
	{
		.opcode = NXP_NCI_RF_PLL_UNLOCKED_NTF,
		.ntf = nxp_nci_rf_pll_unlocked_ntf,
	},
	{
		.opcode = NXP_NCI_RF_TXLDO_ERROR_NTF,
		.ntf = nxp_nci_rf_txldo_error_ntf,
	},
};

/*
 * Complete the request for a proprietary command. The generic core has
 * no handler for a proprietary (GID 0xF) response, so without this the
 * command sent from .post_setup never completes and blocks bring-up for
 * a full command timeout.
 */
static int nxp_nci_prop_rsp(struct nci_dev *ndev, struct sk_buff *skb)
{
	__u8 status = skb->data[0];

	nci_req_complete(ndev, status);
	return 0;
}

static const struct nci_driver_ops nxp_nci_prop_ops[] = {
	{
		/* NXP_ACT_PROP_EXTN */
		.opcode = nci_opcode_pack(NCI_GID_PROPRIETARY, 0x02),
		.rsp = nxp_nci_prop_rsp,
	},
};

static const struct nci_ops nxp_nci_ops = {
	.open = nxp_nci_open,
	.close = nxp_nci_close,
	.send = nxp_nci_send,
	.post_setup = nxp_nci_post_setup,
	.fw_download = nxp_nci_fw_download,
	.get_rfprotocol = nxp_nci_get_rfprotocol,
	.core_ops = nxp_nci_core_ops,
	.n_core_ops = ARRAY_SIZE(nxp_nci_core_ops),
	.prop_ops = nxp_nci_prop_ops,
	.n_prop_ops = ARRAY_SIZE(nxp_nci_prop_ops),
};

int nxp_nci_probe(void *phy_id, struct device *pdev,
		  const struct nxp_nci_phy_ops *phy_ops,
		  unsigned int max_payload,
		  struct nci_dev **ndev)
{
	struct nxp_nci_info *info;
	int r;

	info = devm_kzalloc(pdev, sizeof(struct nxp_nci_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->phy_id = phy_id;
	info->pdev = pdev;
	info->phy_ops = phy_ops;
	info->max_payload = max_payload;
	INIT_WORK(&info->fw_info.work, nxp_nci_fw_work);
	init_completion(&info->fw_info.cmd_completion);
	mutex_init(&info->info_lock);

	if (info->phy_ops->set_mode) {
		r = info->phy_ops->set_mode(info->phy_id, NXP_NCI_MODE_COLD);
		if (r < 0)
			return r;
	}

	info->mode = NXP_NCI_MODE_COLD;

	info->ndev = nci_allocate_device(&nxp_nci_ops, NXP_NCI_NFC_PROTOCOLS,
					 NXP_NCI_HDR_LEN, 0);
	if (!info->ndev)
		return -ENOMEM;

	nci_set_parent_dev(info->ndev, pdev);
	nci_set_drvdata(info->ndev, info);
	r = nci_register_device(info->ndev);
	if (r < 0) {
		nci_free_device(info->ndev);
		return r;
	}

	*ndev = info->ndev;
	return r;
}
EXPORT_SYMBOL(nxp_nci_probe);

void nxp_nci_remove(struct nci_dev *ndev)
{
	struct nxp_nci_info *info = nci_get_drvdata(ndev);

	if (info->mode == NXP_NCI_MODE_FW)
		nxp_nci_fw_work_complete(info, -ESHUTDOWN);
	cancel_work_sync(&info->fw_info.work);

	mutex_lock(&info->info_lock);

	if (info->phy_ops->set_mode)
		info->phy_ops->set_mode(info->phy_id, NXP_NCI_MODE_COLD);

	mutex_unlock(&info->info_lock);

	nci_unregister_device(ndev);
	nci_free_device(ndev);
}
EXPORT_SYMBOL(nxp_nci_remove);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NXP NCI NFC driver");
MODULE_AUTHOR("Clément Perrochaud <clement.perrochaud@nxp.com>");
