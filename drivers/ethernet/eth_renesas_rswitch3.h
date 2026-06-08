/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH3_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH3_H_

#include <zephyr/device.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/ethernet/eth_renesas_rswitch.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Register offsets */
#define RSWITCH3_ETHA_OFFSET 0x1d000
#define RSWITCH3_ETHA_SIZE   0x02000
#define RMRO                 0x1000

/* COMA block (per-agent clock gating, buffer pool) */
#define RSWITCH3_COMA_OFFSET 0x1c000
#define RCEC                 (RSWITCH3_COMA_OFFSET + 0x0008)
#define RCDC                 (RSWITCH3_COMA_OFFSET + 0x000c)
#define CABPPFLC0            (RSWITCH3_COMA_OFFSET + 0x0050)
#define CABPIRM              (RSWITCH3_COMA_OFFSET + 0x0160)

/* Register rswitch3 forwarding */
#define FWPBFC(i)  (0x4a00 + (i) * 0x10)
#define FWPBFC1(i) (0x4a04 + (i) * 0x10)
#define FWPC0(i)   (0x0100 + (i) * 0x10)
#define FWPC1(i)   (0x0104 + (i) * 0x10)
#define FWPC2(i)   (0x0108 + (i) * 0x10)
#define FWMACTIM   0x4680

/* Register rswitch3 etha */
#define EAMC     0x0000
#define EAMS     0x0004
#define EATDRC   0x0008
#define EAIRC    0x0010
#define EATDQSC  0x0014
#define EATDQAC  0x001c
#define EATPEC   0x0020
#define EATMFSC0 0x0040
#define EATDQDC0 0x0060
#define EAVCC    0x0130
#define EACTDQDC 0x0104
#define EATTFC   0x0138
#define MIOC     (RMRO + 0x0010)
#define MPIC     (RMRO + 0x0004)
#define MRMAC0   (RMRO + 0x0084)
#define MRMAC1   (RMRO + 0x0088)
#define MRAFC    (RMRO + 0x008c)
#define MLVC     (RMRO + 0x0180)
#define EATDQC   0x0018

#define EATMFSC(q) (EATMFSC0 + (q) * 0x04)
#define EATDQDC(q) (EATDQDC0 + (q) * 0x04)

/* EAMC/EAMS */
enum rsw3_etha_mode {
	EAMC_OPC_DISABLE = 1,
	EAMC_OPC_CONFIG = 2,
	EAMC_OPC_OPERATION = 3,
};
#define EAMS_OPS_MASK EAMC_OPC_OPERATION

/* EAVCC */
#define EAVCC_VEM_NO_TAG (0)
#define EAVCC_VEM_SC_TAG (0x3 << 16)

/* MRAFC */
#define RSW3_MRAFC_RX_PROMISC 0x07870787

/* MIOC */
#define MIOC_FORCE_PHY_LINK BIT(5)
#define MIOC_BIT3_SET       BIT(3)

/* MLVC (link verification). */
#define MLVC_PLV      BIT(16)
#define MLVC_PASE     BIT(8)
#define MLVC_LVT_10MS 0x09U /* 10ms wait time, IEEE 802.3 default */

/* MPIC */
#define MPIC_PIS      GENMASK(2, 0)
#define MPIC_PIS_GMII 2
#define MPIC_LSC      GENMASK(5, 3)
#define MPIC_LSC_100M 1
#define MPIC_LSC_1G   2
#define MPIC_LSC_2_5G 3
#define MPIC_PLSPP    BIT(10)

/* EATDQC */
#define EATDQC_DISABLE_CUT_THROUGH BIT(8)

/* TSN-ES egress path (per-priority TX queue depth / max frame size) */
#define RSWITCH3_NUM_PRIOS           8
#define RSWITCH3_TSNA_DESC_RAM_DEPTH 2048
#define RSWITCH3_TSNA_QUEUE_DEPTH    (RSWITCH3_TSNA_DESC_RAM_DEPTH / RSWITCH3_NUM_PRIOS)
#define EATMFSC_MAX                  0xffff

/* RCEC/RCDC (per-agent clock gating) */
#define RCEC_RCE BIT(16)
#define RCDC_RCD BIT(16)

/* CABPIRM/CABPPFLC0 (buffer pool init) */
#define CABPIRM_BPIOG       BIT(0)
#define CABPIRM_BPR         BIT(1)
#define CABPPFLC_INIT_VALUE 0x00800080

/* FWMACTIM (forwarding MAC table init) */
#define FWMACTIM_MACTIOG BIT(0)
#define FWMACTIM_MACTR   BIT(1)

/* FWPBFC */
#define FWPBFC_PBDV GENMASK(14, 0)

/* FWPBFC1 */
#define FWPBFC1_PBRP     GENMASK(19, 16)
#define FWPBFC1_PBRP_TSN 0xf

/* FWPC0 */
#define FWPC0_MACSDA BIT(20)

/* FWPC1 */
#define FWCP1_LTHFW GENMASK(31, 16)

/* FWPC2 */
#define FWPC2_LTWFM            GENMASK(31, 16)
#define FWPC2_LTWFM_TO_PORT(p) FIELD_PREP(FWPC2_LTWFM, (p))

/*
 * Forwarding-engine discard counters (read-only, per-port, stride 0x20).
 */
#define FWCTFDCN(i) (0x6300 + (i) * 0x20) /* TX-side: cut-through forward discard */
#define FWPBFDCN(i) (0x6310 + (i) * 0x20) /* TX-side: port-based-forward discard */
#define FWMHLCN(i)  (0x6314 + (i) * 0x20) /* TX-side: hop-limit/loop discard */
#define FWICRDCN(i) (0x6500 + (i) * 0x20) /* RX-side: ingress classification discard */
#define FWWMRDCN(i) (0x6504 + (i) * 0x20) /* RX-side: watermark/buffer discard */
#define FWCTRDCN(i) (0x6508 + (i) * 0x20) /* RX-side: cut-through discard */
#define FWPBRDCN(i) (0x6518 + (i) * 0x20) /* RX-side: port-based-forward discard */

/* Constants */
#define RSWITCH3_NUM_AGENTS 15
#define RSWITCH3_NUM_TSNES  8
#define RSWITCH3_TIMEOUT_US CONFIG_ETH_RENESAS_RSWITCH3_TIMEOUT

struct rswitch3_config {
	mm_reg_t base;
	mm_reg_t secure_base;
	const struct device *clock_dev;
};

struct rswitch3_data {
	struct k_mutex lock;
	/* Bitmask of attached TSN-ES devices */
	unsigned long tsnes_attached;
	/* Bitmask of forwarding ports for each TSN-ES */
	uint32_t tsnes_fwd_mask[RSWITCH3_NUM_TSNES];
	/* Internal ETHA port used by each attached TSN-ES */
	uint32_t tsnes_rswitch_port[RSWITCH3_NUM_TSNES];
	/*
	 * Bitmask of external ports that currently report a real PHY link.
	 */
	uint32_t port_link_mask;
	/* Link-state callback (and its user_data) registered by each attached
	 * TSN-ES via attach_tsnes(), invoked from rswitch3_update_forwarding_for_link()
	 */
	rswitch_fwd_link_cb_t tsnes_link_cb[RSWITCH3_NUM_TSNES];
	void *tsnes_link_cb_data[RSWITCH3_NUM_TSNES];
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH3_H_ */
