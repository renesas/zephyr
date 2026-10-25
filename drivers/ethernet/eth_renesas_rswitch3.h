/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH3_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH3_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ptp_time.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Receive callback. @p buf wraps the frame in its GWCA DMA buffer (no copy)
 * and is owned by the callee: it must be handed to the stack or unref'd.
 * Freeing it gives the ring slot back to hardware. @p ts is the time the port
 * received the frame, or NULL when no valid timestamp came with it.
 */
typedef void (*rswitch_gwca_rx_cb_t)(const struct device *dev, uint32_t port_id,
				     struct net_buf *buf, const struct net_ptp_time *ts,
				     void *user_data);

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
#define EACKSC   0x013c
#define EACKSECN 0x0414 /* checksum error counter, cleared on read */
#define EADQOECN 0x040c /* descriptor queue overflow error counter, cleared on read */
#define MIOC     (RMRO + 0x0010)
#define MPIC     (RMRO + 0x0004)
#define MRMAC0   (RMRO + 0x0084)
#define MRMAC1   (RMRO + 0x0088)
#define MRAFC    (RMRO + 0x008c)
#define MLVC     (RMRO + 0x0180)
#define MTRC     (RMRO + 0x00a4)
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

/* EACKSC (ingress checksum check; erroneous frames are discarded by MFWD) */
#define EACKSC_IP4CKSE  BIT(0)
#define EACKSC_UDPCKSE  BIT(1)
#define EACKSC_TCPCKSE  BIT(2)
#define EACKSC_ICMPCKSE BIT(3)
#define EACKSC_ALL      (EACKSC_IP4CKSE | EACKSC_UDPCKSE | EACKSC_TCPCKSE | EACKSC_ICMPCKSE)

/* MRAFC */
#define RSW3_MRAFC_RX_PROMISC 0x07870787

/* MIOC */
#define MIOC_FORCE_PHY_LINK BIT(5)
#define MIOC_BIT3_SET       BIT(3)

/* MLVC (link verification). */
#define MLVC_PLV      BIT(16)
#define MLVC_PASE     BIT(8)
#define MLVC_LVT_10MS 0x09U /* 10ms wait time, IEEE 802.3 default */

/*
 * MTRC: TRDDE/TRDDP stay 0, so every received frame carries a timestamp of
 * timer DTN. TCTSE/TCTSP stay 0: TX capture is requested per frame (TXC).
 */
#define MTRC_DTN BIT(28)

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

/*
 * FWPBFCSDC(agent, port) is Port-Based-Forwarding Software Destination
 * Configuration.
 */
#define FWPBFCSDC00            0x4b00
#define FWPBFCSDC(agent, port) (FWPBFCSDC00 + (port) * 0x20 + (agent) * 0x04)

/* FWPC0 */
#define FWPC0_MACSDA BIT(20)

/* FWPC1 */
#define FWCP1_LTHFW            GENMASK(31, 16)
#define FWCP1_LTHFW_TO_PORT(p) FIELD_PREP(FWCP1_LTHFW, (p))
#define FWPC1_DDE              BIT(0)

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
#define RSWITCH3_TIMEOUT_US CONFIG_ETH_RENESAS_RSWITCH3_TIMEOUT

/* Descriptor chains per gateway. A port handed to another core names a chain
 * of that core gateway, so this count is needed even when no gateway is built
 * here.
 */
#define RSWITCH3_GWCA_NUM_CHAINS 128 /* AXI_CHAIN_N */

/*
 * GWCA (Gateway CPU Agent) block. GWCA0/GWCA1 are two independent DMA
 * engines sharing this node's register block, at agent indices
 * RSWITCH3_NUM_AGENTS-2 / -1 (13 / 14) respectively (they do share one
 * gwca_gwdis interrupt line, though)
 */
#if defined(CONFIG_ETH_RENESAS_RSWITCH3_GWCA)

#define RSWITCH3_GWCA0_OFFSET 0x37000
#define RSWITCH3_GWCA1_OFFSET 0x39000
#define RSWITCH3_TOP_OFFSET   0x1b000

#define GWMC     0x0000
#define GWMS     0x0004
#define GWMTIRM  0x0100
#define GWARIRM  0x0380
#define GWVCC    0x0130
#define GWTTFC   0x0138
#define GWDCBAC0 0x01e4
#define GWDCBAC1 0x01e8
#define GWMDNC   0x01f0
#define GWTPC0   0x0300
#define GWDCC0   0x0400
#define GWTRC0   0x0200
#define GWDIS0   0x1100
#define GWDIE0   0x1104
#define GWDID0   0x1108
#define GWDQOECN 0x1030 /* RX frames lost: GWCA descriptor queue overflow (read clears) */
#define GWDFECN  0x1038 /* RX frames lost: CPU descriptor ring full (read clears) */
#define GWCKSC   0x013c
#define GWIDPC   0x0b80
#define GWIDC0   0x0c00

/* Timestamp reception */
#define GWTDCAC0(s) (0x0140 + (s) * 8) /* TS chain s address, upper bits */
#define GWTDCAC1(s) (0x0144 + (s) * 8) /* TS chain s address, lower 32 bits */
#define GWTSDCC(t)  (0x0160 + (t) * 4)
#define GWTSDIS     0x1180
#define GWTSDIE     0x1184
#define GWTSDID     0x1188

/* GWTSDCCt: receive the TX timestamps of timer t, into TS chain DCS. */
#define GWTSDCC_TE     BIT(0)
#define GWTSDCC_DCS(s) FIELD_PREP(BIT(1), (s))

#define GWDCC_OFFS(q) (GWDCC0 + (q) * 4)
/*
 * GWTRCi, GWDISi, GWDIEi and GWDIDi hold one bit per descriptor chain, 32
 * chains per register: chain q is bit GWCA_Q_BIT(q) of the register these
 * macros return for q.
 */
#define GWCA_Q_BIT(q) BIT((q) % 32)
#define GWTRC(q)      (GWTRC0 + ((q) / 32) * 4)
#define GWDIS(q)      (GWDIS0 + ((q) / 32) * 0x10)
#define GWDIE(q)      (GWDIE0 + ((q) / 32) * 0x10)
#define GWDID(q)      (GWDID0 + ((q) / 32) * 0x10)
#define GWIDC(q)      (GWIDC0 + (q) * 4)

/* TOP block: routes each GWCA queue's descriptor-empty interrupt to one of
 * RSWITCH3_GWCA_NUM_IRQS physical IRQ lines.
 */
#define TPDEMIMC0            (RSWITCH3_TOP_OFFSET + 0x0100)
#define TPDEMIMC(q)          (TPDEMIMC0 + (q) * 4)
#define TPDEMIMC_GDICM(a, i) (((uint32_t)(i) & 0xf) << ((a) * 4))

/* GWMC/GWMS */
enum rsw3_gwca_mode {
	GWMC_OPC_RESET = 0,
	GWMC_OPC_DISABLE = 1,
	GWMC_OPC_CONFIG = 2,
	GWMC_OPC_OPERATION = 3,
};
#define GWMS_OPS_MASK GWMC_OPC_OPERATION

/* GWMTIRM (multicast table init) */
#define GWMTIRM_MTIOG BIT(0)
#define GWMTIRM_MTR   BIT(1)

/* GWARIRM (AXI RAM init) */
#define GWARIRM_ARIOG BIT(0)
#define GWARIRM_ARR   BIT(1)

/* GWVCC */
#define GWVCC_VEM_SC_TAG (0x3 << 16)

/*
 * GWMDNC (DMA-number config; fixed values required for correct operation).
 * TSDMN + 1 is how many descriptors one timestamp may take. A TS ring slot is
 * reached through the LINKFIX of the previous one, so each timestamp takes 2;
 * leave room above that.
 */
#define RSWITCH3_GWCA_TSDMN 3
#define GWMDNC_TSDMN(n) (((n) << 16) & GENMASK(17, 16))
#define GWMDNC_TXDMN(n) (((n) << 8) & GENMASK(12, 8))
#define GWMDNC_RXDMN(n) ((n) & GENMASK(4, 0))

/* GWTPC0 */
#define GWTPC_PPPL(ipv) BIT(ipv)

/* GWDCC (per-queue descriptor-chain config) */
#define GWDCC_BALR     BIT(24)
#define GWDCC_DCP_MASK GENMASK(18, 16)
#define GWDCC_DCP(p)   FIELD_PREP(GWDCC_DCP_MASK, (p))
#define GWDCC_DQT      BIT(11) /* TX-only */
#define GWDCC_ETS      BIT(9)  /* RX-only: extended-timestamp descriptor format */
#define GWDCC_EDE      BIT(8)

/* GWCKSC */
#define GWCKSC_USMFSPE BIT(31)

/* GWCA IPV */
#define RSWITCH3_GWCA_IPV 0

/* GWCA descriptor formats. */
struct rswitch3_gwca_desc {
	uint16_t size;
	uint8_t type;
	uint8_t addr_hi;
	uint32_t addr_lo;
} __packed;

/* Used for every TX ring*/
struct rswitch3_gwca_tx_desc {
	struct rswitch3_gwca_desc hdr;
	uint32_t meta_lo;
	uint32_t meta_hi;
} __packed;

/* HW-mandated for every RX ring (GWDCC_ETS is set unconditionally for RX) */
struct rswitch3_gwca_rx_desc {
	struct rswitch3_gwca_desc hdr;
	uint32_t meta_lo;
	uint32_t meta_hi;
	uint32_t tsns; /* HW RX timestamp */
	uint32_t tss;  /* HW RX timestamp */
} __packed;

/* TS descriptor: HW writes the TSUN, the ports and the timer into the address field. */
struct rswitch3_gwca_ts_desc {
	struct rswitch3_gwca_desc hdr;
	uint32_t tsns;
	uint32_t tss;
} __packed;

#define RSWITCH3_GWCA_TS_TSUN_MASK GENMASK(7, 0)  /* hdr.addr_lo */
#define RSWITCH3_GWCA_TS_SPN_MASK  GENMASK(11, 8) /* hdr.addr_lo: agent that sent the frame */

/* Descriptor `type` field values */
#define RSWITCH3_GWCA_DESC_TYPE_MASK 0xf0
#define RSWITCH3_GWCA_DESC_FEMPTY_ND 0x30 /* TS: armed, awaiting a timestamp */
#define RSWITCH3_GWCA_DESC_FSINGLE   0x80 /* single-descriptor frame */
#define RSWITCH3_GWCA_DESC_FEMPTY    0x40 /* RX: armed, awaiting HW fill / TX: HW-completed */
#define RSWITCH3_GWCA_DESC_EEMPTY    0xd0 /* SW-owned, free for reuse */
#define RSWITCH3_GWCA_DESC_LINKFIX   0x00 /* linkfix / ring-wrap pointer entry */
#define RSWITCH3_GWCA_DESC_EOS       0xf0 /* end-of-(unused-queue) marker */
#define RSWITCH3_GWCA_DESC_DIE       BIT(3)
#define RSWITCH3_GWCA_DESC_ERR_MASK  GENMASK(2, 0) /* AXIE | DSE | ERR, written back by HW */

/* Descriptor metadata fields */
#define RSWITCH3_GWCA_META_FMT      BIT(2)          /* meta_lo */
#define RSWITCH3_GWCA_META_TXC      BIT(3)          /* meta_lo: capture the TX timestamp */
#define RSWITCH3_GWCA_META_TN(t)    FIELD_PREP(BIT(6), (t))         /* meta_lo: gPTP timer */
#define RSWITCH3_GWCA_META_TSUN(n)  FIELD_PREP(GENMASK(15, 8), (n)) /* meta_lo */
#define RSWITCH3_GWCA_META_L3CC     BIT(24)         /* meta_lo: insert IPv4 header checksum */
#define RSWITCH3_GWCA_META_L4CC     BIT(25)         /* meta_lo: insert TCP/UDP/ICMP checksum */
#define RSWITCH3_GWCA_META_IPV_MASK GENMASK(30, 28) /* meta_lo */
#define RSWITCH3_GWCA_META_IPV(p)   FIELD_PREP(RSWITCH3_GWCA_META_IPV_MASK, (p))
#define RSWITCH3_GWCA_META_SPN_MASK GENMASK(7, 4) /* meta_hi */
#define RSWITCH3_GWCA_META_SPN(p)   FIELD_PREP(RSWITCH3_GWCA_META_SPN_MASK, (p))
#define RSWITCH3_GWCA_META_DV_MASK  GENMASK(31, 16) /* meta_hi */
#define RSWITCH3_GWCA_META_DV(m)    FIELD_PREP(RSWITCH3_GWCA_META_DV_MASK, (m))

/* TS field of an RX or TS descriptor: TSNS[29:0], then the TSV/TSD flags. */
#define RSWITCH3_GWCA_TSNS_MASK GENMASK(29, 0)
#define RSWITCH3_GWCA_TSNS_TSV  BIT(30) /* RX: the frame came with a timestamp */

#define RSWITCH3_GWCA_NUM_IRQS    8
#define RSWITCH3_GWCA_NUM_DI_REGS (RSWITCH3_GWCA_NUM_CHAINS / 32)
#define RSWITCH3_GWCA_ROW_INVALID 0xffU

#define RSWITCH3_GWCA_FRAME_BUF_SIZE 1536
/*
 * RX frames start 2 bytes into their buffer so that, after the 14-byte
 * Ethernet header, the IP header and payload are word aligned: header
 * parsing and the socket's copy to user space then use aligned accesses.
 */
#define RSWITCH3_GWCA_RX_HEADROOM    2
#define RSWITCH3_GWCA_DESC_ALIGN     64 /* >= data cache line */

#ifndef CONFIG_NOCACHE_MEMORY
#define RSWITCH3_GWCA_RAM_SECTION __attribute__((section(".eth_ram")))
#else
#define RSWITCH3_GWCA_RAM_SECTION __nocache
#endif

/*
 * Ring slots: one frame descriptor plus a LINKFIX to the next slot, padded to
 * a full cache line. HW never writes LINKFIX back, so each cache line holds
 * exactly one HW-written descriptor and the CPU's clean/invalidate of one slot
 * can never clobber a write-back to a neighbouring slot.
 */
struct rswitch3_gwca_tx_slot {
	struct rswitch3_gwca_tx_desc desc;
	struct rswitch3_gwca_tx_desc link;
	uint8_t reserved[RSWITCH3_GWCA_DESC_ALIGN - 2 * sizeof(struct rswitch3_gwca_tx_desc)];
} __packed;

struct rswitch3_gwca_rx_slot {
	struct rswitch3_gwca_rx_desc desc;
	struct rswitch3_gwca_rx_desc link;
	uint8_t reserved[RSWITCH3_GWCA_DESC_ALIGN - 2 * sizeof(struct rswitch3_gwca_rx_desc)];
} __packed;

struct rswitch3_gwca_ts_slot {
	struct rswitch3_gwca_ts_desc desc;
	struct rswitch3_gwca_ts_desc link;
	uint8_t reserved[RSWITCH3_GWCA_DESC_ALIGN - 2 * sizeof(struct rswitch3_gwca_ts_desc)];
} __packed;

BUILD_ASSERT(sizeof(struct rswitch3_gwca_tx_slot) == RSWITCH3_GWCA_DESC_ALIGN);
BUILD_ASSERT(sizeof(struct rswitch3_gwca_rx_slot) == RSWITCH3_GWCA_DESC_ALIGN);
BUILD_ASSERT(sizeof(struct rswitch3_gwca_ts_slot) == RSWITCH3_GWCA_DESC_ALIGN);

/*
 * TX timestamps a gateway can wait for at once. The TSUN of a frame is its
 * slot; a timestamp is matched on TSUN and on SPN, the agent of the gateway,
 * because the TS chain also gets the timestamps of frames other agents sent.
 */
#define RSWITCH3_GWCA_TS_SLOTS      64U
/* TS ring slots per gateway; at most RSWITCH3_GWCA_TS_SLOTS are outstanding. */
#define RSWITCH3_GWCA_TS_RING_SIZE  RSWITCH3_GWCA_TS_SLOTS
/* gPTP timers of the switch (PTP_TN), and so TX timestamp chains per GWCA. */
#define RSWITCH3_PTP_TN             2U
/* TS chain of a gateway; every timer it captures with is steered there. */
#define RSWITCH3_GWCA_TS_CHAIN      0
/* A TX timestamp not back after this long is given up. */
#define RSWITCH3_GWCA_TS_TIMEOUT_MS 100U
/* Poll period of the TS ring while timestamps are outstanding. */
#define RSWITCH3_GWCA_TS_POLL_MS    2U
#if defined(CONFIG_DCACHE_LINE_SIZE)
BUILD_ASSERT(RSWITCH3_GWCA_DESC_ALIGN >= CONFIG_DCACHE_LINE_SIZE,
	     "a GWCA ring slot must not share a cache line with its neighbour");
#endif

struct rswitch3_gwca_tx_queue {
	bool used;
	uint32_t index;                     /* GWCA hardware queue index (== row) */
	uint32_t port_id;                   /* owning port's ETHA agent index */
	struct rswitch3_gwca_tx_slot *ring; /* tx_ring_size slots, cyclic */
	uint8_t *bufs; /* [tx_ring_size][RSWITCH3_GWCA_FRAME_BUF_SIZE], this queue's block */
	uint32_t cur;  /* next slot to fill */
};

/*
 * Ring slot a lent RX net_buf came from. The net_buf user data belongs to the
 * network stack (TCP keeps the segment sequence number there), so the slot is
 * kept in a side table indexed by net_buf_id() instead.
 */
struct rswitch3_gwca_rx_buf_meta {
	struct rswitch3_gwca_rx_queue *q;
	uint32_t idx;
};

struct rswitch3_gwca_rx_queue {
	bool used;
	uint32_t index;
	uint32_t port_id;
	struct rswitch3_gwca_rx_slot *ring; /* rx_ring_size slots, cyclic */
	uint8_t *bufs; /* [rx_ring_size][RSWITCH3_GWCA_FRAME_BUF_SIZE], this queue's block */
	/* [ATOMIC_BITMAP_SIZE(rx_ring_size)]: slots whose buffer is lent to the
	 * network stack; re-armed when freed.
	 */
	atomic_t *lent;
	uint32_t cur; /* next slot to poll for a HW-completed frame */
	rswitch_gwca_rx_cb_t rx_cb;
	void *rx_cb_user_data;
};

struct rswitch3_gwca_config {
	const struct device *rswitch_dev;         /* parent switch device */
	struct rswitch3_gwca_tx_slot *tx_rings;   /* [num_ports][tx_ring_size] */
	uint8_t *tx_bufs;                         /* [num_ports][tx_ring_size][FRAME_BUF_SIZE] */
	struct rswitch3_gwca_rx_slot *rx_rings;   /* [num_ports][rx_ring_size] */
	uint8_t *rx_bufs;                         /* [num_ports][rx_ring_size][FRAME_BUF_SIZE] */
	struct rswitch3_gwca_desc *linkfix_table; /* [2 * num_ports] */
	struct rswitch3_gwca_tx_queue *tx_queues; /* [num_ports], row-indexed */
	struct rswitch3_gwca_rx_queue *rx_queues; /* [num_ports], row-indexed */
	atomic_t *rx_lent; /* [num_ports][ATOMIC_BITMAP_SIZE(rx_ring_size)] */
	/* net_buf headers lent to the stack, one per RX ring slot of every row */
	struct net_buf_pool *rx_buf_pool;
	struct rswitch3_gwca_rx_buf_meta *rx_buf_meta; /* indexed by net_buf_id() */
	const uint8_t *port_ids; /* [num_ports]; port_ids[row] == that row's agent index */
	uint8_t num_ports;       /* == ARRAY_SIZE(port_ids); ports that name this gateway */
	uint32_t reg_base;       /* RSWITCH3_GWCA0_OFFSET or RSWITCH3_GWCA1_OFFSET */
	uint8_t agent;           /* forwarding-agent index (13 or 14) among RSWITCH3_NUM_AGENTS */
	uint8_t instance;        /* GWCA sub-instance (0 or 1) for TPDEMIMC_GDICM()/FWPBFCSDC() */
	/* Ring depths, both from this gateway's devicetree node. */
	uint32_t tx_ring_size;
	uint32_t rx_ring_size;
#if defined(CONFIG_ETH_RENESAS_RSWITCH3_PTP)
	/* [RSWITCH3_GWCA_TS_RING_SIZE] TX timestamp ring, if ts_timers != 0 */
	struct rswitch3_gwca_ts_slot *ts_ring;
	/* gPTP timers (bit t = timer t) the ports of this gateway capture with */
	uint8_t ts_timers;
#endif
};

/* Per-gateway GWCA runtime state, reachable via dev->data. */
struct rswitch3_gwca_data {
	const struct device *dev; /* this gateway device */

	uint8_t port_row[RSWITCH3_NUM_AGENTS];
	struct k_sem rx_sem;
	/* Per GWDISi register: GWDIS/GWDIE bits of every RX queue in use */
	uint32_t rx_irq_mask[RSWITCH3_GWCA_NUM_DI_REGS];

	/* TX rings may be fed from several threads; also guards the ts_* fields. */
	struct k_spinlock tx_lock;
	struct k_thread rx_thread_data;

	/* Running totals of the gateway-wide, clear-on-read RX drop counters */
	struct k_spinlock rx_drops_lock;
	uint32_t rx_queue_overflow; /* GWDQOECN */
	uint32_t rx_ring_full;      /* GWDFECN */

#if defined(CONFIG_ETH_RENESAS_RSWITCH3_PTP)
	/* Drains the TS ring: on the TS interrupt, and polled while needed. */
	struct k_work_delayable ts_work;
	/* Frames waiting for their TX timestamp, by TSUN slot. */
	struct net_pkt *ts_pkt[RSWITCH3_GWCA_TS_SLOTS];
	uint32_t ts_sent_ms[RSWITCH3_GWCA_TS_SLOTS]; /* k_uptime_get_32() at send */
	uint32_t ts_inflight;                        /* non-NULL entries of ts_pkt */
	uint32_t ts_next;                            /* next TSUN slot to hand out */
	uint32_t ts_cur;                             /* next TS ring slot to read (ts_work only) */
#endif

	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_RENESAS_RSWITCH3_GWCA_RX_THREAD_STACK_SIZE);
};

#endif /* CONFIG_ETH_RENESAS_RSWITCH3_GWCA */

/* Per-physical-port link-callback context, indexed by ETHA agent id. */
struct rswitch3_phys_port_ctx {
	const struct device *rswitch_dev;
	const struct device *pcs_dev;
	uint32_t port_id;
	/* This agent's GWCA-backed net_if device. */
	const struct device *port_net_dev;
};

struct rswitch3_internal_port {
	const struct device *tsnes_dev;
	uint32_t fwd_mask; /* external agents this internal agent forwards to/from */
};

struct rswitch3_config {
	mm_reg_t base;
	mm_reg_t secure_base;
	/* Merged gateway interrupt mask of the Cortex-R domain. */
	mm_reg_t intc_gwca_cr_mask;
	const struct device *clock_dev;
#if defined(CONFIG_ETH_RENESAS_RSWITCH3_GWCA)
	const struct device *const *gwca_dev; /* [num_gwca] gateway devices of this switch */
	uint8_t num_gwca;
#endif
	rcar_clk_t mod_clk;
};

struct rswitch3_data {
	struct k_mutex lock;
	/* Per-agent state for internal (TSN-ES-connected) rswitch3-port
	 * nodes
	 */
	struct rswitch3_internal_port internal_ports[RSWITCH3_NUM_AGENTS];
	/*
	 * Bitmask of external ports that currently report a real PHY link.
	 */
	uint32_t port_link_mask;
	/* Agents a port forwards to once its link comes up, indexed by port id.
	 * Holds the gateway of this core, or the agents of another core in
	 * parallel mode. Zero when no agent forwards to the port.
	 */
	uint32_t fwd_agent_mask[RSWITCH3_NUM_AGENTS];
	/* Link-callback context for every physical port, indexed by agent id. */
	struct rswitch3_phys_port_ctx phys_ports[RSWITCH3_NUM_AGENTS];
	/* NET_EVENT_IF_ADMIN_UP callback, replays link state onto TSN-ES net_ifs */
	struct net_mgmt_event_callback iface_event_cb;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_RENESAS_RSWITCH3_H_ */
