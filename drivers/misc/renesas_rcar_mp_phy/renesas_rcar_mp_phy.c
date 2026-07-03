/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <soc.h>
#include <zephyr/drivers/misc/renesas_rcar_mp_phy/renesas_rcar_mp_phy.h>

#define DT_DRV_COMPAT renesas_rcar_mp_phy

/* Required by DEVICE_MMIO_NAMED_* macros */
#define DEV_CFG(_dev)  ((const struct mp_phy_renesas_rcar_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct mp_phy_renesas_rcar_data *)(_dev)->data)

/* Common util */
#define MAX_WAIT_LOOPS 10000000

/* Common registers offsets */
#define MP_PHY_CMNCNT1  0x80000
#define MP_PHY_CMNCNT2  0x80004
#define MP_PHY_PCS0REG1 0x85000
#define MP_PHY_PCS0REG5 0x85010

/* Channel register base and offsets */
#define MP_PHY_CHAN_BASE(ch)  (0x81000 + (ch) * 0x1000)
#define MP_PHY_PXTEST_OFFSET  0x00C
#define MP_PHY_RXCNT_OFFSET   0x038
#define MP_PHY_SRAMCNT_OFFSET 0x040
#define MP_PHY_REFCLK_OFFSET  0x014
#define MP_PHY_CNTXT1_OFFSET  0x004
#define MP_PHY_CNTXT2_OFFSET  0x008
#define MP_PHY_TXREQ_OFFSET   0x044
#define MP_PHY_RXREQ1_OFFSET  0x024
#define MP_PHY_CNTXTE_OFFSET  0x10C

/* Channel specific registers */
#define MP_PHY_PXTEST(ch)    (MP_PHY_CHAN_BASE(ch) + MP_PHY_PXTEST_OFFSET)
#define MP_PHY_PXRXCNT(ch)   (MP_PHY_CHAN_BASE(ch) + MP_PHY_RXCNT_OFFSET)
#define MP_PHY_PXSRAMCNT(ch) (MP_PHY_CHAN_BASE(ch) + MP_PHY_SRAMCNT_OFFSET)
#define MP_PHY_PXREFCLK(ch)  (MP_PHY_CHAN_BASE(ch) + MP_PHY_REFCLK_OFFSET)
#define MP_PHY_PXCNTXT1(ch)  (MP_PHY_CHAN_BASE(ch) + MP_PHY_CNTXT1_OFFSET)
#define MP_PHY_PXCNTXT2(ch)  (MP_PHY_CHAN_BASE(ch) + MP_PHY_CNTXT2_OFFSET)
#define MP_PHY_PXTXREQ(ch)   (MP_PHY_CHAN_BASE(ch) + MP_PHY_TXREQ_OFFSET)
#define MP_PHY_PXRXREQ1(ch)  (MP_PHY_CHAN_BASE(ch) + MP_PHY_RXREQ1_OFFSET)
#define MP_PHY_PXCNTXTE(ch)  (MP_PHY_CHAN_BASE(ch) + MP_PHY_CNTXTE_OFFSET)

/* Channel enable bit masks for MP_PHY_CMNCNT1 register */
#define MP_PHY_CMNCNT1_CH_MASK(ch) (0xFF << ((ch) * 8))

/* Channel enable bits for MP_PHY_CMNCNT1 register */
#define MP_PHY_CMNCNT1_ETH_EN(ch) (BIT((ch) * 8))
#define MP_PHY_CMNCNT1_USB_EN(ch)                                                                  \
	((ch) == 2 ? (BIT(16) | BIT(17)) : (ch) == 3 ? (BIT(24) | BIT(25)) : -1)
#define MP_PHY_CMNCNT1_PCI0_EN(ch) (0x00 << ((ch) * 8))
#define MP_PHY_CMNCNT1_PCI1_EN(ch) (0x02 << ((ch) * 8))

/* Channel resistor and clock bits for MP_PHY_CMNCNT2 register */
#define MP_PHY_CMNCNT2_RES_REQ_NO_SHARE(channel) BIT(channel * 4 + 16)
#define MP_PHY_CMNCNT2_RES_ACK_NO_SHARE(channel) BIT(channel * 4 + 17)
#define MP_PHY_CMNCNT2_RES_DEFAULT_SETTING                                                         \
	(MP_PHY_CMNCNT2_RES_REQ_NO_SHARE(0) | MP_PHY_CMNCNT2_RES_ACK_NO_SHARE(0) |                 \
	 MP_PHY_CMNCNT2_RES_REQ_NO_SHARE(1) | MP_PHY_CMNCNT2_RES_ACK_NO_SHARE(1) |                 \
	 MP_PHY_CMNCNT2_RES_REQ_NO_SHARE(2) | MP_PHY_CMNCNT2_RES_ACK_NO_SHARE(2) |                 \
	 MP_PHY_CMNCNT2_RES_REQ_NO_SHARE(3) | MP_PHY_CMNCNT2_RES_ACK_NO_SHARE(3))

#define MP_PHY_CMNCNT2_CLK_USE_PAD(channel)   BIT(channel * 4)
#define MP_PHY_CMNCNT2_REPEAT_CLK_EN(channel) BIT(channel * 4 + 1)

/* PCS0REG5 register mask and values for each channel */
#define MP_PHY_PCS0REG5_CH(ch) (0x03 << (24 + (ch) * 2))

/* PCS0REG1 register bits */
#define MP_PHY_PCS0REG1_VAL 0x00010000
/* PCS0REG2 register bits */
#define MP_PHY_PCS0REG5_VAL 0xff000000

/* PXTEST register bit */
#define MP_PHY_PXTEST_BIT 0x1

/* PXRXCNT register reset value */
#define MP_PHY_PXRXCNT_RESET_VAL 0x202

/* PXSRAMCNT register bits */
#define MP_PHY_PXSRAMCNT_BYPASS BIT(0)
#define MP_PHY_PXSRAMCNT_BIT3   BIT(3)
#define BOOTLOAD_BYPASS_MODE    0x3
#define SRAM_BYPASS_MODE        0xC
#define SRAM_EXT_LD_DONE        0x10
#define SRAM_INIT_DONE          0x20

#define SRAM_CONTROL_SET_BIT                                                                       \
	(BOOTLOAD_BYPASS_MODE | SRAM_BYPASS_MODE | SRAM_EXT_LD_DONE | SRAM_INIT_DONE)

/* PXSRAMCNT Interface values for each interface mode */
#define MP_PHY_PXSRAMCNT_ETH (BOOTLOAD_BYPASS_MODE | SRAM_BYPASS_MODE)
#define MP_PHY_PXSRAMCNT_USB (MP_PHY_PXSRAMCNT_BYPASS | MP_PHY_PXSRAMCNT_BIT3)
#define MP_PHY_PXSRAMCNT_PCI (BOOTLOAD_BYPASS_MODE | SRAM_BYPASS_MODE)

/* RXREQ1 flag */
#define MP_PHY_PXRXREQ1_INIT_BUSY_MASK BIT(1)
#define MP_PHY_PXRXREQ1_INIT_IDLE      0
#define MP_PHY_PXRXREQ1_INIT_BUSY      1

/*-------------------------------------- TCA define -------------------------------*/
/* TCA (Type-C Adapter) Register Offsets within MP-PHY base */
#define MP_PHY_USB_BASE(ch) (0x90000 + (ch) * 0x10000)

/* Channel specific registers */
#define TCA_INTR_OFFSET(ch)      (MP_PHY_USB_BASE(ch) + 0x4)
#define TCA_INTR_STS_OFFSET(ch)  (MP_PHY_USB_BASE(ch) + 0x8)
#define TCA_TCPC_OFFSET(ch)      (MP_PHY_USB_BASE(ch) + 0x14)
#define TCA_VBUS_CTRL_OFFSET(ch) (MP_PHY_USB_BASE(ch) + 0x0040)
#define PSTATE_1_OFFSET(ch)      (MP_PHY_USB_BASE(ch) + 0x0054)

#define VBUS_VALID_OVERRD 0x2
/*---------------------------------------------------------------------------------*/

#define PHY_MODE_USB(mode)                                                                         \
	((mode) == PHY_MODE_USB_HOST     ? PHY_MODE_USB_HOST                                       \
	 : (mode) == PHY_MODE_USB_DEVICE ? PHY_MODE_USB_DEVICE                                     \
	 : (mode) == PHY_MODE_USB_OTG    ? PHY_MODE_USB_OTG                                        \
					 : -1)

/* PXREFCLK register value */
#define MP_PHY_PXREFCLK_VAL     0x35
#define MP_PHY_PXREFCLK_VAL_ETH 0x55

/* PXRXCNT register value */
#define MP_PHY_PXRXCNT_VAL_ETH (BIT(9) | BIT(1))

/* PXTXREQ register value */
#define MP_PHY_PXTXREQ_VAL     0x8
#define MP_PHY_PXTXREQ_VAL_ETH (BIT(19) | BIT(3))

/* Context settings */
#define MP_PHY_CNTXT1_VALUE     0x02010002
#define MP_PHY_CNTXT2_VALUE     0x02020202 /* For channels 1-3 */
#define MP_PHY_CNTXT2_CH0_VALUE 0x02020201 /* Special for channel 0 */
#define MP_PHY_TXREQ_VALUE      0x8

/* Ethernet context */
#define MP_PHY_CNTXTE_ETH 0x000FF0FF
#define MP_PHY_CNTXT1_ETH 0x00180023

#define HIGH_SPEED       0
#define SUPER_SPEED_PLUS 1

enum mp_phy_iface {
	MP_PHY_IF_PCI0,
	MP_PHY_IF_PCI1,
	MP_PHY_IF_ETHERNET,
	MP_PHY_IF_USB,
};

struct mp_phy_channel_config {
	enum mp_phy_iface if_type;
};

struct mp_phy_renesas_rcar_config {
	DEVICE_MMIO_NAMED_ROM(reg_base);
	struct mp_phy_channel_config chan_cfg[CONFIG_RENESAS_RCAR_MP_PHY_NUM_CHANNELS];
	const struct device *clock_dev;
	struct rcar_cpg_clk *p_mod_clk;
	uint8_t num_mod_clk;
	uint8_t use_pad_clk;
	uint8_t repeat_clk_en;
};

struct mp_phy_renesas_rcar_data {
	DEVICE_MMIO_NAMED_RAM(reg_base);
};

static inline uint32_t mp_phy_renesas_rcar_read(const struct device *dev, uint32_t offs)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, reg_base) + offs);
}

static inline void mp_phy_renesas_rcar_write(const struct device *dev, uint32_t offs,
					     uint32_t value)
{
	sys_write32(value, DEVICE_MMIO_NAMED_GET(dev, reg_base) + offs);
}

static void mp_phy_renesas_rcar_update_bits(const struct device *dev, uint32_t offs, uint32_t mask,
					    uint32_t value)
{
	uint32_t temp;

	temp = mp_phy_renesas_rcar_read(dev, offs);
	temp = (temp & ~mask) | (value & mask);
	mp_phy_renesas_rcar_write(dev, offs, temp);
}

static int mp_phy_renesas_rcar_wait_bits(const struct device *dev, uint32_t offs, uint32_t mask,
					 uint32_t expected)
{
	uint32_t val;
	uint32_t loop_count = 0;

	do {
		val = mp_phy_renesas_rcar_read(dev, offs);
		loop_count++;
	} while (((val & mask) != expected) && (loop_count != MAX_WAIT_LOOPS));

	if ((val & mask) == expected) {
		return 0;
	}

	return -ETIME;
}

static int mp_phy_init_ethernet(const struct device *dev, uint32_t channel_id)
{
	mp_phy_renesas_rcar_write(dev, MP_PHY_PXRXCNT(channel_id), MP_PHY_PXRXCNT_RESET_VAL);
	mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PXREFCLK(channel_id), MP_PHY_PXREFCLK_VAL_ETH,
					MP_PHY_PXREFCLK_VAL_ETH);
	mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PXRXCNT(channel_id), MP_PHY_PXRXCNT_VAL_ETH,
					~MP_PHY_PXRXCNT_VAL_ETH);
	mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PXTXREQ(channel_id), MP_PHY_PXTXREQ_VAL_ETH,
					MP_PHY_PXTXREQ_VAL_ETH);

	return 0;
}

static int mp_phy_power_on(const struct device *dev, struct mp_phy_renesas_rcar_cfg phy_cfg)
{
	int ret;

	if (phy_cfg.type == MP_PHY_TYPE_ETH) {
		mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PXSRAMCNT(phy_cfg.channel),
						SRAM_EXT_LD_DONE, SRAM_EXT_LD_DONE);
		ret = mp_phy_renesas_rcar_wait_bits(dev, MP_PHY_PXRXREQ1(phy_cfg.channel),
						    MP_PHY_PXRXREQ1_INIT_BUSY,
						    MP_PHY_PXRXREQ1_INIT_IDLE);
		if (ret < 0) {
			return ret;
		}
	} else {
		return -ENOTSUP;
	}

	return 0;
}

int mp_phy_renesas_rcar_enable(const struct device *dev, struct mp_phy_renesas_rcar_cfg phy_cfg)
{
	const struct mp_phy_renesas_rcar_config *config = dev->config;
	uint8_t if_type;
	int ret;

	if (phy_cfg.channel >= CONFIG_RENESAS_RCAR_MP_PHY_NUM_CHANNELS) {
		return -EINVAL;
	}

	if_type = config->chan_cfg[phy_cfg.channel].if_type;
	/* Validate MP-PHY interface type and configuration */
	switch (phy_cfg.type) {
	case MP_PHY_TYPE_PCIE:
		if ((if_type != MP_PHY_IF_PCI0) && (if_type != MP_PHY_IF_PCI1)) {
			return -EINVAL;
		}
		return -ENOTSUP;
	case MP_PHY_TYPE_ETH:
		if (if_type != MP_PHY_IF_ETHERNET) {
			return -EINVAL;
		}
		ret = mp_phy_init_ethernet(dev, phy_cfg.channel);
		if (ret < 0) {
			return ret;
		}
		break;
	case MP_PHY_TYPE_USB:
		if (if_type != MP_PHY_IF_USB) {
			return -EINVAL;
		}
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	ret = mp_phy_power_on(dev, phy_cfg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int mp_phy_renesas_rcar_init(const struct device *dev)
{
	const struct mp_phy_renesas_rcar_config *config = dev->config;
	uint32_t sramcnt[CONFIG_RENESAS_RCAR_MP_PHY_NUM_CHANNELS];
	uint32_t cmncnt1 = 0;
	uint32_t cmncnt2 = MP_PHY_CMNCNT2_RES_DEFAULT_SETTING;
	uint32_t val;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	DEVICE_MMIO_NAMED_MAP(dev, reg_base, K_MEM_CACHE_NONE);

	/* Power ON the MP-PHYs, keep the reset asserted */
	for (uint32_t i = 0; i < config->num_mod_clk; i++) {
		int ret = clock_control_on(config->clock_dev,
					(clock_control_subsys_t)&config->p_mod_clk[i]);
		if (ret < 0) {
			return ret;
		}
	}

	for (int i = 0; i < CONFIG_RENESAS_RCAR_MP_PHY_NUM_CHANNELS; i++) {
		if (config->chan_cfg[i].if_type == MP_PHY_IF_ETHERNET) {
			sramcnt[i] = MP_PHY_PXSRAMCNT_ETH;
			cmncnt1 |= MP_PHY_CMNCNT1_ETH_EN(i);
		} else if (config->chan_cfg[i].if_type == MP_PHY_IF_PCI0) {
			sramcnt[i] = MP_PHY_PXSRAMCNT_PCI;
			cmncnt1 |= MP_PHY_CMNCNT1_PCI0_EN(i);
		} else if (config->chan_cfg[i].if_type == MP_PHY_IF_PCI1) {
			sramcnt[i] = MP_PHY_PXSRAMCNT_PCI;
			cmncnt1 |= MP_PHY_CMNCNT1_PCI1_EN(i);
		} else if (config->chan_cfg[i].if_type == MP_PHY_IF_USB) {
			sramcnt[i] = MP_PHY_PXSRAMCNT_USB;
			cmncnt1 |= MP_PHY_CMNCNT1_USB_EN(i);
		} else {
			return -EINVAL;
		}

		if (config->use_pad_clk & BIT(i)) {
			cmncnt2 |= MP_PHY_CMNCNT2_CLK_USE_PAD(i);
		}

		if (config->repeat_clk_en & BIT(i)) {
			cmncnt2 |= MP_PHY_CMNCNT2_REPEAT_CLK_EN(i);
		}
	}

	mp_phy_renesas_rcar_update_bits(dev, MP_PHY_CMNCNT1, cmncnt1, cmncnt1);
	mp_phy_renesas_rcar_write(dev, MP_PHY_CMNCNT2, cmncnt2);

	for (int i = 0; i < CONFIG_RENESAS_RCAR_MP_PHY_NUM_CHANNELS; i++) {
		/* Hold reset of the PHYi */
		mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PXTEST(i), MP_PHY_PXTEST_BIT,
						MP_PHY_PXTEST_BIT);

		/* Set SRAM boot mode */
		mp_phy_renesas_rcar_write(dev, MP_PHY_PXSRAMCNT(i), sramcnt[i]);

		/* Release the reset */
		mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PXTEST(i), MP_PHY_PXTEST_BIT,
						~MP_PHY_PXTEST_BIT);
	}

	/* Assert UPCS */
	mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PCS0REG1, MP_PHY_PCS0REG1_VAL,
					~MP_PHY_PCS0REG1_VAL);
	mp_phy_renesas_rcar_update_bits(dev, MP_PHY_PCS0REG5, MP_PHY_PCS0REG5_VAL,
					~MP_PHY_PCS0REG5_VAL);

	return 0;
};

#define MP_PHY_CLOCKS_GET(node_id, prop, idx)                                                      \
	{                                                                                          \
		.module = DT_CLOCKS_CELL_BY_IDX(node_id, idx, module),                             \
		.domain = DT_CLOCKS_CELL_BY_IDX(node_id, idx, domain),                             \
	}

#define _MP_PHY_GET_INTERFACE(if_name) MP_PHY_IF_##if_name
#define MP_PHY_GET_INTERFACE(if_name)  _MP_PHY_GET_INTERFACE(if_name)

#define MP_PHY_GET_CHANNEL_CONFIG(node_id, prop, idx)                                              \
	{                                                                                          \
		.if_type = MP_PHY_GET_INTERFACE(DT_STRING_UPPER_TOKEN_BY_IDX(node_id, prop, idx)), \
	}

#define DUMMY_CHAR

#define MP_PHY_DEVICE_INIT(n)                                                                      \
	static struct rcar_cpg_clk mod_clk##n[DT_INST_NUM_CLOCKS(n)] = {                           \
		DT_INST_FOREACH_PROP_ELEM_SEP(n, clocks, MP_PHY_CLOCKS_GET, (, DUMMY_CHAR)),       \
	};                                                                                         \
                                                                                                   \
	static const struct mp_phy_renesas_rcar_config mp_phy_config##n = {                        \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(n)),                              \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.p_mod_clk = mod_clk##n,                                                           \
		.num_mod_clk = DT_INST_NUM_CLOCKS(n),                                              \
		.chan_cfg =                                                                        \
			{                                                                          \
				DT_INST_FOREACH_PROP_ELEM_SEP(n, interface_names,                  \
							      MP_PHY_GET_CHANNEL_CONFIG,           \
							      (, DUMMY_CHAR)),                     \
			},                                                                         \
		.use_pad_clk = DT_INST_PROP(n, ref_use_pad),                                       \
		.repeat_clk_en = DT_INST_PROP(n, ref_repeat_clk_en),                               \
	};                                                                                         \
                                                                                                   \
	static struct mp_phy_renesas_rcar_data mp_phy_data##n;                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, mp_phy_renesas_rcar_init, NULL, &mp_phy_data##n,                  \
			      &mp_phy_config##n, PRE_KERNEL_1,                                     \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(MP_PHY_DEVICE_INIT)
