/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_rsw3_mdio

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/renesas_cpg_mssr.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(renesas_rcar_rsw3_mdio, CONFIG_MDIO_LOG_LEVEL);

#define RSW3_SLEEP_US   1000
#define RSW3_TIMEOUT_US 1000000

/* Registers */
#define RSW3_BASE_SECURE DT_INST_REG_ADDR_BY_NAME(0, base_secure)

#define RSW3_COMA_OFFSET         0x1c000
#define RSW3_ETHA_OFFSET         0x1d000 /* with RMAC */
#define RSW3_ETHA_SIZE           0x02000 /* with RMAC */
#define RSW3_ETHA_BASE_SECURE(n) (RSW3_BASE_SECURE + RSW3_ETHA_OFFSET + (n) * RSW3_ETHA_SIZE)

#define TARO(n) RSW3_ETHA_BASE_SECURE(n)
#define CARO    (RSW3_BASE_SECURE + RSW3_COMA_OFFSET)
#define RMRO    0

/* ETHA registers, n = 0~12 */
#define EAMC(n) (TARO(n) + 0x0000)
#define EAMS(n) (TARO(n) + 0x0004)

/* COMA registers */
#define RRC  (CARO + 0x0004)
#define RCEC (CARO + 0x0008)
#define RCDC (CARO + 0x000c)

/* RMAC Register */
#define MPSM    (RMRO + 0x0000)
#define MPIC    (RMRO + 0x0004)
#define MIOC    (RMRO + 0x0010)
#define MRMAC0  (RMRO + 0x0084)
#define MRMAC1  (RMRO + 0x0088)
#define MRAFC   (RMRO + 0x008c)
#define MRSCE   (RMRO + 0x0090)
#define MRSCP   (RMRO + 0x0094)
#define MLVC    (RMRO + 0x0180)
#define MLBC    (RMRO + 0x0188)
#define MXGMIIC (RMRO + 0x0190)
#define MPCH    (RMRO + 0x0194)
#define MANM    (RMRO + 0x019c)
#define MMIS0   (RMRO + 0x0210)
#define MMIS1   (RMRO + 0x0220)

/* COMA registers define */
#define RRC_RR   BIT(0)
#define RCEC_RCE BIT(16)
#define RCDC_RCD BIT(16)

/* RMAC registers define */
#define RSW3_MDC_HZ        2500000U
#define MPIC_PSMHT_DEFAULT 0x06U
#define MPIC_PSMHT_MASK    GENMASK(27, 24)
#define MPIC_PSMCS_LO_MASK GENMASK(22, 16)
#define MPIC_PSMCS_HI_MASK GENMASK(15, 13)
#define MPIC_PSMCS_PREP(x)                                                                         \
	(FIELD_PREP(MPIC_PSMCS_LO_MASK, (x) & 0x7f) |                                              \
	 FIELD_PREP(MPIC_PSMCS_HI_MASK, ((x) >> 7) & 0x7))

#define MPSM_PRD_MASK GENMASK(31, 16)
#define MPSM_POP_MASK GENMASK(14, 13)
#define MPSM_PRA_MASK GENMASK(12, 8)
#define MPSM_PDA_MASK GENMASK(7, 3)
#define MPSM_MFF      BIT(2)
#define MPSM_PSME     BIT(0)

/* Completion flags */
#define MMIS1_PPRACS BIT(3) /* Post read access complete */
#define MMIS1_PAACS  BIT(2) /* Address access complete */
#define MMIS1_PWACS  BIT(1) /* Write access complete */
#define MMIS1_PRACS  BIT(0) /* Read access complete */
#define MMIS1_MASK   GENMASK(3, 0)

/* ETHA operation modes */
enum rsw3_etha_mode {
	EAMC_OPC_RESET,
	EAMC_OPC_DISABLE,
	EAMC_OPC_CONFIG,
	EAMC_OPC_OPERATION,
};

#define EAMS_OPS_MASK EAMC_OPC_OPERATION

struct renesas_rcar_rsw3_mdio_config {
	DEVICE_MMIO_ROM; /* Must be first */
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	struct rcar_cpg_clk mdc_clk;
	uint8_t channel;
};

struct renesas_rcar_rsw3_mdio_data {
	DEVICE_MMIO_RAM; /* Must be first */
	struct k_mutex mutex;
};

/**
 * @brief Perform a typical read-modify-write operation on a 32-bits register.
 *
 * @param addr register address.
 * @param mask 32-bit mask value.
 * @param value 32-bit value to be written.
 */
static inline void rsw3_update_bits32(mem_addr_t addr, uint32_t mask, uint32_t val)
{
	uint32_t tmp = sys_read32(addr);

	tmp &= ~mask;
	tmp |= val & mask;
	sys_write32(tmp, addr);
}

static int rsw3_get_clock_rate(const struct device *dev, uint32_t *rate)
{
	const struct renesas_rcar_rsw3_mdio_config *config = dev->config;
	int ret;

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("Clock control device is not ready");
		return -ENODEV;
	}

	ret = clock_control_get_rate(config->clock_dev, (clock_control_subsys_t)&config->mdc_clk,
				     rate);
	if (ret) {
		return ret;
	}

	if (*rate == 0U) {
		return -EINVAL;
	}

	return 0;
}

static uint32_t rsw3_mpic_psmcs(uint32_t rate)
{
	uint32_t psmcs;

	psmcs = DIV_ROUND_CLOSEST(rate, RSW3_MDC_HZ * 2U);
	if (psmcs != 0U) {
		psmcs--;
	}

	return psmcs;
}

/**
 * @brief Turn on RSW3 clock
 */
static void rsw3_init(void)
{
	/* Enable clock */
	sys_set_bits(RCEC, RCEC_RCE);
}

/**
 * @brief Enable/disable RSW3 agent clock
 */
static int rsw3_agent_clock_ctrl(uint8_t channel, bool on)
{
	/* Only channels 0-4 are used for MDIO */
	if (channel > 4) {
		LOG_ERR("Invalid channel %d, should be 0-4", channel);
		return -EINVAL;
	}

	if (on) {
		uint32_t val = sys_read32(RCEC);

		if ((val & (RCEC_RCE | BIT(channel))) != (RCEC_RCE | BIT(channel))) {
			sys_set_bits(RCEC, RCEC_RCE | BIT(channel));
		}
	} else {
		sys_set_bits(RCDC, BIT(channel));
	}

	return 0;
}

/**
 * @brief Control RSW3 ETHA operation mode
 */
static int rsw3_etha_change_mode(const struct device *dev, enum rsw3_etha_mode mode)
{
	const struct renesas_rcar_rsw3_mdio_config *config = dev->config;
	int ret;

	/* Only channels 0-4 are used for MDIO */
	if (config->channel > 4) {
		LOG_ERR("Invalid channel %d, should be 0-4", config->channel);
		return -EINVAL;
	}

	if (mode > EAMC_OPC_OPERATION) {
		LOG_ERR("Invalid mode %d", mode);
		return -EINVAL;
	}

	/* Enable clock */
	ret = rsw3_agent_clock_ctrl(config->channel, true);
	if (ret) {
		return ret;
	}

	sys_write32(mode, EAMC(config->channel));

	/* Wait for mode change to complete */
	ret = WAIT_FOR((sys_read32(EAMS(config->channel)) & EAMS_OPS_MASK) == mode, RSW3_TIMEOUT_US,
		       k_usleep(RSW3_SLEEP_US));
	if (!ret) {
		LOG_ERR("Failed to change mode for ETHA%d", config->channel);
		return -ETIMEDOUT;
	}

	/* Disable clock when not in operation */
	if (mode == EAMC_OPC_DISABLE) {
		ret = rsw3_agent_clock_ctrl(config->channel, false);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

/**
 * @brief Common API to read or write to MDIO Bus using Clause 22 access
 */
static int rsw3_mii_access_c22(const struct device *dev, bool is_read, uint8_t prtad, uint8_t regad,
			       uint16_t user_data)
{
	const uint32_t operation = is_read ? MDIO_OP_C22_READ : MDIO_OP_C22_WRITE;
	uint32_t val;
	int ret;

	/* Station Management Mode : Clause 22 */
	sys_clear_bits(DEVICE_MMIO_GET(dev) + MPSM, MPSM_MFF);

	/* Clear all completion flags */
	sys_set_bits(DEVICE_MMIO_GET(dev) + MMIS1, MMIS1_MASK);

	/* Configure C22 operation */
	val = FIELD_PREP(MPSM_PDA_MASK, prtad) | FIELD_PREP(MPSM_PRA_MASK, regad) |
	      FIELD_PREP(MPSM_POP_MASK, operation) | MPSM_PSME;

	/* Set data for write operation */
	if (!is_read) {
		val |= FIELD_PREP(MPSM_PRD_MASK, user_data);
	}

	/* Start PHY access */
	sys_write32(val, DEVICE_MMIO_GET(dev) + MPSM);

	/* Wait for completion */
	ret = WAIT_FOR((sys_read32(DEVICE_MMIO_GET(dev) + MPSM) & MPSM_PSME) == 0, RSW3_TIMEOUT_US,
		       k_usleep(RSW3_SLEEP_US));

	if (!ret) {
		LOG_ERR("Timeout while waiting for MDIO C22 access completion");
		return -ETIMEDOUT;
	}

	if (!is_read) {
		/* Clear write completion flag */
		sys_set_bits(DEVICE_MMIO_GET(dev) + MMIS1, MMIS1_PWACS);
		return 0;
	}

	/* Read data */
	ret = FIELD_GET(MPSM_PRD_MASK, sys_read32(DEVICE_MMIO_GET(dev) + MPSM));
	/* Clear read completion flag */
	sys_set_bits(DEVICE_MMIO_GET(dev) + MMIS1, MMIS1_PRACS);

	return ret;
}

/**
 * @brief Common API to read or write to MDIO Bus using Clause 45 access
 */
static int rsw3_mii_access_c45(const struct device *dev, bool is_read, uint8_t prtad, uint8_t devad,
			       uint16_t regad, uint16_t user_data)
{
	uint32_t val = 0;
	int ret;

	/* Station Management Mode : Clause 45 */
	val |= MPSM_MFF;

	/* Clear all completion flags */
	sys_set_bits(DEVICE_MMIO_GET(dev) + MMIS1, MMIS1_MASK);

	/* C45 address phase */
	val |= FIELD_PREP(MPSM_PDA_MASK, prtad) | FIELD_PREP(MPSM_PRA_MASK, devad) |
	       FIELD_PREP(MPSM_POP_MASK, MDIO_OP_C45_ADDRESS) | FIELD_PREP(MPSM_PRD_MASK, regad) |
	       MPSM_PSME;

	/* Start address phase */
	sys_write32(val, DEVICE_MMIO_GET(dev) + MPSM);

	/* Wait for completion */
	ret = WAIT_FOR((sys_read32(DEVICE_MMIO_GET(dev) + MMIS1) & MMIS1_PAACS) != 0U,
		       RSW3_TIMEOUT_US, k_usleep(RSW3_SLEEP_US));

	if (!ret) {
		LOG_ERR("Timeout while waiting for MDIO C45 address access completion");
		return -ETIMEDOUT;
	}

	/* Clear address completion flag */
	sys_set_bits(DEVICE_MMIO_GET(dev) + MMIS1, MMIS1_PAACS);

	if (is_read) {
		val &= ~(MPSM_POP_MASK | MPSM_PRD_MASK);
		val |= FIELD_PREP(MPSM_POP_MASK, MDIO_OP_C45_READ);

		/* Start read phase */
		sys_write32(val, DEVICE_MMIO_GET(dev) + MPSM);

		/* Wait for completion */
		ret = WAIT_FOR((sys_read32(DEVICE_MMIO_GET(dev) + MMIS1) & MMIS1_PRACS) != 0U,
			       RSW3_TIMEOUT_US, k_usleep(RSW3_SLEEP_US));

		if (!ret) {
			LOG_ERR("Timeout while waiting for MDIO C45 read completion");
			return -ETIMEDOUT;
		}

		/* Read data */
		ret = FIELD_GET(MPSM_PRD_MASK, sys_read32(DEVICE_MMIO_GET(dev) + MPSM));
		/* Clear read completion flag */
		sys_set_bits(DEVICE_MMIO_GET(dev) + MMIS1, MMIS1_PRACS);
	} else {
		val &= ~(MPSM_POP_MASK | MPSM_PRD_MASK);
		val |= FIELD_PREP(MPSM_POP_MASK, MDIO_OP_C45_WRITE) |
		       FIELD_PREP(MPSM_PRD_MASK, user_data);

		/* Start write phase */
		sys_write32(val, DEVICE_MMIO_GET(dev) + MPSM);

		/* Wait for completion */
		ret = WAIT_FOR((sys_read32(DEVICE_MMIO_GET(dev) + MMIS1) & MMIS1_PWACS) != 0U,
			       RSW3_TIMEOUT_US, k_usleep(RSW3_SLEEP_US));

		if (!ret) {
			LOG_ERR("Timeout while waiting for MDIO C45 write completion");
			return -ETIMEDOUT;
		}

		/* Clear write completion flag */
		sys_set_bits(DEVICE_MMIO_GET(dev) + MMIS1, MMIS1_PWACS);
		return 0;
	}

	return ret;
}

/**
 * @brief Read from MDIO Bus using Clause 22 access
 */
static int renesas_rcar_rsw3_mdio_read(const struct device *dev, uint8_t prtad, uint8_t regad,
				       uint16_t *user_data)
{
	struct renesas_rcar_rsw3_mdio_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = rsw3_mii_access_c22(dev, true, prtad, regad, 0);

	if (ret < 0) {
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	*user_data = ret;
	k_mutex_unlock(&data->mutex);

	return 0;
}

/**
 * @brief Write to MDIO Bus using Clause 22 access
 */
static int renesas_rcar_rsw3_mdio_write(const struct device *dev, uint8_t prtad, uint8_t regad,
					uint16_t user_data)
{
	struct renesas_rcar_rsw3_mdio_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = rsw3_mii_access_c22(dev, false, prtad, regad, user_data);

	if (ret < 0) {
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	k_mutex_unlock(&data->mutex);

	return 0;
}

/**
 * @brief Read from MDIO Bus using Clause 45 access
 */
static int renesas_rcar_rsw3_mdio_read_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
					   uint16_t regad, uint16_t *user_data)
{
	struct renesas_rcar_rsw3_mdio_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = rsw3_mii_access_c45(dev, true, prtad, devad, regad, 0);

	if (ret < 0) {
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	*user_data = ret;
	k_mutex_unlock(&data->mutex);

	return 0;
}

/**
 * @brief Write to MDIO Bus using Clause 45 access
 */
static int renesas_rcar_rsw3_mdio_write_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
					    uint16_t regad, uint16_t user_data)
{
	struct renesas_rcar_rsw3_mdio_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = rsw3_mii_access_c45(dev, false, prtad, devad, regad, user_data);

	if (ret < 0) {
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	k_mutex_unlock(&data->mutex);

	return 0;
}

static int renesas_rcar_rsw3_mdio_init(const struct device *dev)
{
	const struct renesas_rcar_rsw3_mdio_config *config = dev->config;
	struct renesas_rcar_rsw3_mdio_data *data = dev->data;
	uint32_t rate;
	int ret;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	ret = k_mutex_init(&data->mutex);
	if (ret) {
		return ret;
	}

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	rsw3_init();

	ret = rsw3_etha_change_mode(dev, EAMC_OPC_DISABLE);
	if (ret) {
		return ret;
	}

	ret = rsw3_etha_change_mode(dev, EAMC_OPC_CONFIG);
	if (ret) {
		return ret;
	}

	/* Clear MPIC value */
	sys_write32(0x0, DEVICE_MMIO_GET(dev) + MPIC);

	ret = rsw3_get_clock_rate(dev, &rate);
	if (ret) {
		return ret;
	}

	/* Enable and configure MDC clock */
	rsw3_update_bits32(DEVICE_MMIO_GET(dev) + MPIC,
			   MPIC_PSMCS_LO_MASK | MPIC_PSMCS_HI_MASK | MPIC_PSMHT_MASK,
			   MPIC_PSMCS_PREP(rsw3_mpic_psmcs(rate)) |
				   FIELD_PREP(MPIC_PSMHT_MASK, MPIC_PSMHT_DEFAULT));

	ret = rsw3_etha_change_mode(dev, EAMC_OPC_OPERATION);
	if (ret) {
		return ret;
	}

	return 0;
}

static DEVICE_API(mdio, renesas_rcar_rsw3_mdio_api) = {
	.read = renesas_rcar_rsw3_mdio_read,
	.write = renesas_rcar_rsw3_mdio_write,
	.read_c45 = renesas_rcar_rsw3_mdio_read_c45,
	.write_c45 = renesas_rcar_rsw3_mdio_write_c45,
};

/**
 * ************************* DRIVER REGISTER SECTION ***************************
 */

#define RENESAS_RCAR_RSW3_MDIO_INIT(inst)                                                          \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static const struct renesas_rcar_rsw3_mdio_config renesas_rcar_rsw3_mdio_config_##inst = { \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
		.channel = DT_INST_PROP(inst, channel),                                            \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.mdc_clk.module = DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, module),                     \
		.mdc_clk.domain = DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, domain),                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
	};                                                                                         \
                                                                                                   \
	static struct renesas_rcar_rsw3_mdio_data renesas_rcar_rsw3_mdio_data_##inst = {};         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, renesas_rcar_rsw3_mdio_init, NULL,                             \
			      &renesas_rcar_rsw3_mdio_data_##inst,                                 \
			      &renesas_rcar_rsw3_mdio_config_##inst, POST_KERNEL,                  \
			      CONFIG_MDIO_INIT_PRIORITY, &renesas_rcar_rsw3_mdio_api);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_RCAR_RSW3_MDIO_INIT)
