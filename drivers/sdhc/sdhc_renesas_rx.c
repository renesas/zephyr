/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_sdhc

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/interrupt_controller/intc_renesas_rx_grp_int_fsp.h>

/* Renesas include */
#include "sdhc_renesas_rx.h"
#include "r_sdhi.h"
#include "r_dtc.h"
#include "r_sdhi_private.h"

LOG_MODULE_REGISTER(sdhc_renesas_rx, CONFIG_SDHC_LOG_LEVEL);

/*
 * The extern functions below are implemented in the r_sdhi.c source file.
 * For more information, please refer to r_sdhi.c in HAL Renesas
 */
extern fsp_err_t r_sdhi_transfer_write(sdhi_instance_ctrl_t *const p_ctrl, uint32_t block_count,
				       uint32_t bytes, const uint8_t *p_data);
extern fsp_err_t r_sdhi_transfer_read(sdhi_instance_ctrl_t *const p_ctrl, uint32_t block_count,
				      uint32_t bytes, void *p_data);
extern fsp_err_t r_sdhi_max_clock_rate_set(sdhi_instance_ctrl_t *p_ctrl, uint32_t max_rate);
extern fsp_err_t r_sdhi_hw_cfg(sdhi_instance_ctrl_t *const p_ctrl);
extern fsp_err_t r_sdhi_read_and_block(sdhi_instance_ctrl_t *const p_ctrl, uint32_t command,
				       uint32_t argument, uint32_t byte_count);
extern fsp_err_t r_sdhi_wait_for_device(sdhi_instance_ctrl_t *const p_ctrl);
extern fsp_err_t r_sdhi_wait_for_event(sdhi_instance_ctrl_t *const p_ctrl, uint32_t bit,
				       uint32_t timeout);
extern void r_sdhi_command_send_no_wait(sdhi_instance_ctrl_t *p_ctrl, uint32_t command,
					uint32_t argument);
extern void r_sdhi_read_write_common(sdhi_instance_ctrl_t *const p_ctrl, uint32_t sector_count,
				     uint32_t sector_size, uint32_t command, uint32_t argument);
extern void sdhimmc_accs_isr(void);
extern void sdhimmc_card_isr(void);
extern void sdhimmc_dma_req_isr(void);

struct sdhc_rx_config {
	const struct pinctrl_dev_config *pcfg;
	struct sdhc_host_props props;
	void *const regs;
};

struct sdhc_rx_data {
	sdhi_instance_ctrl_t sdmmc_ctrl;
	sdmmc_cfg_t fsp_config;
	struct gpio_dt_spec sdhi_en;
	struct sdmmc_rx_event sdmmc_event;
	struct k_sem thread_lock;
	enum sdhc_timing_mode timing;
	enum sdhc_power power_mode;
	bool app_cmd;
	uint8_t status;
	uint8_t bus_width;
	uint32_t bus_clock;

	/* Group interrupt controller */
	const struct device *sdhc_intc;
	unsigned int card_num;
	unsigned int accs_num;

	/* Transfer DTC */
	struct st_transfer_info transfer_info DTC_TRANSFER_INFO_ALIGNMENT;
	transfer_instance_t transfer;
	dtc_instance_ctrl_t transfer_ctrl;
	transfer_cfg_t transfer_cfg;
	dtc_extended_cfg_t transfer_cfg_extend;

	/* User data*/
	sdhc_interrupt_cb_t sdhc_cb;
	void *sdhc_cb_user_data;

	bool card_irq_enabled;
};

static void rx_sdmmc_dma_req_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	sdhimmc_dma_req_isr();
}

static int sdhc_rx_get_card_present(const struct device *dev)
{
	struct sdhc_rx_data *priv = dev->data;
	fsp_err_t fsp_err;
	sdmmc_status_t status;

	/* SDMMC_CARD_DETECT_CD must be configured as true to check here */
	fsp_err = R_SDHI_StatusGet(&priv->sdmmc_ctrl, &status);
	if (fsp_err != 0) {
		LOG_ERR("Failed to get status for card presence");
		return -EIO;
	}

	return (status.card_inserted);
}

static int sdhc_rx_card_busy(const struct device *dev)
{
	struct sdhc_rx_data *priv = dev->data;
	fsp_err_t fsp_err;
	sdmmc_status_t status;

	fsp_err = R_SDHI_StatusGet(&priv->sdmmc_ctrl, &status);
	if (fsp_err != 0) {
		LOG_ERR("Failed to get status for card busy");
		return -EIO;
	}

	return (status.transfer_in_progress);
}

static int sdhi_command_send_wait(sdhi_instance_ctrl_t *p_ctrl, uint32_t command, uint32_t argument,
				  uint32_t timeout_ms)
{
	fsp_err_t fsp_err = r_sdhi_wait_for_device(p_ctrl);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("Card is busy");
		return -EIO;
	}

	/* Convert timeout to us */
	uint32_t timeout_us = timeout_ms * 1000U;

	/* Send the command. */
	r_sdhi_command_send_no_wait(p_ctrl, command, argument);
	/* Wait for end of response, error or timeout */
	return r_sdhi_wait_for_event(p_ctrl, SDHI_PRV_RESPONSE_BIT, timeout_us);
}

static int sdhc_rx_send_cmd(struct sdhc_rx_data *priv, struct sdmmc_rx_command *rx_cmd, int retries)
{
	int err = 0;

	while (retries > 0) {
		err = sdhi_command_send_wait(&priv->sdmmc_ctrl, rx_cmd->opcode, rx_cmd->arg,
					     rx_cmd->timeout_ms);
		if (err != 0) {
			retries--; /* error, retry */
		} else {
			break;
		}
	}

	return err;
}

/*
 * Send CMD or CMD/DATA via SDHC
 */
static int sdhc_rx_request(const struct device *dev, struct sdhc_command *cmd,
			   struct sdhc_data *data)
{
	struct sdhc_rx_data *priv = dev->data;
	fsp_err_t fsp_err;
	sdmmc_priv_csd_reg_t p_csd_reg;
	/* Timeout configuration for command and data */
	uint32_t timeout_cfg = 0;
	int retries = (int)(cmd->retries + 1); /* first try plus retries */
	int ret = 0;

	struct sdmmc_rx_command rx_cmd = {
		.opcode = cmd->opcode,
		.arg = cmd->arg,
	};

	if (data) {
		rx_cmd.data = (uint8_t *)data->data;
		rx_cmd.sector_count = data->blocks;
		rx_cmd.sector_size = data->block_size;
		timeout_cfg = data->timeout_ms;
	} else {
		timeout_cfg = cmd->timeout_ms;
	}

	if (cmd->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		rx_cmd.timeout_ms = SDHI_TIME_OUT_MAX;
	} else {
		rx_cmd.timeout_ms = timeout_cfg;
	}

	k_sem_reset(&priv->sdmmc_event.transfer_sem);
	k_sem_take(&priv->thread_lock, K_FOREVER);

	/*
	 * Handle opcode with RX specifics
	 */
	switch (cmd->opcode) {
	case SD_GO_IDLE_STATE:
	case SD_ALL_SEND_CID:
	case SD_SEND_RELATIVE_ADDR:
	case SD_SELECT_CARD:
	case SD_SEND_IF_COND:
	case SD_SET_BLOCK_SIZE:
	case SD_ERASE_BLOCK_START:
	case SD_ERASE_BLOCK_END:
	case SD_ERASE_BLOCK_OPERATION:
	case SD_APP_CMD:
	case SD_SEND_STATUS:
		/* Send command with argument */
		ret = sdhc_rx_send_cmd(priv, &rx_cmd, retries);
		if (ret < 0) {
			goto end;
		}
		break;
	case SD_SEND_CSD:
		/* Read card specific data register */
		ret = sdhc_rx_send_cmd(priv, &rx_cmd, retries);
		if (ret < 0) {
			goto end;
		}
		/* SDResponseR2 are bits from 8-127, first 8 MSBs are reserved */
		p_csd_reg.reg.sdrsp10 = priv->sdmmc_ctrl.p_reg->SD_RSP10;
		p_csd_reg.reg.sdrsp32 = priv->sdmmc_ctrl.p_reg->SD_RSP32;
		p_csd_reg.reg.sdrsp54 = priv->sdmmc_ctrl.p_reg->SD_RSP54;
		p_csd_reg.reg.sdrsp76 = priv->sdmmc_ctrl.p_reg->SD_RSP76;

		/* Get the CSD version. */
		uint32_t csd_version = (uint32_t)(p_csd_reg.csd_v1_b.csd_structure);
		uint32_t mult;
		if ((SDHI_PRV_CSD_VERSION_1_0 == csd_version) ||
		    (SDMMC_CARD_TYPE_MMC == priv->sdmmc_ctrl.device.card_type)) {
			mult = (1U << (p_csd_reg.csd_v1_b.c_size_mult + 2));
			uint32_t c_size = (uint32_t)((p_csd_reg.csd_v1_b.c_size_u << 2) |
						     p_csd_reg.csd_v1_b.c_size_l);
			priv->sdmmc_ctrl.device.sector_count = (uint32_t)((c_size + 1U) * mult);

			/* Scale the sector count by the actual block size. */
			uint32_t read_sector_size = 1U << p_csd_reg.csd_v1_b.read_bl_len;
			priv->sdmmc_ctrl.device.sector_count =
				priv->sdmmc_ctrl.device.sector_count *
				(read_sector_size / SDHI_MAX_BLOCK_SIZE);

			if (SDMMC_CARD_TYPE_MMC == priv->sdmmc_ctrl.device.card_type) {
				/* If c_size is 0xFFF, then sector_count should be obtained from the
				 * extended CSD. Set it to 0 to indicate it should come from the
				 * extended CSD later. */
				if (SDHI_PRV_SECTOR_COUNT_IN_EXT_CSD == c_size) {
					priv->sdmmc_ctrl.device.sector_count = 0U;
				}
			}
		}

#if SDHI_CFG_SD_SUPPORT_ENABLE
		else if (SDHI_PRV_CSD_VERSION_2_0 == csd_version) {
			priv->sdmmc_ctrl.device.sector_count =
				(uint32_t)((p_csd_reg.csd_v2_b.c_size + 1U) *
					   SDHI_PRV_BYTES_PER_KILOBYTE);
		} else {
			/* Do Nothing */
		}

		if (SDHI_PRV_CSD_VERSION_1_0 == csd_version) {
			/* Get the minimum erasable unit (in 512 byte sectors). */
			priv->sdmmc_ctrl.device.erase_sector_count =
				(uint32_t)(p_csd_reg.csd_v1_b.sector_size + 1U);
		} else
#endif
		{
			/*
			 * For SDHC and SDXC cards, there are no erase group restrictions.
			 * Using the eMMC TRIM operation, there are no erase group restrictions.
			 */
			priv->sdmmc_ctrl.device.erase_sector_count = 1U;
		}
		break;
	case SD_APP_SEND_OP_COND:
		rx_cmd.opcode |= SDHI_PRV_CMD_C_ACMD;
		ret = sdhc_rx_send_cmd(priv, &rx_cmd, retries);
		if (ret < 0) {
			goto end;
		}
		sdmmc_response_t response;
		/* get response of ACMD41 (R3) */
		response.status = priv->sdmmc_ctrl.p_reg->SD_RSP10;
		/*  Initialization complete? */
		if (response.r3.power_up_status) {
			/* High capacity card ? */
			/*  0 = SDSC, 1 = SDHC or SDXC */
			priv->sdmmc_ctrl.sector_addressing =
				(response.r3.card_capacity_status > 0U);
			priv->sdmmc_ctrl.device.card_type = SDMMC_CARD_TYPE_SD;
		}
		priv->sdmmc_ctrl.initialized = true;
		break;
	case SD_SWITCH:
		/* Check app cmd */
		if (priv->app_cmd && cmd->opcode == SD_APP_SET_BUS_WIDTH) {
			/* ACMD41 */
			rx_cmd.opcode |= SDHI_PRV_CMD_C_ACMD;
			ret = sdhc_rx_send_cmd(priv, &rx_cmd, retries);
			if (ret < 0) {
				goto end;
			}
		} else {
			/* SD SWITCH CMD6 */
			fsp_err = r_sdhi_read_and_block(&priv->sdmmc_ctrl, rx_cmd.opcode,
							rx_cmd.arg, rx_cmd.sector_size);
			if (fsp_err != FSP_SUCCESS) {
				ret = -EIO;
				goto end;
			}
			arch_dcache_flush_and_invd_all();
			memcpy(rx_cmd.data, priv->sdmmc_ctrl.aligned_buff, 8);
			priv->sdmmc_event.transfer_completed = false;
			break;
		}
		break;

	/* Read write with data */
	case SD_APP_SEND_SCR:
		rx_cmd.opcode = cmd->opcode | SDHI_PRV_CMD_C_ACMD;
		fsp_err = r_sdhi_read_and_block(&priv->sdmmc_ctrl, rx_cmd.opcode, rx_cmd.arg,
						rx_cmd.sector_size);

		if (fsp_err != 0) {
			ret = -ETIMEDOUT;
			goto end;
		}
		arch_dcache_flush_and_invd_all();
		memcpy(rx_cmd.data, priv->sdmmc_ctrl.aligned_buff, 8);
		priv->sdmmc_event.transfer_completed = false;
		break;
	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK:
		/* Configure the transfer interface for reading.*/
		fsp_err = r_sdhi_transfer_read(&priv->sdmmc_ctrl, rx_cmd.sector_count,
					       rx_cmd.sector_size, rx_cmd.data);
		if (fsp_err != FSP_SUCCESS) {
			LOG_ERR("Failed to configure read transfer");
			ret = -EIO;
			goto end;
		}

		r_sdhi_read_write_common(&priv->sdmmc_ctrl, rx_cmd.sector_count, rx_cmd.sector_size,
					 rx_cmd.opcode, rx_cmd.arg);

		/* Verify card is back in transfer state after write */
		ret = k_sem_take(&priv->sdmmc_event.transfer_sem, K_MSEC(rx_cmd.timeout_ms));
		if (ret < 0) {
			LOG_ERR("Can not take sem!");
			goto end;
		}

		if (!priv->sdmmc_event.transfer_completed) {
			LOG_ERR("Transfer not completed!");
			ret = -EIO;
			goto end;
		}

		arch_dcache_flush_and_invd_all();
		priv->sdmmc_event.transfer_completed = false;

		break;

	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		arch_dcache_flush_and_invd_all();
		fsp_err = r_sdhi_transfer_write(&priv->sdmmc_ctrl, rx_cmd.sector_count,
						rx_cmd.sector_size, rx_cmd.data);
		if (fsp_err != FSP_SUCCESS) {
			ret = -EIO;
			goto end;
		}
		/* Send command with data for reading */
		r_sdhi_read_write_common(&priv->sdmmc_ctrl, rx_cmd.sector_count, rx_cmd.sector_size,
					 rx_cmd.opcode, rx_cmd.arg);

		/* Verify card is back in transfer state after write */
		ret = k_sem_take(&priv->sdmmc_event.transfer_sem, K_MSEC(rx_cmd.timeout_ms));
		if (ret < 0) {
			LOG_ERR("Can not take sem!");
			goto end;
		}

		if (!priv->sdmmc_event.transfer_completed) {
			ret = -EIO;
			goto end;
		}

		priv->sdmmc_event.transfer_completed = false;
		break;

	default:
		LOG_INF("SDHC driver: command %u not supported", cmd->opcode);
		ret = -ENOTSUP;
	}

	if (rx_cmd.opcode == SD_ALL_SEND_CID || rx_cmd.opcode == SD_SEND_CSD) {
		/* SDResponseR2 are bits from 8-127, first 8 MSBs are reserved */
		p_csd_reg.reg.sdrsp10 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP10 << 8;
		p_csd_reg.reg.sdrsp32 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP32 << 8;
		p_csd_reg.reg.sdrsp54 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP54 << 8;
		p_csd_reg.reg.sdrsp76 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP76 << 8;

		memcpy(cmd->response, &p_csd_reg.reg, sizeof(cmd->response));
	} else {
		/* Fill response buffer */
		p_csd_reg.reg.sdrsp10 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP10;
		p_csd_reg.reg.sdrsp32 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP32;
		p_csd_reg.reg.sdrsp54 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP54;
		p_csd_reg.reg.sdrsp76 = (uint32_t)priv->sdmmc_ctrl.p_reg->SD_RSP76;

		memcpy(cmd->response, &p_csd_reg.reg, sizeof(cmd->response));
	}
end:
	if (cmd->opcode == SD_APP_CMD) {
		priv->app_cmd = true;
	} else {
		priv->app_cmd = false;
	}

	k_sem_give(&priv->thread_lock);

	return ret;
}

/*
 * Get host properties
 */
static int sdhc_rx_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct sdhc_rx_config *cfg = dev->config;

	memcpy(props, &cfg->props, sizeof(struct sdhc_host_props));
	return 0;
}

static int sdhc_rx_reset(const struct device *dev)
{
	struct sdhc_rx_data *priv = dev->data;
	const struct sdhc_rx_config *cfg = dev->config;

	k_sem_take(&priv->thread_lock, K_USEC(50));

	/* Reset SDHI. */
	((R_SDHI0_Type *)cfg->regs)->SOFT_RST = 0x0U;
	((R_SDHI0_Type *)cfg->regs)->SOFT_RST = 0x1U;

	k_sem_give(&priv->thread_lock);

	return 0;
}

static int sdhc_rx_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhc_rx_data *priv = dev->data;
	const struct sdhc_rx_config *cfg = dev->config;
	struct st_sdmmc_instance_ctrl *p_ctrl = &priv->sdmmc_ctrl;
	int fsp_err;
	int ret = 0;

	uint8_t bus_width;

	if (ios->bus_width > 0) {
		uint32_t sd_option = SDHI_PRV_SD_OPTION_DEFAULT;
		/* Set bus width, SD bus interface doesn't support 8BIT */
		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			bus_width = 1;
			sd_option |= BIT(15); /* SD_OPTION.WIDTH */
			break;
		case SDHC_BUS_WIDTH4BIT:
			/* WIDTH = 0, WIDTH8 = 0 -> 4-bit */
			bus_width = 4;
			break;
		default:
			ret = -ENOTSUP;
			goto end;
		}

		if (priv->bus_width != bus_width) {
			/* Set the bus width in the SDHI peripheral. */
			((R_SDHI0_Type *)cfg->regs)->SD_OPTION = sd_option;
			priv->bus_width = bus_width;
		}
	}

	if (ios->clock) {
		if (ios->clock > cfg->props.f_max || ios->clock < cfg->props.f_min) {
			LOG_ERR("Proposed clock outside supported host range");
			return -EINVAL;
		}

		if (priv->bus_clock != (uint32_t)ios->clock) {
			fsp_err = r_sdhi_max_clock_rate_set(p_ctrl, ios->clock);
			if (fsp_err != FSP_SUCCESS) {
				ret = -EIO;
				goto end;
			}
			priv->bus_clock = ios->clock;
		}
	}

	if (ios->timing > 0) {
		/* Set I/O timing */
		if (priv->timing != ios->timing) {
			switch (ios->timing) {
			case SDHC_TIMING_LEGACY:
				/*
				 * Default speed.
				 * No timing register in RX SDHI.
				 * Effective timing depends on clock.
				 */
				break;
			case SDHC_TIMING_HS:
				/*
				 * High Speed.
				 * Card has already been switched by CMD6.
				 * Host enters HS implicitly when clock is increased.
				 * No additional HW config required.
				 */
				break;
			default:
				LOG_ERR("Timing mode %d not supported for this device",
					ios->timing);
				ret = -ENOTSUP;
				goto end;
				break;
			}

			priv->timing = ios->timing;
		}
	}
end:
	return ret;
}

static int sdhc_rx_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback,
				     int sources, void *user_data)
{
	struct sdhc_rx_data *priv = dev->data;

	if (sources & SDHC_INT_SDIO) {
		LOG_ERR("SDIO interrupt source is not supported");
		return -ENOTSUP;
	}

	if ((sources & SDHC_INT_INSERTED) || (sources & SDHC_INT_REMOVED)) {
		if(priv->card_irq_enabled) {
			LOG_DBG("Card detect interrupt is already enabled, only set callback");
		} else {
			 /* Enable card detect interrupt	by default */
			int ret = rx_grp_intc_set_gen(priv->sdhc_intc, priv->card_num, true);
			if (ret) {
				LOG_ERR("Failed to enable SDHI card detect interrupt");
				return ret;
			}
			LOG_INF("Card detect interrupt enabled");
			priv->card_irq_enabled = true;
		}
		/* Store the callback function and user data */
		priv->sdhc_cb = callback;
		priv->sdhc_cb_user_data = user_data;
	}

	return 0;
}

static int sdhc_rx_disable_interrupt(const struct device *dev, int sources)
{
	struct sdhc_rx_data *priv = dev->data;

	if (sources & SDHC_INT_SDIO) {
		LOG_ERR("SDIO interrupt source is not supported");
		return -ENOTSUP;
	}

	if ((sources & SDHC_INT_INSERTED) || (sources & SDHC_INT_REMOVED)) {
		if(!priv->card_irq_enabled) {
			LOG_DBG("Card detect interrupt is already disabled");
		} else {
			int ret = rx_grp_intc_set_gen(priv->sdhc_intc, priv->card_num, false);
			if (ret) {
				LOG_ERR("Failed to disable SDHI card detect interrupt");
				return ret;
			}
			priv->card_irq_enabled = false;
			LOG_INF("Card detect interrupt disabled");
		}
		priv->sdhc_cb = NULL;
		priv->sdhc_cb_user_data = NULL;
	}

	return 0;
}

static int sdhc_rx_enable_access_interrupts(const struct device *dev)
{
	struct sdhc_rx_data *priv = dev->data;
	int ret;

	/* Enable and set callback for card access interrupt */
	ret = rx_grp_intc_enable(priv->sdhc_intc, priv->accs_num);
	if (ret) {
		LOG_ERR("Failed to enable SDHI card access interrupt");
		return ret;
	}

	ret = rx_grp_intc_callback_set(priv->sdhc_intc, priv->accs_num, sdhimmc_accs_isr,
				       (void *)dev);
	if (ret) {
		LOG_ERR("Failed to set SDHI card access interrupt callback");
		return ret;
	}

	/* Enable card detect interrupt	by default */
	ret = rx_grp_intc_enable(priv->sdhc_intc, priv->card_num);
	if (ret) {
		LOG_ERR("Failed to enable SDHI card detect interrupt");
		return ret;
	}

	ret = rx_grp_intc_callback_set(priv->sdhc_intc, priv->card_num, sdhimmc_card_isr,
				       (void *)dev);
	if (ret) {
		LOG_ERR("Failed to set SDHI card detect interrupt callback");
		return ret;
	}

	priv->card_irq_enabled = true;

	return 0;
}

static int sdhc_rx_sdmmc_init(const struct device *dev)
{
	const struct sdhc_rx_config *config = dev->config;
	struct sdhc_rx_data *priv = dev->data;
	fsp_err_t fsp_err;
	int timeout = SDHI_PRV_ACCESS_TIMEOUT_US;
	int ret = 0;

	priv->sdmmc_event.transfer_completed = false;
	k_sem_init(&priv->sdmmc_event.transfer_sem, 0, 1);
	k_sem_init(&priv->thread_lock, 1, 1);

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}
	if (priv->sdhi_en.port != NULL) {
		int err = gpio_pin_configure_dt(&priv->sdhi_en, GPIO_OUTPUT_HIGH);
		if (err) {
			return err;
		}
		k_sleep(K_MSEC(50));
	}

	/* Enable card access interrupt */
	ret = sdhc_rx_enable_access_interrupts(dev);
	if (ret) {
		return ret;
	}

	fsp_err = R_SDHI_Open(&priv->sdmmc_ctrl, &priv->fsp_config);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("R_SDHI_Open error: %d", fsp_err);
		return -EIO;
	}

	k_busy_wait(100);

	k_sem_take(&priv->thread_lock, K_USEC(timeout));

	/* Configure SDHI peripheral*/
	fsp_err = r_sdhi_hw_cfg(&priv->sdmmc_ctrl);
	k_busy_wait(100);
	if (fsp_err != FSP_SUCCESS) {
		LOG_ERR("Failed to configure SDHI peripheral");
		goto end;
	}

	priv->bus_width = SDMMC_BUS_WIDTH_1_BIT;
	priv->timing = SDHC_TIMING_LEGACY;
	priv->bus_clock = SDMMC_CLOCK_400KHZ;
end:
	k_sem_give(&priv->thread_lock);
	return ret;
}

static DEVICE_API(sdhc, sdhc_api) = {
	.reset = sdhc_rx_reset,
	.request = sdhc_rx_request,
	.set_io = sdhc_rx_set_io,
	.get_card_present = sdhc_rx_get_card_present,
	.card_busy = sdhc_rx_card_busy,
	.get_host_props = sdhc_rx_get_host_props,
	.enable_interrupt = sdhc_rx_enable_interrupt,
	.disable_interrupt = sdhc_rx_disable_interrupt,
	.execute_tuning = NULL,
};

#define SDHC_RENESAS_RX_SW_CONFIGURABLE_INT(index)                                                 \
	do {                                                                                       \
		int sbfai_irq = DT_IRQ_BY_NAME(DT_DRV_INST(index), sbfai, irq);                    \
                                                                                                   \
		if (128 <= sbfai_irq && sbfai_irq < 144) {                                         \
			R_ICU->SLIXR[sbfai_irq - 128].SLIXR = ICU_EVENT_SDHI_SBFAI;                \
		} else if (144 <= sbfai_irq) {                                                     \
			R_ICU->SLIR[sbfai_irq - 144].SLIR = ICU_EVENT_SDHI_SBFAI;                  \
		}                                                                                  \
                                                                                                   \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_DRV_INST(index), sbfai, irq),                        \
			    DT_IRQ_BY_NAME(DT_DRV_INST(index), sbfai, priority),                   \
			    rx_sdmmc_dma_req_isr, DEVICE_DT_INST_GET(index), 0);                   \
		irq_enable(sbfai_irq);                                                             \
	} while (0)

#define RX_SDHI_EN(index) .sdhi_en = GPIO_DT_SPEC_INST_GET_OR(index, enable_gpios, {0})

#define RX_SDMMC_DTC_INIT(index)                                                                   \
	sdhc_rx_data_##index.fsp_config.p_lower_lvl_transfer = &sdhc_rx_data_##index.transfer;

#define RX_SDMMC_DTC_STRUCT_INIT(index)                                                            \
	.transfer_info =                                                                           \
		{                                                                                  \
			.transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED, \
			.transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,  \
			.transfer_settings_word_b.irq = TRANSFER_IRQ_END,                          \
			.transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,       \
			.transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED,        \
			.transfer_settings_word_b.size = TRANSFER_SIZE_4_BYTE,                     \
			.transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,                     \
			.p_dest = (void *)NULL,                                                    \
			.p_src = (void const *)NULL,                                               \
			.num_blocks = 0,                                                           \
			.length = 128,                                                             \
	},                                                                                         \
	.transfer_cfg_extend =                                                                     \
		{                                                                                  \
			.activation_source = (IRQn_Type)DT_INST_IRQ_BY_NAME(index, sbfai, irq),    \
	},                                                                                         \
	.transfer_cfg =                                                                            \
		{                                                                                  \
			.p_info = &sdhc_rx_data_##index.transfer_info,                             \
			.p_extend = &sdhc_rx_data_##index.transfer_cfg_extend,                     \
	},                                                                                         \
	.transfer = {                                                                              \
		.p_ctrl = &sdhc_rx_data_##index.transfer_ctrl,                                     \
		.p_cfg = &sdhc_rx_data_##index.transfer_cfg,                                       \
		.p_api = &g_transfer_on_dtc,                                                       \
	},

#define RX_SDHC_INIT(index)                                                                        \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	static const struct sdhc_rx_config sdhc_rx_config_##index = {                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.regs = (R_SDHI0_Type *)DT_INST_REG_ADDR(index),                                   \
		.props =                                                                           \
			{                                                                          \
				.is_spi = false,                                                   \
				.f_max = DT_INST_PROP(index, max_bus_freq),                        \
				.f_min = DT_INST_PROP(index, min_bus_freq),                        \
				.max_current_330 = DT_INST_PROP(index, max_current_330),           \
				.max_current_180 = DT_INST_PROP(index, max_current_180),           \
				.power_delay = DT_INST_PROP_OR(index, power_delay_ms, 0),          \
				.host_caps =                                                       \
					{                                                          \
						.vol_180_support = false,                          \
						.vol_300_support = false,                          \
						.vol_330_support = true,                           \
						.suspend_res_support = false,                      \
						.sdma_support = true,                              \
						.high_spd_support =                                \
							(DT_INST_PROP(index, bus_width) == 4)      \
								? true                             \
								: false,                           \
						.adma_2_support = false,                           \
						.max_blk_len = 0,                                  \
						.ddr50_support = false,                            \
						.sdr104_support = false,                           \
						.sdr50_support = false,                            \
						.bus_8_bit_support = false,                        \
						.bus_4_bit_support =                               \
							(DT_INST_PROP(index, bus_width) == 4)      \
								? true                             \
								: false,                           \
						.hs200_support = false,                            \
						.hs400_support = false,                            \
					},                                                         \
			},                                                                         \
                                                                                                   \
	};                                                                                         \
	void r_sdhi_callback_##index(sdmmc_callback_args_t *p_args)                                \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_INST_GET(index);                              \
		struct sdhc_rx_data *priv = dev->data;                                             \
		int reason = 0;                                                                    \
                                                                                                   \
		if (p_args->event & SDMMC_EVENT_TRANSFER_COMPLETE) {                               \
			priv->sdmmc_event.transfer_completed = true;                               \
			k_sem_give(&priv->sdmmc_event.transfer_sem);                               \
		}                                                                                  \
                                                                                                   \
		if (p_args->event & SDMMC_EVENT_TRANSFER_ERROR) {                                  \
			priv->sdmmc_event.transfer_completed = false;                              \
			k_sem_give(&priv->sdmmc_event.transfer_sem);                               \
		}                                                                                  \
                                                                                                   \
		if (p_args->event & SDMMC_EVENT_CARD_INSERTED) {                                   \
			reason |= SDHC_INT_INSERTED;                                               \
		}                                                                                  \
                                                                                                   \
		if (p_args->event & SDMMC_EVENT_CARD_REMOVED) {                                    \
			reason |= SDHC_INT_REMOVED;                                                \
		}                                                                                  \
                                                                                                   \
		if (priv->sdhc_cb) {                                                               \
			priv->sdhc_cb(dev, reason, priv->sdhc_cb_user_data);                       \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static struct sdhc_rx_data sdhc_rx_data_##index = {                                        \
		.power_mode = SDHC_POWER_ON,                                                       \
		.timing = SDHC_TIMING_LEGACY,                                                      \
		.card_num = DT_INST_IRQ_BY_NAME(index, card, irq),                                 \
		.accs_num = DT_INST_IRQ_BY_NAME(index, accs, irq),                                 \
		.fsp_config =                                                                      \
			{                                                                          \
				.channel = DT_INST_PROP(index, channel),                           \
				.bus_width = DT_INST_PROP(index, bus_width),                       \
				.access_ipl = DT_INST_IRQ_BY_NAME(index, accs, priority),          \
				.access_irq = (IRQn_Type)DT_INST_IRQ_BY_NAME(index, accs, irq)     \
						      << 8 |                                       \
					      102,                                                 \
				.card_ipl = DT_INST_IRQ_BY_NAME(index, card, priority),            \
				.card_irq = (IRQn_Type)DT_INST_IRQ_BY_NAME(index, card, irq)       \
						    << 8 |                                         \
					    102,                                                   \
				.dma_req_ipl = DT_INST_IRQ_BY_NAME(index, sbfai, priority),        \
				.dma_req_irq = DT_INST_IRQ_BY_NAME(index, sbfai, irq),             \
				.p_context = NULL,                                                 \
				.p_callback = r_sdhi_callback_##index,                             \
				.card_detect = DT_INST_PROP(index, card_detect),                   \
				.write_protect = DT_INST_PROP(index, write_protect),               \
				.p_extend = NULL,                                                  \
				.p_lower_lvl_transfer = &sdhc_rx_data_##index.transfer,            \
			},                                                                         \
		.sdhc_intc = DEVICE_DT_GET(DT_IRQ_INTC_BY_NAME(DT_DRV_INST(index), accs)),         \
		RX_SDHI_EN(index),                                                                 \
		RX_SDMMC_DTC_STRUCT_INIT(index)};                                                  \
                                                                                                   \
	static int sdhc_rx_init##index(const struct device *dev)                                   \
	{                                                                                          \
		RX_SDMMC_DTC_INIT(index);                                                          \
		SDHC_RENESAS_RX_SW_CONFIGURABLE_INT(index);                                        \
                                                                                                   \
		int err = sdhc_rx_sdmmc_init(dev);                                                 \
		if (err != 0) {                                                                    \
			return err;                                                                \
		}                                                                                  \
		return 0;                                                                          \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, sdhc_rx_init##index, NULL, &sdhc_rx_data_##index,             \
			      &sdhc_rx_config_##index, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,     \
			      &sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(RX_SDHC_INIT)
