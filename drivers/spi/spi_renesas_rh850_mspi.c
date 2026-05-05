/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_spi

#include <r_mspi.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rh850_spi);

#include "spi_context.h"

#define SPI_RH850_MSPI_RASTAD_CLEAR		(0x00U)		/* Clear RASTAD setting */
#define SPI_RH850_MSPI_INTFC_CLEAR		(0x01U)		/* Clear interrupt flags */
#define SPI_RH850_MSPI_MAX_BYTES_PER_PACKAGE(operation_mode) \
	(((MSPI_TRANSFER_MODE_DIRECT_ACCESS == operation_mode)) ? 65535U : 0U)

#define SPI_RH850_MSPI_TXI_INTF_INDEX	(0U)	/* Index of TX interrupt flag in INTF register */
#define SPI_RH850_MSPI_RXI_INTF_INDEX	(1U)	/* Index of RX interrupt flag in INTF register */
#define SPI_RH850_MSPI_FEI_INTF_INDEX	(2U)	/* Index of FE interrupt flag in INTF register */
#define SPI_RH850_MSPI_ERI_INTF_INDEX	(3U)	/* Index of ERR interrupt flag in INTF register */
#define SPI_RH850_MSPI_PRCS_MAX_VALUE	(3U)	/* Max value of PRCS */
#define SPI_RH850_MSPI_CDIV_MAX_VALUE	(32U)	/* Max value of CDIV */
#define SPI_RH850_MSPI_TX_DUMMY_DATA	(0U)

struct spi_rh850_config {
	const struct pinctrl_dev_config *pinctrl_dev;
	const spi_api_t *fsp_api;
};

struct spi_rh850_data {
	struct spi_context ctx;
	uint32_t data_len;
	volatile uint32_t time_out;
	spi_cfg_t *fsp_config;
	mspi_instance_ctrl_t *fsp_ctrl;
	uint8_t dfs;
};

#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT
extern void mspi_txi_isr(void);
extern void mspi_rxi_isr(void);
extern void mspi_err_isr(void);
extern void mspi_fe_isr(void);

static void spi_rh850_rxi_isr(const struct device *dev);
static void spi_rh850_txi_isr(const struct device *dev);
static void spi_rh850_fe_isr(const struct device *dev);
static void spi_rh850_eri_isr(const struct device *dev);
#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */

static bool spi_rh850_transfer_done(struct spi_context *ctx)
{
	return (!spi_context_tx_on(ctx) && !spi_context_rx_on(ctx));
}

static int spi_rh850_data_complete_update(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	const uint8_t dfs = data->dfs;

	if (spi_context_rx_on(&data->ctx)) {
		spi_context_update_rx(&data->ctx, dfs, data->data_len);
	}

	if (spi_context_tx_on(&data->ctx)) {
		spi_context_update_tx(&data->ctx, dfs, data->data_len);
	}

	return 0;
}

#ifndef CONFIG_SPI_RENESAS_RH850_INTERRUPT

static int spi_rh850_config_transfer(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	const mspi_instance_ctrl_t *spi_ctrl = data->fsp_ctrl;
	mspi_extended_cfg_t *spi_config_extend =
				(mspi_extended_cfg_t *)(data->fsp_config->p_extend);
	R_MSPI0_MSPI0_CH_Type *p_mspi_regs = (R_MSPI0_MSPI0_CH_Type *)spi_ctrl->p_regs->MSPI0_CH;
	uint8_t  channel = spi_ctrl->p_cfg->channel;
	R_MSPI0_INTF_MSPI0_INTF_CMM_Type *p_mspi_intf_regs =
		(R_MSPI0_INTF_MSPI0_INTF_CMM_Type *) spi_ctrl->p_regs_intf->MSPI0_INTF_CMM;
	mspi_transfer_mode_t spi_operation_mode =
		spi_config_extend->p_mspi_config_cfg0->mspi_chip_select_cfg0.mspi_transfer_mode;
	uint32_t intfc_clear = (uint32_t)(SPI_RH850_MSPI_INTFC_CLEAR << channel);

	if (MSPI_TRANSFER_MODE_DIRECT_ACCESS != spi_operation_mode) {
		return -EIO;
	}

	/* Reset the timeout counter. */
	data->time_out = 0U;

	/* Clear channel enable bit */
	p_mspi_regs[channel].CSTC |= R_MSPI0_MSPI0_CH_CSTS_CHENS_Msk;
	while ((p_mspi_regs[channel].CSTR_b.CHEN != 0U) &&
		(CONFIG_SPI_RH850_SYNC_TRANSFER_TIMEOUT_VALUE > data->time_out)) {
		data->time_out++;
	}

	if (p_mspi_regs[channel].CSTR_b.CHEN != 0U) {
		return -ETIMEDOUT;
	}

	/* Clear ERR flag */
	p_mspi_regs[channel].CESTC = (R_MSPI0_MSPI0_CH_CESTC_PEC_Msk |
					R_MSPI0_MSPI0_CH_CESTC_CEC_Msk |
					R_MSPI0_MSPI0_CH_CESTC_DCEC_Msk |
					R_MSPI0_MSPI0_CH_CESTC_OVRUEC_Msk |
					R_MSPI0_MSPI0_CH_CESTC_OVWREC_Msk |
					R_MSPI0_MSPI0_CH_CESTC_OVREEC_Msk);

	/* Clear TX flag */
	p_mspi_intf_regs[SPI_RH850_MSPI_TXI_INTF_INDEX].INTFC |= intfc_clear;

	/* Clear RX flag */
	p_mspi_intf_regs[SPI_RH850_MSPI_RXI_INTF_INDEX].INTFC |= intfc_clear;

	/* Clear FE flag */
	p_mspi_intf_regs[SPI_RH850_MSPI_FEI_INTF_INDEX].INTFC |= intfc_clear;

	/* Clear ERR flag */
	p_mspi_intf_regs[SPI_RH850_MSPI_ERI_INTF_INDEX].INTFC |= intfc_clear;

	/* Initialize CFG registers */
	p_mspi_regs[channel].CFG0 =
		(uint32_t) (spi_config_extend->p_mspi_config_cfg0->chip_select_config_0_u32);
	p_mspi_regs[channel].CFG1 =
		(uint32_t) (spi_config_extend->p_mspi_config_cfg1->chip_select_config_1_u32);
	p_mspi_regs[channel].CFG4 =
		(uint32_t) (spi_config_extend->p_mspi_config_cfg4->chip_select_config_4_u32);
	p_mspi_regs[channel].SEUP =
		(uint16_t) (spi_config_extend->mspi_setup_time);
	p_mspi_regs[channel].HOLD =
		(uint16_t) (spi_config_extend->mspi_hold_time);
	p_mspi_regs[channel].IDLE =
		(uint16_t) (spi_config_extend->mspi_idle_time);

	/* Set bit width */
	p_mspi_regs[channel].CFG2 = spi_ctrl->bitwidth;

	if (MSPI_TRANSFER_MODE_DIRECT_ACCESS == spi_operation_mode) {
		/* Set Direct Access Mode and clear RAM address */
		p_mspi_regs[channel].CFG0  &= ~R_MSPI0_MSPI0_CH_CFG0_MD_Msk;
		p_mspi_regs[channel].RASTAD = SPI_RH850_MSPI_RASTAD_CLEAR;

		/* Set the frame count */
		p_mspi_regs[channel].CFSET = data->data_len;

		/* Enable TX and RX */
		p_mspi_regs[channel].CFG0 |= (R_MSPI0_MSPI0_CH_CFG0_TXE_Msk
			| R_MSPI0_MSPI0_CH_CFG0_RXE_Msk);

		/* Enable MSPI channel and start transfer */
		p_mspi_regs[channel].CSTS = (R_MSPI0_MSPI0_CH_CSTS_CHENS_Msk
			| R_MSPI0_MSPI0_CH_CSTS_ACTFS_Msk);
	}

	return 0;
}

static void spi_rh850_transmit_handling(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	mspi_instance_ctrl_t *spi_ctrl = data->fsp_ctrl;
	R_MSPI0_MSPI0_CH_Type *p_mspi_regs =
				(R_MSPI0_MSPI0_CH_Type *) spi_ctrl->p_regs->MSPI0_CH;
	R_MSPI0_INTF_MSPI0_INTF_CMM_Type *p_mspi_intf_regs =
		(R_MSPI0_INTF_MSPI0_INTF_CMM_Type *) spi_ctrl->p_regs_intf->MSPI0_INTF_CMM;
	uint8_t channel = spi_ctrl->p_cfg->channel;
	uint32_t tx;
	uint32_t tx_offset = 0;

	/* Clear TX flag */
	p_mspi_intf_regs[SPI_RH850_MSPI_TXI_INTF_INDEX].INTFC |=
		SPI_RH850_MSPI_INTFC_CLEAR << channel;

	/* Check if tx_count is still within the transmit range */
	if (data->data_len > data->fsp_ctrl->tx_count) {
		tx_offset = spi_ctrl->tx_count;

		/* TX transfer */
		if (spi_context_tx_buf_on(&data->ctx)) {
			if (spi_ctrl->bitwidth > SPI_BIT_WIDTH_16_BITS) {
				tx = UNALIGNED_GET((uint32_t *)(data->ctx.tx_buf) + tx_offset);
			} else if (spi_ctrl->bitwidth > SPI_BIT_WIDTH_8_BITS) {
				tx = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf) + tx_offset);
			} else {
				tx = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf) + tx_offset);
			}
		} else {
			tx = 0U;
		}

		/* Write tx data to TXDA0 register */
		p_mspi_regs[channel].TXDA0 = tx;
		spi_ctrl->tx_count++;
	}
}

static void spi_rh850_receive_handling(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	mspi_instance_ctrl_t *spi_ctrl = data->fsp_ctrl;
	R_MSPI0_MSPI0_CH_Type *p_mspi_regs =
				(R_MSPI0_MSPI0_CH_Type *) spi_ctrl->p_regs->MSPI0_CH;
	R_MSPI0_INTF_MSPI0_INTF_CMM_Type *p_mspi_intf_regs =
		(R_MSPI0_INTF_MSPI0_INTF_CMM_Type *) spi_ctrl->p_regs_intf->MSPI0_INTF_CMM;
	uint8_t channel = spi_ctrl->p_cfg->channel;
	uint32_t rx;
	uint32_t rx_offset = 0;

	/* Clear RX flag */
	p_mspi_intf_regs[SPI_RH850_MSPI_RXI_INTF_INDEX].INTFC |=
		SPI_RH850_MSPI_INTFC_CLEAR << channel;

	rx_offset = spi_ctrl->rx_count;

	/* Check if rx_count is still within the transmit range */
	if (data->data_len > data->fsp_ctrl->rx_count) {
		/* RX transfer */
		if (spi_context_rx_buf_on(&data->ctx)) {
			/* Read data from RXDA0 register */
			rx = p_mspi_regs[channel].RXDA0;

			/* Store data to rx buffer */
			if (spi_ctrl->bitwidth > SPI_BIT_WIDTH_16_BITS) {
				UNALIGNED_PUT(rx, ((uint32_t *)(data->ctx.rx_buf) + rx_offset));
			} else if (spi_ctrl->bitwidth > SPI_BIT_WIDTH_8_BITS) {
				UNALIGNED_PUT(rx, ((uint16_t *)(data->ctx.rx_buf) + rx_offset));
			} else {
				UNALIGNED_PUT(rx, ((uint8_t *)(data->ctx.rx_buf) + rx_offset));
			}
		} else {
			/* Dummy read */
			rx = p_mspi_regs[channel].RXDA0;
		}
		spi_ctrl->rx_count++;
	}
}

static void spi_rh850_frame_end_handling(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	const mspi_instance_ctrl_t *spi_ctrl = data->fsp_ctrl;
	R_MSPI0_MSPI0_CH_Type *p_mspi_regs = (R_MSPI0_MSPI0_CH_Type *) spi_ctrl->p_regs->MSPI0_CH;
	R_MSPI0_INTF_MSPI0_INTF_CMM_Type *p_mspi_intf_regs =
			(R_MSPI0_INTF_MSPI0_INTF_CMM_Type *) spi_ctrl->p_regs_intf->MSPI0_INTF_CMM;
	uint8_t  channel = spi_ctrl->p_cfg->channel;

	/* Clear the frame end interrupt flag. */
	p_mspi_intf_regs[SPI_RH850_MSPI_FEI_INTF_INDEX].INTFC |=
		SPI_RH850_MSPI_INTFC_CLEAR << channel;

	mspi_transfer_mode_t spi_operation_mode = p_mspi_regs[channel].CFG0_b.MD;

	if (MSPI_TRANSFER_MODE_DIRECT_ACCESS == spi_operation_mode) {
		/* Do nothing */
	}
}

static void spi_rh850_err_handling(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	const mspi_instance_ctrl_t *spi_ctrl = data->fsp_ctrl;
	R_MSPI0_MSPI0_CH_Type *p_mspi_regs = (R_MSPI0_MSPI0_CH_Type *) spi_ctrl->p_regs->MSPI0_CH;
	R_MSPI0_INTF_MSPI0_INTF_CMM_Type *p_mspi_intf_regs =
			(R_MSPI0_INTF_MSPI0_INTF_CMM_Type *) spi_ctrl->p_regs_intf->MSPI0_INTF_CMM;
	uint8_t  channel = spi_ctrl->p_cfg->channel;

	/* Clear ERR flag */
	p_mspi_intf_regs[SPI_RH850_MSPI_ERI_INTF_INDEX].INTFC |=
		SPI_RH850_MSPI_INTFC_CLEAR << channel;

	/* Check error type and clear */
	if (0U != p_mspi_regs[channel].CEST) {
		p_mspi_regs[channel].CESTC = (R_MSPI0_MSPI0_CH_CESTC_PEC_Msk |
						R_MSPI0_MSPI0_CH_CESTC_CEC_Msk |
						R_MSPI0_MSPI0_CH_CESTC_DCEC_Msk |
						R_MSPI0_MSPI0_CH_CESTC_OVRUEC_Msk |
						R_MSPI0_MSPI0_CH_CESTC_OVWREC_Msk |
						R_MSPI0_MSPI0_CH_CESTC_OVREEC_Msk);
	}

	/* Turn off SPI driver */
	spi_ctrl->p_regs->CTL0 &= (uint8_t) ~R_MSPI0_CTL0_EN_Msk;
}

static int spi_rh850_main_function_handling(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	mspi_instance_ctrl_t *spi_ctrl = data->fsp_ctrl;
	R_MSPI0_MSPI0_CH_Type *p_mspi_regs =
				(R_MSPI0_MSPI0_CH_Type *) spi_ctrl->p_regs->MSPI0_CH;
	R_MSPI0_INTF_MSPI0_INTF_CMM_Type *p_mspi_intf_regs =
		(R_MSPI0_INTF_MSPI0_INTF_CMM_Type *) spi_ctrl->p_regs_intf->MSPI0_INTF_CMM;
	uint8_t channel = spi_ctrl->p_cfg->channel;
	int result = 0;

	/* Clear the time-out value to 0 */
	data->time_out = 0U;
	spi_ctrl->tx_count = 0U;
	spi_ctrl->rx_count = 0U;

	do {
		bool timeout_flag = true;

		/* Check whether the RX interrupt flag was set */
		if (((p_mspi_intf_regs[SPI_RH850_MSPI_RXI_INTF_INDEX].INTF)
					& (1U << channel)) == (1U << channel)) {

			/* TX interrupt has higher priority than RX, issue the TX handling first */
			if (((p_mspi_intf_regs[SPI_RH850_MSPI_TXI_INTF_INDEX].INTF)
					& (1U << channel)) == (1U << channel)) {
				/* TX handling */
				spi_rh850_transmit_handling(dev);
			}

			/* RX handling */
			spi_rh850_receive_handling(dev);
			timeout_flag = false;
		}

		/* Check whether the TX interrupt flag was set */
		if (((p_mspi_intf_regs[SPI_RH850_MSPI_TXI_INTF_INDEX].INTF)
					& (1U << channel)) == (1U << channel)) {
			/* TX handling */
			spi_rh850_transmit_handling(dev);
			timeout_flag = false;
		}

		/* Check whether the FE interrupt flag was set */
		if (((p_mspi_intf_regs[SPI_RH850_MSPI_FEI_INTF_INDEX].INTF)
					& (1U << channel)) == (1U << channel)) {
			/* Frame end handling */
			spi_rh850_frame_end_handling(dev);
			timeout_flag = false;
		}

		if (((p_mspi_intf_regs[SPI_RH850_MSPI_ERI_INTF_INDEX].INTF)
					& (1U << channel)) == (1U << channel)) {
			/* Error handling */
			spi_rh850_err_handling(dev);
			result = -EIO;
			break;
		}

		if (timeout_flag) {
			/* Increase the timeout count */
			data->time_out++;
		} else {
			data->time_out = 0U;
		}

		/* Check the timeout */
		if (CONFIG_SPI_RH850_SYNC_TRANSFER_TIMEOUT_VALUE < data->time_out) {
			/* Turn off the SPI driver */
			spi_ctrl->p_regs->CTL0 &= (uint8_t) ~R_MSPI0_CTL0_EN_Msk;
			result = -ETIMEDOUT;
			break;
		}
	} while (!result &&
			((data->data_len > spi_ctrl->tx_count) ||
						(data->data_len > spi_ctrl->rx_count)));

	return result;
}

#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */

static int spi_rh850_transfer_next_packet(const struct device *dev)
{
	struct spi_rh850_data *data = dev->data;
	const mspi_extended_cfg_t *spi_config_extend =
				(mspi_extended_cfg_t *)(data->fsp_config->p_extend);
#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT
	fsp_err_t err;
#endif
	int ret = 0;

	data->data_len = spi_context_max_continuous_chunk(&data->ctx);

	if (data->data_len == 0U) {
		return 0;
	}

	data->data_len = MIN(data->data_len,
			SPI_RH850_MSPI_MAX_BYTES_PER_PACKAGE(spi_config_extend->p_mspi_config_cfg0
					->mspi_chip_select_cfg0.mspi_transfer_mode));

	if (data->data_len == 0U) {
		return -EIO;
	}

#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT
	const struct spi_rh850_config *config = dev->config;

	if (data->ctx.tx_buf == NULL) {
		/* If there is only the rx buffer */
		err = config->fsp_api->read(data->fsp_ctrl, data->ctx.rx_buf, data->data_len,
						data->fsp_ctrl->bitwidth);
	} else if (data->ctx.rx_buf == NULL) {
		/* If there is only the tx buffer */
		err = config->fsp_api->write(data->fsp_ctrl, data->ctx.tx_buf, data->data_len,
						 data->fsp_ctrl->bitwidth);
	} else {
		err = config->fsp_api->writeRead(data->fsp_ctrl, data->ctx.tx_buf, data->ctx.rx_buf,
						 data->data_len, data->fsp_ctrl->bitwidth);
	}

	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: R_MSPI transfer failed: %d", dev->name, err);
		return -EIO;
	}

#else
	/* Configure SPI transfer */
	ret = spi_rh850_config_transfer(dev);

	/* Perform transfer */
	ret = spi_rh850_main_function_handling(dev);

#endif
	return ret;
}

static void spi_rh850_transfer_callback(spi_callback_args_t *p_args)
{
	const struct device *dev = p_args->p_context;
	struct spi_rh850_data *data = dev->data;
	int ret = 0;

	/* If the event was ERROR, print log and return error */
	if ((p_args->event != SPI_EVENT_TRANSFER_COMPLETE) &&
			(p_args->event != SPI_EVENT_RECEIVE_COMPLETE)) {
		LOG_ERR("%s: SPI transfer callback failed (event=%d)", dev->name, p_args->event);
		ret = -EIO;
		goto out_complete;
	}

	/* If the event was RECEIVE COMPLETE, do nothing */
	if (p_args->event == SPI_EVENT_RECEIVE_COMPLETE) {
		return;
	}

	/* If the event was TRANSFER COMPLETE, update the TX/RX context */
	ret = spi_rh850_data_complete_update(dev);
	if (ret != 0) {
		goto out_complete;
	}

	/* If there is nothing left to transfer, go to out_complete label */
	if (spi_rh850_transfer_done(&data->ctx)) {
		goto out_complete;
	}

	/* If any data remains, issue transfer again */
	ret = spi_rh850_transfer_next_packet(dev);
	if (ret == 0) {
		return;
	}

	LOG_ERR("%s: Failed to start next SPI packet (err %d)", dev->name, ret);

out_complete:
	spi_context_cs_control(&data->ctx, false);
	spi_context_complete(&data->ctx, dev, ret);
}

static uint16_t spi_rh850_baudrate_calculate(uint32_t frequency)
{
	uint32_t best_error = UINT32_MAX;
	uint8_t best_prcs = 0U;
	uint8_t best_cdiv = 0U;

	/* Loop through PRCS values (0 to 3) */
	for (uint8_t prcs = 0U; prcs <= SPI_RH850_MSPI_PRCS_MAX_VALUE; prcs++) {

		/* prcs_multiplier = 4^prcs */
		uint32_t prcs_multiplier = 1U << (2U * prcs);

		/* Loop through CDIV values (1 to 32). */
		for (uint8_t cdiv = 1U; cdiv <= SPI_RH850_MSPI_CDIV_MAX_VALUE; cdiv++) {
			uint32_t denominator = prcs_multiplier * (uint32_t)cdiv * 2U;
			uint32_t actual = BSP_CFG_CLOCK_CLK_MSPI_HZ / denominator;
			uint32_t error;

			if (frequency > actual) {
				error = frequency - actual;
			} else {
				error = actual - frequency;
			}

			if (error < best_error) {
				best_error = error;
				best_prcs = prcs;
				best_cdiv = cdiv;

				if (best_error == 0U) {
					break;
				}
			}
		}
		if (best_error == 0U) {
			break;
		}
	}

	return ((uint16_t)best_prcs << 8) | best_cdiv;
}

static int spi_rh850_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	struct spi_rh850_data *data = dev->data;
	const struct spi_rh850_config *config = dev->config;
	mspi_extended_cfg_t *spi_config_extend =
				(mspi_extended_cfg_t *)(data->fsp_config->p_extend);
	mspi_instance_ctrl_t *spi_ctrl = data->fsp_ctrl;
	fsp_err_t err;
#ifdef BSP_FEATURE_DEVICE_HAS_ORED_IRQ
#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT
	static bool ored_irq_not_init = true;
#endif
#endif

	/* If the MSPI instance has been opened, close it */
	if (spi_ctrl->open != 0U) {
		err = config->fsp_api->close(spi_ctrl);
		if (err != FSP_SUCCESS) {
			LOG_ERR("%s: R_MSPI_Close error: %d", dev->name, err);
			return -EIO;
		}
	}

	/* TI frame is not supported */
	if ((spi_cfg->operation & SPI_FRAME_FORMAT_TI) == SPI_FRAME_FORMAT_TI) {
		LOG_ERR("%s: TI frame not supported", dev->name);
		return -ENOTSUP;
	}

	/* Only MISO single line mode is supported */
	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
		((uint32_t)spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("%s: Only MISO single line mode is supported", dev->name);
		return -ENOTSUP;
	}

	/* SPI mode */
	if (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_SLAVE) {
		if (IS_ENABLED(CONFIG_SPI_SLAVE)) {
			data->fsp_config->operating_mode = SPI_MODE_SLAVE;
			spi_config_extend->mspi_ctl1.mspi_bit_control_register_1.mspi_mode_selection
						= SPI_MODE_SLAVE;
		} else {
			LOG_ERR("%s: Kconfig for enable SPI in slave mode is not enabled",
											dev->name);
			return -ENOTSUP;
		}
	} else {
		data->fsp_config->operating_mode = SPI_MODE_MASTER;
		spi_config_extend->mspi_ctl1.mspi_bit_control_register_1.mspi_mode_selection
					= SPI_MODE_MASTER;
	}

	/* SPI polarity */
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL) {
		/* Configure SCK is high during idle time only when SPI is in master mode */
		if (SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_SLAVE) {
			data->fsp_config->clk_polarity = SPI_CLK_POLARITY_HIGH;
			spi_config_extend->p_mspi_config_cfg1
				->mspi_chip_select_cfg1.mspi_sck_polarity = MSPI_POLARITY_HIGH;
		} else {
			LOG_ERR("%s: Slave mode does not support a high SCK during idle time",
											dev->name);
			return -ENOTSUP;
		}

	} else {
		data->fsp_config->clk_polarity = SPI_CLK_POLARITY_LOW;
		spi_config_extend->p_mspi_config_cfg1->mspi_chip_select_cfg1.mspi_sck_polarity
						= MSPI_POLARITY_LOW;
	}

	/* SPI phase */
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA) {
		data->fsp_config->clk_phase = SPI_CLK_PHASE_EDGE_EVEN;
		spi_config_extend->p_mspi_config_cfg1->mspi_chip_select_cfg1.mspi_phase_select
						= MSPI_PHASE_SELECT_EVEN;
	} else {
		data->fsp_config->clk_phase = SPI_CLK_PHASE_EDGE_ODD;
		spi_config_extend->p_mspi_config_cfg1->mspi_chip_select_cfg1.mspi_phase_select
						= MSPI_PHASE_SELECT_ODD;
	}

	/* SPI bit order */
	if (spi_cfg->operation & SPI_TRANSFER_LSB) {
		data->fsp_config->bit_order = SPI_BIT_ORDER_LSB_FIRST;
		spi_config_extend->p_mspi_config_cfg1->mspi_chip_select_cfg1.mspi_direction
						= SPI_BIT_ORDER_LSB_FIRST;
	} else {
		data->fsp_config->bit_order = SPI_BIT_ORDER_MSB_FIRST;
		spi_config_extend->p_mspi_config_cfg1->mspi_chip_select_cfg1.mspi_direction
						= SPI_BIT_ORDER_MSB_FIRST;
	}

	/* Check valid bit width for SPI */
	if ((SPI_WORD_SIZE_GET(spi_cfg->operation) < SPI_BIT_WIDTH_2_BITS) ||
		(SPI_WORD_SIZE_GET(spi_cfg->operation) > SPI_BIT_WIDTH_128_BITS)) {
		LOG_ERR("%s: Bit width not supported", dev->name);
		return -ENOTSUP;
	}

	/* Set bit width for SPI */
	data->dfs = ((SPI_WORD_SIZE_GET(spi_cfg->operation) - 1) / 8) + 1;
	spi_ctrl->bitwidth = (spi_bit_width_t)(SPI_WORD_SIZE_GET(spi_cfg->operation));

	/* SPI full duplex mode */
	spi_config_extend->mspi_select = MSPI_SELECTABLE_MODE_TRANSMIT_RECEIVE;

	/* Chip select */
	uint8_t chip_select = spi_cfg->slave;

	if (chip_select >= spi_config_extend->mspi_number_of_cs) {
		LOG_ERR("%s: Chip select not supported", dev->name);
		return -EINVAL;
	}

	/* SPI chip select polarity */
	if (spi_cfg->operation & SPI_CS_ACTIVE_HIGH) {
		spi_config_extend->mspi_ctl1.mspi_bit_control_register_1.mspi_control_polarity
				|= (mspi_control_polarity_t)((uint16_t) 1U << chip_select);
	} else {
		spi_config_extend->mspi_ctl1.mspi_bit_control_register_1.mspi_control_polarity
				&= (mspi_control_polarity_t) ~((uint16_t) 1U << chip_select);
	}

	/* Calculate baudrate for SPI module */
	if ((spi_cfg->frequency > 0) &&
				(SPI_OP_MODE_GET(spi_cfg->operation) != SPI_OP_MODE_SLAVE)) {
		spi_config_extend->mspi_baudrate_channel =
						spi_rh850_baudrate_calculate(spi_cfg->frequency);
	}

	/* Enable loopback mode */
	if (spi_cfg->operation & SPI_MODE_LOOP) {
		spi_config_extend->mspi_loopback = MSPI_LOOPBACK_MODE_ENABLE;
	} else {
		spi_config_extend->mspi_loopback = MSPI_LOOPBACK_MODE_DISABLE;
	}


#ifndef CONFIG_SPI_RENESAS_RH850_INTERRUPT
	data->fsp_config->txi_ipl = BSP_IRQ_DISABLED;
	data->fsp_config->rxi_ipl = BSP_IRQ_DISABLED;
	data->fsp_config->eri_ipl = BSP_IRQ_DISABLED;
	data->fsp_config->tei_ipl = BSP_IRQ_DISABLED;
#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */

	/* Open module MSPI. */
	err = config->fsp_api->open(spi_ctrl, data->fsp_config);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: R_MSPI_Open error: %d", dev->name, err);
		return -EIO;
	}

#ifdef BSP_FEATURE_DEVICE_HAS_ORED_IRQ
#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT
	/* Enable FE and ERR ORED interrupt once. */
	if (true == ored_irq_not_init) {
		IRQ_CONNECT(VECTOR_NUMBER_INTMSPICSERR0,
			data->fsp_config->eri_ipl, spi_rh850_eri_isr,
			NULL, 0);
		IRQ_CONNECT(VECTOR_NUMBER_INTMSPICSFE0,
				data->fsp_config->tei_ipl, spi_rh850_fe_isr,
				NULL, 0);
		irq_enable(VECTOR_NUMBER_INTMSPICSERR0);
		irq_enable(VECTOR_NUMBER_INTMSPICSFE0);

		ored_irq_not_init = false;
	}
#else
	/* Disable ORED interrupt for polling mode */
	R_BSP_ORed_IrqCfgDisable(spi_config_extend->error_ored_int);
	R_BSP_ORed_IrqCfgDisable(spi_config_extend->frame_ored_int);
#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */
#endif /* BSP_FEATURE_DEVICE_HAS_ORED_IRQ */

	data->ctx.config = spi_cfg;

	return 0;
}

static int transceive(const struct device *dev, const struct spi_config *spi_cfg,
			  const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
			  bool asynchronous, spi_callback_t cb, void *userdata)
{
	struct spi_rh850_data *data = dev->data;
	struct spi_context *spi_ctx = &data->ctx;
	int ret = 0;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

#ifndef CONFIG_SPI_RENESAS_RH850_INTERRUPT
	/* Asynchronous transfers are not supported in polling mode */
	if (asynchronous) {
		return -ENOTSUP;
	}
#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */

	spi_context_lock(spi_ctx, asynchronous, cb, userdata, spi_cfg);

	/* Configure module SPI. */
	ret = spi_rh850_configure(dev, spi_cfg);
	if (ret) {
		spi_context_release(spi_ctx, ret);
		return ret;
	}

	/* Setup tx buffer and rx buffer info. */
	spi_context_buffers_setup(spi_ctx, tx_bufs, rx_bufs, data->dfs);
	spi_context_cs_control(spi_ctx, true);

	if (!spi_context_total_tx_len(&data->ctx) && !spi_context_total_rx_len(&data->ctx)) {
		goto end_transceive;
	}


#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT
	/* Start transfer */
	ret = spi_rh850_transfer_next_packet(dev);

	if (!ret) {
		/* Wait for the transfer to be completed */
		ret = spi_context_wait_for_completion(spi_ctx);
	} else {
		/* Transfer error */
		LOG_ERR("%s: Async transfer fail: %d", dev->name, ret);
		spi_context_cs_control(spi_ctx, false);
		spi_context_release(spi_ctx, ret);
		return ret;
	}
#else
	do {
		/* Start transfer */
		ret = spi_rh850_transfer_next_packet(dev);

		/* If transfer complete successfully, update the TX/RX context */
		if (!ret) {
			ret = spi_rh850_data_complete_update(dev);
		}
	} while (!ret && !spi_rh850_transfer_done(spi_ctx));

#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */

end_transceive:

#ifdef CONFIG_SPI_SLAVE
	/* If it's in slave mode, update the return value with the receive frames value */
	if (spi_context_is_slave(spi_ctx) && !ret) {
		ret = spi_ctx->recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

	spi_context_cs_control(spi_ctx, false);

	spi_context_release(spi_ctx, ret);

	return ret;
}

static int spi_rh850_transceive_sync(const struct device *dev, const struct spi_config *spi_cfg,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

static int spi_rh850_release(const struct device *dev, const struct spi_config *config)
{
	ARG_UNUSED(config);
	struct spi_rh850_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_rh850_transceive_async(const struct device *dev, const struct spi_config *spi_cfg,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				   void *userdata)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static DEVICE_API(spi, spi_rh850_driver_api) = {
	.transceive = spi_rh850_transceive_sync,
	.release = spi_rh850_release,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_rh850_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
};

#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT

static void spi_rh850_rxi_isr(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Call the MSPI RX handler */
	mspi_rxi_isr();
}

static void spi_rh850_txi_isr(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Call the MSPI TX handler */
	mspi_txi_isr();
}

static void spi_rh850_fe_isr(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Call the MSPI FE handler */
	mspi_fe_isr();
}

static void spi_rh850_eri_isr(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Call the MSPI ERR handler */
	mspi_err_isr();
}

#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */

static int spi_rh850_init(const struct device *dev)
{
	const struct spi_rh850_config *config = dev->config;
	struct spi_rh850_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(config->pinctrl_dev, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s: pinctrl_apply_state fail: %d", dev->name, ret);
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		LOG_ERR("%s: spi_context_cs_configure_all fail: %d", dev->name, ret);
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_RENESAS_RH850_INTERRUPT
#ifdef BSP_FEATURE_DEVICE_HAS_ORED_IRQ
#define RH850_SPI_IRQ_INIT(n) \
	do {										   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, rxi, irq),				   \
				DT_INST_IRQ_BY_NAME(n, rxi, priority), spi_rh850_rxi_isr,  \
				DEVICE_DT_INST_GET(n), 0);				   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, txi, irq),				   \
				DT_INST_IRQ_BY_NAME(n, txi, priority), spi_rh850_txi_isr,  \
				DEVICE_DT_INST_GET(n), 0);				   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, rxi, irq));				   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, txi, irq));				   \
	} while (0)

#else
#define RH850_SPI_IRQ_INIT(n)								   \
	do {										   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, rxi, irq),				   \
				DT_INST_IRQ_BY_NAME(n, rxi, priority), spi_rh850_rxi_isr,  \
				DEVICE_DT_INST_GET(n), 0);				   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, txi, irq),				   \
				DT_INST_IRQ_BY_NAME(n, txi, priority), spi_rh850_txi_isr,  \
				DEVICE_DT_INST_GET(n), 0);				   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, eri, irq),				   \
				DT_INST_IRQ_BY_NAME(n, eri, priority), spi_rh850_eri_isr,  \
				DEVICE_DT_INST_GET(n), 0);				   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, tei, irq),				   \
				DT_INST_IRQ_BY_NAME(n, tei, priority), spi_rh850_fe_isr,   \
				DEVICE_DT_INST_GET(n), 0);				   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, rxi, irq));				   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, txi, irq));				   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, eri, irq));				   \
		irq_enable(DT_INST_IRQ_BY_NAME(n, tei, irq));				   \
	} while (0)

#endif /* BSP_FEATURE_DEVICE_HAS_ORED_IRQ */

#else
#define RH850_SPI_IRQ_INIT(n)

#endif /* CONFIG_SPI_RENESAS_RH850_INTERRUPT */

#ifdef BSP_FEATURE_DEVICE_HAS_ORED_IRQ
#define SPI_RH850_ORED_CONFIG(n)						  \
	.com_ored_int = BSP_ORED_MSPI_DISABLE,					  \
	.rec_ored_int = BSP_ORED_MSPI_DISABLE,					  \
	.frame_ored_int = BSP_ORED_FRAMECOUNT_MSPI##n##_ORED0,			  \
	.error_ored_int = BSP_ORED_ERROR_MSPI##n##_ORED0,
#else
#define SPI_RH850_ORED_CONFIG(n)

#endif /* BSP_FEATURE_DEVICE_HAS_ORED_IRQ */

#define SPI_RH850_INIT(n)									   \
	PINCTRL_DT_INST_DEFINE(n);							           \
	static mspi_instance_ctrl_t g_spi##n##_ctrl;						   \
												   \
	static mspi_chip_select_cfg0_t g_spi##n##_chip_select_cfg00 =				   \
	{											   \
		.mspi_chip_select_cfg0.mspi_txe_inr = MSPI_INTERRUPT_OUT_ENABLE,		   \
		.mspi_chip_select_cfg0.mspi_rxe_inr = MSPI_INTERRUPT_OUT_ENABLE,		   \
		.mspi_chip_select_cfg0.mspi_fee_inr = MSPI_INTERRUPT_OUT_ENABLE,		   \
		.mspi_chip_select_cfg0.mspi_ere_inr = MSPI_INTERRUPT_OUT_ENABLE,		   \
		.mspi_chip_select_cfg0.mspi_frame_end = MSPI_CHANNEL_FRAME_END_CLEAR,		   \
		.mspi_chip_select_cfg0.mspi_lock = MSPI_CHANNEL_LOCK_DISABLE,			   \
		.mspi_chip_select_cfg0.mspi_priority = MSPI_CHANNEL_PRIORITY_8,			   \
		.mspi_chip_select_cfg0.mspi_transfer_mode = MSPI_TRANSFER_MODE_DIRECT_ACCESS,	   \
		.mspi_chip_select_cfg0.mspi_enable_rec = MSPI_RECEPTION_ENABLE,			   \
		.mspi_chip_select_cfg0.mspi_enable_trans = MSPI_TRANSMISSION_ENABLE		   \
	};											   \
												   \
	static mspi_chip_select_cfg1_t g_spi##n##_chip_select_cfg01 =				   \
	{											   \
		.mspi_chip_select_cfg1.mspi_sck_polarity = MSPI_POLARITY_LOW,			   \
		.mspi_chip_select_cfg1.mspi_phase_select = MSPI_PHASE_SELECT_EVEN,		   \
		.mspi_chip_select_cfg1.mspi_direction = SPI_BIT_ORDER_MSB_FIRST,		   \
		.mspi_chip_select_cfg1.mspi_idle_level = MSPI_IDLE_LEVEL_LOW,			   \
		.mspi_chip_select_cfg1.mspi_idle_end = MSPI_INSERT_IDLE_END_DISABLE,		   \
		.mspi_chip_select_cfg1.mspi_idle_last = MSPI_INSERT_IDLE_LAST_ACTIVE,		   \
		.mspi_chip_select_cfg1.mspi_crc_mask_enable = MSPI_CRC_MASK_DISABLE,		   \
		.mspi_chip_select_cfg1.mspi_safe_format = MSPI_SAFE_FORMAT_IN,			   \
		.mspi_chip_select_cfg1.mspi_safe_enable = MSPI_SAFE_DISABLE,			   \
		.mspi_chip_select_cfg1.mspi_odd_parity = MSPI_NO_PARITY,			   \
		.mspi_chip_select_cfg1.mspi_parity_check = MSPI_PARITY_CHECK_DISABLE		   \
	};											   \
												   \
	static mspi_chip_select_cfg4_t g_spi##n##_chip_select_cfg04 =				   \
	{											   \
		.mspi_chip_select_cfg4.mspi_hw_trigger = MSPI_HARDWARE_TRIGGER_DISABLE,		   \
		.mspi_chip_select_cfg4.mspi_fifo_stage_size = MSPI_FIFO_STAGE_SIZE_8		   \
	};											   \
												   \
	static mspi_extended_cfg_t g_spi##n##_ext_cfg =						   \
	{											   \
		.mspi_ctl1.mspi_bit_control_register_1.mspi_mode_selection = SPI_MODE_MASTER,	   \
		.mspi_ctl1.mspi_bit_control_register_1.mspi_slave_select			   \
								= MSPI_SLAVE_SELECT_IGNORED,	   \
		.mspi_ctl1.mspi_bit_control_register_1.mspi_internal_sampling			   \
								= MSPI_INTERNAL_SAMPLING_STANDARD, \
		.mspi_ctl1.mspi_bit_control_register_1.mspi_clock_level = MSPI_CLOCK_LEVEL_LOW,	   \
		.mspi_ctl1.mspi_bit_control_register_1.mspi_sout_function = MSPI_SOUT_FUNC_LOW_0,  \
		.mspi_ctl1.mspi_bit_control_register_1.mspi_control_polarity			   \
								= MSPI_CONTROL_POLARITY_LOW,	   \
		.mspi_loopback = MSPI_LOOPBACK_MODE_DISABLE,					   \
		.mspi_consistency_check = MSPI_CONSISTENCY_CHECK_DISABLE,			   \
		.mspi_select = MSPI_SELECTABLE_MODE_TRANSMIT_RECEIVE,				   \
		.mspi_chip_selection = MSPI_CHIP_SELECT_STATE_ACTIVE,				   \
		.mspi_baudrate_channel = 266,							   \
		.p_mspi_config_cfg0 = &g_spi##n##_chip_select_cfg00,				   \
		.p_mspi_config_cfg1 = &g_spi##n##_chip_select_cfg01,				   \
		.p_mspi_config_cfg4 = &g_spi##n##_chip_select_cfg04,				   \
		.mspi_setup_time = DT_INST_PROP_OR(n, spi_cs_sck_delay, 4095U),			   \
		.mspi_hold_time = DT_INST_PROP_OR(n, spi_sck_cs_delay, 1U),			   \
		.mspi_idle_time = DT_INST_PROP_OR(n, spi_cs_cs_delay, 1U),			   \
		.mspi_inter_data = DT_INST_PROP_OR(n, spi_sck_sck_delay, 4095U),		   \
		.mspi_ram_start = 0x0000,							   \
		.mspi_tx_isr_mask = MSPI_MASKED_INTERRUPT_DISABLE,				   \
		.mspi_rx_isr_mask = MSPI_MASKED_INTERRUPT_DISABLE,				   \
		.mspi_fe_isr_mask = MSPI_MASKED_INTERRUPT_DISABLE,				   \
		.mspi_err_isr_mask = MSPI_MASKED_INTERRUPT_DISABLE,				   \
		.mspi_transfer_trigger = MSPI_TRANSFER_TRIGGER_DMA,				   \
		.mspi_trigger_source = MSPI_TRIGGER_SOURCE_NONE,				   \
		.unit = n,									   \
		.mspi_dma_alternative = MSPI_TRANSFER_ALTERNATIVE_DISABLE,			   \
		.mspi_dts_alternative = MSPI_TRANSFER_ALTERNATIVE_DISABLE,			   \
		SPI_RH850_ORED_CONFIG(n)                                                           \
		.mspi_number_of_cs = BSP_FEATURE_MSPI_NUMBER_CHIP_SELECT_UNIT_##n		   \
	};											   \
												   \
	static spi_cfg_t g_spi##n##_cfg =							   \
	{											   \
		.operating_mode = SPI_MODE_MASTER,						   \
		.channel = DT_INST_PROP(n, channel),						   \
		.txi_irq = DT_INST_IRQ_BY_NAME(n, txi, irq),					   \
		.rxi_irq = DT_INST_IRQ_BY_NAME(n, rxi, irq),					   \
		.tei_irq = DT_INST_IRQ_BY_NAME(n, tei, irq),					   \
		.eri_irq = DT_INST_IRQ_BY_NAME(n, eri, irq),					   \
		.txi_ipl = DT_INST_IRQ_BY_NAME(n, txi, priority),				   \
		.rxi_ipl = DT_INST_IRQ_BY_NAME(n, rxi, priority),				   \
		.tei_ipl = DT_INST_IRQ_BY_NAME(n, tei, priority),				   \
		.eri_ipl = DT_INST_IRQ_BY_NAME(n, eri, priority),				   \
		.p_callback = spi_rh850_transfer_callback,					   \
		.p_transfer_tx = NULL,								   \
		.p_transfer_rx = NULL,								   \
		.p_context = (void *)DEVICE_DT_INST_GET(n),					   \
		.p_extend = (void *) &g_spi##n##_ext_cfg					   \
	};											   \
												   \
	static const struct spi_rh850_config spi_rh850_config_##n = {				   \
		.pinctrl_dev = PINCTRL_DT_INST_DEV_CONFIG_GET(n),				   \
		.fsp_api = &g_spi_on_mspi,							   \
	};											   \
												   \
	static struct spi_rh850_data spi_rh850_data_##n = {					   \
		SPI_CONTEXT_INIT_LOCK(spi_rh850_data_##n, ctx),					   \
		SPI_CONTEXT_INIT_SYNC(spi_rh850_data_##n, ctx),					   \
		.fsp_ctrl = &g_spi##n##_ctrl,							   \
		.fsp_config = &g_spi##n##_cfg,							   \
	};											   \
												   \
	static int spi_rh850_init_##n(const struct device *dev)					   \
	{											   \
		int err = spi_rh850_init(dev);							   \
		if (err != 0) {									   \
			return err;								   \
		}										   \
		RH850_SPI_IRQ_INIT(n);								   \
		return 0;									   \
	}											   \
	DEVICE_DT_INST_DEFINE(n, &spi_rh850_init_##n, NULL, &spi_rh850_data_##n,		   \
	&spi_rh850_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_RH850_INIT)
