/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_uart_sci_b

#include <r_sci_b_uart.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_RENESAS_RX_GRP_INTC_FSP
#include <zephyr/drivers/interrupt_controller/intc_renesas_rx_grp_int_fsp.h>
#endif /* CONFIG_RENESAS_RX_GRP_INTC_FSP */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(renesas_rx_uart_sci_b, CONFIG_UART_LOG_LEVEL);

struct uart_renesas_rx_sci_b_config {
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	struct clock_control_rx_subsys_cfg clock_subsys;
	R_SCI_B0_Type *const regs;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uint8_t rxi_irq;
	uint8_t txi_irq;
#ifndef CONFIG_RENESAS_RX_GRP_INTC_FSP
	uint8_t tei_irq;
	uint8_t eri_irq;
#else  /* CONFIG_RENESAS_RX_GRP_INTC_FSP */
	/*  Group interrupt controller for the transmit end interrupt (TEI) */
	const struct device *tei_ctrl;
	/*  Group interrupt number for the transmit end interrupt (TEI) */
	uint8_t tei_num;
	/*  Group interrupt controller for the error interrupt (ERI) */
	const struct device *eri_ctrl;
	/*  Group interrupt number for the error interrupt (ERI) */
	uint8_t eri_num;
#endif /* CONFIG_RENESAS_RX_GRP_INTC_FSP */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct uart_renesas_rx_sci_b_data {
	const struct device *dev;
	struct st_sci_b_uart_instance_ctrl sci;
	struct uart_config uart_config;
	struct st_uart_cfg fsp_config;
	struct st_sci_b_uart_extended_cfg fsp_config_extend;
	struct st_sci_b_baud_setting_t fsp_baud_setting;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
	uint32_t sci_status;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int uart_renesas_rx_sci_b_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	if (cfg->regs->SSR_b.RDRF == 0U) {
		/* There are no characters available to read. */
		return -1;
	}

	/* got a character */
	*c = (unsigned char)cfg->regs->RDR;

	if (IS_ENABLED(CONFIG_UART_RENESAS_RX_SCI_B_UART_FIFO_ENABLE) && data->sci.fifo_depth > 0) {
		/* In case using FIFO, the RDRF flag only be cleared when set 1 to RDRFC */
		cfg->regs->SSCR_b.RDRFC = 1;
	}

	return 0;
}

static void uart_renesas_rx_sci_b_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	while (cfg->regs->SSR_b.TEND == 0U) {
	}

	cfg->regs->TDR_BY = c;

	while (cfg->regs->SSR_b.TEND == 0U) {
	}
}

static int uart_renesas_rx_sci_b_err_check(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	const uint32_t status = cfg->regs->SSR;
	int errors = 0;

	if ((status & R_SCI_B0_SSR_ORER_Msk) != 0) {
		errors |= UART_ERROR_OVERRUN;
	}
	if ((status & R_SCI_B0_SSR_APER_Msk) != 0) {
		errors |= UART_ERROR_PARITY;
	}
	if ((status & R_SCI_B0_SSR_AFER_Msk) != 0) {
		errors |= UART_ERROR_FRAMING;
	}

	return errors;
}

static int uart_rx_sci_b_apply_config(const struct uart_config *config,
				      struct st_uart_cfg *fsp_config,
				      struct st_sci_b_uart_extended_cfg *fsp_config_extend,
				      struct st_sci_b_baud_setting_t *fsp_baud_setting)
{
	fsp_err_t fsp_err;

	fsp_err = R_SCI_B_UART_BaudCalculate(config->baudrate, false, 5000, fsp_baud_setting);
	__ASSERT(fsp_err == 0, "sci_uart: baud calculate error");

	switch (config->parity) {
	case UART_CFG_PARITY_NONE:
		fsp_config->parity = UART_PARITY_OFF;
		break;
	case UART_CFG_PARITY_ODD:
		fsp_config->parity = UART_PARITY_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		fsp_config->parity = UART_PARITY_EVEN;
		break;
	case UART_CFG_PARITY_MARK:
		return -ENOTSUP;
	case UART_CFG_PARITY_SPACE:
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	switch (config->stop_bits) {
	case UART_CFG_STOP_BITS_0_5:
		return -ENOTSUP;
	case UART_CFG_STOP_BITS_1:
		fsp_config->stop_bits = UART_STOP_BITS_1;
		break;
	case UART_CFG_STOP_BITS_1_5:
		return -ENOTSUP;
	case UART_CFG_STOP_BITS_2:
		fsp_config->stop_bits = UART_STOP_BITS_2;
		break;
	default:
		return -EINVAL;
	}

	switch (config->data_bits) {
	case UART_CFG_DATA_BITS_5:
		return -ENOTSUP;
	case UART_CFG_DATA_BITS_6:
		return -ENOTSUP;
	case UART_CFG_DATA_BITS_7:
		fsp_config->data_bits = UART_DATA_BITS_7;
		break;
	case UART_CFG_DATA_BITS_8:
		fsp_config->data_bits = UART_DATA_BITS_8;
		break;
	case UART_CFG_DATA_BITS_9:
		fsp_config->data_bits = UART_DATA_BITS_9;
		break;
	default:
		return -EINVAL;
	}

	fsp_config_extend->clock = SCI_B_UART_CLOCK_INT;
	fsp_config_extend->rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE;
	fsp_config_extend->noise_cancel = SCI_B_UART_NOISE_CANCELLATION_DISABLE;
	fsp_config_extend->flow_control_pin = UINT16_MAX;

	switch (config->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		fsp_config_extend->flow_control = 0;
		fsp_config_extend->rs485_setting.enable = false;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		fsp_config_extend->flow_control = SCI_B_UART_FLOW_CONTROL_HARDWARE_CTSRTS;
		fsp_config_extend->rs485_setting.enable = false;
		break;
	case UART_CFG_FLOW_CTRL_DTR_DSR:
		return -ENOTSUP;
	case UART_CFG_FLOW_CTRL_RS485:
		/* TODO: implement this config */
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_renesas_rx_sci_b_configure(const struct device *dev, const struct uart_config *cfg)
{
	int err;
	fsp_err_t fsp_err;
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	err = uart_rx_sci_b_apply_config(cfg, &data->fsp_config, &data->fsp_config_extend,
					 &data->fsp_baud_setting);
	if (err) {
		return err;
	}

	fsp_err = R_SCI_B_UART_Close(&data->sci);
	__ASSERT(fsp_err == 0, "sci_uart: configure: fsp close failed");

	fsp_err = R_SCI_B_UART_Open(&data->sci, &data->fsp_config);
	__ASSERT(fsp_err == 0, "sci_uart: configure: fsp open failed");

	memcpy(&data->uart_config, cfg, sizeof(struct uart_config));

	return 0;
}

static int uart_renesas_rx_sci_b_config_get(const struct device *dev, struct uart_config *cfg)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	memcpy(cfg, &data->uart_config, sizeof(*cfg));
	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_renesas_rx_sci_b_eri_callback(void)
{
	IRQn_Type irq = R_FSP_CurrentIrqGet();
	struct st_sci_b_uart_instance_ctrl *sci =
		(struct st_sci_b_uart_instance_ctrl *)R_FSP_IsrContextGet(irq);
	const struct device *dev = *(struct device **)sci->p_context;
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	if (data->user_cb != NULL) {
		data->user_cb(dev, data->user_data);
	}
}

static void uart_renesas_rx_sci_b_tei_callback(void)
{
	IRQn_Type irq = R_FSP_CurrentIrqGet();
	struct st_sci_b_uart_instance_ctrl *sci =
		(struct st_sci_b_uart_instance_ctrl *)R_FSP_IsrContextGet(irq);
	const struct device *dev = *(struct device **)sci->p_context;
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	if (data->user_cb != NULL) {
		data->user_cb(dev, data->user_data);
	}
}

static int uart_renesas_rx_sci_b_fifo_fill(const struct device *dev, const uint8_t *tx_data,
					   int size)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;
	int num_tx = 0U;

	if (IS_ENABLED(CONFIG_UART_RENESAS_RX_SCI_B_UART_FIFO_ENABLE) && data->sci.fifo_depth > 0) {
		while ((size - num_tx > 0) && cfg->regs->TFSR != 0x10U) {
			/* TFSR flag will be cleared with byte write to TDR register */

			/* Send a character (8bit , parity none) */
			cfg->regs->TDR_BY = tx_data[num_tx++];
		}
	} else {
		if (size > 0 && cfg->regs->SSR_b.TDRE) {
			/* TDRE flag will be cleared with byte write to TDR register */

			/* Send a character (8bit , parity none) */
			cfg->regs->TDR_BY = tx_data[num_tx++];
		}
	}

	return num_tx;
}

static int uart_renesas_rx_sci_b_fifo_read(const struct device *dev, uint8_t *rx_data,
					   const int size)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;
	int num_rx = 0U;

	if (IS_ENABLED(CONFIG_UART_RENESAS_RX_SCI_B_UART_FIFO_ENABLE) && data->sci.fifo_depth > 0) {
		while ((size - num_rx > 0) && cfg->regs->RFSR_b.R > 0U) {
			/* FRSR.DR flag will be cleared with byte write to RDR register */

			/* Receive a character (8bit , parity none) */
			rx_data[num_rx++] = cfg->regs->RDR;
		}
		if (cfg->regs->RFSR_b.R == 0U) {
			cfg->regs->SSCR_b.RDRFC = 1U;
			cfg->regs->RFSCR_b.DRC = 1U;
		}
	} else {
		if (size > 0 && cfg->regs->SSR_b.RDRF) {
			/* Receive a character (8bit , parity none) */
			rx_data[num_rx++] = cfg->regs->RDR;
		}
	}

	/* Clear overrun error flag */
	cfg->regs->SSCR_b.ORERC = 1U;

	return num_rx;
}

static void uart_renesas_rx_sci_b_irq_tx_enable(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	cfg->regs->SCR0 |= (R_SCI_B0_SCR0_TIE_Msk | R_SCI_B0_SCR0_TEIE_Msk);
}

static void uart_renesas_rx_sci_b_irq_tx_disable(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	cfg->regs->SCR0 &= ~(R_SCI_B0_SCR0_TIE_Msk | R_SCI_B0_SCR0_TEIE_Msk);
}

static int uart_renesas_rx_sci_b_irq_tx_ready(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	return (cfg->regs->SCR0_b.TIE == 1U) &&
	       (data->sci_status & (R_SCI_B0_SSR_TDRE_Msk | R_SCI_B0_SSR_TEND_Msk));
}

static void uart_renesas_rx_sci_b_irq_rx_enable(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	cfg->regs->SCR0_b.RIE = 1U;
}

static void uart_renesas_rx_sci_b_irq_rx_disable(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	cfg->regs->SCR0_b.RIE = 0U;
}

static int uart_renesas_rx_sci_b_irq_tx_complete(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	return (cfg->regs->SCR0_b.TEIE == 1U) && (data->sci_status & R_SCI_B0_SSR_TEND_Msk);
}

static int uart_renesas_rx_sci_b_irq_rx_ready(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	return (cfg->regs->SCR0_b.RIE == 1U) &&
	       ((data->sci_status & R_SCI_B0_SSR_RDRF_Msk) ||
		(IS_ENABLED(CONFIG_UART_RENESAS_RX_SCI_B_UART_FIFO_ENABLE) &&
		 cfg->regs->RFSR_b.DR == 1U));
}

static void uart_renesas_rx_sci_b_irq_err_enable(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

#ifndef CONFIG_RENESAS_RX_GRP_INTC_FSP
	irq_enable(cfg->eri_irq);
#else
	int err;

	err = rx_grp_intc_enable(cfg->eri_ctrl, cfg->eri_num);
	if (err != 0) {
		LOG_ERR("Failed to allow interrupt request for ERI: %d", err);
		return;
	}
#endif /* CONFIG_RENESAS_RX_GRP_INTC */
}

static void uart_renesas_rx_sci_b_irq_err_disable(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

#ifndef CONFIG_RENESAS_RX_GRP_INTC_FSP
	irq_disable(cfg->eri_irq);
#else
	int err;

	err = rx_grp_intc_disable(cfg->eri_ctrl, cfg->eri_num);
	if (err != 0) {
		LOG_ERR("Failed disable ERI interrupt: %d", err);
		return;
	}
#endif /* CONFIG_RENESAS_RX_GRP_INTC */
}

static int uart_renesas_rx_sci_b_irq_is_pending(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;

	const uint32_t scr0 = cfg->regs->SCR0;
	const uint32_t ssr = cfg->regs->SSR;

	const bool tx_pending = ((scr0 & R_SCI_B0_SCR0_TIE_Msk) &&
				 (ssr & (R_SCI_B0_SSR_TEND_Msk | R_SCI_B0_SSR_TDRE_Msk)));
	const bool rx_pending = ((scr0 & R_SCI_B0_SCR0_RIE_Msk) &&
				 ((ssr & (R_SCI_B0_SSR_RDRF_Msk | R_SCI_B0_SSR_APER_Msk |
					  R_SCI_B0_SSR_AFER_Msk | R_SCI_B0_SSR_ORER_Msk)) ||
				  (IS_ENABLED(CONFIG_UART_RENESAS_RX_SCI_B_UART_FIFO_ENABLE) &&
				   cfg->regs->RFSR_b.DR == 1U)));

	return tx_pending || rx_pending;
}

static int uart_renesas_rx_sci_b_irq_update(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;
	const struct uart_renesas_rx_sci_b_config *cfg = dev->config;
	uint32_t sscr = 0;

	data->sci_status = cfg->regs->SSR;

	if (data->sci_status & R_SCI_B0_SSR_APER_Msk) {
		sscr |= R_SCI_B0_SSCR_APERC_Msk;
	}
	if (data->sci_status & R_SCI_B0_SSR_AFER_Msk) {
		sscr |= R_SCI_B0_SSCR_AFERC_Msk;
	}
	if (data->sci_status & R_SCI_B0_SSR_ORER_Msk) {
		sscr |= R_SCI_B0_SSCR_ORERC_Msk;
	}

	cfg->regs->SSCR = sscr;

	return 1;
}

static void uart_renesas_rx_sci_b_irq_callback_set(const struct device *dev,
						   uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	data->user_cb = cb;
	data->user_data = cb_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static DEVICE_API(uart, uart_renesas_rx_sci_b_driver_api) = {
	.poll_in = uart_renesas_rx_sci_b_poll_in,
	.poll_out = uart_renesas_rx_sci_b_poll_out,
	.err_check = uart_renesas_rx_sci_b_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_renesas_rx_sci_b_configure,
	.config_get = uart_renesas_rx_sci_b_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_renesas_rx_sci_b_fifo_fill,
	.fifo_read = uart_renesas_rx_sci_b_fifo_read,
	.irq_tx_enable = uart_renesas_rx_sci_b_irq_tx_enable,
	.irq_tx_disable = uart_renesas_rx_sci_b_irq_tx_disable,
	.irq_tx_ready = uart_renesas_rx_sci_b_irq_tx_ready,
	.irq_rx_enable = uart_renesas_rx_sci_b_irq_rx_enable,
	.irq_rx_disable = uart_renesas_rx_sci_b_irq_rx_disable,
	.irq_tx_complete = uart_renesas_rx_sci_b_irq_tx_complete,
	.irq_rx_ready = uart_renesas_rx_sci_b_irq_rx_ready,
	.irq_err_enable = uart_renesas_rx_sci_b_irq_err_enable,
	.irq_err_disable = uart_renesas_rx_sci_b_irq_err_disable,
	.irq_is_pending = uart_renesas_rx_sci_b_irq_is_pending,
	.irq_update = uart_renesas_rx_sci_b_irq_update,
	.irq_callback_set = uart_renesas_rx_sci_b_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int uart_renesas_rx_sci_b_init(const struct device *dev)
{
	const struct uart_renesas_rx_sci_b_config *config = dev->config;
	struct uart_renesas_rx_sci_b_data *data = dev->data;
	int ret;
	fsp_err_t fsp_err;

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_on(config->clock_dev, (clock_control_subsys_t)&config->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	/* Setup fsp sci_uart setting */
	ret = uart_rx_sci_b_apply_config(&data->uart_config, &data->fsp_config,
					 &data->fsp_config_extend, &data->fsp_baud_setting);
	if (ret != 0) {
		return ret;
	}

	data->fsp_config_extend.p_baud_setting = &data->fsp_baud_setting;
	data->fsp_config.p_extend = &data->fsp_config_extend;
	data->fsp_config.p_context = &data->dev;

	fsp_err = R_SCI_B_UART_Open(&data->sci, &data->fsp_config);
	__ASSERT(fsp_err == 0, "sci_uart: initialization: open failed");

	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_rx_sci_b_rxi_isr(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	if (data->user_cb != NULL) {
		data->user_cb(dev, data->user_data);
	}
}

static void uart_rx_sci_b_txi_isr(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	if (data->user_cb != NULL) {
		data->user_cb(dev, data->user_data);
	}
}

#define ICU_EVENT_SCI_RXI(channel) CONCAT(ICU_EVENT_RSCI, channel, _RXI)
#define ICU_EVENT_SCI_TXI(channel) CONCAT(ICU_EVENT_RSCI, channel, _TXI)

#define UART_RX_SCI_B_TXI_RXI_INIT(index)                                                          \
	do {                                                                                       \
		int rxi_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), rxi, irq);                     \
		int txi_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), txi, irq);                     \
                                                                                                   \
		if (128 <= rxi_irq && rxi_irq < 144) {                                             \
			R_ICU->SLIXR[rxi_irq - 128].SLIXR =                                        \
				ICU_EVENT_SCI_RXI(DT_PROP(DT_INST_PARENT(index), channel));        \
		} else if (144 <= rxi_irq) {                                                       \
			R_ICU->SLIR[rxi_irq - 144].SLIR =                                          \
				ICU_EVENT_SCI_RXI(DT_PROP(DT_INST_PARENT(index), channel));        \
		}                                                                                  \
                                                                                                   \
		if (128 <= txi_irq && txi_irq < 144) {                                             \
			R_ICU->SLIXR[txi_irq - 128].SLIXR =                                        \
				ICU_EVENT_SCI_TXI(DT_PROP(DT_INST_PARENT(index), channel));        \
		} else if (144 <= txi_irq) {                                                       \
			R_ICU->SLIR[txi_irq - 144].SLIR =                                          \
				ICU_EVENT_SCI_TXI(DT_PROP(DT_INST_PARENT(index), channel));        \
		}                                                                                  \
                                                                                                   \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST_PARENT(index), rxi, irq),                       \
			    DT_IRQ_BY_NAME(DT_INST_PARENT(index), rxi, priority),                  \
			    uart_rx_sci_b_rxi_isr, DEVICE_DT_INST_GET(index), 0);                  \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST_PARENT(index), txi, irq),                       \
			    DT_IRQ_BY_NAME(DT_INST_PARENT(index), txi, priority),                  \
			    uart_rx_sci_b_txi_isr, DEVICE_DT_INST_GET(index), 0);                  \
		irq_enable(rxi_irq);                                                               \
		irq_enable(txi_irq);                                                               \
	} while (0)

#ifndef CONFIG_RENESAS_RX_GRP_INTC_FSP
static void uart_rx_sci_b_tei_isr(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	if (data->user_cb != NULL) {
		data->user_cb(dev, data->user_data);
	}
}

static void uart_rx_sci_b_eri_isr(const struct device *dev)
{
	struct uart_renesas_rx_sci_b_data *data = dev->data;

	if (data->user_cb != NULL) {
		data->user_cb(dev, data->user_data);
	}
}

#define UART_RX_SCI_B_IRQ_CONFIG_INIT(index)                                                       \
	.rxi_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), rxi, irq),                                \
	.txi_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), txi, irq),                                \
	.tei_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), tei, irq),                                \
	.eri_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), eri, irq),

#define UART_RX_SCI_B_TEI_ERI_INIT(index)                                                          \
	do {                                                                                       \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST_PARENT(index), tei, irq),                       \
			    DT_IRQ_BY_NAME(DT_INST_PARENT(index), tei, priority),                  \
			    uart_rx_sci_b_tei_isr, DEVICE_DT_INST_GET(index), 0);                  \
		IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST_PARENT(index), eri, irq),                       \
			    DT_IRQ_BY_NAME(DT_INST_PARENT(index), eri, priority),                  \
			    uart_rx_sci_b_eri_isr, DEVICE_DT_INST_GET(index), 0);                  \
		irq_enable(DT_IRQ_BY_NAME(DT_INST_PARENT(index), tei, irq));                       \
	} while (0)

#else /* CONFIG_RENESAS_RX_GRP_INTC_FSP */

#define UART_RX_SCI_B_TEI_ERI_INIT(index)                                                          \
	do {                                                                                       \
		const struct uart_renesas_rx_sci_b_config *cfg = dev->config;                      \
		struct uart_renesas_rx_sci_b_data *data = dev->data;                               \
		int err = 0;                                                                       \
		err = rx_grp_intc_callback_set(cfg->eri_ctrl, cfg->eri_num,                        \
					       uart_renesas_rx_sci_b_eri_callback,                 \
					       (void *)&data->sci);                                \
		if (err != 0) {                                                                    \
			LOG_ERR("Failed to set callback for group interrupt ERI: %d", err);        \
			return err;                                                                \
		}                                                                                  \
		err = rx_grp_intc_callback_set(cfg->tei_ctrl, cfg->tei_num,                        \
					       uart_renesas_rx_sci_b_tei_callback,                 \
					       (void *)&data->sci);                                \
		if (err != 0) {                                                                    \
			LOG_ERR("Failed to set callback for group interrupt TEI: %d", err);        \
			return err;                                                                \
		}                                                                                  \
                                                                                                   \
		err = rx_grp_intc_enable(cfg->tei_ctrl, cfg->tei_num);                             \
		if (err != 0) {                                                                    \
			LOG_ERR("Failed to allow interrupt request for TEI: %d", err);             \
			return err;                                                                \
		}                                                                                  \
	} while (0)

#define UART_RX_SCI_B_IRQ_CONFIG_INIT(index)                                                       \
	.rxi_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), rxi, irq),                                \
	.txi_irq = DT_IRQ_BY_NAME(DT_INST_PARENT(index), txi, irq),                                \
	.tei_ctrl = DEVICE_DT_GET(DT_PHANDLE(DT_INST_PARENT(index), tei_ctrl)),                    \
	.tei_num = DT_PROP(DT_INST_PARENT(index), tei_number),                                     \
	.eri_ctrl = DEVICE_DT_GET(DT_PHANDLE(DT_INST_PARENT(index), eri_ctrl)),                    \
	.eri_num = DT_PROP(DT_INST_PARENT(index), eri_number),
#endif /* CONFIG_RENESAS_RX_GRP_INTC_FSP */
#else  /* CONFIG_UART_INTERRUPT_DRIVEN */
#define UART_RX_SCI_B_IRQ_CONFIG_INIT(index)
#define UART_RX_SCI_B_TXI_RXI_INIT(index)
#define UART_RX_SCI_B_TEI_ERI_INIT(index)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_RX_SCI_B_INIT(index)                                                                  \
	PINCTRL_DT_DEFINE(DT_INST_PARENT(index));                                                  \
	static const struct uart_renesas_rx_sci_b_config uart_renesas_rx_sci_b_config_##index = {  \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_PARENT(index)),                          \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(index))),                 \
		.clock_subsys =                                                                    \
			{                                                                          \
				.mstp = DT_CLOCKS_CELL(DT_INST_PARENT(index), mstp),               \
				.stop_bit = DT_CLOCKS_CELL(DT_INST_PARENT(index), stop_bit),       \
			},                                                                         \
		.regs = (R_SCI_B0_Type *)DT_REG_ADDR(DT_INST_PARENT(index)),                       \
		UART_RX_SCI_B_IRQ_CONFIG_INIT(index)};                                             \
	static struct uart_renesas_rx_sci_b_data uart_renesas_rx_sci_b_data_##index = {            \
		.dev = DEVICE_DT_GET(DT_DRV_INST(index)),                                          \
		.uart_config =                                                                     \
			{                                                                          \
				.baudrate = DT_INST_PROP(index, current_speed),                    \
				.parity = UART_CFG_PARITY_NONE,                                    \
				.stop_bits = UART_CFG_STOP_BITS_1,                                 \
				.data_bits = UART_CFG_DATA_BITS_8,                                 \
				.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,                              \
			},                                                                         \
		.fsp_config =                                                                      \
			{                                                                          \
				.channel = DT_PROP(DT_INST_PARENT(index), channel),                \
			},                                                                         \
		.fsp_config_extend = {},                                                           \
		.fsp_baud_setting = {},                                                            \
	};                                                                                         \
	static int uart_renesas_rx_sci_b_init_##index(const struct device *dev)                    \
	{                                                                                          \
		UART_RX_SCI_B_TXI_RXI_INIT(index);                                                 \
		UART_RX_SCI_B_TEI_ERI_INIT(index);                                                 \
		return uart_renesas_rx_sci_b_init(dev);                                            \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(index, uart_renesas_rx_sci_b_init_##index, NULL,                     \
			      &uart_renesas_rx_sci_b_data_##index,                                 \
			      &uart_renesas_rx_sci_b_config_##index, PRE_KERNEL_1,                 \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_renesas_rx_sci_b_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_RX_SCI_B_INIT)
