/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* PSI5 sensor emulator for hardware tests. */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <soc.h>
#include "bsp_api.h"

LOG_MODULE_REGISTER(psi5_emu, LOG_LEVEL_INF);

#define PSI5_EMU_HALF_BIT_US       4U
#define PSI5_EMU_START_BITS        2U
#define PSI5_EMU_MAX_DATA_BITS     28U
#define PSI5_EMU_BUF_SIZE          (((PSI5_EMU_START_BITS + PSI5_EMU_MAX_DATA_BITS + 1U) * 2U) + 1U)
#define PSI5_EMU_SEND_TIMEOUT_MS   50U
#define PSI5_EMU_END               0xFFU
#define PSI5_EMU_SMC_FRAMES        18U
#define PSI5_EMU_SMC_CRC_FRAMES    6U
#define PSI5_EMU_SMC_DATA_LEN      10U
#define PSI5_EMU_SMC_IDLE_US       8U
#define PSI5_EMU_SYNC_DELAY_US  20U
#define PSI5_EMU_SYNC_TIMEOUT_US  2000U
#define PSI5_EMU_REG            ((R_PSI50_Type *)DT_REG_ADDR(DT_ALIAS(psi5_0)))

#define PSI5_EMU_TAUD_NODE DT_NODELABEL(pwm0_ch0)
BUILD_ASSERT(DT_NODE_EXISTS(PSI5_EMU_TAUD_NODE), "pwm0_ch0 node not found");
BUILD_ASSERT(DT_PROP(PSI5_EMU_TAUD_NODE, unit) == 0, "Emulator uses TAUD0 only");
BUILD_ASSERT(DT_PROP(PSI5_EMU_TAUD_NODE, channel) == 0,
	     "PSI5 emulator ISR requires TAUD0 channel 0");
BUILD_ASSERT(!IS_ENABLED(CONFIG_PWM_RENESAS_RH850),
	     "TAUD0 ch0 is owned by the PWM driver; disable CONFIG_PWM for psi5_rx");

#define PSI5_EMU_TAUD_CH       DT_PROP(PSI5_EMU_TAUD_NODE, channel)
#define PSI5_EMU_TAUD_IRQ      DT_IRQN(PSI5_EMU_TAUD_NODE)
#define PSI5_EMU_IRQ_PRIO      0
#define PSI5_EMU_TAUD_CLK_ID   DT_CLOCKS_CELL(PSI5_EMU_TAUD_NODE, clk_id)
#define PSI5_EMU_TAUD_CH_MASK  ((uint16_t)(1U << PSI5_EMU_TAUD_CH))
#define PSI5_EMU_TAUD_CMOR     0x0001U

#ifdef BSP_CFG_CLOCK_CLK_HSB_HZ
#define PSI5_EMU_CLK_HZ BSP_CFG_CLOCK_CLK_HSB_HZ
#else
#define PSI5_EMU_CLK_HZ 80000000U
#endif

#define PSI5_EMU_PERIOD_COUNTS \
	(PSI5_EMU_HALF_BIT_US * (PSI5_EMU_CLK_HZ / 1000000U))

BUILD_ASSERT(PSI5_EMU_PERIOD_COUNTS > 0U, "Invalid TAUD period");
BUILD_ASSERT(PSI5_EMU_PERIOD_COUNTS <= UINT16_MAX, "TAUD period does not fit CDR");

static const struct device *const taud_clk_dev =
	DEVICE_DT_GET(DT_CLOCKS_CTLR(PSI5_EMU_TAUD_NODE));
static const struct gpio_dt_spec emu_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), psi5_emu_gpios);

static const uint16_t psi5_emu_smc_data[PSI5_EMU_SMC_FRAMES] = {
	0x3F2, 0x3F2, 0x3F2, 0x3F2, 0x3F2, 0x3F2,
	0x0F0, 0x0F0, 0x3F1, 0x3F3, 0x3F1, 0x3F3,
	0x0F1, 0x3F0, 0x3F2, 0x3F0, 0x3F2, 0x0F1,
};

static uint8_t psi5_emu_buf[PSI5_EMU_BUF_SIZE];
static atomic_t psi5_emu_busy;
static const uint8_t *volatile psi5_emu_pattern;
static volatile uint32_t psi5_emu_idx;

static inline void psi5_emu_timer_stop(void)
{
	R_TAUD0->TAUDnTT = PSI5_EMU_TAUD_CH_MASK;
}

static void psi5_emu_isr(const void *arg)
{
	const uint8_t *pattern = psi5_emu_pattern;
	uint8_t level;

	ARG_UNUSED(arg);
	R_BSP_IrqStatusClear((IRQn_Type)PSI5_EMU_TAUD_IRQ);

	if (pattern == NULL) {
		return;
	}

	level = pattern[psi5_emu_idx];
	if (level == PSI5_EMU_END) {
		psi5_emu_timer_stop();
		(void)gpio_pin_set_dt(&emu_gpio, 0);
		psi5_emu_pattern = NULL;
		return;
	}

	(void)gpio_pin_set_dt(&emu_gpio, level);
	psi5_emu_idx++;
}

int psi5_emu_init(void)
{
	static bool initialized;
	int ret;

	if (initialized) {
		return 0;
	}

	if (!gpio_is_ready_dt(&emu_gpio)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&emu_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	if (!device_is_ready(taud_clk_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(taud_clk_dev,
		(clock_control_subsys_t)(uintptr_t)PSI5_EMU_TAUD_CLK_ID);
	if (ret < 0) {
		return ret;
	}

	psi5_emu_timer_stop();
	R_TAUD0->TAUDnTPS_b.TAUDnPRS0 = 0U;
	R_TAUD0->TAUDnCMOR[PSI5_EMU_TAUD_CH].TAUDnCMOR = PSI5_EMU_TAUD_CMOR;
	R_TAUD0->TAUDnCDR[PSI5_EMU_TAUD_CH].TAUDnCDR =
		(uint16_t)(PSI5_EMU_PERIOD_COUNTS - 1U);
	R_TAUD0->TAUDnTOE &= (uint16_t)~PSI5_EMU_TAUD_CH_MASK;

	IRQ_CONNECT(PSI5_EMU_TAUD_IRQ, PSI5_EMU_IRQ_PRIO, psi5_emu_isr, NULL, 0);
	R_BSP_IrqStatusClear((IRQn_Type)PSI5_EMU_TAUD_IRQ);
	irq_enable(PSI5_EMU_TAUD_IRQ);

	atomic_clear(&psi5_emu_busy);
	initialized = true;

	LOG_INF("PSI5 emulator ready: TAUD0 ch%d, half-bit %u us",
		PSI5_EMU_TAUD_CH, PSI5_EMU_HALF_BIT_US);
	return 0;
}

int psi5_emu_send_pattern(const uint8_t *pattern)
{
	uint32_t start_cycles;
	uint32_t timeout_cycles;
	int ret = 0;

	if (pattern == NULL) {
		return -EINVAL;
	}

	if (!atomic_cas(&psi5_emu_busy, 0, 1)) {
		return -EBUSY;
	}

	psi5_emu_idx = 0U;
	psi5_emu_pattern = pattern;
	(void)gpio_pin_set_dt(&emu_gpio, 0);

	R_BSP_IrqStatusClear((IRQn_Type)PSI5_EMU_TAUD_IRQ);
	R_TAUD0->TAUDnTS = PSI5_EMU_TAUD_CH_MASK;

	start_cycles = k_cycle_get_32();
	timeout_cycles = k_ms_to_cyc_ceil32(PSI5_EMU_SEND_TIMEOUT_MS);
	while (psi5_emu_pattern != NULL) {
		if ((k_cycle_get_32() - start_cycles) > timeout_cycles) {
			psi5_emu_timer_stop();
			psi5_emu_pattern = NULL;
			(void)gpio_pin_set_dt(&emu_gpio, 0);
			ret = -ETIMEDOUT;
			break;
		}
	}

	atomic_clear(&psi5_emu_busy);
	return ret;
}

int psi5_emu_send_frame(uint32_t data, uint8_t data_len, bool has_parity)
{
	uint32_t idx = 0U;
	uint32_t ones = 0U;

	if ((data_len == 0U) || (data_len > PSI5_EMU_MAX_DATA_BITS)) {
		return -EINVAL;
	}

	for (uint32_t i = 0U; i < PSI5_EMU_START_BITS; i++) {
		psi5_emu_buf[idx++] = 0U;
		psi5_emu_buf[idx++] = 1U;
	}

	for (uint8_t i = 0U; i < data_len; i++) {
		uint8_t bit = (uint8_t)((data >> i) & 1U);

		ones += bit;
		psi5_emu_buf[idx++] = bit;
		psi5_emu_buf[idx++] = (uint8_t)!bit;
	}

	if (has_parity) {
		uint8_t parity = (uint8_t)(ones & 1U);

		psi5_emu_buf[idx++] = parity;
		psi5_emu_buf[idx++] = (uint8_t)!parity;
	}

	psi5_emu_buf[idx] = PSI5_EMU_END;
	return psi5_emu_send_pattern(psi5_emu_buf);
}

int psi5_emu_send_smc_pattern(uint8_t crc6)
{
	for (uint32_t frame = 0U; frame < PSI5_EMU_SMC_FRAMES; frame++) {
		uint32_t data = psi5_emu_smc_data[frame];
		int ret;

		if (frame < PSI5_EMU_SMC_CRC_FRAMES) {
			data |= ((uint32_t)crc6 >>
				 (PSI5_EMU_SMC_CRC_FRAMES - 1U - frame)) & 1U;
		}

		ret = psi5_emu_send_frame(data, PSI5_EMU_SMC_DATA_LEN, true);
		if (ret != 0) {
			return ret;
		}
		k_busy_wait(PSI5_EMU_SMC_IDLE_US);
	}

	return 0;
}

int psi5_emu_send_frame_synced(uint32_t data, uint8_t data_len, bool has_parity)
{
	uint32_t guard = PSI5_EMU_SYNC_TIMEOUT_US;

	PSI5_EMU_REG->PSI5nTXSTCLR = R_PSI50_PSI5nTXSTCLR_SYNCEDCLR_Msk;
	while ((PSI5_EMU_REG->PSI5nTXST & R_PSI50_PSI5nTXST_SYNCED_Msk) == 0U) {
		if (--guard == 0U) {
			return -ETIMEDOUT;
		}
		k_busy_wait(1);
	}
	k_busy_wait(PSI5_EMU_SYNC_DELAY_US);

	return psi5_emu_send_frame(data, data_len, has_parity);
}
