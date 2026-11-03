/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sent_taud_emulator.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "bsp_api.h"

LOG_MODULE_REGISTER(
	sent_taud_emulator,
	CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Alias points to the synchronized output child of the standard TAUD PWM
 * binding. The emulator owns TAUD0 directly, so the normal PWM driver must
 * not be enabled in this test image.
 */
#define SENT_TAUD_EMULATOR_NODE \
	DT_ALIAS(sent_taud_emulator)

#if !DT_NODE_EXISTS(SENT_TAUD_EMULATOR_NODE)
#error "sent-taud-emulator Devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(SENT_TAUD_EMULATOR_NODE)
#error "SENT TAUD emulator output channel is not enabled"
#endif

#define EMU_MASTER_NODE DT_PARENT(SENT_TAUD_EMULATOR_NODE)

#if !DT_NODE_HAS_STATUS_OKAY(EMU_MASTER_NODE)
#error "SENT TAUD emulator primary channel is not enabled"
#endif

BUILD_ASSERT(!IS_ENABLED(CONFIG_PWM_RENESAS_RH850),
	"SENT emulator owns TAUD0 directly; disable CONFIG_PWM_RENESAS_RH850");

#define EMU_TAUD_UNIT DT_PROP(EMU_MASTER_NODE, unit)
#define EMU_MASTER_CH DT_PROP(EMU_MASTER_NODE, channel)
#define EMU_SLAVE_CH  DT_REG_ADDR(SENT_TAUD_EMULATOR_NODE)

#define EMU_CLOCK_CTLR DT_CLOCKS_CTLR(EMU_MASTER_NODE)
#define EMU_CLOCK_ID   DT_CLOCKS_CELL(EMU_MASTER_NODE, clk_id)

static const struct device *const emulator_clock_dev =
	DEVICE_DT_GET(EMU_CLOCK_CTLR);

#define EMU_MASTER_IRQ          DT_IRQN(EMU_MASTER_NODE)
#define EMU_SLAVE_IRQ           DT_IRQN(SENT_TAUD_EMULATOR_NODE)
#define EMU_MASTER_IRQ_PRIORITY DT_IRQ(EMU_MASTER_NODE, priority)
#define EMU_SLAVE_IRQ_PRIORITY  DT_IRQ(SENT_TAUD_EMULATOR_NODE, priority)

BUILD_ASSERT(EMU_TAUD_UNIT == 0U,
	"Current SENT emulator supports TAUD0 only");
BUILD_ASSERT(EMU_MASTER_CH == 0U,
	"Current SENT emulator expects TAUD0 channel 0 as primary");
BUILD_ASSERT(EMU_SLAVE_CH == 1U,
	"Current SENT emulator expects TAUD0 channel 1 as output");

#define EMU_CH_MASK (BIT(EMU_MASTER_CH) | BIT(EMU_SLAVE_CH))

/* The new binding routes every child pin through the primary node. */
PINCTRL_DT_DEFINE(EMU_MASTER_NODE);
static const struct pinctrl_dev_config *const emulator_pinctrl =
	PINCTRL_DT_DEV_CONFIG_GET(EMU_MASTER_NODE);

/*
 * The current TAUD configuration produces a 10 MHz counter clock.
 *
 * Therefore:
 *
 *     1 us = 10 TAUD counts
 */
#define EMU_COUNTS_PER_US 10U

/*
 * Fixed low-level pulse width used by the TAUD slave output channel.
 *
 * This matches the original FSP test emulator:
 *
 *     slave CDR = 5 SENT ticks
 */
#define EMU_LOW_TICKS 5U

K_SEM_DEFINE(tx_done, 0, 1);

/*
 * Active raw pulse stream.
 *
 * The caller must keep the source array valid until
 * sent_taud_emulator_send_raw() returns.
 */
static const uint32_t *active_pulses;
static size_t active_pulse_count;
static volatile size_t active_pulse_index;
static bool emulator_opened;

/*
 * Debug information retained after a timeout.
 *
 * These variables can be inspected directly from the debugger.
 */
volatile size_t sent_taud_debug_pulse_index;
volatile size_t sent_taud_debug_pulse_count;
volatile uint32_t sent_taud_debug_current_ticks;

/*
 * Convert a raw SENT tick count into the value expected by TAUDnCDR.
 *
 * TAUD counts from the configured value down to zero, therefore the
 * programmed value is:
 *
 *     number_of_counts - 1
 */
static uint16_t sent_taud_ticks_to_counter(uint32_t ticks)
{
	uint64_t counter_value;

	__ASSERT_NO_MSG(ticks > 0U);

	counter_value =
		(uint64_t)ticks *
		(uint64_t)CONFIG_SENT_TAUD_TICK_US *
		(uint64_t)EMU_COUNTS_PER_US;

	__ASSERT_NO_MSG(
		counter_value <=
		((uint64_t)UINT16_MAX + 1U));

	return (uint16_t)(counter_value - 1U);
}

/*
 * The master interrupt is used internally as the slave-channel trigger.
 *
 * The interrupt status must still be acknowledged, but the next pulse is
 * loaded from the slave interrupt.
 */
static void sent_taud_master_isr(const void *arg)
{
	ARG_UNUSED(arg);

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_MASTER_IRQ);
}

/*
 * Load the next raw pulse period.
 *
 * The initial period is loaded by sent_taud_emulator_send_raw(). Each
 * subsequent slave interrupt advances to the next array entry.
 */
static void sent_taud_slave_isr(const void *arg)
{
	ARG_UNUSED(arg);

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_SLAVE_IRQ);

	sent_taud_debug_pulse_index =
		active_pulse_index;

	sent_taud_debug_pulse_count =
		active_pulse_count;

	/*
	 * Stop after the final pulse period has completed.
	 *
	 * Unlike the FSP test emulator, no SENT_EMU_END_NIBBLE_MARK is
	 * required. The supplied array length determines the end.
	 */
	if ((active_pulses == NULL) ||
	    (active_pulse_index >= active_pulse_count)) {
		R_TAUD0->TAUDnTT =
			EMU_CH_MASK;

		k_sem_give(&tx_done);

		return;
	}

	sent_taud_debug_current_ticks =
		active_pulses[active_pulse_index];

	R_TAUD0
		->TAUDnCDR[EMU_MASTER_CH]
		.TAUDnCDR =
		sent_taud_ticks_to_counter(
			active_pulses[active_pulse_index]);

	active_pulse_index++;
}

static int sent_taud_emulator_clock_init(void)
{
	int ret;

	if (!device_is_ready(emulator_clock_dev)) {
		LOG_ERR("TAUD clock device is not ready");
		return -ENODEV;
	}

	ret = clock_control_on(
		emulator_clock_dev,
		(clock_control_subsys_t)(uintptr_t)EMU_CLOCK_ID);
	if (ret < 0) {
		LOG_ERR("Failed to enable TAUD clock: %d", ret);
		return ret;
	}

	return 0;
}

int sent_taud_emulator_open(void)
{
	int ret;

	if (emulator_opened) {
		return -EALREADY;
	}

	ret = sent_taud_emulator_clock_init();
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(
		emulator_pinctrl,
		PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		LOG_ERR(
			"Failed to apply TAUD output pinctrl: %d",
			ret);

		return ret;
	}

	/*
	 * Ensure both channels are stopped before reconfiguration.
	 */
	R_TAUD0->TAUDnTT =
		EMU_CH_MASK;

	/*
	 * Configure the TAUD counter clock.
	 *
	 * Peripheral input clock:
	 *
	 *     40 MHz
	 *
	 * Effective TAUD counter clock:
	 *
	 *     10 MHz
	 */
	R_TAUD0->TAUDnTPS_b.TAUDnPRS3 =
		0U;

	R_TAUD0->TAUDnBRS =
		7U;

	R_TAUD0
		->TAUDnCMOR[EMU_MASTER_CH]
		.TAUDnCMOR_b.TAUDnCKS =
		3U;

	R_TAUD0
		->TAUDnCMOR[EMU_MASTER_CH]
		.TAUDnCMOR_b.TAUDnCCS =
		0U;

	R_TAUD0
		->TAUDnCMOR[EMU_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnCKS =
		3U;

	R_TAUD0
		->TAUDnCMOR[EMU_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnCCS =
		0U;

	/*
	 * Configure the master as an interval timer.
	 *
	 * Software starts the master channel. Each expiration supplies the
	 * trigger used by the slave output channel.
	 */
	R_TAUD0
		->TAUDnCMOR[EMU_MASTER_CH]
		.TAUDnCMOR_b.TAUDnMAS =
		1U;

	R_TAUD0
		->TAUDnCMOR[EMU_MASTER_CH]
		.TAUDnCMOR_b.TAUDnSTS =
		0U;

	R_TAUD0
		->TAUDnCMOR[EMU_MASTER_CH]
		.TAUDnCMOR_b.TAUDnMD =
		1U;

	/*
	 * Configure the slave as the SENT waveform output.
	 *
	 * The master-channel interrupt is used as the slave start trigger.
	 * Start triggers received while operating remain valid.
	 */
	R_TAUD0
		->TAUDnCMOR[EMU_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnMAS =
		0U;

	R_TAUD0
		->TAUDnCMOR[EMU_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnSTS =
		4U;

	R_TAUD0
		->TAUDnCMOR[EMU_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnMD =
		9U;

	BUILD_ASSERT(
		EMU_SLAVE_CH == 1U,
		"Current U2x configuration expects TAUD0 channel 1");

	/*
	 * Enable TAUD0 channel 1 output.
	 *
	 * Configuration matches the reference FSP SENT emulator.
	 */
	R_TAUD0->TAUDnTOM_b.TAUDnTOM01 = 1U;
	R_TAUD0->TAUDnTOC_b.TAUDnTOC01 = 0U;
	R_TAUD0->TAUDnTOL_b.TAUDnTOL01 = 1U;
	R_TAUD0->TAUDnTO_b.TAUDnTO01 = 1U;
	R_TAUD0->TAUDnTOE_b.TAUDnTOE01 = 1U;

	active_pulses = NULL;
	active_pulse_count = 0U;
	active_pulse_index = 0U;

	sent_taud_debug_pulse_index = 0U;
	sent_taud_debug_pulse_count = 0U;
	sent_taud_debug_current_ticks = 0U;

	k_sem_reset(&tx_done);

	IRQ_CONNECT(
		EMU_MASTER_IRQ,
		EMU_MASTER_IRQ_PRIORITY,
		sent_taud_master_isr,
		NULL,
		0);

	IRQ_CONNECT(
		EMU_SLAVE_IRQ,
		EMU_SLAVE_IRQ_PRIORITY,
		sent_taud_slave_isr,
		NULL,
		0);

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_MASTER_IRQ);

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_SLAVE_IRQ);

	irq_enable(EMU_MASTER_IRQ);
	irq_enable(EMU_SLAVE_IRQ);

	emulator_opened = true;

	LOG_INF(
		"TAUD SENT raw emulator opened: "
		"unit=%u master_ch=%u master_irq=%u, "
		"slave_ch=%u slave_irq=%u",
		EMU_TAUD_UNIT,
		EMU_MASTER_CH,
		EMU_MASTER_IRQ,
		EMU_SLAVE_CH,
		EMU_SLAVE_IRQ);

	return 0;
}

int sent_taud_emulator_send_raw(
	const uint32_t *pulse_ticks,
	size_t count)
{
	int ret;

	if (!emulator_opened) {
		return -EACCES;
	}

	if ((pulse_ticks == NULL) ||
	    (count == 0U)) {
		return -EINVAL;
	}

	/*
	 * Validate the entire array before starting TAUD.
	 *
	 * This prevents an invalid later entry from being discovered only
	 * after transmission has already started.
	 */
	for (size_t i = 0U; i < count; i++) {
		uint64_t counter_value;

		if (pulse_ticks[i] == 0U) {
			return -EINVAL;
		}

		counter_value =
			(uint64_t)pulse_ticks[i] *
			(uint64_t)CONFIG_SENT_TAUD_TICK_US *
			(uint64_t)EMU_COUNTS_PER_US;

		if (counter_value >
		    ((uint64_t)UINT16_MAX + 1U)) {
			return -ERANGE;
		}
	}

	/*
	 * Stop both channels before installing the new pulse stream.
	 */
	R_TAUD0->TAUDnTT =
		EMU_CH_MASK;

	active_pulses = pulse_ticks;
	active_pulse_count = count;

	/*
	 * Pulse zero is loaded here, so the ISR begins with pulse one.
	 */
	active_pulse_index = 1U;

	sent_taud_debug_pulse_index = 0U;
	sent_taud_debug_pulse_count = count;
	sent_taud_debug_current_ticks =
		pulse_ticks[0];

	k_sem_reset(&tx_done);

	/*
	 * Load the first falling-edge-to-falling-edge period.
	 */
	R_TAUD0
		->TAUDnCDR[EMU_MASTER_CH]
		.TAUDnCDR =
		sent_taud_ticks_to_counter(
			pulse_ticks[0]);

	/*
	 * Configure the fixed low-level pulse duration on the output
	 * channel, matching the original FSP emulator.
	 */
	R_TAUD0
		->TAUDnCDR[EMU_SLAVE_CH]
		.TAUDnCDR =
		sent_taud_ticks_to_counter(
			EMU_LOW_TICKS);

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_MASTER_IRQ);

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_SLAVE_IRQ);

	/*
	 * Simultaneously start master and slave.
	 */
	R_TAUD0->TAUDnTS =
		EMU_CH_MASK;

	ret = k_sem_take(
		&tx_done,
		K_MSEC(
			CONFIG_SENT_TAUD_TX_TIMEOUT_MS));

	if (ret != 0) {
		R_TAUD0->TAUDnTT =
			EMU_CH_MASK;

		/*
		 * Prevent either TAUD ISR from observing cleared transmission
		 * state while recovering from a timeout.
		 */
		irq_disable(EMU_MASTER_IRQ);
		irq_disable(EMU_SLAVE_IRQ);

		R_BSP_IrqStatusClear(
			(IRQn_Type)EMU_MASTER_IRQ);

		R_BSP_IrqStatusClear(
			(IRQn_Type)EMU_SLAVE_IRQ);

		active_pulses = NULL;
		active_pulse_count = 0U;
		active_pulse_index = 0U;

		irq_enable(EMU_MASTER_IRQ);
		irq_enable(EMU_SLAVE_IRQ);

		LOG_ERR(
			"TAUD raw transmission timeout: "
			"pulse_index=%u pulse_count=%u "
			"current_ticks=%u",
			(uint32_t)
				sent_taud_debug_pulse_index,
			(uint32_t)
				sent_taud_debug_pulse_count,
			sent_taud_debug_current_ticks);

		return -ETIMEDOUT;
	}

	active_pulses = NULL;
	active_pulse_count = 0U;
	active_pulse_index = 0U;

	return 0;
}

int sent_taud_emulator_close(void)
{
	if (!emulator_opened) {
		return 0;
	}

	irq_disable(EMU_MASTER_IRQ);
	irq_disable(EMU_SLAVE_IRQ);

	R_TAUD0->TAUDnTT =
		EMU_CH_MASK;

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_MASTER_IRQ);

	R_BSP_IrqStatusClear(
		(IRQn_Type)EMU_SLAVE_IRQ);

	R_TAUD0->TAUDnTOE_b.TAUDnTOE01 = 0U;
	R_TAUD0->TAUDnTO_b.TAUDnTO01 = 0U;
	R_TAUD0->TAUDnTOL_b.TAUDnTOL01 = 0U;
	R_TAUD0->TAUDnTOC_b.TAUDnTOC01 = 0U;
	R_TAUD0->TAUDnTOM_b.TAUDnTOM01 = 0U;

	active_pulses = NULL;
	active_pulse_count = 0U;
	active_pulse_index = 0U;

	emulator_opened = false;

	k_sem_reset(&tx_done);

	LOG_INF(
		"TAUD SENT raw emulator closed");

	return 0;
}
