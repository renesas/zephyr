/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT renesas_rh850_emu_pwm

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
 * The sent-taud-emulator alias points directly to the TAUD slave
 * channel used as the SENT output.
 *
 * Example:
 *
 * / {
 *     aliases {
 *         sent-taud-emulator = &pwm0_slave;
 *     };
 * };
 */
#define SENT_TAUD_EMULATOR_NODE \
	DT_ALIAS(sent_taud_emulator)

#if !DT_NODE_EXISTS(SENT_TAUD_EMULATOR_NODE)
#error "sent-taud-emulator Devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(SENT_TAUD_EMULATOR_NODE)
#error "SENT TAUD emulator Devicetree node is not enabled"
#endif

/*
 * Optional second slave output.
 *
 * When sent-taud-emulator-secondary is present and enabled, both slaves
 * follow the same master period stream. Only the primary slave interrupt
 * advances active_pulse_index; the secondary slave is output-only.
 */
#define SENT_TAUD_EMULATOR_SECONDARY_NODE \
	DT_ALIAS(sent_taud_emulator_secondary)

#if DT_NODE_EXISTS(SENT_TAUD_EMULATOR_SECONDARY_NODE) && \
	DT_NODE_HAS_STATUS_OKAY(SENT_TAUD_EMULATOR_SECONDARY_NODE)
#define EMU_HAS_SECONDARY_SLAVE 1
#else
#define EMU_HAS_SECONDARY_SLAVE 0
#endif

/*
 * The slave node contains:
 *
 *     master = <&pwm0_master>;
 *
 * Retrieve the associated master channel from the phandle.
 */
#define EMU_MASTER_NODE \
	DT_PHANDLE(SENT_TAUD_EMULATOR_NODE, master)

/*
 * TAUD instance and channel configuration.
 */
#define EMU_TAUD_UNIT \
	DT_PROP(SENT_TAUD_EMULATOR_NODE, unit)

#define EMU_MASTER_CH \
	DT_PROP(EMU_MASTER_NODE, channel)

#define EMU_SLAVE_CH \
	DT_PROP(SENT_TAUD_EMULATOR_NODE, channel)

#if EMU_HAS_SECONDARY_SLAVE
#define EMU_SECONDARY_MASTER_NODE \
	DT_PHANDLE(SENT_TAUD_EMULATOR_SECONDARY_NODE, master)
#define EMU_SECONDARY_SLAVE_CH \
	DT_PROP(SENT_TAUD_EMULATOR_SECONDARY_NODE, channel)
#endif

/*
 * TAUD clock configuration.
 */
#define EMU_MASTER_CLOCK_CTLR \
	DT_CLOCKS_CTLR(EMU_MASTER_NODE)

#define EMU_MASTER_CLOCK_ID \
	DT_CLOCKS_CELL(EMU_MASTER_NODE, clk_id)

#define EMU_SLAVE_CLOCK_CTLR \
	DT_CLOCKS_CTLR(SENT_TAUD_EMULATOR_NODE)

#define EMU_SLAVE_CLOCK_ID \
	DT_CLOCKS_CELL(SENT_TAUD_EMULATOR_NODE, clk_id)

static const struct device *const emulator_master_clock_dev =
	DEVICE_DT_GET(EMU_MASTER_CLOCK_CTLR);

static const struct device *const emulator_slave_clock_dev =
	DEVICE_DT_GET(EMU_SLAVE_CLOCK_CTLR);

#if EMU_HAS_SECONDARY_SLAVE
#define EMU_SECONDARY_SLAVE_CLOCK_CTLR \
	DT_CLOCKS_CTLR(SENT_TAUD_EMULATOR_SECONDARY_NODE)
#define EMU_SECONDARY_SLAVE_CLOCK_ID \
	DT_CLOCKS_CELL(SENT_TAUD_EMULATOR_SECONDARY_NODE, clk_id)

static const struct device *const emulator_secondary_slave_clock_dev =
	DEVICE_DT_GET(EMU_SECONDARY_SLAVE_CLOCK_CTLR);
#endif

/*
 * TAUD interrupt configuration.
 */
#define EMU_MASTER_IRQ \
	DT_IRQ(EMU_MASTER_NODE, irq)

#define EMU_SLAVE_IRQ \
	DT_IRQ(SENT_TAUD_EMULATOR_NODE, irq)

#define EMU_MASTER_IRQ_PRIORITY \
	DT_IRQ(EMU_MASTER_NODE, priority)

#define EMU_SLAVE_IRQ_PRIORITY \
	DT_IRQ(SENT_TAUD_EMULATOR_NODE, priority)

BUILD_ASSERT(
	EMU_TAUD_UNIT == DT_PROP(EMU_MASTER_NODE, unit),
	"TAUD master and slave must use the same TAUD unit");

BUILD_ASSERT(
	EMU_TAUD_UNIT == 0U,
	"Current SENT emulator supports TAUD0 only");

#if EMU_HAS_SECONDARY_SLAVE
BUILD_ASSERT(
	DT_SAME_NODE(EMU_MASTER_NODE, EMU_SECONDARY_MASTER_NODE),
	"Primary and secondary slaves must use the same TAUD master");
BUILD_ASSERT(
	EMU_TAUD_UNIT == DT_PROP(SENT_TAUD_EMULATOR_SECONDARY_NODE, unit),
	"Primary and secondary slaves must use the same TAUD unit");
BUILD_ASSERT(
	EMU_SLAVE_CH != EMU_SECONDARY_SLAVE_CH,
	"Primary and secondary TAUD slave channels must differ");
#define EMU_SECONDARY_CH_MASK BIT(EMU_SECONDARY_SLAVE_CH)
#else
#define EMU_SECONDARY_CH_MASK 0U
#endif

#define EMU_CH_MASK \
	(BIT(EMU_MASTER_CH) | BIT(EMU_SLAVE_CH) | EMU_SECONDARY_CH_MASK)

/*
 * Apply the output pin configuration attached to the slave TAUD node.
 */
PINCTRL_DT_DEFINE(SENT_TAUD_EMULATOR_NODE);

static const struct pinctrl_dev_config *const emulator_pinctrl =
	PINCTRL_DT_DEV_CONFIG_GET(
		SENT_TAUD_EMULATOR_NODE);

#if EMU_HAS_SECONDARY_SLAVE
PINCTRL_DT_DEFINE(SENT_TAUD_EMULATOR_SECONDARY_NODE);

static const struct pinctrl_dev_config *const emulator_secondary_pinctrl =
	PINCTRL_DT_DEV_CONFIG_GET(
		SENT_TAUD_EMULATOR_SECONDARY_NODE);
#endif

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

	if (!device_is_ready(emulator_master_clock_dev)) {
		LOG_ERR("TAUD master clock device is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(emulator_slave_clock_dev)) {
		LOG_ERR("TAUD slave clock device is not ready");
		return -ENODEV;
	}

#if EMU_HAS_SECONDARY_SLAVE
	if (!device_is_ready(emulator_secondary_slave_clock_dev)) {
		LOG_ERR("TAUD secondary slave clock device is not ready");
		return -ENODEV;
	}
#endif

	ret = clock_control_on(
		emulator_master_clock_dev,
		(clock_control_subsys_t)(uintptr_t)EMU_MASTER_CLOCK_ID);
	if (ret < 0) {
		LOG_ERR("Failed to enable TAUD master clock: %d", ret);
		return ret;
	}

	ret = clock_control_on(
		emulator_slave_clock_dev,
		(clock_control_subsys_t)(uintptr_t)EMU_SLAVE_CLOCK_ID);
	if (ret < 0) {
		LOG_ERR("Failed to enable TAUD slave clock: %d", ret);
		return ret;
	}

#if EMU_HAS_SECONDARY_SLAVE
	ret = clock_control_on(
		emulator_secondary_slave_clock_dev,
		(clock_control_subsys_t)(uintptr_t)EMU_SECONDARY_SLAVE_CLOCK_ID);
	if (ret < 0) {
		LOG_ERR("Failed to enable TAUD secondary slave clock: %d", ret);
		return ret;
	}
#endif

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

#if EMU_HAS_SECONDARY_SLAVE
	ret = pinctrl_apply_state(
		emulator_secondary_pinctrl,
		PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR(
			"Failed to apply secondary TAUD output pinctrl: %d",
			ret);
		return ret;
	}
#endif

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

#if EMU_HAS_SECONDARY_SLAVE
	R_TAUD0
		->TAUDnCMOR[EMU_SECONDARY_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnCKS =
		3U;

	R_TAUD0
		->TAUDnCMOR[EMU_SECONDARY_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnCCS =
		0U;
#endif

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

#if EMU_HAS_SECONDARY_SLAVE
	R_TAUD0
		->TAUDnCMOR[EMU_SECONDARY_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnMAS =
		0U;

	R_TAUD0
		->TAUDnCMOR[EMU_SECONDARY_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnSTS =
		4U;

	R_TAUD0
		->TAUDnCMOR[EMU_SECONDARY_SLAVE_CH]
		.TAUDnCMOR_b.TAUDnMD =
		9U;
#endif

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

#if EMU_HAS_SECONDARY_SLAVE
	BUILD_ASSERT(
		EMU_SECONDARY_SLAVE_CH == 2U,
		"Current U2x secondary output expects TAUD0 channel 2");

	R_TAUD0->TAUDnTOM_b.TAUDnTOM02 = 1U;
	R_TAUD0->TAUDnTOC_b.TAUDnTOC02 = 0U;
	R_TAUD0->TAUDnTOL_b.TAUDnTOL02 = 1U;
	R_TAUD0->TAUDnTO_b.TAUDnTO02 = 1U;
	R_TAUD0->TAUDnTOE_b.TAUDnTOE02 = 1U;
#endif

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
		"slave_ch=%u slave_irq=%u secondary=%u",
		EMU_TAUD_UNIT,
		EMU_MASTER_CH,
		EMU_MASTER_IRQ,
		EMU_SLAVE_CH,
		EMU_SLAVE_IRQ,
		EMU_HAS_SECONDARY_SLAVE);

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


#if EMU_HAS_SECONDARY_SLAVE
	R_TAUD0
		->TAUDnCDR[EMU_SECONDARY_SLAVE_CH]
		.TAUDnCDR =
		sent_taud_ticks_to_counter(
			EMU_LOW_TICKS);
#endif

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

#if EMU_HAS_SECONDARY_SLAVE
	R_TAUD0->TAUDnTOE_b.TAUDnTOE02 = 0U;
	R_TAUD0->TAUDnTO_b.TAUDnTO02 = 0U;
	R_TAUD0->TAUDnTOL_b.TAUDnTOL02 = 0U;
	R_TAUD0->TAUDnTOC_b.TAUDnTOC02 = 0U;
	R_TAUD0->TAUDnTOM_b.TAUDnTOM02 = 0U;
#endif

	active_pulses = NULL;
	active_pulse_count = 0U;
	active_pulse_index = 0U;

	emulator_opened = false;

	k_sem_reset(&tx_done);

	LOG_INF(
		"TAUD SENT raw emulator closed");

	return 0;
}
