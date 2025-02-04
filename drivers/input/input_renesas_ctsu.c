/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rx_ctsu

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/pinctrl.h>
#include <r_ctsu_qe_pinset.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <r_ctsu_qe_if.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

LOG_MODULE_REGISTER(renesas_rx_ctsu, CONFIG_INPUT_LOG_LEVEL);

#define BUTTON_TYPE 0
#define SLIDER_TYPE 1
#define WHEEL_TYPE  2

typedef enum {
	UNTOUCH = 0,
	TOUCHING = 1,
} touch_state_t;

typedef enum {
	INITIALIZING = 0,
	TUNING = 1,
	SCANNING = 2,
} working_phase_t;

enum touch_event {
	RELEASE = 0, /* state change from TOUCHING to UNTOUCH */
	PRESS = 1,   /* state change from UNTOUCH to TOUCHING */
};

/** Configuration of each component */
typedef struct st_touch_component_config {
	const uint8_t *p_channel_num; /** channel number array used by this component. */
	uint8_t num_elements;         /** Number of elements used by this component. */
	uint16_t threshold;           /** Position calculation start threshold value. */
	uint16_t resolution;          /** component resolution (slider - wheel) */
	uint16_t hysteresis;  /** Threshold hysteresis for chattering prevention. (button) */
	uint16_t zephyr_code; /** Zephyr input event code. */
} touch_component_cfg_t;

typedef struct st_touch_channel_config {
	uint8_t channel_num;
	ctsu_element_cfg_t config;
} touch_channel_cfg_t;

struct renesas_rx_ctsu_config {
	const struct pinctrl_dev_config *pcfg;
	int debounce_interval_ms;
	touch_channel_cfg_t *channel_cfgs;
	touch_component_cfg_t *button_cfgs;
	touch_component_cfg_t *slider_cfgs;
	touch_component_cfg_t *wheel_cfgs;
	uint8_t num_buttons;
	uint8_t num_sliders;
	uint8_t num_wheels;
	uint8_t *channels_index_map;
};

struct renesas_rx_ctsu_data {
	const struct device *dev;
	struct k_work_delayable work;
	ctsu_instance_ctrl_t ctsu_ctrl;
	ctsu_cfg_t ctsu_cfg;
	touch_state_t *buttons_curr_states;
	touch_state_t *buttons_prev_states;
	uint16_t *sliders_curr_position;
	uint16_t *sliders_prev_position;
	uint16_t *wheels_curr_position;
	uint16_t *wheels_prev_position;
	uint16_t *channels_count_result;
	uint16_t *channels_reference;
	struct k_sem tune_scan_end;
	working_phase_t work_phase;
};

void ctsu_ctsuend_isr(void);
void ctsu_ctsuwr_isr(void);
void ctsu_ctsurd_isr(void);

void ctsuwr_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	ctsu_ctsuwr_isr();
}

void ctsurd_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	ctsu_ctsurd_isr();
}

void ctsufn_isr(const struct device *dev)
{
	struct renesas_rx_ctsu_data *data = dev->data;
	const struct renesas_rx_ctsu_config *config = dev->config;

	ctsu_ctsuend_isr();

	if (data->work_phase == TUNING) {
		k_sem_give(&data->tune_scan_end);
	}

	if (data->work_phase == SCANNING) {
#ifdef CONFIG_CTSU_SHOW_COUNT
		ARG_UNUSED(config);
		k_work_reschedule(&data->work, K_MSEC(100));
#else
		(void)k_work_reschedule(&data->work, K_MSEC(config->debounce_interval_ms));
#endif
	}
}

#ifndef CONFIG_CTSU_SHOW_COUNT
static void get_button_state(touch_component_cfg_t button_cfg, uint16_t *channels_count,
			     uint16_t *channels_ref, uint8_t channel_index,
			     touch_state_t *button_state)
{
	if (channels_ref[channel_index] == 0) {
		/* In case of first time measure, not update state, just update reference*/
		channels_ref[channel_index] = channels_count[channel_index];
		return;
	}

	int higher_boundary = button_cfg.threshold + channels_ref[channel_index];
	int lower_boundary =
		(button_cfg.threshold + channels_ref[channel_index] - button_cfg.hysteresis);

	if (channels_count[channel_index] > higher_boundary) {
		*button_state = TOUCHING;
	} else if (channels_count[channel_index] < lower_boundary) {
		*button_state = UNTOUCH;
	} else {
		/* State unchange */
	}
}

static inline int get_component_count(touch_component_cfg_t component_cfg, uint16_t *channels_count,
				      uint8_t *channel_pos, uint8_t index)
{
	uint8_t channel_index = channel_pos[component_cfg.p_channel_num[index]];
	return channels_count[channel_index];
}

static int calculate_component_position(touch_component_cfg_t component_cfg,
					uint16_t *channels_count, uint8_t *channel_pos,
					uint16_t *position, uint8_t component_type)
{
	uint8_t max_index = 0;
	uint8_t lower_index;
	uint8_t higher_index;
	uint16_t max_data;
	uint16_t lower_index_data;
	uint16_t higher_index_data;
	uint8_t num_elements = component_cfg.num_elements;
	uint16_t resolution = component_cfg.resolution;
	uint16_t resolution_cap = resolution * num_elements;
	uint16_t d1;
	uint16_t d2;
	uint16_t d3;
	uint16_t dsum;

	/** Find the index of the highest value */
	for (uint8_t i = 1; i < num_elements; i++) {
		if (get_component_count(component_cfg, channels_count, channel_pos, i) >
		    get_component_count(component_cfg, channels_count, channel_pos, max_index)) {
			max_index = i;
		}
	}

	switch (component_type) {
	case SLIDER_TYPE:
		if (max_index == 0) {
			lower_index = 2;
			higher_index = 1;
		} else if (max_index == num_elements - 1) {
			lower_index = num_elements - 2;
			higher_index = num_elements - 3;
		} else {
			lower_index = max_index - 1;
			higher_index = max_index + 1;
		}
		break;
	case WHEEL_TYPE:
		if (max_index == 0) {
			lower_index = num_elements - 1;
			higher_index = 1;
		} else if (max_index == num_elements - 1) {
			lower_index = num_elements - 2;
			higher_index = 0;
		} else {
			lower_index = max_index - 1;
			higher_index = max_index + 1;
		}
		break;
	default:
		*position = 0;
		LOG_ERR("Unsupported component type");
		return -EINVAL;
	}

	max_data = get_component_count(component_cfg, channels_count, channel_pos, max_index);
	lower_index_data =
		get_component_count(component_cfg, channels_count, channel_pos, lower_index);
	higher_index_data =
		get_component_count(component_cfg, channels_count, channel_pos, higher_index);

	d1 = max_data - lower_index_data;
	d2 = max_data - higher_index_data;
	dsum = d1 + d2;

	if (dsum < component_cfg.threshold) {
		*position = 0;
		return 0;
	}

	if (d1 == 0) {
		d1 = 1;
	}

	d3 = (d1 * resolution) / dsum + (max_index * resolution);

	switch (component_type) {
	case SLIDER_TYPE:
		if (d3 == 0) {
			*position = 1;
		} else if (resolution_cap - d3 <= (resolution / 2 + 1)) {
			*position = resolution;
		} else if (d3 <= resolution / 2 + 1) {
			*position = 1;
		} else {
			*position = d3 / num_elements;
		}
		break;
	case WHEEL_TYPE:
		*position = d3 / num_elements;
		if (*position == 0) {
			*position = resolution;
		} else if (*position > resolution + 1) {
			*position = 1;
		} else {
			/** Do nothing */
		}
		break;
	}

	return 0;
}
#endif

static void work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct renesas_rx_ctsu_data *data = CONTAINER_OF(dwork, struct renesas_rx_ctsu_data, work);
	const struct device *dev = data->dev;
	const struct renesas_rx_ctsu_config *config = dev->config;
	uint16_t *ctsu_value = data->channels_count_result;
	fsp_err_t ret = FSP_SUCCESS;

	ret = R_CTSU_DataGet(&data->ctsu_ctrl, ctsu_value);
	if (ret != FSP_SUCCESS) {
		LOG_ERR("CTSU: Failed to get data %d", ret);
		return;
	}

#ifdef CONFIG_CTSU_SHOW_COUNT
	touch_channel_cfg_t *channel_cfgs = config->channel_cfgs;

	printk("********** CTSU count **********\n");
	for (int i = 0; i < data->ctsu_ctrl.num_elements; i++) {
		printk("count value of TS%02d: %d\n", channel_cfgs[i].channel_num, ctsu_value[i]);
	}
#else
	uint8_t *channels_pos = config->channels_index_map;
	/** Buttons */
	for (int i = 0; i < config->num_buttons; i++) {
		touch_state_t *curr_state = data->buttons_curr_states;
		touch_state_t *prev_state = data->buttons_prev_states;
		touch_component_cfg_t *button_cfgs = config->button_cfgs;
		int channel_data_index = channels_pos[button_cfgs[i].p_channel_num[0]];

		get_button_state(button_cfgs[i], ctsu_value, data->channels_reference,
				 channel_data_index, &curr_state[i]);
		if (prev_state[i] == UNTOUCH && curr_state[i] == TOUCHING) {
			input_report_key(dev, button_cfgs[i].zephyr_code, PRESS, true, K_FOREVER);
		} else if (prev_state[i] == TOUCHING && curr_state[i] == UNTOUCH) {
			input_report_key(dev, button_cfgs[i].zephyr_code, RELEASE, true, K_FOREVER);
		} else {
			/** Do nothing */
		}
		prev_state[i] = curr_state[i];
	}

	/** Sliders */
	for (int i = 0; i < config->num_sliders; i++) {
		uint16_t *curr_pos = data->sliders_curr_position;
		uint16_t *prev_pos = data->sliders_prev_position;
		touch_component_cfg_t *slider_cfgs = config->slider_cfgs;

		calculate_component_position(slider_cfgs[i], ctsu_value, channels_pos, &curr_pos[i],
					     SLIDER_TYPE);

		if (curr_pos[i] == 0 && prev_pos[i] == 0) {
			/** If slider stay untouch -> no need to report input */
			continue;
		} else {
			input_report_abs(dev, slider_cfgs[i].zephyr_code, curr_pos[i], true,
					 K_FOREVER);
			prev_pos[i] = curr_pos[i];
		}
	}

	/** Wheels */
	for (int i = 0; i < config->num_wheels; i++) {
		uint16_t *curr_pos = data->wheels_curr_position;
		uint16_t *prev_pos = data->wheels_prev_position;
		touch_component_cfg_t *wheel_cfgs = config->wheel_cfgs;

		calculate_component_position(wheel_cfgs[i], ctsu_value, channels_pos, &curr_pos[i],
					     WHEEL_TYPE);

		if (curr_pos[i] == 0 && prev_pos[i] == 0) {
			/** If wheel stay untouch -> no need to report input */
			continue;
		} else {
			input_report_abs(dev, wheel_cfgs[i].zephyr_code, curr_pos[i], true,
					 K_FOREVER);
			prev_pos[i] = curr_pos[i];
		}
	}
#endif
	/** Start next scan */
	ret = R_CTSU_ScanStart(&data->ctsu_ctrl);
	if (ret != FSP_SUCCESS) {
		LOG_ERR("CTSU: Failed to restart scan");
		return;
	}
}

static inline int set_scan_channel(const struct device *dev)
{
	struct renesas_rx_ctsu_data *data = dev->data;
	const struct renesas_rx_ctsu_config *config = dev->config;
	touch_channel_cfg_t *channel_cfgs = config->channel_cfgs;

	for (int i = 0; i < data->ctsu_cfg.num_rx; i++) {
		int cha_reg = channel_cfgs[i].channel_num / 8;
		int cha_pos = channel_cfgs[i].channel_num % 8;

		switch (cha_reg) {
		case 0:
			data->ctsu_cfg.ctsuchac0 |= (1 << cha_pos);
			break;
		case 1:
			data->ctsu_cfg.ctsuchac1 |= (1 << cha_pos);
			break;
		case 2:
			data->ctsu_cfg.ctsuchac2 |= (1 << cha_pos);
			break;
		case 3:
			data->ctsu_cfg.ctsuchac3 |= (1 << cha_pos);
			break;
		case 4:
			data->ctsu_cfg.ctsuchac4 |= (1 << cha_pos);
			break;
		default:
			LOG_ERR("Invalid TS channel");
			return -EINVAL;
		}
	}

	return 0;
}

static int renesas_rx_ctsu_init(const struct device *dev)
{
	struct renesas_rx_ctsu_data *data = dev->data;
	const struct renesas_rx_ctsu_config *config = dev->config;
	fsp_err_t ret = FSP_SUCCESS;
	int err = 0;

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("CTSU: Failed to set pinctrl");
		return err;
	}

	k_sem_init(&data->tune_scan_end, 0, 1);

	err = set_scan_channel(dev);
	if (err < 0) {
		LOG_ERR("CTSU: Failed to set scan channel");
		return err;
	}

	/** Set initial states */
	for (int i = 0; i < config->num_buttons; i++) {
		data->buttons_curr_states[i] = UNTOUCH;
	}
	for (int i = 0; i < config->num_sliders; i++) {
		data->sliders_curr_position[i] = 0;
	}
	for (int i = 0; i < config->num_wheels; i++) {
		data->wheels_curr_position[i] = 0;
	}

	data->work_phase = INITIALIZING;
	ret = R_CTSU_Open(&data->ctsu_ctrl, &data->ctsu_cfg);
	if (ret != FSP_SUCCESS) {
		LOG_ERR("CTSU Open failed");
		return -EIO;
	}

	data->work_phase = TUNING;
	do {
		ret = R_CTSU_ScanStart(&data->ctsu_ctrl);
		if (ret != FSP_SUCCESS) {
			LOG_ERR("CTSU: Failed to start scan");
			return -EIO;
		}
		k_sem_take(&data->tune_scan_end, K_FOREVER);
		ret = R_CTSU_OffsetTuning(&data->ctsu_ctrl);
	} while (ret != FSP_SUCCESS);

	data->dev = dev;
	k_work_init_delayable(&data->work, work_handler);

	data->work_phase = SCANNING;
	ret = R_CTSU_ScanStart(&data->ctsu_ctrl);
	if (ret != FSP_SUCCESS) {
		LOG_ERR("CTSU: Failed to start scan");
		return -EIO;
	}
	return 0;
}

#define CHANNEL_GET_CONFIG(node_id, prop, idx)                                                     \
	{                                                                                          \
		.channel_num = DT_PROP_BY_IDX(node_id, prop, idx),                                 \
		.config =                                                                          \
			{                                                                          \
				.ssdiv = DT_PROP(node_id, ssdiv),                                  \
				.so = DT_PROP(node_id, so),                                        \
				.snum = DT_PROP(node_id, snum),                                    \
				.sdpa = DT_PROP(node_id, sdpa),                                    \
			},                                                                         \
	},

#define CTSU_CHANNEL_CFG_INIT(node_id)                                                             \
	DT_FOREACH_PROP_ELEM(node_id, channels_num, CHANNEL_GET_CONFIG)

#define CTSU_GET_SENSORS_COUNT(node_id) DT_PROP_LEN(node_id, channels_num)

#define COMPONENT_GET_CONFIG(node_id, component)                                                   \
	COND_CODE_1(IS_EQ(DT_ENUM_IDX(node_id, component_type), component),                        \
	({                                                                                         \
		.p_channel_num = (uint8_t[])DT_PROP(node_id, channels_num),                        \
		.num_elements = DT_PROP_LEN(node_id, channels_num),                                \
		.threshold = DT_PROP(node_id, touch_count_threshold),                              \
		.resolution = DT_PROP(node_id, resolution),                                        \
		.hysteresis = DT_PROP(node_id, threshold_range),                                   \
		.zephyr_code = DT_PROP(node_id, zephyr_code),                                      \
	},),                                                                                       \
	())

#define COMPONENT_GET_COUNT(node_id, component)                                                    \
	+COND_CODE_1(IS_EQ(DT_ENUM_IDX(node_id, component_type), component), (1), (0))

#define RENESAS_RX_CTSU_INIT(idx)                                                                  \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	static void ctsu_irq_config_func_##idx(void)                                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, ctsuwr, irq),                                 \
			    DT_INST_IRQ_BY_NAME(idx, ctsuwr, priority), ctsuwr_isr,                \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, ctsurd, irq),                                 \
			    DT_INST_IRQ_BY_NAME(idx, ctsurd, priority), ctsurd_isr,                \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(idx, ctsufn, irq),                                 \
			    DT_INST_IRQ_BY_NAME(idx, ctsufn, priority), ctsufn_isr,                \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQ_BY_NAME(idx, ctsuwr, irq));                                 \
		irq_enable(DT_INST_IRQ_BY_NAME(idx, ctsurd, irq));                                 \
		irq_enable(DT_INST_IRQ_BY_NAME(idx, ctsufn, irq));                                 \
	}                                                                                          \
                                                                                                   \
	static touch_channel_cfg_t ctsu_channel_cfgs_##idx[] = {                                   \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(idx, CTSU_CHANNEL_CFG_INIT)};                    \
                                                                                                   \
	static touch_component_cfg_t button_cfgs_##idx[] = {                                       \
		DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(idx, COMPONENT_GET_CONFIG, BUTTON_TYPE)};  \
	static touch_component_cfg_t slider_cfgs_##idx[] = {                                       \
		DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(idx, COMPONENT_GET_CONFIG, SLIDER_TYPE)};  \
	static touch_component_cfg_t wheel_cfgs_##idx[] = {                                        \
		DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(idx, COMPONENT_GET_CONFIG, WHEEL_TYPE)};   \
                                                                                                   \
	static uint8_t channels_index_map_##idx[DT_PROP(DT_NODELABEL(ctsu), max_num_sensors)];     \
                                                                                                   \
	static const struct renesas_rx_ctsu_config renesas_rx_ctsu_config_##idx = {                \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		.channel_cfgs = ctsu_channel_cfgs_##idx,                                           \
		.button_cfgs = button_cfgs_##idx,                                                  \
		.num_buttons = DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(idx, COMPONENT_GET_COUNT,   \
								       BUTTON_TYPE),               \
		.slider_cfgs = slider_cfgs_##idx,                                                  \
		.num_sliders = DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(idx, COMPONENT_GET_COUNT,   \
								       SLIDER_TYPE),               \
		.wheel_cfgs = wheel_cfgs_##idx,                                                    \
		.num_wheels = DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(idx, COMPONENT_GET_COUNT,    \
								      WHEEL_TYPE),                 \
		.debounce_interval_ms = CONFIG_CTSU_DEBOUNCE_INTERVAL_MS,                          \
		.channels_index_map = channels_index_map_##idx,                                    \
	};                                                                                         \
                                                                                                   \
	static ctsu_element_cfg_t ctsu_element_cfgs_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(   \
		idx, CTSU_GET_SENSORS_COUNT, (+))];                                                \
	static touch_state_t button_prev_state_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(      \
		idx, COMPONENT_GET_COUNT, BUTTON_TYPE)];                                           \
	static touch_state_t button_curr_state_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(      \
		idx, COMPONENT_GET_COUNT, BUTTON_TYPE)];                                           \
	static uint16_t slider_prev_position_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(        \
		idx, COMPONENT_GET_COUNT, SLIDER_TYPE)];                                           \
	static uint16_t slider_curr_position_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(        \
		idx, COMPONENT_GET_COUNT, SLIDER_TYPE)];                                           \
	static uint16_t wheels_prev_position_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(        \
		idx, COMPONENT_GET_COUNT, WHEEL_TYPE)];                                            \
	static uint16_t wheels_curr_position_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(        \
		idx, COMPONENT_GET_COUNT, WHEEL_TYPE)];                                            \
	static uint16_t channel_result_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(                \
		idx, CTSU_GET_SENSORS_COUNT, (+))];                                                \
	static uint16_t channel_reference_##idx[DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(             \
		idx, CTSU_GET_SENSORS_COUNT, (+))] = {0};                                          \
	static struct renesas_rx_ctsu_data renesas_rx_ctsu_data_##idx = {                          \
		.ctsu_cfg =                                                                        \
			{                                                                          \
				.cap = CTSU_CAP_SOFTWARE,                                          \
				.md = CTSU_MODE_SELF_MULTI_SCAN,                                   \
				.num_rx = DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(                   \
					idx, CTSU_GET_SENSORS_COUNT, (+)),                         \
				.num_moving_average = CONFIG_CTSU_NUM_MOVING_AVERAGE,              \
				.atune1 = CONFIG_CTSU_POWER_SUPPLY_CAPACITY,                       \
				.txvsel = CONFIG_CTSU_TRANSMISSION_POWER_SUPPLY,                   \
				.ctsuchac0 = 0,                                                    \
				.ctsuchac1 = 0,                                                    \
				.ctsuchac2 = 0,                                                    \
				.ctsuchac3 = 0,                                                    \
				.ctsuchac4 = 0,                                                    \
				.ctsuchtrc0 = 0,                                                   \
				.ctsuchtrc1 = 0,                                                   \
				.ctsuchtrc2 = 0,                                                   \
				.ctsuchtrc3 = 0,                                                   \
				.ctsuchtrc4 = 0,                                                   \
				.tuning_enable = true,                                             \
				.p_elements = ctsu_element_cfgs_##idx,                             \
				.p_callback = NULL,                                                \
				.p_context = NULL,                                                 \
			},                                                                         \
		.buttons_prev_states = button_prev_state_##idx,                                    \
		.buttons_curr_states = button_curr_state_##idx,                                    \
		.sliders_prev_position = slider_prev_position_##idx,                               \
		.sliders_curr_position = slider_curr_position_##idx,                               \
		.wheels_prev_position = wheels_prev_position_##idx,                                \
		.wheels_curr_position = wheels_curr_position_##idx,                                \
		.channels_count_result = channel_result_##idx,                                     \
		.channels_reference = channel_reference_##idx,                                     \
		.work_phase = INITIALIZING,                                                        \
	};                                                                                         \
                                                                                                   \
	static void sort_configs_by_channel_num##idx(const struct device *dev)                     \
	{                                                                                          \
		struct renesas_rx_ctsu_data *data = dev->data;                                     \
                                                                                                   \
		memset(channels_index_map_##idx, 0xff, sizeof(channels_index_map_##idx));          \
		for (int i = 0; i < data->ctsu_cfg.num_rx; i++) {                                  \
			int min_idx = i;                                                           \
			for (int j = i + 1; j < data->ctsu_cfg.num_rx; j++) {                      \
				if (ctsu_channel_cfgs_##idx[j].channel_num <                       \
				    ctsu_channel_cfgs_##idx[min_idx].channel_num) {                \
					min_idx = j;                                               \
				}                                                                  \
			}                                                                          \
			if (min_idx != i) {                                                        \
				touch_channel_cfg_t tmp = ctsu_channel_cfgs_##idx[i];              \
				ctsu_channel_cfgs_##idx[i] = ctsu_channel_cfgs_##idx[min_idx];     \
				ctsu_channel_cfgs_##idx[min_idx] = tmp;                            \
			}                                                                          \
			ctsu_element_cfgs_##idx[i] = ctsu_channel_cfgs_##idx[i].config;            \
			channels_index_map_##idx[ctsu_channel_cfgs_##idx[i].channel_num] = i;      \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static int renesas_rx_ctsu_init_##idx(const struct device *dev)                            \
	{                                                                                          \
		sort_configs_by_channel_num##idx(dev);                                             \
		ctsu_irq_config_func_##idx();                                                      \
		int err = renesas_rx_ctsu_init(dev);                                               \
		return err;                                                                        \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, &renesas_rx_ctsu_init_##idx, NULL, &renesas_rx_ctsu_data_##idx, \
			      &renesas_rx_ctsu_config_##idx, POST_KERNEL,                          \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RENESAS_RX_CTSU_INIT)
