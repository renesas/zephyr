/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>

static const struct device *const test_touch_dev = DEVICE_DT_GET(DT_NODELABEL(ctsu));
#define BUTTON_1_IDX DT_NODE_CHILD_IDX(DT_NODELABEL(onboard_button_1))
#define BUTTON_2_IDX DT_NODE_CHILD_IDX(DT_NODELABEL(onboard_button_2))
#define SLIDER_IDX   DT_NODE_CHILD_IDX(DT_NODELABEL(onboard_slider))

enum {
	TOUCH_BUTTON_1 = 10,
	TOUCH_BUTTON_2 = 11,
	TOUCH_SLIDER = 1,
};

static volatile int event_count[DT_CHILD_NUM_STATUS_OKAY(DT_NODELABEL(ctsu))] = {0};
static uint16_t last_code;
static volatile int32_t last_val[DT_CHILD_NUM_STATUS_OKAY(DT_NODELABEL(ctsu))];
static struct k_sem btn1_sem;
static struct k_sem btn2_sem;
static struct k_sem sldr_sem;

static void sem_setup(void)
{
	k_sem_init(&btn1_sem, 0, 1);
	k_sem_init(&btn2_sem, 0, 1);
	k_sem_init(&sldr_sem, 0, 1);
}

static void test_touch_keys_cb_handler(struct input_event *evt, void *user_data)
{
	switch (evt->code) {
	case TOUCH_BUTTON_1:
		if (evt->value == 0) {
			printk("Button 1 released\n");
			if (last_val[BUTTON_1_IDX] != 0) {
				k_sem_give(&btn1_sem);
			}
		} else {
			event_count[BUTTON_1_IDX]++;
			printk("Button 1 pressed %d time(s)\n", event_count[BUTTON_1_IDX]);
		}
		last_val[BUTTON_1_IDX] = evt->value;
		break;

	case TOUCH_BUTTON_2:
		if (evt->value == 0) {
			printk("Button 2 released\n");
			if (last_val[BUTTON_2_IDX] != 0) {
				k_sem_give(&btn2_sem);
			}
		} else {
			event_count[BUTTON_2_IDX]++;
			printk("Button 2 pressed %d time(s)\n", event_count[BUTTON_2_IDX]);
		}
		last_val[BUTTON_2_IDX] = evt->value;
		break;

	case TOUCH_SLIDER:
		if (evt->value == 0) {
			printk("Slider released\n");
			if (last_val[SLIDER_IDX] != 0) {
				k_sem_give(&sldr_sem);
			}
		} else {
			if (last_val[SLIDER_IDX] == 0) {
				event_count[SLIDER_IDX]++;
				printk("Slider pressed\n");
			}
			printk("Position: %d\n", evt->value);
		}
		last_val[SLIDER_IDX] = evt->value;
		break;

	default:
		break;
	}
	last_code = evt->code;
}
INPUT_CALLBACK_DEFINE(test_touch_dev, test_touch_keys_cb_handler, NULL);

int main(void)
{
	sem_setup();
	printk("On board CTSU components sample started!\n");
	printk("Press on touch nodes for sample\n");
	return 0;
}
