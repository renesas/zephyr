/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/video.h>
#include <lvgl.h>
#include "r_glcdc.h"

#include "display.h"
#include "common_util.h"

LOG_MODULE_REGISTER(app_display, CONFIG_LOG_DEFAULT_LEVEL);

#define INFO_PANEL_WIDTH  (1024 - CONFIG_VIDEO_WIDTH)
#define INFO_PANEL_HEIGHT 600
#define INFO_FONT         (&lv_font_montserrat_24)
#define INFO_BLOCK_Y      24

static const struct device *display_dev;
static st_ai_classification_point_t *ai_results;
static lv_obj_t *detection_canvas;
static lv_obj_t *info_panel;
static lv_obj_t *info_label;

static void update_video_canvas(uint8_t *frame_buf)
{
	lv_layer_t layer;

	lv_canvas_set_buffer(detection_canvas, frame_buf, CONFIG_VIDEO_WIDTH, CONFIG_VIDEO_HEIGHT,
			     LV_COLOR_FORMAT_RGB565);
	lv_canvas_init_layer(detection_canvas, &layer);
	lv_canvas_finish_layer(detection_canvas, &layer);
}

/* Rebuilds info_label's text; LVGL redraws it on the next lv_timer_handler(). */
static void update_info_panel(uint32_t inference_time_ms, st_ai_classification_point_t *results,
			      uint8_t result_count)
{
	char text[512];
	size_t off = 0;
	int n;

	n = snprintf(&text[off], sizeof(text) - off, "Inference: %u ms", inference_time_ms);
	if (n > 0) {
		off += (size_t)n;
	}

	if (result_count > 0) {
		for (uint8_t i = 0; i < result_count && off < sizeof(text); i++) {
			n = snprintf(&text[off], sizeof(text) - off, "\n%u. %s %.0f%%", i + 1,
				     results[i].label, (double)(results[i].probability * 100.0f));
			if (n > 0) {
				off += (size_t)n;
			}
		}
	} else if (off < sizeof(text)) {
		n = snprintf(&text[off], sizeof(text) - off, "\n--");
		if (n > 0) {
			off += (size_t)n;
		}
	}

	lv_label_set_text(info_label, text);
}

int display_init(void)
{
	lv_display_t *lv_disp;
	lv_obj_t *scr;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display device not ready");
		return -1;
	}

	display_blanking_off(display_dev);

	lv_disp = lv_display_get_default();
	if (lv_disp == NULL) {
		LOG_ERR("No LVGL display registered");
		return -1;
	}
	lv_display_set_render_mode(lv_disp, LV_DISPLAY_RENDER_MODE_FULL);

	scr = lv_display_get_screen_active(lv_disp);

	detection_canvas = lv_canvas_create(scr);
	if (detection_canvas == NULL) {
		LOG_ERR("Failed to create LVGL detection canvas");
		return -1;
	}
	lv_obj_set_pos(detection_canvas, 0, 0);

	info_panel = lv_obj_create(scr);
	if (info_panel == NULL) {
		LOG_ERR("Failed to create LVGL info panel");
		return -1;
	}
	lv_obj_set_pos(info_panel, CONFIG_VIDEO_WIDTH, 0);
	lv_obj_set_size(info_panel, INFO_PANEL_WIDTH, INFO_PANEL_HEIGHT);
	lv_obj_set_style_bg_color(info_panel, lv_color_white(), 0);
	lv_obj_set_style_border_width(info_panel, 0, 0);
	lv_obj_set_style_radius(info_panel, 0, 0);
	lv_obj_set_style_pad_all(info_panel, 0, 0);

	info_label = lv_label_create(info_panel);
	if (info_label == NULL) {
		LOG_ERR("Failed to create LVGL info label");
		return -1;
	}
	lv_obj_set_pos(info_label, 16, INFO_BLOCK_Y);
	lv_obj_set_width(info_label, INFO_PANEL_WIDTH - 32);
	lv_obj_set_style_text_font(info_label, INFO_FONT, 0);
	lv_obj_set_style_text_color(info_label, lv_color_black(), 0);

	update_info_panel(0, NULL, 0);

	lv_sysmon_show_performance(lv_disp);

	LOG_INF("- Display initialized");
	return 0;
}

void display_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);

	int err;
	struct k_msgq *display_frame_msgq = (struct k_msgq *)arg1;
	struct k_msgq *ai_result_msgq = (struct k_msgq *)arg2;
	camera_frame_msg_t camera_frame_msg;
	ai_result_msg_t ai_result;
	static camera_frame_msg_t held_frame_msg;
	static bool have_held_frame;

	while (1) {
		if (k_msgq_get(display_frame_msgq, &camera_frame_msg, K_FOREVER) != 0) {
			continue;
		}

		if (k_msgq_get(ai_result_msgq, &ai_result, K_NO_WAIT) == 0) {
			ai_results = ai_result.results;
			update_info_panel(ai_result.inference_time_ms, ai_results,
					  ai_result.result_count);
		}
		update_video_canvas(camera_frame_msg.vbuf->buffer);

		lv_timer_handler();

		if (have_held_frame) {
			err = video_enqueue(held_frame_msg.video_dev, held_frame_msg.vbuf);
			if (err) {
				LOG_ERR("Unable to requeue video buf");
			}
		}
		held_frame_msg = camera_frame_msg;
		have_held_frame = true;
	}
}
