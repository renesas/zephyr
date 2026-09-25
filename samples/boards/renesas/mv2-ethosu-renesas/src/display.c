/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

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

#ifdef CONFIG_APP_PROFILING
/* "On screen" = LVGL finished a flush. The GLCDC driver blocks until the
 * vsync that follows the buffer swap before returning, so this is when the
 * new image starts being scanned out.
 */
static volatile uint32_t flush_count;
static volatile uint32_t flush_cycles;

static void flush_finish_cb(lv_event_t *e)
{
	ARG_UNUSED(e);
	flush_cycles = k_cycle_get_32();
	flush_count++;
}

enum prof_metric {
	P_PRE,
	P_CONVERT,
	P_QUANT,
	P_PRE_WAIT,
	P_INFER,
	P_POST,
	P_POST_AI,
	P_POST_DISP,
	P_E2E,
	P_VIDEO,
	P_COUNT,
};

static const char *const prof_label[P_COUNT] = {
	[P_PRE] = "preprocess total (capture -> NPU start)",
	[P_CONVERT] = "  crop/resize/RGB565->RGB888",
	[P_QUANT] = "  quantize LUT",
	[P_PRE_WAIT] = "  queue waits",
	[P_INFER] = "inference (Method::execute)",
	[P_POST] = "postprocess total (NPU done -> on screen)",
	[P_POST_AI] = "  get_outputs + top-k",
	[P_POST_DISP] = "  queue + render + flush",
	[P_E2E] = "end-to-end (capture -> result on screen)",
	[P_VIDEO] = "video latency (capture -> frame on screen)",
};

static uint64_t prof_sum[P_COUNT];
static uint32_t prof_max[P_COUNT];
static uint32_t prof_cnt[P_COUNT];

static frame_timing_t prof_pending;
static bool prof_pending_valid;

static uint32_t cyc_us(uint32_t from, uint32_t to)
{
	return k_cyc_to_us_floor32(to - from);
}

static void prof_add(enum prof_metric m, uint32_t us)
{
	prof_sum[m] += us;
	prof_cnt[m]++;
	if (us > prof_max[m]) {
		prof_max[m] = us;
	}
}

static void prof_report(void)
{
	for (int m = 0; m < P_COUNT; m++) {
		if (prof_cnt[m] == 0) {
			continue;
		}

		uint32_t avg = (uint32_t)(prof_sum[m] / prof_cnt[m]);

		LOG_INF("PROF %-44s avg %u.%02u  max %u.%02u ms  (n=%u)", prof_label[m],
			avg / 1000U, (avg % 1000U) / 10U, prof_max[m] / 1000U,
			(prof_max[m] % 1000U) / 10U, prof_cnt[m]);
	}

	memset(prof_sum, 0, sizeof(prof_sum));
	memset(prof_max, 0, sizeof(prof_max));
	memset(prof_cnt, 0, sizeof(prof_cnt));
}

static void prof_result_arrived(const frame_timing_t *t)
{
	prof_pending = *t;
	prof_pending_valid = true;
}

static uint32_t prof_flush_count(void)
{
	return flush_count;
}

/* Called after an lv_timer_handler() pass that flushed: everything set on the
 * label/canvas before it is now on screen.
 */
static void prof_frame_flushed(uint32_t video_capture)
{
	uint32_t shown = flush_cycles;

	prof_add(P_VIDEO, cyc_us(video_capture, shown));

	if (!prof_pending_valid) {
		return;
	}
	prof_pending_valid = false;

	const frame_timing_t *t = &prof_pending;
	uint32_t pre = cyc_us(t->capture, t->exec_start);
	uint32_t compute = t->convert_us + t->quant_us;

	prof_add(P_PRE, pre);
	prof_add(P_CONVERT, t->convert_us);
	prof_add(P_QUANT, t->quant_us);
	prof_add(P_PRE_WAIT, pre > compute ? pre - compute : 0);
	prof_add(P_INFER, cyc_us(t->exec_start, t->exec_end));
	prof_add(P_POST, cyc_us(t->exec_end, shown));
	prof_add(P_POST_AI, cyc_us(t->exec_end, t->post_done));
	prof_add(P_POST_DISP, cyc_us(t->post_done, shown));
	prof_add(P_E2E, cyc_us(t->capture, shown));

	if (prof_cnt[P_E2E] >= CONFIG_APP_PROFILING_PERIOD) {
		prof_report();
	}
}
#else
static inline void prof_result_arrived(const frame_timing_t *t)
{
	ARG_UNUSED(t);
}

static inline uint32_t prof_flush_count(void)
{
	return 0;
}

static inline void prof_frame_flushed(uint32_t video_capture)
{
	ARG_UNUSED(video_capture);
}
#endif /* CONFIG_APP_PROFILING */

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

#ifdef CONFIG_APP_PROFILING
	lv_display_add_event_cb(lv_disp, flush_finish_cb, LV_EVENT_FLUSH_FINISH, NULL);
#endif

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
			prof_result_arrived(&ai_result.timing);
		}
		update_video_canvas(camera_frame_msg.vbuf->buffer);

		uint32_t flushes_before = prof_flush_count();

		lv_timer_handler();

		if (prof_flush_count() != flushes_before) {
			prof_frame_flushed(camera_frame_msg.capture);
		}

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
