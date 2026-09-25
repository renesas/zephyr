/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef COMMON_UTIL_H__
#define COMMON_UTIL_H__

#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>

#define DISPLAY_QUEUE_DEPTH   CONFIG_VIDEO_BUFFER_POOL_NUM_MAX

extern struct k_sem ai_buffer_free_sem;
#define AI_RAW_FRAME_QUEUE_DEPTH 1
#define AI_INPUT_QUEUE_DEPTH  1
#define AI_RESULT_QUEUE_DEPTH 1

#define CAMERA_THREAD_PRIORITY       -4
#define AI_PREPROCESS_THREAD_PRIORITY -3
#define DISPLAY_THREAD_PRIORITY      -2
#define AI_THREAD_PRIORITY           -1

typedef struct {
	size_t size;
	uint8_t *data;
} ai_input_msg_t;

typedef struct {
	uint32_t label_idx;
	const char *label;
	float probability;
} st_ai_classification_point_t;

typedef struct {
	st_ai_classification_point_t *results;
	uint32_t inference_time_ms;
	uint8_t result_count;
} ai_result_msg_t;

typedef struct {
	struct video_buffer *vbuf;
	struct device *video_dev;
} camera_frame_msg_t;

#endif /* COMMON_UTIL_H__ */
