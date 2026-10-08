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

/* Timestamps (k_cycle_get_32) and durations collected as one frame moves
 * through the pipeline; only consumed when CONFIG_APP_PROFILING is set.
 */
typedef struct {
	uint32_t capture;    /* camera_task dequeued the frame from VIN */
	uint32_t convert_us; /* RGB565 -> planar RGB888 crop/resize */
	uint32_t quant_us;   /* uint8 -> model input LUT */
	uint32_t exec_start; /* just before Method::execute() */
	uint32_t exec_end;   /* Method::execute() returned */
	uint32_t post_done;  /* get_outputs() + get_top_k() finished */
} frame_timing_t;

#ifdef CONFIG_APP_SERIALIZE_FRAMES
/* One AI frame in flight: taken when camera_task hands a frame to the AI
 * pipeline, given when its result has been flushed (or the frame is dropped).
 */
extern struct k_sem ai_frame_gate_sem;

static inline bool ai_frame_gate_take(void)
{
	return k_sem_take(&ai_frame_gate_sem, K_NO_WAIT) == 0;
}

static inline void ai_frame_gate_give(void)
{
	k_sem_give(&ai_frame_gate_sem);
}
#else
static inline bool ai_frame_gate_take(void)
{
	return true;
}

static inline void ai_frame_gate_give(void)
{
}
#endif

typedef struct {
	size_t size;
	uint8_t *data;
	frame_timing_t timing;
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
	frame_timing_t timing;
} ai_result_msg_t;

typedef struct {
	struct video_buffer *vbuf;
	struct device *video_dev;
	uint32_t capture; /* k_cycle_get_32() when camera_task dequeued it */
} camera_frame_msg_t;

#endif /* COMMON_UTIL_H__ */
