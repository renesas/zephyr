/*
 * Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**********************************************************************************************************************
 * File Name    : common_utils.h
 * Version      : .
 * Description  : .
 *********************************************************************************************************************/
#ifndef COMMON_UTIL_H__
#define COMMON_UTIL_H__

#include <string.h>
#include <stdint.h>
#include <zephyr/drivers/video.h>

#define DISPLAY_QUEUE_DEPTH   CONFIG_VIDEO_BUFFER_POOL_NUM_MAX
#define AI_INPUT_QUEUE_DEPTH  1
#define AI_RESULT_QUEUE_DEPTH CONFIG_VIDEO_BUFFER_POOL_NUM_MAX

#define CAMERA_THREAD_PRIORITY  4
#define DISPLAY_THREAD_PRIORITY 5
#define AI_THREAD_PRIORITY      6

/* RGB565 extraction macros */
#define RGB565_R_MASK 0xF800U
#define RGB565_G_MASK 0x07E0U
#define RGB565_B_MASK 0x001FU

#define RGB565_R_SHIFT 11
#define RGB565_G_SHIFT 5
#define RGB565_B_SHIFT 0

/* Grayscale weighting constants (approx 0.25/0.50/0.125) */
#define GRAY_WEIGHT_R 2U /* equivalent to 0.25*8 */
#define GRAY_WEIGHT_G 1U /* equivalent to 0.50*2 */
#define GRAY_WEIGHT_B 1U /* equivalent to 0.125*8, approximated */

/* Center crop macro */
#define CROP_OFFSET(in_w, in_h) (((in_w) > (in_h)) ? ((in_w) - (in_h)) / 2U : 0U)

/* Convert RGB565 pixel to grayscale int8_t */
#define RGB565_TO_INT8_GRAY(pixel)                                                                 \
	((int8_t)(((((pixel & RGB565_R_MASK) >> RGB565_R_SHIFT) * GRAY_WEIGHT_R) +                 \
		   (((pixel & RGB565_G_MASK) >> RGB565_G_SHIFT) * GRAY_WEIGHT_G) +                 \
		   (((pixel & RGB565_B_MASK) >> RGB565_B_SHIFT) * GRAY_WEIGHT_B)) -                \
		  0x80))

/* The coordinate of the bounding box corner, bounding box width and height based on 192x192 gray
 * pixel area */
typedef struct ai_detection_point_t {
	signed short m_x;
	signed short m_y;
	signed short m_w;
	signed short m_h;
} st_ai_detection_point_t;

typedef struct {
	size_t size;
	int8_t *data;
} ai_input_msg_t;

typedef struct {
	struct video_buffer *vbuf;
	struct device *video_dev;
} camera_frame_msg_t;

#endif /* COMMON_UTIL_H__ */
