/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>

#include "camera.h"
#include "common_util.h"

LOG_MODULE_REGISTER(camera, CONFIG_LOG_DEFAULT_LEVEL);

static uint8_t model_buffer_rgb[CONFIG_AI_INPUT_WIDTH * CONFIG_AI_INPUT_HEIGHT * 3] __aligned(4);
K_SEM_DEFINE(ai_buffer_free_sem, 1, 1);
#ifdef CONFIG_APP_SERIALIZE_FRAMES
K_SEM_DEFINE(ai_frame_gate_sem, 1, 1);
#endif
const struct device *video_dev;
static struct video_buffer *buffers[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];
static enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;

static void set_control(const struct device *dev, uint32_t id, int32_t value)
{
	struct video_control ctrl = {
		.id = id,
		.val = value,
	};
	int ret = video_set_ctrl(dev, &ctrl);

	LOG_INF("Set CID=0x%x val=%d ret=%d", id, value, ret);
}

int camera_init(void)
{
	struct video_format fmt;
	struct video_caps caps;
	size_t bsize;
	int ret;

	video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));

	ret = device_is_ready(video_dev);
	if (ret < 0) {
		LOG_ERR("%s device is not ready", video_dev->name);
		return ret;
	}

	LOG_INF("- Device name: %s", video_dev->name);

	/* Get capabilities */
	caps.type = type;
	ret = video_get_caps(video_dev, &caps);
	if (ret < 0) {
		LOG_ERR("Unable to retrieve video capabilities");
		return ret;
	}

	/* Get default/native format */
	fmt.type = type;
	ret = video_get_format(video_dev, &fmt);
	if (ret < 0) {
		LOG_ERR("Unable to retrieve video format");
		return ret;
	}

	fmt.width = CONFIG_VIDEO_WIDTH;
	fmt.height = CONFIG_VIDEO_HEIGHT;
	fmt.pixelformat = VIDEO_PIX_FMT_YUYV;
	ret = video_set_format(video_dev, &fmt);
	if (ret < 0) {
		LOG_ERR("Unable to set up video format %d", ret);
		return ret;
	}

	set_control(video_dev, VIDEO_CID_HFLIP, 1);

	/* Size to allocate for each buffer */
	bsize = fmt.pitch * fmt.height;

	/* Alloc video buffers and enqueue for capture */
	for (int i = 0; i < ARRAY_SIZE(buffers); i++) {
		buffers[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_NO_WAIT);
		if (buffers[i] == NULL) {
			LOG_ERR("Unable to alloc video buffer");
			return -1;
		}
		buffers[i]->type = type;
		video_enqueue(video_dev, buffers[i]);
	}

	/* Start video capture */
	ret = video_stream_start(video_dev, type);
	if (ret < 0) {
		LOG_ERR("Unable to start capture (interface)");
		return ret;
	}

	LOG_INF("- Camera initialized and capture started");
	return 0;
}

int image_rgb565_to_planar_rgb888(const void *p_input_image_buff, void *p_output_image_buff,
				  uint16_t in_width, uint16_t in_height, uint16_t out_width,
				  uint16_t out_height)
{
	if ((p_input_image_buff == NULL) || (p_output_image_buff == NULL)) {
		return -EINVAL;
	}

	if (in_width < in_height) {
		return -EINVAL;
	}

	register uint16_t *p_input = (uint16_t *)p_input_image_buff;
	const uint32_t plane_size = (uint32_t)out_width * out_height;
	uint8_t *p_plane_r = (uint8_t *)p_output_image_buff;
	uint8_t *p_plane_g = p_plane_r + plane_size;
	uint8_t *p_plane_b = p_plane_g + plane_size;

	uint32_t crop_offset = (in_width - in_height) / 2;

	for (uint32_t y = 0; y < out_height; y++) {
		uint32_t y_offset = in_width * ((in_height * y) / out_height);
		uint16_t *p_input_base = p_input + crop_offset + y_offset;
		uint32_t row = y * out_width;

		for (uint32_t x = 0; x < out_width; x++) {
			uint16_t input = *(p_input_base + ((in_height * x) / out_width));

			uint8_t r = (input >> 11) & 0x1F;
			uint8_t g = (input >> 5) & 0x3F;
			uint8_t b = input & 0x1F;

			p_plane_r[row + x] = (r * 255) / 31;
			p_plane_g[row + x] = (g * 255) / 63;
			p_plane_b[row + x] = (b * 255) / 31;
		}
	}

	return 0;
}

void camera_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);

	struct video_buffer *vbuf = &(struct video_buffer){};
	int err;
	struct k_msgq *display_frame_msgq = (struct k_msgq *)arg1;
	struct k_msgq *ai_raw_frame_msgq = (struct k_msgq *)arg2;
	camera_frame_msg_t camera_frame_msg;

	vbuf->type = type;

	while (1) {
		err = video_dequeue(video_dev, &vbuf, K_FOREVER);
		if (err) {
			LOG_ERR("Unable to dequeue video buf");
			continue;
		}

		camera_frame_msg.capture = k_cycle_get_32();
		camera_frame_msg.vbuf = vbuf;
		camera_frame_msg.video_dev = (struct device *)video_dev;

		if (k_msgq_num_free_get(ai_raw_frame_msgq) > 0 && ai_frame_gate_take()) {
			err = k_msgq_put(ai_raw_frame_msgq, &camera_frame_msg, K_NO_WAIT);
			if (err) {
				LOG_ERR("Failed to put frame into AI raw frame queue");
				ai_frame_gate_give();
			}
		}

		err = k_msgq_put(display_frame_msgq, &camera_frame_msg, K_NO_WAIT);
		if (err) {
			video_enqueue(video_dev, vbuf);
		}
	}
}

void camera_ai_preprocess_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);

	struct k_msgq *ai_raw_frame_msgq = (struct k_msgq *)arg1;
	struct k_msgq *ai_input_msgq = (struct k_msgq *)arg2;
	camera_frame_msg_t camera_frame_msg;
	ai_input_msg_t ai_input_msg;
	int err;

	while (1) {
		if (k_msgq_get(ai_raw_frame_msgq, &camera_frame_msg, K_FOREVER) != 0) {
			continue;
		}

		if (k_sem_take(&ai_buffer_free_sem, K_NO_WAIT) != 0) {
			ai_frame_gate_give();
			continue;
		}

		uint32_t convert_start = k_cycle_get_32();

		image_rgb565_to_planar_rgb888(
			&camera_frame_msg.vbuf->buffer[0], &model_buffer_rgb[0], CONFIG_VIDEO_WIDTH,
			CONFIG_VIDEO_HEIGHT, CONFIG_AI_INPUT_WIDTH, CONFIG_AI_INPUT_HEIGHT);
		ai_input_msg.timing.capture = camera_frame_msg.capture;
		ai_input_msg.timing.convert_us =
			k_cyc_to_us_floor32(k_cycle_get_32() - convert_start);
		ai_input_msg.size = sizeof(model_buffer_rgb);
		ai_input_msg.data = &model_buffer_rgb[0];

		err = k_msgq_put(ai_input_msgq, &ai_input_msg, K_NO_WAIT);
		if (err) {
			LOG_ERR("Failed to put frame into AI input queue");
			ai_frame_gate_give();
		}
	}
}
