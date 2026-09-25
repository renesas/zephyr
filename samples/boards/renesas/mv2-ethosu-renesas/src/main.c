/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>

#include "ai_processing.h"
#include "camera.h"
#include "display.h"
#include "common_util.h"

#ifdef CONFIG_SOC_SERIES_RA8P1
#include <soc.h>

static void ra8p1_bus_arbitration_round_robin(void)
{
	R_BUS->BUSMABT_b.ARBS = 1;
	R_BUS->BUSSABT0.MRE0BI_b.ARBS = 1;
}
#endif /* CONFIG_SOC_SERIES_RA8P1 */

K_THREAD_STACK_DEFINE(camera_stack_area, CONFIG_CAMERA_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(display_stack_area, CONFIG_DISPLAY_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(ai_preprocess_stack_area, CONFIG_AI_PREPROCESS_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(ai_stack_area, CONFIG_AI_THREAD_STACK_SIZE);

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static struct k_thread camera_thread;
static struct k_thread display_thread;
static struct k_thread ai_preprocess_thread;
static struct k_thread ai_thread;

K_MSGQ_DEFINE(display_frame_msgq, sizeof(camera_frame_msg_t), DISPLAY_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(ai_raw_frame_msgq, sizeof(camera_frame_msg_t), AI_RAW_FRAME_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(ai_input_msgq, sizeof(ai_input_msg_t), AI_INPUT_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(ai_result_msgq, sizeof(ai_result_msg_t), AI_RESULT_QUEUE_DEPTH, 4);

int main(void)
{
	LOG_INF("ExecuTorch MobileNetV2 live camera classification (mv2-ethosu-renesas)");

	int ret;

#ifdef CONFIG_SOC_SERIES_RA8P1
	ra8p1_bus_arbitration_round_robin();
#endif
	ret = camera_init();
	if (ret != 0) {
		LOG_ERR("Camera initialization failed");
		return 0;
	}

	ret = display_init();
	if (ret != 0) {
		LOG_ERR("Display initialization failed");
		return 0;
	}

	ret = ai_init();
	if (ret != 0) {
		LOG_ERR("AI (ExecuTorch) initialization failed");
		return 0;
	}

	k_thread_create(&camera_thread, camera_stack_area, K_THREAD_STACK_SIZEOF(camera_stack_area),
			camera_task, &display_frame_msgq, &ai_raw_frame_msgq, NULL,
			CAMERA_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&ai_preprocess_thread, ai_preprocess_stack_area,
			K_THREAD_STACK_SIZEOF(ai_preprocess_stack_area), camera_ai_preprocess_task,
			&ai_raw_frame_msgq, &ai_input_msgq, NULL, AI_PREPROCESS_THREAD_PRIORITY, 0,
			K_NO_WAIT);

	k_thread_create(&ai_thread, ai_stack_area, K_THREAD_STACK_SIZEOF(ai_stack_area), ai_task,
			&ai_input_msgq, &ai_result_msgq, NULL, AI_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&display_thread, display_stack_area,
			K_THREAD_STACK_SIZEOF(display_stack_area), display_task,
			&display_frame_msgq, &ai_result_msgq, NULL, DISPLAY_THREAD_PRIORITY, 0,
			K_NO_WAIT);

	k_sleep(K_FOREVER);

	return 0;
}
