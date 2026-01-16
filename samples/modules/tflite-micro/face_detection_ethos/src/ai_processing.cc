#include <zephyr/logging/log.h>
#include <zephyr/drivers/video.h>

#include "inference_process.hpp"
#include "common_util.h"
#include "ai_processing.h"
#include "model_data.h"

LOG_MODULE_REGISTER(ai_processing, CONFIG_LOG_DEFAULT_LEVEL);

using namespace InferenceProcess;

namespace
{
TENSOR_ARENA_ATTR uint8_t inferenceProcessTensorArena[TENSOR_ARENA_SIZE];

std::array<st_ai_detection_point_t, CONFIG_AI_MAX_DETECTION_NUM> g_ai_detection;

static InferenceProcess::InferenceProcess inference(inferenceProcessTensorArena, TENSOR_ARENA_SIZE);
} // namespace

void ai_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);

	struct k_msgq *ai_input_msgq = (struct k_msgq *)arg1;
	struct k_msgq *ai_result_msgq = (struct k_msgq *)arg2;
	st_ai_detection_point_t *p_det;
	ai_input_msg_t ai_input_msg;

	for (;;) {
		if (k_msgq_get(ai_input_msgq, &ai_input_msg, K_FOREVER) == 0) {
			InferenceProcess::InferenceJob job(
				"FaceDetection", {DataPtr(model_data, sizeof(model_data))},
				{DataPtr(ai_input_msg.data, ai_input_msg.size)});

			inference.runJob(job);

			auto &results = job.getResults();

			g_ai_detection.fill({0, 0, 0, 0});

			for (uint16_t i = 0; i < results.size(); ++i) {
				g_ai_detection[i].m_x = results[i].m_x0;
				g_ai_detection[i].m_y = results[i].m_y0;
				g_ai_detection[i].m_w = results[i].m_w;
				g_ai_detection[i].m_h = results[i].m_h;
				LOG_INF("x=%d, y=%d, w=%d, h=%d\n", g_ai_detection[i].m_x,
					g_ai_detection[i].m_y, g_ai_detection[i].m_w,
					g_ai_detection[i].m_h);
			}

			p_det = &g_ai_detection[0];

			k_msgq_put(ai_result_msgq, &p_det, K_NO_WAIT);
		}
	}
}
