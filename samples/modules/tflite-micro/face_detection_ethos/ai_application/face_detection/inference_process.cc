/*
 * Copyright 2019-2022 Arm Limited and/or its affiliates <open-source-office@arm.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inference_process.hpp"

#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/cortex_m_generic/debug_log_callback.h>
#include <tensorflow/lite/micro/micro_log.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_profiler.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include <cmsis_compiler.h>
#include <inttypes.h>
#include "DetectorPostProcessing.hpp"

using namespace std;

namespace InferenceProcess
{
	const float anchor1[] = {38, 77, 47, 97, 61, 126};
	const float anchor2[] = {14, 26, 19, 37, 28, 55};

	DataPtr::DataPtr(void *_data, size_t _size) : data(_data), size(_size)
	{
	}

	void DataPtr::invalidate()
	{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
		SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(data), size);
#endif
	}

	void DataPtr::clean()
	{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
		SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(data), size);
#endif
	}

	InferenceJob::InferenceJob()
	{
		results.clear();
	}

	InferenceJob::InferenceJob(const string &_name, const DataPtr &_networkModel,
				   const vector<DataPtr> &_input)
	    : name(_name), networkModel(_networkModel), input(_input)
	{
		results.clear();
	}

	void InferenceJob::invalidate()
	{
		networkModel.invalidate();

		for (auto &it : input)
		{
			it.invalidate();
		}

	}

	void InferenceJob::clean()
	{
		networkModel.clean();

		for (auto &it : input)
		{
			it.clean();
		}

	}

	const std::vector<arm::app::object_detection::DetectionResult>&
	InferenceJob::getResults() const
	{
		return results;
	}

	bool InferenceProcess::runJob(InferenceJob &job)
	{
		const tflite::Model *model = ::tflite::GetModel(job.networkModel.data);
		if (model->version() != TFLITE_SCHEMA_VERSION)
		{
			printf("Model schema version unsupported: version=%" PRIu32 ", supported=%d.\n",
			       model->version(), TFLITE_SCHEMA_VERSION);
			return true;
		}

		/* Create the TFL micro interpreter */
		tflite::MicroMutableOpResolver<1> resolver;
		resolver.AddEthosU();

		tflite::MicroInterpreter interpreter(model, resolver, tensorArena, tensorArenaSize);

		/* Allocate tensors */
		TfLiteStatus allocate_status = interpreter.AllocateTensors();
		if (allocate_status != kTfLiteOk)
		{
			printf("Failed to allocate tensors for inference. job=%p\n", &job);
			return true;
		}

		if (job.input.size() != interpreter.inputs_size())
		{
			printf("Number of job and network inputs do not match. input=%zu, network=%zu\n",
			       job.input.size(), interpreter.inputs_size());
			return true;
		}

		/* Copy input data */
		for (size_t i = 0; i < interpreter.inputs_size(); ++i)
		{
			const DataPtr &input = job.input[i];
			const TfLiteTensor *tensor = interpreter.input(i);

			if (input.size != tensor->bytes)
			{
				printf("Input tensor size mismatch. index=%zu, input=%zu, network=%u\n", i,
				       input.size, tensor->bytes);
				return true;
			}

			copy(static_cast<char *>(input.data), static_cast<char *>(input.data) + input.size,
			     tensor->data.uint8);
		}

		/* Run the inference */
		TfLiteStatus invoke_status = interpreter.Invoke();
		if (invoke_status != kTfLiteOk)
		{
			printf("Invoke failed for inference. job=%s\n", job.name.c_str());
			return true;
		}

		arm::app::object_detection::PostProcessParams postProcessParams{
		    CONFIG_AI_INPUT_HEIGHT, CONFIG_AI_INPUT_WIDTH, CONFIG_AI_INPUT_WIDTH, anchor1, anchor2};
		job.results.clear();

		TfLiteTensor *outputTensor0 = interpreter.output(0);
		TfLiteTensor *outputTensor1 = interpreter.output(1);

		arm::app::DetectorPostProcess postProcess = arm::app::DetectorPostProcess(outputTensor0, outputTensor1,
											  job.results, postProcessParams);

		if (!postProcess.DoPostProcess())
		{
			printf("Post-processing failed.");
			return true;
		}

		return false;
	}

} /* namespace InferenceProcess */
