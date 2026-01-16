/*
 * Copyright 2019-2022 Arm Limited and/or its affiliates <open-source-office@arm.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <queue>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cstdint>

#include "DetectionResult.hpp"

namespace InferenceProcess
{
	struct DataPtr
	{
		void *data;
		size_t size;

		DataPtr(void *data = nullptr, size_t size = 0);

		void invalidate();
		void clean();
	};

	struct InferenceJob
	{
	public:
		InferenceJob();
		InferenceJob(const std::string &name, const DataPtr &networkModel,
			     const std::vector<DataPtr> &input);

		const std::vector<arm::app::object_detection::DetectionResult>& getResults() const;
		void invalidate();
		void clean();

	public:
		std::string name;
		DataPtr networkModel;
		std::vector<DataPtr> input;
		std::vector<arm::app::object_detection::DetectionResult> results;
	};

	class InferenceProcess
	{
	public:
		InferenceProcess(uint8_t *_tensorArena, size_t _tensorArenaSize)
		    : tensorArena(_tensorArena), tensorArenaSize(_tensorArenaSize)
		{
		}

		bool runJob(InferenceJob &job);

	private:
		uint8_t *tensorArena;
		const size_t tensorArenaSize;
	};
} /* namespace InferenceProcess */
