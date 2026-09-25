/*
 * Copyright 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <executorch/examples/arm/executor_runner/arm_memory_allocator.h>
#include <executorch/extension/data_loader/buffer_data_loader.h>
#include <executorch/runtime/executor/program.h>
#include <executorch/runtime/platform/log.h>
#include <executorch/runtime/platform/platform.h>
#include <executorch/runtime/platform/runtime.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "ai_processing.h"
#include "common_util.h"
#include "imagenet_labels.h"
#include "model_pte.h"

LOG_MODULE_REGISTER(ai_processing, CONFIG_LOG_DEFAULT_LEVEL);

#if defined(ET_ARM_MODEL_PTE_DMA_ACCESSIBLE)
static const unsigned char *model_pte_ptr = model_pte;
#else
alignas(16) static unsigned char model_pte_runtime[sizeof(model_pte)];
static bool model_pte_runtime_initialized = false;
#endif

using executorch::aten::ScalarType;
using executorch::aten::Tensor;
using executorch::aten::TensorImpl;
using executorch::extension::BufferDataLoader;
using executorch::runtime::Error;
using executorch::runtime::EValue;
using executorch::runtime::HierarchicalAllocator;
using executorch::runtime::MemoryAllocator;
using executorch::runtime::MemoryManager;
using executorch::runtime::Method;
using executorch::runtime::MethodMeta;
using executorch::runtime::Program;
using executorch::runtime::Result;
using executorch::runtime::Span;
using executorch::runtime::Tag;
using executorch::runtime::TensorInfo;

namespace
{

#if !defined(ET_ARM_METHOD_ALLOCATOR_POOL_SIZE)
#define ET_ARM_METHOD_ALLOCATOR_POOL_SIZE (1572864)
#endif
const size_t method_allocation_pool_size = ET_ARM_METHOD_ALLOCATOR_POOL_SIZE;
unsigned char __attribute__((section(".bss.method_pool"),
			     aligned(16))) method_allocation_pool[method_allocation_pool_size];

#if !defined(ET_ARM_BAREMETAL_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE)
#define ET_ARM_BAREMETAL_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE (1572864)
#endif
const size_t temp_allocation_pool_size = ET_ARM_BAREMETAL_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE;
unsigned char __attribute__((section(".bss.tensor_arena"),
			     aligned(16))) temp_allocation_pool[temp_allocation_pool_size];

#if !defined(ET_ARM_BAREMETAL_FAST_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE)
#define ET_ARM_BAREMETAL_FAST_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE 0x600
#endif
extern "C" {
size_t ethosu_fast_scratch_size = ET_ARM_BAREMETAL_FAST_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE;
unsigned char __attribute__((
	section(".bss.ethosu_scratch"),
	aligned(16))) dedicated_sram[ET_ARM_BAREMETAL_FAST_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE];
unsigned char *ethosu_fast_scratch = dedicated_sram;
}

ArmMemoryAllocator method_allocator(method_allocation_pool_size, method_allocation_pool);
ArmMemoryAllocator temp_allocator(temp_allocation_pool_size, temp_allocation_pool);

BufferDataLoader *g_loader = nullptr;
Result<Program> *g_program = nullptr;
Result<Method> *g_method = nullptr;
HierarchicalAllocator *g_planned_memory = nullptr;
MemoryManager *g_memory_manager = nullptr;

std::vector<uint8_t *> g_planned_buffers;
std::vector<Span<uint8_t>> g_planned_spans;
std::vector<EValue> g_outputs;

int8_t *g_input_data_ptr = nullptr;
size_t g_input_num_elements = 0;

#define INPUT_QUANT_MIN (-128)
#define INPUT_QUANT_MAX (127)

const float INPUT_QUANT_SCALE = strtof(CONFIG_AI_INPUT_QUANT_SCALE, nullptr);
constexpr int32_t INPUT_QUANT_ZERO_POINT = CONFIG_AI_INPUT_QUANT_ZERO_POINT;

const float OUTPUT_QUANT_SCALE = strtof(CONFIG_AI_OUTPUT_QUANT_SCALE, nullptr);
constexpr int32_t OUTPUT_QUANT_ZERO_POINT = CONFIG_AI_OUTPUT_QUANT_ZERO_POINT;

int8_t g_input_quant_lut[256];

void init_input_quant_lut()
{
	for (int v = 0; v < 256; v++) {
		float x = (static_cast<float>(v) - 128.0f) / 128.0f;
		float q = std::round(x / INPUT_QUANT_SCALE) + INPUT_QUANT_ZERO_POINT;
		q = std::max(static_cast<float>(INPUT_QUANT_MIN),
			     std::min(static_cast<float>(INPUT_QUANT_MAX), q));
		g_input_quant_lut[v] = static_cast<int8_t>(q);
	}
}

#define AI_TOP_K 5

st_ai_classification_point_t g_top_k_results[AI_TOP_K];

Error prepare_input_tensor(Method &method, MemoryAllocator &allocator, void **out_data_ptr,
			   size_t *out_nbytes)
{
	MethodMeta method_meta = method.method_meta();
	size_t num_inputs = method_meta.num_inputs();
	size_t tensor_inputs_seen = 0;

	for (size_t i = 0; i < num_inputs; i++) {
		auto tag = method_meta.input_tag(i);
		if (!tag.ok()) {
			return tag.error();
		}

		if (tag.get() != Tag::Tensor) {
			ET_LOG(Debug, "Skipping non-tensor input %zu", i);
			continue;
		}
		if (tensor_inputs_seen > 0) {
			ET_LOG(Error,
			       "Model has multiple tensor inputs; this sample supports only one");
			return Error::InvalidArgument;
		}
		tensor_inputs_seen++;

		Result<TensorInfo> tensor_meta = method_meta.input_tensor_meta(i);
		if (!tensor_meta.ok()) {
			return tensor_meta.error();
		}

		void *data_ptr = allocator.allocate(tensor_meta->nbytes());
		if (data_ptr == nullptr) {
			ET_LOG(Error, "Could not allocate memory for input buffer");
			return Error::MemoryAllocationFailed;
		}

		size_t num_elements = 1;
		auto sizes = tensor_meta->sizes();
		for (size_t k = 0; k < sizes.size(); k++) {
			num_elements *= sizes[k];
		}

		ET_LOG(Info, "Input tensor: scalar_type=%s, numel=%lu, nbytes=%lu",
		       executorch::runtime::toString(tensor_meta->scalar_type()),
		       static_cast<unsigned long>(num_elements),
		       static_cast<unsigned long>(tensor_meta->nbytes()));

		*out_data_ptr = data_ptr;
		*out_nbytes = tensor_meta->nbytes();
	}
	return Error::Ok;
}

Error rebind_input_tensor(Method &method, void *data_ptr)
{
	MethodMeta method_meta = method.method_meta();
	Result<TensorInfo> tensor_meta = method_meta.input_tensor_meta(0);
	if (!tensor_meta.ok()) {
		return tensor_meta.error();
	}

	TensorImpl impl = TensorImpl(
		tensor_meta.get().scalar_type(), tensor_meta.get().sizes().size(),
		const_cast<TensorImpl::SizesType *>(tensor_meta.get().sizes().data()), data_ptr,
		const_cast<TensorImpl::DimOrderType *>(tensor_meta.get().dim_order().data()));
	Tensor t(&impl);

	return method.set_input(t, 0);
}

void get_top_k(const std::vector<EValue> &outputs)
{
	if (outputs.empty() || !outputs[0].isTensor()) {
		ET_LOG(Error, "Output is not a tensor");
		return;
	}

	Tensor output_tensor = outputs[0].toTensor();
	ScalarType scalar_type = output_tensor.scalar_type();
	size_t num_classes = output_tensor.numel();

	int top_indices[AI_TOP_K] = {0};
	float top_values[AI_TOP_K];

	for (int j = 0; j < AI_TOP_K; j++) {
		top_values[j] = -1e9f;
	}

	float max_val = 0.0f;

	for (size_t i = 0; i < num_classes; i++) {
		float val;
		switch (scalar_type) {
		case ScalarType::Float:
			val = output_tensor.const_data_ptr<float>()[i];
			break;
		case ScalarType::Int:
			val = static_cast<float>(output_tensor.const_data_ptr<int>()[i]);
			break;
		case ScalarType::Char:
			val = (static_cast<float>(output_tensor.const_data_ptr<int8_t>()[i]) -
			       OUTPUT_QUANT_ZERO_POINT) *
			      OUTPUT_QUANT_SCALE;
			break;
		case ScalarType::Byte:
			val = (static_cast<float>(output_tensor.const_data_ptr<uint8_t>()[i]) -
			       OUTPUT_QUANT_ZERO_POINT) *
			      OUTPUT_QUANT_SCALE;
			break;
		default:
			ET_LOG(Error, "Unsupported output scalar type: %s",
			       executorch::runtime::toString(scalar_type));
			return;
		}

		if (i == 0 || val > max_val) {
			max_val = val;
		}

		for (int j = 0; j < AI_TOP_K; j++) {
			if (val > top_values[j]) {
				for (int m = AI_TOP_K - 1; m > j; m--) {
					top_values[m] = top_values[m - 1];
					top_indices[m] = top_indices[m - 1];
				}
				top_values[j] = val;
				top_indices[j] = static_cast<int>(i);
				break;
			}
		}
	}

	double sum_exp = 0.0;
	for (size_t i = 0; i < num_classes; i++) {
		float val;
		switch (scalar_type) {
		case ScalarType::Float:
			val = output_tensor.const_data_ptr<float>()[i];
			break;
		case ScalarType::Int:
			val = static_cast<float>(output_tensor.const_data_ptr<int>()[i]);
			break;
		case ScalarType::Char:
			val = (static_cast<float>(output_tensor.const_data_ptr<int8_t>()[i]) -
			       OUTPUT_QUANT_ZERO_POINT) *
			      OUTPUT_QUANT_SCALE;
			break;
		case ScalarType::Byte:
			val = (static_cast<float>(output_tensor.const_data_ptr<uint8_t>()[i]) -
			       OUTPUT_QUANT_ZERO_POINT) *
			      OUTPUT_QUANT_SCALE;
			break;
		default:
			val = 0.0f;
			break;
		}
		sum_exp += std::exp(static_cast<double>(val - max_val));
	}

	double top_prob[AI_TOP_K];
	double top_prob_sum = 1e-6;
	for (int j = 0; j < AI_TOP_K; j++) {
		top_prob[j] = std::exp(static_cast<double>(top_values[j] - max_val)) / sum_exp;
		top_prob_sum += top_prob[j];
	}

	if (IS_ENABLED(CONFIG_APP_RUNTIME_LOG)) {
		ET_LOG(Info, "\nTop-%d predictions:", AI_TOP_K);
	}
	for (int j = 0; j < AI_TOP_K; j++) {
		int idx = top_indices[j];
		const char *label =
			(idx >= 0 && idx < IMAGENET_NUM_CLASSES) ? imagenet_labels[idx] : "?";
		double probability = top_prob[j] / top_prob_sum;

		g_top_k_results[j].label_idx = static_cast<uint32_t>(idx);
		g_top_k_results[j].label = label;
		g_top_k_results[j].probability = static_cast<float>(probability);

		if (!IS_ENABLED(CONFIG_APP_RUNTIME_LOG)) {
			continue;
		}
#if defined(CONFIG_AI_OUTPUT_SCORE_AS_PERCENT)
		ET_LOG(Info, "  [%d] class %d (%s): %.2f%%", j + 1, idx, label,
		       probability * 100.0);
#else
		ET_LOG(Info, "  [%d] class %d (%s): %.4f", j + 1, idx, label,
		       static_cast<double>(top_values[j]));
#endif
	}
}

} // namespace

extern "C" int ai_init(void)
{
	executorch::runtime::runtime_init();
	init_input_quant_lut();

	size_t pte_size = sizeof(model_pte);
	ET_LOG(Info, "Model PTE at %p, Size: %lu bytes", model_pte,
	       static_cast<unsigned long>(pte_size));

#if defined(ET_ARM_MODEL_PTE_DMA_ACCESSIBLE)
	const void *program_data = model_pte_ptr;
#else
	if (!model_pte_runtime_initialized) {
		std::memcpy(model_pte_runtime, model_pte, sizeof(model_pte));
		model_pte_runtime_initialized = true;
	}
	const void *program_data = model_pte_runtime;
#endif
	size_t program_data_len = pte_size;
	g_loader = new BufferDataLoader(program_data, program_data_len);

	ET_LOG(Info, "Model data loaded. Size: %lu bytes.",
	       static_cast<unsigned long>(program_data_len));

	g_program = new Result<Program>(Program::load(g_loader));

	if (!(*g_program).ok()) {
		ET_LOG(Error, "Program loading failed @ 0x%p: 0x%" PRIx32, program_data,
		       static_cast<uint32_t>((*g_program).error()));
		return 1;
	}

	ET_LOG(Info, "Model loaded, has %lu methods",
	       static_cast<unsigned long>((*g_program)->num_methods()));

	const char *method_name = nullptr;
	{
		const auto method_name_result = (*g_program)->get_method_name(0);
		ET_CHECK_MSG(method_name_result.ok(), "Program has no methods");
		method_name = *method_name_result;
	}
	ET_LOG(Info, "Running method: %s", method_name);

	Result<MethodMeta> method_meta = (*g_program)->method_meta(method_name);
	if (!method_meta.ok()) {
		ET_LOG(Error, "Failed to get method_meta for %s: 0x%x", method_name,
		       (unsigned int)method_meta.error());
		return 1;
	}

	ET_LOG(Info, "Method allocator pool size: %lu bytes.",
	       static_cast<unsigned long>(method_allocation_pool_size));

	size_t num_memory_planned_buffers = method_meta->num_memory_planned_buffers();

	for (size_t id = 0; id < num_memory_planned_buffers; ++id) {
		Result<int64_t> buffer_size_result = method_meta->memory_planned_buffer_size(id);
		if (!buffer_size_result.ok()) {
			ET_LOG(Error, "Failed to get planned buffer size for buffer %lu: 0x%x",
			       static_cast<unsigned long>(id),
			       (unsigned int)buffer_size_result.error());
			return 1;
		}
		size_t buffer_size = static_cast<size_t>(buffer_size_result.get());
		ET_LOG(Info, "Setting up planned buffer %lu, size %lu.",
		       static_cast<unsigned long>(id), static_cast<unsigned long>(buffer_size));

		uint8_t *buffer =
			reinterpret_cast<uint8_t *>(method_allocator.allocate(buffer_size));
		ET_CHECK_MSG(buffer != nullptr, "Could not allocate planned buffer size %lu",
			     static_cast<unsigned long>(buffer_size));
		g_planned_buffers.push_back(buffer);
		g_planned_spans.push_back({g_planned_buffers.back(), buffer_size});
	}

	g_planned_memory =
		new HierarchicalAllocator({g_planned_spans.data(), g_planned_spans.size()});

	g_memory_manager = new MemoryManager(&method_allocator, g_planned_memory, &temp_allocator);

	ET_LOG(Info, "Loading method...");
	executorch::runtime::EventTracer *event_tracer_ptr = nullptr;

	g_method = new Result<Method>(
		(*g_program)->load_method(method_name, g_memory_manager, event_tracer_ptr));

	if (!(*g_method).ok()) {
		ET_LOG(Error, "Loading of method %s failed with status 0x%" PRIx32, method_name,
		       static_cast<uint32_t>((*g_method).error()));
		return 1;
	}
	ET_LOG(Info, "Method '%s' loaded successfully", method_name);

	{
		void *input_data_ptr = nullptr;
		size_t input_nbytes = 0;
		Error input_err = ::prepare_input_tensor(**g_method, method_allocator,
							 &input_data_ptr, &input_nbytes);

		if (input_err != Error::Ok) {
			ET_LOG(Error, "Preparing input failed: 0x%" PRIx32,
			       static_cast<uint32_t>(input_err));
			return 1;
		}

		g_input_data_ptr = static_cast<int8_t *>(input_data_ptr);
		g_input_num_elements = input_nbytes;
	}

	g_outputs.resize((*g_method)->outputs_size());

	ET_LOG(Info, "Model size: %lu bytes", static_cast<unsigned long>(pte_size));
	ET_LOG(Info, "Input tensor: %lu bytes", static_cast<unsigned long>(g_input_num_elements));
	return 0;
}

void ai_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);

	struct k_msgq *ai_input_msgq = (struct k_msgq *)arg1;
	struct k_msgq *ai_result_msgq = (struct k_msgq *)arg2;
	ai_input_msg_t ai_input_msg;

	for (;;) {
		if (k_msgq_get(ai_input_msgq, &ai_input_msg, K_FOREVER) != 0) {
			continue;
		}

		if (ai_input_msg.size != g_input_num_elements) {
			ET_LOG(Error, "Input size mismatch: got %zu, expected %zu",
			       ai_input_msg.size, g_input_num_elements);
			continue;
		}

		for (size_t j = 0; j < ai_input_msg.size; j++) {
			g_input_data_ptr[j] = g_input_quant_lut[ai_input_msg.data[j]];
		}

		/* ai_input_msg.data's bytes are fully copied out now, so camera.c's
		 * single-slot model_buffer_rgb is safe for camera_task to reuse. */
		k_sem_give(&ai_buffer_free_sem);

		Error rebind_err = rebind_input_tensor(**g_method, g_input_data_ptr);
		if (rebind_err != Error::Ok) {
			ET_LOG(Error, "Failed to rebind input tensor: 0x%" PRIx32,
			       static_cast<uint32_t>(rebind_err));
			continue;
		}

		uint32_t inference_start_cycles = k_cycle_get_32();
		Error status = (*g_method)->execute();
		uint32_t inference_us =
			k_cyc_to_us_floor32(k_cycle_get_32() - inference_start_cycles);
		if (status != Error::Ok) {
			ET_LOG(Error, "Execution failed: 0x%" PRIx32,
			       static_cast<uint32_t>(status));
			continue;
		}

		status = (*g_method)->get_outputs(g_outputs.data(), g_outputs.size());
		if (status != Error::Ok) {
			ET_LOG(Error, "get_outputs failed: 0x%" PRIx32,
			       static_cast<uint32_t>(status));
			continue;
		}

		if (IS_ENABLED(CONFIG_APP_RUNTIME_LOG)) {
			ET_LOG(Info, "\n--- Loop inference ---");
		}
		get_top_k(g_outputs);

		if (IS_ENABLED(CONFIG_APP_RUNTIME_LOG)) {
			ET_LOG(Info, "Inference time: %u us (%u ms)", inference_us,
			       inference_us / 1000);
		}

		ai_result_msg_t ai_result;
		ai_result.results = g_top_k_results;
		ai_result.inference_time_ms = inference_us / 1000;
		ai_result.result_count = AI_TOP_K;
		k_msgq_put(ai_result_msgq, &ai_result, K_NO_WAIT);
	}
}
