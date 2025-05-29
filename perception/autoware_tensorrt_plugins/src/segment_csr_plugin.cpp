// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/tensorrt_plugins/segment_csr_plugin.hpp"

#include "autoware/scatter_ops/reduction.h"
#include "autoware/scatter_ops/segment_csr.h"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <cstdint>
#include <exception>
#include <string>
#include <tuple>
#include <vector>
namespace nvinfer1::plugin
{

SegmentCSRPlugin::SegmentCSRPlugin(const std::string & name, const std::string & reduce)
: layer_name_{name}, reduce_{reduce}
{
  initFieldsToSerialize();
}

void SegmentCSRPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back(
    "reduce", reduce_.c_str(), PluginFieldType::kCHAR, reduce_.size());

  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

IPluginCapability * SegmentCSRPlugin::getCapabilityInterface(PluginCapabilityType type) noexcept
{
  try {
    if (type == PluginCapabilityType::kBUILD) {
      return static_cast<IPluginV3OneBuild *>(this);
    }
    if (type == PluginCapabilityType::kRUNTIME) {
      return static_cast<IPluginV3OneRuntime *>(this);
    }
    PLUGIN_ASSERT(type == PluginCapabilityType::kCORE);
    return static_cast<IPluginV3OneCore *>(this);
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

IPluginV3 * SegmentCSRPlugin::clone() noexcept
{
  try {
    IPluginV3 * const plugin{new SegmentCSRPlugin{layer_name_, reduce_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * SegmentCSRPlugin::getPluginName() const noexcept
{
  return kSEGMENT_CSR_PLUGIN_NAME;
}

char const * SegmentCSRPlugin::getPluginVersion() const noexcept
{
  return kSEGMENT_CSR_PLUGIN_VERSION;
}

char const * SegmentCSRPlugin::getPluginNamespace() const noexcept
{
  return kSEGMENT_CSR_PLUGIN_NAMESPACE;
}

std::int32_t SegmentCSRPlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t SegmentCSRPlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  // Validate input arguments.
  PLUGIN_ASSERT(num_inputs == 2);
  PLUGIN_ASSERT(num_outputs == 1);

  PLUGIN_ASSERT(in[INOUT_IN_SRC_INDEX].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(in[INOUT_IN_INDPTR_INDEX].desc.dims.nbDims == 1);

  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 2);

  PLUGIN_ASSERT(in[INOUT_IN_SRC_INDEX].desc.type == out[0].desc.type);

  return 0;
}

bool SegmentCSRPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 2);
  PLUGIN_ASSERT(num_outputs == 1);

  bool supported = in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR;

  switch (pos) {
    case INOUT_IN_SRC_INDEX:
      supported &=
        (in_out[pos].desc.type == nvinfer1::DataType::kFLOAT ||
         in_out[pos].desc.type == nvinfer1::DataType::kHALF);
      break;
    case INOUT_IN_INDPTR_INDEX:
      supported &= in_out[pos].desc.type == nvinfer1::DataType::kINT64;
      break;
    case INOUT_OUT_INDEX:
      supported &= in_out[pos].desc.type == in_out[INOUT_IN_SRC_INDEX].desc.type;
      break;
    default:
      supported = false;
      break;
  }

  return supported;
}

std::int32_t SegmentCSRPlugin::getOutputDataTypes(
  DataType * output_types, std::int32_t num_outputs, DataType const * input_types,
  std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 2);
  PLUGIN_ASSERT(num_outputs == 1);

  output_types[0] = input_types[0];

  return 0;
}

std::int32_t SegmentCSRPlugin::getOutputShapes(
  DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, std::int32_t num_outputs,
  [[maybe_unused]] IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 2);
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(inputs[0].nbDims == 2);
  PLUGIN_ASSERT(inputs[1].nbDims == 1);

  outputs[0].nbDims = 2;
  outputs[0].d[0] =
    expr_builder.operation(DimensionOperation::kSUB, *inputs[1].d[0], *expr_builder.constant(1));
  outputs[0].d[1] = inputs[0].d[1];

  return 0;
}

std::int32_t SegmentCSRPlugin::enqueue(
  PluginTensorDesc const * input_desc, [[maybe_unused]] PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  std::vector<int32_t> src_size{
    static_cast<int32_t>(input_desc[0].dims.d[0]), static_cast<int32_t>(input_desc[0].dims.d[1])};
  std::vector<int32_t> indptr_size{static_cast<int32_t>(input_desc[1].dims.d[0])};

  std::int32_t result = 0;

  if (input_desc[0].type == nvinfer1::DataType::kFLOAT) {
    const float * src_ptr = reinterpret_cast<const float *>(inputs[0]);
    const std::int64_t * indptr_ptr = reinterpret_cast<const std::int64_t *>(inputs[1]);

    std::tuple<float *, std::int64_t *> out =
      std::make_tuple(static_cast<float *>(outputs[0]), nullptr);

    AT_DISPATCH_REDUCTION_TYPES(reduce_, [&] {
      result =
        segment_csr_launch<float, REDUCE>(src_ptr, src_size, indptr_ptr, indptr_size, out, stream);
    });
  } else if (input_desc[0].type == nvinfer1::DataType::kHALF) {
    const half * src_ptr = reinterpret_cast<const half *>(inputs[0]);
    const std::int64_t * indptr_ptr = reinterpret_cast<const std::int64_t *>(inputs[1]);

    std::tuple<half *, std::int64_t *> out =
      std::make_tuple(static_cast<half *>(outputs[0]), nullptr);

    AT_DISPATCH_REDUCTION_TYPES(reduce_, [&] {
      result =
        segment_csr_launch<half, REDUCE>(src_ptr, src_size, indptr_ptr, indptr_size, out, stream);
    });
  }

  return result;
}

std::int32_t SegmentCSRPlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

IPluginV3 * SegmentCSRPlugin::attachToContext(
  [[maybe_unused]] IPluginResourceContext * context) noexcept
{
  return clone();
}

PluginFieldCollection const * SegmentCSRPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t SegmentCSRPlugin::getWorkspaceSize(
  [[maybe_unused]] DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return 0;
}

}  // namespace nvinfer1::plugin
