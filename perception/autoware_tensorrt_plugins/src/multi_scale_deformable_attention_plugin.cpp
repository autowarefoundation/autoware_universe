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

#include "autoware/tensorrt_plugins/multi_scale_deformable_attention_plugin.hpp"

#include "autoware/multi_scale_deform_attn_ops/ms_deform_attn_kernel.hpp"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_fp16.h>

#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

namespace autoware::tensorrt_plugins
{

MultiScaleDeformableAttentionPlugin::MultiScaleDeformableAttentionPlugin(
  const std::string & name) noexcept
: layer_name_{name}
{
  initFieldsToSerialize();
}

void MultiScaleDeformableAttentionPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

nvinfer1::IPluginCapability * MultiScaleDeformableAttentionPlugin::getCapabilityInterface(
  nvinfer1::PluginCapabilityType type) noexcept
{
  try {
    if (type == nvinfer1::PluginCapabilityType::kBUILD) {
      return static_cast<nvinfer1::IPluginV3OneBuild *>(this);
    }
    if (type == nvinfer1::PluginCapabilityType::kRUNTIME) {
      return static_cast<nvinfer1::IPluginV3OneRuntime *>(this);
    }
    PLUGIN_ASSERT(type == nvinfer1::PluginCapabilityType::kCORE);
    return static_cast<nvinfer1::IPluginV3OneCore *>(this);
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

nvinfer1::IPluginV3 * MultiScaleDeformableAttentionPlugin::clone() noexcept
{
  try {
    nvinfer1::IPluginV3 * const plugin{new MultiScaleDeformableAttentionPlugin{layer_name_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * MultiScaleDeformableAttentionPlugin::getPluginName() const noexcept
{
  return kMULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_NAME;
}

char const * MultiScaleDeformableAttentionPlugin::getPluginVersion() const noexcept
{
  return kMULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_VERSION;
}

char const * MultiScaleDeformableAttentionPlugin::getPluginNamespace() const noexcept
{
  return kMULTI_SCALE_DEFORMABLE_ATTENTION_PLUGIN_NAMESPACE;
}

std::int32_t MultiScaleDeformableAttentionPlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t MultiScaleDeformableAttentionPlugin::configurePlugin(
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * in, std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * out, std::int32_t num_outputs) noexcept
{
  // Validate input arguments
  PLUGIN_ASSERT(
    num_inputs == 5);  // value, spatial_shapes, level_start_index, sampling_loc, attn_weight
  PLUGIN_ASSERT(num_outputs == 1);

  return 0;
}

bool MultiScaleDeformableAttentionPlugin::supportsFormatCombination(
  std::int32_t pos, nvinfer1::DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 5);
  PLUGIN_ASSERT(num_outputs == 1);

  if (in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR) {
    if ((pos == 1) || (pos == 2)) {
      // spatial_shapes and level_start_index should be INT32
      return (in_out[pos].desc.type == nvinfer1::DataType::kINT32);
    } else {
      // Other tensors should be FLOAT or HALF and match input 0 type
      bool f1 = (in_out[pos].desc.type == nvinfer1::DataType::kFLOAT) ||
                (in_out[pos].desc.type == nvinfer1::DataType::kHALF);
      return ((in_out[pos].desc.type == in_out[0].desc.type) && f1);
    }
  }
  return false;
}

std::int32_t MultiScaleDeformableAttentionPlugin::getOutputDataTypes(
  nvinfer1::DataType * output_types, std::int32_t num_outputs,
  nvinfer1::DataType const * input_types, std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 5);
  PLUGIN_ASSERT(num_outputs == 1);

  output_types[0] = input_types[0];

  return 0;
}

std::int32_t MultiScaleDeformableAttentionPlugin::getOutputShapes(
  nvinfer1::DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DimsExprs const * shape_inputs,
  [[maybe_unused]] std::int32_t num_shape_inputs, nvinfer1::DimsExprs * outputs,
  std::int32_t num_outputs, nvinfer1::IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 5);
  PLUGIN_ASSERT(num_outputs == 1);

  outputs[0].nbDims = 3;
  outputs[0].d[0] = inputs[0].d[0];  // batch
  outputs[0].d[1] = inputs[3].d[1];  // num_query
  outputs[0].d[2] = expr_builder.operation(
    nvinfer1::DimensionOperation::kPROD, *inputs[0].d[2], *inputs[0].d[3]);  // num_heads * channels

  return 0;
}

std::int32_t MultiScaleDeformableAttentionPlugin::enqueue(
  nvinfer1::PluginTensorDesc const * input_desc,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * output_desc, void const * const * inputs,
  void * const * outputs, [[maybe_unused]] void * workspace, cudaStream_t stream) noexcept
{
  // Extract dimensions from inputs
  auto batch = input_desc[0].dims.d[0];
  auto spatial_size = input_desc[0].dims.d[1];
  auto num_heads = input_desc[0].dims.d[2];
  auto channels = input_desc[0].dims.d[3];

  auto num_levels = input_desc[1].dims.d[0];
  auto num_query = input_desc[3].dims.d[1];
  auto num_point = input_desc[3].dims.d[4];

  // Call the CUDA kernel
  if (input_desc[0].type == nvinfer1::DataType::kFLOAT) {
    ms_deform_attn_cuda_forward<float>(
      reinterpret_cast<const float *>(inputs[0]),    // value
      reinterpret_cast<const int32_t *>(inputs[1]),  // spatial_shapes
      reinterpret_cast<const int32_t *>(inputs[2]),  // level_start_index
      reinterpret_cast<const float *>(inputs[3]),    // sampling_loc
      reinterpret_cast<const float *>(inputs[4]),    // attn_weight
      reinterpret_cast<float *>(outputs[0]),         // output
      batch, spatial_size, num_heads, channels, num_levels, num_query, num_point, stream);
  } else if (input_desc[0].type == nvinfer1::DataType::kHALF) {
    ms_deform_attn_cuda_forward<__half>(
      reinterpret_cast<const __half *>(inputs[0]),   // value
      reinterpret_cast<const int32_t *>(inputs[1]),  // spatial_shapes
      reinterpret_cast<const int32_t *>(inputs[2]),  // level_start_index
      reinterpret_cast<const __half *>(inputs[3]),   // sampling_loc
      reinterpret_cast<const __half *>(inputs[4]),   // attn_weight
      reinterpret_cast<__half *>(outputs[0]),        // output
      batch, spatial_size, num_heads, channels, num_levels, num_query, num_point, stream);
  }

  return 0;
}

std::int32_t MultiScaleDeformableAttentionPlugin::onShapeChange(
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * out,
  [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

nvinfer1::IPluginV3 * MultiScaleDeformableAttentionPlugin::attachToContext(
  [[maybe_unused]] nvinfer1::IPluginResourceContext * context) noexcept
{
  return clone();
}

nvinfer1::PluginFieldCollection const *
MultiScaleDeformableAttentionPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t MultiScaleDeformableAttentionPlugin::getWorkspaceSize(
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * inputs,
  [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return 0;
}

}  // namespace autoware::tensorrt_plugins
