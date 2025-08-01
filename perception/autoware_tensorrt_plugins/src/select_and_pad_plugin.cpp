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

#include "autoware/tensorrt_plugins/select_and_pad_plugin.hpp"

#include "autoware/select_and_pad_ops/select_and_pad_kernel.h"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

// Forward declaration of the select_and_pad_launch function
template <typename T>
void select_and_pad_launch(
  T * feat, int * flags, T * invalid, int B, int Q, int C, int P, T * out, void * tmp,
  size_t tmp_bytes, cudaStream_t stream);

namespace autoware::tensorrt_plugins
{

SelectAndPadPlugin::SelectAndPadPlugin(const std::string & name) noexcept : layer_name_{name}
{
  initFieldsToSerialize();
}

SelectAndPadPlugin::SelectAndPadPlugin(const std::string & name, int P) noexcept
: layer_name_{name}, P_{P}
{
  initFieldsToSerialize();
}

void SelectAndPadPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back("P", &P_, nvinfer1::PluginFieldType::kINT32, 1);
  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

nvinfer1::IPluginCapability * SelectAndPadPlugin::getCapabilityInterface(
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

nvinfer1::IPluginV3 * SelectAndPadPlugin::clone() noexcept
{
  try {
    nvinfer1::IPluginV3 * const plugin{new SelectAndPadPlugin{layer_name_, P_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * SelectAndPadPlugin::getPluginName() const noexcept
{
  return kSELECT_AND_PAD_PLUGIN_NAME;
}

char const * SelectAndPadPlugin::getPluginVersion() const noexcept
{
  return kSELECT_AND_PAD_PLUGIN_VERSION;
}

char const * SelectAndPadPlugin::getPluginNamespace() const noexcept
{
  return kSELECT_AND_PAD_PLUGIN_NAMESPACE;
}

std::int32_t SelectAndPadPlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t SelectAndPadPlugin::configurePlugin(
  nvinfer1::DynamicPluginTensorDesc const * in, std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * out, std::int32_t num_outputs) noexcept
{
  // Validate input arguments
  PLUGIN_ASSERT(num_inputs == 3);  // feat, flags, invalid
  PLUGIN_ASSERT(num_outputs == 1);

  // Cache Q value from input shape
  Q_ = in[1].desc.dims.d[0];

  // Calculate temporary storage size
  tmp_bytes_ = decideTemp();

  return 0;
}

bool SelectAndPadPlugin::supportsFormatCombination(
  std::int32_t pos, nvinfer1::DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 3);
  PLUGIN_ASSERT(num_outputs == 1);

  if (in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR) {
    if (pos == 1) {
      // flags should be INT32
      return (in_out[pos].desc.type == nvinfer1::DataType::kINT32);
    } else {
      // feat, invalid, and output should match and be FLOAT or HALF
      return (
        (in_out[pos].desc.type == in_out[0].desc.type) &&
        ((in_out[pos].desc.type == nvinfer1::DataType::kFLOAT) ||
         (in_out[pos].desc.type == nvinfer1::DataType::kHALF)));
    }
  }
  return false;
}

std::int32_t SelectAndPadPlugin::getOutputDataTypes(
  nvinfer1::DataType * output_types, std::int32_t num_outputs,
  nvinfer1::DataType const * input_types, std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 3);
  PLUGIN_ASSERT(num_outputs == 1);

  output_types[0] = input_types[0];

  return 0;
}

std::int32_t SelectAndPadPlugin::getOutputShapes(
  nvinfer1::DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DimsExprs const * shape_inputs,
  [[maybe_unused]] std::int32_t num_shape_inputs, nvinfer1::DimsExprs * outputs,
  std::int32_t num_outputs, nvinfer1::IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 3);
  PLUGIN_ASSERT(num_outputs == 1);

  // Output shape: [B, P, C]
  outputs[0].nbDims = 3;
  outputs[0].d[0] = inputs[0].d[0];             // B
  outputs[0].d[1] = expr_builder.constant(P_);  // P
  outputs[0].d[2] = inputs[0].d[2];             // C

  return 0;
}

std::int32_t SelectAndPadPlugin::enqueue(
  nvinfer1::PluginTensorDesc const * input_desc,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * output_desc, void const * const * inputs,
  void * const * outputs, void * workspace, cudaStream_t stream) noexcept
{
  nvinfer1::Dims feat_dims = input_desc[0].dims;  // B, Q, C
  int B = feat_dims.d[0];
  int Q = feat_dims.d[1];
  int C = feat_dims.d[2];

  auto data_type = input_desc[0].type;
  if (data_type == nvinfer1::DataType::kFLOAT) {
    select_and_pad_launch<float>(
      const_cast<float *>(reinterpret_cast<const float *>(inputs[0])),
      const_cast<int *>(reinterpret_cast<const int *>(inputs[1])),
      const_cast<float *>(reinterpret_cast<const float *>(inputs[2])), B, Q, C, P_,
      reinterpret_cast<float *>(outputs[0]), workspace, tmp_bytes_, stream);
  } else if (data_type == nvinfer1::DataType::kHALF) {
    select_and_pad_launch<__half>(
      const_cast<__half *>(reinterpret_cast<const __half *>(inputs[0])),
      const_cast<int *>(reinterpret_cast<const int *>(inputs[1])),
      const_cast<__half *>(reinterpret_cast<const __half *>(inputs[2])), B, Q, C, P_,
      reinterpret_cast<__half *>(outputs[0]), workspace, tmp_bytes_, stream);
  }

  return 0;
}

std::int32_t SelectAndPadPlugin::onShapeChange(
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * out,
  [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

nvinfer1::IPluginV3 * SelectAndPadPlugin::attachToContext(
  [[maybe_unused]] nvinfer1::IPluginResourceContext * context) noexcept
{
  return clone();
}

nvinfer1::PluginFieldCollection const * SelectAndPadPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t SelectAndPadPlugin::getWorkspaceSize(
  nvinfer1::DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  int max_Q = inputs[1].max.d[0];

  // Estimate CUB temp storage size
  std::size_t _temp_storage_bytes = 767 + max_Q * sizeof(int);

  // Total workspace: buf + indices + cub_tmp + d_num
  return sizeof(int) * max_Q * 2 + _temp_storage_bytes + sizeof(int);
}

std::size_t SelectAndPadPlugin::decideTemp()
{
  // Estimate CUB temp storage size based on Q
  // This is a conservative estimate for DeviceSelect::Flagged
  std::size_t _temp_storage_bytes = 767 + Q_ * sizeof(int);
  return _temp_storage_bytes;
}

}  // namespace autoware::tensorrt_plugins
