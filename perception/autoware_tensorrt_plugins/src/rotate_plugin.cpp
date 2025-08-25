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

#include "autoware/tensorrt_plugins/rotate_plugin.hpp"

#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_fp16.h>

#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

// Forward declaration of the rotate kernel function
template <typename T>
void rotate(
  T * output, T * input, T * angle, T * center, int * input_dims, RotateInterpolation interp,
  cudaStream_t stream);

namespace autoware::tensorrt_plugins
{

RotatePlugin::RotatePlugin(const std::string & name) noexcept : layer_name_{name}
{
  initFieldsToSerialize();
}

RotatePlugin::RotatePlugin(const std::string & name, RotateInterpolation mode) noexcept
: layer_name_{name}, mode_{mode}
{
  initFieldsToSerialize();
}

void RotatePlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back("interpolation", &mode_, nvinfer1::PluginFieldType::kINT32, 1);
  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

nvinfer1::IPluginCapability * RotatePlugin::getCapabilityInterface(
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

nvinfer1::IPluginV3 * RotatePlugin::clone() noexcept
{
  try {
    nvinfer1::IPluginV3 * const plugin{new RotatePlugin{layer_name_, mode_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * RotatePlugin::getPluginName() const noexcept
{
  return kROTATE_PLUGIN_NAME;
}

char const * RotatePlugin::getPluginVersion() const noexcept
{
  return kROTATE_PLUGIN_VERSION;
}

char const * RotatePlugin::getPluginNamespace() const noexcept
{
  return kROTATE_PLUGIN_NAMESPACE;
}

std::int32_t RotatePlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t RotatePlugin::configurePlugin(
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * in, std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * out, std::int32_t num_outputs) noexcept
{
  // Validate input arguments
  PLUGIN_ASSERT(num_inputs == 3);  // input, angle, center
  PLUGIN_ASSERT(num_outputs == 1);

  return 0;
}

bool RotatePlugin::supportsFormatCombination(
  std::int32_t pos, nvinfer1::DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 3);
  PLUGIN_ASSERT(num_outputs == 1);

  if (in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR) {
    if ((pos == 1) || (pos == 2)) {
      // angle and center can be FLOAT or HALF
      return (in_out[pos].desc.type == nvinfer1::DataType::kFLOAT) ||
             (in_out[pos].desc.type == nvinfer1::DataType::kHALF);
    } else {
      // input and output should match and be FLOAT or HALF
      return (
        (in_out[pos].desc.type == in_out[0].desc.type) &&
        ((in_out[pos].desc.type == nvinfer1::DataType::kFLOAT) ||
         (in_out[pos].desc.type == nvinfer1::DataType::kHALF)));
    }
  }
  return false;
}

std::int32_t RotatePlugin::getOutputDataTypes(
  nvinfer1::DataType * output_types, std::int32_t num_outputs,
  nvinfer1::DataType const * input_types, std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 3);
  PLUGIN_ASSERT(num_outputs == 1);

  output_types[0] = input_types[0];

  return 0;
}

std::int32_t RotatePlugin::getOutputShapes(
  nvinfer1::DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DimsExprs const * shape_inputs,
  [[maybe_unused]] std::int32_t num_shape_inputs, nvinfer1::DimsExprs * outputs,
  std::int32_t num_outputs, [[maybe_unused]] nvinfer1::IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 3);
  PLUGIN_ASSERT(num_outputs == 1);

  // Output shape is same as input shape
  outputs[0] = inputs[0];

  return 0;
}

std::int32_t RotatePlugin::enqueue(
  nvinfer1::PluginTensorDesc const * input_desc,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * output_desc, void const * const * inputs,
  void * const * outputs, [[maybe_unused]] void * workspace, cudaStream_t stream) noexcept
{
  // Extract dimensions
  int input_dims[4];
  for (int i = 0; i < input_desc[0].dims.nbDims; ++i) {
    input_dims[i] = input_desc[0].dims.d[i];
  }

  // Call the appropriate kernel based on data type
  if (input_desc[0].type == nvinfer1::DataType::kFLOAT) {
    rotate<float>(
      reinterpret_cast<float *>(outputs[0]),
      const_cast<float *>(reinterpret_cast<const float *>(inputs[0])),
      const_cast<float *>(reinterpret_cast<const float *>(inputs[1])),
      const_cast<float *>(reinterpret_cast<const float *>(inputs[2])), input_dims, mode_, stream);
  } else if (input_desc[0].type == nvinfer1::DataType::kHALF) {
    rotate<__half>(
      reinterpret_cast<__half *>(outputs[0]),
      const_cast<__half *>(reinterpret_cast<const __half *>(inputs[0])),
      const_cast<__half *>(reinterpret_cast<const __half *>(inputs[1])),
      const_cast<__half *>(reinterpret_cast<const __half *>(inputs[2])), input_dims, mode_, stream);
  }

  return 0;
}

std::int32_t RotatePlugin::onShapeChange(
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * out,
  [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

nvinfer1::IPluginV3 * RotatePlugin::attachToContext(
  [[maybe_unused]] nvinfer1::IPluginResourceContext * context) noexcept
{
  return clone();
}

nvinfer1::PluginFieldCollection const * RotatePlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t RotatePlugin::getWorkspaceSize(
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * inputs,
  [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return 0;
}

}  // namespace autoware::tensorrt_plugins
