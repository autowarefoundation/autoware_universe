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

#include "autoware/tensorrt_plugins/unique_plugin.hpp"

#include "autoware/tensorrt_plugins/plugin_utils.hpp"
#include "autoware/unique_ops/unique.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>

#include <cstdint>
#include <exception>
#include <string>
#include <vector>

namespace nvinfer1::plugin
{

UniquePlugin::UniquePlugin(const std::string & name) noexcept : layer_name_(name)
{
  initFieldsToSerialize();
}

void UniquePlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

IPluginCapability * UniquePlugin::getCapabilityInterface(PluginCapabilityType type) noexcept
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

IPluginV3 * UniquePlugin::clone() noexcept
{
  try {
    IPluginV3 * const plugin{new UniquePlugin{layer_name_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * UniquePlugin::getPluginName() const noexcept
{
  return kUNIQUE_PLUGIN_NAME;
}

char const * UniquePlugin::getPluginVersion() const noexcept
{
  return kUNIQUE_PLUGIN_VERSION;
}

char const * UniquePlugin::getPluginNamespace() const noexcept
{
  return kUNIQUE_PLUGIN_NAMESPACE;
}

std::int32_t UniquePlugin::getNbOutputs() const noexcept
{
  return 4;
}

std::int32_t UniquePlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  // Validate input arguments.
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 4);
  PLUGIN_ASSERT(in[0].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(out[1].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(out[2].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(out[3].desc.dims.nbDims == 0);

  PLUGIN_ASSERT(out[0].desc.type == in[0].desc.type);
  PLUGIN_ASSERT(out[1].desc.type == in[0].desc.type);
  PLUGIN_ASSERT(out[2].desc.type == in[0].desc.type);
  PLUGIN_ASSERT(out[3].desc.type == in[0].desc.type);

  return 0;
}

bool UniquePlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 4);

  return (
    in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR &&
    in_out[pos].desc.type == nvinfer1::DataType::kINT64);
}

std::int32_t UniquePlugin::getOutputDataTypes(
  DataType * output_types, std::int32_t num_outputs, DataType const * input_types,
  std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 4);

  output_types[0] = input_types[0];
  output_types[1] = input_types[0];
  output_types[2] = input_types[0];
  output_types[3] = input_types[0];

  return 0;
}

std::int32_t UniquePlugin::getOutputShapes(
  DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, std::int32_t num_outputs, IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 4);
  PLUGIN_ASSERT(inputs[0].nbDims == 1);

  outputs[0].nbDims = 1;
  outputs[0].d[0] = expr_builder.declareSizeTensor(3, *inputs[0].d[0], *inputs[0].d[0]);

  outputs[1] = inputs[0];

  outputs[2].nbDims = 1;
  outputs[2].d[0] = expr_builder.declareSizeTensor(3, *inputs[0].d[0], *inputs[0].d[0]);

  outputs[3].nbDims = 0;

  return 0;
}

std::int32_t UniquePlugin::enqueue(
  PluginTensorDesc const * input_desc, [[maybe_unused]] PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  std::int64_t num_elements = input_desc[0].dims.d[0];

  std::int64_t num_unique_elements = unique(
    reinterpret_cast<const std::int64_t *>(inputs[0]), reinterpret_cast<std::int64_t *>(outputs[0]),
    reinterpret_cast<std::int64_t *>(outputs[1]), reinterpret_cast<std::int64_t *>(outputs[2]),
    workspace, num_elements, workspace_size_, stream);

  cudaMemcpyAsync(
    reinterpret_cast<std::int64_t *>(outputs[3]), &num_unique_elements, sizeof(std::int64_t),
    cudaMemcpyHostToDevice, stream);

  cudaStreamSynchronize(stream);

  return 0;
}

std::int32_t UniquePlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

IPluginV3 * UniquePlugin::attachToContext(
  [[maybe_unused]] IPluginResourceContext * context) noexcept
{
  return clone();
}

PluginFieldCollection const * UniquePlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t UniquePlugin::getWorkspaceSize(
  [[maybe_unused]] DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return get_unique_workspace_size(inputs[0].max.d[0]);
}

}  // namespace nvinfer1::plugin
