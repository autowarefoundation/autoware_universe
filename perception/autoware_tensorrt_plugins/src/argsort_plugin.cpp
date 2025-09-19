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

#include "autoware/tensorrt_plugins/argsort_plugin.hpp"

#include "autoware/argsort_ops/argsort.hpp"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>

#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

namespace nvinfer1::plugin
{

ArgsortPlugin::ArgsortPlugin(const std::string & name) noexcept : layer_name_{name}
{
  initFieldsToSerialize();
}

void ArgsortPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

IPluginCapability * ArgsortPlugin::getCapabilityInterface(PluginCapabilityType type) noexcept
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

IPluginV3 * ArgsortPlugin::clone() noexcept
{
  try {
    IPluginV3 * const plugin{new ArgsortPlugin{layer_name_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * ArgsortPlugin::getPluginName() const noexcept
{
  return kARGSORT_PLUGIN_NAME;
}

char const * ArgsortPlugin::getPluginVersion() const noexcept
{
  return kARGSORT_PLUGIN_VERSION;
}

char const * ArgsortPlugin::getPluginNamespace() const noexcept
{
  return kARGSORT_PLUGIN_NAMESPACE;
}

std::int32_t ArgsortPlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t ArgsortPlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  // Validate input arguments.
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(in[0].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 1);

  PLUGIN_ASSERT(out[0].desc.type == in[0].desc.type);

  return 0;
}

bool ArgsortPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 1);

  return (
    in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR &&
    in_out[pos].desc.type == nvinfer1::DataType::kINT64);
}

std::int32_t ArgsortPlugin::getOutputDataTypes(
  DataType * output_types, std::int32_t num_outputs, DataType const * input_types,
  std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 1);

  output_types[0] = input_types[0];

  return 0;
}

std::int32_t ArgsortPlugin::getOutputShapes(
  DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, std::int32_t num_outputs,
  [[maybe_unused]] IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(inputs[0].nbDims == 1);

  outputs[0] = inputs[0];

  return 0;
}

std::int32_t ArgsortPlugin::enqueue(
  PluginTensorDesc const * input_desc, [[maybe_unused]] PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  auto num_elements = static_cast<std::size_t>(input_desc[0].dims.d[0]);
  if (max_num_elements_ < num_elements) {
    max_num_elements_ = num_elements;
    argsort_workspace_size_ = get_argsort_workspace_size(max_num_elements_);
  }

  return argsort(
    reinterpret_cast<std::int64_t const *>(inputs[0]), reinterpret_cast<std::int64_t *>(outputs[0]),
    workspace, num_elements, argsort_workspace_size_, stream);
}

std::int32_t ArgsortPlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

IPluginV3 * ArgsortPlugin::attachToContext(
  [[maybe_unused]] IPluginResourceContext * context) noexcept
{
  return clone();
}

PluginFieldCollection const * ArgsortPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t ArgsortPlugin::getWorkspaceSize(
  [[maybe_unused]] DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  std::int64_t max_num_elements = inputs[0].max.d[0];
  return get_argsort_workspace_size(max_num_elements) +
         sizeof(std::int64_t) * 2 * (max_num_elements + 1);
}

}  // namespace nvinfer1::plugin
