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

#ifndef AUTOWARE__TENSORRT_PLUGINS__ROTATE_PLUGIN_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__ROTATE_PLUGIN_HPP_

#include "autoware/rotate_ops/rotate_kernel.h"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

constexpr char const * const kROTATE_PLUGIN_NAME{"RotatePlugin"};
constexpr char const * const kROTATE_PLUGIN_VERSION{"1"};
constexpr char const * const kROTATE_PLUGIN_NAMESPACE{""};

namespace autoware::tensorrt_plugins
{

class RotatePlugin : public nvinfer1::IPluginV3,
                     public nvinfer1::IPluginV3OneCore,
                     public nvinfer1::IPluginV3OneBuild,
                     public nvinfer1::IPluginV3OneRuntime
{
public:
  explicit RotatePlugin(const std::string & name) noexcept;
  RotatePlugin(const std::string & name, RotateInterpolation mode) noexcept;

  ~RotatePlugin() override = default;

  // IPluginV3 Methods

  nvinfer1::IPluginCapability * getCapabilityInterface(
    nvinfer1::PluginCapabilityType type) noexcept override;

  nvinfer1::IPluginV3 * clone() noexcept override;

  // IPluginV3OneCore Methods

  char const * getPluginName() const noexcept override;

  char const * getPluginVersion() const noexcept override;

  char const * getPluginNamespace() const noexcept override;

  // IPluginV3OneBuild Methods

  std::int32_t getNbOutputs() const noexcept override;

  std::int32_t configurePlugin(
    nvinfer1::DynamicPluginTensorDesc const * in, std::int32_t num_inputs,
    nvinfer1::DynamicPluginTensorDesc const * out, std::int32_t num_outputs) noexcept override;

  bool supportsFormatCombination(
    std::int32_t pos, nvinfer1::DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
    std::int32_t num_outputs) noexcept override;

  std::int32_t getOutputDataTypes(
    nvinfer1::DataType * output_types, std::int32_t num_outputs,
    nvinfer1::DataType const * input_types, std::int32_t num_inputs) const noexcept override;

  std::int32_t getOutputShapes(
    nvinfer1::DimsExprs const * inputs, std::int32_t num_inputs,
    nvinfer1::DimsExprs const * shape_inputs, std::int32_t num_shape_inputs,
    nvinfer1::DimsExprs * outputs, std::int32_t num_outputs,
    nvinfer1::IExprBuilder & expr_builder) noexcept override;

  // IPluginV3OneRuntime Methods

  std::int32_t enqueue(
    nvinfer1::PluginTensorDesc const * input_desc, nvinfer1::PluginTensorDesc const * output_desc,
    void const * const * inputs, void * const * outputs, void * workspace,
    cudaStream_t stream) noexcept override;

  std::int32_t onShapeChange(
    nvinfer1::PluginTensorDesc const * in, std::int32_t num_inputs,
    nvinfer1::PluginTensorDesc const * out, std::int32_t num_outputs) noexcept override;

  nvinfer1::IPluginV3 * attachToContext(
    nvinfer1::IPluginResourceContext * context) noexcept override;

  nvinfer1::PluginFieldCollection const * getFieldsToSerialize() noexcept override;

  std::size_t getWorkspaceSize(
    nvinfer1::DynamicPluginTensorDesc const * inputs, std::int32_t num_inputs,
    nvinfer1::DynamicPluginTensorDesc const * outputs,
    std::int32_t num_outputs) const noexcept override;

private:
  void initFieldsToSerialize();

  std::string layer_name_;
  RotateInterpolation mode_{RotateInterpolation::Nearest};
  std::vector<nvinfer1::PluginField> data_to_serialize_;
  nvinfer1::PluginFieldCollection fc_to_serialize_;
};

}  // namespace autoware::tensorrt_plugins

#endif  // AUTOWARE__TENSORRT_PLUGINS__ROTATE_PLUGIN_HPP_
