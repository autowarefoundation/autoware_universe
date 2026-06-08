// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_PLUGINS__SERIALIZED_POOLING_PLUGIN_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__SERIALIZED_POOLING_PLUGIN_HPP_

#include "autoware/ptv3_ops/serialized_pooling.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

constexpr char const * const kPTV3_SERIALIZED_POOLING_PLUGIN_NAME{"PTv3SerializedPooling"};
constexpr char const * const kPTV3_SERIALIZED_POOLING_PLUGIN_VERSION{"1"};
// Empty namespace to match every other plugin in this package: the ONNX-parser fallback importer
// resolves custom ops (domain "autoware") against the default/empty plugin namespace.
constexpr char const * const kPTV3_SERIALIZED_POOLING_PLUGIN_NAMESPACE{""};

namespace autoware::ptv3
{

class PTv3SerializedPoolingPlugin : public nvinfer1::IPluginV3,
                                    public nvinfer1::IPluginV3OneCore,
                                    public nvinfer1::IPluginV3OneBuild,
                                    public nvinfer1::IPluginV3OneRuntime
{
public:
  PTv3SerializedPoolingPlugin(const std::string & name, const std::string & reduce);

  ~PTv3SerializedPoolingPlugin() override = default;

  nvinfer1::IPluginCapability * getCapabilityInterface(
    nvinfer1::PluginCapabilityType type) noexcept override;

  nvinfer1::IPluginV3 * clone() noexcept override;

  char const * getPluginName() const noexcept override;

  char const * getPluginVersion() const noexcept override;

  char const * getPluginNamespace() const noexcept override;

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
  static constexpr std::int32_t kFeatureInput = 0;
  static constexpr std::int32_t kCoordInput = 1;
  static constexpr std::int32_t kIndicesInput = 2;
  static constexpr std::int32_t kIndptrInput = 3;
  static constexpr std::int32_t kFeatureOutput = 4;
  static constexpr std::int32_t kCoordOutput = 5;

  void initFieldsToSerialize();

  std::string layer_name_;
  std::string reduce_;
  SerializedPoolingReduce reduce_type_;
  std::vector<nvinfer1::PluginField> data_to_serialize_;
  nvinfer1::PluginFieldCollection fc_to_serialize_{};
};

/// Convert the ONNX plugin `reduce` attribute into the kernel reduction mode.
SerializedPoolingReduce parseSerializedPoolingReduce(const std::string & reduce);

}  // namespace autoware::ptv3

#endif  // AUTOWARE__TENSORRT_PLUGINS__SERIALIZED_POOLING_PLUGIN_HPP_
