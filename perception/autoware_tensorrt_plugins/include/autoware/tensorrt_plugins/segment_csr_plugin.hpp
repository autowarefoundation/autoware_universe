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

#ifndef AUTOWARE__TENSORRT_PLUGINS__SEGMENT_CSR_PLUGIN_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__SEGMENT_CSR_PLUGIN_HPP_

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_runtime.h>

#include <cstdint>
#include <string>
#include <vector>

constexpr char const * const kSEGMENT_CSR_PLUGIN_NAME{"SegmentCSR"};
constexpr char const * const kSEGMENT_CSR_PLUGIN_VERSION{"1"};
constexpr char const * const kSEGMENT_CSR_PLUGIN_NAMESPACE{""};

namespace nvinfer1::plugin
{

class SegmentCSRPlugin : public IPluginV3,
                         public IPluginV3OneCore,
                         public IPluginV3OneBuild,
                         public IPluginV3OneRuntime
{
public:
  SegmentCSRPlugin(const std::string & name, const std::string & reduce);

  ~SegmentCSRPlugin() override = default;

  // IPluginV3 Methods

  IPluginCapability * getCapabilityInterface(PluginCapabilityType type) noexcept override;

  IPluginV3 * clone() noexcept override;

  // IPluginV3OneCore Methods

  char const * getPluginName() const noexcept override;

  char const * getPluginVersion() const noexcept override;

  char const * getPluginNamespace() const noexcept override;

  // IPluginV3OneBuild Methods

  std::int32_t getNbOutputs() const noexcept override;

  std::int32_t configurePlugin(
    DynamicPluginTensorDesc const * in, std::int32_t num_inputs,
    DynamicPluginTensorDesc const * out, std::int32_t num_outputs) noexcept override;

  bool supportsFormatCombination(
    std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
    std::int32_t num_outputs) noexcept override;

  std::int32_t getOutputDataTypes(
    DataType * output_types, std::int32_t num_outputs, DataType const * input_types,
    std::int32_t num_inputs) const noexcept override;

  std::int32_t getOutputShapes(
    DimsExprs const * inputs, std::int32_t num_inputs, DimsExprs const * shape_inputs,
    std::int32_t num_shape_inputs, DimsExprs * outputs, std::int32_t num_outputs,
    IExprBuilder & expr_builder) noexcept override;

  // IPluginV3OneRuntime Methods

  std::int32_t enqueue(
    PluginTensorDesc const * input_desc, PluginTensorDesc const * output_desc,
    void const * const * inputs, void * const * outputs, void * workspace,
    cudaStream_t stream) noexcept override;

  std::int32_t onShapeChange(
    PluginTensorDesc const * in, std::int32_t num_inputs, PluginTensorDesc const * out,
    std::int32_t num_outputs) noexcept override;

  IPluginV3 * attachToContext(IPluginResourceContext * context) noexcept override;

  PluginFieldCollection const * getFieldsToSerialize() noexcept override;

  std::size_t getWorkspaceSize(
    DynamicPluginTensorDesc const * inputs, std::int32_t num_inputs,
    DynamicPluginTensorDesc const * outputs, std::int32_t num_outputs) const noexcept override;

private:
  static constexpr std::int32_t INOUT_IN_SRC_INDEX{0};
  static constexpr std::int32_t INOUT_IN_INDPTR_INDEX{1};
  static constexpr std::int32_t INOUT_OUT_INDEX{2};

  void initFieldsToSerialize();

  std::string layer_name_;
  std::string reduce_;
  std::vector<nvinfer1::PluginField> data_to_serialize_;
  nvinfer1::PluginFieldCollection fc_to_serialize_;
};

}  // namespace nvinfer1::plugin

#endif  // AUTOWARE__TENSORRT_PLUGINS__SEGMENT_CSR_PLUGIN_HPP_
