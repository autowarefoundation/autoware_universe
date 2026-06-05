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

#include "autoware/tensorrt_plugins/serialized_pooling_plugin.hpp"

#include "autoware/ptv3_ops/serialized_pooling.hpp"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace
{

bool isFeatureType(nvinfer1::DataType type_in)
{
  return type_in == nvinfer1::DataType::kFLOAT || type_in == nvinfer1::DataType::kHALF;
}

}  // namespace

SerializedPoolingReduce parseSerializedPoolingReduce(const std::string & reduce)
{
  if (reduce == "sum") {
    return SerializedPoolingReduce::kSum;
  }
  if (reduce == "mean") {
    return SerializedPoolingReduce::kMean;
  }
  if (reduce == "min") {
    return SerializedPoolingReduce::kMin;
  }
  if (reduce == "max") {
    return SerializedPoolingReduce::kMax;
  }
  throw std::invalid_argument("Unsupported SerializedPooling reduce mode: " + reduce);
}

SerializedPoolingPlugin::SerializedPoolingPlugin(
  const std::string & name, const std::string & reduce)
: layer_name_{name}, reduce_{reduce}, reduce_type_{parseSerializedPoolingReduce(reduce)}
{
  initFieldsToSerialize();
}

void SerializedPoolingPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back(
    "reduce", reduce_.c_str(), nvinfer1::PluginFieldType::kCHAR, reduce_.size());

  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

nvinfer1::IPluginCapability * SerializedPoolingPlugin::getCapabilityInterface(
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

nvinfer1::IPluginV3 * SerializedPoolingPlugin::clone() noexcept
{
  try {
    return new SerializedPoolingPlugin{layer_name_, reduce_};
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * SerializedPoolingPlugin::getPluginName() const noexcept
{
  return kSERIALIZED_POOLING_PLUGIN_NAME;
}

char const * SerializedPoolingPlugin::getPluginVersion() const noexcept
{
  return kSERIALIZED_POOLING_PLUGIN_VERSION;
}

char const * SerializedPoolingPlugin::getPluginNamespace() const noexcept
{
  return kSERIALIZED_POOLING_PLUGIN_NAMESPACE;
}

std::int32_t SerializedPoolingPlugin::getNbOutputs() const noexcept
{
  return 2;
}

std::int32_t SerializedPoolingPlugin::configurePlugin(
  nvinfer1::DynamicPluginTensorDesc const * in, std::int32_t num_inputs,
  nvinfer1::DynamicPluginTensorDesc const * out, std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 4);
  PLUGIN_ASSERT(num_outputs == 2);

  PLUGIN_ASSERT(in[kFeatureInput].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(in[kCoordInput].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(in[kIndicesInput].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(in[kIndptrInput].desc.dims.nbDims == 1);

  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(out[1].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(out[0].desc.type == in[kFeatureInput].desc.type);
  PLUGIN_ASSERT(out[1].desc.type == nvinfer1::DataType::kFLOAT);

  return 0;
}

bool SerializedPoolingPlugin::supportsFormatCombination(
  std::int32_t pos, nvinfer1::DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 4);
  PLUGIN_ASSERT(num_outputs == 2);

  bool supported = in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR;
  switch (pos) {
    case kFeatureInput:
      supported &= isFeatureType(in_out[pos].desc.type);
      break;
    case kCoordInput:
      supported &= in_out[pos].desc.type == nvinfer1::DataType::kFLOAT;
      break;
    case kIndicesInput:
    case kIndptrInput:
      supported &= in_out[pos].desc.type == nvinfer1::DataType::kINT64;
      break;
    case kFeatureOutput:
      supported &= in_out[pos].desc.type == in_out[kFeatureInput].desc.type;
      break;
    case kCoordOutput:
      supported &= in_out[pos].desc.type == nvinfer1::DataType::kFLOAT;
      break;
    default:
      supported = false;
      break;
  }

  return supported;
}

std::int32_t SerializedPoolingPlugin::getOutputDataTypes(
  nvinfer1::DataType * output_types, std::int32_t num_outputs,
  nvinfer1::DataType const * input_types, std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 4);
  PLUGIN_ASSERT(num_outputs == 2);

  output_types[0] = input_types[kFeatureInput];
  output_types[1] = nvinfer1::DataType::kFLOAT;

  return 0;
}

std::int32_t SerializedPoolingPlugin::getOutputShapes(
  nvinfer1::DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DimsExprs const * shape_inputs,
  [[maybe_unused]] std::int32_t num_shape_inputs, nvinfer1::DimsExprs * outputs,
  std::int32_t num_outputs, nvinfer1::IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 4);
  PLUGIN_ASSERT(num_outputs == 2);
  PLUGIN_ASSERT(inputs[kFeatureInput].nbDims == 2);
  PLUGIN_ASSERT(inputs[kCoordInput].nbDims == 2);
  PLUGIN_ASSERT(inputs[kIndptrInput].nbDims == 1);

  auto * num_segments = expr_builder.operation(
    nvinfer1::DimensionOperation::kSUB, *inputs[kIndptrInput].d[0], *expr_builder.constant(1));

  outputs[0].nbDims = 2;
  outputs[0].d[0] = num_segments;
  outputs[0].d[1] = inputs[kFeatureInput].d[1];

  outputs[1].nbDims = 2;
  outputs[1].d[0] = num_segments;
  outputs[1].d[1] = inputs[kCoordInput].d[1];

  return 0;
}

std::int32_t SerializedPoolingPlugin::enqueue(
  nvinfer1::PluginTensorDesc const * input_desc,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * output_desc, void const * const * inputs,
  void * const * outputs, [[maybe_unused]] void * workspace, cudaStream_t stream) noexcept
{
  const auto num_segments = static_cast<std::int32_t>(input_desc[kIndptrInput].dims.d[0] - 1);
  const auto num_channels = static_cast<std::int32_t>(input_desc[kFeatureInput].dims.d[1]);
  const auto * coord_in = static_cast<const float *>(inputs[kCoordInput]);
  const auto * indices_in = static_cast<const std::int64_t *>(inputs[kIndicesInput]);
  const auto * indptr_in = static_cast<const std::int64_t *>(inputs[kIndptrInput]);
  auto * coord_out = static_cast<float *>(outputs[1]);

  cudaError_t status = cudaErrorInvalidValue;
  if (input_desc[kFeatureInput].type == nvinfer1::DataType::kFLOAT) {
    status = serialized_pooling_float(
      static_cast<const float *>(inputs[kFeatureInput]), coord_in, indices_in, indptr_in,
      static_cast<float *>(outputs[0]), coord_out, num_segments, num_channels, reduce_type_,
      stream);
  } else if (input_desc[kFeatureInput].type == nvinfer1::DataType::kHALF) {
    status = serialized_pooling_half(
      static_cast<const half *>(inputs[kFeatureInput]), coord_in, indices_in, indptr_in,
      static_cast<half *>(outputs[0]), coord_out, num_segments, num_channels, reduce_type_, stream);
  }

  return status == cudaSuccess ? 0 : -1;
}

std::int32_t SerializedPoolingPlugin::onShapeChange(
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::PluginTensorDesc const * out,
  [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

nvinfer1::IPluginV3 * SerializedPoolingPlugin::attachToContext(
  [[maybe_unused]] nvinfer1::IPluginResourceContext * context) noexcept
{
  return clone();
}

nvinfer1::PluginFieldCollection const * SerializedPoolingPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t SerializedPoolingPlugin::getWorkspaceSize(
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * inputs,
  [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] nvinfer1::DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return 0;
}

}  // namespace autoware::ptv3
