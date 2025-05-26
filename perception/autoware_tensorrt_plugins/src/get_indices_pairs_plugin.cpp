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

#include "autoware/tensorrt_plugins/get_indices_pairs_plugin.hpp"

#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>  // cSpell:ignore spconvlib
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/convops/SimpleExternalSpconvMatmul.h>
#include <spconvlib/spconv/csrc/sparse/convops/gemmops/GemmTunerSimple.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>
#include <spconvlib/spconv/csrc/sparse/inference/InferenceOps.h>

#include <cstdint>
#include <exception>
#include <functional>
#include <string>
#include <vector>

namespace nvinfer1::plugin
{

// cSpell:ignore INDICE indice
GetIndicesPairsPlugin::GetIndicesPairsPlugin(
  const std::string & name, GetIndicesPairsParameters const & params)
: layer_name_{name}, params_{params}
{
  initFieldsToSerialize();
}

void GetIndicesPairsPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back("batch_size", &params_.batch_size, PluginFieldType::kINT32, 1);
  data_to_serialize_.emplace_back("algo", &params_.algo, PluginFieldType::kINT32, 1);
  data_to_serialize_.emplace_back(
    "dilation_dims", &params_.dilation_dims, PluginFieldType::kDIMS, 1);
  data_to_serialize_.emplace_back("ksize_dims", &params_.ksize_dims, PluginFieldType::kDIMS, 1);
  data_to_serialize_.emplace_back(
    "out_padding_dims", &params_.out_padding_dims, PluginFieldType::kDIMS, 1);
  data_to_serialize_.emplace_back("padding_dims", &params_.padding_dims, PluginFieldType::kDIMS, 1);
  data_to_serialize_.emplace_back(
    "spatial_shape_dims", &params_.spatial_shape_dims, PluginFieldType::kDIMS, 1);
  data_to_serialize_.emplace_back("stride_dims", &params_.stride_dims, PluginFieldType::kDIMS, 1);
  data_to_serialize_.emplace_back(
    "subm", &params_.subm, PluginFieldType::kINT32, 1);  // cSpell:ignore subm
  data_to_serialize_.emplace_back("transpose", &params_.transpose, PluginFieldType::kINT32, 1);

  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

IPluginCapability * GetIndicesPairsPlugin::getCapabilityInterface(
  PluginCapabilityType type) noexcept
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

IPluginV3 * GetIndicesPairsPlugin::clone() noexcept
{
  try {
    IPluginV3 * const plugin{new GetIndicesPairsPlugin{layer_name_, params_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * GetIndicesPairsPlugin::getPluginName() const noexcept
{
  return kGET_INDICES_PAIRS_PLUGIN_NAME;
}

char const * GetIndicesPairsPlugin::getPluginVersion() const noexcept
{
  return kGET_INDICES_PAIRS_PLUGIN_VERSION;
}

char const * GetIndicesPairsPlugin::getPluginNamespace() const noexcept
{
  return kGET_INDICES_PAIRS_PLUGIN_NAMESPACE;
}

std::int32_t GetIndicesPairsPlugin::getNbOutputs() const noexcept
{
  return 4;
}

std::int32_t GetIndicesPairsPlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  // Validate input arguments.
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 4);
  PLUGIN_ASSERT(in[0].desc.dims.nbDims == 2);

  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(out[1].desc.dims.nbDims == 3);
  PLUGIN_ASSERT(out[2].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(out[3].desc.dims.nbDims == 0);

  std::int64_t kernel_volume = 1;
  for (const std::int64_t ksize : params_.ksize) {
    kernel_volume *= ksize;
  }

  PLUGIN_ASSERT(in[0].desc.dims.d[1] == 4);  // coords + 1

  PLUGIN_ASSERT(out[0].desc.dims.d[1] == 4);  // coords + 1
  PLUGIN_ASSERT(out[0].desc.type == in[0].desc.type);

  PLUGIN_ASSERT(out[1].desc.dims.d[0] == 2);
  PLUGIN_ASSERT(out[1].desc.dims.d[1] == kernel_volume);
  PLUGIN_ASSERT(out[1].desc.type == in[0].desc.type);

  PLUGIN_ASSERT(out[2].desc.dims.d[0] == kernel_volume);
  PLUGIN_ASSERT(out[2].desc.type == in[0].desc.type);

  PLUGIN_ASSERT(out[3].desc.type == in[0].desc.type);

  return 0;
}

bool GetIndicesPairsPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 4);

  return (
    in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR &&
    in_out[pos].desc.type == nvinfer1::DataType::kINT32);
}

std::int32_t GetIndicesPairsPlugin::getOutputDataTypes(
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

std::int32_t GetIndicesPairsPlugin::getOutputShapes(
  DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, std::int32_t num_outputs, IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 1);
  PLUGIN_ASSERT(num_outputs == 4);
  PLUGIN_ASSERT(inputs[0].nbDims == 2);

  std::int64_t kernel_volume = 1;

  for (std::size_t i = 0; i < params_.ksize.size(); ++i) {
    kernel_volume *= params_.ksize[i];
  }

  PLUGIN_ASSERT(params_.subm == 1);  // NOTE(knzo25): we have only tested subm

  if (params_.subm) {
    outputs[0] = inputs[0];
    outputs[0].d[1] = inputs[0].d[1];

    outputs[1].nbDims = 3;
    outputs[1].d[0] = expr_builder.constant(2);
    outputs[1].d[1] = expr_builder.constant(kernel_volume);
    outputs[1].d[2] = inputs[0].d[0];

    outputs[2].nbDims = 1;
    outputs[2].d[0] = expr_builder.constant(kernel_volume);
  } else {
    auto opt_value = expr_builder.operation(
      DimensionOperation::kCEIL_DIV, *inputs[0].d[0], *expr_builder.constant(2));

    outputs[0].nbDims = 2;
    outputs[0].d[0] =
      expr_builder.declareSizeTensor(3, *opt_value, *expr_builder.constant(out_indices_num_limit_));
    outputs[0].d[1] = inputs[0].d[1];

    outputs[1].nbDims = 3;
    outputs[1].d[0] = expr_builder.constant(2);
    outputs[1].d[1] = expr_builder.constant(kernel_volume);
    outputs[1].d[2] =
      expr_builder.declareSizeTensor(3, *opt_value, *expr_builder.constant(out_indices_num_limit_));

    outputs[2].nbDims = 1;
    outputs[2].d[0] = expr_builder.constant(kernel_volume);
  }

  outputs[3].nbDims = 0;

  return 0;
}

std::int32_t GetIndicesPairsPlugin::enqueue(
  PluginTensorDesc const * input_desc, [[maybe_unused]] PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  using SpconvOps = spconvlib::spconv::csrc::sparse::all::SpconvOps;
  using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;

  const bool is_subm = params_.subm;
  const int num_act_in = input_desc[0].dims.d[0];

  std::vector<int32_t> ksize(params_.ksize.begin(), params_.ksize.end());
  std::vector<int32_t> stride(params_.stride.begin(), params_.stride.end());
  std::vector<int32_t> padding(params_.padding.begin(), params_.padding.end());
  std::vector<int32_t> dilation(params_.dilation.begin(), params_.dilation.end());
  std::vector<int32_t> input_dims(params_.spatial_shape.begin(), params_.spatial_shape.end());

  auto out_dims = SpconvOps::get_conv_output_size(input_dims, ksize, stride, padding, dilation);
  std::vector<std::int64_t> output_dims_i64(out_dims.begin(), out_dims.end());
  std::int64_t out_spatial_volume = std::accumulate(
    output_dims_i64.begin(), output_dims_i64.end(), static_cast<std::int64_t>(1),
    std::multiplies<std::int64_t>());

  bool use_int64_hash_k =
    out_spatial_volume >= static_cast<std::int64_t>(std::numeric_limits<int>::max());

  int kernel_volume =
    std::accumulate(params_.ksize.begin(), params_.ksize.end(), 1, std::multiplies<int>());

  auto ws_tensors = SpconvOps::get_indice_gen_tensors_from_workspace(
    reinterpret_cast<std::uint8_t *>(workspace), kernel_volume, out_indices_num_limit_,
    out_indices_num_limit_, 0, is_subm, use_int64_hash_k, false);

  tv::Tensor pair = tv::from_blob(outputs[1], {2, kernel_volume, num_act_in}, tv::int32, 0);
  tv::Tensor indices_kernel_num = tv::from_blob(outputs[2], {kernel_volume}, tv::int32, 0);

  cudaMemsetAsync(indices_kernel_num.data_ptr(), 0, kernel_volume * sizeof(std::int32_t), stream);

  tv::Tensor out_indices =
    tv::from_blob(outputs[0], {is_subm ? num_act_in : out_indices_num_limit_, 4}, tv::int32, 0);

  ws_tensors.insert({SPCONV_ALLOC_PAIR_FWD, pair});
  ws_tensors.insert({SPCONV_ALLOC_INDICE_NUM_PER_LOC, indices_kernel_num});
  ws_tensors.insert({SPCONV_ALLOC_OUT_INDICES, out_indices});

  tv::Tensor input_indices = tv::from_blob(inputs[0], {num_act_in, 4}, tv::int32, 0);

  StaticAllocator alloc(ws_tensors);

  int num_act_out_real = SpconvOps::get_indice_pairs(
    alloc, input_indices, params_.batch_size, is_subm ? input_dims : out_dims,
    static_cast<int>(tv::gemm::SparseConvAlgo::kNative), ksize, stride, padding, dilation,
    {0, 0, 0}, is_subm, false, reinterpret_cast<std::uintptr_t>(stream), out_indices_num_limit_,
    num_act_in);

  if (is_subm) {
    cudaMemcpyAsync(
      outputs[0], inputs[0], num_act_in * 4 * sizeof(std::int32_t), cudaMemcpyDeviceToDevice,
      stream);
  }

  std::int32_t * num_act_out_data = static_cast<std::int32_t *>(outputs[3]);

  cudaError_t const status = cudaMemcpyAsync(
    num_act_out_data, &num_act_out_real, sizeof(std::int32_t), cudaMemcpyHostToDevice, stream);

  cudaStreamSynchronize(stream);

  return status;
}

std::int32_t GetIndicesPairsPlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

IPluginV3 * GetIndicesPairsPlugin::attachToContext(
  [[maybe_unused]] IPluginResourceContext * context) noexcept
{
  return clone();
}

PluginFieldCollection const * GetIndicesPairsPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t GetIndicesPairsPlugin::getWorkspaceSize(
  [[maybe_unused]] DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  using SpconvOps = spconvlib::spconv::csrc::sparse::all::SpconvOps;

  bool is_subm = params_.subm;

  std::vector<int> ksize(params_.ksize.begin(), params_.ksize.end());
  std::vector<int> stride(params_.stride.begin(), params_.stride.end());
  std::vector<int> padding(params_.padding.begin(), params_.padding.end());
  std::vector<int> dilation(params_.dilation.begin(), params_.dilation.end());
  std::vector<int> input_dims(params_.spatial_shape.begin(), params_.spatial_shape.end());

  auto out_dims = SpconvOps::get_conv_output_size(input_dims, ksize, stride, padding, dilation);
  std::vector<std::int64_t> output_dims_i64(out_dims.begin(), out_dims.end());
  std::int64_t out_spatial_volume = std::accumulate(
    output_dims_i64.begin(), output_dims_i64.end(), static_cast<std::int64_t>(1),
    std::multiplies<int64_t>());
  bool use_int64_hash_k =
    out_spatial_volume >= static_cast<std::int64_t>(std::numeric_limits<int>::max());

  int kernel_volume =
    std::accumulate(params_.ksize.begin(), params_.ksize.end(), 1, std::multiplies<int>());

  int workspace_size = SpconvOps::get_indice_gen_workspace_size(
    kernel_volume, out_indices_num_limit_, out_indices_num_limit_, 0, is_subm, use_int64_hash_k,
    false);

  return static_cast<std::size_t>(workspace_size);
}

}  // namespace nvinfer1::plugin
