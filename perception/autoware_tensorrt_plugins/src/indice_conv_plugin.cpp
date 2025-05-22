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

// cSpell:ignore Indice INDICE Matmul indice
#include "autoware/tensorrt_plugins/indice_conv_plugin.hpp"

#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <spconvlib/cumm/gemm/main/GemmMainUnitTest.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>  // cSpell:ignore spconvlib
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/convops/SimpleExternalSpconvMatmul.h>
#include <spconvlib/spconv/csrc/sparse/convops/gemmops/GemmTunerSimple.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>
#include <spconvlib/spconv/csrc/sparse/inference/InferenceOps.h>

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace nvinfer1::plugin
{

IndiceConvPlugin::IndiceConvPlugin(const std::string & name, IndiceConvParameters const & params)
: layer_name_{name}, params_{params}
{
  using ConvGemmOps = spconvlib::spconv::csrc::sparse::convops::spops::ConvGemmOps;
  using GemmMain = spconvlib::cumm::gemm::main::GemmMainUnitTest;

  initFieldsToSerialize();

  arch_ = ConvGemmOps::get_compute_capability();
  tuner_fp16_ptr_ =
    std::make_unique<GemmTunerSimple>(GemmMain::get_all_algo_desp());  // cSpell:ignore desp
  tuner_fp32_ptr_ = std::make_unique<GemmTunerSimple>(GemmMain::get_all_algo_desp());
}

void IndiceConvPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back(
    "is_subm", &params_.is_subm, PluginFieldType::kINT32, 1);  // cSpell:ignore subm

  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

IPluginCapability * IndiceConvPlugin::getCapabilityInterface(PluginCapabilityType type) noexcept
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

IPluginV3 * IndiceConvPlugin::clone() noexcept
{
  try {
    IPluginV3 * const plugin{new IndiceConvPlugin{layer_name_, params_}};
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * IndiceConvPlugin::getPluginName() const noexcept
{
  return kINDICE_CONV_PLUGIN_NAME;
}

char const * IndiceConvPlugin::getPluginVersion() const noexcept
{
  return kINDICE_CONV_PLUGIN_VERSION;
}

char const * IndiceConvPlugin::getPluginNamespace() const noexcept
{
  return kINDICE_CONV_PLUGIN_NAMESPACE;
}

std::int32_t IndiceConvPlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t IndiceConvPlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  // Validate input arguments.
  PLUGIN_ASSERT(num_inputs == 5);
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(in[INOUT_IN_FEATURES_INDEX].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(in[INOUT_FILTERS_INDEX].desc.dims.nbDims == 5);
  PLUGIN_ASSERT(in[INOUT_INDICE_PAIRS_INDEX].desc.dims.nbDims == 3);
  PLUGIN_ASSERT(in[INOUT_INDICE_PAIRS_NUM_INDEX].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(in[INOUT_NUM_ACTIVATE_OUT_INDEX].desc.dims.nbDims == 0);
  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(
    in[INOUT_FILTERS_INDEX].desc.dims.d[4] == in[INOUT_IN_FEATURES_INDEX].desc.dims.d[1]);

  PLUGIN_ASSERT(in[INOUT_INDICE_PAIRS_INDEX].desc.dims.d[0] == 2);

  PLUGIN_ASSERT(in[INOUT_IN_FEATURES_INDEX].desc.type == in[INOUT_FILTERS_INDEX].desc.type);
  PLUGIN_ASSERT(in[INOUT_IN_FEATURES_INDEX].desc.type == out[0].desc.type);
  PLUGIN_ASSERT(
    in[INOUT_INDICE_PAIRS_INDEX].desc.type == in[INOUT_INDICE_PAIRS_NUM_INDEX].desc.type);
  return 0;
}

bool IndiceConvPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == 5);
  PLUGIN_ASSERT(num_outputs == 1);

  bool supported = in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR;

  switch (pos) {
    case INOUT_IN_FEATURES_INDEX:
      supported &=
        (in_out[pos].desc.type == nvinfer1::DataType::kFLOAT ||
         in_out[pos].desc.type == nvinfer1::DataType::kHALF);
      break;
    case INOUT_FILTERS_INDEX:
    case INOUT_OUT_FEATURES_INDEX:
      supported &= in_out[pos].desc.type == in_out[INOUT_IN_FEATURES_INDEX].desc.type;
      break;
    case INOUT_INDICE_PAIRS_INDEX:
    case INOUT_INDICE_PAIRS_NUM_INDEX:
    case INOUT_NUM_ACTIVATE_OUT_INDEX:
      supported &= in_out[pos].desc.type == nvinfer1::DataType::kINT32;
      break;
    default:
      supported = false;
      break;
  }

  return supported;
}

std::int32_t IndiceConvPlugin::getOutputDataTypes(
  DataType * output_types, std::int32_t num_outputs, DataType const * input_types,
  std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(num_inputs == 5);
  PLUGIN_ASSERT(num_outputs == 1);

  output_types[0] = input_types[INOUT_IN_FEATURES_INDEX];

  return 0;
}

std::int32_t IndiceConvPlugin::getOutputShapes(
  DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, std::int32_t num_outputs,
  [[maybe_unused]] IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(num_inputs == 5);
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(inputs[0].nbDims == 2);

  outputs[0].nbDims = 2;
  outputs[0].d[0] = inputs[INOUT_INDICE_PAIRS_INDEX].d[2];
  outputs[0].d[1] = inputs[INOUT_FILTERS_INDEX].d[0];

  return 0;
}

std::int32_t IndiceConvPlugin::enqueue(
  PluginTensorDesc const * input_desc, [[maybe_unused]] PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;
  using ConvGemmOps = spconvlib::spconv::csrc::sparse::convops::spops::ConvGemmOps;
  using SimpleExternalSpconvMatmul =
    spconvlib::spconv::csrc::sparse::convops::SimpleExternalSpconvMatmul;

  std::int64_t num_act_in = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[0];
  std::int64_t num_in_features = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[1];
  std::int64_t num_act_out = input_desc[INOUT_INDICE_PAIRS_INDEX].dims.d[2];
  std::int64_t num_out_features = input_desc[INOUT_FILTERS_INDEX].dims.d[0];

  auto in_features_type = input_desc[INOUT_IN_FEATURES_INDEX].type;
  [[maybe_unused]] auto filters_type = input_desc[INOUT_FILTERS_INDEX].type;
  [[maybe_unused]] auto out_features_type = input_desc[INOUT_OUT_FEATURES_INDEX].type;

  assert(in_features_type == filters_type);
  assert(in_features_type == out_features_type);

  auto dtype = in_features_type == DataType::kFLOAT ? tv::float32 : tv::float16;

  tv::Tensor input_features =
    tv::from_blob(inputs[INOUT_IN_FEATURES_INDEX], {num_act_in, num_in_features}, dtype, 0);

  tv::Tensor input_features_fp32 =
    tv::from_blob(inputs[INOUT_IN_FEATURES_INDEX], {num_act_in, num_in_features}, tv::float32, 0);

  tv::Tensor weights = tv::from_blob(
    inputs[INOUT_FILTERS_INDEX],
    {input_desc[INOUT_FILTERS_INDEX].dims.d[0], input_desc[INOUT_FILTERS_INDEX].dims.d[1],
     input_desc[INOUT_FILTERS_INDEX].dims.d[2], input_desc[INOUT_FILTERS_INDEX].dims.d[3],
     input_desc[INOUT_FILTERS_INDEX].dims.d[4]},
    dtype, 0);

  tv::Tensor pairs = tv::from_blob(
    inputs[INOUT_INDICE_PAIRS_INDEX],
    {input_desc[INOUT_INDICE_PAIRS_INDEX].dims.d[0], input_desc[INOUT_INDICE_PAIRS_INDEX].dims.d[1],
     input_desc[INOUT_INDICE_PAIRS_INDEX].dims.d[2]},
    tv::int32, 0);

  tv::Tensor pairs_num = tv::from_blob(
    inputs[INOUT_INDICE_PAIRS_NUM_INDEX], {input_desc[INOUT_INDICE_PAIRS_NUM_INDEX].dims.d[0]},
    tv::int32, 0);

  tv::Tensor out_features = tv::from_blob(outputs[0], {num_act_out, num_out_features}, dtype, 0);

  auto & tuner_ptr = dtype == tv::float32 ? tuner_fp32_ptr_ : tuner_fp16_ptr_;

  std::unordered_map<std::string, tv::Tensor> tensor_dict{
    {SPCONV_ALLOC_FEATURES, input_features},
    {SPCONV_ALLOC_FILTERS, weights},
    {SPCONV_ALLOC_OUT_FEATURES, out_features}};
  StaticAllocator alloc2(tensor_dict);

  SimpleExternalSpconvMatmul ext_mm(alloc2);

  ConvGemmOps::indice_conv(
    alloc2, ext_mm, *tuner_ptr, true, false, input_features, weights, pairs, pairs_num, arch_,
    out_features.dim(0), false, params_.is_subm,
    static_cast<int>(tv::gemm::SparseConvAlgo::kNative), reinterpret_cast<std::uintptr_t>(stream),
    tv::Tensor(), 0.f, 0.f, tv::gemm::Activation::kNone, false);

  return 0;
}

std::int32_t IndiceConvPlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

IPluginV3 * IndiceConvPlugin::attachToContext(
  [[maybe_unused]] IPluginResourceContext * context) noexcept
{
  return clone();
}

PluginFieldCollection const * IndiceConvPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t IndiceConvPlugin::getWorkspaceSize(
  [[maybe_unused]] DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return 0;
}

}  // namespace nvinfer1::plugin
