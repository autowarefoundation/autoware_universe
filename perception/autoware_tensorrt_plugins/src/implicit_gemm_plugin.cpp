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

#include "autoware/tensorrt_plugins/implicit_gemm_plugin.hpp"

#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>  // cSpell:ignore spconvlib
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/convops/SimpleExternalSpconvMatmul.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>
#include <spconvlib/spconv/csrc/sparse/inference/InferenceOps.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

/** Validate the bias inputs, then wrap the bias device pointer into a tv::Tensor of the
 * activation dtype (no copy).
 *
 * Spconv's fused bias-add needs the bias dtype to match the activation dtype; the ONNX export
 * already emits it that way, so we only assert the match and wrap the pointer. */
void validate_and_wrap_bias_tensor(
  void const * bias_in, std::int64_t c_bias, tv::DType activation_dtype,
  nvinfer1::DataType bias_trt_type, int device, tv::Tensor * bias_out) noexcept
{
  PLUGIN_ASSERT(bias_out != nullptr);
  PLUGIN_ASSERT(bias_in != nullptr);
  PLUGIN_ASSERT(activation_dtype == tv::float16 || activation_dtype == tv::float32);
  PLUGIN_ASSERT(
    bias_trt_type ==
    (activation_dtype == tv::float16 ? nvinfer1::DataType::kHALF : nvinfer1::DataType::kFLOAT));
  *bias_out = tv::from_blob(const_cast<void *>(bias_in), {c_bias}, activation_dtype, device);
}

tv::gemm::Activation activation_from_int(std::int32_t const v) noexcept
{
  PLUGIN_ASSERT(
    v >= static_cast<std::int32_t>(tv::gemm::Activation::kNone) &&
    v <= static_cast<std::int32_t>(tv::gemm::Activation::kLeakyReLU));
  return static_cast<tv::gemm::Activation>(v);
}
}  // namespace

namespace nvinfer1::plugin
{

ImplicitGemmPlugin::ImplicitGemmPlugin(
  const std::string & name, ImplicitGemmParameters const & params)
: layer_name_{name}, params_{params}
{
  using ConvGemmOps = spconvlib::spconv::csrc::sparse::convops::spops::ConvGemmOps;
  using ConvMain = spconvlib::cumm::conv::main::ConvMainUnitTest;

  initFieldsToSerialize();

  arch_ = ConvGemmOps::get_compute_capability();
  tuner_fp16_ptr_ =
    std::make_unique<ConvTunerSimple>(ConvMain::get_all_conv_algo_desp());  // cSpell:ignore desp
  tuner_fp32_ptr_ = std::make_unique<ConvTunerSimple>(ConvMain::get_all_conv_algo_desp());

  // Pre-allocate CPU mask tensor to avoid heap allocation during CUDA graph capture.
  mask_tensor_ = tv::zeros({1}, tv::uint32, -1);
  mask_tensor_.data_ptr<uint32_t>()[0] = 0xffffffff;
}

void ImplicitGemmPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back(
    "parameters", &params_, PluginFieldType::kUNKNOWN, sizeof(ImplicitGemmParameters));

  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

IPluginCapability * ImplicitGemmPlugin::getCapabilityInterface(PluginCapabilityType type) noexcept
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

IPluginV3 * ImplicitGemmPlugin::clone() noexcept
{
  try {
    ImplicitGemmPlugin * const plugin{new ImplicitGemmPlugin{layer_name_, params_}};
    plugin->num_plugin_inputs_ = num_plugin_inputs_;
    return plugin;
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * ImplicitGemmPlugin::getPluginName() const noexcept
{
  return kIMPLICIT_GEMM_PLUGIN_NAME;
}

char const * ImplicitGemmPlugin::getPluginVersion() const noexcept
{
  return kIMPLICIT_GEMM_PLUGIN_VERSION;
}

char const * ImplicitGemmPlugin::getPluginNamespace() const noexcept
{
  return kIMPLICIT_GEMM_PLUGIN_NAMESPACE;
}

std::int32_t ImplicitGemmPlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t ImplicitGemmPlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  // Validate input arguments.
  // 5 inputs: standard implicit GEMM path without fused bias.
  // 6 inputs: implicit GEMM path with optional per-channel bias.
  // TODO(vividf): should use PLUGIN_VALIDATE_AND_RETURN instead of PLUGIN_ASSERT for input
  // arguments
  PLUGIN_ASSERT(in != nullptr);
  PLUGIN_ASSERT(out != nullptr);
  PLUGIN_ASSERT(num_inputs == NUM_PLUGIN_INPUTS_NO_BIAS || num_inputs == NUM_PLUGIN_INPUTS_BIAS);
  PLUGIN_ASSERT(num_outputs == 1);

  num_plugin_inputs_ = num_inputs;

  PLUGIN_ASSERT(in[INOUT_IN_FEATURES_INDEX].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(in[INOUT_FILTERS_INDEX].desc.dims.nbDims == 5);
  PLUGIN_ASSERT(in[INOUT_PAIR_FWD_INDEX].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(in[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].desc.dims.nbDims == 1);
  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 2);
  PLUGIN_ASSERT(
    in[INOUT_FILTERS_INDEX].desc.dims.d[4] == in[INOUT_IN_FEATURES_INDEX].desc.dims.d[1]);
  PLUGIN_ASSERT(
    in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.dims.d[0] == in[INOUT_PAIR_FWD_INDEX].desc.dims.d[1]);
  PLUGIN_ASSERT(in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.dims.d[1] == 1);
  PLUGIN_ASSERT(
    in[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].desc.dims.d[0] ==
    in[INOUT_PAIR_FWD_INDEX].desc.dims.d[1]);
  PLUGIN_ASSERT(in[INOUT_IN_FEATURES_INDEX].desc.type == in[INOUT_FILTERS_INDEX].desc.type);
  PLUGIN_ASSERT(in[INOUT_IN_FEATURES_INDEX].desc.type == out[0].desc.type);
  PLUGIN_ASSERT(
    in[INOUT_PAIR_FWD_INDEX].desc.type == in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.type);
  PLUGIN_ASSERT(
    in[INOUT_PAIR_FWD_INDEX].desc.type == in[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].desc.type);

  if (num_inputs == NUM_PLUGIN_INPUTS_BIAS) {
    PLUGIN_ASSERT(in[INOUT_OPTIONAL_BIAS_INDEX].desc.dims.nbDims == 1);
    PLUGIN_ASSERT(
      in[INOUT_OPTIONAL_BIAS_INDEX].desc.dims.d[0] == in[INOUT_FILTERS_INDEX].desc.dims.d[0]);
    DataType const bias_t = in[INOUT_OPTIONAL_BIAS_INDEX].desc.type;
    PLUGIN_ASSERT(bias_t == in[INOUT_IN_FEATURES_INDEX].desc.type);
  }
  PLUGIN_ASSERT(
    params_.act_type >= static_cast<std::int32_t>(tv::gemm::Activation::kNone) &&
    params_.act_type <= static_cast<std::int32_t>(tv::gemm::Activation::kLeakyReLU));

  return 0;
}

bool ImplicitGemmPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(in_out != nullptr);
  PLUGIN_ASSERT(num_inputs == NUM_PLUGIN_INPUTS_NO_BIAS || num_inputs == NUM_PLUGIN_INPUTS_BIAS);
  PLUGIN_ASSERT(num_outputs == 1);
  // in_out is the combined [inputs..., outputs...] tensor array; n_slots = num_inputs +
  // num_outputs.
  std::int32_t const n_slots = num_inputs + num_outputs;
  PLUGIN_ASSERT(pos >= 0 && pos < n_slots);

  // spconv only supports contiguous (LINEAR) layout for all tensors.
  bool supported = in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR;

  // Output features must match the input features dtype.
  std::int32_t const out_pos = num_inputs;
  if (pos == out_pos) {
    supported &= in_out[pos].desc.type == in_out[INOUT_IN_FEATURES_INDEX].desc.type;
    return supported;
  }

  switch (pos) {
    // Input features: fp32 or fp16 (spconv supports both precisions).
    case INOUT_IN_FEATURES_INDEX:
      supported &=
        (in_out[pos].desc.type == nvinfer1::DataType::kFLOAT ||
         in_out[pos].desc.type == nvinfer1::DataType::kHALF);
      break;
    // Filter weights must match input features dtype for mixed-precision-free GEMM.
    case INOUT_FILTERS_INDEX:
      supported &= in_out[pos].desc.type == in_out[INOUT_IN_FEATURES_INDEX].desc.type;
      break;
    // Index / mask tensors are always int32 regardless of activation dtype.
    case INOUT_PAIR_FWD_INDEX:
    case INOUT_PAIR_MASK_FWD_SPLITS_INDEX:
    case INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX:
      supported &= in_out[pos].desc.type == nvinfer1::DataType::kINT32;
      break;
    // Optional fused bias must match input features dtype; rejected when there are only 5 inputs.
    case INOUT_OPTIONAL_BIAS_INDEX:
      if (num_inputs == NUM_PLUGIN_INPUTS_BIAS) {
        supported &= in_out[pos].desc.type == in_out[INOUT_IN_FEATURES_INDEX].desc.type;
      } else {
        supported = false;
      }
      break;
    default:
      supported = false;
      break;
  }

  return supported;
}

std::int32_t ImplicitGemmPlugin::getOutputDataTypes(
  DataType * output_types, std::int32_t num_outputs, DataType const * input_types,
  std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(output_types != nullptr);
  PLUGIN_ASSERT(input_types != nullptr);
  PLUGIN_ASSERT(num_inputs == NUM_PLUGIN_INPUTS_NO_BIAS || num_inputs == NUM_PLUGIN_INPUTS_BIAS);
  PLUGIN_ASSERT(num_outputs == 1);

  output_types[0] = input_types[INOUT_IN_FEATURES_INDEX];

  return 0;
}

std::int32_t ImplicitGemmPlugin::getOutputShapes(
  DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, std::int32_t num_outputs,
  [[maybe_unused]] IExprBuilder & expr_builder) noexcept
{
  PLUGIN_ASSERT(inputs != nullptr);
  PLUGIN_ASSERT(outputs != nullptr);
  PLUGIN_ASSERT(num_inputs == NUM_PLUGIN_INPUTS_NO_BIAS || num_inputs == NUM_PLUGIN_INPUTS_BIAS);
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(inputs[0].nbDims == 2);

  outputs[0].nbDims = 2;
  outputs[0].d[0] = inputs[3].d[0];
  outputs[0].d[1] = inputs[1].d[0];

  return 0;
}

std::int32_t ImplicitGemmPlugin::enqueue(
  PluginTensorDesc const * input_desc, [[maybe_unused]] PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;
  using ConvGemmOps = spconvlib::spconv::csrc::sparse::convops::spops::ConvGemmOps;

  PLUGIN_ASSERT(
    num_plugin_inputs_ == NUM_PLUGIN_INPUTS_NO_BIAS ||
    num_plugin_inputs_ == NUM_PLUGIN_INPUTS_BIAS);

  std::int64_t num_act_in = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[0];
  std::int64_t num_in_features = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[1];
  // std::int64_t kernel_volume = input_desc[INOUT_PAIR_FWD_INDEX].dims.d[0];
  std::int64_t num_act_out = input_desc[INOUT_PAIR_FWD_INDEX].dims.d[1];
  std::int64_t num_out_features = input_desc[INOUT_FILTERS_INDEX].dims.d[0];

  auto in_features_type = input_desc[INOUT_IN_FEATURES_INDEX].type;
  [[maybe_unused]] auto filters_type = input_desc[INOUT_FILTERS_INDEX].type;
  [[maybe_unused]] auto out_features_type = output_desc[0].type;

  PLUGIN_ASSERT(in_features_type == filters_type);
  PLUGIN_ASSERT(in_features_type == out_features_type);

  auto dtype = in_features_type == DataType::kFLOAT ? tv::float32 : tv::float16;

  tv::Tensor input_features =
    tv::from_blob(inputs[INOUT_IN_FEATURES_INDEX], {num_act_in, num_in_features}, dtype, 0);

  tv::Tensor weights = tv::from_blob(
    inputs[INOUT_FILTERS_INDEX],
    {input_desc[INOUT_FILTERS_INDEX].dims.d[0], input_desc[INOUT_FILTERS_INDEX].dims.d[1],
     input_desc[INOUT_FILTERS_INDEX].dims.d[2], input_desc[INOUT_FILTERS_INDEX].dims.d[3],
     input_desc[INOUT_FILTERS_INDEX].dims.d[4]},
    dtype, 0);

  tv::Tensor pair_fwd = tv::from_blob(
    inputs[INOUT_PAIR_FWD_INDEX],
    {input_desc[INOUT_PAIR_FWD_INDEX].dims.d[0], input_desc[INOUT_PAIR_FWD_INDEX].dims.d[1]},
    tv::int32, 0);

  tv::Tensor pair_mask_fwd_splits = tv::from_blob(
    inputs[INOUT_PAIR_MASK_FWD_SPLITS_INDEX],
    {1, input_desc[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].dims.d[0]}, tv::int32, 0);

  tv::Tensor mask_argsort_fwd_splits = tv::from_blob(
    inputs[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX],
    {
      1,
      input_desc[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].dims.d[0],
    },
    tv::int32, 0);

  PLUGIN_ASSERT(
    input_desc[INOUT_PAIR_FWD_INDEX].dims.d[1] ==
    input_desc[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].dims.d[0]);
  PLUGIN_ASSERT(input_desc[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].dims.nbDims == 1);

  tv::Tensor out_features = tv::from_blob(outputs[0], {num_act_out, num_out_features}, dtype, 0);

  // Wrap the optional per-channel bias (BN-folded, 6th ONNX input) into a tv::Tensor.
  // When present, spconv fuses it inside the GEMM kernel instead of a separate bias_add kernel.
  // bias_tv stays empty (default-constructed) when there are only 5 inputs (no bias), in which
  // case implicit_gemm runs without a bias term.
  tv::Tensor bias_tv{};
  if (num_plugin_inputs_ == NUM_PLUGIN_INPUTS_BIAS) {
    auto const bias_type = input_desc[INOUT_OPTIONAL_BIAS_INDEX].type;
    std::int64_t const c_bias = input_desc[INOUT_OPTIONAL_BIAS_INDEX].dims.d[0];
    validate_and_wrap_bias_tensor(
      inputs[INOUT_OPTIONAL_BIAS_INDEX], c_bias, dtype, bias_type, input_features.device(),
      &bias_tv);
  }

  std::vector<tv::Tensor> pair_mask_splits;
  std::vector<tv::Tensor> mask_argsort_splits;

  pair_mask_splits.push_back(pair_mask_fwd_splits);
  mask_argsort_splits.push_back(mask_argsort_fwd_splits);

  std::unordered_map<std::string, tv::Tensor> tensor_dict{
    {SPCONV_ALLOC_FEATURES, input_features},
    {SPCONV_ALLOC_FILTERS, weights},
    {SPCONV_ALLOC_OUT_FEATURES, out_features}};
  StaticAllocator alloc2(tensor_dict);

  auto & tuner_ptr = dtype == tv::float32 ? tuner_fp32_ptr_ : tuner_fp16_ptr_;

  [[maybe_unused]] auto const conv_run_status = ConvGemmOps::implicit_gemm(
    alloc2, *tuner_ptr, input_features, weights, pair_fwd, pair_mask_splits, mask_argsort_splits,
    num_act_out, mask_tensor_, arch_, false, params_.is_subm,  // cSpell:ignore subm
    reinterpret_cast<std::uintptr_t>(stream), tv::CUDAKernelTimer(false), true, false, bias_tv,
    params_.act_alpha, params_.act_beta, activation_from_int(params_.act_type), false, 1.0,
    tv::Tensor(), tv::Tensor(), 0.0, -1);

  return 0;
}

std::int32_t ImplicitGemmPlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == NUM_PLUGIN_INPUTS_NO_BIAS || num_inputs == NUM_PLUGIN_INPUTS_BIAS);
  num_plugin_inputs_ = num_inputs;
  return 0;
}

IPluginV3 * ImplicitGemmPlugin::attachToContext(
  [[maybe_unused]] IPluginResourceContext * context) noexcept
{
  return clone();
}

PluginFieldCollection const * ImplicitGemmPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t ImplicitGemmPlugin::getWorkspaceSize(
  [[maybe_unused]] DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return 0;
}

}  // namespace nvinfer1::plugin
