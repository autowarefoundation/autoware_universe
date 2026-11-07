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

#include "autoware/implicit_gemm_ops/epilogue_constants.hpp"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_fp16.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>  // cSpell:ignore spconvlib
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/convops/SimpleExternalSpconvMatmul.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>
#include <spconvlib/spconv/csrc/sparse/inference/InferenceOps.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
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

char const * type_name(nvinfer1::DataType const type) noexcept
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT:
      return "FP32";
    case nvinfer1::DataType::kHALF:
      return "FP16";
    case nvinfer1::DataType::kINT8:
      return "INT8";
    case nvinfer1::DataType::kINT32:
      return "INT32";
    case nvinfer1::DataType::kFP8:
      return "FP8";
    case nvinfer1::DataType::kBF16:
      return "BF16";
    default:
      return "an unsupported type";
  }
}

/// Log why this layer cannot be built, naming the layer.
void refuse(std::string const & layer_name, std::string const & reason)
{
  std::string const message = "ImplicitGemm " + layer_name + ": " + reason;
  logError(message.c_str());
}

/// Length of a 1-D input, or -1 when the input is not 1-D.
std::int64_t vector_length(nvinfer1::Dims const & dims) noexcept
{
  return dims.nbDims == 1 ? dims.d[0] : -1;
}

bool valid_num_inputs(std::int32_t const num_inputs) noexcept
{
  return num_inputs >= 5 && num_inputs <= 7;
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
  if (params_.has_input_scale != 0) {
    tuner_int8_ptr_ = std::make_unique<ConvTunerSimple>(ConvMain::get_all_conv_algo_desp());
  }

  // Pre-allocate CPU mask tensor to avoid heap allocation during CUDA graph capture.
  mask_tensor_ = tv::zeros({1}, tv::uint32, -1);
  mask_tensor_.data_ptr<uint32_t>()[0] = 0xffffffff;
}

ImplicitGemmPlugin::~ImplicitGemmPlugin()
{
  releaseEpilogueConstants();
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

bool ImplicitGemmPlugin::checkContract(
  DataType const * input_types, std::int32_t num_inputs, bool log) const noexcept
{
  auto const fail = [&](std::string const & reason) {
    if (log) {
      refuse(layer_name_, reason);
    }
    return false;
  };

  if (!valid_num_inputs(num_inputs)) {
    return fail(
      std::to_string(num_inputs) + " inputs; expected 5, 6 (bias) or 7 (weight_scale, bias)");
  }
  DataType const features = input_types[INOUT_IN_FEATURES_INDEX];
  DataType const filters = input_types[INOUT_FILTERS_INDEX];

  if (features == DataType::kINT8) {
    if (num_inputs != NUM_PLUGIN_INPUTS_SCALE_BIAS) {
      return fail(
        "INT8 features need 7 inputs (weight_scale at 5, bias at 6), got " +
        std::to_string(num_inputs));
    }
    if (filters != DataType::kINT8) {
      return fail(std::string("INT8 features need INT8 filters, got ") + type_name(filters));
    }
    if (params_.has_input_scale == 0) {
      return fail(
        "INT8 features need the attribute input_scale (the scale of the QuantizeLinear in "
        "front)");
    }
    if (params_.output_type != kIMPLICIT_GEMM_OUTPUT_TYPE_F16) {
      return fail("INT8 features need the attribute output_type \"f16\"");
    }
    return true;
  }

  if (features != DataType::kFLOAT && features != DataType::kHALF) {
    return fail(
      std::string("the features are ") + type_name(features) +
      "; this plugin runs FP32, FP16 or INT8 features");
  }
  if (filters != features) {
    return fail(
      std::string("the filters are ") + type_name(filters) + " but the features are " +
      type_name(features));
  }
  if (params_.has_input_scale != 0) {
    return fail(
      std::string("input_scale is set but the features are ") + type_name(features) +
      ": a quantized node takes INT8 features from a QuantizeLinear");
  }
  if (params_.output_type != kIMPLICIT_GEMM_OUTPUT_TYPE_UNSET) {
    return fail(
      std::string("output_type is set but the features are ") + type_name(features) +
      ": only a quantized (INT8) node carries it");
  }
  return true;
}

std::int32_t ImplicitGemmPlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(in != nullptr);
  PLUGIN_ASSERT(out != nullptr);
  PLUGIN_ASSERT(num_outputs == 1);

  if (!valid_num_inputs(num_inputs)) {
    refuse(layer_name_, std::to_string(num_inputs) + " inputs; expected 5, 6 or 7");
    return -1;
  }
  DataType input_types[NUM_PLUGIN_INPUTS_SCALE_BIAS];
  for (std::int32_t i = 0; i < num_inputs; ++i) {
    input_types[i] = in[i].desc.type;
  }
  if (!checkContract(input_types, num_inputs, true)) {
    return -1;
  }
  num_plugin_inputs_ = num_inputs;

  // TODO(vividf): should use PLUGIN_VALIDATE_AND_RETURN instead of PLUGIN_ASSERT for input
  // arguments
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
  PLUGIN_ASSERT(
    in[INOUT_PAIR_FWD_INDEX].desc.type == in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.type);
  PLUGIN_ASSERT(
    in[INOUT_PAIR_FWD_INDEX].desc.type == in[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].desc.type);
  PLUGIN_ASSERT(
    params_.act_type >= static_cast<std::int32_t>(tv::gemm::Activation::kNone) &&
    params_.act_type <= static_cast<std::int32_t>(tv::gemm::Activation::kLeakyReLU));

  bool const is_int8 = in[INOUT_IN_FEATURES_INDEX].desc.type == DataType::kINT8;
  DataType const out_type = is_int8 ? DataType::kHALF : in[INOUT_IN_FEATURES_INDEX].desc.type;
  if (out[0].desc.type != out_type) {
    refuse(
      layer_name_, std::string("the output is ") + type_name(out[0].desc.type) + ", expected " +
                     type_name(out_type));
    return -1;
  }
  std::int64_t const c_out = in[INOUT_FILTERS_INDEX].desc.dims.d[0];

  if (num_inputs == NUM_PLUGIN_INPUTS_BIAS) {
    PLUGIN_ASSERT(in[INOUT_OPTIONAL_BIAS_INDEX].desc.dims.nbDims == 1);
    PLUGIN_ASSERT(in[INOUT_OPTIONAL_BIAS_INDEX].desc.dims.d[0] == c_out);
    PLUGIN_ASSERT(in[INOUT_OPTIONAL_BIAS_INDEX].desc.type == in[INOUT_IN_FEATURES_INDEX].desc.type);
    return 0;
  }
  if (num_inputs == NUM_PLUGIN_INPUTS_SCALE_BIAS) {
    std::int64_t const scale_length = vector_length(in[INOUT_WEIGHT_SCALE_INDEX].desc.dims);
    std::int64_t const bias_length = vector_length(in[INOUT_BIAS_INDEX].desc.dims);
    if (is_int8 && scale_length != c_out) {
      refuse(
        layer_name_, "INT8 features need input 5 (weight_scale) as [C_out=" +
                       std::to_string(c_out) + "], got [" + std::to_string(scale_length) + "]");
      return -1;
    }
    if (!is_int8 && scale_length != 0) {
      refuse(
        layer_name_, std::string(type_name(in[INOUT_IN_FEATURES_INDEX].desc.type)) +
                       " features take no weight_scale: input 5 must be [0], got [" +
                       std::to_string(scale_length) + "]");
      return -1;
    }
    if (bias_length != 0 && bias_length != c_out) {
      refuse(
        layer_name_, "input 6 (bias) must be [C_out=" + std::to_string(c_out) + "] or [0], got [" +
                       std::to_string(bias_length) + "]");
      return -1;
    }
    return allocateEpilogueConstants(c_out);
  }
  return 0;
}

bool ImplicitGemmPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(in_out != nullptr);
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(pos >= 0 && pos < num_inputs + num_outputs);

  // spconv only supports contiguous (LINEAR) layout for all tensors.
  if (!valid_num_inputs(num_inputs) || in_out[pos].desc.format != TensorFormat::kLINEAR) {
    return false;
  }
  DataType const type = in_out[pos].desc.type;
  DataType const features = in_out[INOUT_IN_FEATURES_INDEX].desc.type;

  // The output is FP16 for INT8 features (the epilogue dequantizes), else the features' type.
  if (pos == num_inputs) {
    return type == (features == DataType::kINT8 ? DataType::kHALF : features);
  }

  switch (pos) {
    // A quantized node (input_scale set) takes INT8 features only: offered FP16 as well, a weakly
    // typed build may hand it the QuantizeLinear's input instead of its output.
    case INOUT_IN_FEATURES_INDEX:
      if (params_.has_input_scale != 0) {
        return type == DataType::kINT8 && num_inputs == NUM_PLUGIN_INPUTS_SCALE_BIAS;
      }
      return type == DataType::kFLOAT || type == DataType::kHALF;
    // Filter weights share the features' type.
    case INOUT_FILTERS_INDEX:
      return type == features;
    // Index / mask tensors are always int32 regardless of activation dtype.
    case INOUT_PAIR_FWD_INDEX:
    case INOUT_PAIR_MASK_FWD_SPLITS_INDEX:
    case INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX:
      return type == DataType::kINT32;
    // Six inputs: the bias in the features' type. Seven: FP32 weight_scale and bias, whatever the
    // features are; letting them follow the features would make TensorRT cast them.
    case INOUT_OPTIONAL_BIAS_INDEX:
      return num_inputs == NUM_PLUGIN_INPUTS_BIAS ? type == features : type == DataType::kFLOAT;
    case INOUT_BIAS_INDEX:
      return type == DataType::kFLOAT;
    default:
      return false;
  }
}

std::int32_t ImplicitGemmPlugin::getOutputDataTypes(
  DataType * output_types, std::int32_t num_outputs, DataType const * input_types,
  std::int32_t num_inputs) const noexcept
{
  PLUGIN_ASSERT(output_types != nullptr);
  PLUGIN_ASSERT(input_types != nullptr);
  PLUGIN_ASSERT(num_outputs == 1);

  // The input types are known here, before TensorRT negotiates formats: refusing now names the
  // reason instead of ending in TensorRT's generic "no supported formats" error.
  if (!checkContract(input_types, num_inputs, true)) {
    return -1;
  }
  output_types[0] = input_types[INOUT_IN_FEATURES_INDEX] == DataType::kINT8
                      ? DataType::kHALF
                      : input_types[INOUT_IN_FEATURES_INDEX];

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
  PLUGIN_ASSERT(num_outputs == 1);
  if (!valid_num_inputs(num_inputs) || inputs[0].nbDims != 2) {
    return -1;
  }

  // num_act_out from pair_mask_fwd (index 3, dim 0); C_out from filters (index 1, dim 0).
  outputs[0].nbDims = 2;
  outputs[0].d[0] = inputs[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].d[0];
  outputs[0].d[1] = inputs[INOUT_FILTERS_INDEX].d[0];

  return 0;
}

std::int32_t ImplicitGemmPlugin::enqueue(
  PluginTensorDesc const * input_desc, [[maybe_unused]] PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;
  using ConvGemmOps = spconvlib::spconv::csrc::sparse::convops::spops::ConvGemmOps;

  PLUGIN_ASSERT(valid_num_inputs(num_plugin_inputs_));
  if (input_desc[INOUT_IN_FEATURES_INDEX].type == DataType::kINT8) {
    return enqueueInt8(input_desc, inputs, outputs, stream);
  }

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

  // Wrap the optional per-channel bias (BN-folded, 6th or 7th ONNX input) into a tv::Tensor.
  // When present, spconv fuses it inside the GEMM kernel instead of a separate bias_add kernel.
  // bias_tv stays empty (default-constructed) without a bias, in which case implicit_gemm runs
  // without a bias term.
  tv::Tensor bias_tv{};
  if (num_plugin_inputs_ == NUM_PLUGIN_INPUTS_BIAS) {
    auto const bias_type = input_desc[INOUT_OPTIONAL_BIAS_INDEX].type;
    std::int64_t const c_bias = input_desc[INOUT_OPTIONAL_BIAS_INDEX].dims.d[0];
    validate_and_wrap_bias_tensor(
      inputs[INOUT_OPTIONAL_BIAS_INDEX], c_bias, dtype, bias_type, input_features.device(),
      &bias_tv);
  } else if (
    num_plugin_inputs_ == NUM_PLUGIN_INPUTS_SCALE_BIAS &&
    vector_length(input_desc[INOUT_BIAS_INDEX].dims) > 0) {
    // Seven inputs: the bias is FP32. Spconv's FP16 kernels add a bias of the activation dtype,
    // so an FP16 layer adds a copy converted once.
    PLUGIN_ASSERT(input_desc[INOUT_BIAS_INDEX].type == DataType::kFLOAT);
    std::int64_t const c_bias = input_desc[INOUT_BIAS_INDEX].dims.d[0];
    PLUGIN_ASSERT(c_bias == num_out_features);
    if (dtype == tv::float32) {
      bias_tv = tv::from_blob(
        const_cast<void *>(inputs[INOUT_BIAS_INDEX]), {c_bias}, tv::float32,
        input_features.device());
    } else {
      if (allocateEpilogueConstants(c_bias) != 0) {
        return -1;
      }
      std::lock_guard<std::mutex> lock(epilogue_constants_mutex_);
      if (!epilogue_constants_filled_) {
        launch_float_to_half(
          static_cast<float const *>(inputs[INOUT_BIAS_INDEX]),
          static_cast<__half *>(epilogue_constants_ptr_), c_bias, stream);
        if (PLUGIN_CUDA_CHECK(cudaGetLastError()) != cudaSuccess) {
          return -1;
        }
        epilogue_constants_filled_ = true;
      }
      bias_tv =
        tv::from_blob(epilogue_constants_ptr_, {c_bias}, tv::float16, input_features.device());
    }
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

std::int32_t ImplicitGemmPlugin::enqueueInt8(
  PluginTensorDesc const * input_desc, void const * const * inputs, void * const * outputs,
  cudaStream_t stream) noexcept
{
  using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;
  using ConvGemmOps = spconvlib::spconv::csrc::sparse::convops::spops::ConvGemmOps;

  PLUGIN_ASSERT(num_plugin_inputs_ == NUM_PLUGIN_INPUTS_SCALE_BIAS);
  PLUGIN_ASSERT(input_desc[INOUT_FILTERS_INDEX].type == DataType::kINT8);
  PLUGIN_ASSERT(tuner_int8_ptr_ != nullptr);

  std::int64_t const num_act_in = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[0];
  std::int64_t const c_in = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[1];
  std::int64_t const c_out = input_desc[INOUT_FILTERS_INDEX].dims.d[0];
  std::int64_t const k1 = input_desc[INOUT_FILTERS_INDEX].dims.d[1];
  std::int64_t const k2 = input_desc[INOUT_FILTERS_INDEX].dims.d[2];
  std::int64_t const k3 = input_desc[INOUT_FILTERS_INDEX].dims.d[3];
  std::int64_t const num_act_out = input_desc[INOUT_PAIR_FWD_INDEX].dims.d[1];

  // The GEMM's per-channel scale, input_scale * weight_scale[n], derived once: the s8s8f16
  // epilogue multiplies by a per-channel scale only, never by a scalar alpha. Spconv's INT8
  // kernels also need a bias tensor (without one the tuner finds no algorithm), so a node
  // without a bias gets zeros.
  bool const has_bias = vector_length(input_desc[INOUT_BIAS_INDEX].dims) > 0;
  if (allocateEpilogueConstants(c_out) != 0) {
    return -1;
  }
  float * const gemm_scale = static_cast<float *>(epilogue_constants_ptr_);
  float * const zero_bias = gemm_scale + c_out;
  {
    std::lock_guard<std::mutex> lock(epilogue_constants_mutex_);
    if (!epilogue_constants_filled_) {
      launch_scale_per_channel(
        static_cast<float const *>(inputs[INOUT_WEIGHT_SCALE_INDEX]), params_.input_scale,
        gemm_scale, c_out, stream);
      if (PLUGIN_CUDA_CHECK(cudaGetLastError()) != cudaSuccess) {
        return -1;
      }
      if (
        !has_bias &&
        PLUGIN_CUDA_CHECK(cudaMemsetAsync(
          zero_bias, 0, static_cast<std::size_t>(c_out) * sizeof(float), stream)) != cudaSuccess) {
        return -1;
      }
      epilogue_constants_filled_ = true;
    }
  }

  // Features (from the graph's QuantizeLinear) and filters are INT8 already.
  tv::Tensor features_tv = tv::from_blob(
    const_cast<void *>(inputs[INOUT_IN_FEATURES_INDEX]), {num_act_in, c_in}, tv::int8, 0);
  tv::Tensor weights_tv = tv::from_blob(
    const_cast<void *>(inputs[INOUT_FILTERS_INDEX]), {c_out, k1, k2, k3, c_in}, tv::int8, 0);
  tv::Tensor pair_fwd = tv::from_blob(
    inputs[INOUT_PAIR_FWD_INDEX],
    {input_desc[INOUT_PAIR_FWD_INDEX].dims.d[0], input_desc[INOUT_PAIR_FWD_INDEX].dims.d[1]},
    tv::int32, 0);
  tv::Tensor pair_mask_fwd = tv::from_blob(
    inputs[INOUT_PAIR_MASK_FWD_SPLITS_INDEX],
    {1, input_desc[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].dims.d[0]}, tv::int32, 0);
  tv::Tensor mask_argsort_fwd = tv::from_blob(
    inputs[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX],
    {1, input_desc[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].dims.d[0]}, tv::int32, 0);
  tv::Tensor out_features = tv::from_blob(outputs[0], {num_act_out, c_out}, tv::float16, 0);
  tv::Tensor channel_scale_tv = tv::from_blob(gemm_scale, {c_out}, tv::float32, 0);
  tv::Tensor bias_tv = tv::from_blob(
    has_bias ? const_cast<void *>(inputs[INOUT_BIAS_INDEX]) : static_cast<void *>(zero_bias),
    {c_out}, tv::float32, 0);

  std::vector<tv::Tensor> pair_mask_splits{pair_mask_fwd};
  std::vector<tv::Tensor> mask_argsort_splits{mask_argsort_fwd};

  std::unordered_map<std::string, tv::Tensor> tensor_dict{
    {SPCONV_ALLOC_FEATURES, features_tv},
    {SPCONV_ALLOC_FILTERS, weights_tv},
    {SPCONV_ALLOC_OUT_FEATURES, out_features}};
  StaticAllocator alloc(tensor_dict);

  ConvGemmOps::implicit_gemm(
    alloc, *tuner_int8_ptr_, features_tv, weights_tv, pair_fwd, pair_mask_splits,
    mask_argsort_splits, static_cast<int>(num_act_out), mask_tensor_, arch_,
    /*is_train=*/false, /*is_subm=*/static_cast<bool>(params_.is_subm),
    reinterpret_cast<std::uintptr_t>(stream), tv::CUDAKernelTimer(false),
    /*auto_fp32_accum=*/true, /*fp32_accum=*/false, /*bias=*/bias_tv, params_.act_alpha,
    params_.act_beta, activation_from_int(params_.act_type), /*use_tf32=*/false,
    /*output_scale=*/1.0f, /*scale=*/channel_scale_tv, /*output_add=*/tv::Tensor(),
    /*output_add_scale=*/0.0f, /*output_dtype=*/static_cast<int>(tv::float16));

  return 0;
}

std::int32_t ImplicitGemmPlugin::onShapeChange(
  PluginTensorDesc const * in, std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  if (!valid_num_inputs(num_inputs)) {
    refuse(layer_name_, std::to_string(num_inputs) + " inputs; expected 5, 6 or 7");
    return -1;
  }
  num_plugin_inputs_ = num_inputs;

  // Runtime path: configurePlugin (build-only) is not called here, so allocate the epilogue
  // constants now that the shapes are known and we are outside any CUDA graph capture.
  if (num_inputs == NUM_PLUGIN_INPUTS_SCALE_BIAS && in != nullptr) {
    return allocateEpilogueConstants(in[INOUT_FILTERS_INDEX].dims.d[0]);
  }
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

std::int32_t ImplicitGemmPlugin::allocateEpilogueConstants(std::int64_t c_out) noexcept
{
  std::lock_guard<std::mutex> lock(epilogue_constants_mutex_);
  if (epilogue_constants_ptr_ != nullptr && epilogue_constants_c_out_ == c_out) {
    return 0;
  }
  if (c_out <= 0) {
    refuse(layer_name_, "invalid C_out " + std::to_string(c_out) + " for the epilogue constants");
    return -1;
  }
  releaseEpilogueConstants();

  // Room for either use: the INT8 GEMM's FP32 scale and zero bias, or an FP16 copy of the bias.
  cudaError_t const status =
    cudaMalloc(&epilogue_constants_ptr_, static_cast<std::size_t>(2 * c_out) * sizeof(float));
  if (status != cudaSuccess) {
    refuse(layer_name_, std::string("cudaMalloc failed: ") + cudaGetErrorString(status));
    epilogue_constants_ptr_ = nullptr;
    return -1;
  }
  epilogue_constants_c_out_ = c_out;
  epilogue_constants_filled_ = false;
  return 0;
}

void ImplicitGemmPlugin::releaseEpilogueConstants() noexcept
{
  if (epilogue_constants_ptr_ != nullptr) {
    cudaFree(epilogue_constants_ptr_);
    epilogue_constants_ptr_ = nullptr;
  }
  epilogue_constants_c_out_ = 0;
  epilogue_constants_filled_ = false;
}

}  // namespace nvinfer1::plugin
