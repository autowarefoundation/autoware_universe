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
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>  // cSpell:ignore spconvlib
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/convops/SimpleExternalSpconvMatmul.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>
#include <spconvlib/spconv/csrc/sparse/inference/InferenceOps.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
std::atomic<int> g_implicit_gemm_plugin_timing_seq{0};

/** Spconv fused bias/add expects bias tv::Tensor dtype to match activations (see InferenceOps
 * bias_add). */
bool build_bias_tensor_matching_activation(
  tv::Tensor const & input_features, void const * bias_input, std::int64_t c_bias,
  tv::DType activation_dtype, nvinfer1::DataType bias_trt_type, cudaStream_t stream,
  std::string const & layer_name, tv::Tensor * out_bias) noexcept
{
  int const dev = input_features.device();
  if (activation_dtype == tv::float16) {
    if (bias_trt_type == nvinfer1::DataType::kHALF) {
      *out_bias = tv::from_blob(const_cast<void *>(bias_input), {c_bias}, tv::float16, dev);
      return true;
    }
    if (bias_trt_type == nvinfer1::DataType::kFLOAT) {
      tv::Tensor const bias_f32 =
        tv::from_blob(const_cast<void *>(bias_input), {c_bias}, tv::float32, dev);
      *out_bias = tv::empty({c_bias}, tv::float16, dev);
      std::vector<float> host_f(static_cast<std::size_t>(c_bias));
      cudaError_t e0 = cudaMemcpy(
        host_f.data(), bias_f32.data_ptr<float>(), static_cast<std::size_t>(c_bias) * sizeof(float),
        cudaMemcpyDeviceToHost);
      if (e0 != cudaSuccess) {
        std::fprintf(
          stderr, "[ImplicitGemmPlugin] %s: cudaMemcpy bias f32 D2H failed: %s\n",
          layer_name.c_str(), cudaGetErrorString(e0));
        return false;
      }
      std::vector<__half> host_h(static_cast<std::size_t>(c_bias));
      for (std::int64_t i = 0; i < c_bias; ++i) {
        host_h[static_cast<std::size_t>(i)] = __float2half(host_f[static_cast<std::size_t>(i)]);
      }
      cudaError_t e1 = cudaMemcpyAsync(
        out_bias->data_ptr<__half>(), host_h.data(),
        static_cast<std::size_t>(c_bias) * sizeof(__half), cudaMemcpyHostToDevice, stream);
      if (e1 != cudaSuccess) {
        std::fprintf(
          stderr, "[ImplicitGemmPlugin] %s: cudaMemcpyAsync bias f16 H2D failed: %s\n",
          layer_name.c_str(), cudaGetErrorString(e1));
        return false;
      }
      return true;
    }
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: unsupported bias TRT dtype for FP16 activations\n",
      layer_name.c_str());
    return false;
  }
  if (activation_dtype == tv::float32) {
    if (bias_trt_type == nvinfer1::DataType::kFLOAT) {
      *out_bias = tv::from_blob(const_cast<void *>(bias_input), {c_bias}, tv::float32, dev);
      return true;
    }
    if (bias_trt_type == nvinfer1::DataType::kHALF) {
      tv::Tensor const bias_f16 =
        tv::from_blob(const_cast<void *>(bias_input), {c_bias}, tv::float16, dev);
      *out_bias = tv::empty({c_bias}, tv::float32, dev);
      std::vector<__half> host_h(static_cast<std::size_t>(c_bias));
      cudaError_t e0 = cudaMemcpy(
        host_h.data(), bias_f16.data_ptr<__half>(),
        static_cast<std::size_t>(c_bias) * sizeof(__half), cudaMemcpyDeviceToHost);
      if (e0 != cudaSuccess) {
        std::fprintf(
          stderr, "[ImplicitGemmPlugin] %s: cudaMemcpy bias f16 D2H failed: %s\n",
          layer_name.c_str(), cudaGetErrorString(e0));
        return false;
      }
      std::vector<float> host_f(static_cast<std::size_t>(c_bias));
      for (std::int64_t i = 0; i < c_bias; ++i) {
        host_f[static_cast<std::size_t>(i)] = __half2float(host_h[static_cast<std::size_t>(i)]);
      }
      cudaError_t e1 = cudaMemcpyAsync(
        out_bias->data_ptr<float>(), host_f.data(),
        static_cast<std::size_t>(c_bias) * sizeof(float), cudaMemcpyHostToDevice, stream);
      if (e1 != cudaSuccess) {
        std::fprintf(
          stderr, "[ImplicitGemmPlugin] %s: cudaMemcpyAsync bias f32 H2D failed: %s\n",
          layer_name.c_str(), cudaGetErrorString(e1));
        return false;
      }
      return true;
    }
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: unsupported bias TRT dtype for FP32 activations\n",
      layer_name.c_str());
    return false;
  }
  return false;
}

tv::gemm::Activation activation_from_int(std::int32_t const v) noexcept
{
  switch (v) {
    case 0:
      return tv::gemm::Activation::kNone;
    case 1:
      return tv::gemm::Activation::kReLU;
    case 2:
      return tv::gemm::Activation::kSigmoid;
    case 3:
      return tv::gemm::Activation::kLeakyReLU;
    default:
      return tv::gemm::Activation::kNone;
  }
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

ImplicitGemmPlugin::~ImplicitGemmPlugin()
{
  destroyTimingEvents();
}

bool ImplicitGemmPlugin::ensureTimingEvents() noexcept
{
  if (timing_events_created_) {
    return true;
  }
  cudaError_t e0 = cudaEventCreateWithFlags(&timing_ev_implicit_start_, cudaEventDefault);
  cudaError_t e1 = cudaEventCreateWithFlags(&timing_ev_implicit_end_, cudaEventDefault);
  if (e0 != cudaSuccess || e1 != cudaSuccess) {
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: cudaEventCreate failed (%s %s)\n", layer_name_.c_str(),
      cudaGetErrorString(e0), cudaGetErrorString(e1));
    if (e0 == cudaSuccess) {
      cudaEventDestroy(timing_ev_implicit_start_);
    }
    if (e1 == cudaSuccess) {
      cudaEventDestroy(timing_ev_implicit_end_);
    }
    return false;
  }
  timing_events_created_ = true;
  return true;
}

void ImplicitGemmPlugin::destroyTimingEvents() noexcept
{
  if (!timing_events_created_) {
    return;
  }
  cudaEventDestroy(timing_ev_implicit_start_);
  cudaEventDestroy(timing_ev_implicit_end_);
  timing_events_created_ = false;
}

void ImplicitGemmPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back("act_alpha", &params_.act_alpha, PluginFieldType::kFLOAT32, 1);
  data_to_serialize_.emplace_back("act_beta", &params_.act_beta, PluginFieldType::kFLOAT32, 1);

  data_to_serialize_.emplace_back(
    "is_subm", &params_.is_subm, PluginFieldType::kINT32, 1);  // cSpell:ignore subm
  data_to_serialize_.emplace_back("is_train", &params_.is_train, PluginFieldType::kINT32, 1);

  data_to_serialize_.emplace_back(
    "output_add_scale", &params_.output_add_scale, PluginFieldType::kFLOAT32, 1);
  data_to_serialize_.emplace_back(
    "output_scale", &params_.output_scale, PluginFieldType::kFLOAT32, 1);
  data_to_serialize_.emplace_back(
    "timing_enabled", &params_.timing_enabled, PluginFieldType::kINT32, 1);
  data_to_serialize_.emplace_back(
    "timing_max_logs", &params_.timing_max_logs, PluginFieldType::kINT32, 1);
  data_to_serialize_.emplace_back("act_type", &params_.act_type, PluginFieldType::kINT32, 1);

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
  // Validate input arguments (5 = legacy; 6 = optional per-channel bias for ONNX-fused Add).
  // Never abort: TensorRT probes invalid combinations during build; return an error code instead.
  if (in == nullptr || out == nullptr) {
    return -1;
  }
  if (num_inputs != 5 && num_inputs != 6) {
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: configurePlugin expected 5 or 6 inputs, got %d\n",
      layer_name_.c_str(), static_cast<int>(num_inputs));
    return -1;
  }
  if (num_outputs != 1) {
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: configurePlugin expected 1 output, got %d\n",
      layer_name_.c_str(), static_cast<int>(num_outputs));
    return -1;
  }

  num_plugin_inputs_ = num_inputs;

  if (
    in[INOUT_IN_FEATURES_INDEX].desc.dims.nbDims != 2 ||
    in[INOUT_FILTERS_INDEX].desc.dims.nbDims != 5 ||
    in[INOUT_PAIR_FWD_INDEX].desc.dims.nbDims != 2 ||
    in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.dims.nbDims != 2 ||
    in[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].desc.dims.nbDims != 1 || out[0].desc.dims.nbDims != 2) {
    std::fprintf(
      stderr,
      "[ImplicitGemmPlugin] %s: configurePlugin unexpected tensor ranks (features/filters/pairs)\n",
      layer_name_.c_str());
    return -1;
  }

  if (
    in[INOUT_FILTERS_INDEX].desc.dims.d[4] != in[INOUT_IN_FEATURES_INDEX].desc.dims.d[1] ||
    in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.dims.d[0] !=
      in[INOUT_PAIR_FWD_INDEX].desc.dims.d[1] ||
    in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.dims.d[1] != 1 ||
    in[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].desc.dims.d[0] !=
      in[INOUT_PAIR_FWD_INDEX].desc.dims.d[1]) {
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: configurePlugin dimension mismatch (pairs vs features)\n",
      layer_name_.c_str());
    return -1;
  }

  if (
    in[INOUT_IN_FEATURES_INDEX].desc.type != in[INOUT_FILTERS_INDEX].desc.type ||
    in[INOUT_IN_FEATURES_INDEX].desc.type != out[0].desc.type ||
    in[INOUT_PAIR_FWD_INDEX].desc.type != in[INOUT_PAIR_MASK_FWD_SPLITS_INDEX].desc.type ||
    in[INOUT_PAIR_FWD_INDEX].desc.type != in[INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX].desc.type) {
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: configurePlugin dtype mismatch between IO tensors\n",
      layer_name_.c_str());
    return -1;
  }

  if (num_inputs == 6) {
    if (
      in[INOUT_OPTIONAL_BIAS_INDEX].desc.dims.nbDims != 1 ||
      in[INOUT_OPTIONAL_BIAS_INDEX].desc.dims.d[0] != in[INOUT_FILTERS_INDEX].desc.dims.d[0]) {
      std::fprintf(
        stderr,
        "[ImplicitGemmPlugin] %s: configurePlugin optional bias must be [C_out], matching filters "
        "dim 0\n",
        layer_name_.c_str());
      return -1;
    }
    DataType const bias_t = in[INOUT_OPTIONAL_BIAS_INDEX].desc.type;
    if (bias_t != DataType::kFLOAT && bias_t != DataType::kHALF) {
      std::fprintf(
        stderr, "[ImplicitGemmPlugin] %s: configurePlugin optional bias must be FLOAT or HALF\n",
        layer_name_.c_str());
      return -1;
    }
  }

  return 0;
}

bool ImplicitGemmPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  std::int32_t num_outputs) noexcept
{
  // TRT calls this for many (pos, format) pairs; must never abort.
  if (in_out == nullptr) {
    return false;
  }
  if (num_outputs != 1) {
    return false;
  }
  if (num_inputs != 5 && num_inputs != 6) {
    return false;
  }
  std::int32_t const n_slots = num_inputs + num_outputs;
  if (pos < 0 || pos >= n_slots) {
    return false;
  }

  bool supported = in_out[pos].desc.format == nvinfer1::TensorFormat::kLINEAR;

  std::int32_t const out_pos = num_inputs;
  if (pos == out_pos) {
    supported &= in_out[pos].desc.type == in_out[INOUT_IN_FEATURES_INDEX].desc.type;
    return supported;
  }

  switch (pos) {
    case INOUT_IN_FEATURES_INDEX:
      supported &=
        (in_out[pos].desc.type == nvinfer1::DataType::kFLOAT ||
         in_out[pos].desc.type == nvinfer1::DataType::kHALF);
      break;
    case INOUT_FILTERS_INDEX:
      supported &= in_out[pos].desc.type == in_out[INOUT_IN_FEATURES_INDEX].desc.type;
      break;
    case INOUT_PAIR_FWD_INDEX:
    case INOUT_PAIR_MASK_FWD_SPLITS_INDEX:
    case INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX:
      supported &= in_out[pos].desc.type == nvinfer1::DataType::kINT32;
      break;
    case INOUT_OPTIONAL_BIAS_INDEX:
      if (num_inputs == 6) {
        supported &=
          (in_out[pos].desc.type == in_out[INOUT_IN_FEATURES_INDEX].desc.type ||
           in_out[pos].desc.type == nvinfer1::DataType::kFLOAT);
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
  if (output_types == nullptr || input_types == nullptr) {
    return -1;
  }
  if (num_outputs != 1 || (num_inputs != 5 && num_inputs != 6)) {
    return -1;
  }

  output_types[0] = input_types[INOUT_IN_FEATURES_INDEX];

  return 0;
}

std::int32_t ImplicitGemmPlugin::getOutputShapes(
  DimsExprs const * inputs, std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, std::int32_t num_outputs,
  [[maybe_unused]] IExprBuilder & expr_builder) noexcept
{
  if (inputs == nullptr || outputs == nullptr) {
    return -1;
  }
  if (num_outputs != 1 || (num_inputs != 5 && num_inputs != 6)) {
    return -1;
  }
  if (inputs[0].nbDims != 2) {
    return -1;
  }

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

  if (num_plugin_inputs_ != 5 && num_plugin_inputs_ != 6) {
    std::fprintf(
      stderr,
      "[ImplicitGemmPlugin] %s: enqueue expected 5 or 6 inputs (set in configure/onShapeChange), "
      "got num_plugin_inputs_=%d\n",
      layer_name_.c_str(), static_cast<int>(num_plugin_inputs_));
    return -1;
  }

  std::int64_t num_act_in = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[0];
  std::int64_t num_in_features = input_desc[INOUT_IN_FEATURES_INDEX].dims.d[1];
  // std::int64_t kernel_volume = input_desc[INOUT_PAIR_FWD_INDEX].dims.d[0];
  std::int64_t num_act_out = input_desc[INOUT_PAIR_FWD_INDEX].dims.d[1];
  std::int64_t num_out_features = input_desc[INOUT_FILTERS_INDEX].dims.d[0];

  auto in_features_type = input_desc[INOUT_IN_FEATURES_INDEX].type;
  [[maybe_unused]] auto filters_type = input_desc[INOUT_FILTERS_INDEX].type;
  [[maybe_unused]] auto out_features_type = output_desc[0].type;

  if (in_features_type != filters_type || in_features_type != out_features_type) {
    std::fprintf(
      stderr, "[ImplicitGemmPlugin] %s: enqueue dtype mismatch (features/filters/out)\n",
      layer_name_.c_str());
    return -1;
  }

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

  tv::Tensor bias_tv{};
  if (num_plugin_inputs_ >= 6) {
    auto const bias_type = input_desc[INOUT_OPTIONAL_BIAS_INDEX].type;
    std::int64_t const c_bias = input_desc[INOUT_OPTIONAL_BIAS_INDEX].dims.d[0];
    if (!build_bias_tensor_matching_activation(
          input_features, inputs[INOUT_OPTIONAL_BIAS_INDEX], c_bias, dtype, bias_type, stream,
          layer_name_, &bias_tv)) {
      return -1;
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

  bool do_timing = false;
  int timing_seq = -1;
  if (params_.timing_enabled != 0) {
    timing_seq = g_implicit_gemm_plugin_timing_seq.fetch_add(1, std::memory_order_relaxed);
    do_timing = timing_seq < params_.timing_max_logs && ensureTimingEvents();
  }
  if (do_timing) {
    cudaEventRecord(timing_ev_implicit_start_, stream);
  }

  [[maybe_unused]] auto const conv_run_status = ConvGemmOps::implicit_gemm(
    alloc2, *tuner_ptr, input_features, weights, pair_fwd, pair_mask_splits, mask_argsort_splits,
    num_act_out, mask_tensor_, arch_, false, params_.is_subm,
    reinterpret_cast<std::uintptr_t>(stream), tv::CUDAKernelTimer(false), true, false, bias_tv, 0.0,
    0.0, activation_from_int(params_.act_type), false, 1.0, tv::Tensor(), tv::Tensor(), 0.0, -1);

  if (do_timing) {
    cudaEventRecord(timing_ev_implicit_end_, stream);
    cudaError_t sync_st = cudaEventSynchronize(timing_ev_implicit_end_);
    if (sync_st != cudaSuccess) {
      std::fprintf(
        stderr, "[AUTOWARE_IMPLICIT_GEMM_TIMING] seq=%d layer=%s cudaEventSynchronize failed: %s\n",
        timing_seq, layer_name_.c_str(), cudaGetErrorString(sync_st));
    } else {
      float ms_implicit = 0.F;
      cudaEventElapsedTime(&ms_implicit, timing_ev_implicit_start_, timing_ev_implicit_end_);
      char const * dt = in_features_type == DataType::kHALF ? "fp16" : "fp32";
      std::fprintf(
        stderr,
        "[AUTOWARE_IMPLICIT_GEMM_TIMING] seq=%d layer=%s dtype=%s implicit_gemm_ms=%.5f "
        "(sparse conv plugin time)\n",
        timing_seq, layer_name_.c_str(), dt, ms_implicit);
    }
  }

  return 0;
}

std::int32_t ImplicitGemmPlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  if (num_inputs == 5 || num_inputs == 6) {
    num_plugin_inputs_ = num_inputs;
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

}  // namespace nvinfer1::plugin
