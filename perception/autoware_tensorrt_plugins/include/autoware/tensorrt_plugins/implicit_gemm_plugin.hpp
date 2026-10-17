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

#ifndef AUTOWARE__TENSORRT_PLUGINS__IMPLICIT_GEMM_PLUGIN_HPP_
#define AUTOWARE__TENSORRT_PLUGINS__IMPLICIT_GEMM_PLUGIN_HPP_

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_runtime.h>
#include <spconvlib/cumm/conv/main/ConvMainUnitTest.h>  // cSpell:ignore spconvlib cumm
#include <spconvlib/spconv/csrc/sparse/convops/gemmops/GemmTunerSimple.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

constexpr char const * const kIMPLICIT_GEMM_PLUGIN_NAME{"ImplicitGemm"};
constexpr char const * const kIMPLICIT_GEMM_PLUGIN_VERSION{"1"};
constexpr char const * const kIMPLICIT_GEMM_PLUGIN_NAMESPACE{""};

namespace nvinfer1::plugin
{

struct ImplicitGemmParameters
{
  float act_alpha{0.0F};
  float act_beta{0.0F};
  std::int32_t is_subm{0};  // cSpell:ignore subm
  std::int32_t is_train{0};
  float output_add_scale{1.0F};
  float output_scale{1.0F};

  /// tv::gemm::Activation as integer:
  /// kNone=0, kReLU=1, kSigmoid=2, kLeakyReLU=3.
  std::int32_t act_type{static_cast<std::int32_t>(tv::gemm::Activation::kNone)};

  /// Precision mode: 0 = floating point (FP16/FP32), 1 = INT8.
  /// In INT8 mode the node carries two extra FP32 inputs (channel_scale, bias_scaled),
  /// FP16 features/weights are quantized to INT8 inside enqueue, and the GEMM is run with
  /// per-output-channel scales (see sparse_int8_onnx_transform.py / Path B).
  std::int32_t precision{0};
  /// INT8 only: input_amax / 127, used to quantize the FP16 features to INT8.
  float input_scale{1.0F};
};

/// Precision values for ImplicitGemmParameters::precision.
constexpr std::int32_t kIMPLICIT_GEMM_PRECISION_FP{0};
constexpr std::int32_t kIMPLICIT_GEMM_PRECISION_INT8{1};

class ImplicitGemmPlugin : public IPluginV3,
                           public IPluginV3OneCore,
                           public IPluginV3OneBuild,
                           public IPluginV3OneRuntime
{
public:
  using ConvTunerSimple = spconvlib::spconv::csrc::sparse::convops::spops::ConvTuner;
  ImplicitGemmPlugin(const std::string & name, ImplicitGemmParameters const & params);

  ~ImplicitGemmPlugin() override;

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
  static constexpr std::int32_t INOUT_IN_FEATURES_INDEX{0};
  static constexpr std::int32_t INOUT_FILTERS_INDEX{1};
  static constexpr std::int32_t INOUT_PAIR_FWD_INDEX{2};
  static constexpr std::int32_t INOUT_PAIR_MASK_FWD_SPLITS_INDEX{3};
  static constexpr std::int32_t INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX{4};
  /// FP path (precision==0): optional per-channel bias at input index 5 (num_inputs == 6).
  static constexpr std::int32_t INOUT_OPTIONAL_BIAS_INDEX{5};
  /// INT8 path (precision==1): 7 inputs total — FP32 per-output-channel scale and bias.
  static constexpr std::int32_t INOUT_CHANNEL_SCALE_INDEX{5};
  static constexpr std::int32_t INOUT_BIAS_SCALED_INDEX{6};

  static constexpr std::int32_t NUM_INPUTS_INT8{7};

  static constexpr std::int32_t NUM_PLUGIN_INPUTS_NO_BIAS{5};
  static constexpr std::int32_t NUM_PLUGIN_INPUTS_BIAS{6};

  void initFieldsToSerialize();

  bool is_int8() const noexcept { return params_.precision == kIMPLICIT_GEMM_PRECISION_INT8; }

  // INT8 path helpers (no-ops / unused in FP path).
  std::int32_t enqueueInt8(
    PluginTensorDesc const * input_desc, void const * const * inputs, void * const * outputs,
    void * workspace, cudaStream_t stream) noexcept;
  /// Allocate the persistent INT8 weight/scale/bias cache for a (constant) filter shape.
  /// Called from ``configurePlugin`` (build) and ``onShapeChange`` (runtime) so the cudaMalloc
  /// happens outside CUDA graph capture; the quantization kernels that fill it run on the first
  /// enqueue (filter device data is only available there). Idempotent for an unchanged shape.
  std::int32_t allocateConstantCache(
    std::int64_t c_out, std::int64_t k1, std::int64_t k2, std::int64_t k3,
    std::int64_t c_in) noexcept;
  void releaseConstantCache() noexcept;

  std::string layer_name_;
  ImplicitGemmParameters params_;
  /// Set in ``configurePlugin`` / ``onShapeChange``.
  std::int32_t num_plugin_inputs_{NUM_PLUGIN_INPUTS_NO_BIAS};
  std::tuple<int, int> arch_;
  std::vector<nvinfer1::PluginField> data_to_serialize_;
  nvinfer1::PluginFieldCollection fc_to_serialize_;

  std::unique_ptr<ConvTunerSimple> tuner_fp32_ptr_{};
  std::unique_ptr<ConvTunerSimple> tuner_fp16_ptr_{};
  std::unique_ptr<ConvTunerSimple> tuner_int8_ptr_{};

  // Pre-allocated CPU mask tensor to avoid heap allocation during CUDA graph capture.
  tv::Tensor mask_tensor_;

  // ── INT8 persistent constant cache (precision==1 only) ──────────────────────────────────
  // Filters / scales / bias are graph constants, so we quantize them once and reuse the device
  // buffers across enqueues. Buffers are cudaMalloc'd in configurePlugin and filled lazily on the
  // first enqueue (filter device data is only available there).
  std::mutex cache_mutex_{};
  bool cache_allocated_{false};
  bool cache_filled_{false};
  std::int64_t cached_c_out_{0};
  std::int64_t cached_k1_{0};
  std::int64_t cached_k2_{0};
  std::int64_t cached_k3_{0};
  std::int64_t cached_c_in_{0};
  std::int8_t * cached_weight_int8_ptr_{nullptr};
  float * cached_w_scales_ptr_{nullptr};
  float * cached_gemm_bias_ptr_{nullptr};
};

}  // namespace nvinfer1::plugin

#endif  // AUTOWARE__TENSORRT_PLUGINS__IMPLICIT_GEMM_PLUGIN_HPP_
