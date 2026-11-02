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

  /// INT8 nodes only: the scale of the QuantizeLinear that feeds the features.
  float input_scale{1.0F};
  /// 1 when the node carries ``input_scale``; a default value cannot say it is absent.
  std::int32_t has_input_scale{0};
  /// The ``output_type`` attribute, one of kIMPLICIT_GEMM_OUTPUT_TYPE_*.
  std::int32_t output_type{0};
};

/// Values of ImplicitGemmParameters::output_type. Only ``"f16"`` is accepted; the other types
/// of the shared contract (``"bf16"``, ``"s8"``, ``"e4m3"``) are refused when the node is parsed.
constexpr std::int32_t kIMPLICIT_GEMM_OUTPUT_TYPE_UNSET{0};
constexpr std::int32_t kIMPLICIT_GEMM_OUTPUT_TYPE_F16{1};

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
  // Inputs: 0 features [N, C_in], 1 filters [C_out, k, k, k, C_in] (same type as the
  // features), 2-4 rulebook (INT32), then one of
  //   5 inputs: no bias;
  //   6 inputs: 5 bias [C_out], same type as the features (FP16/FP32 only);
  //   7 inputs (INT8 features): 5 weight_scale, 6 bias, each FP32 [C_out] or [0] (a zero-length
  //     tensor is absent).
  //
  // The features' type selects the mode; the inputs and attributes each mode accepts:
  //
  //                 FP16 / FP32 features      INT8 features
  //   inputs        5 or 6                    7
  //   filters       same type as features     INT8
  //   weight_scale  -                         [C_out]
  //   bias          -                         [0] or [C_out]
  //   input_scale   absent                    required: the scale of the QuantizeLinear in front
  //   output_type   absent                    required: "f16"
  //   output_scale  absent or 1.0             absent or 1.0
  //   output        same type as features     FP16
  //
  // INT8 epilogue: out = act(input_scale * weight_scale[n] * acc + bias[n]), INT32 accumulation.
  // Any other feature type (FP8, BF16, ...) is refused. The same layout is what t4sparse's
  // plugin of the same name accepts, so a graph written to it builds against either library.
  static constexpr std::int32_t INOUT_IN_FEATURES_INDEX{0};
  static constexpr std::int32_t INOUT_FILTERS_INDEX{1};
  static constexpr std::int32_t INOUT_PAIR_FWD_INDEX{2};
  static constexpr std::int32_t INOUT_PAIR_MASK_FWD_SPLITS_INDEX{3};
  static constexpr std::int32_t INOUT_MASK_ARGSORT_FWD_SPLITS_INDEX{4};
  /// Six inputs: the fused bias, in the features' type.
  static constexpr std::int32_t INOUT_OPTIONAL_BIAS_INDEX{5};
  /// Seven inputs: FP32 per-output-channel weight scale and bias.
  static constexpr std::int32_t INOUT_WEIGHT_SCALE_INDEX{5};
  static constexpr std::int32_t INOUT_BIAS_INDEX{6};

  static constexpr std::int32_t NUM_PLUGIN_INPUTS_NO_BIAS{5};
  static constexpr std::int32_t NUM_PLUGIN_INPUTS_BIAS{6};
  static constexpr std::int32_t NUM_PLUGIN_INPUTS_SCALE_BIAS{7};

  void initFieldsToSerialize();

  /// Check the input count, the features' and filters' types and the attributes against the
  /// table above; on a mismatch, log why (naming the layer) and return false.
  bool checkContract(
    DataType const * input_types, std::int32_t num_inputs, bool log) const noexcept;

  std::int32_t enqueueInt8(
    PluginTensorDesc const * input_desc, void const * const * inputs, void * const * outputs,
    cudaStream_t stream) noexcept;
  /// Device buffer of the INT8 GEMM's per-channel scale followed by a zero bias, filled on the
  /// first enqueue (input data is only available there).
  /// Allocated in ``configurePlugin`` (build) and ``onShapeChange`` (runtime) so no cudaMalloc
  /// runs inside CUDA graph capture. Idempotent for an unchanged C_out.
  std::int32_t allocateEpilogueConstants(std::int64_t c_out) noexcept;
  void releaseEpilogueConstants() noexcept;

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

  std::mutex epilogue_constants_mutex_{};
  void * epilogue_constants_ptr_{nullptr};
  std::int64_t epilogue_constants_c_out_{0};
  bool epilogue_constants_filled_{false};
};

}  // namespace nvinfer1::plugin

#endif  // AUTOWARE__TENSORRT_PLUGINS__IMPLICIT_GEMM_PLUGIN_HPP_
