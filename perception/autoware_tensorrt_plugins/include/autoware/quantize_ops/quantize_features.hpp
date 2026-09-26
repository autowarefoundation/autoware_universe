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

#ifndef AUTOWARE__QUANTIZE_OPS__QUANTIZE_FEATURES_HPP_
#define AUTOWARE__QUANTIZE_OPS__QUANTIZE_FEATURES_HPP_

#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <cstdint>

// INT8 quantization helpers for the ImplicitGemm plugin's INT8 (precision==1) path.

/// Quantize FP16 features to INT8: out = round(in / scale) clamped to [-128, 127].
void launch_quantize_features(
  const __half * input, std::int8_t * output, float scale, std::int64_t total_elements,
  cudaStream_t stream);

/// Quantize FP16 weights to INT8 per output channel using ``w_scales[c]``.
void launch_quantize_weights_per_channel(
  const __half * input, std::int8_t * output, const float * w_scales, std::int64_t c_out,
  std::int64_t elements_per_channel, cudaStream_t stream);

/// Recover per-channel weight scales from the export-side channel_scale:
/// w_scale[c] = channel_scale[c] * output_scale / input_scale.
void launch_compute_w_scales(
  const float * channel_scale, float * w_scales, float output_scale, float input_scale,
  std::int64_t c_out, cudaStream_t stream);

/// Fold the output dequant into per-channel scale/bias for the s8s8f16 epilogue (which never
/// multiplies ``alpha``): gemm_channel_scale[c] = channel_scale[c] * output_scale,
/// gemm_bias[c] = bias_scaled[c] * output_scale. The GEMM is then called with output_scale = 1.
void launch_fuse_output_scale_into_gemm_scale_bias(
  const float * channel_scale, const float * bias_scaled, float output_scale,
  float * gemm_channel_scale_out, float * gemm_bias_out, std::int64_t c_out, cudaStream_t stream);

#endif  // AUTOWARE__QUANTIZE_OPS__QUANTIZE_FEATURES_HPP_
