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

#include "autoware/quantize_ops/quantize_features.hpp"

namespace
{
constexpr int kBlockSize = 256;
}  // namespace

__global__ void quantize_features_kernel(
  const __half * __restrict__ input, std::int8_t * __restrict__ output, float inv_scale,
  std::int64_t total_elements)
{
  std::int64_t idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx < total_elements) {
    float val = __half2float(input[idx]) * inv_scale;
    val = fminf(fmaxf(roundf(val), -128.0f), 127.0f);
    output[idx] = static_cast<std::int8_t>(val);
  }
}

__global__ void quantize_weights_per_channel_kernel(
  const __half * __restrict__ input, std::int8_t * __restrict__ output,
  const float * __restrict__ w_scales, std::int64_t c_out, std::int64_t elements_per_channel)
{
  std::int64_t idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  std::int64_t total = c_out * elements_per_channel;
  if (idx < total) {
    std::int64_t c = idx / elements_per_channel;
    float inv_scale = 1.0f / w_scales[c];
    float val = __half2float(input[idx]) * inv_scale;
    val = fminf(fmaxf(roundf(val), -128.0f), 127.0f);
    output[idx] = static_cast<std::int8_t>(val);
  }
}

__global__ void compute_w_scales_kernel(
  const float * __restrict__ channel_scale, float * __restrict__ w_scales, float output_scale,
  float input_scale, std::int64_t c_out)
{
  std::int64_t idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx < c_out) {
    // channel_scale[c] = (input_scale * w_scale[c]) / output_scale
    // => w_scale[c] = channel_scale[c] * output_scale / input_scale
    w_scales[idx] = channel_scale[idx] * output_scale / input_scale;
  }
}

__global__ void fuse_output_scale_into_gemm_scale_bias_kernel(
  const float * __restrict__ channel_scale, const float * __restrict__ bias_scaled,
  float output_scale, float * __restrict__ gemm_channel_scale_out,
  float * __restrict__ gemm_bias_out, std::int64_t c_out)
{
  std::int64_t idx = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (idx < c_out) {
    gemm_channel_scale_out[idx] = channel_scale[idx] * output_scale;
    gemm_bias_out[idx] = bias_scaled[idx] * output_scale;
  }
}

void launch_quantize_features(
  const __half * input, std::int8_t * output, float scale, std::int64_t total_elements,
  cudaStream_t stream)
{
  float inv_scale = 1.0f / scale;
  int grid = static_cast<int>((total_elements + kBlockSize - 1) / kBlockSize);
  quantize_features_kernel<<<grid, kBlockSize, 0, stream>>>(
    input, output, inv_scale, total_elements);
}

void launch_quantize_weights_per_channel(
  const __half * input, std::int8_t * output, const float * w_scales, std::int64_t c_out,
  std::int64_t elements_per_channel, cudaStream_t stream)
{
  std::int64_t total = c_out * elements_per_channel;
  int grid = static_cast<int>((total + kBlockSize - 1) / kBlockSize);
  quantize_weights_per_channel_kernel<<<grid, kBlockSize, 0, stream>>>(
    input, output, w_scales, c_out, elements_per_channel);
}

void launch_compute_w_scales(
  const float * channel_scale, float * w_scales, float output_scale, float input_scale,
  std::int64_t c_out, cudaStream_t stream)
{
  int grid = static_cast<int>((c_out + kBlockSize - 1) / kBlockSize);
  compute_w_scales_kernel<<<grid, kBlockSize, 0, stream>>>(
    channel_scale, w_scales, output_scale, input_scale, c_out);
}

void launch_fuse_output_scale_into_gemm_scale_bias(
  const float * channel_scale, const float * bias_scaled, float output_scale,
  float * gemm_channel_scale_out, float * gemm_bias_out, std::int64_t c_out, cudaStream_t stream)
{
  int grid = static_cast<int>((c_out + kBlockSize - 1) / kBlockSize);
  fuse_output_scale_into_gemm_scale_bias_kernel<<<grid, kBlockSize, 0, stream>>>(
    channel_scale, bias_scaled, output_scale, gemm_channel_scale_out, gemm_bias_out, c_out);
}
