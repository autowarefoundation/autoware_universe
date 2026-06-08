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

#include "autoware/scatter_ops/gather_segment_csr.hpp"

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <cstdint>

namespace autoware::scatter_ops
{
namespace
{

constexpr int kThreadsPerBlock = 256;
constexpr int kCoordChannels = 3;
constexpr float kFloatMax = 3.402823466e+38F;

/// Converts feature values to and from the FP32 accumulator type used by all reductions.
template <typename Scalar>
struct FeatureCast
{
  __device__ static float toFloat(Scalar value_in) { return static_cast<float>(value_in); }

  __device__ static Scalar fromFloat(float value_in) { return static_cast<Scalar>(value_in); }
};

template <>
struct FeatureCast<half>
{
  __device__ static float toFloat(half value_in) { return __half2float(value_in); }

  __device__ static half fromFloat(float value_in) { return __float2half(value_in); }
};

/// Returns the neutral accumulator value for the requested reduction mode.
__device__ float initial_value(GatherSegmentCSRReduce reduce_in)
{
  if (reduce_in == GatherSegmentCSRReduce::kMin) {
    return kFloatMax;
  }
  if (reduce_in == GatherSegmentCSRReduce::kMax) {
    return -kFloatMax;
  }
  return 0.0F;
}

/// Merges one source value into the current accumulator for sum/mean/min/max reductions.
__device__ float update_value(float current_in, float next_in, GatherSegmentCSRReduce reduce_in)
{
  if (reduce_in == GatherSegmentCSRReduce::kMin) {
    return fminf(current_in, next_in);
  }
  if (reduce_in == GatherSegmentCSRReduce::kMax) {
    return fmaxf(current_in, next_in);
  }
  return current_in + next_in;
}

/// Reduces one `(output segment, feature channel)` element from a gathered CSR group.
template <typename Scalar>
__global__ void reduce_features_kernel(
  const Scalar * feature_in, const std::int64_t * indices_in, const std::int64_t * indptr_in,
  Scalar * feature_out, std::int32_t num_segments_in, std::int32_t num_channels_in,
  GatherSegmentCSRReduce reduce_in)
{
  const auto linear_index = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto output_size =
    static_cast<std::int64_t>(num_segments_in) * static_cast<std::int64_t>(num_channels_in);
  if (linear_index >= output_size) {
    return;
  }

  const auto segment_index = static_cast<std::int32_t>(linear_index / num_channels_in);
  const auto channel_index = static_cast<std::int32_t>(linear_index % num_channels_in);
  const auto begin = indptr_in[segment_index];
  const auto end = indptr_in[segment_index + 1];
  const auto count = end - begin;

  float value = initial_value(reduce_in);
  for (std::int64_t cursor = begin; cursor < end; ++cursor) {
    const auto source_index = indices_in[cursor];
    const auto input_offset =
      source_index * static_cast<std::int64_t>(num_channels_in) + channel_index;
    value = update_value(value, FeatureCast<Scalar>::toFloat(feature_in[input_offset]), reduce_in);
  }

  if (count == 0) {
    value = 0.0F;
  } else if (reduce_in == GatherSegmentCSRReduce::kMean) {
    value /= static_cast<float>(count);
  }

  feature_out[linear_index] = FeatureCast<Scalar>::fromFloat(value);
}

/// Averages output coordinates from the same gathered CSR groups used for features.
__global__ void reduce_coords_mean_kernel(
  const float * coord_in, const std::int64_t * indices_in, const std::int64_t * indptr_in,
  float * coord_out, std::int32_t num_segments_in)
{
  const auto linear_index = static_cast<std::int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  const auto output_size =
    static_cast<std::int64_t>(num_segments_in) * static_cast<std::int64_t>(kCoordChannels);
  if (linear_index >= output_size) {
    return;
  }

  const auto segment_index = static_cast<std::int32_t>(linear_index / kCoordChannels);
  const auto channel_index = static_cast<std::int32_t>(linear_index % kCoordChannels);
  const auto begin = indptr_in[segment_index];
  const auto end = indptr_in[segment_index + 1];
  const auto count = end - begin;

  float value = 0.0F;
  for (std::int64_t cursor = begin; cursor < end; ++cursor) {
    const auto source_index = indices_in[cursor];
    value += coord_in[source_index * kCoordChannels + channel_index];
  }

  coord_out[linear_index] = count == 0 ? 0.0F : value / static_cast<float>(count);
}

/// Launches feature and coordinate reductions for one gathered CSR stage.
template <typename Scalar>
cudaError_t gather_segment_csr(
  const Scalar * feature_in, const float * coord_in, const std::int64_t * indices_in,
  const std::int64_t * indptr_in, Scalar * feature_out, float * coord_out,
  std::int32_t num_segments_in, std::int32_t num_channels_in,
  GatherSegmentCSRReduce feature_reduce_in, cudaStream_t stream_in)
{
  if (num_segments_in < 0 || num_channels_in <= 0) {
    return cudaErrorInvalidValue;
  }

  const auto feature_output_size =
    static_cast<std::int64_t>(num_segments_in) * static_cast<std::int64_t>(num_channels_in);
  const auto feature_blocks =
    static_cast<unsigned int>((feature_output_size + kThreadsPerBlock - 1) / kThreadsPerBlock);
  if (feature_blocks > 0U) {
    reduce_features_kernel<<<feature_blocks, kThreadsPerBlock, 0, stream_in>>>(
      feature_in, indices_in, indptr_in, feature_out, num_segments_in, num_channels_in,
      feature_reduce_in);
    if (const auto status = cudaPeekAtLastError(); status != cudaSuccess) {
      return status;
    }
  }

  const auto coord_output_size =
    static_cast<std::int64_t>(num_segments_in) * static_cast<std::int64_t>(kCoordChannels);
  const auto coord_blocks =
    static_cast<unsigned int>((coord_output_size + kThreadsPerBlock - 1) / kThreadsPerBlock);
  if (coord_blocks > 0U) {
    reduce_coords_mean_kernel<<<coord_blocks, kThreadsPerBlock, 0, stream_in>>>(
      coord_in, indices_in, indptr_in, coord_out, num_segments_in);
    if (const auto status = cudaPeekAtLastError(); status != cudaSuccess) {
      return status;
    }
  }

  return cudaSuccess;
}

}  // namespace

cudaError_t gather_segment_csr_float(
  const float * feature_in, const float * coord_in, const std::int64_t * indices_in,
  const std::int64_t * indptr_in, float * feature_out, float * coord_out,
  std::int32_t num_segments_in, std::int32_t num_channels_in,
  GatherSegmentCSRReduce feature_reduce_in, cudaStream_t stream_in)
{
  return gather_segment_csr(
    feature_in, coord_in, indices_in, indptr_in, feature_out, coord_out, num_segments_in,
    num_channels_in, feature_reduce_in, stream_in);
}

cudaError_t gather_segment_csr_half(
  const half * feature_in, const float * coord_in, const std::int64_t * indices_in,
  const std::int64_t * indptr_in, half * feature_out, float * coord_out,
  std::int32_t num_segments_in, std::int32_t num_channels_in,
  GatherSegmentCSRReduce feature_reduce_in, cudaStream_t stream_in)
{
  return gather_segment_csr(
    feature_in, coord_in, indices_in, indptr_in, feature_out, coord_out, num_segments_in,
    num_channels_in, feature_reduce_in, stream_in);
}

}  // namespace autoware::scatter_ops
