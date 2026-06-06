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

#ifndef AUTOWARE__PTV3_OPS__SERIALIZED_POOLING_HPP_
#define AUTOWARE__PTV3_OPS__SERIALIZED_POOLING_HPP_

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <cstdint>

namespace autoware::ptv3
{

enum class SerializedPoolingReduce : std::int32_t {
  kSum = 0,
  kMean = 1,
  kMin = 2,
  kMax = 3,
};

/// \brief Run PTv3 serialized pooling over CSR-encoded voxel groups.
///
/// Inputs:
/// - `feature_in`: dense `[num_input_voxels, num_channels]` feature tensor. This must be the
///   ungathered projected feature tensor; the kernel applies `indices_in` internally.
/// - `coord_in`: dense `[num_input_voxels, 3]` float coordinate tensor.
/// - `indices_in`: concatenated source voxel indices sorted by output segment.
/// - `indptr_in`: CSR pointer tensor of shape `[num_segments + 1]`.
///
/// Outputs:
/// - `feature_out`: dense `[num_segments, num_channels]` reduced features.
/// - `coord_out`: dense `[num_segments, 3]` mean-reduced coordinates.
///
/// The output shape is fully determined by `indptr_in.shape[0] - 1`, so TensorRT sees the plugin
/// as a non-data-dependent-shape operation when `indptr_in` is provided as an engine input.
cudaError_t serialized_pooling_float(
  const float * feature_in, const float * coord_in, const std::int64_t * indices_in,
  const std::int64_t * indptr_in, float * feature_out, float * coord_out,
  std::int32_t num_segments_in, std::int32_t num_channels_in,
  SerializedPoolingReduce feature_reduce_in, cudaStream_t stream_in);

cudaError_t serialized_pooling_half(
  const half * feature_in, const float * coord_in, const std::int64_t * indices_in,
  const std::int64_t * indptr_in, half * feature_out, float * coord_out,
  std::int32_t num_segments_in, std::int32_t num_channels_in,
  SerializedPoolingReduce feature_reduce_in, cudaStream_t stream_in);

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3_OPS__SERIALIZED_POOLING_HPP_
