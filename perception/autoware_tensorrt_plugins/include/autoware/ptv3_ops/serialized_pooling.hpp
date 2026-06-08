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

/// Feature reduction applied to all input voxels that map to the same pooled voxel.
enum class SerializedPoolingReduce : std::int32_t {
  kSum = 0,
  kMean = 1,
  kMin = 2,
  kMax = 3,
};

/// Run PTv3 serialized pooling for FP32 feature tensors.
///
/// `indices_in` and `indptr_in` encode CSR groups over input voxels. Output segment `s` reads
/// source indices from `indices_in[indptr_in[s]:indptr_in[s + 1]]`, reduces the corresponding
/// feature rows, and writes one output row. Coordinates are averaged over the same CSR group.
cudaError_t serialized_pooling_float(
  const float * feature_in, const float * coord_in, const std::int64_t * indices_in,
  const std::int64_t * indptr_in, float * feature_out, float * coord_out,
  std::int32_t num_segments_in, std::int32_t num_channels_in,
  SerializedPoolingReduce feature_reduce_in, cudaStream_t stream_in);

/// Run PTv3 serialized pooling for FP16 feature tensors and FP32 coordinates.
///
/// The grouping contract is identical to `serialized_pooling_float`; feature accumulation is done
/// in FP32 and converted back to FP16 at the output.
cudaError_t serialized_pooling_half(
  const half * feature_in, const float * coord_in, const std::int64_t * indices_in,
  const std::int64_t * indptr_in, half * feature_out, float * coord_out,
  std::int32_t num_segments_in, std::int32_t num_channels_in,
  SerializedPoolingReduce feature_reduce_in, cudaStream_t stream_in);

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3_OPS__SERIALIZED_POOLING_HPP_
