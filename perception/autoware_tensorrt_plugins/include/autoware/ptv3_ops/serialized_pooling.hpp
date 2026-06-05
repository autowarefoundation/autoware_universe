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
