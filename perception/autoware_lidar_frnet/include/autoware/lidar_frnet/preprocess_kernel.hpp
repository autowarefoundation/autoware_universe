// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LIDAR_FRNET__PREPROCESS_KERNEL_HPP_
#define AUTOWARE__LIDAR_FRNET__PREPROCESS_KERNEL_HPP_

#include "autoware/lidar_frnet/point_type.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <cuda_runtime_api.h>

#include <cstdint>

namespace autoware::lidar_frnet
{
struct Coord
{
  int64_t batch;
  int64_t y;
  int64_t x;

  __host__ __device__ bool operator==(const Coord & other) const
  {
    return y == other.y && x == other.x;
  }

  __host__ __device__ bool operator<(const Coord & other) const
  {
    if (y == other.y) return x < other.x;
    return y < other.y;
  }
};

class PreprocessCuda
{
public:
  PreprocessCuda(const utils::PreprocessingParams & params, cudaStream_t stream);

  void generateUniqueCoors(
    const uint32_t num_points, const int64_t * coors, const int64_t * coors_keys,
    uint32_t & output_num_unique_coors, int64_t * output_voxel_coors, int64_t * output_inverse_map);

  cudaError_t projectPoints_launch(
    const InputPointType * cloud, const uint32_t num_points, uint32_t * output_num_points,
    float * output_points, int64_t * output_coors, int64_t * output_coors_keys,
    uint32_t * output_proj_idxs, uint64_t * output_proj_2d);

  cudaError_t interpolatePoints_launch(
    uint32_t * proj_idxs, uint64_t * proj_2d, uint32_t * output_num_points, float * output_points,
    int64_t * output_coors, int64_t * output_coors_keys);

private:
  const utils::Dims2d interpolation_;
  cudaStream_t stream_;
};

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__PREPROCESS_KERNEL_HPP_
