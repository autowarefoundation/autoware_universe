// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__UTILS_KERNEL_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__UTILS_KERNEL_HPP_

#include <Eigen/Core>

#include <cuda_runtime.h>

#include <cstdint>

namespace autoware::occupancy_grid_map
{
namespace utils
{

void __device__ setCellValue(
  float wx, float wy, float origin_x, float origin_y, float resolution_inv, int size_x, int size_y,
  std::uint8_t value, std::uint8_t * costmap_tensor);

void __device__ raytrace(
  const float source_x, const float source_y, const float target_x, const float target_y,
  const float origin_x, float origin_y, const float resolution_inv, const int size_x,
  const int size_y, const std::uint8_t cost, std::uint8_t * costmap_tensor);

void copyMapRegionLaunch(
  const std::uint8_t * source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
  unsigned int sm_size_x, unsigned int sm_size_y, std::uint8_t * dest_map,
  unsigned int dm_lower_left_x, unsigned int dm_lower_left_y, unsigned int dm_size_x,
  unsigned int dm_size_y, unsigned int region_size_x, unsigned int region_size_y,
  cudaStream_t stream);

void transformPointCloudLaunch(
  std::uint8_t * points, std::size_t num_points, std::size_t points_step,
  const Eigen::Matrix3f * rotation, const Eigen::Vector3f * translation, cudaStream_t stream);

}  // namespace utils
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__UTILS_KERNEL_HPP_
