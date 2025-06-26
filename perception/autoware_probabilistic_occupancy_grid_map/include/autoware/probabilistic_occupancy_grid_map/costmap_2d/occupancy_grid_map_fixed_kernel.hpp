// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_FIXED_KERNEL_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_FIXED_KERNEL_HPP_

#include "autoware/probabilistic_occupancy_grid_map/utils/utils_kernel.hpp"

#include <Eigen/Core>

#include <cstdint>

namespace autoware::occupancy_grid_map
{
namespace costmap_2d::map_fixed
{

void prepareTensorLaunch(
  const float * input_pointcloud, const std::size_t num_points, const std::size_t points_step,
  const std::size_t angle_bins, const std::size_t range_bins, const float min_height,
  const float max_height, const float min_angle, const float angle_increment_inv,
  const float range_resolution_inv, const Eigen::Matrix3f * rotation_map,
  const Eigen::Vector3f * translation_map, const Eigen::Matrix3f * rotation_scan,
  const Eigen::Vector3f * translation_scan, std::uint64_t * points_tensor, cudaStream_t stream);

void fillEmptySpaceLaunch(
  const std::uint64_t * points_tensor, const std::size_t angle_bins, const std::size_t range_bins,
  const float map_resolution_inv, const float scan_origin_x, const float scan_origin_y,
  const float map_origin_x, const float map_origin_y, const int num_cells_x, const int num_cells_y,
  std::uint8_t empty_value, std::uint8_t * costmap_tensor, cudaStream_t stream);

void fillUnknownSpaceLaunch(
  const std::uint64_t * raw_points_tensor, const std::uint64_t * obstacle_points_tensor,
  const float distance_margin, const std::size_t angle_bins, const std::size_t range_bins,
  const float map_resolution_inv, const float scan_origin_x, const float scan_origin_y,
  const float map_origin_x, const float map_origin_y, const int num_cells_x, const int num_cells_y,
  std::uint8_t free_space_value, std::uint8_t no_information_value, std::uint8_t * costmap_tensor,
  cudaStream_t stream);

void fillObstaclesLaunch(
  const std::uint64_t * points_tensor, const float distance_margin, const std::size_t angle_bins,
  const std::size_t range_bins, const float map_resolution_inv, const float map_origin_x,
  const float map_origin_y, const int num_cells_x, const int num_cells_y,
  std::uint8_t obstacle_value, std::uint8_t * costmap_tensor, cudaStream_t stream);

}  // namespace costmap_2d::map_fixed
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_FIXED_KERNEL_HPP_
