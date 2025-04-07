// Copyright 2024 Tier IV, Inc.
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

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_projective_kernel.hpp"
#include "autoware/probabilistic_occupancy_grid_map/utils/utils_kernel.hpp"

#include <stdio.h>
#include <stdlib.h>

#include <cmath>
#include <cstdint>
#include <iostream>

namespace autoware::occupancy_grid_map
{
namespace costmap_2d::map_projective
{

static constexpr float RANGE_DISCRETIZATION_RESOLUTION = 0.001f;

bool __device__ isVisibleBeyondObstacle(
  const std::uint64_t * obstacle_element, const std::uint64_t * raw_element,
  const float & scan_origin_z, const float & robot_pose_z)
{
  std::uint64_t raw_range_and_z = raw_element[2];
  std::uint32_t raw_range_int = raw_range_and_z >> 32;
  float raw_range = raw_range_int * RANGE_DISCRETIZATION_RESOLUTION;
  float raw_world_z = __uint_as_float(raw_range_and_z & 0xFFFFFFFF);

  std::uint64_t obstacle_range_and_pl = raw_element[3];
  std::uint32_t obstacle_range_int = obstacle_range_and_pl >> 32;
  float obstacle_range = obstacle_range_int * RANGE_DISCRETIZATION_RESOLUTION;
  float obstacle_pl = __uint_as_float(obstacle_range_and_pl & 0xFFFFFFFF);

  if (raw_range < obstacle_range) {
    return false;
  }

  if (std::isinf(obstacle_pl)) {
    return false;
  }

  // y = ax + b
  const double a = -(scan_origin_z - robot_pose_z) / (obstacle_range + obstacle_pl);
  const double b = scan_origin_z;
  return raw_world_z > (a * raw_range + b);
};

__global__ void prepareRawTensorKernel(
  const float * input_pointcloud, const std::size_t num_points, const std::size_t points_step,
  const std::size_t angle_bins, const std::size_t range_bins, const float min_height,
  const float max_height, const float min_angle, const float angle_increment_inv,
  const float range_resolution_inv, const Eigen::Matrix3f * rotation_map,
  const Eigen::Vector3f * translation_map, const Eigen::Matrix3f * rotation_scan,
  const Eigen::Vector3f * translation_scan, std::uint64_t * points_tensor)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }

  Eigen::Map<const Eigen::Vector3f> point(input_pointcloud + idx * points_step);

  if (
    point.z() > max_height || point.z() < min_height || !isfinite(point.x()) ||
    !isfinite(point.y()) || !isfinite(point.z())) {
    return;
  }

  Eigen::Vector3f map_point = (*rotation_map) * point + (*translation_map);
  Eigen::Vector3f scan_point = (*rotation_scan) * map_point + (*translation_scan);

  float angle = atan2(scan_point.y(), scan_point.x());
  int angle_bin_index = static_cast<int>((angle - min_angle) * angle_increment_inv);
  float range = sqrt(scan_point.y() * scan_point.y() + scan_point.x() * scan_point.x());
  int range_bin_index = static_cast<int>(range * range_resolution_inv);

  if (
    angle_bin_index < 0 || angle_bin_index >= angle_bins || range_bin_index < 0 ||
    range_bin_index >= range_bins) {
    return;
  }

  std::uint64_t range_int = static_cast<std::int64_t>(range / RANGE_DISCRETIZATION_RESOLUTION);
  std::uint32_t world_x_int = __float_as_uint(map_point.x());
  std::uint32_t world_y_int = __float_as_uint(map_point.y());
  std::uint32_t world_z_int = __float_as_uint(map_point.z());
  std::uint32_t projected_length_int = __float_as_uint(0.f);
  std::uint32_t projected_world_x_int = __float_as_uint(0.f);
  std::uint32_t projected_world_y_int = __float_as_uint(0.f);

  // Can not use std::uint64_t for cuda reasons...
  static_assert(
    sizeof(std::uint64_t) == sizeof(unsigned long long int),
    "unsigned long long int is not 8 bytes");
  unsigned long long int range_and_x_int = (range_int << 32) | world_x_int;
  unsigned long long int range_and_y_int = (range_int << 32) | world_y_int;
  unsigned long long int range_and_z_int = (range_int << 32) | world_z_int;
  unsigned long long int range_and_pl_int = (range_int << 32) | projected_length_int;
  unsigned long long int range_and_px_int = (range_int << 32) | projected_world_x_int;
  unsigned long long int range_and_py_int = (range_int << 32) | projected_world_y_int;

  std::uint64_t * element_address =
    points_tensor + 6 * (angle_bin_index * range_bins + range_bin_index);

  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 0), range_and_x_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 1), range_and_y_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 2), range_and_z_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 3), range_and_pl_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 4), range_and_px_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 5), range_and_py_int);
}

__global__ void prepareObstacleTensorKernel(
  const float * input_pointcloud, const std::size_t num_points, const std::size_t points_step,
  const std::size_t angle_bins, const std::size_t range_bins, const float min_height,
  const float max_height, const float min_angle, const float angle_increment_inv,
  const float range_resolution_inv, const float projection_dz_threshold,
  const Eigen::Vector3f * translation_scan_origin, const Eigen::Matrix3f * rotation_map,
  const Eigen::Vector3f * translation_map, const Eigen::Matrix3f * rotation_scan,
  const Eigen::Vector3f * translation_scan, std::uint64_t * points_tensor)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }

  Eigen::Map<const Eigen::Vector3f> point(input_pointcloud + idx * points_step);

  if (
    point.z() > max_height || point.z() < min_height || !isfinite(point.x()) ||
    !isfinite(point.y()) || !isfinite(point.z())) {
    return;
  }

  Eigen::Vector3f map_point = (*rotation_map) * point + (*translation_map);
  Eigen::Vector3f scan_point = (*rotation_scan) * map_point + (*translation_scan);

  float angle = atan2(scan_point.y(), scan_point.x());
  int angle_bin_index = static_cast<int>((angle - min_angle) * angle_increment_inv);
  float range = sqrt(scan_point.y() * scan_point.y() + scan_point.x() * scan_point.x());
  int range_bin_index = static_cast<int>(range * range_resolution_inv);

  if (
    angle_bin_index < 0 || angle_bin_index >= angle_bins || range_bin_index < 0 ||
    range_bin_index >= range_bins) {
    return;
  }

  std::uint64_t range_int = static_cast<std::int64_t>(range / RANGE_DISCRETIZATION_RESOLUTION);
  std::uint32_t world_x_int = __float_as_uint(map_point.x());
  std::uint32_t world_y_int = __float_as_uint(map_point.y());
  std::uint32_t world_z_int = __float_as_uint(map_point.z());

  const float scan_z = translation_scan_origin->z();
  -translation_map->z();
  const float obstacle_z = map_point.z() - translation_scan_origin->z();
  const float dz = scan_z - obstacle_z;

  float projected_length, projected_world_x, projected_world_y;

  if (dz > projection_dz_threshold) {
    const float ratio = obstacle_z / dz;
    projected_length = range * ratio;
    projected_world_x = map_point.x() + (map_point.x() - translation_scan_origin->x()) * ratio;
    projected_world_y = map_point.y() + (map_point.y() - translation_scan_origin->y()) * ratio;
  } else {
    projected_length = INFINITY;
    projected_world_x = INFINITY;
    projected_world_y = INFINITY;
  }

  std::uint32_t projected_length_int = __float_as_uint(projected_length);
  std::uint32_t projected_world_x_int = __float_as_uint(projected_world_x);
  std::uint32_t projected_world_y_int = __float_as_uint(projected_world_y);

  // Can not use std::uint64_t for cuda reasons...
  static_assert(
    sizeof(std::uint64_t) == sizeof(unsigned long long int),
    "unsigned long long int is not 8 bytes");
  unsigned long long int range_and_x_int = (range_int << 32) | world_x_int;
  unsigned long long int range_and_y_int = (range_int << 32) | world_y_int;
  unsigned long long int range_and_z_int = (range_int << 32) | world_z_int;
  unsigned long long int range_and_pl_int = (range_int << 32) | projected_length_int;
  unsigned long long int range_and_px_int = (range_int << 32) | projected_world_x_int;
  unsigned long long int range_and_py_int = (range_int << 32) | projected_world_y_int;

  std::uint64_t * element_address =
    points_tensor + 6 * (angle_bin_index * range_bins + range_bin_index);

  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 0), range_and_x_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 1), range_and_y_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 2), range_and_z_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 3), range_and_pl_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 4), range_and_px_int);
  atomicMin(reinterpret_cast<unsigned long long int *>(element_address + 5), range_and_py_int);
}

__global__ void fillEmptySpaceKernel(
  const std::uint64_t * points_tensor, const std::size_t angle_bins, const std::size_t range_bins,
  const float map_resolution_inv, const float scan_origin_x, const float scan_origin_y,
  const float map_origin_x, const float map_origin_y, const int num_cells_x, const int num_cells_y,
  std::uint8_t empty_value, std::uint8_t * costmap_tensor)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= angle_bins * range_bins) return;

  int angle_bin_index = idx / range_bins;
  int range_bin_index = idx % range_bins;

  std::uint64_t range_and_x =
    points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 0];
  std::uint32_t range_int = range_and_x >> 32;

  if (range_int == 0xFFFFFFFF) {
    return;
  }

  std::uint64_t range_and_y =
    points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 1];
  float world_x = __uint_as_float(range_and_x & 0xFFFFFFFF);
  float world_y = __uint_as_float(range_and_y & 0xFFFFFFFF);

  if (world_x < map_origin_x || world_y < map_origin_y) {
    return;
  }

  autoware::occupancy_grid_map::utils::raytrace(
    scan_origin_x, scan_origin_y, world_x, world_y, map_origin_x, map_origin_y, map_resolution_inv,
    num_cells_x, num_cells_y, empty_value, costmap_tensor);
}

__global__ void fillUnknownSpaceKernel(
  const std::uint64_t * raw_points_tensor, const std::uint64_t * obstacle_points_tensor,
  const float obstacle_separation_threshold, const std::size_t angle_bins,
  const std::size_t range_bins, const float map_resolution_inv, const float scan_origin_x,
  const float scan_origin_y, const float scan_origin_z, const float map_origin_x,
  const float map_origin_y, const float robot_pose_z, const int num_cells_x, const int num_cells_y,
  std::uint8_t free_space_value, std::uint8_t no_information_value, std::uint8_t * costmap_tensor)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= angle_bins * range_bins) return;

  int angle_bin_index = idx / range_bins;
  int range_bin_index = idx % range_bins;

  if (range_bin_index == range_bins - 1) {
    return;
  }

  std::uint64_t obs_range_and_x =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 0];
  std::uint64_t obs_range_and_y =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 1];
  std::uint64_t obs_range_and_px =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 4];
  std::uint64_t obs_range_and_py =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 5];
  std::uint32_t obs_range_int = obs_range_and_x >> 32;
  float obs_range = obs_range_int * RANGE_DISCRETIZATION_RESOLUTION;
  float obs_world_x = __uint_as_float(obs_range_and_x & 0xFFFFFFFF);
  float obs_world_y = __uint_as_float(obs_range_and_y & 0xFFFFFFFF);
  float obs_world_px = __uint_as_float(obs_range_and_px & 0xFFFFFFFF);
  float obs_world_py = __uint_as_float(obs_range_and_py & 0xFFFFFFFF);

  if (obs_range_int == 0xFFFFFFFF || obs_world_x < map_origin_x || obs_world_y < map_origin_y) {
    return;
  }

  int next_raw_range_bin_index = range_bin_index + 1;
  int next_obs_range_bin_index = range_bin_index + 1;

  for (; next_raw_range_bin_index < range_bins - 1; next_raw_range_bin_index++) {
    std::uint64_t next_raw_range_and_x =
      raw_points_tensor[6 * (angle_bin_index * range_bins + next_raw_range_bin_index) + 0];
    std::uint32_t next_raw_range_int = next_raw_range_and_x >> 32;

    if (
      next_raw_range_int != 0xFFFFFFFF &&
      isVisibleBeyondObstacle(
        obstacle_points_tensor + 6 * (angle_bin_index * range_bins + range_bin_index),
        raw_points_tensor + 6 * (angle_bin_index * range_bins + next_raw_range_bin_index),
        scan_origin_z, robot_pose_z)) {
      break;
    }
  }

  for (; next_obs_range_bin_index < range_bins - 1; next_obs_range_bin_index++) {
    std::uint64_t next_obs_range_and_x =
      obstacle_points_tensor[6 * (angle_bin_index * range_bins + next_obs_range_bin_index) + 0];
    std::uint32_t next_obs_range_int = next_obs_range_and_x >> 32;

    if (next_obs_range_int != 0xFFFFFFFF) {
      break;
    }
  }

  const std::uint64_t next_obs_range_and_x =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + next_obs_range_bin_index) + 0];
  const std::uint64_t next_obs_range_and_y =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + next_obs_range_bin_index) + 1];
  const std::uint64_t next_obs_range_and_px =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + next_obs_range_bin_index) + 4];
  const std::uint64_t next_obs_range_and_py =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + next_obs_range_bin_index) + 5];
  const std::uint32_t next_obs_range_int = next_obs_range_and_x >> 32;
  const float next_obs_world_x = __uint_as_float(next_obs_range_and_x & 0xFFFFFFFF);
  const float next_obs_world_y = __uint_as_float(next_obs_range_and_y & 0xFFFFFFFF);
  const float next_obs_world_px = __uint_as_float(next_obs_range_and_px & 0xFFFFFFFF);
  const float next_obs_world_py = __uint_as_float(next_obs_range_and_py & 0xFFFFFFFF);

  float next_obs_range = next_obs_range_int * RANGE_DISCRETIZATION_RESOLUTION;
  float obs_to_obs_distance = next_obs_range - obs_range;

  const std::uint64_t next_raw_range_and_x =
    raw_points_tensor[6 * (angle_bin_index * range_bins + next_raw_range_bin_index) + 0];
  const std::uint64_t next_raw_range_and_y =
    raw_points_tensor[6 * (angle_bin_index * range_bins + next_raw_range_bin_index) + 1];
  const std::uint64_t next_raw_range_and_px =
    raw_points_tensor[6 * (angle_bin_index * range_bins + next_raw_range_bin_index) + 4];
  const std::uint64_t next_raw_range_and_py =
    raw_points_tensor[6 * (angle_bin_index * range_bins + next_raw_range_bin_index) + 5];
  const std::uint32_t next_raw_range_int = next_raw_range_and_x >> 32;
  const float next_raw_world_x = __uint_as_float(next_raw_range_and_x & 0xFFFFFFFF);
  const float next_raw_world_y = __uint_as_float(next_raw_range_and_y & 0xFFFFFFFF);
  const float next_raw_world_px = __uint_as_float(next_raw_range_and_px & 0xFFFFFFFF);
  const float next_raw_world_py = __uint_as_float(next_raw_range_and_py & 0xFFFFFFFF);

  if (next_raw_range_int == 0xFFFFFFFF) {
    autoware::occupancy_grid_map::utils::raytrace(
      obs_world_x, obs_world_y, obs_world_px, obs_world_py, map_origin_x, map_origin_y,
      map_resolution_inv, num_cells_x, num_cells_y, no_information_value, costmap_tensor);
    return;
  }

  if (next_obs_range_int == 0xFFFFFFFF) {
    autoware::occupancy_grid_map::utils::raytrace(
      obs_world_x, obs_world_y, obs_world_px, obs_world_py, map_origin_x, map_origin_y,
      map_resolution_inv, num_cells_x, num_cells_y, no_information_value, costmap_tensor);
    return;
  }

  float next_raw_range = next_raw_range_int * RANGE_DISCRETIZATION_RESOLUTION;
  float raw_to_obs_distance = abs(next_raw_range - obs_range);

  if (obs_to_obs_distance <= obstacle_separation_threshold) {
    return;
  }

  if (raw_to_obs_distance < obs_to_obs_distance) {
    if (next_raw_world_x < map_origin_x || next_raw_world_y < map_origin_y) {
      return;
    }

    autoware::occupancy_grid_map::utils::raytrace(
      obs_world_x, obs_world_y, next_raw_world_x, next_raw_world_y, map_origin_x, map_origin_y,
      map_resolution_inv, num_cells_x, num_cells_y, no_information_value, costmap_tensor);

    autoware::occupancy_grid_map::utils::setCellValue(
      next_raw_world_x, next_raw_world_y, map_origin_x, map_origin_y, map_resolution_inv,
      num_cells_x, num_cells_y, free_space_value, costmap_tensor);
    return;
  } else {
    // fill with no information between obstacles

    if (next_obs_world_x < map_origin_x || next_obs_world_y < map_origin_y) {
      return;
    }

    autoware::occupancy_grid_map::utils::raytrace(
      obs_world_x, obs_world_y, next_obs_world_x, next_obs_world_y, map_origin_x, map_origin_y,
      map_resolution_inv, num_cells_x, num_cells_y, no_information_value, costmap_tensor);

    return;
  }
}

__global__ void fillObstaclesKernel(
  const std::uint64_t * obstacle_points_tensor, const float distance_margin,
  const std::size_t angle_bins, const std::size_t range_bins, const float map_resolution_inv,
  const float map_origin_x, const float map_origin_y, const int num_cells_x, const int num_cells_y,
  std::uint8_t obstacle_value, std::uint8_t * costmap_tensor)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= angle_bins * range_bins) return;

  int angle_bin_index = idx / range_bins;
  int range_bin_index = idx % range_bins;

  std::uint64_t range_and_x =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 0];
  std::uint64_t range_and_y =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 1];

  std::uint32_t range_int = range_and_x >> 32;
  float range = range_int * RANGE_DISCRETIZATION_RESOLUTION;
  float world_x = __uint_as_float(range_and_x & 0xFFFFFFFF);
  float world_y = __uint_as_float(range_and_y & 0xFFFFFFFF);

  if (range < 0.0 || range_int == 0xFFFFFFFF) {
    return;
  }

  autoware::occupancy_grid_map::utils::setCellValue(
    world_x, world_y, map_origin_x, map_origin_y, map_resolution_inv, num_cells_x, num_cells_y,
    obstacle_value, costmap_tensor);

  // Look for the next obstacle point
  int next_obs_range_bin_index = range_bin_index + 1;
  for (; next_obs_range_bin_index < range_bins - 1; next_obs_range_bin_index++) {
    std::uint64_t next_obs_range_and_x =
      obstacle_points_tensor[6 * (angle_bin_index * range_bins + next_obs_range_bin_index) + 0];
    std::uint32_t next_obs_range_int = next_obs_range_and_x >> 32;

    if (next_obs_range_int != 0xFFFFFFFF) {
      break;
    }
  }

  next_obs_range_bin_index = std::min<int>(next_obs_range_bin_index, range_bins - 1);

  std::uint64_t next_obs_range_and_x =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 0];
  std::uint64_t next_obs_range_and_y =
    obstacle_points_tensor[6 * (angle_bin_index * range_bins + range_bin_index) + 1];

  std::uint32_t next_obs_range_int = next_obs_range_and_x >> 32;
  float next_obs_range = next_obs_range_int * RANGE_DISCRETIZATION_RESOLUTION;
  float next_obs_world_x = __uint_as_float(next_obs_range_and_x & 0xFFFFFFFF);
  float next_obs_world_y = __uint_as_float(next_obs_range_and_y & 0xFFFFFFFF);

  if (next_obs_range_int == 0xFFFFFFFF || abs(next_obs_range - range) > distance_margin) {
    return;
  }

  if (next_obs_world_x < map_origin_x || next_obs_world_y < map_origin_y) {
    return;
  }

  autoware::occupancy_grid_map::utils::raytrace(
    world_x, world_y, next_obs_world_x, next_obs_world_y, map_origin_x, map_origin_y,
    map_resolution_inv, num_cells_x, num_cells_y, obstacle_value, costmap_tensor);
}

void prepareRawTensorLaunch(
  const float * input_pointcloud, const std::size_t num_points, const std::size_t points_step,
  const std::size_t angle_bins, const std::size_t range_bins, const float min_height,
  const float max_height, const float min_angle, const float angle_increment_inv,
  const float range_resolution_inv, const Eigen::Matrix3f * rotation_map,
  const Eigen::Vector3f * translation_map, const Eigen::Matrix3f * rotation_scan,
  const Eigen::Vector3f * translation_scan, std::uint64_t * points_tensor, cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks = (num_points + threads_per_block - 1) / threads_per_block;

  prepareRawTensorKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    input_pointcloud, num_points, points_step, angle_bins, range_bins, min_height, max_height,
    min_angle, angle_increment_inv, range_resolution_inv, rotation_map, translation_map,
    rotation_scan, translation_scan, points_tensor);
}

void prepareObstacleTensorLaunch(
  const float * input_pointcloud, const std::size_t num_points, const std::size_t points_step,
  const std::size_t angle_bins, const std::size_t range_bins, const float min_height,
  const float max_height, const float min_angle, const float angle_increment_inv,
  const float range_resolution_inv, const float projection_dz_threshold,
  const Eigen::Vector3f * translation_scan_origin, const Eigen::Matrix3f * rotation_map,
  const Eigen::Vector3f * translation_map, const Eigen::Matrix3f * rotation_scan,
  const Eigen::Vector3f * translation_scan, std::uint64_t * points_tensor, cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks = (num_points + threads_per_block - 1) / threads_per_block;

  prepareObstacleTensorKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    input_pointcloud, num_points, points_step, angle_bins, range_bins, min_height, max_height,
    min_angle, angle_increment_inv, range_resolution_inv, projection_dz_threshold,
    translation_scan_origin, rotation_map, translation_map, rotation_scan, translation_scan,
    points_tensor);
}

void fillEmptySpaceLaunch(
  const std::uint64_t * points_tensor, const std::size_t angle_bins, const std::size_t range_bins,
  const float map_resolution_inv, const float scan_origin_x, const float scan_origin_y,
  const float map_origin_x, const float map_origin_y, const int num_cells_x, const int num_cells_y,
  std::uint8_t empty_value, std::uint8_t * costmap_tensor, cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks = (angle_bins * range_bins + threads_per_block - 1) / threads_per_block;

  fillEmptySpaceKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    points_tensor, angle_bins, range_bins, map_resolution_inv, scan_origin_x, scan_origin_y,
    map_origin_x, map_origin_y, num_cells_x, num_cells_y, empty_value, costmap_tensor);
}

void fillUnknownSpaceLaunch(
  const std::uint64_t * raw_points_tensor, const std::uint64_t * obstacle_points_tensor,
  const float obstacle_separation_threshold, const std::size_t angle_bins,
  const std::size_t range_bins, const float map_resolution_inv, const float scan_origin_x,
  const float scan_origin_y, const float scan_origin_z, const float map_origin_x,
  const float map_origin_y, const float robot_pose_z, const int num_cells_x, const int num_cells_y,
  std::uint8_t free_space_value, std::uint8_t no_information_value, std::uint8_t * costmap_tensor,
  cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks = (angle_bins * range_bins + threads_per_block - 1) / threads_per_block;

  fillUnknownSpaceKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    raw_points_tensor, obstacle_points_tensor, obstacle_separation_threshold, angle_bins,
    range_bins, map_resolution_inv, scan_origin_x, scan_origin_y, scan_origin_z, map_origin_x,
    map_origin_y, robot_pose_z, num_cells_x, num_cells_y, free_space_value, no_information_value,
    costmap_tensor);
}

void fillObstaclesLaunch(
  const std::uint64_t * obstacle_points_tensor, const float distance_margin,
  const std::size_t angle_bins, const std::size_t range_bins, const float map_resolution_inv,
  const float map_origin_x, const float map_origin_y, const int num_cells_x, const int num_cells_y,
  std::uint8_t obstacle_value, std::uint8_t * costmap_tensor, cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks = (angle_bins * range_bins + threads_per_block - 1) / threads_per_block;

  fillObstaclesKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    obstacle_points_tensor, distance_margin, angle_bins, range_bins, map_resolution_inv,
    map_origin_x, map_origin_y, num_cells_x, num_cells_y, obstacle_value, costmap_tensor);
}

}  // namespace costmap_2d::map_projective
}  // namespace autoware::occupancy_grid_map
