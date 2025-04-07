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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include "autoware/probabilistic_occupancy_grid_map/utils/utils_kernel.hpp"

namespace autoware::occupancy_grid_map
{
namespace utils
{

inline __device__ bool worldToMap(
  float wx, float wy, unsigned int & mx, unsigned int & my, float origin_x, float origin_y,
  float resolution_inv, int size_x, int size_y)
{
  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<int>(std::floor((wx - origin_x) * resolution_inv));
  my = static_cast<int>(std::floor((wy - origin_y) * resolution_inv));

  if (mx < size_x && my < size_y) {
    return true;
  }

  return false;
}

__device__ void setCellValue(
  float wx, float wy, float origin_x, float origin_y, float resolution_inv, int size_x, int size_y,
  std::uint8_t value, std::uint8_t * costmap_tensor)
{
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my, origin_x, origin_y, resolution_inv, size_x, size_y)) {
    return;
  }

  costmap_tensor[my * size_x + mx] = value;
}

inline __device__ void bresenham2D(
  unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b,
  unsigned int offset, unsigned int max_length, std::uint8_t cost, std::uint8_t * costmap_tensor)
{
  unsigned int end = min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i) {
    costmap_tensor[offset] = cost;
    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da) {
      offset += offset_b;
      error_b -= abs_da;
    }
  }
  costmap_tensor[offset] = cost;
}

/**
 * @brief  Raytrace a line and apply some action at each step
 * @param  at The action to take... a functor
 * @param  x0 The starting x coordinate
 * @param  y0 The starting y coordinate
 * @param  x1 The ending x coordinate
 * @param  y1 The ending y coordinate
 * @param  max_length The maximum desired length of the segment...
 * allows you to not go all the way to the endpoint
 * @param  min_length The minimum desired length of the segment
 */
inline __device__ void raytraceLine(
  unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length,
  unsigned int min_length, unsigned int size_x, std::uint8_t cost, std::uint8_t * costmap_tensor)
{
  int dx_full = x1 - x0;
  int dy_full = y1 - y0;

  // we need to chose how much to scale our dominant dimension,
  // based on the maximum length of the line
  float dist = sqrt((float)(dx_full * dx_full + dy_full * dy_full));  // hypot(dx_full, dy_full);
  if (dist < min_length) {
    return;
  }

  unsigned int min_x0, min_y0;
  if (dist > 0.0) {
    // Adjust starting point and offset to start from min_length distance
    min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
    min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
  } else {
    // dist can be 0 if [x0, y0]==[x1, y1].
    // In this case only this cell should be processed.
    min_x0 = x0;
    min_y0 = y0;
  }
  unsigned int offset = min_y0 * size_x + min_x0;

  int dx = x1 - min_x0;
  int dy = y1 - min_y0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = dx > 0 ? 1 : -1;            // sign(dx);
  int offset_dy = dy > 0 ? size_x : -size_x;  // sign(dy) * size_x;

  constexpr float epsilon = 1e-6;
  float scale = (dist < epsilon) ? 1.0 : min(1.f, max_length / dist);
  // if x is dominant
  if (abs_dx >= abs_dy) {
    int error_y = abs_dx / 2;

    bresenham2D(
      abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx), cost,
      costmap_tensor);
    return;
  }

  // otherwise y is dominant
  int error_x = abs_dy / 2;

  bresenham2D(
    abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy), cost,
    costmap_tensor);
}

void __device__ raytrace(
  const float source_x, const float source_y, const float target_x, const float target_y,
  const float origin_x, float origin_y, const float resolution_inv, const int size_x,
  const int size_y, const std::uint8_t cost, std::uint8_t * costmap_tensor)
{
  unsigned int x0{};
  unsigned int y0{};
  const float ox{source_x};
  const float oy{source_y};

  if (!worldToMap(ox, oy, x0, y0, origin_x, origin_y, resolution_inv, size_x, size_y)) {
    return;
  }

  // we can pre-compute the endpoints of the map outside of the inner loop... we'll need these later
  const float resolution = 1.0 / resolution_inv;
  const float map_end_x = origin_x + size_x * resolution;
  const float map_end_y = origin_y + size_y * resolution;

  float wx = target_x;
  float wy = target_y;

  // now we also need to make sure that the endpoint we're ray-tracing
  // to isn't off the costmap and scale if necessary
  const float a = wx - ox;
  const float b = wy - oy;

  // the minimum value to raytrace from is the origin
  if (wx < origin_x) {
    const float t = (origin_x - ox) / a;
    wx = origin_x;
    wy = oy + b * t;
  }
  if (wy < origin_y) {
    const float t = (origin_y - oy) / b;
    wx = ox + a * t;
    wy = origin_y;
  }

  // the maximum value to raytrace to is the end of the map
  if (wx > map_end_x) {
    const float t = (map_end_x - ox) / a;
    wx = map_end_x - .001;
    wy = oy + b * t;
  }
  if (wy > map_end_y) {
    const float t = (map_end_y - oy) / b;
    wx = ox + a * t;
    wy = map_end_y - .001;
  }

  // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
  unsigned int x1{};
  unsigned int y1{};

  // check for legality just in case
  if (!worldToMap(wx, wy, x1, y1, origin_x, origin_y, resolution_inv, size_x, size_y)) {
    return;
  }

  constexpr unsigned int cell_raytrace_range = 10000;  // large number to ignore range threshold
  raytraceLine(x0, y0, x1, y1, cell_raytrace_range, 0, size_x, cost, costmap_tensor);
}

__global__ void transformPointCloudKernel(
  std::uint8_t * points, std::size_t num_points, std::size_t points_step,
  const Eigen::Matrix3f * rotation, const Eigen::Vector3f * translation)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }

  Eigen::Map<Eigen::Vector3f> point_map(reinterpret_cast<float *>(points + idx * points_step));
  point_map = rotation[0] * point_map + translation[0];
}

__global__ void copyMapRegionKernel(
  const std::uint8_t * source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
  unsigned int sm_size_x, unsigned int sm_size_y, std::uint8_t * dest_map,
  unsigned int dm_lower_left_x, unsigned int dm_lower_left_y, unsigned int dm_size_x,
  unsigned int dm_size_y, unsigned int region_size_x, unsigned int region_size_y)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= region_size_x * region_size_y) return;

  const int region_y = idx / region_size_x;
  const int region_x = idx % region_size_x;

  const int sm_index =
    sm_lower_left_y * sm_size_x + sm_lower_left_x + region_y * sm_size_x + region_x;
  const int dm_index =
    dm_lower_left_y * dm_size_x + dm_lower_left_x + region_y * dm_size_x + region_x;

  if (sm_index < sm_size_x * sm_size_y && dm_index < dm_size_x * dm_size_y) {
    dest_map[dm_index] = source_map[sm_index];
  }
}

void transformPointCloudLaunch(
  std::uint8_t * points, std::size_t num_points, std::size_t points_step,
  const Eigen::Matrix3f * rotation, const Eigen::Vector3f * translation, cudaStream_t stream)
{
  // Launch kernel
  const int threads_per_block = 256;
  int num_blocks = (num_points + threads_per_block - 1) / threads_per_block;
  transformPointCloudKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    points, num_points, points_step, rotation, translation);
}

void copyMapRegionLaunch(
  const std::uint8_t * source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
  unsigned int sm_size_x, unsigned int sm_size_y, std::uint8_t * dest_map,
  unsigned int dm_lower_left_x, unsigned int dm_lower_left_y, unsigned int dm_size_x,
  unsigned int dm_size_y, unsigned int region_size_x, unsigned int region_size_y,
  cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks =
    (region_size_x * region_size_y + threads_per_block - 1) / threads_per_block;

  if (num_blocks == 0) {
    return;
  }

  copyMapRegionKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    source_map, sm_lower_left_x, sm_lower_left_y, sm_size_x, sm_size_y, dest_map, dm_lower_left_x,
    dm_lower_left_y, dm_size_x, dm_size_y, region_size_x, region_size_y);
}

}  // namespace utils
}  // namespace autoware::occupancy_grid_map
