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

#ifndef AUTOWARE__probabilistic_occupancy_grid_map__UPDATER__BINARY_BAYES_FILTER_UPDATER_KERNEL_HPP_
#define AUTOWARE__probabilistic_occupancy_grid_map__UPDATER__BINARY_BAYES_FILTER_UPDATER_KERNEL_HPP_

#include "autoware/probabilistic_occupancy_grid_map/updater/binary_bayes_filter_updater_kernel.hpp"

#include <cstdint>

__global__ void applyBBFKernel(
  const std::uint8_t * z_costmap, const float * probability_matrix, const int num_states,
  const int free_index, const int occupied_index, const std::uint8_t free_space_value,
  const std::uint8_t lethal_obstacle_value, const std::uint8_t no_information_value,
  const double v_ratio_, const int num_elements, std::uint8_t * o_costmap)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_elements) {
    return;
  }

  const std::uint8_t z = z_costmap[idx];
  const std::uint8_t o = o_costmap[idx];

  constexpr float cost2p = 1.f / 255.f;
  const float po = o * cost2p;
  float pz{};
  float not_pz{};
  float po_hat{};
  if (z == lethal_obstacle_value) {
    pz = probability_matrix[occupied_index * num_states + occupied_index];
    not_pz = probability_matrix[free_index * num_states + occupied_index];
    po_hat = ((po * pz) / ((po * pz) + ((1.f - po) * not_pz)));
  } else if (z == free_space_value) {
    pz = 1.f - probability_matrix[free_index * num_states + free_index];
    not_pz = 1.f - probability_matrix[occupied_index * num_states + free_index];
    po_hat = ((po * pz) / ((po * pz) + ((1.f - po) * not_pz)));
  } else if (z == no_information_value) {
    const float inv_v_ratio = 1.f / v_ratio_;
    po_hat = ((po + (0.5f * inv_v_ratio)) / ((1.f * inv_v_ratio) + 1.f));
  }

  o_costmap[idx] = std::min(
    std::max(
      static_cast<unsigned char>(std::lround(po_hat * 255.f)), static_cast<unsigned char>(1)),
    static_cast<unsigned char>(254));
}

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{

void applyBBFLaunch(
  const std::uint8_t * z_costmap, const float * probability_matrix, const int num_states,
  const int free_index, const int occupied_index, const std::uint8_t free_space_value,
  const std::uint8_t lethal_obstacle_value, const std::uint8_t no_information_value,
  const double v_ratio_, const int num_elements, std::uint8_t * o_costmap, cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks = (num_elements + threads_per_block - 1) / threads_per_block;
  applyBBFKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    z_costmap, probability_matrix, num_states, free_index, occupied_index, free_space_value,
    lethal_obstacle_value, no_information_value, v_ratio_, num_elements, o_costmap);
}

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__probabilistic_occupancy_grid_map__UPDATER__BINARY_BAYES_FILTER_UPDATER_KERNEL_HPP_
