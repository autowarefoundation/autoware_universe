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

#ifndef AUTOWARE__probabilistic_occupancy_grid_map__UPDATER__log_odds_bayes_filter_updater_kernel_HPP_
#define AUTOWARE__probabilistic_occupancy_grid_map__UPDATER__log_odds_bayes_filter_updater_kernel_HPP_

#include "autoware/probabilistic_occupancy_grid_map/updater/log_odds_bayes_filter_updater_kernel.hpp"

#include <cstdint>

#define EPSILON_PROB 0.03

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{

__host__ __device__ __forceinline__ double convertCharToProbability(const std::uint8_t value)
{
  return static_cast<double>(value) / 255.0;
}

__host__ __device__ __forceinline__ std::uint8_t convertProbabilityToChar(const double value)
{
  return static_cast<std::uint8_t>(std::max(0.0, std::min(1.0, value)) * 255.0);
  ;
}

__host__ __device__ __forceinline__ double logOddsFusion(const double p1, const double p2)
{
  double log_odds = 0.0;

  const double p1_norm = std::max(EPSILON_PROB, std::min(1.0 - EPSILON_PROB, p1));
  log_odds += std::log(p1_norm / (1.0 - p1_norm));

  const double p2_norm = std::max(EPSILON_PROB, std::min(1.0 - EPSILON_PROB, p2));
  log_odds += std::log(p2_norm / (1.0 - p2_norm));

  return 1.0 / (1.0 + std::exp(-log_odds));
}

// cspell: ignore LOBF
__global__ void applyLOBFKernel(
  const std::uint8_t * z_costmap, const std::uint8_t unknown_value, const int num_elements,
  std::uint8_t * o_costmap)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_elements) {
    return;
  }

  const std::uint8_t z = z_costmap[idx];
  const std::uint8_t o = o_costmap[idx];

  const std::uint8_t unknown_margin = 1;
  const double tau = 0.75;
  const double sample_time = 0.1;

  if (z >= unknown_value - unknown_margin && z <= unknown_value + unknown_margin) {
    const int diff = static_cast<int>(o) - static_cast<int>(unknown_value);
    const double decay = std::exp(-sample_time / tau);
    const double fused = static_cast<double>(unknown_value) + static_cast<double>(diff) * decay;
    o_costmap[idx] = static_cast<std::uint8_t>(fused);
  } else {
    const unsigned char fused = convertProbabilityToChar(
      logOddsFusion(convertCharToProbability(z), convertCharToProbability(o)));
    o_costmap[idx] = static_cast<std::uint8_t>(fused);
  }
}

void applyLOBFLaunch(
  const std::uint8_t * z_costmap, const std::uint8_t no_information_value, const int num_elements,
  std::uint8_t * o_costmap, cudaStream_t stream)
{
  const int threads_per_block = 256;
  const int num_blocks = (num_elements + threads_per_block - 1) / threads_per_block;
  applyLOBFKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    z_costmap, no_information_value, num_elements, o_costmap);
}

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__probabilistic_occupancy_grid_map__UPDATER__log_odds_bayes_filter_updater_kernel_HPP_
