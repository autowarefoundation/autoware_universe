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

#ifndef INFERENCE__GUIDANCE__UTILS_HPP_
#define INFERENCE__GUIDANCE__UTILS_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner
{

constexpr int64_t k_ego_agent_index = 0;

size_t trajectory_index(
  const int64_t batch, const int64_t agent, const int64_t t, const int64_t dim);

std::vector<std::vector<Eigen::Vector2d>> extract_denormalized_trajectories_from_model_output(
  const std::vector<float> & model_output, float x_mean, float y_mean, float x_std, float y_std);

std::vector<float> create_delta_from_denormalized_trajectories(
  const std::vector<std::vector<Eigen::Vector2d>> & trajectories,
  const std::vector<std::vector<Eigen::Vector2d>> & guided_trajectories, float x_std, float y_std);

}  // namespace autoware::diffusion_planner

#endif  // INFERENCE__GUIDANCE__UTILS_HPP_
