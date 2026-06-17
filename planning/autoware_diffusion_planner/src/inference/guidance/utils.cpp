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

#include "utils.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

size_t trajectory_index(
  const int64_t batch, const int64_t agent, const int64_t t, const int64_t dim)
{
  return ((static_cast<size_t>(batch) * MAX_NUM_AGENTS + agent) * (OUTPUT_T + 1) + t) * POSE_DIM +
         dim;
}

std::vector<std::vector<Eigen::Vector2d>> extract_denormalized_trajectories_from_model_output(
  const std::vector<float> & model_output, float x_mean, float y_mean, float x_std, float y_std)
{
  const size_t trajectory_size = static_cast<size_t>(MAX_NUM_AGENTS) * (OUTPUT_T + 1) * POSE_DIM;
  if (trajectory_size == 0 || model_output.empty() || model_output.size() % trajectory_size != 0) {
    return {};
  }

  const int64_t batch_size = static_cast<int64_t>(model_output.size() / trajectory_size);
  std::vector<std::vector<Eigen::Vector2d>> trajectories;
  trajectories.reserve(batch_size);

  for (int64_t b = 0; b < batch_size; ++b) {
    std::vector<Eigen::Vector2d> trajectory;
    trajectory.reserve(OUTPUT_T + 1);

    trajectory.emplace_back(0.0, 0.0);

    for (int64_t t = 1; t <= OUTPUT_T; ++t) {
      const double normalized_x = model_output[trajectory_index(b, k_ego_agent_index, t, 0)];
      const double normalized_y = model_output[trajectory_index(b, k_ego_agent_index, t, 1)];
      trajectory.emplace_back(normalized_x * x_std + x_mean, normalized_y * y_std + y_mean);
    }

    trajectories.push_back(std::move(trajectory));
  }

  return trajectories;
}

std::vector<float> create_delta_from_denormalized_trajectories(
  const std::vector<std::vector<Eigen::Vector2d>> & trajectories,
  const std::vector<std::vector<Eigen::Vector2d>> & guided_trajectories, float x_std, float y_std)
{
  const size_t trajectory_size = static_cast<size_t>(MAX_NUM_AGENTS) * (OUTPUT_T + 1) * POSE_DIM;
  if (trajectory_size == 0 || trajectories.empty()) {
    return {};
  }

  const int64_t batch_size = static_cast<int64_t>(trajectories.size());
  if (guided_trajectories.size() != trajectories.size()) {
    return {};
  }

  for (int64_t b = 0; b < batch_size; ++b) {
    const auto & trajectory = trajectories[b];
    const auto & guided_trajectory = guided_trajectories[b];
    if (
      trajectory.size() != static_cast<size_t>(OUTPUT_T + 1) ||
      guided_trajectory.size() != trajectory.size()) {
      return {};
    }
  }

  std::vector<float> delta(static_cast<size_t>(batch_size) * trajectory_size, 0.0f);
  for (int64_t b = 0; b < batch_size; ++b) {
    const auto & trajectory = trajectories[b];
    const auto & guided_trajectory = guided_trajectories[b];
    for (int64_t t = 0; t <= OUTPUT_T; ++t) {
      const size_t x_idx = trajectory_index(b, k_ego_agent_index, t, 0);
      const size_t y_idx = trajectory_index(b, k_ego_agent_index, t, 1);
      delta[x_idx] = static_cast<float>((guided_trajectory[t].x() - trajectory[t].x()) / x_std);
      delta[y_idx] = static_cast<float>((guided_trajectory[t].y() - trajectory[t].y()) / y_std);
    }
  }

  return delta;
}

}  // namespace autoware::diffusion_planner
