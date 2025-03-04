// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl/processing/postprocessor.hpp"

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/prediction.hpp"

#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

namespace autoware::simpl::processing
{
namespace
{
/**
 * @brief Transform predicted state from agent centric to map coordinate.
 *
 * @param x Predicted location x.
 * @param y Predicted location y.
 * @param vx Predicted x-direction velocity.
 * @param vy Predicted y-direction velocity.
 * @param current_state Current agent state w.r.t map coordinate.
 */
std::tuple<double, double, double, double> transform_2d(
  double x, double y, double vx, double vy, const archetype::AgentState & current_state)
{
  const double cos = std::cos(current_state.yaw);
  const double sin = std::sin(current_state.yaw);

  double ret_x = x * cos - y * sin + current_state.x;
  double ret_y = x * sin + y * cos + current_state.y;
  double ret_vx = vx * cos - vy * sin;
  double ret_vy = vx * sin + vy * cos;

  return {ret_x, ret_y, ret_vx, ret_vy};
}
}  // namespace

using output_type = PostProcessor::output_type;

PostProcessor::PostProcessor(size_t num_mode, size_t num_future)
: num_mode_(num_mode), num_future_(num_future)
{
}

output_type PostProcessor::process(
  const std::vector<float> & scores, const std::vector<float> & trajectories,
  archetype::AgentStates & current_states) const
{
  const auto num_attribute = archetype::PredictedState::num_attribute();

  output_type predictions;
  for (size_t n = 0; n < current_states.size(); ++n) {
    const auto & current_state = current_states.at(n);
    std::vector<double> agent_scores;
    std::vector<double> agent_trajectories;
    for (size_t m = 0; m < num_mode_; ++m) {
      for (size_t t = 0; t < num_future_; ++t) {
        auto score_idx = n * num_mode_ * num_future_ + m;
        agent_scores.emplace_back(scores.at(score_idx));

        // Index: (n * M * Tf + m * Tf + t) * Dp
        auto trajectory_idx = (n * num_mode_ * num_future_ + m * num_future_ + t) * num_attribute;
        double x = static_cast<double>(trajectories.at(trajectory_idx));
        double y = static_cast<double>(trajectories.at(trajectory_idx + 1));
        double vx = static_cast<double>(trajectories.at(trajectory_idx + 2));
        double vy = static_cast<double>(trajectories.at(trajectory_idx + 3));

        // Transform from current state centric to map coordinate
        auto [ret_x, ret_y, ret_vx, ret_vy] = transform_2d(x, y, vx, vy, current_state);
        agent_trajectories.emplace_back(ret_x);
        agent_trajectories.emplace_back(ret_y);
        agent_trajectories.emplace_back(ret_vx);
        agent_trajectories.emplace_back(ret_vy);
      }
    }
    predictions.emplace_back(num_mode_, num_future_, agent_scores, agent_trajectories);
  }
  return predictions;
}
}  // namespace autoware::simpl::processing
