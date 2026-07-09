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

#include "autoware/diffusion_planner/conversion/agent_history_alignment.hpp"

#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::diffusion_planner
{

MotionState propagate_motion(
  const MotionState & state, const double speed, const double yaw_rate, double dt,
  const HistoryAlignmentParams & params)
{
  if (dt <= 0.0) {
    return state;
  }
  dt = std::min(dt, params.max_extrapolation_time);

  // A near-zero yaw rate degenerates the constant-turn-rate model into straight constant velocity.
  const double effective_yaw_rate =
    (std::abs(yaw_rate) < params.yaw_rate_threshold) ? 0.0 : yaw_rate;

  // Split the horizon into equal sub-steps to bound the forward-Euler linearization error, matching
  // the tracker's dt_max sub-stepping.
  const double sub_step_max = std::max(params.dt_sub_step_max, 1e-3);
  const int num_steps = std::max(1, static_cast<int>(std::ceil(dt / sub_step_max)));
  const double step = dt / static_cast<double>(num_steps);

  MotionState s = state;
  for (int i = 0; i < num_steps; ++i) {
    // Forward-Euler CTRV step (CV when effective_yaw_rate == 0):
    //   x   += v * cos(yaw) * step
    //   y   += v * sin(yaw) * step
    //   yaw += w * step
    s.x += speed * std::cos(s.yaw) * step;
    s.y += speed * std::sin(s.yaw) * step;
    s.yaw += effective_yaw_rate * step;
  }
  s.yaw = autoware_utils_math::normalize_radian(s.yaw);
  return s;
}

double interpolate_yaw(const double yaw_a, const double yaw_b, const double ratio)
{
  const double delta = autoware_utils_math::normalize_radian(yaw_b - yaw_a);
  return autoware_utils_math::normalize_radian(yaw_a + ratio * delta);
}

}  // namespace autoware::diffusion_planner
