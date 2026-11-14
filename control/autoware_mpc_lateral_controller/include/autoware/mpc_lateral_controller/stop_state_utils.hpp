// Copyright 2026 The Autoware Foundation
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__STOP_STATE_UTILS_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__STOP_STATE_UTILS_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace autoware::motion::control::mpc_lateral_controller
{
// The candidate point contributes only when its arc length from the nearest
// point is inside the lookahead. Check the boundary *before* taking its speed.
inline double min_target_velocity_within_distance(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const std::size_t nearest,
  const double distance_margin)
{
  const auto & points = trajectory.points;
  double min_velocity = points.at(nearest).longitudinal_velocity_mps;
  double covered_distance = 0.0;
  for (std::size_t i = nearest + 1; i < points.size(); ++i) {
    const auto & prev = points[i - 1].pose.position;
    const auto & next = points[i].pose.position;
    covered_distance += std::hypot(next.x - prev.x, next.y - prev.y);
    if (covered_distance > distance_margin) {
      break;
    }
    min_velocity = std::min(
      min_velocity, static_cast<double>(points[i].longitudinal_velocity_mps));
  }
  return min_velocity;
}
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__STOP_STATE_UTILS_HPP_
