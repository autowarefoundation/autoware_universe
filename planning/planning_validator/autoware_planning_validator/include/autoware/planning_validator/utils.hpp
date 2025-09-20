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

#ifndef AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

std::pair<double, size_t> getAbsMaxValAndIdx(const std::vector<double> & v);

Trajectory resampleTrajectory(const Trajectory & trajectory, const double min_interval);

Trajectory getStopTrajectory(
  const Trajectory & trajectory, const int nearest_traj_idx, const double current_vel,
  const double current_accel, const double decel, const double jerk_limit);

void shiftPose(geometry_msgs::msg::Pose & pose, double longitudinal);

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__UTILS_HPP_
