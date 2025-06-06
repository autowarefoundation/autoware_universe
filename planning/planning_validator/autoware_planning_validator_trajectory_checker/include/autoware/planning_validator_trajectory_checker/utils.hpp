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

#ifndef AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__UTILS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator::trajectory_checker_utils
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

std::pair<double, size_t> calcMaxCurvature(const Trajectory & trajectory);

void calc_interval_distance(
  const Trajectory & trajectory, std::vector<double> & interval_distance_vector);

std::pair<double, size_t> calcMaxIntervalDistance(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxLateralAcceleration(const Trajectory & trajectory);

std::pair<double, size_t> calc_max_lateral_jerk(const Trajectory & trajectory);

std::pair<double, size_t> getMaxLongitudinalAcc(const Trajectory & trajectory);

std::pair<double, size_t> getMinLongitudinalAcc(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxRelativeAngles(const Trajectory & trajectory);

std::pair<double, size_t> calcMaxSteeringAngles(
  const Trajectory & trajectory, const double wheelbase);

std::pair<double, size_t> calcMaxSteeringRates(
  const Trajectory & trajectory, const double wheelbase);

bool checkFinite(const TrajectoryPoint & point);

}  // namespace autoware::planning_validator::trajectory_checker_utils

#endif  // AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__UTILS_HPP_
