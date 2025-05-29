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

#ifndef AUTOWARE__PLANNING_VALIDATOR_TEST_UTILS__PLANNING_VALIDATOR_TEST_UTILS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_TEST_UTILS__PLANNING_VALIDATOR_TEST_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <limits>
#include <string>
#include <vector>

namespace autoware::planning_validator::test_utils
{
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

Trajectory generateTrajectoryWithConstantAcceleration(
  const double interval_distance, const double speed, const double yaw, const size_t size,
  const double acceleration);

Trajectory generateTrajectoryWithVariableAcceleration(
  const double interval_distance, const double initial_speed, const double yaw, const size_t size,
  const std::function<double(size_t)> & acceleration_pattern);

Trajectory generateTrajectoryWithSinusoidalAcceleration(
  const double interval_distance, const double initial_speed, const double yaw, const size_t size,
  const double max_acceleration, const double oscillation_period);

Trajectory generateTrajectoryWithStepAcceleration(
  const double interval_distance, const double initial_speed, const double yaw, const size_t size,
  const std::vector<double> & acceleration_values, const size_t steps_per_value);

Trajectory generateTrajectory(
  const double interval_distance, const double speed = 1.0, const double yaw = 0.0,
  const size_t size = 10);

Trajectory generateTrajectoryWithConstantCurvature(
  const double interval_distance, const double speed, const double curvature, const size_t size,
  const double wheelbase);

Trajectory generateTrajectoryWithConstantSteering(
  const double interval_distance, const double speed, const double steering_angle_rad,
  const size_t size, const double wheelbase);

Trajectory generateTrajectoryWithConstantSteeringRate(
  const double interval_distance, const double speed, const double steering_rate, const size_t size,
  const double wheelbase);

Trajectory generateShiftedTrajectory(
  const Trajectory & trajectory, const double lat_shift = 0.0, const double lon_shift = 0.0,
  const size_t size = std::numeric_limits<size_t>::max());

Trajectory generateNanTrajectory();

Trajectory generateInfTrajectory();

Trajectory generateBadCurvatureTrajectory();

Odometry generateDefaultOdometry(const double x = 0.0, const double y = 0.0, const double vx = 0.0);

AccelWithCovarianceStamped generateDefaultAcceleration(const double ax = 0.0);

rclcpp::NodeOptions getNodeOptionsWithDefaultParams();

}  // namespace autoware::planning_validator::test_utils

#endif  // AUTOWARE__PLANNING_VALIDATOR_TEST_UTILS__PLANNING_VALIDATOR_TEST_UTILS_HPP_
