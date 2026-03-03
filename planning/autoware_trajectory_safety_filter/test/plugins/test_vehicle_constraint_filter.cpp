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

#include "autoware/trajectory_safety_filter/filters/vehicle_constraint_filter.hpp"

#include <gtest/gtest.h>

namespace
{
autoware_planning_msgs::msg::TrajectoryPoint create_trajectory_point(
  double x, double y, double z, double vx, double vy, double time_from_start_sec,
  double heading_rate_rps = 0.0)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = z;
  point.longitudinal_velocity_mps = vx;
  point.lateral_velocity_mps = vy;
  point.heading_rate_rps = heading_rate_rps;
  point.time_from_start.sec = static_cast<int32_t>(time_from_start_sec);
  point.time_from_start.nanosec =
    static_cast<uint32_t>((time_from_start_sec - static_cast<int32_t>(time_from_start_sec)) * 1e9);
  return point;
}
}  // namespace

namespace autoware::trajectory_safety_filter::plugin::testing
{
TEST(VehicleConstraintFilterTest, FeasibleWhenAllConstraintsSatisfied)
{
  // Create a simple trajectory that satisfies all constraints
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 5.5, 0.1, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 6.0, 0.2, 2.0)};

  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase

  VehicleConstraintFilter filter;
  filter.update_parameters(
    {rclcpp::Parameter("vehicle_constraint.max_speed", 10.0),
     rclcpp::Parameter("vehicle_constraint.max_acceleration", 2.0),
     rclcpp::Parameter("vehicle_constraint.max_deceleration", 2.0),
     rclcpp::Parameter("vehicle_constraint.max_steering_angle", 0.5),
     rclcpp::Parameter("vehicle_constraint.max_steering_rate", 0.1)});
  filter.set_vehicle_info(vehicle_info);

  FilterContext context;  // Empty context for now
  auto result = filter.is_feasible(traj_points, context);

  EXPECT_TRUE(result.has_value());
}

TEST(VehicleConstraintFilterTest, InfeasibleWhenSpeedExceedsMax)
{
  // Create a trajectory that exceeds max speed
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 6.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 11.0, 0.0, 2.0)};  // Last point exceeds max speed

  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase

  VehicleConstraintFilter filter;
  filter.update_parameters({rclcpp::Parameter("vehicle_constraint.max_speed", 10.0)});
  filter.set_vehicle_info(vehicle_info);

  FilterContext context;  // Empty context for now
  auto result = filter.is_feasible(traj_points, context);

  EXPECT_FALSE(result.has_value());
}

TEST(VehicleConstraintFilterTest, InfeasibleWhenAccelerationExceedsMax)
{
  // Create a trajectory that exceeds max acceleration
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 1.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 3.5, 0.0, 2.0)};  // Last point exceeds max acceleration

  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase

  VehicleConstraintFilter filter;
  filter.update_parameters({
    rclcpp::Parameter("vehicle_constraint.max_acceleration", 2.0),
  });
  filter.set_vehicle_info(vehicle_info);

  FilterContext context;  // Empty context for now
  auto result = filter.is_feasible(traj_points, context);

  EXPECT_FALSE(result.has_value());
}

TEST(VehicleConstraintFilterTest, InfeasibleWhenDecelerationExceedsMax)
{
  // Create a trajectory that exceeds max deceleration
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 3.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 0.0, 0.0, 2.0)};  // Last point exceeds max deceleration

  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase

  VehicleConstraintFilter filter;
  filter.update_parameters({
    rclcpp::Parameter("vehicle_constraint.max_deceleration", 2.0),
  });
  filter.set_vehicle_info(vehicle_info);

  FilterContext context;  // Empty context for now
  auto result = filter.is_feasible(traj_points, context);

  EXPECT_FALSE(result.has_value());
}

TEST(VehicleConstraintFilterTest, InfeasibleWhenSteeringAngleExceedsMax)
{
  // Create a trajectory that exceeds max steering angle
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 0.0, 0.0, 5.0, 0.0, 1.0),   // Exceeds max steering angle
    create_trajectory_point(1.0, 1.0, 0.0, 5.0, 0.0, 2.0)};  // Exceeds max steering angle

  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase

  VehicleConstraintFilter filter;

  filter.update_parameters({
    rclcpp::Parameter("vehicle_constraint.max_steering_angle", 0.5),
  });
  filter.set_vehicle_info(vehicle_info);

  FilterContext context;  // Empty context for now
  auto result = filter.is_feasible(traj_points, context);

  EXPECT_FALSE(result.has_value());
}

TEST(VehicleConstraintFilterTest, InfeasibleWhenSteeringRateExceedsMax)
{
  // Create a trajectory that exceeds max steering rate
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 0.0, 0.0, 5.0, 0.0, 1.0),
    create_trajectory_point(2.0, 0.0, 0.0, 5.0, 0.0, 2.0),
    create_trajectory_point(2.0, 1.0, 0.0, 5.0, 0.0, 3.0)};  // Exceeds max steering rate

  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase

  VehicleConstraintFilter filter;
  filter.update_parameters({
    rclcpp::Parameter("vehicle_constraint.max_steering_rate", 0.1),
  });
  filter.set_vehicle_info(vehicle_info);

  FilterContext context;  // Empty context for now
  auto result = filter.is_feasible(traj_points, context);

  EXPECT_FALSE(result.has_value());
}

// --- is_speed_ok(...) tests ---

TEST(IsSpeedOkTest, TrueWhenAllSpeedsBelowMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 6.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 7.0, 0.0, 2.0)};
  double max_speed = 10.0;  // m/s

  EXPECT_TRUE(is_speed_ok(traj_points, max_speed));
}

TEST(IsSpeedOkTest, FalseWhenAnySpeedAboveMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 6.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 11.0, 0.0, 2.0)};
  double max_speed = 10.0;  // m/s

  EXPECT_FALSE(is_speed_ok(traj_points, max_speed));
}

// --- is_acceleration_ok(...) tests ---

TEST(IsAccelerationOkTest, TrueWhenAllAccelerationsBelowMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 1.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 2.0, 0.0, 2.0)};
  double max_acceleration = 2.0;  // m/s^2

  EXPECT_TRUE(is_acceleration_ok(traj_points, max_acceleration));
}

TEST(IsAccelerationOkTest, FalseWhenAnyAccelerationAboveMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 1.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 3.5, 0.0, 2.0)};
  double max_acceleration = 2.0;  // m/s^2

  EXPECT_FALSE(is_acceleration_ok(traj_points, max_acceleration));
}

// --- is_deceleration_ok(...) tests ---

TEST(IsDecelerationOkTest, TrueWhenAllDecelerationsBelowMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 3.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 2.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 1.0, 0.0, 2.0)};
  double max_deceleration = 2.0;  // m/s^2

  EXPECT_TRUE(is_deceleration_ok(traj_points, max_deceleration));
}

TEST(IsDecelerationOkTest, FalseWhenAnyDecelerationAboveMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 1.0, 0.0, 3.0, 0.0, 1.0),
    create_trajectory_point(2.0, 2.0, 0.0, 0.0, 0.0, 2.0)};
  double max_deceleration = 2.0;  // m/s^2

  EXPECT_FALSE(is_deceleration_ok(traj_points, max_deceleration));
}

// --- is_steering_angle_ok(...) tests ---

TEST(IsSteeringAngleOkTest, TrueWhenAllSteeringAnglesBelowMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 0.0, 0.0, 5.0, 0.0, 1.0),
    create_trajectory_point(2.0, 0.0, 0.0, 5.0, 0.0, 2.0)};
  VehicleInfo vehicle_info;         // Fill in with appropriate values
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase
  double max_steering_angle = 0.5;  // rad

  EXPECT_TRUE(is_steering_angle_ok(traj_points, vehicle_info, max_steering_angle));
}

TEST(IsSteeringAngleOkTest, FalseWhenAnySteeringAngleAboveMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 0.0, 0.0, 5.0, 0.0, 1.0),
    create_trajectory_point(1.0, 1.0, 0.0, 5.0, 0.0, 2.0)};
  VehicleInfo vehicle_info;         // Fill in with appropriate values
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase
  double max_steering_angle = 0.5;  // rad

  EXPECT_FALSE(is_steering_angle_ok(traj_points, vehicle_info, max_steering_angle));
}

// --- is_steering_rate_ok(...) tests ---

TEST(IsSteeringRateOkTest, TrueWhenAllSteeringRatesBelowMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 0.0, 0.0, 5.0, 0.0, 1.0),
    create_trajectory_point(2.0, 0.01, 0.0, 5.0, 0.0, 2.0),
    create_trajectory_point(3.0, 0.02, 0.0, 5.0, 0.0, 3.0)};
  VehicleInfo vehicle_info;         // Fill in with appropriate values
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase
  double max_steering_rate = 0.1;   // rad/s

  EXPECT_TRUE(is_steering_rate_ok(traj_points, vehicle_info, max_steering_rate));
}

TEST(IsSteeringRateOkTest, FalseWhenAnySteeringRateAboveMax)
{
  TrajectoryPoints traj_points = {
    create_trajectory_point(0.0, 0.0, 0.0, 5.0, 0.0, 0.0),
    create_trajectory_point(1.0, 0.0, 0.0, 5.0, 0.0, 1.0),
    create_trajectory_point(2.0, 0.0, 0.0, 5.0, 0.0, 2.0),
    create_trajectory_point(2.0, 1.0, 0.0, 5.0, 0.0, 3.0)};
  VehicleInfo vehicle_info;         // Fill in with appropriate values
  vehicle_info.wheel_base_m = 2.5;  // Example wheelbase
  double max_steering_rate = 0.1;   // rad/s

  EXPECT_FALSE(is_steering_rate_ok(traj_points, vehicle_info, max_steering_rate));
}
}  // namespace autoware::trajectory_safety_filter::plugin::testing
