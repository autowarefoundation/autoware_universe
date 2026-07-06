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

#include "velocity_smoother.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{

TrajectoryPoint make_traj_point(double x, double y, float vel, float acc = 0.0f)
{
  TrajectoryPoint pt;
  pt.pose.position.x = x;
  pt.pose.position.y = y;
  pt.pose.position.z = 0.0;
  pt.pose.orientation.w = 1.0;
  pt.longitudinal_velocity_mps = vel;
  pt.acceleration_mps2 = acc;
  return pt;
}

TrajectoryPoints make_straight_trajectory(
  size_t num_points, double spacing, float velocity, float acc = 0.0f)
{
  TrajectoryPoints points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    points.push_back(make_traj_point(spacing * static_cast<double>(i), 0.0, velocity, acc));
  }
  return points;
}

nav_msgs::msg::Odometry make_odometry(double x, double y, double speed)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = speed;
  return odom;
}

VelocitySmootherParams make_default_params()
{
  VelocitySmootherParams params;
  params.nearest_dist_threshold_m = 3.0;
  params.nearest_yaw_threshold_deg = 45.0;
  params.target_pull_out_speed_mps = 2.0;
  params.target_pull_out_acc_mps2 = 1.0;
  params.max_speed_mps = 20.0;
  params.max_lateral_accel_mps2 = 2.0;
  params.stop_dist_to_prohibit_engage = 3.0;
  params.set_engage_speed = false;
  params.limit_speed = false;
  params.limit_lateral_acceleration = false;
  params.smooth_velocities = false;
  return params;
}

}  // namespace

// ============================================================
// VelocitySmoother construction without Node
// ============================================================

TEST(VelocitySmootherTest, ConstructWithoutNode)
{
  auto params = make_default_params();
  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  // Construct without JerkFilteredSmoother (nullptr) - smoothing disabled
  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  auto traj = make_straight_trajectory(10, 1.0, 10.0f);
  auto odom = make_odometry(0.0, 0.0, 10.0);
  // Should not crash with smooth_velocities=false and nullptr smoother
  smoother.optimize(traj, odom, 0.0);
  EXPECT_FALSE(traj.empty());
}

// ============================================================
// max_speed_mps limit
// ============================================================

TEST(VelocitySmootherTest, LimitSpeedClampsVelocity)
{
  auto params = make_default_params();
  params.limit_speed = true;
  params.max_speed_mps = 5.0;

  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  auto traj = make_straight_trajectory(10, 1.0, 10.0f);
  auto odom = make_odometry(0.0, 0.0, 10.0);
  smoother.optimize(traj, odom, 0.0);

  for (const auto & pt : traj) {
    EXPECT_LE(pt.longitudinal_velocity_mps, static_cast<float>(params.max_speed_mps) + 1e-3f);
  }
}

TEST(VelocitySmootherTest, LimitSpeedDoesNotAffectSlowerPoints)
{
  auto params = make_default_params();
  params.limit_speed = true;
  params.max_speed_mps = 20.0;

  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  auto traj = make_straight_trajectory(10, 1.0, 10.0f);
  auto odom = make_odometry(0.0, 0.0, 10.0);
  smoother.optimize(traj, odom, 0.0);

  // All points should remain at 10.0 since max_speed is 20.0
  for (size_t i = 0; i < traj.size(); ++i) {
    EXPECT_NEAR(traj[i].longitudinal_velocity_mps, 10.0f, 1e-3f);
  }
}

// ============================================================
// Engage speed (pull-out)
// ============================================================

TEST(VelocitySmootherTest, EngageSpeedClampsMinVelocity)
{
  auto params = make_default_params();
  params.set_engage_speed = true;
  params.target_pull_out_speed_mps = 3.0;
  params.target_pull_out_acc_mps2 = 1.0;
  params.stop_dist_to_prohibit_engage = 2.0;

  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  // Trajectory with low velocities, no zero-velocity stop point
  auto traj = make_straight_trajectory(20, 1.0, 1.0f);
  auto odom = make_odometry(0.0, 0.0, 0.5);  // ego is slow -> engage condition
  smoother.optimize(traj, odom, 0.0);

  // After engage clamp, velocities should be >= target_pull_out_speed_mps
  for (const auto & pt : traj) {
    EXPECT_GE(
      pt.longitudinal_velocity_mps, static_cast<float>(params.target_pull_out_speed_mps) - 1e-3f);
  }
}

TEST(VelocitySmootherTest, EngageSpeedProhibitedNearStopPoint)
{
  auto params = make_default_params();
  params.set_engage_speed = true;
  params.target_pull_out_speed_mps = 3.0;
  params.target_pull_out_acc_mps2 = 1.0;
  params.stop_dist_to_prohibit_engage = 10.0;  // large threshold

  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  // Trajectory with a stop point at index 5 (distance = 5m < 10m threshold)
  auto traj = make_straight_trajectory(10, 1.0, 1.0f);
  traj[5].longitudinal_velocity_mps = 0.0f;

  auto odom = make_odometry(0.0, 0.0, 0.5);  // ego is slow
  smoother.optimize(traj, odom, 0.0);

  // Engage should NOT be applied, so low-velocity points should remain low
  EXPECT_LT(
    traj[0].longitudinal_velocity_mps, static_cast<float>(params.target_pull_out_speed_mps));
}

// ============================================================
// Empty and minimal trajectory
// ============================================================

TEST(VelocitySmootherTest, EmptyTrajectoryDoesNotCrash)
{
  auto params = make_default_params();
  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  TrajectoryPoints traj;
  auto odom = make_odometry(0.0, 0.0, 0.0);
  smoother.optimize(traj, odom, 0.0);
  EXPECT_TRUE(traj.empty());
}

TEST(VelocitySmootherTest, SinglePointTrajectoryDoesNotCrash)
{
  auto params = make_default_params();
  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  TrajectoryPoints traj;
  traj.push_back(make_traj_point(0.0, 0.0, 10.0f));
  auto odom = make_odometry(0.0, 0.0, 0.0);
  smoother.optimize(traj, odom, 0.0);
  EXPECT_EQ(traj.size(), 1u);
}

// ============================================================
// set_max_velocity: partial over-speed segment
// ============================================================

TEST(VelocitySmootherTest, LimitSpeedPartialSegment)
{
  auto params = make_default_params();
  params.limit_speed = true;
  params.max_speed_mps = 10.0;

  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  // Points: [5, 5, 15, 15, 15, 5, 5] m/s
  TrajectoryPoints traj;
  traj.push_back(make_traj_point(0.0, 0.0, 5.0f, 1.0f));
  traj.push_back(make_traj_point(1.0, 0.0, 5.0f, 1.0f));
  traj.push_back(make_traj_point(2.0, 0.0, 15.0f, 1.0f));
  traj.push_back(make_traj_point(3.0, 0.0, 15.0f, 1.0f));
  traj.push_back(make_traj_point(4.0, 0.0, 15.0f, 1.0f));
  traj.push_back(make_traj_point(5.0, 0.0, 5.0f, -1.0f));
  traj.push_back(make_traj_point(6.0, 0.0, 5.0f, 0.0f));

  auto odom = make_odometry(0.0, 0.0, 5.0);
  smoother.optimize(traj, odom, 0.0);

  // All points should be <= max_speed_mps
  for (const auto & pt : traj) {
    EXPECT_LE(pt.longitudinal_velocity_mps, 10.0f + 1e-3f);
  }

  // Points outside the over-speed segment should remain unchanged
  EXPECT_NEAR(traj[0].longitudinal_velocity_mps, 5.0f, 1e-3f);
  EXPECT_NEAR(traj[1].longitudinal_velocity_mps, 5.0f, 1e-3f);
  EXPECT_NEAR(traj[5].longitudinal_velocity_mps, 5.0f, 1e-3f);
}

// ============================================================
// Pull-out initial motion
// ============================================================

TEST(VelocitySmootherTest, PullOutUsesTargetSpeedWhenSlow)
{
  auto params = make_default_params();
  // Enable limit_speed so we can observe clamped velocities
  params.limit_speed = true;
  params.max_speed_mps = 20.0;
  params.set_engage_speed = true;
  params.target_pull_out_speed_mps = 3.0;
  params.target_pull_out_acc_mps2 = 1.5;
  params.stop_dist_to_prohibit_engage = 1.0;

  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  // Trajectory with moderate speed, no stop point nearby
  auto traj = make_straight_trajectory(20, 1.0, 5.0f);
  // Ego is slow (below pull-out speed)
  auto odom = make_odometry(0.0, 0.0, 1.0);
  smoother.optimize(traj, odom, 0.0);

  // Since ego speed < target_pull_out_speed, engage speed clamp should raise velocities
  for (const auto & pt : traj) {
    EXPECT_GE(
      pt.longitudinal_velocity_mps, static_cast<float>(params.target_pull_out_speed_mps) - 1e-3f);
  }
}

// ============================================================
// All features disabled: trajectory passes through unchanged
// ============================================================

TEST(VelocitySmootherTest, AllFeaturesDisabledPassthrough)
{
  auto params = make_default_params();
  // All features disabled by default in make_default_params

  auto logger = rclcpp::get_logger("test_velocity_smoother");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  VelocitySmoother smoother(params, logger, clock, vehicle_info, nullptr);

  auto traj = make_straight_trajectory(10, 1.0, 7.5f, 0.5f);
  const auto traj_original = traj;
  auto odom = make_odometry(0.0, 0.0, 7.5);
  smoother.optimize(traj, odom, 0.0);

  // Trajectory should be unchanged when all features are disabled
  ASSERT_EQ(traj.size(), traj_original.size());
  for (size_t i = 0; i < traj.size(); ++i) {
    EXPECT_NEAR(
      traj[i].longitudinal_velocity_mps, traj_original[i].longitudinal_velocity_mps, 1e-6f);
    EXPECT_NEAR(traj[i].acceleration_mps2, traj_original[i].acceleration_mps2, 1e-6f);
  }
}

}  // namespace autoware::minimum_rule_based_planner
