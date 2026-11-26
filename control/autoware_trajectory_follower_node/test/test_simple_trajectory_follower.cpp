//  Copyright 2026 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

// Unit tests for the ROS-independent control logic
// simple_trajectory_follower::SimpleTrajectoryFollower.
//
// These tests exercise the extracted core directly: no rclcpp::init(), no node, no publishers or
// subscribers. compute_control_command() is a pure function of (trajectory, odometry, stamp), so
// each case is a plain Arrange / Act / Assert with hand-derived expected values. The matching
// node-level wiring is covered separately by the minimal integration test in
// test_simple_trajectory_follower_node.cpp.
//
// The control law under test (the single source of these hand-derived numbers) is:
//   lookahead   = min_lookahead(3.0) + lookahead_time(3.0) * |ego_vx|
//   kp_steer    = 2 * wheel_base(4.0) / lookahead^2
//   kd_steer    = 2 * wheel_base(4.0) / lookahead
//   lat_err     = lateral_deviation(closest_pose, ego_pos) - lateral_deviation_param
//   yaw_err     = yaw(ego) - yaw(closest_pose)
//   steer       = clamp(-kp_steer * lat_err - kd_steer * yaw_err, -0.6, 0.6)
//   target_vel  = use_external_target_vel ? external_target_vel : closest.longitudinal_velocity_mps
//   velocity    = target_vel
//   acc         = clamp(-kp_acc(0.5) * (ego_vx - target_vel), -2.0, 2.0)
// where lateral_deviation() is autoware_utils' cross-product deviation of the ego position from the
// closest trajectory point pose, so for a trajectory point whose heading (yaw) is 0 it reduces to
// (ego.y - point.y).

#include "autoware/trajectory_follower_node/simple_trajectory_follower.hpp"

#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using simple_trajectory_follower::Odometry;
using simple_trajectory_follower::SimpleTrajectoryFollower;
using simple_trajectory_follower::SimpleTrajectoryFollowerConfig;
using simple_trajectory_follower::SimpleTrajectoryFollowerResult;
using simple_trajectory_follower::Trajectory;

constexpr float tolerance = 1e-5f;

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

// The three fields all default to the "off / neutral" values the node would read from an all-zero
// parameter set; each test overrides only the field whose effect it checks.
SimpleTrajectoryFollowerConfig make_default_config()
{
  SimpleTrajectoryFollowerConfig config;
  config.use_external_target_vel = false;
  config.external_target_vel = 0.0;
  config.lateral_deviation = 0.0;
  return config;
}

TrajectoryPoint make_trajectory_point(
  const double x, const double y, const double yaw, const double velocity_mps)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = yaw_to_quaternion(yaw);
  point.longitudinal_velocity_mps = static_cast<float>(velocity_mps);
  return point;
}

Odometry make_odometry(const double x, const double y, const double yaw, const double velocity_mps)
{
  Odometry odometry;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation = yaw_to_quaternion(yaw);
  odometry.twist.twist.linear.x = velocity_mps;
  return odometry;
}

// A straight trajectory along the x-axis whose points all share one heading and target speed. Three
// points are the minimum that still exercises nearest-point selection (the point at x = 0 is
// closest to an ego near the origin).
Trajectory make_straight_trajectory(const double yaw, const double velocity_mps)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points = {
    make_trajectory_point(-1.0, 0.0, yaw, velocity_mps),
    make_trajectory_point(0.0, 0.0, yaw, velocity_mps),
    make_trajectory_point(1.0, 0.0, yaw, velocity_mps),
  };
  return trajectory;
}

// Timestamp handed to the core; its concrete value is irrelevant except in the stamp test.
rclcpp::Time make_stamp()
{
  return rclcpp::Time{0, 0, RCL_ROS_TIME};
}
}  // namespace

TEST(SimpleTrajectoryFollowerTest, ConstructorDoesNotThrow)
{
  // Arrange
  const auto config = make_default_config();

  // Act + Assert
  EXPECT_NO_THROW({ SimpleTrajectoryFollower follower(config); });
}

TEST(SimpleTrajectoryFollowerTest, ReturnsErrorWhenTrajectoryIsEmpty)
{
  // Arrange: an empty trajectory has no nearest point to track, so the core cannot produce a
  // command and must report that by returning an error (tl::unexpected) instead of throwing, which
  // previously crashed the node from inside its timer callback.
  const SimpleTrajectoryFollower follower(make_default_config());
  const Trajectory empty_trajectory;
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 1.0);

  // Act
  const auto result = follower.compute_control_command(empty_trajectory, odometry, make_stamp());

  // Assert
  EXPECT_FALSE(result.has_value());
}

TEST(SimpleTrajectoryFollowerTest, OutputsNeutralCommandWhenPerfectlyTrackingTrajectory)
{
  // Arrange: ego is exactly on a straight trajectory (heading 0) AND already driving at the
  // trajectory's target speed (1.0 m/s). Nominal case: nothing to correct.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 1.0);
  // Hand-derived (do not recompute with the production formula):
  //   lat_err = 0, yaw_err = 0 -> steer = 0.
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx 1.0 - target 1.0) = 0.0 (ego already at the target speed).
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, SelectsNearestTrajectoryPointForControl)
{
  // Arrange: three points with distinct target speeds. Ego is positioned next to the LAST point, so
  // the output speed reveals which point was selected (a wrong index would publish 1.0 or 5.0, not
  // the last point's 9.0). Ego already drives at that point's speed to keep acceleration at zero.
  const SimpleTrajectoryFollower follower(make_default_config());
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points = {
    make_trajectory_point(-10.0, 0.0, 0.0, 1.0),
    make_trajectory_point(0.0, 0.0, 0.0, 5.0),
    make_trajectory_point(10.0, 0.0, 0.0, 9.0),  // closest point to ego at (9.8, 0)
  };
  const auto odometry = make_odometry(9.8, 0.0, 0.0, 9.0);
  // Hand-derived (do not recompute with the production formula):
  //   nearest point is (10, 0) -> velocity tracks its speed 9.0.
  //   ego is on the x-axis with heading 0 -> lat_err = yaw_err = 0 -> steer = 0.
  //   acc = -0.5 * (ego_vx 9.0 - target 9.0) = 0.0.
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 9.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, SteersNegativeToCorrectLeftwardLateralOffset)
{
  // Arrange: ego is 1.0 m to the left (+y) of a straight x-axis trajectory, driving at 1.0 m/s,
  // matching the trajectory's target speed so no acceleration is commanded (isolates steering).
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 1.0, 0.0, 1.0);
  // Hand-derived (do not recompute with the production formula):
  //   lat_err = (ego.y 1.0 - point.y 0.0) - lateral_deviation_param 0.0 = 1.0, yaw_err = 0.
  //   lookahead = 3.0 + 3.0 * |1.0| = 6.0 ; kp = 2 * 4.0 / 6.0^2 = 8/36.
  //   steer = -(8/36) * 1.0 = -0.2222... (negative = steer right to reduce the leftward offset).
  constexpr float expected_steer = -8.0f / 36.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->command.lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, LateralDeviationParameterShiftsSteeringError)
{
  // Arrange: ego is exactly on the trajectory, but the lateral_deviation parameter biases the error
  // by -1.0, so the core steers as if it had a +1.0 lateral error (mirror of the test above).
  auto config = make_default_config();
  config.lateral_deviation = 1.0;
  const SimpleTrajectoryFollower follower(config);
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 1.0);
  // Hand-derived (do not recompute with the production formula):
  //   lat_err = (ego.y 0.0 - point.y 0.0) - lateral_deviation_param 1.0 = -1.0, yaw_err = 0.
  //   lookahead = 3.0 + 3.0 * |1.0| = 6.0 ; kp = 8/36.
  //   steer = -(8/36) * (-1.0) = +0.2222...
  constexpr float expected_steer = 8.0f / 36.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->command.lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, SteersFromYawDeviationOfClosestPoint)
{
  // Arrange: ego is on the trajectory position-wise (zero lateral error) but the closest trajectory
  // point has a heading of 0.1 rad, isolating the yaw (kd) term of the steering law.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.1, 1.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 1.0);
  // Hand-derived (do not recompute with the production formula):
  //   lat_err = 0 (ego on point, lateral_deviation_param 0).
  //   yaw_err = yaw(ego) 0.0 - yaw(point) 0.1 = -0.1.
  //   lookahead = 3.0 + 3.0 * |1.0| = 6.0 ; kd = 2 * 4.0 / 6.0 = 8/6.
  //   steer = -(8/6) * (-0.1) = +0.13333...
  constexpr float expected_steer = (8.0f / 6.0f) * 0.1f;
  constexpr double expected_yaw_error = -0.1;  // reported in debug; see assertion below
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->command.lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_NEAR(result->debug.yaw_error, expected_yaw_error, tolerance);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, CombinesLateralAndYawErrorsInSteering)
{
  // Arrange: drive BOTH steering terms at once. ego is 1.0 m to the left of a straight x-axis
  // trajectory (heading 0) AND its own heading is 0.1 rad, so lat_err and yaw_err are both non-zero
  // and the output must be their superposition (not just one term).
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 1.0, 0.1, 1.0);
  // Hand-derived (do not recompute with the production formula):
  //   lat_err = (ego.y 1.0 - point.y 0.0) - lateral_deviation_param 0.0 = 1.0.
  //   yaw_err = yaw(ego) 0.1 - yaw(point) 0.0 = +0.1.
  //   lookahead = 3.0 + 3.0 * |1.0| = 6.0 ; kp = 8/36 ; kd = 8/6.
  //   steer = -(8/36) * 1.0 - (8/6) * 0.1 = -0.22222 - 0.13333 = -0.35556.
  constexpr float expected_steer = -(8.0f / 36.0f) - (8.0f / 6.0f) * 0.1f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->command.lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, UsesAbsoluteEgoSpeedForLookahead)
{
  // Arrange: a NEGATIVE ego velocity (reversing). The lookahead uses |ego_vx|, so the steering gain
  // must match the +1.0 m/s leftward-offset case. If the absolute value were dropped, lookahead
  // would be 3.0 + 3.0 * (-1.0) = 0.0, blowing up kp and clamping the steer to the limit -0.6
  // instead.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 1.0, 0.0, -1.0);
  // Hand-derived (do not recompute with the production formula):
  //   lat_err = 1.0, yaw_err = 0.
  //   lookahead = 3.0 + 3.0 * |-1.0| = 6.0 ; kp = 8/36 -> steer = -(8/36) * 1.0 = -0.2222...
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx -1.0 - target 1.0) = -0.5 * (-2.0) = 1.0.
  constexpr float expected_steer = -8.0f / 36.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 1.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->command.lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, ClampsSteeringToLowerSteerLimit)
{
  // Arrange: a huge leftward lateral offset (+100 m) would command a steer far below the limit.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 0.0);
  const auto odometry = make_odometry(0.0, 100.0, 0.0, 0.0);  // ego_vx 0 -> lookahead 3.0, kp 8/9
  // Hand-derived (do not recompute with the production formula):
  //   raw steer = -(8/9) * 100.0 = -88.9, clamped to the lower steer_lim of -0.6.
  constexpr float expected_steer = -0.6f;
  constexpr float expected_velocity = 0.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, ClampsSteeringToUpperSteerLimit)
{
  // Arrange: a huge rightward lateral offset (-100 m) would command a steer far above the limit.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 0.0);
  const auto odometry = make_odometry(0.0, -100.0, 0.0, 0.0);  // ego_vx 0 -> lookahead 3.0, kp 8/9
  // Hand-derived (do not recompute with the production formula):
  //   raw steer = -(8/9) * (-100.0) = +88.9, clamped to the upper steer_lim of +0.6.
  constexpr float expected_steer = 0.6f;
  constexpr float expected_velocity = 0.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, UsesExternalTargetVelocityWhenEnabled)
{
  // Arrange: external target velocity enabled; its value (5.0) differs from the trajectory's own
  // speed (1.0) so we can tell which one the core uses.
  auto config = make_default_config();
  config.use_external_target_vel = true;
  config.external_target_vel = 5.0;
  const SimpleTrajectoryFollower follower(config);
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 2.0);
  // Hand-derived (do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity comes from external_target_vel (5.0), not the trajectory speed (1.0).
  //   acc = -0.5 * (ego_vx 2.0 - external_target 5.0) = -0.5 * (-3.0) = 1.5.
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 5.0f;
  constexpr float expected_acceleration = 1.5f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, AcceleratesWhenEgoBelowTargetVelocity)
{
  // Arrange: ego (2.0 m/s) is slower than the trajectory target (3.0 m/s).
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 3.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 2.0);
  // Hand-derived (do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity tracks the closest point's speed (3.0).
  //   acc = -0.5 * (ego_vx 2.0 - target 3.0) = -0.5 * (-1.0) = 0.5 (accelerating).
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 3.0f;
  constexpr float expected_acceleration = 0.5f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, BrakesWhenEgoExceedsTargetVelocity)
{
  // Arrange: ego (3.0 m/s) is faster than the trajectory target (1.0 m/s).
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 3.0);
  // Hand-derived (do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx 3.0 - target 1.0) = -0.5 * 2.0 = -1.0 (braking).
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = -1.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, ClampsAccelerationToLowerAccLimit)
{
  // Arrange: a large positive velocity error (ego 10.0 m/s, target 0.0) would command a
  // deceleration far below the acceleration limit.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 0.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 10.0);
  // Hand-derived (do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity tracks the closest point's speed (0.0).
  //   raw acc = -0.5 * (ego_vx 10.0 - target 0.0) = -5.0, clamped to the lower acc_lim of -2.0.
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 0.0f;
  constexpr float expected_acceleration = -2.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, ClampsAccelerationToUpperAccLimit)
{
  // Arrange: a large negative velocity error (ego 0.0 m/s, target 10.0) would command an
  // acceleration far above the acceleration limit.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 10.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 0.0);
  // Hand-derived (do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity tracks the closest point's speed (10.0).
  //   raw acc = -0.5 * (ego_vx 0.0 - target 10.0) = +5.0, clamped to the upper acc_lim of +2.0.
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 10.0f;
  constexpr float expected_acceleration = 2.0f;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->command.lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(result->command.longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(result->command.longitudinal.acceleration, expected_acceleration);
}

TEST(SimpleTrajectoryFollowerTest, StampsAllCommandHeadersWithGivenTime)
{
  // Arrange: a distinctive stamp so we can tell it is the one that reaches every header. Setting
  // the output stamp is the core's responsibility, so the node only needs to pass
  // get_clock()->now().
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 0.0, 0.0, 1.0);
  const rclcpp::Time stamp{123, 456789, RCL_ROS_TIME};
  constexpr int32_t expected_sec = 123;
  constexpr uint32_t expected_nanosec = 456789;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, stamp);

  // Assert: the top-level command and both sub-commands carry the given stamp.
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->command.stamp.sec, expected_sec);
  EXPECT_EQ(result->command.stamp.nanosec, expected_nanosec);
  EXPECT_EQ(result->command.lateral.stamp.sec, expected_sec);
  EXPECT_EQ(result->command.lateral.stamp.nanosec, expected_nanosec);
  EXPECT_EQ(result->command.longitudinal.stamp.sec, expected_sec);
  EXPECT_EQ(result->command.longitudinal.stamp.nanosec, expected_nanosec);
}

TEST(SimpleTrajectoryFollowerTest, PopulatesDebugIntermediatesForLogging)
{
  // Arrange: a scenario with non-trivial, independently hand-derivable intermediates. The debug
  // struct is the core's reporting contract for the values the node logs, so it is verified
  // directly. ego is 2.0 m to the left of a straight x-axis trajectory and faster than its target
  // speed.
  const SimpleTrajectoryFollower follower(make_default_config());
  const auto trajectory = make_straight_trajectory(0.0, 1.0);
  const auto odometry = make_odometry(0.0, 2.0, 0.0, 3.0);
  // Hand-derived (do not recompute with the production formula):
  //   lateral_error = (ego.y 2.0 - point.y 0.0) - lateral_deviation_param 0.0 = 2.0.
  //   yaw_error = yaw(ego) 0.0 - yaw(point) 0.0 = 0.0.
  //   velocity_error = ego_vx 3.0 - target 1.0 = 2.0.
  //   lookahead = 3.0 + 3.0 * |3.0| = 12.0 ; kp = 2 * 4.0 / 12.0^2 = 8/144 ; kd = 2 * 4.0 / 12.0 =
  //   8/12.
  constexpr double expected_lateral_error = 2.0;
  constexpr double expected_yaw_error = 0.0;
  constexpr double expected_velocity_error = 2.0;
  constexpr double expected_steer_kp = 8.0 / 144.0;
  constexpr double expected_steer_kd = 8.0 / 12.0;

  // Act
  const auto result = follower.compute_control_command(trajectory, odometry, make_stamp());

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->debug.lateral_error, expected_lateral_error, tolerance);
  EXPECT_NEAR(result->debug.yaw_error, expected_yaw_error, tolerance);
  EXPECT_NEAR(result->debug.velocity_error, expected_velocity_error, tolerance);
  EXPECT_NEAR(result->debug.steer_kp, expected_steer_kp, tolerance);
  EXPECT_NEAR(result->debug.steer_kd, expected_steer_kd, tolerance);
}
