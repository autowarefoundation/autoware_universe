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

// Characterization test for simple_trajectory_follower::SimpleTrajectoryFollower.
//
// The purpose of this test is NOT to assert the "correct" behavior, but to freeze the CURRENT
// behavior of the node before refactoring so that regressions become visible. Production code
// (simple_trajectory_follower.{hpp,cpp}) is intentionally left untouched.
//
// The node is driven through its real ROS interface:
//   subscribes : ~/input/kinematics  (nav_msgs/Odometry)
//                ~/input/trajectory  (autoware_planning_msgs/Trajectory)
//   publishes  : ~/output/control_cmd (autoware_control_msgs/Control), from a 30 ms timer
//
// The control law captured here (read from the source at the time of writing) is:
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
//
// Notes:
//   - An EMPTY trajectory (points.size() == 0) makes the timer callback throw; this is a latent
//     node-death defect that IS frozen here (see ThrowsWhenTrajectoryHasNoPoints), separate from
//     the "no trajectory message at all" case (DoesNotPublishControlWhenTrajectoryMissing).
//   - The output command stamp is set from get_clock()->now(); it is non-deterministic and so is
//     not asserted.

#include "autoware/trajectory_follower_node/simple_trajectory_follower.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace
{
using autoware_control_msgs::msg::Control;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using simple_trajectory_follower::SimpleTrajectoryFollower;

// The node name is hard-coded in the constructor, so the topics resolve to these fully-qualified
// names.
constexpr char kKinematicsTopic[] = "/simple_trajectory_follower/input/kinematics";
constexpr char kTrajectoryTopic[] = "/simple_trajectory_follower/input/trajectory";
constexpr char kControlCmdTopic[] = "/simple_trajectory_follower/output/control_cmd";

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

// All three parameters are declared without a default in the node, so every test must provide them.
struct FollowerParams
{
  bool use_external_target_vel{false};
  double external_target_vel{0.0};
  double lateral_deviation{0.0};
};
}  // namespace

class SimpleTrajectoryFollowerTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override
  {
    executor_.reset();
    test_node_.reset();
    follower_node_.reset();
    rclcpp::shutdown();
  }

  // --- builders -------------------------------------------------------------------------------
  static TrajectoryPoint make_trajectory_point(
    const double x, const double y, const double yaw, const double velocity_mps)
  {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.orientation = yaw_to_quaternion(yaw);
    point.longitudinal_velocity_mps = static_cast<float>(velocity_mps);
    return point;
  }

  // --- high-level helpers (used directly by the test cases) -----------------------------------
  void start_follower(const FollowerParams & params)
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_external_target_vel", params.use_external_target_vel);
    options.append_parameter_override("external_target_vel", params.external_target_vel);
    options.append_parameter_override("lateral_deviation", params.lateral_deviation);
    follower_node_ = std::make_shared<SimpleTrajectoryFollower>(options);

    test_node_ = std::make_shared<rclcpp::Node>("simple_trajectory_follower_test_node");
    odom_pub_ = test_node_->create_publisher<Odometry>(kKinematicsTopic, rclcpp::QoS{1});
    trajectory_pub_ = test_node_->create_publisher<Trajectory>(kTrajectoryTopic, rclcpp::QoS{1});
    control_cmd_sub_ = test_node_->create_subscription<Control>(
      kControlCmdTopic, rclcpp::QoS{1},
      [this](const Control::SharedPtr msg) { latest_control_cmd_ = msg; });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(follower_node_);
    executor_->add_node(test_node_);
  }

  void set_ego_odometry(const double x, const double y, const double yaw, const double velocity_mps)
  {
    Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = yaw_to_quaternion(yaw);
    odom.twist.twist.linear.x = velocity_mps;
    ego_odometry_ = odom;
  }

  void set_trajectory(const std::vector<TrajectoryPoint> & points)
  {
    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    trajectory.points = points;
    trajectory_ = trajectory;
  }

  // Repeatedly publishes the currently-set inputs while spinning, and returns the first control
  // command produced by the node (nullptr on timeout). Inputs are latched by the node's polling
  // subscribers, so every 30 ms timer tick uses the same inputs and produces the same command;
  // re-publishing each iteration just avoids a pub/sub discovery race (QoS is volatile).
  // Exceptions thrown inside the node's timer callback are not caught here: spin_some() re-throws
  // them, so they propagate to the caller (used by ThrowsWhenTrajectoryHasNoPoints).
  Control::SharedPtr spin_until_control_command(
    const std::chrono::milliseconds timeout = std::chrono::milliseconds{3000})
  {
    latest_control_cmd_ = nullptr;
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (ego_odometry_) odom_pub_->publish(*ego_odometry_);
      if (trajectory_) trajectory_pub_->publish(*trajectory_);
      executor_->spin_some();
      if (latest_control_cmd_) return latest_control_cmd_;
      if (std::chrono::steady_clock::now() - start > timeout) return nullptr;
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return nullptr;
  }

  std::shared_ptr<SimpleTrajectoryFollower> follower_node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<Control>::SharedPtr control_cmd_sub_;

  std::optional<Odometry> ego_odometry_;
  std::optional<Trajectory> trajectory_;
  Control::SharedPtr latest_control_cmd_;
};

TEST_F(SimpleTrajectoryFollowerTest, DoesNotPublishControlWhenTrajectoryMissing)
{
  // Arrange: only odometry is available, the trajectory is never published.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);

  // Act: give the 30 ms timer plenty of chances to fire.
  const auto cmd = spin_until_control_command(std::chrono::milliseconds{1000});

  // Assert: processData() reports "not ready" and the node publishes nothing.
  EXPECT_EQ(cmd, nullptr);
}

// NOTE(characterization): This freezes a LATENT DEFECT. With a non-empty odometry but an empty
// trajectory (points.size() == 0), onTimer() -> updateClosest() -> motion_utils::findNearestIndex()
// throws (validateNonEmpty(), currently std::invalid_argument) from *inside the timer callback*.
// In production this exception escapes the node's own executor spin and kills the node process.
// The test can observe it safely because our own spin_some() re-throws the callback's exception to
// us. A later PR should make the node handle an empty trajectory gracefully (e.g. skip publishing,
// like the "data not ready" path); when it does, THIS test must be updated to the new no-throw
// behavior.
TEST_F(SimpleTrajectoryFollowerTest, ThrowsWhenTrajectoryHasNoPoints)
{
  // Arrange: odometry is ready, but the trajectory carries zero points.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);
  set_trajectory(std::vector<TrajectoryPoint>{});

  // Act + Assert: driving the node makes the timer callback throw. spin_until_control_command()
  // does not catch it, so the exception propagates straight out of the helper. Assert the base
  // std::exception rather than the concrete type so the test is not coupled to which utility
  // (currently std::invalid_argument from validateNonEmpty) raised it.
  EXPECT_THROW(spin_until_control_command(), std::exception);
}

TEST_F(SimpleTrajectoryFollowerTest, OutputsZeroSteerAndZeroAccelWhenPerfectlyTrackingTrajectory)
{
  // Arrange: ego is exactly on a straight trajectory (points on the x-axis, heading 0) AND already
  // driving at the trajectory's target speed (1.0 m/s). Nominal case: nothing to correct.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(0.0, 0.0, 0.0, 1.0),  // closest point to ego at (0, 0)
    make_trajectory_point(1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(2.0, 0.0, 0.0, 1.0),
  });
  // Expected outputs (hand-derived; do not recompute with the production formula):
  //   lat_err = 0, yaw_err = 0 -> steer = 0.
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx 1.0 - target 1.0) = 0.0 (ego already at the target speed).
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_FLOAT_EQ(cmd->lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, FollowsStraightTrajectoryWithZeroSteerAndTracksTargetSpeed)
{
  // Arrange: ego exactly on a straight trajectory (points on the x-axis, heading 0), driving at
  // 2.0 m/s toward a trajectory whose target speed is 3.0 m/s.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 2.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 3.0),
    make_trajectory_point(0.0, 0.0, 0.0, 3.0),  // closest point to ego at (0, 0)
    make_trajectory_point(1.0, 0.0, 0.0, 3.0),
    make_trajectory_point(2.0, 0.0, 0.0, 3.0),
  });
  // Expected outputs (hand-derived; do not recompute with the production formula):
  //   lat_err = 0, yaw_err = 0 -> steer = 0.
  //   use_external_target_vel = false -> velocity tracks the closest point's speed (3.0).
  //   acc = -0.5 * (ego_vx 2.0 - target 3.0) = -0.5 * (-1.0) = 0.5.
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 3.0f;
  constexpr float expected_acceleration = 0.5f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_FLOAT_EQ(cmd->lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, SteersNegativeToCorrectLeftwardLateralOffset)
{
  // Arrange: ego is 1.0 m to the left (+y) of a straight x-axis trajectory, driving at 1.0 m/s,
  // matching the trajectory's target speed so no acceleration is commanded (isolates steering).
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 1.0, 0.0, 1.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(0.0, 0.0, 0.0, 1.0),  // closest point to ego at (0, 1)
    make_trajectory_point(1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(2.0, 0.0, 0.0, 1.0),
  });
  // Expected output (hand-derived; do not recompute with the production formula):
  //   lat_err = (ego.y 1.0 - point.y 0.0) - lateral_deviation_param 0.0 = 1.0, yaw_err = 0.
  //   lookahead = 3.0 + 3.0 * |1.0| = 6.0 ; kp = 2 * 4.0 / 6.0^2 = 8/36.
  //   steer = -(8/36) * 1.0 = -0.2222... (negative = steer right to reduce the leftward offset).
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx 1.0 - target 1.0) = 0.0 (ego already at the target speed).
  constexpr float expected_steer = -8.0f / 36.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;
  constexpr float tolerance = 1e-5f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_NEAR(cmd->lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, LateralDeviationParameterShiftsSteeringError)
{
  // Arrange: ego is exactly on the trajectory, but the lateral_deviation parameter biases the
  // error by -1.0, so the node steers as if it had a +1.0 lateral error (opposite the test above).
  FollowerParams params;
  params.lateral_deviation = 1.0;
  start_follower(params);
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(0.0, 0.0, 0.0, 1.0),  // closest point to ego at (0, 0)
    make_trajectory_point(1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(2.0, 0.0, 0.0, 1.0),
  });
  // ego drives at 1.0 m/s, matching the trajectory's target speed so no acceleration is commanded.
  // Expected output (hand-derived; do not recompute with the production formula):
  //   lat_err = (ego.y 0.0 - point.y 0.0) - lateral_deviation_param 1.0 = -1.0, yaw_err = 0.
  //   lookahead = 3.0 + 3.0 * |1.0| = 6.0 ; kp = 8/36.
  //   steer = -(8/36) * (-1.0) = +0.2222...
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx 1.0 - target 1.0) = 0.0 (ego already at the target speed).
  constexpr float expected_steer = 8.0f / 36.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;
  constexpr float tolerance = 1e-5f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_NEAR(cmd->lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, SteersFromYawDeviationOfClosestPoint)
{
  // Arrange: ego is on the trajectory position-wise (zero lateral error) but the closest trajectory
  // point has a heading of 0.1 rad, isolating the yaw (kd) term of the steering law. (The points
  // lie on the x-axis; only the closest point's stored heading matters to the control law.)
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.1, 1.0),
    make_trajectory_point(0.0, 0.0, 0.1, 1.0),  // closest point to ego at (0, 0)
    make_trajectory_point(1.0, 0.0, 0.1, 1.0),
  });
  // ego drives at 1.0 m/s, matching the trajectory's target speed so no acceleration is commanded.
  // Expected output (hand-derived; do not recompute with the production formula):
  //   lat_err = 0 (ego on point, lateral_deviation_param 0).
  //   yaw_err = yaw(ego) 0.0 - yaw(point) 0.1 = -0.1.
  //   lookahead = 3.0 + 3.0 * |1.0| = 6.0 ; kd = 2 * 4.0 / 6.0 = 8/6.
  //   steer = -(8/6) * (-0.1) = +0.13333...
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx 1.0 - target 1.0) = 0.0 (ego already at the target speed).
  constexpr float expected_steer = (8.0f / 6.0f) * 0.1f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;
  constexpr float tolerance = 1e-5f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_NEAR(cmd->lateral.steering_tire_angle, expected_steer, tolerance);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, ClampsSteeringToSteerLimit)
{
  // Arrange: a huge lateral offset (100 m) would command a steer far beyond the limit.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 100.0, 0.0, 0.0);  // ego_vx 0 -> lookahead = 3.0, kp = 8/9
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 0.0),
    make_trajectory_point(0.0, 0.0, 0.0, 0.0),  // closest point to ego at (0, 100)
    make_trajectory_point(1.0, 0.0, 0.0, 0.0),
    make_trajectory_point(2.0, 0.0, 0.0, 0.0),
  });
  // Expected outputs (hand-derived; do not recompute with the production formula):
  //   raw steer = -(8/9) * 100.0 = -88.9, clamped to the lower steer_lim of -0.6.
  //   velocity tracks the closest point's speed (0.0).
  //   acc = -0.5 * (ego_vx 0.0 - target 0.0) = 0.0.
  constexpr float expected_steer = -0.6f;
  constexpr float expected_velocity = 0.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_FLOAT_EQ(cmd->lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, UsesExternalTargetVelocityWhenEnabled)
{
  // Arrange: external target velocity enabled; its value (5.0) differs from the trajectory's own
  // speed (1.0) so we can tell which one the node uses.
  FollowerParams params;
  params.use_external_target_vel = true;
  params.external_target_vel = 5.0;
  start_follower(params);
  set_ego_odometry(0.0, 0.0, 0.0, 2.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(0.0, 0.0, 0.0, 1.0),  // closest point's speed (1.0) must be ignored
    make_trajectory_point(1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(2.0, 0.0, 0.0, 1.0),
  });
  // Expected outputs (hand-derived; do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity comes from external_target_vel (5.0), not the trajectory speed (1.0).
  //   acc = -0.5 * (ego_vx 2.0 - external_target 5.0) = -0.5 * (-3.0) = 1.5.
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 5.0f;
  constexpr float expected_acceleration = 1.5f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_FLOAT_EQ(cmd->lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, BrakesWhenEgoExceedsTargetVelocity)
{
  // Arrange: ego (3.0 m/s) is faster than the trajectory target (1.0 m/s).
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 3.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(0.0, 0.0, 0.0, 1.0),  // closest point to ego at (0, 0)
    make_trajectory_point(1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(2.0, 0.0, 0.0, 1.0),
  });
  // Expected outputs (hand-derived; do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity tracks the closest point's speed (1.0).
  //   acc = -0.5 * (ego_vx 3.0 - target 1.0) = -0.5 * 2.0 = -1.0 (braking).
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = -1.0f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_FLOAT_EQ(cmd->lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

TEST_F(SimpleTrajectoryFollowerTest, ClampsAccelerationToAccLimit)
{
  // Arrange: a large velocity error (ego 10.0 m/s, target 0.0) would command a deceleration far
  // beyond the acceleration limit.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 10.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 0.0),
    make_trajectory_point(0.0, 0.0, 0.0, 0.0),  // closest point to ego at (0, 0)
    make_trajectory_point(1.0, 0.0, 0.0, 0.0),
    make_trajectory_point(2.0, 0.0, 0.0, 0.0),
  });
  // Expected outputs (hand-derived; do not recompute with the production formula):
  //   steer = 0 (ego on the trajectory, lat_err = yaw_err = 0).
  //   velocity tracks the closest point's speed (0.0).
  //   raw acc = -0.5 * (ego_vx 10.0 - target 0.0) = -5.0, clamped to the lower acc_lim of -2.0.
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 0.0f;
  constexpr float expected_acceleration = -2.0f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert
  ASSERT_NE(cmd, nullptr);
  EXPECT_FLOAT_EQ(cmd->lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}
