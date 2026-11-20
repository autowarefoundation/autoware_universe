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

// Integration (wiring) test for simple_trajectory_follower::SimpleTrajectoryFollowerNode.
//
// The control-law behavior is now covered exhaustively by the ROS-independent unit tests in
// test_simple_trajectory_follower.cpp. What those unit tests CANNOT check is the ROS wiring: that
// the node's polling subscribers latch the inputs, that the 30 ms timer fires, that the result is
// published on the output topic, that the "data not ready" guard suppresses publishing, and that an
// empty trajectory is skipped gracefully (the core returns an error) instead of crashing the node.
// This file keeps just the minimal cases that exercise that wiring end to end.
//
// The node is driven through its real ROS interface:
//   subscribes : ~/input/kinematics  (nav_msgs/Odometry)
//                ~/input/trajectory  (autoware_planning_msgs/Trajectory)
//   publishes  : ~/output/control_cmd (autoware_control_msgs/Control), from a 30 ms timer

#include "autoware/trajectory_follower_node/simple_trajectory_follower_node.hpp"

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
using simple_trajectory_follower::SimpleTrajectoryFollowerNode;

// The node name is hard-coded in the constructor, so the topics resolve to these fully-qualified
// names.
constexpr char kinematics_topic[] = "/simple_trajectory_follower/input/kinematics";
constexpr char trajectory_topic[] = "/simple_trajectory_follower/input/trajectory";
constexpr char control_cmd_topic[] = "/simple_trajectory_follower/output/control_cmd";

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

class SimpleTrajectoryFollowerNodeTest : public ::testing::Test
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
    follower_node_ = std::make_shared<SimpleTrajectoryFollowerNode>(options);

    test_node_ = std::make_shared<rclcpp::Node>("simple_trajectory_follower_test_node");
    odom_pub_ = test_node_->create_publisher<Odometry>(kinematics_topic, rclcpp::QoS{1});
    trajectory_pub_ = test_node_->create_publisher<Trajectory>(trajectory_topic, rclcpp::QoS{1});
    control_cmd_sub_ = test_node_->create_subscription<Control>(
      control_cmd_topic, rclcpp::QoS{1},
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

  std::shared_ptr<SimpleTrajectoryFollowerNode> follower_node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<Control>::SharedPtr control_cmd_sub_;

  std::optional<Odometry> ego_odometry_;
  std::optional<Trajectory> trajectory_;
  Control::SharedPtr latest_control_cmd_;
};

// Happy-path wiring: valid inputs flow subscriber -> timer -> core -> publisher. The published
// values (zero steer / neutral command for perfect tracking) only confirm that the inputs reached
// the core unchanged; the control law itself is verified in
// SimpleTrajectoryFollowerTest.OutputsNeutralCommandWhenPerfectlyTrackingTrajectory.
TEST_F(SimpleTrajectoryFollowerNodeTest, PublishesControlCommandFromSubscribedInputs)
{
  // Arrange: ego exactly on a straight trajectory, already at the trajectory's target speed.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);
  set_trajectory({
    make_trajectory_point(-1.0, 0.0, 0.0, 1.0),
    make_trajectory_point(0.0, 0.0, 0.0, 1.0),  // closest point to ego at (0, 0)
    make_trajectory_point(1.0, 0.0, 0.0, 1.0),
  });
  constexpr float expected_steer = 0.0f;
  constexpr float expected_velocity = 1.0f;
  constexpr float expected_acceleration = 0.0f;

  // Act
  const auto cmd = spin_until_control_command();

  // Assert: a command was published, and it carries the core's neutral output for these inputs.
  ASSERT_NE(cmd, nullptr);
  EXPECT_FLOAT_EQ(cmd->lateral.steering_tire_angle, expected_steer);
  EXPECT_FLOAT_EQ(cmd->longitudinal.velocity, expected_velocity);
  EXPECT_FLOAT_EQ(cmd->longitudinal.acceleration, expected_acceleration);
}

// Wiring of the empty-trajectory path. An empty trajectory (points.size() == 0) has no nearest
// point, so the core returns an error and the node must skip publishing (and must not crash). This
// is the node-level counterpart of
// SimpleTrajectoryFollowerTest.ReturnsErrorWhenTrajectoryIsEmpty: the unit test proves the core
// returns an error, this proves the timer callback handles it and keeps the node alive. (Previously
// the core threw from findNearestIndex() and the exception escaped the timer callback, killing the
// node process.)
TEST_F(SimpleTrajectoryFollowerNodeTest, DoesNotPublishControlWhenTrajectoryIsEmpty)
{
  // Arrange: odometry is ready, but the trajectory carries zero points.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);
  set_trajectory(std::vector<TrajectoryPoint>{});

  // Act: give the 30 ms timer plenty of chances to fire; the node should keep running and publish
  // nothing.
  const auto cmd = spin_until_control_command(std::chrono::milliseconds{1000});

  // Assert
  EXPECT_EQ(cmd, nullptr);
}

// Wiring of the "data not ready" guard: with only odometry available, process_data() reports not
// ready and the timer callback must publish nothing (and must not crash).
TEST_F(SimpleTrajectoryFollowerNodeTest, DoesNotPublishControlWhenTrajectoryMissing)
{
  // Arrange: only odometry is available, the trajectory is never published.
  start_follower(FollowerParams{});
  set_ego_odometry(0.0, 0.0, 0.0, 1.0);

  // Act: give the 30 ms timer plenty of chances to fire.
  const auto cmd = spin_until_control_command(std::chrono::milliseconds{1000});

  // Assert
  EXPECT_EQ(cmd, nullptr);
}
