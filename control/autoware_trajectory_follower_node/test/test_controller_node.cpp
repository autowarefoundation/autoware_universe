// Copyright 2021 The Autoware Foundation
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware/fake_test_node/fake_test_node.hpp"
#include "autoware/trajectory_follower_node/controller_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using Controller = autoware::motion::control::trajectory_follower_node::Controller;
using Control = autoware_control_msgs::msg::Control;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
using VehicleOdometry = nav_msgs::msg::Odometry;
using SteeringReport = autoware_vehicle_msgs::msg::SteeringReport;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using geometry_msgs::msg::AccelWithCovarianceStamped;

using FakeNodeFixture = autoware::fake_test_node::FakeTestNode;

const rclcpp::Duration one_second(1, 0);

rclcpp::NodeOptions makeNodeOptions(const bool enable_keep_stopped_until_steer_convergence = false)
{
  // Pass default parameter file to the node
  const auto share_dir =
    ament_index_cpp::get_package_share_directory("autoware_trajectory_follower_node");
  const auto longitudinal_share_dir =
    ament_index_cpp::get_package_share_directory("autoware_pid_longitudinal_controller");
  const auto lateral_share_dir =
    ament_index_cpp::get_package_share_directory("autoware_mpc_lateral_controller");
  const auto test_utils_share_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("lateral_controller_mode", "mpc");
  node_options.append_parameter_override("longitudinal_controller_mode", "pid");
  node_options.append_parameter_override("steer_offset_param_name", "steer_offset");
  node_options.append_parameter_override(
    "enable_keep_stopped_until_steer_convergence",
    enable_keep_stopped_until_steer_convergence);  // longitudinal
  node_options.arguments(
    {"--ros-args", "--params-file",
     lateral_share_dir + "/param/lateral_controller_defaults.param.yaml", "--params-file",
     lateral_share_dir + "/param/steer_offset.param.yaml", "--params-file",
     longitudinal_share_dir + "/config/autoware_pid_longitudinal_controller.param.yaml",
     "--params-file", test_utils_share_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file", test_utils_share_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", share_dir + "/param/trajectory_follower_node.param.yaml"});

  return node_options;
}

std::shared_ptr<Controller> makeNode(const rclcpp::NodeOptions & node_options)
{
  std::shared_ptr<Controller> node = std::make_shared<Controller>(node_options);

  // Enable all logging in the node
  auto ret =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    std::cout << "Failed to set logging severity to DEBUG\n";
  }
  return node;
}

class ControllerTester
{
public:
  explicit ControllerTester(FakeNodeFixture * _fnf, const rclcpp::NodeOptions & node_options)
  : fnf(_fnf), node(makeNode(node_options))
  {
  }

  FakeNodeFixture * fnf;
  std::shared_ptr<Controller> node;

  Control::SharedPtr cmd_msg;
  bool received_control_command = false;

  void publish_default_odom()
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_pub->publish(odom_msg);
  };

  void publish_odom_vx(const double vx)
  {
    VehicleOdometry odom_msg;
    odom_msg.header.stamp = node->now();
    odom_msg.twist.twist.linear.x = vx;
    odom_pub->publish(odom_msg);
  };

  void publish_default_steer()
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_pub->publish(steer_msg);
  };

  void publish_steer_angle(const double steer)
  {
    SteeringReport steer_msg;
    steer_msg.stamp = node->now();
    steer_msg.steering_tire_angle = steer;
    steer_pub->publish(steer_msg);
  };

  void publish_default_acc()
  {
    AccelWithCovarianceStamped acc_msg;
    acc_msg.header.stamp = node->now();
    accel_pub->publish(acc_msg);
  };

  void publish_autonomous_operation_mode()
  {
    OperationModeState msg;
    msg.stamp = node->now();
    msg.mode = OperationModeState::AUTONOMOUS;
    msg.is_autoware_control_enabled = true;
    operation_mode_pub->publish(msg);
  };

  /// Spins the controller and the fake node until a control command arrives or max_wait_time
  /// passes. Returns whether a control command arrived.
  bool wait_for_control_command(const std::chrono::milliseconds max_wait_time)
  {
    const std::chrono::milliseconds spin_interval{100};
    std::chrono::milliseconds waited_time{0};
    while (!received_control_command && waited_time <= max_wait_time) {
      rclcpp::spin_some(node);
      rclcpp::spin_some(fnf->get_fake_node());
      std::this_thread::sleep_for(spin_interval);
      waited_time += spin_interval;
    }
    return received_control_command;
  }

  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    fnf->create_publisher<Trajectory>("controller/input/reference_trajectory");

  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    fnf->create_publisher<VehicleOdometry>("controller/input/current_odometry");

  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    fnf->create_publisher<SteeringReport>("controller/input/current_steering");

  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr accel_pub =
    fnf->create_publisher<AccelWithCovarianceStamped>("controller/input/current_accel");

  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_pub =
    fnf->create_publisher<OperationModeState>(
      "controller/input/current_operation_mode", std::chrono::milliseconds{100},
      rclcpp::QoS(1).transient_local());

  rclcpp::Subscription<Control>::SharedPtr cmd_sub = fnf->create_subscription<Control>(
    "controller/output/control_cmd", *fnf->get_fake_node(), [this](const Control::SharedPtr msg) {
      cmd_msg = msg;
      received_control_command = true;
    });
};

TrajectoryPoint make_traj_point(const double px, const double py, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = px;
  p.pose.position.y = py;
  p.longitudinal_velocity_mps = vx;
  return p;
}

TEST_F(FakeNodeFixture, no_input)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  // No published data: expect a stopped command
  EXPECT_FALSE(tester.wait_for_control_command(std::chrono::seconds{1}));
}

// The control behaviors are tested in test_trajectory_follower.cpp. This test
// checks that the node reads the input topics and publishes the control command.
TEST_F(FakeNodeFixture, straight_trajectory)
{
  const auto node_options = makeNodeOptions();
  ControllerTester tester(this, node_options);

  tester.publish_odom_vx(1.0);
  tester.publish_autonomous_operation_mode();
  tester.publish_default_steer();
  tester.publish_default_acc();

  Trajectory traj_msg;
  traj_msg.header.stamp = tester.node->now();
  traj_msg.header.frame_id = "map";
  traj_msg.points.push_back(make_traj_point(-1.0, 0.0, 1.0f));
  traj_msg.points.push_back(make_traj_point(0.0, 0.0, 1.0f));
  traj_msg.points.push_back(make_traj_point(1.0, 0.0, 1.0f));
  traj_msg.points.push_back(make_traj_point(2.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj_msg);

  ASSERT_TRUE(tester.wait_for_control_command(std::chrono::seconds{10}));
  // following conditions will pass even if the MPC solution does not converge
  EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_EQ(tester.cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(tester.cmd_msg->longitudinal.velocity, 0.0f);
  EXPECT_GT(rclcpp::Time(tester.cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

// The behavior of the guard is tested in test_trajectory_follower.cpp. This test
// checks that the node passes the enable_keep_stopped_until_steer_convergence parameter to the
// longitudinal controller.
TEST_F(FakeNodeFixture, longitudinal_check_steer_converged)
{
  // set enable_keep_stopped_until_steer_convergence true
  const auto node_options = makeNodeOptions(true);
  ControllerTester tester(this, node_options);

  tester.publish_default_odom();
  tester.publish_autonomous_operation_mode();
  tester.publish_default_acc();

  // steering_tire_angle has to be larger than the threshold to check convergence.
  const double steering_tire_angle = -0.5;
  tester.publish_steer_angle(steering_tire_angle);

  // Publish trajectory starting away from the current ego pose
  Trajectory traj;
  traj.header.stamp = tester.node->now();
  traj.header.frame_id = "map";
  traj.points.push_back(make_traj_point(0.0, 0.0, 1.0f));
  traj.points.push_back(make_traj_point(50.0, 0.0, 1.0f));
  traj.points.push_back(make_traj_point(100.0, 0.0, 1.0f));
  tester.traj_pub->publish(traj);

  ASSERT_TRUE(tester.wait_for_control_command(std::chrono::seconds{10}));

  // Keep stopped state when the lateral control is not converged.
  EXPECT_DOUBLE_EQ(tester.cmd_msg->longitudinal.velocity, 0.0f);
}
