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

#include <sstream>

#define private public
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#undef private

#include "autoware/trajectory/interpolator/linear.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "gtest/gtest.h"

#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pid_longitudinal_controller = ::autoware::motion::control::pid_longitudinal_controller;
namespace experimental_trajectory = ::autoware::experimental::trajectory;
namespace
{

using TrajectoryExperimental = pid_longitudinal_controller::TrajectoryExperimental;

autoware_planning_msgs::msg::TrajectoryPoint makeTrajectoryPoint(
  const double x, const double y, const double z, const double velocity, const double acceleration)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = z;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = velocity;
  point.acceleration_mps2 = acceleration;
  return point;
}

TrajectoryExperimental makeContinuousTrajectory(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points)
{
  const auto trajectory =
    TrajectoryExperimental::Builder{}
      .template set_xy_interpolator<experimental_trajectory::interpolator::Linear>()
      .template set_z_interpolator<experimental_trajectory::interpolator::Linear>()
      .build(points);
  if (!trajectory) {
    throw std::runtime_error("failed to build experimental trajectory for test");
  }
  return trajectory.value();
}

std::vector<rclcpp::Parameter> makeControllerParameters()
{
  return {
    rclcpp::Parameter("ctrl_period", 0.03),
    rclcpp::Parameter("wheel_radius", 0.39),
    rclcpp::Parameter("wheel_width", 0.42),
    rclcpp::Parameter("wheel_base", 2.74),
    rclcpp::Parameter("wheel_tread", 1.63),
    rclcpp::Parameter("front_overhang", 1.0),
    rclcpp::Parameter("rear_overhang", 1.03),
    rclcpp::Parameter("left_overhang", 0.1),
    rclcpp::Parameter("right_overhang", 0.1),
    rclcpp::Parameter("vehicle_height", 2.5),
    rclcpp::Parameter("max_steer_angle", 0.7),
    rclcpp::Parameter("delay_compensation_time", 0.17),
    rclcpp::Parameter("enable_smooth_stop", true),
    rclcpp::Parameter("enable_overshoot_emergency", true),
    rclcpp::Parameter("enable_large_tracking_error_emergency", true),
    rclcpp::Parameter("enable_slope_compensation", true),
    rclcpp::Parameter("enable_keep_stopped_until_steer_convergence", true),
    rclcpp::Parameter("drive_state_stop_dist", 0.5),
    rclcpp::Parameter("drive_state_offset_stop_dist", 1.0),
    rclcpp::Parameter("stopping_state_stop_dist", 0.5),
    rclcpp::Parameter("stopped_state_entry_duration_time", 0.1),
    rclcpp::Parameter("stopped_state_entry_vel", 0.01),
    rclcpp::Parameter("stopped_state_entry_acc", 0.1),
    rclcpp::Parameter("emergency_state_overshoot_stop_dist", 1.5),
    rclcpp::Parameter("kp", 1.0),
    rclcpp::Parameter("ki", 0.1),
    rclcpp::Parameter("kd", 0.0),
    rclcpp::Parameter("max_out", 1.0),
    rclcpp::Parameter("min_out", -1.0),
    rclcpp::Parameter("max_p_effort", 1.0),
    rclcpp::Parameter("min_p_effort", -1.0),
    rclcpp::Parameter("max_i_effort", 0.3),
    rclcpp::Parameter("min_i_effort", -0.3),
    rclcpp::Parameter("max_d_effort", 0.0),
    rclcpp::Parameter("min_d_effort", 0.0),
    rclcpp::Parameter("lpf_vel_error_gain", 0.9),
    rclcpp::Parameter("enable_integration_at_low_speed", false),
    rclcpp::Parameter("current_vel_threshold_pid_integration", 0.5),
    rclcpp::Parameter("time_threshold_before_pid_integration", 2.0),
    rclcpp::Parameter("ff_scale_min", 0.5),
    rclcpp::Parameter("ff_scale_max", 2.0),
    rclcpp::Parameter("enable_brake_keeping_before_stop", false),
    rclcpp::Parameter("brake_keeping_acc", -0.2),
    rclcpp::Parameter("smooth_stop_max_strong_acc", -0.5),
    rclcpp::Parameter("smooth_stop_min_strong_acc", -0.8),
    rclcpp::Parameter("smooth_stop_weak_acc", -0.3),
    rclcpp::Parameter("smooth_stop_weak_stop_acc", -0.8),
    rclcpp::Parameter("smooth_stop_strong_stop_acc", -3.4),
    rclcpp::Parameter("smooth_stop_max_fast_vel", 0.5),
    rclcpp::Parameter("smooth_stop_min_running_vel", 0.01),
    rclcpp::Parameter("smooth_stop_min_running_acc", 0.01),
    rclcpp::Parameter("smooth_stop_weak_stop_time", 0.8),
    rclcpp::Parameter("smooth_stop_weak_stop_dist", -0.3),
    rclcpp::Parameter("smooth_stop_strong_stop_dist", -0.5),
    rclcpp::Parameter("stopped_vel", 0.0),
    rclcpp::Parameter("stopped_acc", -3.4),
    rclcpp::Parameter("emergency_vel", 0.0),
    rclcpp::Parameter("emergency_acc", -5.0),
    rclcpp::Parameter("emergency_jerk", -3.0),
    rclcpp::Parameter("lpf_acc_error_gain", 0.98),
    rclcpp::Parameter("acc_feedback_gain", 0.0),
    rclcpp::Parameter("max_acc", 3.0),
    rclcpp::Parameter("min_acc", -5.0),
    rclcpp::Parameter("max_jerk", 2.0),
    rclcpp::Parameter("min_jerk", -5.0),
    rclcpp::Parameter("max_acc_cmd_diff", 50.0),
    rclcpp::Parameter("lpf_pitch_gain", 0.95),
    rclcpp::Parameter("slope_source", std::string("trajectory_goal_adaptive")),
    rclcpp::Parameter("adaptive_trajectory_velocity_th", 1.0),
    rclcpp::Parameter("max_pitch_rad", 0.1),
    rclcpp::Parameter("min_pitch_rad", -0.1),
    rclcpp::Parameter("ego_nearest_dist_threshold", 3.0),
    rclcpp::Parameter("ego_nearest_yaw_threshold", 1.046),
  };
}

void declareBootstrapParameters(rclcpp::Node & node)
{
  node.declare_parameter<double>("ctrl_period", 0.03);
  node.declare_parameter<double>("wheel_radius", 0.39);
  node.declare_parameter<double>("wheel_width", 0.42);
  node.declare_parameter<double>("wheel_base", 2.74);
  node.declare_parameter<double>("wheel_tread", 1.63);
  node.declare_parameter<double>("front_overhang", 1.0);
  node.declare_parameter<double>("rear_overhang", 1.03);
  node.declare_parameter<double>("left_overhang", 0.1);
  node.declare_parameter<double>("right_overhang", 0.1);
  node.declare_parameter<double>("vehicle_height", 2.5);
  node.declare_parameter<double>("max_steer_angle", 0.7);
}

class TestablePidLongitudinalController
: public pid_longitudinal_controller::PidLongitudinalController
{
public:
  explicit TestablePidLongitudinalController(rclcpp::Node & node)
  : pid_longitudinal_controller::PidLongitudinalController(
      node, std::make_shared<diagnostic_updater::Updater>(&node))
  {
  }
};

class PidLongitudinalControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(PidLongitudinalControllerTest, goalOverrunKeepsContinuousControlData)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(makeControllerParameters());
  auto node = std::make_shared<rclcpp::Node>("test_pid_longitudinal_controller", options);
  declareBootstrapParameters(*node);

  TestablePidLongitudinalController controller(*node);

  const auto trajectory = makeContinuousTrajectory(
    {makeTrajectoryPoint(0.0, 0.0, 0.0, 3.0, 0.0), makeTrajectoryPoint(2.0, 0.0, 0.0, 2.0, 0.0),
     makeTrajectoryPoint(4.0, 0.0, 0.0, 1.0, 0.0), makeTrajectoryPoint(5.0, 0.0, 0.0, 0.0, 0.0)});

  controller.setTrajectory(trajectory);

  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = 6.5;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 1.0;
  controller.setKinematicState(odometry);

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.accel.accel.linear.x = -1.0;
  controller.setCurrentAcceleration(accel);

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
  operation_mode.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  controller.setCurrentOperationMode(operation_mode);

  const auto near_overrun_control_data = controller.getExperimentalControlData(odometry.pose.pose);

  ASSERT_TRUE(near_overrun_control_data);

  odometry.pose.pose.position.x = 9.0;
  controller.setKinematicState(odometry);

  const auto control_data = controller.getExperimentalControlData(odometry.pose.pose);

  ASSERT_TRUE(control_data);
}

TEST_F(PidLongitudinalControllerTest, goalOverrunKeepsContinuousControlDataEvenIfPoseIsImperfect)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(makeControllerParameters());
  auto node = std::make_shared<rclcpp::Node>("test_pid_longitudinal_controller_rejection", options);
  declareBootstrapParameters(*node);

  TestablePidLongitudinalController controller(*node);

  const auto trajectory = makeContinuousTrajectory(
    {makeTrajectoryPoint(0.0, 0.0, 0.0, 3.0, 0.0), makeTrajectoryPoint(2.0, 0.0, 0.0, 2.0, 0.0),
     makeTrajectoryPoint(4.0, 0.0, 0.0, 1.0, 0.0), makeTrajectoryPoint(5.0, 0.0, 0.0, 0.0, 0.0)});

  controller.setTrajectory(trajectory);

  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = 6.5;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 1.0;
  controller.setKinematicState(odometry);

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.accel.accel.linear.x = -1.0;
  controller.setCurrentAcceleration(accel);

  autoware_adapi_v1_msgs::msg::OperationModeState operation_mode;
  operation_mode.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  controller.setCurrentOperationMode(operation_mode);

  tf2::Quaternion misaligned_quaternion;
  misaligned_quaternion.setRPY(0.0, 0.0, 1.2);
  odometry.pose.pose.orientation.x = misaligned_quaternion.x();
  odometry.pose.pose.orientation.y = misaligned_quaternion.y();
  odometry.pose.pose.orientation.z = misaligned_quaternion.z();
  odometry.pose.pose.orientation.w = misaligned_quaternion.w();
  controller.setKinematicState(odometry);
  EXPECT_TRUE(controller.getExperimentalControlData(odometry.pose.pose));

  odometry.pose.pose.orientation.w = 1.0;
  odometry.pose.pose.orientation.x = 0.0;
  odometry.pose.pose.orientation.y = 0.0;
  odometry.pose.pose.orientation.z = 0.0;
  odometry.pose.pose.position.y = 3.5;
  controller.setKinematicState(odometry);
  EXPECT_TRUE(controller.getExperimentalControlData(odometry.pose.pose));
}

}  // namespace
