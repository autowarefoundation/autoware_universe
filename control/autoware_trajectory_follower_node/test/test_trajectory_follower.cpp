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

// Runs TrajectoryFollower with the MPC lateral core and the PID longitudinal core, without the
// ROS 2 runtime. The cases come from test_controller_node.cpp, which checks the same behaviors
// through the topics of the node.

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/trajectory_follower_node/trajectory_follower.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace
{
using autoware::motion::control::mpc_lateral_controller::MpcLateralController;
using autoware::motion::control::mpc_lateral_controller::MpcLateralControllerConfig;
using autoware::motion::control::mpc_lateral_controller::no_log;
using autoware::motion::control::pid_longitudinal_controller::PidLongitudinalController;
using autoware::motion::control::pid_longitudinal_controller::PidLongitudinalControllerConfig;
using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower_node::TrajectoryFollower;
using autoware::motion::control::trajectory_follower_node::TrajectoryFollowerConfig;
using autoware::motion::control::trajectory_follower_node::TrajectoryFollowerError;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr double control_period = 0.03;

/// The values of config/test_vehicle_info.param.yaml in autoware_test_utils.
constexpr double wheel_base = 2.79;
constexpr double front_overhang = 1.0;
constexpr double max_steer_angle = 0.70;

/// The values of config/test_nearest_search.param.yaml in autoware_test_utils.
constexpr double ego_nearest_dist_threshold = 3.0;
constexpr double ego_nearest_yaw_threshold = 1.046;

/// The values of lateral_controller_defaults.param.yaml in autoware_mpc_lateral_controller,
/// combined with the vehicle and the nearest search values above.
MpcLateralControllerConfig make_default_mpc_config()
{
  constexpr double degree_to_radian = M_PI / 180.0;

  MpcLateralControllerConfig config;
  config.ctrl_period = control_period;

  auto & filtering = config.trajectory_filtering;
  filtering.enable_path_smoothing = false;
  filtering.path_filter_moving_ave_num = 25;
  filtering.curvature_smoothing_num_traj = 15;
  filtering.curvature_smoothing_num_ref_steer = 15;
  filtering.traj_resample_dist = 0.1;
  filtering.extend_trajectory_for_end_yaw_control = false;

  config.stop_state_entry_ego_speed = 0.001;
  config.stop_state_entry_target_speed = 0.001;
  config.converged_steer_rad = 0.1;
  config.keep_steer_control_until_converged = true;
  config.new_traj_duration_time = 1.0;
  config.new_traj_end_dist = 0.3;
  config.mpc_converged_threshold_rps = 0.01;

  config.ego_nearest_dist_threshold = ego_nearest_dist_threshold;
  config.ego_nearest_yaw_threshold = ego_nearest_yaw_threshold;

  config.enable_auto_steering_offset_removal = true;
  config.steer_offset_max_update_th = 0.01;
  config.steer_offset_filter_cutoff_hz = 0.5;

  config.steering_lpf_cutoff_hz = 3.0;
  config.error_deriv_lpf_cutoff_hz = 5.0;

  config.wheelbase = wheel_base;
  config.steer_lim = max_steer_angle;
  config.vehicle_model_type = "kinematics";

  config.steer_rate_lim_by_curvature = {
    {0.001, 40.0 * degree_to_radian},
    {0.002, 50.0 * degree_to_radian},
    {0.01, 60.0 * degree_to_radian}};
  config.steer_rate_lim_by_velocity = {
    {10.0, 60.0 * degree_to_radian},
    {15.0, 50.0 * degree_to_radian},
    {20.0, 40.0 * degree_to_radian}};

  config.qp_solver_type = "osqp";
  auto & mpc = config.mpc_param;
  mpc.prediction_horizon = 50;
  mpc.prediction_dt = 0.1;
  mpc.steer_tau = 0.27;
  mpc.zero_ff_steer_deg = 0.5;
  mpc.acceleration_limit = 2.0;
  mpc.velocity_time_constant = 0.3;
  mpc.min_prediction_length = 5.0;
  mpc.low_curvature_thresh_curvature = 0.0;

  auto & nominal = mpc.nominal_weight;
  nominal.lat_error = 1.0;
  nominal.heading_error = 0.0;
  nominal.heading_error_squared_vel = 0.3;
  nominal.steering_input = 1.0;
  nominal.steering_input_squared_vel = 0.25;
  nominal.lat_jerk = 0.1;
  nominal.steer_rate = 0.0;
  nominal.steer_acc = 0.000001;
  nominal.terminal_lat_error = 1.0;
  nominal.terminal_heading_error = 0.1;

  auto & low_curvature = mpc.low_curvature_weight;
  low_curvature.lat_error = 0.1;
  low_curvature.heading_error = 0.0;
  low_curvature.heading_error_squared_vel = 0.3;
  low_curvature.steering_input = 1.0;
  low_curvature.steering_input_squared_vel = 0.25;
  low_curvature.lat_jerk = 0.0;
  low_curvature.steer_rate = 0.0;
  low_curvature.steer_acc = 0.000001;

  config.input_delay = 0.24;
  config.use_steer_prediction = false;
  config.use_delayed_initial_state = true;
  config.use_temporal_trajectory = false;
  config.publish_debug_trajectories = true;
  return config;
}

/// The values of autoware_pid_longitudinal_controller.param.yaml, combined with the vehicle and
/// the nearest search values above. enable_keep_stopped_until_steer_convergence is false here
/// although the shipped file sets it to true, so that only the tests about the guard turn it on.
PidLongitudinalControllerConfig make_default_pid_config()
{
  PidLongitudinalControllerConfig config{};

  config.wheel_base = wheel_base;
  config.front_overhang = front_overhang;
  config.longitudinal_ctrl_period = control_period;

  config.delay_compensation_time = 0.17;
  config.use_temporal_trajectory = false;

  config.enable_smooth_stop = true;
  config.enable_overshoot_emergency = true;
  config.enable_large_tracking_error_emergency = true;
  config.enable_slope_compensation = true;
  config.enable_keep_stopped_until_steer_convergence = false;

  config.state_transition_params.drive_state_stop_dist = 0.5;
  config.state_transition_params.drive_state_offset_stop_dist = 1.0;
  config.state_transition_params.stopping_state_stop_dist = 0.5;
  config.state_transition_params.stopped_state_entry_duration_time = 0.1;
  config.state_transition_params.stopped_state_entry_vel = 0.01;
  config.state_transition_params.stopped_state_entry_acc = 0.1;
  config.state_transition_params.emergency_state_overshoot_stop_dist = 1.5;

  config.pid_gains.kp = 1.0;
  config.pid_gains.ki = 0.1;
  config.pid_gains.kd = 0.0;

  config.pid_limits.max_out = 1.0;
  config.pid_limits.min_out = -1.0;
  config.pid_limits.max_p_effort = 1.0;
  config.pid_limits.min_p_effort = -1.0;
  config.pid_limits.max_i_effort = 0.3;
  config.pid_limits.min_i_effort = -0.3;
  config.pid_limits.max_d_effort = 0.0;
  config.pid_limits.min_d_effort = 0.0;

  config.lpf_vel_error_gain = 0.9;
  config.enable_integration_at_low_speed = false;
  config.current_vel_threshold_pid_integrate = 0.5;
  config.time_threshold_before_pid_integrate = 2.0;
  config.ff_scale_min = 0.5;
  config.ff_scale_max = 2.0;
  config.enable_brake_keeping_before_stop = false;
  config.brake_keeping_acc = -0.2;

  config.smooth_stop_params.max_strong_acc = -0.5;
  config.smooth_stop_params.min_strong_acc = -0.8;
  config.smooth_stop_params.weak_acc = -0.3;
  config.smooth_stop_params.weak_stop_acc = -0.8;
  config.smooth_stop_params.strong_stop_acc = -3.4;
  config.smooth_stop_params.min_fast_vel = 0.5;
  config.smooth_stop_params.min_running_vel = 0.01;
  config.smooth_stop_params.min_running_acc = 0.01;
  config.smooth_stop_params.weak_stop_time = 0.8;
  config.smooth_stop_params.weak_stop_dist = -0.3;
  config.smooth_stop_params.strong_stop_dist = -0.5;

  config.stopped_state_params.vel = 0.0;
  config.stopped_state_params.acc = -3.4;

  config.emergency_state_params.vel = 0.0;
  config.emergency_state_params.acc = -5.0;
  config.emergency_state_params.jerk = -3.0;

  config.acc_feedback_gain = 0.0;
  config.lpf_acc_error_gain = 0.98;

  config.max_acc = 3.0;
  config.min_acc = -5.0;
  config.max_jerk = 2.0;
  config.min_jerk = -5.0;
  config.max_acc_cmd_diff = 50.0;

  config.slope_source = PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_GOAL_ADAPTIVE;
  config.adaptive_trajectory_velocity_th = 1.0;
  config.lpf_pitch_gain = 0.95;
  config.max_pitch_rad = 0.1;
  config.min_pitch_rad = -0.1;

  config.ego_nearest_dist_threshold = ego_nearest_dist_threshold;
  config.ego_nearest_yaw_threshold = ego_nearest_yaw_threshold;

  return config;
}
rclcpp::Time make_time(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

TrajectoryFollower make_trajectory_follower(
  const MpcLateralControllerConfig & mpc_config, const PidLongitudinalControllerConfig & pid_config)
{
  TrajectoryFollowerConfig config;
  config.timeout_thr_sec = 0.5;
  config.enable_control_cmd_horizon = false;
  return TrajectoryFollower(
    config, std::make_unique<MpcLateralController>(mpc_config, no_log),
    std::make_unique<PidLongitudinalController>(pid_config));
}

TrajectoryFollower make_default_trajectory_follower()
{
  return make_trajectory_follower(make_default_mpc_config(), make_default_pid_config());
}

TrajectoryPoint make_point(const double x, const double y, const float velocity)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.longitudinal_velocity_mps = velocity;
  return point;
}

Trajectory make_trajectory(const std::vector<TrajectoryPoint> & points)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points = points;
  return trajectory;
}

/// A trajectory along the x axis with points 50 m apart, so that no stop point is near the ego.
Trajectory make_long_straight_trajectory(const float velocity)
{
  return make_trajectory(
    {make_point(0.0, 0.0, velocity), make_point(50.0, 0.0, velocity),
     make_point(100.0, 0.0, velocity)});
}

/// The ego stands at the origin, heads along the x axis and is under the control of Autoware.
InputData make_input_data(
  const Trajectory & trajectory, const double ego_velocity, const double measured_steering_angle)
{
  InputData input_data;
  input_data.current_trajectory = trajectory;
  input_data.current_odometry.header.frame_id = "map";
  input_data.current_odometry.twist.twist.linear.x = ego_velocity;
  input_data.current_steering.steering_tire_angle = static_cast<float>(measured_steering_angle);
  input_data.current_operation_mode.mode = OperationModeState::AUTONOMOUS;
  input_data.current_operation_mode.is_autoware_control_enabled = true;
  return input_data;
}
}  // namespace

TEST(TrajectoryFollowerTest, ReturnsErrorWhenTrajectoryIsEmpty)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto input_data = make_input_data(make_trajectory({}), 0.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().level, TrajectoryFollowerError::Level::info);
}

// lateral

TEST(TrajectoryFollowerTest, KeepsSteeringStraightOnStraightTrajectory)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto trajectory = make_trajectory(
    {make_point(-1.0, 0.0, 1.0f), make_point(0.0, 0.0, 1.0f), make_point(1.0, 0.0, 1.0f),
     make_point(2.0, 0.0, 1.0f)});
  const auto input_data = make_input_data(trajectory, 1.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->control_cmd.lateral.steering_tire_angle, 0.0f);
  EXPECT_EQ(result->control_cmd.lateral.steering_tire_rotation_rate, 0.0f);
}

TEST(TrajectoryFollowerTest, SteersRightOnRightTurningTrajectory)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto trajectory = make_trajectory(
    {make_point(-1.0, -1.0, 1.0f), make_point(0.0, 0.0, 1.0f), make_point(1.0, -1.0, 1.0f),
     make_point(2.0, -2.0, 1.0f)});
  const auto input_data = make_input_data(trajectory, 1.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_LT(result->control_cmd.lateral.steering_tire_angle, 0.0f);
  EXPECT_LT(result->control_cmd.lateral.steering_tire_rotation_rate, 0.0f);
}

TEST(TrajectoryFollowerTest, SteersLeftOnLeftTurningTrajectory)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto trajectory = make_trajectory(
    {make_point(-1.0, 1.0, 1.0f), make_point(0.0, 0.0, 1.0f), make_point(1.0, 1.0, 1.0f),
     make_point(2.0, 2.0, 1.0f)});
  const auto input_data = make_input_data(trajectory, 1.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result->control_cmd.lateral.steering_tire_angle, 0.0f);
  EXPECT_GT(result->control_cmd.lateral.steering_tire_rotation_rate, 0.0f);
}

TEST(TrajectoryFollowerTest, HoldsMeasuredSteeringWhileStopped)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto trajectory = make_trajectory(
    {make_point(-1.0, 0.0, 0.0f), make_point(0.0, 0.0, 0.0f), make_point(1.0, 0.0, 0.0f),
     make_point(2.0, 0.0, 0.0f)});
  const auto input_data = make_input_data(trajectory, 0.0, -0.5);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->control_cmd.lateral.steering_tire_angle, -0.5f);
  EXPECT_EQ(result->control_cmd.lateral.steering_tire_rotation_rate, 0.0f);
}

// longitudinal

TEST(TrajectoryFollowerTest, KeepsVelocityWhenEgoDrivesAtTargetVelocity)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto input_data = make_input_data(make_long_straight_trajectory(1.0f), 1.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 1.0f);
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.acceleration, 0.0f);
}

TEST(TrajectoryFollowerTest, DeceleratesWhenEgoIsFasterThanTargetVelocity)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto input_data = make_input_data(make_long_straight_trajectory(0.5f), 1.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 0.5f);
  EXPECT_LT(result->control_cmd.longitudinal.acceleration, 0.0f);
}

TEST(TrajectoryFollowerTest, AcceleratesWhenEgoIsSlowerThanTargetVelocity)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto input_data = make_input_data(make_long_straight_trajectory(1.0f), 0.5, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 1.0f);
  EXPECT_GT(result->control_cmd.longitudinal.acceleration, 0.0f);
}

TEST(TrajectoryFollowerTest, BrakesWhenTrajectoryAsksForStop)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto input_data = make_input_data(make_long_straight_trajectory(0.0f), 0.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 0.0f);
  EXPECT_LT(result->control_cmd.longitudinal.acceleration, 0.0f);
}

TEST(TrajectoryFollowerTest, DrivesBackwardOnReverseTrajectory)
{
  // Arrange
  auto trajectory_follower = make_default_trajectory_follower();
  const auto input_data = make_input_data(make_long_straight_trajectory(-1.0f), 0.0, 0.0);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, -1.0f);
  EXPECT_GT(result->control_cmd.longitudinal.acceleration, 0.0f);
}

// interaction between the two controllers

TEST(TrajectoryFollowerTest, DepartsWithUnconvergedSteeringWhenGuardIsDisabled)
{
  // Arrange
  auto pid_config = make_default_pid_config();
  pid_config.enable_keep_stopped_until_steer_convergence = false;
  auto trajectory_follower = make_trajectory_follower(make_default_mpc_config(), pid_config);
  const auto input_data = make_input_data(make_long_straight_trajectory(1.0f), 0.0, -0.5);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 1.0f);
}

TEST(TrajectoryFollowerTest, KeepsStoppedWhileSteeringIsNotConvergedWhenGuardIsEnabled)
{
  // Arrange
  auto pid_config = make_default_pid_config();
  pid_config.enable_keep_stopped_until_steer_convergence = true;
  auto trajectory_follower = make_trajectory_follower(make_default_mpc_config(), pid_config);
  const auto input_data = make_input_data(make_long_straight_trajectory(1.0f), 0.0, -0.5);

  // Act
  const auto result = trajectory_follower.run(input_data, make_time(0.0));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_FLOAT_EQ(result->control_cmd.longitudinal.velocity, 0.0f);
}
