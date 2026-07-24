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

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "gtest/gtest.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <cstddef>
#include <cstdint>

namespace
{
using autoware::motion::control::pid_longitudinal_controller::ControlState;
using autoware::motion::control::pid_longitudinal_controller::OperationModeState;
using autoware::motion::control::pid_longitudinal_controller::PidLongitudinalController;
using autoware::motion::control::pid_longitudinal_controller::PidLongitudinalControllerConfig;
using autoware::motion::control::trajectory_follower::InputData;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

// Helper function that builds a fully-populated, valid config equivalent to the shipped
// default parameters (autoware_pid_longitudinal_controller.param.yaml). Tests override only the
// fields relevant to what they verify.
PidLongitudinalControllerConfig make_default_config()
{
  PidLongitudinalControllerConfig config{};

  config.wheel_base = 2.79;
  config.front_overhang = 1.0;
  config.longitudinal_ctrl_period = 0.03;

  config.delay_compensation_time = 0.17;
  config.use_temporal_trajectory = false;

  config.enable_smooth_stop = true;
  config.enable_overshoot_emergency = true;
  config.enable_large_tracking_error_emergency = true;
  config.enable_slope_compensation = true;
  config.enable_keep_stopped_until_steer_convergence = true;

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

  config.ego_nearest_dist_threshold = 3.0;
  config.ego_nearest_yaw_threshold = 1.0472;

  return config;
}

// Helper function that builds a straight, flat trajectory along the +x axis starting at start_x.
// Every point shares the given constant velocity and zero acceleration.
Trajectory make_straight_trajectory(
  const std::size_t num_points, const double spacing, const double velocity,
  const double start_x = 0.0)
{
  Trajectory trajectory;
  for (std::size_t point_index = 0; point_index < num_points; ++point_index) {
    TrajectoryPoint point;
    point.pose.position.x = start_x + static_cast<double>(point_index) * spacing;
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;  // identity orientation, heading along +x
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.acceleration_mps2 = 0.0f;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

// Helper function that builds a straight, flat trajectory carrying a per-point time_from_start,
// used to exercise the temporal (time-based) trajectory path.
Trajectory make_temporal_trajectory(
  const std::size_t num_points, const double spacing, const double velocity, const double time_step,
  const double stamp_sec)
{
  Trajectory trajectory = make_straight_trajectory(num_points, spacing, velocity);
  trajectory.header.stamp = rclcpp::Time(static_cast<int32_t>(stamp_sec), 0);
  for (std::size_t point_index = 0; point_index < num_points; ++point_index) {
    trajectory.points.at(point_index).time_from_start =
      rclcpp::Duration::from_seconds(static_cast<double>(point_index) * time_step);
  }
  return trajectory;
}

// Helper function that builds input data. The ego sits at ego_x with the given longitudinal
// velocity; autonomous selects the autoware-controlled AUTONOMOUS operation mode.
InputData make_input_data(
  const Trajectory & trajectory, const double current_vel = 0.0, const double ego_x = 0.0,
  const bool autonomous = false)
{
  InputData input_data;
  input_data.current_trajectory = trajectory;
  input_data.current_odometry.pose.pose.position.x = ego_x;
  input_data.current_odometry.pose.pose.orientation.w = 1.0;
  input_data.current_odometry.twist.twist.linear.x = current_vel;
  input_data.current_accel.accel.accel.linear.x = 0.0;
  if (autonomous) {
    input_data.current_operation_mode.is_autoware_control_enabled = true;
    input_data.current_operation_mode.mode = OperationModeState::AUTONOMOUS;
  }
  return input_data;
}
}  // namespace

TEST(PidLongitudinalController, StaysStoppedWhenCloseToStopPoint)
{
  // Arrange
  // A zero-velocity trajectory places the stop point at the ego, so the distance to the stop
  // point stays below drive_state_stop_dist and no departure is triggered.
  const auto config = make_default_config();
  PidLongitudinalController controller(config);
  const auto input_data = make_input_data(make_straight_trajectory(10, 1.0, 0.0));

  // Act
  const auto result = controller.run(input_data, rclcpp::Time(1, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.velocity, static_cast<float>(config.stopped_state_params.vel));
  // The stopped-state acceleration is negative by config; slope compensation may further adjust
  // it, but STOPPED must never command a positive (accelerating) output.
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, StoppedStateOutputsStoppedStateCommand)
{
  // Arrange
  // Raise max_acc_cmd_diff so the stopped acceleration passes the command diff filter unchanged
  // on the first cycle, letting us assert the exact stopped-state command.
  auto config = make_default_config();
  config.max_acc_cmd_diff = 1000.0;
  PidLongitudinalController controller(config);
  const auto input_data = make_input_data(make_straight_trajectory(10, 1.0, 0.0));

  // Act
  const auto result = controller.run(input_data, rclcpp::Time(1, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.velocity, static_cast<float>(config.stopped_state_params.vel));
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.acceleration, static_cast<float>(config.stopped_state_params.acc));
}

TEST(PidLongitudinalController, DepartsToDriveWhenFarFromStopAndKeepStoppedDisabled)
{
  // Arrange
  // A forward trajectory with no stop point keeps the ego far from any stop point, and disabling
  // the keep-stopped-until-steer-convergence guard lets the ego depart immediately.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  const auto input_data = make_input_data(make_straight_trajectory(20, 1.0, 5.0));

  // Act
  const auto result = controller.run(input_data, rclcpp::Time(1, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::DRIVE);
  // The target velocity (5.0) passes through unchanged, and the ego is far below it, so the PID
  // feedback must command a positive (accelerating) acceleration.
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 5.0f);
  EXPECT_GT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, KeepsStoppedUntilSteerConvergesWhenGuardEnabled)
{
  // Arrange
  // Far from the stop point but with the keep-stopped guard enabled and steering not converged,
  // the ego must remain STOPPED even though the departure distance condition is met.
  const auto config = make_default_config();
  PidLongitudinalController controller(config);
  const auto input_data = make_input_data(make_straight_trajectory(20, 1.0, 5.0));

  // Act
  const auto result = controller.run(input_data, rclcpp::Time(1, 0), false);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.velocity, static_cast<float>(config.stopped_state_params.vel));
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, SetConfigAppliesNewStoppedStateCommand)
{
  // Arrange
  // Construct with one stopped-state acceleration, then apply a new config via setConfig and
  // verify the updated stopped-state command is emitted. max_acc_cmd_diff is raised so the
  // command passes the diff filter unchanged on the first cycle.
  auto config = make_default_config();
  config.max_acc_cmd_diff = 1000.0;
  config.stopped_state_params.acc = -3.4;
  PidLongitudinalController controller(config);

  auto new_config = config;
  new_config.stopped_state_params.acc = -2.0;
  controller.setConfig(new_config);

  const auto input_data = make_input_data(make_straight_trajectory(10, 1.0, 0.0));

  // Act
  const auto result = controller.run(input_data, rclcpp::Time(1, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.acceleration,
    static_cast<float>(new_config.stopped_state_params.acc));
}

TEST(PidLongitudinalController, TransitionsToStoppingNearStopPoint)
{
  // Arrange
  // Reach DRIVE first (far forward trajectory, keep-stopped guard disabled), then present a stop
  // point 0.3 m ahead so the distance to the stop point falls below stopping_state_stop_dist.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  controller.run(make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(1, 0), true);

  // Act
  const auto near_stop_trajectory = make_straight_trajectory(20, 1.0, 0.0, 0.3);
  const auto result =
    controller.run(make_input_data(near_stop_trajectory), rclcpp::Time(2, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPING);
  // STOPPING always targets the stopped-state velocity, and the smooth-stop controller must not
  // command acceleration while approaching a stop.
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 0.0f);
  EXPECT_LE(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, TransitionsToEmergencyWhenOvershootingStopPoint)
{
  // Arrange
  // Reach DRIVE first, then present a zero-velocity trajectory whose stop point lies well behind
  // the ego (ego at x=10, stop point at x=0), i.e. the ego has overshot the stop point.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  controller.run(make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(1, 0), true);

  // Act
  const auto overshoot_trajectory = make_straight_trajectory(20, 1.0, 0.0);
  const auto result =
    controller.run(make_input_data(overshoot_trajectory, 0.0, 10.0), rclcpp::Time(2, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::EMERGENCY);
  EXPECT_TRUE(result.emergency_stop_reason.has_value());
  // EMERGENCY targets a full stop with a rate-limited velocity command, so the commanded velocity
  // must be reduced from the pre-emergency target (5.0), and the commanded acceleration must be a
  // hard braking command, never a positive (accelerating) one.
  EXPECT_LT(result.output.control_cmd.velocity, 5.0f);
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, KeepsBrakeBeforeStopWhenEnabled)
{
  // Arrange
  // Keep the ego in DRIVE toward a stop point far ahead (smooth stop disabled) while brake
  // keeping is enabled, exercising the keep-brake-before-stop path.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  config.enable_smooth_stop = false;
  config.enable_brake_keeping_before_stop = true;
  PidLongitudinalController controller(config);
  controller.run(make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(1, 0), true);

  auto trajectory_with_stop = make_straight_trajectory(20, 1.0, 3.0);
  trajectory_with_stop.points.at(18).longitudinal_velocity_mps = 0.0f;
  trajectory_with_stop.points.at(19).longitudinal_velocity_mps = 0.0f;

  // Act
  const auto result =
    controller.run(make_input_data(trajectory_with_stop), rclcpp::Time(2, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::DRIVE);
  // Brake keeping caps the acceleration at brake_keeping_acc (non-positive) on approach to the
  // stop point, so the ego must not be commanded to accelerate here.
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 3.0f);
  EXPECT_LE(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, PredictsStateFromCommandHistoryWhenAutonomous)
{
  // Arrange
  // Under autonomous control the delay compensation predicts the future state from the recorded
  // command history. Drive one cycle to fill the command buffer, then run a second cycle whose
  // time is within delay_compensation_time so the history-based prediction loop executes.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  const auto trajectory = make_straight_trajectory(30, 1.0, 5.0);
  controller.run(
    make_input_data(trajectory, 3.0, 0.0, true), rclcpp::Time(1, 0, RCL_ROS_TIME), true);

  // Act
  const auto result = controller.run(
    make_input_data(trajectory, 3.0, 1.0, true), rclcpp::Time(1, 100000000, RCL_ROS_TIME), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::DRIVE);
  // The target velocity (5.0) passes through unchanged regardless of the history-based
  // delay-compensated prediction used internally for the PID feedback term.
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 5.0f);
}

TEST(PidLongitudinalController, RawPitchSlopeSourceProducesStoppedState)
{
  // Arrange
  auto config = make_default_config();
  config.slope_source = PidLongitudinalControllerConfig::SlopeSource::RAW_PITCH;
  PidLongitudinalController controller(config);
  const auto input_data = make_input_data(make_straight_trajectory(10, 1.0, 0.0));

  // Act
  const auto result = controller.run(input_data, rclcpp::Time(1, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.velocity, static_cast<float>(config.stopped_state_params.vel));
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, DepartsFromStoppingWhenStopPointRecedes)
{
  // Arrange
  // Enter STOPPING with a nearby stop point, then present a far stop point so the distance
  // exceeds drive_state_stop_dist + drive_state_offset_stop_dist and the ego departs to DRIVE.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  controller.run(make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(1, 0), true);
  controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0, 0.3)), rclcpp::Time(2, 0), true);

  // Act
  const auto result = controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(3, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::DRIVE);
  // The target velocity (5.0) passes through unchanged, and departing STOPPING clamps the
  // previous raw acceleration to be non-negative so the ego does not stall on departure.
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 5.0f);
  EXPECT_GE(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, RecoversFromEmergencyToDriveWhenControlReleased)
{
  // Arrange
  // Drive, overshoot into EMERGENCY, then present a valid forward trajectory while not under
  // autoware control so the emergency clears and the ego returns to DRIVE.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  controller.run(make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(1, 0), true);
  controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0), 0.0, 10.0), rclcpp::Time(2, 0), true);

  // Act
  const auto result = controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(3, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::DRIVE);
  // The target velocity (5.0) passes through unchanged once DRIVE resumes.
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 5.0f);
}

TEST(PidLongitudinalController, DrivesInReverseForNegativeVelocityTrajectory)
{
  // Arrange
  // A trajectory with negative target velocity puts the ego in the reverse shift, exercising the
  // reverse-direction sign handling in the shift, velocity-feedback, and acc-feedback paths.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  const auto reverse_trajectory = make_straight_trajectory(20, 1.0, -5.0);

  // Act
  const auto result =
    controller.run(make_input_data(reverse_trajectory, -3.0), rclcpp::Time(1, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::DRIVE);
  // Target velocity (-5.0) passes through unchanged; a positive acceleration means "speed up in
  // the current (reverse) gear direction" per the direction-agnostic accel command convention.
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, -5.0f);
  EXPECT_GT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, KeepStoppedShowsVirtualWallUnderAutonomousControl)
{
  // Arrange
  // Under autonomous control, far from the stop point but with steering not converged, the ego
  // keeps STOPPED and raises a virtual wall marker. Two cycles cover the branch that also
  // considers the previous keep-stopped condition.
  const auto config = make_default_config();
  PidLongitudinalController controller(config);
  const auto trajectory = make_straight_trajectory(20, 1.0, 5.0);
  controller.run(
    make_input_data(trajectory, 0.0, 0.0, true), rclcpp::Time(1, 0, RCL_ROS_TIME), false);

  // Act
  const auto result = controller.run(
    make_input_data(trajectory, 0.0, 0.0, true), rclcpp::Time(2, 0, RCL_ROS_TIME), false);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_TRUE(result.virtual_wall_marker.has_value());
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.velocity, static_cast<float>(config.stopped_state_params.vel));
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.acceleration, static_cast<float>(config.stopped_state_params.acc));
}

TEST(PidLongitudinalController, DrivesToStoppedAfterStandstillDuration)
{
  // Arrange
  // Drive while running (setting the last-running time), then stand still past
  // stopped_state_entry_duration_time while not under control, so DRIVE transitions to STOPPED.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  const auto trajectory = make_straight_trajectory(20, 1.0, 5.0);
  controller.run(make_input_data(trajectory, 3.0), rclcpp::Time(1, 0), true);

  // Act
  const auto result =
    controller.run(make_input_data(trajectory, 0.0), rclcpp::Time(1, 200000000), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.velocity, static_cast<float>(config.stopped_state_params.vel));
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, StaysStoppingWhileStopPointHolds)
{
  // Arrange
  // Enter STOPPING, then hold the same near stop point so no exit condition fires and the ego
  // remains in STOPPING.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  const auto near_stop_trajectory = make_straight_trajectory(20, 1.0, 0.0, 0.3);
  controller.run(make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(1, 0), true);
  controller.run(make_input_data(near_stop_trajectory), rclcpp::Time(2, 0), true);

  // Act
  const auto result =
    controller.run(make_input_data(near_stop_trajectory), rclcpp::Time(3, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPING);
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 0.0f);
  EXPECT_LE(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, StoppingTransitionsToEmergencyOnOvershoot)
{
  // Arrange
  // Enter STOPPING, then overshoot the stop point so STOPPING transitions to EMERGENCY.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  controller.run(make_input_data(make_straight_trajectory(20, 1.0, 5.0)), rclcpp::Time(1, 0), true);
  controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0, 0.3)), rclcpp::Time(2, 0), true);

  // Act
  const auto result = controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0), 0.0, 10.0), rclcpp::Time(3, 0), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::EMERGENCY);
  EXPECT_FLOAT_EQ(result.output.control_cmd.velocity, 0.0f);
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, EmergencyTransitionsToStoppedAfterStandstill)
{
  // Arrange
  // Drive while running, overshoot into EMERGENCY, then stay overshot past the standstill
  // duration so EMERGENCY transitions to STOPPED.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 5.0), 3.0), rclcpp::Time(1, 0), true);
  controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0), 0.0, 10.0), rclcpp::Time(1, 200000000),
    true);

  // Act
  const auto result = controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0), 0.0, 10.0), rclcpp::Time(1, 400000000),
    true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::STOPPED);
  EXPECT_FLOAT_EQ(
    result.output.control_cmd.velocity, static_cast<float>(config.stopped_state_params.vel));
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, StaysInEmergencyWhileOvershootPersistsUnderControl)
{
  // Arrange
  // Under autonomous control, reach EMERGENCY by overshooting and keep overshooting so the
  // emergency condition persists and the ego stays in EMERGENCY.
  auto config = make_default_config();
  config.enable_keep_stopped_until_steer_convergence = false;
  PidLongitudinalController controller(config);
  controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 5.0), 0.0, 0.0, true),
    rclcpp::Time(1, 0, RCL_ROS_TIME), true);
  controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0), 0.0, 10.0, true),
    rclcpp::Time(1, 100000000, RCL_ROS_TIME), true);

  // Act
  const auto result = controller.run(
    make_input_data(make_straight_trajectory(20, 1.0, 0.0), 0.0, 10.0, true),
    rclcpp::Time(1, 200000000, RCL_ROS_TIME), true);

  // Assert
  EXPECT_EQ(result.control_state, ControlState::EMERGENCY);
  // EMERGENCY's rate-limited velocity command must have decreased from the pre-emergency target
  // (5.0), and the acceleration must remain a braking (negative) command.
  EXPECT_LT(result.output.control_cmd.velocity, 5.0f);
  EXPECT_LT(result.output.control_cmd.acceleration, 0.0);
}

TEST(PidLongitudinalController, TemporalTrajectoryProducesValidCommand)
{
  // Arrange
  // A time-parameterized trajectory drives the temporal (time-based) nearest/target point
  // selection instead of the spatial geometric projection.
  auto config = make_default_config();
  config.use_temporal_trajectory = true;
  PidLongitudinalController controller(config);
  const auto temporal_trajectory = make_temporal_trajectory(20, 1.0, 3.0, 0.1, 1.0);

  // Act
  const auto result = controller.run(
    make_input_data(temporal_trajectory, 3.0), rclcpp::Time(1, 500000000, RCL_ROS_TIME), true);

  // Assert
  EXPECT_FALSE(result.received_invalid_trajectory);
}
