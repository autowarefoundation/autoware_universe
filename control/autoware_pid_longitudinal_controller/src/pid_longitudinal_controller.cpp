// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
namespace
{
bool is_valid_trajectory(
  const autoware_planning_msgs::msg::Trajectory & msg, const bool use_temporal_trajectory)
{
  if (!longitudinal_utils::isValidTrajectory(msg, use_temporal_trajectory)) {
    return false;
  }

  if (msg.points.size() < 2) {
    return false;
  }

  return true;
}
}  // namespace

PidLongitudinalController::PidLongitudinalController(const PidLongitudinalControllerConfig & cfg)
: config(cfg)
{
  m_pid_vel.setGains(config.pid_gains.kp, config.pid_gains.ki, config.pid_gains.kd);
  m_pid_vel.setLimits(
    config.pid_limits.max_out, config.pid_limits.min_out, config.pid_limits.max_p_effort,
    config.pid_limits.min_p_effort, config.pid_limits.max_i_effort, config.pid_limits.min_i_effort,
    config.pid_limits.max_d_effort, config.pid_limits.min_d_effort);

  m_lpf_vel_error = std::make_shared<LowpassFilter1d>(0.0, config.lpf_vel_error_gain);
  m_lpf_acc_error = std::make_shared<LowpassFilter1d>(0.0, config.lpf_acc_error_gain);
  m_lpf_pitch = std::make_shared<LowpassFilter1d>(0.0, config.lpf_pitch_gain);

  m_smooth_stop.emplace(config.smooth_stop_params);
}

void PidLongitudinalController::setConfig(const PidLongitudinalControllerConfig & new_config)
{
  config = new_config;

  m_pid_vel.setGains(config.pid_gains.kp, config.pid_gains.ki, config.pid_gains.kd);
  m_pid_vel.setLimits(
    config.pid_limits.max_out, config.pid_limits.min_out, config.pid_limits.max_p_effort,
    config.pid_limits.min_p_effort, config.pid_limits.max_i_effort, config.pid_limits.min_i_effort,
    config.pid_limits.max_d_effort, config.pid_limits.min_d_effort);
  m_lpf_vel_error->setGain(config.lpf_vel_error_gain);
  m_lpf_acc_error->setGain(config.lpf_acc_error_gain);
  m_smooth_stop->setParams(config.smooth_stop_params);
}

PidLongitudinalControllerResult PidLongitudinalController::run(
  const trajectory_follower::InputData & input_data, const rclcpp::Time & current_time,
  const bool is_steer_converged)
{
  // capture the time and lateral convergence state once for this control cycle
  m_current_time = current_time;
  m_is_steer_converged = is_steer_converged;
  m_received_invalid_trajectory = false;
  m_emergency_stop_reason = std::nullopt;

  // check input data
  if (!is_valid_trajectory(input_data.current_trajectory, config.use_temporal_trajectory)) {
    m_received_invalid_trajectory = true;
  }

  // calculate control data
  const auto control_data = getControlData(input_data, current_time);

  // update control state
  updateControlState(control_data);

  // calculate control command
  const Motion ctrl_cmd = calcCtrlCmd(control_data);

  // create control command
  const auto cmd_msg = createCtrlCmdMsg(ctrl_cmd, current_time);
  trajectory_follower::LongitudinalOutput output;
  output.control_cmd = cmd_msg;

  // create control command horizon
  output.control_cmd_horizon.controls.push_back(cmd_msg);
  output.control_cmd_horizon.time_step_ms = 0.0;

  // update debug data
  setDebugValues(ctrl_cmd, control_data);

  PidLongitudinalControllerResult result;
  result.output = output;
  result.control_state = m_control_state;

  result.debug_message.stamp = current_time;
  for (const auto & v : m_debug_values.getValues()) {
    result.debug_message.data.push_back(
      static_cast<decltype(result.debug_message.data)::value_type>(v));
  }

  result.slope_message.stamp = current_time;
  result.slope_message.data.push_back(
    static_cast<decltype(result.slope_message.data)::value_type>(control_data.slope_angle));

  if (m_virtual_wall_marker) {
    result.virtual_wall_marker = m_virtual_wall_marker;
    m_virtual_wall_marker.reset();
  }
  result.received_invalid_trajectory = m_received_invalid_trajectory;
  result.emergency_stop_reason = m_emergency_stop_reason;

  return result;
}

PidLongitudinalController::ControlData PidLongitudinalController::getControlData(
  const trajectory_follower::InputData & input_data, const rclcpp::Time & current_time)
{
  ControlData control_data{};
  control_data.current_time = current_time;

  const geometry_msgs::msg::Pose & current_pose = input_data.current_odometry.pose.pose;

  // update trajectory if valid
  if (is_valid_trajectory(input_data.current_trajectory, config.use_temporal_trajectory)) {
    m_last_valid_trajectory = input_data.current_trajectory;
  }

  // dt
  control_data.dt = getDt(current_time);

  // current velocity and acceleration
  control_data.current_motion.vel = input_data.current_odometry.twist.twist.linear.x;
  control_data.current_motion.acc = input_data.current_accel.accel.accel.linear.x;
  control_data.current_pose = current_pose;
  control_data.operation_mode = input_data.current_operation_mode;
  control_data.interpolated_traj = m_last_valid_trajectory;
  if (control_data.interpolated_traj.points.size() < 2) {
    return control_data;
  }
  const double traj_start_time =
    rclcpp::Duration(control_data.interpolated_traj.points.front().time_from_start).seconds();
  const double traj_end_time =
    rclcpp::Duration(control_data.interpolated_traj.points.back().time_from_start).seconds();

  autoware_planning_msgs::msg::TrajectoryPoint nearest_point;
  autoware_planning_msgs::msg::TrajectoryPoint target_point;

  if (config.use_temporal_trajectory) {
    const rclcpp::Time traj_stamp(m_last_valid_trajectory.header.stamp);
    const double elapsed_time = (current_time - traj_stamp).seconds();
    const double nearest_time = std::clamp(elapsed_time, traj_start_time, traj_end_time);
    control_data.temporal_predicted_time = nearest_time;
    control_data.temporal_fused_time = nearest_time;
    m_prev_nearest_time = nearest_time;

    const auto nearest_interpolated_point = longitudinal_utils::lerpTrajectoryPointByTime(
      control_data.interpolated_traj.points, nearest_time);
    control_data.nearest_idx = nearest_interpolated_point.second + 1;
    control_data.target_idx = control_data.nearest_idx;
    control_data.interpolated_traj.points.insert(
      control_data.interpolated_traj.points.begin() + control_data.nearest_idx,
      nearest_interpolated_point.first);
    nearest_point = nearest_interpolated_point.first;
    target_point = nearest_interpolated_point.first;
  } else {
    // Calculate the interpolated nearest point from geometric projection.
    const auto current_interpolated_pose =
      calcInterpolatedTrajPointAndSegment(control_data.interpolated_traj, current_pose);

    // Insert the interpolated point
    control_data.interpolated_traj.points.insert(
      control_data.interpolated_traj.points.begin() + current_interpolated_pose.second + 1,
      current_interpolated_pose.first);
    control_data.nearest_idx = current_interpolated_pose.second + 1;
    control_data.target_idx = control_data.nearest_idx;
    nearest_point = current_interpolated_pose.first;
    target_point = current_interpolated_pose.first;
  }

  // Delay compensation - Calculate the distance we got, predicted velocity and predicted
  // acceleration after delay
  control_data.state_after_delay =
    predictedStateAfterDelay(control_data, config.delay_compensation_time);

  // calculate the target motion for delay compensation
  constexpr double min_running_dist = 0.01;
  if (config.use_temporal_trajectory) {
    const double nearest_time = rclcpp::Duration(nearest_point.time_from_start).seconds();
    const double target_time = nearest_time + config.delay_compensation_time;
    const auto target_interpolated_point = longitudinal_utils::lerpTrajectoryPointByTime(
      control_data.interpolated_traj.points, target_time);
    control_data.target_idx = target_interpolated_point.second + 1;
    control_data.interpolated_traj.points.insert(
      control_data.interpolated_traj.points.begin() + control_data.target_idx,
      target_interpolated_point.first);
    target_point = target_interpolated_point.first;
  } else if (control_data.state_after_delay.running_distance > min_running_dist) {
    control_data.interpolated_traj.points =
      autoware::motion_utils::removeOverlapPoints(control_data.interpolated_traj.points);
    const auto target_pose = longitudinal_utils::findTrajectoryPoseAfterDistance(
      control_data.nearest_idx, control_data.state_after_delay.running_distance,
      control_data.interpolated_traj);
    const auto target_interpolated_point =
      calcInterpolatedTrajPointAndSegment(control_data.interpolated_traj, target_pose);
    control_data.target_idx = target_interpolated_point.second + 1;
    control_data.interpolated_traj.points.insert(
      control_data.interpolated_traj.points.begin() + control_data.target_idx,
      target_interpolated_point.first);
    target_point = target_interpolated_point.first;
  }

  // ==========================================================================================
  // NOTE (spatial path): removeOverlapPoints() may change point indices, so previously computed
  // nearest/target indices can become invalid and must be re-acquired after de-duplication.
  // Temporal path intentionally skips removeOverlapPoints() to preserve same-position points with
  // different timestamps.
  // ==========================================================================================
  // Spatial-only de-duplication and index re-acquisition after inserting interpolated points.
  if (!config.use_temporal_trajectory) {
    m_prev_nearest_time.reset();
    control_data.interpolated_traj.points =
      autoware::motion_utils::removeOverlapPoints(control_data.interpolated_traj.points);
    control_data.nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      control_data.interpolated_traj.points, nearest_point.pose, config.ego_nearest_dist_threshold,
      config.ego_nearest_yaw_threshold);
    control_data.target_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      control_data.interpolated_traj.points, target_point.pose, config.ego_nearest_dist_threshold,
      config.ego_nearest_yaw_threshold);
  }

  // send debug values
  m_debug_values.setValues(DebugValues::TYPE::PREDICTED_VEL, control_data.state_after_delay.vel);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_VEL,
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps);

  // shift
  control_data.shift = getCurrentShift(control_data);
  if (control_data.shift != m_prev_shift) {
    m_pid_vel.reset();
  }
  m_prev_shift = control_data.shift;

  // distance to stopline
  control_data.stop_dist = longitudinal_utils::calcStopDistance(
    current_pose, control_data.interpolated_traj, config.ego_nearest_dist_threshold,
    config.ego_nearest_yaw_threshold);

  // pitch
  // NOTE: getPitchByTraj() calculates the pitch angle as defined in
  // ../media/slope_definition.drawio.svg while getPitchByPose() is not, so `raw_pitch` is reversed
  const double raw_pitch = (-1.0) * longitudinal_utils::getPitchByPose(current_pose.orientation);
  m_lpf_pitch->filter(raw_pitch);
  const double traj_pitch = longitudinal_utils::getPitchByTraj(
    control_data.interpolated_traj, control_data.target_idx, config.wheel_base);

  if (config.slope_source == PidLongitudinalControllerConfig::SlopeSource::RAW_PITCH) {
    control_data.slope_angle = m_lpf_pitch->getValue();
  } else if (
    config.slope_source == PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_PITCH) {
    control_data.slope_angle = traj_pitch;
  } else if (
    config.slope_source == PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_ADAPTIVE ||
    config.slope_source == PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_GOAL_ADAPTIVE) {
    // if velocity is high, use target idx for slope, otherwise, use raw_pitch
    const bool is_vel_slow =
      control_data.current_motion.vel < config.adaptive_trajectory_velocity_th &&
      config.slope_source == PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_ADAPTIVE;

    const double goal_dist = autoware::motion_utils::calcSignedArcLength(
      control_data.interpolated_traj.points, current_pose.position,
      control_data.interpolated_traj.points.size() - 1);
    const bool is_close_to_trajectory_end =
      goal_dist < config.wheel_base &&
      config.slope_source == PidLongitudinalControllerConfig::SlopeSource::TRAJECTORY_GOAL_ADAPTIVE;

    control_data.slope_angle =
      (is_close_to_trajectory_end || is_vel_slow) ? m_lpf_pitch->getValue() : traj_pitch;

    if (m_previous_slope_angle.has_value()) {
      constexpr double gravity_const = 9.8;
      control_data.slope_angle = std::clamp(
        control_data.slope_angle,
        m_previous_slope_angle.value() + config.min_jerk * control_data.dt / gravity_const,
        m_previous_slope_angle.value() + config.max_jerk * control_data.dt / gravity_const);
    }
    m_previous_slope_angle = control_data.slope_angle;
  } else {
    // config.slope_source is validated when the config is created, so this branch is unreachable.
  }

  updatePitchDebugValues(control_data.slope_angle, traj_pitch, raw_pitch, m_lpf_pitch->getValue());

  return control_data;
}

PidLongitudinalController::Motion PidLongitudinalController::calcEmergencyCtrlCmd(
  const ControlData & control_data)
{
  const double dt = control_data.dt;

  // These accelerations are without slope compensation
  const auto & p = config.emergency_state_params;
  Motion raw_ctrl_cmd{p.vel, p.acc};

  raw_ctrl_cmd.vel =
    longitudinal_utils::applyDiffLimitFilter(raw_ctrl_cmd.vel, m_prev_raw_ctrl_cmd.vel, dt, p.acc);
  raw_ctrl_cmd.acc = std::clamp(raw_ctrl_cmd.acc, config.min_acc, config.max_acc);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, raw_ctrl_cmd.acc);
  raw_ctrl_cmd.acc =
    longitudinal_utils::applyDiffLimitFilter(raw_ctrl_cmd.acc, m_prev_raw_ctrl_cmd.acc, dt, p.jerk);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, raw_ctrl_cmd.acc);

  m_virtual_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
    control_data.current_pose, "velocity control\n (emergency)", control_data.current_time, 0,
    config.wheel_base + config.front_overhang);

  return raw_ctrl_cmd;
}

void PidLongitudinalController::changeControlState(
  const ControlState & control_state, const std::string & reason)
{
  if (control_state != m_control_state) {
    if (control_state == ControlState::EMERGENCY) {
      m_emergency_stop_reason = reason;
    }
  }
  m_control_state = control_state;
}

void PidLongitudinalController::updateControlState(const ControlData & control_data)
{
  const double current_vel = control_data.current_motion.vel;
  const double stop_dist = control_data.stop_dist;

  // flags for state transition
  const auto & p = config.state_transition_params;

  const bool departure_condition_from_stopping =
    stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist;
  const bool departure_condition_from_stopped = stop_dist > p.drive_state_stop_dist;

  // NOTE: the same velocity threshold as autoware::motion_utils::searchZeroVelocity
  static constexpr double vel_epsilon = 1e-3;

  const bool stopping_condition = stop_dist < p.stopping_state_stop_dist;

  const bool is_stopped = std::abs(current_vel) < p.stopped_state_entry_vel;

  // Case where the ego slips in the opposite direction of the gear due to e.g. a slope is also
  // considered as a stop
  const bool is_not_running = [&]() {
    if (control_data.shift == Shift::Forward) {
      if (is_stopped || current_vel < 0.0) {
        // NOTE: Stopped or moving backward
        return true;
      }
    } else {
      if (is_stopped || 0.0 < current_vel) {
        // NOTE: Stopped or moving forward
        return true;
      }
    }
    return false;
  }();
  if (!is_not_running || !m_last_running_time) {
    m_last_running_time = std::make_shared<rclcpp::Time>(control_data.current_time);
  }
  const bool stopped_condition = (control_data.current_time - *m_last_running_time).seconds() >
                                 p.stopped_state_entry_duration_time;

  // ==========================================================================================
  // NOTE: due to removeOverlapPoints() in getControlData() m_last_valid_trajectory and
  // control_data.interpolated_traj have different size.
  // ==========================================================================================
  const double current_vel_cmd = std::fabs(
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps);
  const auto emergency_condition = [&]() {
    if (
      config.enable_overshoot_emergency && stop_dist < -p.emergency_state_overshoot_stop_dist &&
      current_vel_cmd < vel_epsilon) {
      return ResultWithReason{
        true, fmt::format("the target velocity {} is less than {}", current_vel_cmd, vel_epsilon)};
    }
    return ResultWithReason{false};
  }();
  const bool is_under_control = control_data.operation_mode.is_autoware_control_enabled &&
                                control_data.operation_mode.mode == OperationModeState::AUTONOMOUS;

  if (is_under_control != m_prev_vehicle_is_under_control) {
    m_prev_vehicle_is_under_control = is_under_control;
    m_under_control_starting_time =
      is_under_control ? std::make_shared<rclcpp::Time>(control_data.current_time) : nullptr;
  }

  if (m_control_state != ControlState::STOPPED) {
    m_prev_keep_stopped_condition = std::nullopt;
  }

  // transit state
  // in DRIVE state
  if (m_control_state == ControlState::DRIVE) {
    if (emergency_condition.result) {
      return changeControlState(ControlState::EMERGENCY, emergency_condition.reason);
    }
    if (!is_under_control && stopped_condition) {
      return changeControlState(ControlState::STOPPED);
    }

    if (config.enable_smooth_stop) {
      if (stopping_condition) {
        // predictions after input time delay
        const double pred_vel_in_target = control_data.state_after_delay.vel;
        const double pred_stop_dist =
          control_data.stop_dist -
          0.5 * (pred_vel_in_target + current_vel) * config.delay_compensation_time;
        m_smooth_stop->init(pred_vel_in_target, pred_stop_dist, control_data.current_time);
        return changeControlState(ControlState::STOPPING);
      }
    } else {
      if (stopped_condition && !departure_condition_from_stopped) {
        return changeControlState(ControlState::STOPPED);
      }
    }
    return;
  }

  // in STOPPING state
  if (m_control_state == ControlState::STOPPING) {
    if (emergency_condition.result) {
      return changeControlState(ControlState::EMERGENCY, emergency_condition.reason);
    }
    if (stopped_condition) {
      return changeControlState(ControlState::STOPPED);
    }

    if (departure_condition_from_stopping) {
      m_pid_vel.reset();
      m_lpf_vel_error->reset(0.0);
      // prevent the car from taking a long time to start to move
      m_prev_raw_ctrl_cmd.acc = std::max(0.0, m_prev_raw_ctrl_cmd.acc);
      return changeControlState(ControlState::DRIVE);
    }
    return;
  }

  // in STOPPED state
  if (m_control_state == ControlState::STOPPED) {
    // keep STOPPED if is_under_control is false
    if (!is_under_control && stopped_condition) return;

    if (departure_condition_from_stopped) {
      // Let vehicle start after the steering is converged for dry steering
      const bool current_keep_stopped_condition =
        std::fabs(current_vel) < vel_epsilon && !m_is_steer_converged;
      // NOTE: Dry steering is considered unnecessary when the steering is converged twice in a
      //       row. This is because m_is_steer_converged is not the current but
      //       the previous value due to the order controllers' run and sync functions.
      const bool keep_stopped_condition =
        !m_prev_keep_stopped_condition ||
        (current_keep_stopped_condition || *m_prev_keep_stopped_condition);
      m_prev_keep_stopped_condition = current_keep_stopped_condition;
      if (config.enable_keep_stopped_until_steer_convergence && keep_stopped_condition) {
        // create debug marker
        if (is_under_control) {
          m_virtual_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
            control_data.current_pose, "velocity control\n(steering not converged)",
            control_data.current_time, 0, config.wheel_base + config.front_overhang);
        }

        // keep STOPPED
        return;
      }

      m_pid_vel.reset();
      m_lpf_vel_error->reset(0.0);
      m_lpf_acc_error->reset(0.0);
      return changeControlState(ControlState::DRIVE);
    }

    return;
  }

  // in EMERGENCY state
  if (m_control_state == ControlState::EMERGENCY) {
    if (stopped_condition) {
      return changeControlState(ControlState::STOPPED);
    }

    if (!emergency_condition.result) {
      if (!is_under_control) {
        // NOTE: On manual driving, no need stopping to exit the emergency.
        return changeControlState(ControlState::DRIVE);
      }
    }
    return;
  }

  // NOTE: unreachable. ControlState has only DRIVE, STOPPING, STOPPED, and EMERGENCY, and every
  // branch above returns.
}

PidLongitudinalController::Motion PidLongitudinalController::calcCtrlCmd(
  const ControlData & control_data)
{
  // store current velocity history
  m_smooth_stop->recordMotion(
    control_data.current_time, control_data.current_motion.vel, control_data.current_motion.acc,
    config.delay_compensation_time);

  const size_t target_idx = control_data.target_idx;

  // velocity and acceleration command
  Motion ctrl_cmd_as_pedal_pos{
    control_data.interpolated_traj.points.at(target_idx).longitudinal_velocity_mps,
    control_data.interpolated_traj.points.at(target_idx).acceleration_mps2};

  if (m_control_state == ControlState::STOPPED) {
    const auto & p = config.stopped_state_params;
    ctrl_cmd_as_pedal_pos.vel = p.vel;
    ctrl_cmd_as_pedal_pos.acc = p.acc;

    m_prev_raw_ctrl_cmd.vel = 0.0;
    m_prev_raw_ctrl_cmd.acc = 0.0;

    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, ctrl_cmd_as_pedal_pos.acc);
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, ctrl_cmd_as_pedal_pos.acc);

    if (config.enable_slope_compensation) {
      const double pitch_limited =
        std::clamp(control_data.slope_angle, config.min_pitch_rad, config.max_pitch_rad);
      ctrl_cmd_as_pedal_pos.acc -= 9.81 * std::sin(std::abs(pitch_limited));
    }
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_SLOPE_APPLIED, ctrl_cmd_as_pedal_pos.acc);
  } else {
    Motion raw_ctrl_cmd{
      control_data.interpolated_traj.points.at(target_idx).longitudinal_velocity_mps,
      control_data.interpolated_traj.points.at(target_idx).acceleration_mps2};
    if (m_control_state == ControlState::EMERGENCY) {
      raw_ctrl_cmd = calcEmergencyCtrlCmd(control_data);
    } else {
      if (m_control_state == ControlState::DRIVE) {
        raw_ctrl_cmd.vel = control_data.interpolated_traj.points.at(control_data.target_idx)
                             .longitudinal_velocity_mps;
        raw_ctrl_cmd.acc = applyVelocityFeedback(control_data);
        raw_ctrl_cmd = keepBrakeBeforeStop(control_data, raw_ctrl_cmd, target_idx);
      } else if (m_control_state == ControlState::STOPPING) {
        const auto smooth_stop_result =
          m_smooth_stop->calculate(control_data.stop_dist, config.delay_compensation_time);
        raw_ctrl_cmd.acc = smooth_stop_result.acc;
        raw_ctrl_cmd.vel = config.stopped_state_params.vel;
        m_debug_values.setValues(
          DebugValues::TYPE::SMOOTH_STOP_MODE, static_cast<int>(smooth_stop_result.mode));
      }
      raw_ctrl_cmd.acc = std::clamp(raw_ctrl_cmd.acc, config.min_acc, config.max_acc);
      m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, raw_ctrl_cmd.acc);
      raw_ctrl_cmd.acc = longitudinal_utils::applyDiffLimitFilter(
        raw_ctrl_cmd.acc, m_prev_raw_ctrl_cmd.acc, control_data.dt, config.max_jerk,
        config.min_jerk);
      m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, raw_ctrl_cmd.acc);
    }

    // store acceleration without slope compensation
    m_prev_raw_ctrl_cmd = raw_ctrl_cmd;

    // calc acc feedback
    const double vel_sign = (control_data.shift == Shift::Forward)
                              ? 1.0
                              : (control_data.shift == Shift::Reverse ? -1.0 : 0.0);
    const double acc_err = control_data.current_motion.acc * vel_sign - raw_ctrl_cmd.acc;
    m_debug_values.setValues(DebugValues::TYPE::ERROR_ACC, acc_err);
    m_lpf_acc_error->filter(acc_err);
    m_debug_values.setValues(DebugValues::TYPE::ERROR_ACC_FILTERED, m_lpf_acc_error->getValue());

    const double acc_cmd =
      raw_ctrl_cmd.acc - m_lpf_acc_error->getValue() * config.acc_feedback_gain;
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_ACC_FB_APPLIED, acc_cmd);

    ctrl_cmd_as_pedal_pos.acc =
      applySlopeCompensation(acc_cmd, control_data.slope_angle, control_data.shift);
    m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_SLOPE_APPLIED, ctrl_cmd_as_pedal_pos.acc);
    ctrl_cmd_as_pedal_pos.vel = raw_ctrl_cmd.vel;
  }

  storeAccelCmd(m_prev_raw_ctrl_cmd.acc, control_data.current_time);

  ctrl_cmd_as_pedal_pos.acc = longitudinal_utils::applyDiffLimitFilter(
    ctrl_cmd_as_pedal_pos.acc, m_prev_ctrl_cmd.acc, control_data.dt, config.max_acc_cmd_diff);

  // update debug visualization
  updateDebugVelAcc(control_data);

  return ctrl_cmd_as_pedal_pos;
}

// Do not use nearest_idx here
autoware_control_msgs::msg::Longitudinal PidLongitudinalController::createCtrlCmdMsg(
  const Motion & ctrl_cmd, const rclcpp::Time & current_time)
{
  autoware_control_msgs::msg::Longitudinal cmd{};
  cmd.stamp = current_time;
  cmd.velocity = static_cast<decltype(cmd.velocity)>(ctrl_cmd.vel);
  cmd.acceleration = static_cast<decltype(cmd.acceleration)>(ctrl_cmd.acc);

  m_prev_ctrl_cmd = ctrl_cmd;

  return cmd;
}

void PidLongitudinalController::setDebugValues(
  const Motion & ctrl_cmd, const ControlData & control_data)
{
  // set debug values
  m_debug_values.setValues(DebugValues::TYPE::DT, control_data.dt);
  m_debug_values.setValues(DebugValues::TYPE::CALCULATED_ACC, control_data.current_motion.acc);
  m_debug_values.setValues(DebugValues::TYPE::SHIFT, static_cast<double>(control_data.shift));
  m_debug_values.setValues(DebugValues::TYPE::STOP_DIST, control_data.stop_dist);
  m_debug_values.setValues(DebugValues::TYPE::CONTROL_STATE, static_cast<double>(m_control_state));
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_PUBLISHED, ctrl_cmd.acc);
  m_debug_values.setValues(
    DebugValues::TYPE::TEMPORAL_PREDICTED_TIME, control_data.temporal_predicted_time);
  m_debug_values.setValues(
    DebugValues::TYPE::TEMPORAL_OBSERVED_TIME, std::numeric_limits<double>::quiet_NaN());
  m_debug_values.setValues(
    DebugValues::TYPE::TEMPORAL_FUSED_TIME, control_data.temporal_fused_time);
  m_debug_values.setValues(DebugValues::TYPE::TEMPORAL_OBSERVATION_USED, 0.0);
  m_debug_values.setValues(
    DebugValues::TYPE::TEMPORAL_WINDOW_MIN, std::numeric_limits<double>::quiet_NaN());
  m_debug_values.setValues(
    DebugValues::TYPE::TEMPORAL_WINDOW_MAX, std::numeric_limits<double>::quiet_NaN());
}

double PidLongitudinalController::getDt(const rclcpp::Time & current_time)
{
  double dt;
  if (!m_prev_control_time) {
    dt = config.longitudinal_ctrl_period;
    m_prev_control_time = std::make_shared<rclcpp::Time>(current_time);
  } else {
    dt = (current_time - *m_prev_control_time).seconds();
    *m_prev_control_time = current_time;
  }
  const double max_dt = config.longitudinal_ctrl_period * 2.0;
  const double min_dt = config.longitudinal_ctrl_period * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

enum PidLongitudinalController::Shift PidLongitudinalController::getCurrentShift(
  const ControlData & control_data) const
{
  constexpr double epsilon = 1e-5;

  const double target_vel =
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps;

  if (target_vel > epsilon) {
    return Shift::Forward;
  } else if (target_vel < -epsilon) {
    return Shift::Reverse;
  }

  return m_prev_shift;
}

void PidLongitudinalController::storeAccelCmd(const double accel, const rclcpp::Time & current_time)
{
  if (m_control_state == ControlState::DRIVE) {
    // convert format
    autoware_control_msgs::msg::Longitudinal cmd;
    cmd.stamp = current_time;
    cmd.acceleration = static_cast<decltype(cmd.acceleration)>(accel);

    // store published ctrl cmd
    m_ctrl_cmd_vec.emplace_back(cmd);
  } else {
    // reset command
    m_ctrl_cmd_vec.clear();
  }

  // remove unused ctrl cmd
  if (m_ctrl_cmd_vec.size() <= 2) {
    return;
  }
  if ((current_time - m_ctrl_cmd_vec.at(1).stamp).seconds() > config.delay_compensation_time) {
    m_ctrl_cmd_vec.erase(m_ctrl_cmd_vec.begin());
  }
}

double PidLongitudinalController::applySlopeCompensation(
  const double input_acc, const double pitch, const Shift shift) const
{
  if (!config.enable_slope_compensation) {
    return input_acc;
  }
  const double pitch_limited =
    std::min(std::max(pitch, config.min_pitch_rad), config.max_pitch_rad);

  // Acceleration command is always positive independent of direction (= shift) when car is running
  double sign = (shift == Shift::Forward) ? 1.0 : (shift == Shift::Reverse ? -1.0 : 0);
  double compensated_acc = input_acc + sign * 9.81 * std::sin(pitch_limited);
  return compensated_acc;
}

PidLongitudinalController::Motion PidLongitudinalController::keepBrakeBeforeStop(
  const ControlData & control_data, const Motion & target_motion, const size_t nearest_idx) const
{
  Motion output_motion = target_motion;

  if (config.enable_brake_keeping_before_stop == false) {
    return output_motion;
  }
  const auto traj = control_data.interpolated_traj;

  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(traj.points);
  if (!stop_idx) {
    return output_motion;
  }

  double min_acc_before_stop = std::numeric_limits<double>::max();
  size_t min_acc_idx = std::numeric_limits<size_t>::max();
  for (int i = static_cast<int>(*stop_idx); i >= 0; --i) {
    const auto ui = static_cast<size_t>(i);
    if (traj.points.at(ui).acceleration_mps2 > static_cast<float>(min_acc_before_stop)) {
      break;
    }
    min_acc_before_stop = traj.points.at(ui).acceleration_mps2;
    min_acc_idx = ui;
  }

  const double brake_keeping_acc = std::max(config.brake_keeping_acc, min_acc_before_stop);
  if (nearest_idx >= min_acc_idx && target_motion.acc > brake_keeping_acc) {
    output_motion.acc = brake_keeping_acc;
  }

  return output_motion;
}

std::pair<autoware_planning_msgs::msg::TrajectoryPoint, size_t>
PidLongitudinalController::calcInterpolatedTrajPointAndSegment(
  const autoware_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & pose) const
{
  if (traj.points.size() == 1) {
    return std::make_pair(traj.points.at(0), 0);
  }

  // apply linear interpolation
  return longitudinal_utils::lerpTrajectoryPoint(
    traj.points, pose, config.ego_nearest_dist_threshold, config.ego_nearest_yaw_threshold);
}

PidLongitudinalController::StateAfterDelay PidLongitudinalController::predictedStateAfterDelay(
  const ControlData & control_data, const double delay_compensation_time) const
{
  const double current_vel = control_data.current_motion.vel;
  const double current_acc = control_data.current_motion.acc;
  double running_distance = 0.0;
  double pred_vel = current_vel;
  double pred_acc = current_acc;

  if (
    m_ctrl_cmd_vec.empty() || control_data.operation_mode.mode != OperationModeState::AUTONOMOUS) {
    // check time to stop
    const double time_to_stop = -current_vel / current_acc;
    const double delay_time_calculation =
      time_to_stop > 0.0 && time_to_stop < delay_compensation_time ? time_to_stop
                                                                   : delay_compensation_time;
    // simple linear prediction
    pred_vel = current_vel + current_acc * delay_time_calculation;
    running_distance = std::abs(
      delay_time_calculation * current_vel +
      0.5 * current_acc * delay_time_calculation * delay_time_calculation);
    // avoid to change sign of current_vel and pred_vel
    return StateAfterDelay{pred_vel, pred_acc, running_distance};
  }

  for (std::size_t i = 0; i < m_ctrl_cmd_vec.size(); ++i) {
    if (
      (control_data.current_time - m_ctrl_cmd_vec.at(i).stamp).seconds() <
      delay_compensation_time) {
      // add velocity to accel * dt
      const double time_to_next_acc =
        (i == m_ctrl_cmd_vec.size() - 1)
          ? std::min(
              (control_data.current_time - m_ctrl_cmd_vec.back().stamp).seconds(),
              delay_compensation_time)
          : std::min(
              (rclcpp::Time(m_ctrl_cmd_vec.at(i + 1).stamp) -
               rclcpp::Time(m_ctrl_cmd_vec.at(i).stamp))
                .seconds(),
              delay_compensation_time);
      const double acc = m_ctrl_cmd_vec.at(i).acceleration;
      // because acc_cmd is positive when vehicle is running backward
      pred_acc = std::copysignf(1.0, static_cast<float>(pred_vel)) * acc;
      running_distance += std::abs(
        std::abs(pred_vel) * time_to_next_acc + 0.5 * acc * time_to_next_acc * time_to_next_acc);
      pred_vel += pred_vel < 0.0 ? (-acc * time_to_next_acc) : (acc * time_to_next_acc);
      if (pred_vel / current_vel < 0.0) {
        // sign of velocity is changed
        pred_vel = 0.0;
        break;
      }
    }
  }

  return StateAfterDelay{pred_vel, pred_acc, running_distance};
}

double PidLongitudinalController::applyVelocityFeedback(const ControlData & control_data)
{
  // NOTE: Acceleration command is always positive even if the ego drives backward.
  const double vel_sign = (control_data.shift == Shift::Forward)
                            ? 1.0
                            : (control_data.shift == Shift::Reverse ? -1.0 : 0.0);
  const double current_vel = control_data.current_motion.vel;
  const auto target_motion = Motion{
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps,
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2};
  const double diff_vel = (target_motion.vel - current_vel) * vel_sign;
  const bool is_under_control = control_data.operation_mode.is_autoware_control_enabled &&
                                control_data.operation_mode.mode == OperationModeState::AUTONOMOUS;

  const bool vehicle_is_moving = std::abs(current_vel) > config.current_vel_threshold_pid_integrate;
  const double time_under_control = getTimeUnderControl(control_data.current_time);
  const bool vehicle_is_stuck =
    !vehicle_is_moving && time_under_control > config.time_threshold_before_pid_integrate;

  const bool enable_integration =
    (vehicle_is_moving || (config.enable_integration_at_low_speed && vehicle_is_stuck)) &&
    is_under_control;

  const double error_vel_filtered = m_lpf_vel_error->filter(diff_vel);

  std::vector<double> pid_contributions(3);
  const double pid_acc =
    m_pid_vel.calculate(error_vel_filtered, control_data.dt, enable_integration, pid_contributions);

  // Feedforward scaling:
  // This is for the coordinate conversion where feedforward is applied, from Time to Arclength.
  // Details: For accurate control, the feedforward should be calculated in the arclength coordinate
  // system, not in the time coordinate system. Otherwise, even if FF is applied, the vehicle speed
  // deviation will be bigger.
  const double ff_scale = std::clamp(
    std::abs(current_vel) / std::max(std::abs(target_motion.vel), 0.1), config.ff_scale_min,
    config.ff_scale_max);
  const double ff_acc =
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2 * ff_scale;

  const double feedback_acc = ff_acc + pid_acc;

  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_PID_APPLIED, feedback_acc);
  m_debug_values.setValues(DebugValues::TYPE::ERROR_VEL_FILTERED, error_vel_filtered);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FB_P_CONTRIBUTION, pid_contributions.at(0));
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FB_I_CONTRIBUTION, pid_contributions.at(1));
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FB_D_CONTRIBUTION, pid_contributions.at(2));
  m_debug_values.setValues(DebugValues::TYPE::FF_SCALE, ff_scale);
  m_debug_values.setValues(DebugValues::TYPE::ACC_CMD_FF, ff_acc);

  return feedback_acc;
}

void PidLongitudinalController::updatePitchDebugValues(
  const double pitch_using, const double traj_pitch, const double localization_pitch,
  const double localization_pitch_lpf)
{
  const double to_degrees = (180.0 / static_cast<double>(M_PI));
  m_debug_values.setValues(DebugValues::TYPE::PITCH_USING_RAD, pitch_using);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_USING_DEG, pitch_using * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_LPF_RAD, localization_pitch_lpf);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_LPF_DEG, localization_pitch_lpf * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_RAD, localization_pitch);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_DEG, localization_pitch * to_degrees);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_RAD, traj_pitch);
  m_debug_values.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_DEG, traj_pitch * to_degrees);
}

void PidLongitudinalController::updateDebugVelAcc(const ControlData & control_data)
{
  m_debug_values.setValues(DebugValues::TYPE::CURRENT_VEL, control_data.current_motion.vel);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_VEL,
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps);
  m_debug_values.setValues(
    DebugValues::TYPE::TARGET_ACC,
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2);
  m_debug_values.setValues(
    DebugValues::TYPE::NEAREST_VEL,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps);
  m_debug_values.setValues(
    DebugValues::TYPE::NEAREST_ACC,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).acceleration_mps2);
  m_debug_values.setValues(
    DebugValues::TYPE::ERROR_VEL,
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps -
      control_data.current_motion.vel);
}

double PidLongitudinalController::getTimeUnderControl(const rclcpp::Time & current_time)
{
  if (!m_under_control_starting_time) return 0.0;
  return (current_time - *m_under_control_starting_time).seconds();
}

}  // namespace autoware::motion::control::pid_longitudinal_controller
