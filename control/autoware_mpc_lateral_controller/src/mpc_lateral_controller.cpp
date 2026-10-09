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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

MpcLateralController::MpcLateralController(
  const MpcLateralControllerConfig & config, const LogWriter & writer)
: writer_(writer)
{
  m_mpc = std::make_unique<MPC>();
  m_mpc->setLogWriter(writer_);
  m_mpc->m_ctrl_period = config.ctrl_period;
  m_trajectory_filtering_param = config.trajectory_filtering;

  m_mpc->m_use_steer_prediction = config.use_steer_prediction;
  m_mpc->m_param = config.mpc_param;

  m_stop_state_entry_ego_speed = config.stop_state_entry_ego_speed;
  m_stop_state_entry_target_speed = config.stop_state_entry_target_speed;
  m_converged_steer_rad = config.converged_steer_rad;
  m_keep_steer_control_until_converged = config.keep_steer_control_until_converged;
  m_new_traj_duration_time = config.new_traj_duration_time;
  m_new_traj_end_dist = config.new_traj_end_dist;
  m_mpc_converged_threshold_rps = config.mpc_converged_threshold_rps;

  m_mpc->m_steer_lim = config.steer_lim;
  m_mpc->m_steer_rate_lim_map_by_curvature = config.steer_rate_lim_by_curvature;
  m_mpc->m_steer_rate_lim_map_by_velocity = config.steer_rate_lim_by_velocity;

  auto vehicle_model_ptr = createVehicleModel(config, writer_);
  if (!vehicle_model_ptr) {
    AW_MPC_ERROR(writer_, "vehicle_model_type is undefined");
  }
  m_mpc->setVehicleModel(vehicle_model_ptr);

  auto qpsolver_ptr = createQPSolverInterface(config);
  if (!qpsolver_ptr) {
    AW_MPC_ERROR(writer_, "qp_solver_type is undefined");
  }
  m_mpc->setQPSolver(qpsolver_ptr);

  /* delay compensation */
  {
    const double delay_step = std::round(config.input_delay / m_mpc->m_ctrl_period);
    m_mpc->m_param.input_delay = delay_step * m_mpc->m_ctrl_period;
    m_mpc->m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }

  /* steering offset compensation */
  enable_auto_steering_offset_removal_ = config.enable_auto_steering_offset_removal;
  m_steer_offset_max_update_th_ = config.steer_offset_max_update_th;
  if (enable_auto_steering_offset_removal_) {
    lpf_steer_offset_ = std::make_shared<Butterworth2dFilter>(
      config.ctrl_period, config.steer_offset_filter_cutoff_hz, m_steering_offset_);
  }

  m_mpc->initializeLowPassFilters(config.steering_lpf_cutoff_hz, config.error_deriv_lpf_cutoff_hz);

  m_ego_nearest_dist_threshold = config.ego_nearest_dist_threshold;
  m_ego_nearest_yaw_threshold = config.ego_nearest_yaw_threshold;
  m_mpc->ego_nearest_dist_threshold = m_ego_nearest_dist_threshold;
  m_mpc->ego_nearest_yaw_threshold = m_ego_nearest_yaw_threshold;

  m_mpc->m_use_delayed_initial_state = config.use_delayed_initial_state;
  m_mpc->m_use_temporal_trajectory = config.use_temporal_trajectory;
  m_mpc->m_publish_debug_trajectories = config.publish_debug_trajectories;

  m_mpc->initializeSteeringPredictor();
}

void MpcLateralController::setMpcParam(const MPCParam & param)
{
  m_mpc->m_param = param;

  // The buffer holds one command per control period of the delay, so it is built again only
  // when the given delay is not a whole number of control periods and has to be rounded.
  const double delay_step = std::round(param.input_delay / m_mpc->m_ctrl_period);
  const double delay = delay_step * m_mpc->m_ctrl_period;
  if (param.input_delay != delay) {
    m_mpc->m_param.input_delay = delay;
    m_mpc->m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }
}

MpcLateralController::~MpcLateralController()
{
}

std::shared_ptr<VehicleModelInterface> MpcLateralController::createVehicleModel(
  const MpcLateralControllerConfig & config, const LogWriter & writer)
{
  const double wheelbase = config.wheelbase;
  const double steer_lim = config.steer_lim;

  if (config.vehicle_model_type == "kinematics") {
    return std::make_shared<KinematicsBicycleModel>(
      wheelbase, steer_lim, config.mpc_param.steer_tau);
  }

  if (config.vehicle_model_type == "kinematics_no_delay") {
    return std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_lim);
  }

  if (config.vehicle_model_type == "dynamics") {
    return std::make_shared<DynamicsBicycleModel>(
      wheelbase, config.mass_fl, config.mass_fr, config.mass_rl, config.mass_rr, config.cf,
      config.cr, writer);
  }

  return nullptr;
}

std::shared_ptr<QPSolverInterface> MpcLateralController::createQPSolverInterface(
  const MpcLateralControllerConfig & config)
{
  if (config.qp_solver_type == "unconstraint_fast") {
    return std::make_shared<QPSolverEigenLeastSquareLLT>();
  }

  if (config.qp_solver_type == "osqp") {
    return std::make_shared<QPSolverOSQP>();
  }

  return nullptr;
}

MpcLateralControllerResult MpcLateralController::run(
  trajectory_follower::InputData const & input_data, const rclcpp::Time & stamp)
{
  MpcLateralControllerResult result;

  // set input data
  setTrajectory(input_data.current_trajectory, input_data.current_odometry);

  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;

  m_steering_offset_target_ = std::invoke([&]() {
    if (!enable_auto_steering_offset_removal_) return 0.0;
    if (abs(m_steering_offset_target_ - m_steering_offset_filtered_) > 1e-4)
      return m_steering_offset_target_;
    const double delta_offset = m_steering_offset_ - m_steering_offset_target_;
    const double delta_offset_clamped =
      std::clamp(delta_offset, -m_steer_offset_max_update_th_, m_steer_offset_max_update_th_);
    return m_steering_offset_target_ + delta_offset_clamped;
  });

  m_steering_offset_filtered_ = enable_auto_steering_offset_removal_
                                  ? lpf_steer_offset_->filter(m_steering_offset_target_)
                                  : 0.0;
  m_current_steering.steering_tire_angle += static_cast<float>(m_steering_offset_filtered_);

  const bool is_under_control = input_data.current_operation_mode.is_autoware_control_enabled &&
                                input_data.current_operation_mode.mode ==
                                  autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;

  if (!m_is_ctrl_cmd_prev_initialized || !is_under_control) {
    m_ctrl_cmd_prev = getInitialControlCommand();
    m_is_ctrl_cmd_prev_initialized = true;
  }

  auto mpc_solved_status =
    m_mpc->calculateMPC(m_current_steering, m_current_kinematic_state, stamp);
  Lateral ctrl_cmd = mpc_solved_status.ctrl_cmd;

  if (
    (m_mpc_solved_status.result == true && mpc_solved_status.result == false) ||
    (!mpc_solved_status.result && mpc_solved_status.reason != m_mpc_solved_status.reason)) {
    AW_MPC_ERROR(writer_, "MPC failed due to {}", mpc_solved_status.reason);
  }
  m_mpc_solved_status = mpc_solved_status;  // for diagnostic updater

  // reset previous MPC result
  // Note: When a large deviation from the trajectory occurs, the optimization stops and
  // the vehicle will return to the path by re-planning the trajectory or external operation.
  // After the recovery, the previous value of the optimization may deviate greatly from
  // the actual steer angle, and it may make the optimization result unstable.
  if (!mpc_solved_status.result || !is_under_control) {
    m_mpc->resetPrevResult(m_current_steering);
  } else {
    setSteeringToHistory(ctrl_cmd, stamp);
  }

  ctrl_cmd.steering_tire_angle -= static_cast<float>(m_steering_offset_filtered_);

  result.mpc = mpc_solved_status;
  result.steering_offset = m_steering_offset_filtered_;

  const auto createLateralOutput =
    [this, &stamp](
      const auto & cmd, const bool is_mpc_solved,
      const auto & cmd_horizon) -> trajectory_follower::LateralOutput {
    trajectory_follower::LateralOutput output;
    output.control_cmd = createCtrlCmdMsg(cmd, stamp);
    output.control_cmd_horizon = createCtrlCmdHorizonMsg(cmd_horizon, stamp);
    // To be sure current steering of the vehicle is desired steering angle, we need to check
    // following conditions.
    // 1. At the last loop, mpc should be solved because command should be optimized output.
    // 2. The mpc should be converged.
    // 3. The steer angle should be converged.
    output.sync_data.is_steer_converged =
      is_mpc_solved && isMpcConverged() && isSteerConverged(cmd);

    return output;
  };

  if (isStoppedState()) {
    // Reset input buffer
    AW_MPC_DEBUG_THROTTLE(writer_, 5.0, "Stopped state detected, use previous control command");
    for (auto & value : m_mpc->m_input_buffer) {
      value = m_ctrl_cmd_prev.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    m_mpc->m_raw_steer_cmd_prev = m_ctrl_cmd_prev.steering_tire_angle;
    result.output = createLateralOutput(m_ctrl_cmd_prev, false, mpc_solved_status.ctrl_cmd_horizon);
    return result;
  }

  if (!mpc_solved_status.result) {
    AW_MPC_DEBUG_THROTTLE(writer_, 5.0, "MPC is not solved, use stop control command");
    ctrl_cmd = getStopControlCommand();
  }

  m_ctrl_cmd_prev = ctrl_cmd;
  result.output =
    createLateralOutput(ctrl_cmd, mpc_solved_status.result, mpc_solved_status.ctrl_cmd_horizon);
  return result;
}

bool MpcLateralController::isSteerConverged(const Lateral & cmd) const
{
  // wait for a while to propagate the trajectory shape to the output command when the trajectory
  // shape is changed.
  if (!m_has_received_first_trajectory || isTrajectoryShapeChanged()) {
    AW_MPC_DEBUG(writer_, "trajectory shaped is changed");
    return false;
  }

  const bool is_converged =
    std::abs(cmd.steering_tire_angle - m_current_steering.steering_tire_angle) <
    static_cast<float>(m_converged_steer_rad);

  return is_converged;
}

bool MpcLateralController::isReady(const trajectory_follower::InputData & input_data)
{
  setTrajectory(input_data.current_trajectory, input_data.current_odometry);
  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;

  if (!m_mpc->hasVehicleModel()) {
    AW_MPC_INFO_THROTTLE(writer_, 5.0, "MPC does not have a vehicle model");
    return false;
  }
  if (!m_mpc->hasQPSolver()) {
    AW_MPC_INFO_THROTTLE(writer_, 5.0, "MPC does not have a QP solver");
    return false;
  }
  if (m_mpc->m_reference_trajectory.empty()) {
    AW_MPC_INFO_THROTTLE(writer_, 5.0, "trajectory size is zero.");
    return false;
  }

  return true;
}

void MpcLateralController::setTrajectory(
  const Trajectory & msg, const Odometry & current_kinematics)
{
  m_current_trajectory = msg;

  if (msg.points.size() < 3) {
    AW_MPC_DEBUG(writer_, "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(msg)) {
    AW_MPC_ERROR(writer_, "Trajectory is invalid!! stop computing.");
    return;
  }

  m_mpc->setReferenceTrajectory(msg, m_trajectory_filtering_param, current_kinematics);

  // update trajectory buffer to check the trajectory shape change.
  m_trajectory_buffer.push_back(m_current_trajectory);
  while (true) {
    const auto time_diff = rclcpp::Time(m_trajectory_buffer.back().header.stamp) -
                           rclcpp::Time(m_trajectory_buffer.front().header.stamp);

    const double first_trajectory_duration_time = 5.0;
    const double duration_time =
      m_has_received_first_trajectory ? m_new_traj_duration_time : first_trajectory_duration_time;
    if (time_diff.seconds() < duration_time) {
      m_has_received_first_trajectory = true;
      break;
    }
    m_trajectory_buffer.pop_front();
  }
}

Lateral MpcLateralController::getStopControlCommand() const
{
  Lateral cmd;
  cmd.steering_tire_angle = static_cast<decltype(cmd.steering_tire_angle)>(m_steer_cmd_prev);
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

Lateral MpcLateralController::getInitialControlCommand() const
{
  Lateral cmd;
  cmd.steering_tire_angle = m_current_steering.steering_tire_angle;
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

bool MpcLateralController::isStoppedState() const
{
  const double current_vel = m_current_kinematic_state.twist.twist.linear.x;
  // If the nearest index is not found, return false
  if (
    m_current_trajectory.points.empty() || std::fabs(current_vel) > m_stop_state_entry_ego_speed) {
    return false;
  }

  const auto latest_published_cmd = m_ctrl_cmd_prev;  // use prev_cmd as a latest published command
  if (m_keep_steer_control_until_converged && !isSteerConverged(latest_published_cmd)) {
    AW_MPC_DEBUG_THROTTLE(writer_, 5.0, "steering is not converged.");
    return false;  // not stopState: keep control
  }

  // Note: This function used to take into account the distance to the stop line
  // for the stop state judgement. However, it has been removed since the steering
  // control was turned off when approaching/exceeding the stop line on a curve or
  // emergency stop situation and it caused large tracking error.
  const size_t nearest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    m_current_trajectory.points, m_current_kinematic_state.pose.pose, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);

  // It is possible that stop is executed earlier than stop point, and velocity controller
  // will not start when the distance from ego to stop point is less than 0.5 meter.
  // So we use a distance margin to ensure we can detect stopped state.
  static constexpr double distance_margin = 1.0;
  const double target_vel = std::invoke([&]() -> double {
    auto min_vel = m_current_trajectory.points.at(nearest).longitudinal_velocity_mps;
    auto covered_distance = 0.0;
    for (auto i = nearest + 1; i < m_current_trajectory.points.size(); ++i) {
      min_vel = std::min(min_vel, m_current_trajectory.points.at(i).longitudinal_velocity_mps);
      covered_distance += autoware_utils::calc_distance2d(
        m_current_trajectory.points.at(i - 1).pose, m_current_trajectory.points.at(i).pose);
      if (covered_distance > distance_margin) break;
    }
    return min_vel;
  });

  return std::fabs(target_vel) < m_stop_state_entry_target_speed;
}

Lateral MpcLateralController::createCtrlCmdMsg(
  const Lateral & ctrl_cmd, const builtin_interfaces::msg::Time & stamp)
{
  auto out = ctrl_cmd;
  out.stamp = stamp;
  m_steer_cmd_prev = out.steering_tire_angle;
  return out;
}

LateralHorizon MpcLateralController::createCtrlCmdHorizonMsg(
  const LateralHorizon & ctrl_cmd_horizon, const builtin_interfaces::msg::Time & stamp) const
{
  auto out = ctrl_cmd_horizon;
  for (auto & cmd : out.controls) {
    cmd.stamp = stamp;
  }
  return out;
}

void MpcLateralController::setSteeringToHistory(
  const Lateral & steering, const rclcpp::Time & stamp)
{
  const auto time = stamp;
  if (m_mpc_steering_history.empty()) {
    m_mpc_steering_history.emplace_back(steering, time);
    m_is_mpc_history_filled = false;
    return;
  }

  m_mpc_steering_history.emplace_back(steering, time);

  // Check the history is filled or not.
  if (rclcpp::Duration(time - m_mpc_steering_history.begin()->second).seconds() >= 1.0) {
    m_is_mpc_history_filled = true;
    // remove old data that is older than 1 sec
    for (auto itr = m_mpc_steering_history.begin(); itr != m_mpc_steering_history.end(); ++itr) {
      if (rclcpp::Duration(time - itr->second).seconds() > 1.0) {
        m_mpc_steering_history.erase(m_mpc_steering_history.begin());
      } else {
        break;
      }
    }
  } else {
    m_is_mpc_history_filled = false;
  }
}

bool MpcLateralController::isMpcConverged()
{
  // If the number of variable below the 2, there is no enough data so MPC is not converged.
  if (m_mpc_steering_history.size() < 2) {
    return false;
  }

  // If the history is not filled, return false.

  if (!m_is_mpc_history_filled) {
    return false;
  }

  // Find the maximum and minimum values of the steering angle in the past 1 second.
  double min_steering_value = m_mpc_steering_history[0].first.steering_tire_angle;
  double max_steering_value = min_steering_value;
  for (size_t i = 1; i < m_mpc_steering_history.size(); i++) {
    if (m_mpc_steering_history.at(i).first.steering_tire_angle < min_steering_value) {
      min_steering_value = m_mpc_steering_history.at(i).first.steering_tire_angle;
    }
    if (m_mpc_steering_history.at(i).first.steering_tire_angle > max_steering_value) {
      max_steering_value = m_mpc_steering_history.at(i).first.steering_tire_angle;
    }
  }
  return (max_steering_value - min_steering_value) < m_mpc_converged_threshold_rps;
}

bool MpcLateralController::isTrajectoryShapeChanged() const
{
  // TODO(Horibe): update implementation to check trajectory shape around ego vehicle.
  // Now temporally check the goal position.
  for (const auto & trajectory : m_trajectory_buffer) {
    const auto change_distance = autoware_utils::calc_distance2d(
      trajectory.points.back().pose, m_current_trajectory.points.back().pose);
    if (change_distance > m_new_traj_end_dist) {
      return true;
    }
  }
  return false;
}

bool MpcLateralController::isValidTrajectory(const Trajectory & traj) const
{
  double prev_time_from_start = -std::numeric_limits<double>::infinity();
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.orientation.w) || !isfinite(p.pose.orientation.x) ||
      !isfinite(p.pose.orientation.y) || !isfinite(p.pose.orientation.z) ||
      !isfinite(p.longitudinal_velocity_mps) || !isfinite(p.lateral_velocity_mps) ||
      !isfinite(p.heading_rate_rps) || !isfinite(p.front_wheel_angle_rad) ||
      !isfinite(p.rear_wheel_angle_rad)) {
      return false;
    }

    if (m_mpc->m_use_temporal_trajectory) {
      const double t = rclcpp::Duration(p.time_from_start).seconds();
      if (!std::isfinite(t) || t <= prev_time_from_start) {
        return false;
      }
      prev_time_from_start = t;
    }
  }
  return true;
}

}  // namespace autoware::motion::control::mpc_lateral_controller
