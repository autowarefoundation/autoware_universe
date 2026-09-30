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
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

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

std::shared_ptr<VehicleModelInterface> MpcLateralController::createVehicleModel(
  const double wheelbase, const double steer_lim, const double steer_tau, rclcpp::Node & node)
{
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr;

  const std::string vehicle_model_type = node.declare_parameter<std::string>("vehicle_model_type");

  if (vehicle_model_type == "kinematics") {
    vehicle_model_ptr = std::make_shared<KinematicsBicycleModel>(wheelbase, steer_lim, steer_tau);
    return vehicle_model_ptr;
  }

  if (vehicle_model_type == "kinematics_no_delay") {
    vehicle_model_ptr = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_lim);
    return vehicle_model_ptr;
  }

  if (vehicle_model_type == "dynamics") {
    const double mass_fl = node.declare_parameter<double>("vehicle.mass_fl");
    const double mass_fr = node.declare_parameter<double>("vehicle.mass_fr");
    const double mass_rl = node.declare_parameter<double>("vehicle.mass_rl");
    const double mass_rr = node.declare_parameter<double>("vehicle.mass_rr");
    const double cf = node.declare_parameter<double>("vehicle.cf");
    const double cr = node.declare_parameter<double>("vehicle.cr");

    // vehicle_model_ptr is only assigned in ctor, so parameter value have to be passed at init time
    vehicle_model_ptr = std::make_shared<DynamicsBicycleModel>(
      wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr, writer_);
    return vehicle_model_ptr;
  }

  AW_MPC_ERROR(writer_, "vehicle_model_type is undefined");
  return vehicle_model_ptr;
}

std::shared_ptr<QPSolverInterface> MpcLateralController::createQPSolverInterface(
  rclcpp::Node & node)
{
  std::shared_ptr<QPSolverInterface> qpsolver_ptr;

  const std::string qp_solver_type = node.declare_parameter<std::string>("qp_solver_type");

  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
    return qpsolver_ptr;
  }

  if (qp_solver_type == "osqp") {
    qpsolver_ptr = std::make_shared<QPSolverOSQP>();
    return qpsolver_ptr;
  }

  AW_MPC_ERROR(writer_, "qp_solver_type is undefined");
  return qpsolver_ptr;
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
  while (rclcpp::ok()) {
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

void MpcLateralController::setSteeringToHistory(const Lateral & steering)
{
  const auto time = clock_->now();
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
