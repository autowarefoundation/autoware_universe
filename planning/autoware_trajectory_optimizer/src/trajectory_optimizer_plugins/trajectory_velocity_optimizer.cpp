// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_velocity_optimizer.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

TrajectoryVelocityOptimizer::TrajectoryVelocityOptimizer(
  const std::string name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const TrajectoryOptimizerParams & params)
: TrajectoryOptimizerPluginBase(name, node_ptr, time_keeper, params)
{
  set_up_params();
  if (velocity_params_.smooth_velocities) {
    set_up_velocity_smoother(node_ptr, time_keeper);
  }
}

void TrajectoryVelocityOptimizer::set_up_velocity_smoother(
  rclcpp::Node * node_ptr, const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
{
  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();
  double wheelbase = vehicle_info.wheel_base_m;  // vehicle_info.wheel_base_m;
  jerk_filtered_smoother_ = std::make_shared<JerkFilteredSmoother>(*node_ptr, time_keeper);
  jerk_filtered_smoother_->setWheelBase(wheelbase);
}

void TrajectoryVelocityOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  if (!params.optimize_velocity) {
    return;
  }

  const auto & current_odometry = data.current_odometry;
  const auto & current_acceleration = data.current_acceleration;
  const auto & current_speed = current_odometry.twist.twist.linear.x;
  const auto & current_linear_acceleration = current_acceleration.accel.accel.linear.x;
  const double & target_pull_out_speed_mps = velocity_params_.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = velocity_params_.target_pull_out_acc_mps2;
  const double & max_speed_mps = velocity_params_.max_speed_mps;

  if (velocity_params_.limit_lateral_acceleration) {
    utils::limit_lateral_acceleration(
      traj_points, velocity_params_.max_lateral_accel_mps2, data.current_odometry);
  }

  auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
  auto initial_motion_acc = (current_speed > target_pull_out_speed_mps)
                              ? current_linear_acceleration
                              : target_pull_out_acc_mps2;

  if (velocity_params_.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    utils::clamp_velocities(
      traj_points, current_odometry, static_cast<float>(initial_motion_speed),
      static_cast<float>(initial_motion_acc));
  }

  if (velocity_params_.limit_speed) {
    utils::set_max_velocity(traj_points, current_odometry, static_cast<float>(max_speed_mps));
  }

  if (velocity_params_.smooth_velocities) {
    if (!jerk_filtered_smoother_) {
      set_up_velocity_smoother(get_node_ptr(), get_time_keeper());
    }
    InitialMotion initial_motion{initial_motion_speed, initial_motion_acc};
    utils::filter_velocity(
      traj_points, initial_motion, velocity_params_.nearest_dist_threshold_m,
      autoware_utils_math::deg2rad(velocity_params_.nearest_yaw_threshold_deg),
      jerk_filtered_smoother_, current_odometry);
  }
}

void TrajectoryVelocityOptimizer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  velocity_params_.nearest_dist_threshold_m = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.nearest_dist_threshold_m");
  velocity_params_.nearest_yaw_threshold_deg = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.nearest_yaw_threshold_deg");
  velocity_params_.target_pull_out_speed_mps = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.target_pull_out_speed_mps");
  velocity_params_.target_pull_out_acc_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.target_pull_out_acc_mps2");
  velocity_params_.max_speed_mps =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_velocity_optimizer.max_speed_mps");
  velocity_params_.max_lateral_accel_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.max_lateral_accel_mps2");
  velocity_params_.set_engage_speed =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.set_engage_speed");
  velocity_params_.limit_speed =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.limit_speed");
  velocity_params_.limit_lateral_acceleration = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_velocity_optimizer.limit_lateral_acceleration");
  velocity_params_.smooth_velocities =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.smooth_velocities");
}

rcl_interfaces::msg::SetParametersResult TrajectoryVelocityOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "trajectory_velocity_optimizer.nearest_dist_threshold_m",
    velocity_params_.nearest_dist_threshold_m);
  update_param(
    parameters, "trajectory_velocity_optimizer.nearest_yaw_threshold_deg",
    velocity_params_.nearest_yaw_threshold_deg);
  update_param(
    parameters, "trajectory_velocity_optimizer.target_pull_out_speed_mps",
    velocity_params_.target_pull_out_speed_mps);
  update_param(
    parameters, "trajectory_velocity_optimizer.target_pull_out_acc_mps2",
    velocity_params_.target_pull_out_acc_mps2);
  update_param(
    parameters, "trajectory_velocity_optimizer.max_speed_mps", velocity_params_.max_speed_mps);
  update_param(
    parameters, "trajectory_velocity_optimizer.max_lateral_accel_mps2",
    velocity_params_.max_lateral_accel_mps2);
  update_param(
    parameters, "trajectory_velocity_optimizer.set_engage_speed",
    velocity_params_.set_engage_speed);
  update_param(
    parameters, "trajectory_velocity_optimizer.limit_speed", velocity_params_.limit_speed);
  update_param(
    parameters, "trajectory_velocity_optimizer.limit_lateral_acceleration",
    velocity_params_.limit_lateral_acceleration);
  update_param(
    parameters, "trajectory_velocity_optimizer.smooth_velocities",
    velocity_params_.smooth_velocities);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
