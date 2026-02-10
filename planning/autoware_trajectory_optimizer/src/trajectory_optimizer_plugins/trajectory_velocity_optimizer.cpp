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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.hpp"

#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>
namespace autoware::trajectory_optimizer::plugin
{

void TrajectoryVelocityOptimizer::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  TrajectoryOptimizerPluginBase::initialize(name, node_ptr, time_keeper);

  set_up_params();

  sub_planning_velocity_ =
    std::make_shared<autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit>>(
      node_ptr, "~/input/external_velocity_limit_mps", rclcpp::QoS{1});

  pub_velocity_limit_ = node_ptr->create_publisher<VelocityLimit>(
    "~/output/current_velocity_limit_mps", rclcpp::QoS{1}.transient_local());

  // publish default max velocity
  VelocityLimit max_vel_msg{};
  max_vel_msg.stamp = node_ptr->now();
  max_vel_msg.max_velocity = static_cast<float>(velocity_params_.default_max_velocity_mps);
  pub_velocity_limit_->publish(max_vel_msg);
}

void TrajectoryVelocityOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  if (!params.use_velocity_optimizer) {
    return;
  }

  const auto & current_odometry = data.current_odometry;
  const auto & current_acceleration = data.current_acceleration;
  const auto & current_speed = current_odometry.twist.twist.linear.x;
  const auto & current_linear_acceleration = current_acceleration.accel.accel.linear.x;
  const double & target_pull_out_speed_mps = velocity_params_.target_pull_out_speed_mps;
  const double & target_pull_out_acc_mps2 = velocity_params_.target_pull_out_acc_mps2;

  // Initialize max velocity per point with global max speed
  std::vector<double> max_velocity_per_point(
    traj_points.size(), std::numeric_limits<double>::max());

  const bool max_speed_update_in_place = !velocity_params_.smooth_velocities;

  // Apply lateral acceleration limiting and update max velocity constraints
  if (velocity_params_.limit_lateral_acceleration) {
    // limit_lateral_acceleration returns per-point max velocities based on curvature
    trajectory_velocity_optimizer_utils::limit_lateral_acceleration(
      traj_points, max_velocity_per_point, velocity_params_.max_lateral_accel_mps2,
      velocity_params_.min_limited_speed_mps, data.current_odometry, max_speed_update_in_place);
  }

  auto initial_motion_speed =
    (current_speed > target_pull_out_speed_mps) ? current_speed : target_pull_out_speed_mps;
  auto initial_motion_acc = (current_speed > target_pull_out_speed_mps)
                              ? current_linear_acceleration
                              : target_pull_out_acc_mps2;

  if (velocity_params_.set_engage_speed && (current_speed < target_pull_out_speed_mps)) {
    trajectory_velocity_optimizer_utils::clamp_velocities(
      traj_points, static_cast<float>(initial_motion_speed),
      static_cast<float>(initial_motion_acc));
  }

  // Apply global speed limit to trajectory and max velocity array
  if (velocity_params_.limit_speed) {
    const auto external_velocity_limit = sub_planning_velocity_->take_data();
    const auto max_speed_mps = (external_velocity_limit)
                                 ? static_cast<float>(external_velocity_limit->max_velocity)
                                 : static_cast<float>(velocity_params_.default_max_velocity_mps);
    if (max_speed_update_in_place) {
      // Directly set max velocity on trajectory points
      trajectory_velocity_optimizer_utils::set_max_velocity(traj_points, max_speed_mps);
    } else {
      // Update per-point max velocity constraints
      for (auto & v : max_velocity_per_point) {
        v = std::min(v, static_cast<double>(max_speed_mps));
      }
    }
    if (external_velocity_limit) {
      pub_velocity_limit_->publish(*external_velocity_limit);
    }
  }

  if (velocity_params_.smooth_velocities) {
    if (!continuous_jerk_smoother_) {
      continuous_jerk_smoother_ =
        std::make_shared<ContinuousJerkSmoother>(velocity_params_.continuous_jerk_smoother_params);
    }
    InitialMotion initial_motion{initial_motion_speed, initial_motion_acc};
    // Pass max velocity per point to enforce lateral acceleration as hard constraint in QP
    trajectory_velocity_optimizer_utils::filter_velocity(
      traj_points, initial_motion, velocity_params_.nearest_dist_threshold_m,
      autoware_utils_math::deg2rad(velocity_params_.nearest_yaw_threshold_deg),
      continuous_jerk_smoother_, current_odometry, max_velocity_per_point);
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
  velocity_params_.max_lateral_accel_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.max_lateral_accel_mps2");
  velocity_params_.min_limited_speed_mps = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.min_limited_speed_mps");
  velocity_params_.default_max_velocity_mps =
    get_or_declare_parameter<double>(*node_ptr, "max_vel");
  velocity_params_.set_engage_speed =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.set_engage_speed");
  velocity_params_.limit_speed =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.limit_speed");
  velocity_params_.limit_lateral_acceleration = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_velocity_optimizer.limit_lateral_acceleration");
  velocity_params_.smooth_velocities =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.smooth_velocities");

  // Continuous jerk smoother parameters - QP weights
  auto & cjs = velocity_params_.continuous_jerk_smoother_params;
  cjs.jerk_weight = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.continuous_jerk_smoother.jerk_weight");
  cjs.over_v_weight = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.continuous_jerk_smoother.over_v_weight");
  cjs.over_a_weight = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.continuous_jerk_smoother.over_a_weight");
  cjs.over_j_weight = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.continuous_jerk_smoother.over_j_weight");
  cjs.velocity_tracking_weight = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.continuous_jerk_smoother.velocity_tracking_weight");
  cjs.accel_tracking_weight = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.continuous_jerk_smoother.accel_tracking_weight");

  // Kinematic limits - loaded from common namespace (limit.*)
  cjs.max_accel = get_or_declare_parameter<double>(*node_ptr, "limit.max_acc");
  cjs.min_decel = get_or_declare_parameter<double>(*node_ptr, "limit.min_acc");
  cjs.max_jerk = get_or_declare_parameter<double>(*node_ptr, "limit.max_jerk");
  cjs.min_jerk = get_or_declare_parameter<double>(*node_ptr, "limit.min_jerk");
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
    parameters, "trajectory_velocity_optimizer.max_lateral_accel_mps2",
    velocity_params_.max_lateral_accel_mps2);
  update_param(
    parameters, "trajectory_velocity_optimizer.min_limited_speed_mps",
    velocity_params_.min_limited_speed_mps);
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

  // Continuous jerk smoother parameters - QP weights
  auto & cjs = velocity_params_.continuous_jerk_smoother_params;
  update_param(
    parameters, "trajectory_velocity_optimizer.continuous_jerk_smoother.jerk_weight",
    cjs.jerk_weight);
  update_param(
    parameters, "trajectory_velocity_optimizer.continuous_jerk_smoother.over_v_weight",
    cjs.over_v_weight);
  update_param(
    parameters, "trajectory_velocity_optimizer.continuous_jerk_smoother.over_a_weight",
    cjs.over_a_weight);
  update_param(
    parameters, "trajectory_velocity_optimizer.continuous_jerk_smoother.over_j_weight",
    cjs.over_j_weight);
  update_param(
    parameters, "trajectory_velocity_optimizer.continuous_jerk_smoother.velocity_tracking_weight",
    cjs.velocity_tracking_weight);
  update_param(
    parameters, "trajectory_velocity_optimizer.continuous_jerk_smoother.accel_tracking_weight",
    cjs.accel_tracking_weight);

  // Kinematic limits - from common namespace (limit.*)
  update_param(parameters, "limit.max_acc", cjs.max_accel);
  update_param(parameters, "limit.min_acc", cjs.min_decel);
  update_param(parameters, "limit.max_jerk", cjs.max_jerk);
  update_param(parameters, "limit.min_jerk", cjs.min_jerk);

  // Update smoother parameters if it exists
  if (continuous_jerk_smoother_) {
    continuous_jerk_smoother_->set_params(cjs);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
