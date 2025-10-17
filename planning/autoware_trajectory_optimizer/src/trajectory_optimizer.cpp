// Copyright 2024 TIER IV, Inc.
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

#include "autoware/trajectory_optimizer/trajectory_optimizer.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <memory>
#include <numeric>
#include <vector>

namespace autoware::trajectory_optimizer
{

TrajectoryOptimizer::TrajectoryOptimizer(const rclcpp::NodeOptions & options)
: Node("trajectory_optimizer", options)
{
  // create time_keeper and its publisher
  // NOTE: This has to be called before setupSmoother to pass the time_keeper to the smoother.
  debug_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_);

  set_up_params();

  // Parameter Callback
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&TrajectoryOptimizer::on_parameter, this, std::placeholders::_1));

  // interface subscriber
  trajectories_sub_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryOptimizer::on_traj, this, std::placeholders::_1));
  // interface publisher
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  trajectories_pub_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
  // debug time keeper
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);
  // last time a trajectory was received
  last_time_ = std::make_shared<rclcpp::Time>(now());
}

void TrajectoryOptimizer::initialize_optimizers()
{
  if (initialized_optimizers_) {
    return;
  }
  // initialize optimizer pointers
  eb_smoother_optimizer_ptr_ = std::make_shared<plugin::TrajectoryEBSmootherOptimizer>(
    "eb_smoother_optimizer", this, time_keeper_, params_);
  trajectory_extender_ptr_ = std::make_shared<plugin::TrajectoryExtender>(
    "trajectory_extender", this, time_keeper_, params_);
  trajectory_point_fixer_ptr_ = std::make_shared<plugin::TrajectoryPointFixer>(
    "trajectory_point_fixer", this, time_keeper_, params_);
  trajectory_spline_smoother_ptr_ = std::make_shared<plugin::TrajectorySplineSmoother>(
    "trajectory_spline_smoother", this, time_keeper_, params_);
  trajectory_velocity_optimizer_ptr_ = std::make_shared<plugin::TrajectoryVelocityOptimizer>(
    "trajectory_velocity_optimizer", this, time_keeper_, params_);
  initialized_optimizers_ = true;
}

rcl_interfaces::msg::SetParametersResult TrajectoryOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;
  auto params = params_;

  update_param<double>(parameters, "nearest_dist_threshold_m", params.nearest_dist_threshold_m);
  update_param<double>(parameters, "nearest_yaw_threshold_rad", params.nearest_yaw_threshold_rad);
  update_param<double>(parameters, "target_pull_out_speed_mps", params.target_pull_out_speed_mps);
  update_param<double>(parameters, "target_pull_out_acc_mps2", params.target_pull_out_acc_mps2);
  update_param<double>(parameters, "max_speed_mps", params.max_speed_mps);
  update_param<double>(parameters, "max_lateral_accel_mps2", params.max_lateral_accel_mps2);
  update_param<double>(
    parameters, "spline_interpolation_resolution_m", params.spline_interpolation_resolution_m);
  update_param<double>(
    parameters, "spline_interpolation_max_yaw_discrepancy_deg",
    params.spline_interpolation_max_yaw_discrepancy_deg);
  update_param<double>(
    parameters, "spline_interpolation_max_distance_discrepancy_m",
    params.spline_interpolation_max_distance_discrepancy_m);
  update_param<double>(
    parameters, "backward_trajectory_extension_m", params.backward_trajectory_extension_m);
  update_param<bool>(
    parameters, "use_akima_spline_interpolation", params.use_akima_spline_interpolation);
  update_param<bool>(parameters, "smooth_velocities", params.smooth_velocities);
  update_param<bool>(parameters, "smooth_trajectories", params.smooth_trajectories);
  update_param<bool>(parameters, "limit_speed", params.limit_speed);
  update_param<bool>(parameters, "limit_lateral_acceleration", params.limit_lateral_acceleration);
  update_param<bool>(parameters, "set_engage_speed", params.set_engage_speed);
  update_param<bool>(parameters, "fix_invalid_points", params.fix_invalid_points);
  update_param<bool>(parameters, "extend_trajectory_backward", params.extend_trajectory_backward);
  update_param<bool>(
    parameters, "spline_copy_original_orientation", params.spline_copy_original_orientation);

  params_ = params;

  // call update_param for all optimizer plugins

  if (eb_smoother_optimizer_ptr_) {
    eb_smoother_optimizer_ptr_->on_parameter(parameters);
  }
  if (trajectory_extender_ptr_) {
    trajectory_extender_ptr_->on_parameter(parameters);
  }
  if (trajectory_point_fixer_ptr_) {
    trajectory_point_fixer_ptr_->on_parameter(parameters);
  }
  if (trajectory_spline_smoother_ptr_) {
    trajectory_spline_smoother_ptr_->on_parameter(parameters);
  }
  if (trajectory_velocity_optimizer_ptr_) {
    trajectory_velocity_optimizer_ptr_->on_parameter(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void TrajectoryOptimizer::initialize_planners()
{
}

void TrajectoryOptimizer::reset_previous_data()
{
}

void TrajectoryOptimizer::set_up_params()
{
  using autoware_utils_rclcpp::get_or_declare_parameter;

  params_.nearest_dist_threshold_m =
    get_or_declare_parameter<double>(*this, "nearest_dist_threshold_m");
  params_.nearest_yaw_threshold_rad =
    get_or_declare_parameter<double>(*this, "nearest_yaw_threshold_rad");
  params_.target_pull_out_speed_mps =
    get_or_declare_parameter<double>(*this, "target_pull_out_speed_mps");
  params_.target_pull_out_acc_mps2 =
    get_or_declare_parameter<double>(*this, "target_pull_out_acc_mps2");
  params_.max_speed_mps = get_or_declare_parameter<double>(*this, "max_speed_mps");
  params_.max_lateral_accel_mps2 =
    get_or_declare_parameter<double>(*this, "max_lateral_accel_mps2");
  params_.spline_interpolation_resolution_m =
    get_or_declare_parameter<double>(*this, "spline_interpolation_resolution_m");
  params_.spline_interpolation_max_yaw_discrepancy_deg =
    get_or_declare_parameter<double>(*this, "spline_interpolation_max_yaw_discrepancy_deg");
  params_.spline_interpolation_max_distance_discrepancy_m =
    get_or_declare_parameter<double>(*this, "spline_interpolation_max_distance_discrepancy_m");
  params_.backward_trajectory_extension_m =
    get_or_declare_parameter<double>(*this, "backward_trajectory_extension_m");
  params_.use_akima_spline_interpolation =
    get_or_declare_parameter<bool>(*this, "use_akima_spline_interpolation");
  params_.smooth_velocities = get_or_declare_parameter<bool>(*this, "smooth_velocities");
  params_.smooth_trajectories = get_or_declare_parameter<bool>(*this, "smooth_trajectories");
  params_.limit_speed = get_or_declare_parameter<bool>(*this, "limit_speed");
  params_.limit_lateral_acceleration =
    get_or_declare_parameter<bool>(*this, "limit_lateral_acceleration");
  params_.set_engage_speed = get_or_declare_parameter<bool>(*this, "set_engage_speed");
  params_.fix_invalid_points = get_or_declare_parameter<bool>(*this, "fix_invalid_points");
  params_.extend_trajectory_backward =
    get_or_declare_parameter<bool>(*this, "extend_trajectory_backward");
  params_.spline_copy_original_orientation =
    get_or_declare_parameter<bool>(*this, "spline_copy_original_orientation");
}

void TrajectoryOptimizer::on_traj([[maybe_unused]] const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  initialize_optimizers();

  current_odometry_ptr_ = sub_current_odometry_.take_data();
  current_acceleration_ptr_ = sub_current_acceleration_.take_data();
  params_.current_odometry = *current_odometry_ptr_;
  params_.current_acceleration = *current_acceleration_ptr_;

  last_time_ = std::make_shared<rclcpp::Time>(now());

  if (!current_odometry_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "No odometry or acceleration data");
    return;
  }

  if (params_.extend_trajectory_backward) {
    utils::add_ego_state_to_trajectory(
      past_ego_state_trajectory_.points, *current_odometry_ptr_, params_);
  }

  CandidateTrajectories output_trajectories = *msg;
  for (auto & trajectory : output_trajectories.candidate_trajectories) {
    // apply optimizers
    trajectory_extender_ptr_->optimize_trajectory(trajectory.points, params_);
    trajectory_point_fixer_ptr_->optimize_trajectory(trajectory.points, params_);
    eb_smoother_optimizer_ptr_->optimize_trajectory(trajectory.points, params_);
    trajectory_spline_smoother_ptr_->optimize_trajectory(trajectory.points, params_);
    trajectory_velocity_optimizer_ptr_->optimize_trajectory(trajectory.points, params_);
    trajectory_point_fixer_ptr_->optimize_trajectory(trajectory.points, params_);
    motion_utils::calculate_time_from_start(
      trajectory.points, current_odometry_ptr_->pose.pose.position);
  }

  trajectories_pub_->publish(output_trajectories);

  Trajectory output_trajectory;
  output_trajectory.header = output_trajectories.candidate_trajectories.front().header;
  output_trajectory.points = output_trajectories.candidate_trajectories.front().points;

  trajectory_pub_->publish(output_trajectory);
}

}  // namespace autoware::trajectory_optimizer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_optimizer::TrajectoryOptimizer)
