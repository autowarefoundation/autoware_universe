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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_mpt_optimizer.hpp"

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_mpt_optimizer_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

void TrajectoryMPTOptimizer::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  TrajectoryOptimizerPluginBase::initialize(name, node_ptr, time_keeper);

  RCLCPP_INFO(node_ptr->get_logger(), "MPT Optimizer plugin: Starting initialization...");

  try {
    // Get vehicle info
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();
    RCLCPP_INFO(node_ptr->get_logger(), "MPT: Vehicle info loaded");

    // Initialize debug data
    debug_data_ptr_ = std::make_shared<DebugData>();

    // Set up parameters
    set_up_params();
    RCLCPP_INFO(node_ptr->get_logger(), "MPT: Parameters set up");

    // Create TimeKeeper for performance profiling
    auto debug_pub = node_ptr->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/mpt_processing_time_detail_ms", 1);
    mpt_time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_pub);
    RCLCPP_INFO(node_ptr->get_logger(), "MPT: TimeKeeper created");

    // Initialize MPT optimizer
    mpt_optimizer_ptr_ = std::make_shared<MPTOptimizer>(
      node_ptr, mpt_params_.enable_debug_info, ego_nearest_param_, vehicle_info_, traj_param_,
      debug_data_ptr_, mpt_time_keeper_);
    RCLCPP_INFO(node_ptr->get_logger(), "MPT: MPTOptimizer created");

    // Create debug markers publisher
    debug_markers_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/mpt_bounds_markers", 1);

    RCLCPP_INFO(node_ptr->get_logger(), "MPT Optimizer plugin initialized successfully!");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_ptr->get_logger(), "MPT Optimizer plugin initialization FAILED: %s", e.what());
    throw;
  }
}

void TrajectoryMPTOptimizer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  mpt_params_.corridor_width_m =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_mpt_optimizer.corridor_width_m");
  mpt_params_.enable_adaptive_width =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_mpt_optimizer.enable_adaptive_width");
  mpt_params_.curvature_width_factor =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_mpt_optimizer.curvature_width_factor");
  mpt_params_.velocity_width_factor =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_mpt_optimizer.velocity_width_factor");
  mpt_params_.min_clearance_m =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_mpt_optimizer.min_clearance_m");

  mpt_params_.reset_previous_data_each_iteration = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_mpt_optimizer.reset_previous_data_each_iteration");
  mpt_params_.enable_debug_info =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_mpt_optimizer.enable_debug_info");

  traj_param_.output_delta_arc_length = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.output_delta_arc_length_m");
  traj_param_.output_backward_traj_length = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.output_backward_traj_length_m");

  // Ego nearest parameters
  ego_nearest_param_.dist_threshold = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.ego_nearest_dist_threshold_m");
  const auto ego_nearest_yaw_threshold_deg = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg");
  ego_nearest_param_.yaw_threshold = autoware_utils_math::deg2rad(ego_nearest_yaw_threshold_deg);

  // Acceleration smoothing parameters
  mpt_params_.acceleration_moving_average_window = get_or_declare_parameter<int>(
    *node_ptr, "trajectory_mpt_optimizer.acceleration_moving_average_window");
}

rcl_interfaces::msg::SetParametersResult TrajectoryMPTOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "trajectory_mpt_optimizer.corridor_width_m", mpt_params_.corridor_width_m);
  update_param(
    parameters, "trajectory_mpt_optimizer.enable_adaptive_width",
    mpt_params_.enable_adaptive_width);
  update_param(
    parameters, "trajectory_mpt_optimizer.curvature_width_factor",
    mpt_params_.curvature_width_factor);
  update_param(
    parameters, "trajectory_mpt_optimizer.velocity_width_factor",
    mpt_params_.velocity_width_factor);
  update_param(parameters, "trajectory_mpt_optimizer.min_clearance_m", mpt_params_.min_clearance_m);

  update_param(
    parameters, "trajectory_mpt_optimizer.reset_previous_data_each_iteration",
    mpt_params_.reset_previous_data_each_iteration);
  update_param(
    parameters, "trajectory_mpt_optimizer.enable_debug_info", mpt_params_.enable_debug_info);

  update_param(
    parameters, "trajectory_mpt_optimizer.output_delta_arc_length_m",
    traj_param_.output_delta_arc_length);
  update_param(
    parameters, "trajectory_mpt_optimizer.output_backward_traj_length_m",
    traj_param_.output_backward_traj_length);

  update_param(
    parameters, "trajectory_mpt_optimizer.ego_nearest_dist_threshold_m",
    ego_nearest_param_.dist_threshold);

  double ego_nearest_yaw_threshold_deg = 0.0;
  if (update_param(
        parameters, "trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg",
        ego_nearest_yaw_threshold_deg)) {
    ego_nearest_param_.yaw_threshold = autoware_utils_math::deg2rad(ego_nearest_yaw_threshold_deg);
  }

  update_param(
    parameters, "trajectory_mpt_optimizer.acceleration_moving_average_window",
    mpt_params_.acceleration_moving_average_window);

  if (mpt_optimizer_ptr_) {
    mpt_optimizer_ptr_->onParam(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void TrajectoryMPTOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());

  // Skip if MPT optimizer is disabled
  if (!params.use_mpt_optimizer) {
    return;
  }

  // Minimum points required for optimization
  constexpr size_t min_points_for_optimization = 10;
  if (traj_points.size() < min_points_for_optimization) {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT: Trajectory too short (%zu < %zu points), skipping", traj_points.size(),
      min_points_for_optimization);
    return;
  }

  // Reset previous data if configured (for diffusion planner's new trajectories each cycle)
  if (mpt_params_.reset_previous_data_each_iteration) {
    mpt_optimizer_ptr_->resetPreviousData();
  }

  // Generate simple perpendicular offset bounds
  const auto bounds = trajectory_mpt_optimizer_utils::generate_bounds(
    traj_points, mpt_params_.corridor_width_m, mpt_params_.enable_adaptive_width,
    mpt_params_.curvature_width_factor, mpt_params_.velocity_width_factor,
    mpt_params_.min_clearance_m, vehicle_info_.vehicle_width_m);

  // Publish debug markers
  if (mpt_params_.enable_debug_info) {
    publish_debug_markers(bounds, traj_points);
  }

  // Create planner data
  const auto planner_data = create_planner_data(traj_points, bounds, data);

  // Store original size for validation
  const size_t original_size = traj_points.size();

  // Run MPT optimization
  const auto optimized_traj = mpt_optimizer_ptr_->optimizeTrajectory(planner_data);

  // Apply optimized trajectory if successful
  if (!optimized_traj) {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT: Optimization failed, keeping original trajectory");
    return;
  }
  // Validate optimized trajectory
  if (optimized_traj->empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT: Returned empty trajectory, keeping original");
    return;
  }

  // Apply optimized trajectory
  traj_points = *optimized_traj;

  // Recalculate acceleration and time_from_start for kinematic consistency
  trajectory_mpt_optimizer_utils::recalculate_trajectory_dynamics(
    traj_points, mpt_params_.acceleration_moving_average_window);

  RCLCPP_DEBUG_THROTTLE(
    get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
    "MPT: Optimized %zu->%zu points, recalculated dynamics", original_size, traj_points.size());
}

PlannerData TrajectoryMPTOptimizer::create_planner_data(
  const TrajectoryPoints & traj_points, const trajectory_mpt_optimizer_utils::BoundsPair & bounds,
  const TrajectoryOptimizerData & data) const
{
  PlannerData planner_data;

  // Create header from odometry frame
  planner_data.header.stamp = get_node_ptr()->now();
  planner_data.header.frame_id = data.current_odometry.header.frame_id;

  // Set trajectory points
  planner_data.traj_points = traj_points;

  // Set bounds
  planner_data.left_bound = bounds.left_bound;
  planner_data.right_bound = bounds.right_bound;

  // Set ego state
  planner_data.ego_pose = data.current_odometry.pose.pose;
  planner_data.ego_vel = data.current_odometry.twist.twist.linear.x;

  return planner_data;
}

void TrajectoryMPTOptimizer::publish_debug_markers(
  const trajectory_mpt_optimizer_utils::BoundsPair & bounds,
  const TrajectoryPoints & traj_points) const
{
  if (debug_markers_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  const auto now = get_node_ptr()->now();
  const std::string frame_id = "map";

  // Left bound marker (green)
  visualization_msgs::msg::Marker left_marker;
  left_marker.header.frame_id = frame_id;
  left_marker.header.stamp = now;
  left_marker.ns = "mpt_left_bound";
  left_marker.id = 0;
  left_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  left_marker.action = visualization_msgs::msg::Marker::ADD;
  left_marker.scale.x = 0.1;  // line width
  left_marker.color.r = 0.0;
  left_marker.color.g = 1.0;
  left_marker.color.b = 0.0;
  left_marker.color.a = 0.8;
  left_marker.points = bounds.left_bound;
  markers.markers.push_back(left_marker);

  // Right bound marker (red)
  visualization_msgs::msg::Marker right_marker;
  right_marker.header.frame_id = frame_id;
  right_marker.header.stamp = now;
  right_marker.ns = "mpt_right_bound";
  right_marker.id = 1;
  right_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  right_marker.action = visualization_msgs::msg::Marker::ADD;
  right_marker.scale.x = 0.1;
  right_marker.color.r = 1.0;
  right_marker.color.g = 0.0;
  right_marker.color.b = 0.0;
  right_marker.color.a = 0.8;
  right_marker.points = bounds.right_bound;
  markers.markers.push_back(right_marker);

  // Reference trajectory marker (blue)
  visualization_msgs::msg::Marker traj_marker;
  traj_marker.header.frame_id = frame_id;
  traj_marker.header.stamp = now;
  traj_marker.ns = "mpt_reference_trajectory";
  traj_marker.id = 2;
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.scale.x = 0.15;
  traj_marker.color.r = 0.0;
  traj_marker.color.g = 0.0;
  traj_marker.color.b = 1.0;
  traj_marker.color.a = 0.6;
  for (const auto & point : traj_points) {
    traj_marker.points.push_back(point.pose.position);
  }
  markers.markers.push_back(traj_marker);

  debug_markers_pub_->publish(markers);
}

}  // namespace autoware::trajectory_optimizer::plugin

// Export plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryMPTOptimizer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
