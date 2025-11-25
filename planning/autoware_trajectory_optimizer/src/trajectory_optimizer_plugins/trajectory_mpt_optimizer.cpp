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

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
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
#include <utility>
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

  // Calculate base velocity for adaptive width
  const double base_velocity = data.current_odometry.twist.twist.linear.x;

  // Generate adaptive corridor bounds
  const auto bounds = generate_adaptive_bounds(traj_points, base_velocity);

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

  auto get_distance = [](const TrajectoryPoint & p1, const TrajectoryPoint & p2) {
    return autoware_utils::calc_distance2d(p1.pose.position, p2.pose.position);
  };

  auto get_acceleration_based_on_velocity_and_distance =
    [&](const TrajectoryPoint & p_curr, const TrajectoryPoint & p_next) {
      const double delta_s = get_distance(p_curr, p_next);
      if (delta_s < 1e-6) {
        return 0.0;
      }
      // Use correct kinematic formula: a = (v² - v₀²) / (2s)
      const double v_next_sq = p_next.longitudinal_velocity_mps * p_next.longitudinal_velocity_mps;
      const double v_curr_sq = p_curr.longitudinal_velocity_mps * p_curr.longitudinal_velocity_mps;
      return (v_next_sq - v_curr_sq) / (2.0 * delta_s);
    };

  auto get_dt_based_on_distance_velocity_and_acceleration = [&](
                                                              const double v, const double a,
                                                              const TrajectoryPoint & p_curr,
                                                              const TrajectoryPoint & p_next) {
    const double delta_s = get_distance(p_curr, p_next);
    constexpr double min_velocity = 1e-3;  // 1mm/s threshold

    if (std::abs(a) < 1e-6) {
      // Constant velocity model
      if (std::abs(v) < min_velocity) {
        // Vehicle nearly stopped, return small dt
        return 0.1;
      }
      return delta_s / v;
    }

    const double discriminant = v * v + 2.0 * a * delta_s;
    if (discriminant < 0.0) {
      // Physically invalid scenario - fallback to constant velocity
      if (std::abs(v) < min_velocity) {
        return 0.1;
      }
      return delta_s / v;
    }

    const double v_next = std::sqrt(discriminant);
    return (v_next - v) / a;
  };

  auto get_time_from_start_and_acceleration =
    [&](const TrajectoryPoint & p_curr, const TrajectoryPoint & p_next) {
      const auto velocity = static_cast<double>(p_curr.longitudinal_velocity_mps);
      const auto acceleration = get_acceleration_based_on_velocity_and_distance(p_curr, p_next);
      const auto dt =
        get_dt_based_on_distance_velocity_and_acceleration(velocity, acceleration, p_curr, p_next);
      return std::make_pair(dt, acceleration);
    };

  // set first point time_from_start to zero
  traj_points.front().time_from_start.sec = 0;
  traj_points.front().time_from_start.nanosec = 0;

  for (size_t i = 1; i < traj_points.size(); ++i) {
    auto & next_point = traj_points[i];
    auto & curr_point = traj_points[i - 1];
    const auto [dt, acc] = get_time_from_start_and_acceleration(curr_point, next_point);

    // Update time_from_start
    const double curr_time = static_cast<double>(curr_point.time_from_start.sec) +
                             static_cast<double>(curr_point.time_from_start.nanosec) * 1e-9;
    const double new_time = curr_time + dt;

    // Split time into seconds and nanoseconds properly
    const auto sec_part = static_cast<int32_t>(new_time);
    const double fractional_part = new_time - static_cast<double>(sec_part);
    next_point.time_from_start.sec = sec_part;
    next_point.time_from_start.nanosec = static_cast<uint32_t>(fractional_part * 1e9);

    // Update acceleration
    curr_point.acceleration_mps2 = static_cast<float>(acc);
  }

  // Apply moving average filter to acceleration
  const int window_size = std::max(1, mpt_params_.acceleration_moving_average_window);
  std::vector<float> original_accelerations;
  original_accelerations.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    original_accelerations.push_back(point.acceleration_mps2);
  }

  for (size_t i = 0; i < traj_points.size() - 1; ++i) {
    // Calculate moving average using backward-looking window
    double sum = 0.0;
    int count = 0;
    const int start_idx = std::max(0, static_cast<int>(i) - window_size + 1);
    for (int j = start_idx; j <= static_cast<int>(i); ++j) {
      sum += original_accelerations[j];
      count++;
    }
    traj_points[i].acceleration_mps2 = static_cast<float>(sum / count);
  }

  // set last point acceleration to zero
  traj_points.back().acceleration_mps2 = 0.0f;

  RCLCPP_DEBUG_THROTTLE(
    get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
    "MPT: Optimized %zu->%zu points, recalculated time", original_size, traj_points.size());
}

BoundsPair TrajectoryMPTOptimizer::generate_adaptive_bounds(
  const TrajectoryPoints & traj_points, [[maybe_unused]] const double base_velocity) const
{
  BoundsPair bounds;
  bounds.left_bound.reserve(traj_points.size());
  bounds.right_bound.reserve(traj_points.size());

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & point = traj_points[i];
    const auto & pose = point.pose;

    // Calculate adaptive corridor width
    double corridor_width = mpt_params_.corridor_width_m;

    if (mpt_params_.enable_adaptive_width) {
      const double curvature = calculate_curvature_at_point(traj_points, i);
      const double velocity = point.longitudinal_velocity_mps;
      corridor_width =
        calculate_adaptive_corridor_width(curvature, velocity, mpt_params_.corridor_width_m);
    }

    // Ensure minimum clearance
    const double min_width = vehicle_info_.vehicle_width_m + mpt_params_.min_clearance_m;
    corridor_width = std::max(corridor_width, min_width);

    // Get yaw from quaternion
    const double yaw = tf2::getYaw(pose.orientation);

    // Calculate perpendicular offset (90 degrees to the left and right)
    const double left_yaw = yaw + M_PI_2;
    const double right_yaw = yaw - M_PI_2;

    // Left bound point
    geometry_msgs::msg::Point left_point;
    left_point.x = pose.position.x + corridor_width * std::cos(left_yaw);
    left_point.y = pose.position.y + corridor_width * std::sin(left_yaw);
    left_point.z = pose.position.z;
    bounds.left_bound.push_back(left_point);

    // Right bound point
    geometry_msgs::msg::Point right_point;
    right_point.x = pose.position.x + corridor_width * std::cos(right_yaw);
    right_point.y = pose.position.y + corridor_width * std::sin(right_yaw);
    right_point.z = pose.position.z;
    bounds.right_bound.push_back(right_point);
  }

  return bounds;
}

double TrajectoryMPTOptimizer::calculate_adaptive_corridor_width(
  const double curvature, const double velocity, const double base_width) const
{
  // Widen corridor in curves (higher curvature needs more space)
  const double curvature_addition = mpt_params_.curvature_width_factor * std::abs(curvature);

  // Widen corridor at low speeds (more maneuvering room)
  constexpr double max_velocity = 15.0;  // m/s (~54 km/h)
  const double velocity_factor = std::max(0.0, (max_velocity - velocity) / max_velocity);
  const double velocity_addition = mpt_params_.velocity_width_factor * velocity_factor;

  return base_width + curvature_addition + velocity_addition;
}

double TrajectoryMPTOptimizer::calculate_curvature_at_point(
  const TrajectoryPoints & traj_points, const size_t idx)
{
  if (traj_points.size() < 3 || idx == 0 || idx >= traj_points.size() - 1) {
    return 0.0;
  }

  const auto & p_prev = traj_points[idx - 1].pose.position;
  const auto & p_curr = traj_points[idx].pose.position;
  const auto & p_next = traj_points[idx + 1].pose.position;

  const double dx1 = p_curr.x - p_prev.x;
  const double dy1 = p_curr.y - p_prev.y;
  const double dx2 = p_next.x - p_curr.x;
  const double dy2 = p_next.y - p_curr.y;

  const double angle1 = std::atan2(dy1, dx1);
  const double angle2 = std::atan2(dy2, dx2);
  double angle_diff = angle2 - angle1;

  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  const double arc_length = std::hypot(dx1, dy1) + std::hypot(dx2, dy2);

  // κ = Δθ / arc_length
  return (arc_length > 1e-6) ? (angle_diff / arc_length) : 0.0;
}

PlannerData TrajectoryMPTOptimizer::create_planner_data(
  const TrajectoryPoints & traj_points, const BoundsPair & bounds,
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
  const BoundsPair & bounds, const TrajectoryPoints & traj_points) const
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
