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

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/envelope.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

TrajectoryMPTOptimizer::TrajectoryMPTOptimizer(
  const std::string name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const TrajectoryOptimizerParams & params)
: TrajectoryOptimizerPluginBase(name, node_ptr, time_keeper, params)
{
  // Get vehicle info
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();

  // Initialize debug data
  debug_data_ptr_ = std::make_shared<autoware::path_optimizer::DebugData>();

  // Set up parameters
  set_up_params();

  // Initialize MPT optimizer
  mpt_optimizer_ptr_ = std::make_shared<autoware::path_optimizer::MPTOptimizer>(
    node_ptr, mpt_params_.enable_debug_info, ego_nearest_param_, vehicle_info_, traj_param_,
    debug_data_ptr_, time_keeper);

  // Subscribe to lanelet map (within plugin, not main node)
  map_sub_ = node_ptr->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryMPTOptimizer::on_map, this, std::placeholders::_1));

  // Create debug marker publisher
  debug_markers_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/mpt_bounds_markers", 1);
}

void TrajectoryMPTOptimizer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  // MPT-specific parameters
  mpt_params_.bounds_lateral_offset_m =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_mpt_optimizer.bounds_lateral_offset_m");
  mpt_params_.enable_debug_info =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_mpt_optimizer.enable_debug_info");

  // Trajectory parameters (required by MPT optimizer)
  traj_param_.output_delta_arc_length = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.output_delta_arc_length_m");
  traj_param_.output_backward_traj_length = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.output_backward_traj_length_m");

  // Ego nearest parameters (required by MPT optimizer)
  ego_nearest_param_.dist_threshold = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.ego_nearest_dist_threshold_m");
  const double ego_nearest_yaw_threshold_deg = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg");
  ego_nearest_param_.yaw_threshold = autoware_utils_math::deg2rad(ego_nearest_yaw_threshold_deg);
}

rcl_interfaces::msg::SetParametersResult TrajectoryMPTOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "trajectory_mpt_optimizer.bounds_lateral_offset_m",
    mpt_params_.bounds_lateral_offset_m);
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

  // Handle yaw threshold with deg2rad conversion
  double ego_nearest_yaw_threshold_deg = 0.0;
  if (update_param(
        parameters, "trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg",
        ego_nearest_yaw_threshold_deg)) {
    ego_nearest_param_.yaw_threshold = autoware_utils_math::deg2rad(ego_nearest_yaw_threshold_deg);
  }

  // Propagate parameter updates to MPT optimizer
  if (mpt_optimizer_ptr_) {
    mpt_optimizer_ptr_->onParam(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void TrajectoryMPTOptimizer::on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
  RCLCPP_DEBUG(
    get_node_ptr()->get_logger(), "TrajectoryMPTOptimizer: Received lanelet map with %zu lanelets",
    lanelet_map_ptr_->laneletLayer.size());
}

BoundsPair TrajectoryMPTOptimizer::generate_bounds_from_lanelet_map(
  const TrajectoryPoints & traj_points) const
{
  BoundsPair bounds;

  if (!lanelet_map_ptr_ || traj_points.empty()) {
    return bounds;
  }

  constexpr double overlap_threshold = 0.01;

  // Step 1: Create trajectory linestring for spatial query
  autoware_utils::LineString2d trajectory_ls;
  trajectory_ls.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    trajectory_ls.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  RCLCPP_INFO(
    get_node_ptr()->get_logger(),
    "MPT Optimizer: Trajectory linestring has %zu points, first=(%.2f, %.2f), last=(%.2f, %.2f)",
    trajectory_ls.size(), trajectory_ls.front().x(), trajectory_ls.front().y(),
    trajectory_ls.back().x(), trajectory_ls.back().y());

  // Step 2: Use R-tree spatial index to get candidate lanelets (FAST!)
  const auto candidate_lanelets = lanelet_map_ptr_->laneletLayer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));

  RCLCPP_INFO(
    get_node_ptr()->get_logger(), "MPT Optimizer: R-tree returned %zu candidate lanelets",
    candidate_lanelets.size());

  if (candidate_lanelets.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: No candidate lanelets found for trajectory");
    return bounds;
  }

  // Step 3: Find ALL lanelets that the trajectory actually passes through
  std::vector<lanelet::ConstLanelet> trajectory_lanelets;

  for (const auto & lanelet : candidate_lanelets) {
    // Check subtype = "road"
    const bool is_road =
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
    if (!is_road) {
      continue;
    }

    // Check if trajectory intersects lanelet polygon
    if (!boost::geometry::disjoint(trajectory_ls, lanelet.polygon2d().basicPolygon())) {
      trajectory_lanelets.push_back(lanelet);
    }
  }

  RCLCPP_INFO(
    get_node_ptr()->get_logger(),
    "MPT Optimizer: Found %zu trajectory lanelets that intersect (from %zu candidates)",
    trajectory_lanelets.size(), candidate_lanelets.size());

  if (trajectory_lanelets.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: No lanelets found that intersect trajectory");
    return bounds;
  }

  // Step 4: Build DrivableLanes structure - start with all trajectory lanelets
  // Then expand to include same-direction neighbors
  std::vector<lanelet::ConstLanelet> same_direction_lanelets = trajectory_lanelets;

  // Step 5: Recursively find all adjacent same-direction lanelets
  // Keep expanding left and right until no more neighbors found
  bool found_new_neighbor = true;
  while (found_new_neighbor) {
    found_new_neighbor = false;

    for (const auto & candidate : candidate_lanelets) {
      const bool is_road =
        candidate.hasAttribute(lanelet::AttributeName::Subtype) &&
        candidate.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
      if (!is_road) {
        continue;
      }

      // Check if already added
      bool already_added = false;
      for (const auto & existing : same_direction_lanelets) {
        if (existing.id() == candidate.id()) {
          already_added = true;
          break;
        }
      }
      if (already_added) {
        continue;
      }

      // Check if this candidate shares a boundary with any existing lanelet
      for (const auto & existing : same_direction_lanelets) {
        // Left neighbor: candidate's right bound = existing's left bound
        if (candidate.rightBound().id() == existing.leftBound().id()) {
          same_direction_lanelets.push_back(candidate);
          found_new_neighbor = true;
          break;
        }
        // Right neighbor: candidate's left bound = existing's right bound
        if (candidate.leftBound().id() == existing.rightBound().id()) {
          same_direction_lanelets.push_back(candidate);
          found_new_neighbor = true;
          break;
        }
      }
    }
  }

  // Step 6: For each trajectory point, find leftmost/rightmost lanelets, then get bounds
  // This ensures we get the WIDEST bounds, not inner boundaries
  bounds.left_bound.reserve(traj_points.size());
  bounds.right_bound.reserve(traj_points.size());

  for (const auto & traj_point : traj_points) {
    // Find which lanelets contain this trajectory point
    std::vector<lanelet::ConstLanelet> containing_lanelets;
    const autoware_utils::Point2d traj_pt_2d(
      traj_point.pose.position.x, traj_point.pose.position.y);

    for (const auto & ll : same_direction_lanelets) {
      if (boost::geometry::within(traj_pt_2d, ll.polygon2d().basicPolygon())) {
        containing_lanelets.push_back(ll);
      }
    }

    // If no lanelet contains this point, find closest lanelet
    if (containing_lanelets.empty()) {
      double min_dist = std::numeric_limits<double>::max();
      for (const auto & ll : same_direction_lanelets) {
        const auto centerline = ll.centerline();
        for (const auto & cl_pt : centerline) {
          const double dist = std::hypot(
            cl_pt.x() - traj_point.pose.position.x, cl_pt.y() - traj_point.pose.position.y);
          if (dist < min_dist) {
            min_dist = dist;
            if (containing_lanelets.empty() || containing_lanelets.back().id() != ll.id()) {
              containing_lanelets.clear();
              containing_lanelets.push_back(ll);
            }
          }
        }
      }
    }

    if (containing_lanelets.empty()) {
      continue;
    }

    // Find leftmost and rightmost lanelets at this point
    lanelet::ConstLanelet leftmost_ll = containing_lanelets.front();
    lanelet::ConstLanelet rightmost_ll = containing_lanelets.front();

    for (const auto & ll : containing_lanelets) {
      // Check if ll has no left neighbor among containing lanelets
      bool has_left_neighbor = false;
      for (const auto & other : containing_lanelets) {
        if (other.id() != ll.id() && other.rightBound().id() == ll.leftBound().id()) {
          has_left_neighbor = true;
          break;
        }
      }
      if (!has_left_neighbor) {
        leftmost_ll = ll;
      }

      // Check if ll has no right neighbor among containing lanelets
      bool has_right_neighbor = false;
      for (const auto & other : containing_lanelets) {
        if (other.id() != ll.id() && other.leftBound().id() == ll.rightBound().id()) {
          has_right_neighbor = true;
          break;
        }
      }
      if (!has_right_neighbor) {
        rightmost_ll = ll;
      }
    }

    // Get closest point on leftmost lanelet's left bound
    const auto left_bound_3d = leftmost_ll.leftBound();
    double min_left_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point closest_left_point;
    for (const auto & bound_pt : left_bound_3d) {
      const double dist = std::hypot(
        bound_pt.x() - traj_point.pose.position.x, bound_pt.y() - traj_point.pose.position.y);
      if (dist < min_left_dist) {
        min_left_dist = dist;
        closest_left_point = lanelet::utils::conversion::toGeomMsgPt(bound_pt);
      }
    }

    // Get closest point on rightmost lanelet's right bound
    const auto right_bound_3d = rightmost_ll.rightBound();
    double min_right_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point closest_right_point;
    for (const auto & bound_pt : right_bound_3d) {
      const double dist = std::hypot(
        bound_pt.x() - traj_point.pose.position.x, bound_pt.y() - traj_point.pose.position.y);
      if (dist < min_right_dist) {
        min_right_dist = dist;
        closest_right_point = lanelet::utils::conversion::toGeomMsgPt(bound_pt);
      }
    }

    // Add points with deduplication
    if (
      bounds.left_bound.empty() ||
      std::hypot(
        closest_left_point.x - bounds.left_bound.back().x,
        closest_left_point.y - bounds.left_bound.back().y) > overlap_threshold) {
      bounds.left_bound.push_back(closest_left_point);
    }

    if (
      bounds.right_bound.empty() ||
      std::hypot(
        closest_right_point.x - bounds.right_bound.back().x,
        closest_right_point.y - bounds.right_bound.back().y) > overlap_threshold) {
      bounds.right_bound.push_back(closest_right_point);
    }
  }

  RCLCPP_INFO(
    get_node_ptr()->get_logger(),
    "MPT Optimizer: Generated bounds from %zu trajectory lanelets + %zu neighbors = %zu total, "
    "left=%zu pts, right=%zu pts",
    trajectory_lanelets.size(), same_direction_lanelets.size() - trajectory_lanelets.size(),
    same_direction_lanelets.size(), bounds.left_bound.size(), bounds.right_bound.size());

  return bounds;
}

void TrajectoryMPTOptimizer::publish_bounds_markers(const BoundsPair & bounds) const
{
  using autoware_utils::create_default_marker;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;
  using visualization_msgs::msg::Marker;
  using visualization_msgs::msg::MarkerArray;

  MarkerArray marker_array;

  if (bounds.left_bound.empty() || bounds.right_bound.empty()) {
    RCLCPP_WARN(get_node_ptr()->get_logger(), "MPT Optimizer: Cannot publish empty bounds!");
    return;
  }

  // Left bound marker (green)
  auto left_marker = create_default_marker(
    "map", get_node_ptr()->now(), "mpt_left_bound", 0, Marker::LINE_STRIP,
    create_marker_scale(0.15, 0.0, 0.0), create_marker_color(0.0, 1.0, 0.0, 0.99));
  left_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  left_marker.points = bounds.left_bound;
  marker_array.markers.push_back(left_marker);

  // Right bound marker (red)
  auto right_marker = create_default_marker(
    "map", get_node_ptr()->now(), "mpt_right_bound", 1, Marker::LINE_STRIP,
    create_marker_scale(0.15, 0.0, 0.0), create_marker_color(1.0, 0.0, 0.0, 0.99));
  right_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  right_marker.points = bounds.right_bound;
  marker_array.markers.push_back(right_marker);

  RCLCPP_INFO(
    get_node_ptr()->get_logger(),
    "Publishing bounds markers: left=%zu pts (%.2f,%.2f)->(%.2f,%.2f), right=%zu pts "
    "(%.2f,%.2f)->(%.2f,%.2f)",
    bounds.left_bound.size(), bounds.left_bound.front().x, bounds.left_bound.front().y,
    bounds.left_bound.back().x, bounds.left_bound.back().y, bounds.right_bound.size(),
    bounds.right_bound.front().x, bounds.right_bound.front().y, bounds.right_bound.back().x,
    bounds.right_bound.back().y);

  debug_markers_pub_->publish(marker_array);
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
      "MPT Optimizer: Trajectory too short (< %zu points), skipping optimization",
      min_points_for_optimization);
    return;
  }

  // Generate bounds from lanelet map
  const auto bounds = generate_bounds_from_lanelet_map(traj_points);

  // If bounds extraction failed, skip optimization
  if (bounds.left_bound.empty() || bounds.right_bound.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: Failed to generate bounds from lanelet map, skipping optimization");
    return;
  }

  // Publish debug markers for visualization
  publish_bounds_markers(bounds);

  // Create planner data
  const auto planner_data = create_planner_data(traj_points, bounds, data);

  // Debug logging (only at DEBUG level)
  RCLCPP_DEBUG(
    get_node_ptr()->get_logger(),
    "MPT Optimizer Input: traj_points=%zu, left_bound=%zu, right_bound=%zu, ego_vel=%.2f m/s",
    planner_data.traj_points.size(), planner_data.left_bound.size(),
    planner_data.right_bound.size(), planner_data.ego_vel);

  // Store original size for validation
  const size_t original_size = traj_points.size();

  // Reset previous optimization data since diffusion planner generates new trajectories each cycle
  // Without this, MPT warm start uses stale data from previous (different) trajectories
  mpt_optimizer_ptr_->resetPreviousData();

  // Run MPT optimization
  const auto optimized_traj = mpt_optimizer_ptr_->optimizeTrajectory(planner_data);

  // Apply optimized trajectory if successful with validation
  if (optimized_traj) {
    // Validate optimized trajectory
    if (optimized_traj->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
        "MPT Optimizer: Returned empty trajectory, keeping original");
      return;
    }

    if (optimized_traj->size() < min_points_for_optimization) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
        "MPT Optimizer: Returned trajectory too short (%zu < %zu), keeping original",
        optimized_traj->size(), min_points_for_optimization);
      return;
    }

    // Log output trajectory info at DEBUG level
    RCLCPP_DEBUG(
      get_node_ptr()->get_logger(), "MPT Output: Successfully optimized %zu → %zu points",
      original_size, optimized_traj->size());

    traj_points = *optimized_traj;

    // CRITICAL: MPT does not set acceleration field, recalculate from velocities
    // Without this, acceleration will be 0.0 and vehicle won't move!
    autoware::trajectory_optimizer::utils::recalculate_longitudinal_acceleration(
      traj_points, false);
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: Optimization failed, keeping original trajectory");
  }
}

BoundsPair TrajectoryMPTOptimizer::generate_bounds_from_trajectory(
  const TrajectoryPoints & traj_points, const double lateral_offset_m) const
{
  BoundsPair bounds;
  bounds.left_bound.reserve(traj_points.size());
  bounds.right_bound.reserve(traj_points.size());

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & point = traj_points[i];
    const auto & pose = point.pose;

    // Get yaw from quaternion
    const double yaw = tf2::getYaw(pose.orientation);

    // Calculate perpendicular offset (90 degrees to the left and right)
    const double left_yaw = yaw + M_PI_2;   // Left is +90 degrees
    const double right_yaw = yaw - M_PI_2;  // Right is -90 degrees

    // Left bound point
    geometry_msgs::msg::Point left_point;
    left_point.x = pose.position.x + lateral_offset_m * std::cos(left_yaw);
    left_point.y = pose.position.y + lateral_offset_m * std::sin(left_yaw);
    left_point.z = pose.position.z;
    bounds.left_bound.push_back(left_point);

    // Right bound point
    geometry_msgs::msg::Point right_point;
    right_point.x = pose.position.x + lateral_offset_m * std::cos(right_yaw);
    right_point.y = pose.position.y + lateral_offset_m * std::sin(right_yaw);
    right_point.z = pose.position.z;
    bounds.right_bound.push_back(right_point);
  }

  return bounds;
}

autoware::path_optimizer::PlannerData TrajectoryMPTOptimizer::create_planner_data(
  const TrajectoryPoints & traj_points, const BoundsPair & bounds,
  const TrajectoryOptimizerData & data) const
{
  autoware::path_optimizer::PlannerData planner_data;

  // Create header from odometry frame (ego state is in this frame)
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

}  // namespace autoware::trajectory_optimizer::plugin
