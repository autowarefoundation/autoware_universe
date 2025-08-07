// Copyright 2022 TIER IV, Inc.
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

// cspell:ignore Kashi Al-Kashi
#include "autoware/behavior_path_start_planner_module/clothoid_pull_out.hpp"

#include "autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_start_planner_module/data_structs.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using autoware::motion_utils::findNearestIndex;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_offset_pose;
using lanelet::utils::getArcCoordinates;
namespace autoware::behavior_path_planner
{
using autoware::universe_utils::normalizeRadian;
using autoware_utils::deg2rad;
using autoware_utils::rad2deg;
using start_planner_utils::get_lane_ids_from_pose;
using start_planner_utils::getPullOutLanes;
using start_planner_utils::print_path_with_lane_id_details;
using start_planner_utils::set_lane_ids_to_path_point;

std::vector<geometry_msgs::msg::Point> correct_clothoid_by_rigid_transform(
  const std::vector<geometry_msgs::msg::Point> & clothoid_points,
  const ArcSegment & original_segment, const geometry_msgs::msg::Pose & start_pose)
{
  if (clothoid_points.size() < 2) {
    return clothoid_points;
  }

  const auto clothoid_start = clothoid_points.front();
  const auto clothoid_end = clothoid_points.back();

  // Get target start and end positions
  const auto target_start = start_pose.position;
  const auto target_end = original_segment.getPointAtAngle(original_segment.calculateEndAngle());

  // Calculate direction vectors
  const double clothoid_dx = clothoid_end.x - clothoid_start.x;
  const double clothoid_dy = clothoid_end.y - clothoid_start.y;
  const double clothoid_length = std::sqrt(clothoid_dx * clothoid_dx + clothoid_dy * clothoid_dy);

  const double target_dx = target_end.x - target_start.x;
  const double target_dy = target_end.y - target_start.y;
  const double target_length = std::sqrt(target_dx * target_dx + target_dy * target_dy);

  // Calculate scaling factor
  const double scale_factor = (clothoid_length > 1e-10) ? target_length / clothoid_length : 1.0;

  // Calculate rotation angle
  const double clothoid_angle = std::atan2(clothoid_dy, clothoid_dx);
  const double target_angle = std::atan2(target_dy, target_dx);
  double rotation_angle = target_angle - clothoid_angle;

  rotation_angle = normalizeRadian(rotation_angle);

  // Choose shorter rotation if over 180 degrees
  if (std::abs(rotation_angle) > M_PI) {
    rotation_angle = (rotation_angle > 0) ? rotation_angle - 2 * M_PI : rotation_angle + 2 * M_PI;
  }

  // Calculate transformation matrix elements
  const double cos_theta = std::cos(rotation_angle);
  const double sin_theta = std::sin(rotation_angle);

  // Apply rigid transformation
  std::vector<geometry_msgs::msg::Point> corrected_points;
  corrected_points.reserve(clothoid_points.size());

  for (size_t i = 0; i < clothoid_points.size(); ++i) {
    geometry_msgs::msg::Point corrected_point;

    // Move to origin
    double rel_x = clothoid_points[i].x - clothoid_start.x;
    double rel_y = clothoid_points[i].y - clothoid_start.y;

    // Scale
    rel_x *= scale_factor;
    rel_y *= scale_factor;

    // Rotate
    double rotated_x = cos_theta * rel_x - sin_theta * rel_y;
    double rotated_y = sin_theta * rel_x + cos_theta * rel_y;

    // Translate to target start
    corrected_point.x = rotated_x + target_start.x;
    corrected_point.y = rotated_y + target_start.y;
    corrected_point.z = clothoid_points[i].z;  // Keep Z coordinate as is

    corrected_points.push_back(corrected_point);
  }

  return corrected_points;
}

std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_clothoid_entry_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  const double A = segment.A;
  const double L = segment.L;
  const double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  const double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Pose> poses;

  // Entry Clothoid: linearly increase curvature from 0 to target curvature
  const double target_curvature = (L / (A * A)) * direction_factor;
  const double start_curvature = 0.0;

  // Accurate calculation using numerical integration
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    // Create current point and add to poses
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_x;
    pose.position.y = current_y;
    pose.position.z = 0.0;  // This is temporarily set to 0.0. The z value will be overwritten from
                            // the lanelet when generating the final path.
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);
    poses.push_back(pose);

    // If not the last point, perform integration calculation to the next point
    if (i < num_points - 1) {
      // Infinitesimal interval to next point
      double ds = L / (num_points - 1);

      // Calculate curvature at current position
      double progress = static_cast<double>(i) / (num_points - 1);
      double current_curvature = start_curvature + (target_curvature - start_curvature) * progress;

      // Update coordinates using numerical integration
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // Create terminal state
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = 0.0;  // This is temporarily set to 0.0. The z value will be overwritten
                              // from the lanelet when generating the final path.
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

  return {poses, end_pose};
}

std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_circular_segment_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double radius = segment.radius;
  double angle = segment.angle;
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;
  double start_yaw = tf2::getYaw(start_pose.orientation);

  std::vector<geometry_msgs::msg::Pose> poses;

  // Calculate arc center
  double center_x = start_pose.position.x - radius * std::sin(start_yaw) * direction_factor;
  double center_y = start_pose.position.y + radius * std::cos(start_yaw) * direction_factor;

  for (int i = 0; i < num_points; ++i) {
    double progress = static_cast<double>(i) / (num_points - 1);
    double angle_progress = angle * progress * direction_factor;
    double current_psi = start_yaw + angle_progress;

    // Calculate position on arc (angle from center)
    double angle_from_center = current_psi - M_PI / 2.0 * direction_factor;

    geometry_msgs::msg::Pose pose;
    pose.position.x = center_x + radius * std::cos(angle_from_center);
    pose.position.y = center_y + radius * std::sin(angle_from_center);
    pose.position.z = 0.0;
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

    poses.push_back(pose);
  }

  // Terminal state (final yaw angle)
  double final_psi = start_yaw + angle * direction_factor;

  geometry_msgs::msg::Pose end_pose;
  end_pose.position = poses.back().position;
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(final_psi);

  return {poses, end_pose};
}

std::pair<std::vector<geometry_msgs::msg::Pose>, geometry_msgs::msg::Pose>
generate_clothoid_exit_with_yaw(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points)
{
  double L = segment.L;
  double start_yaw = tf2::getYaw(start_pose.orientation);
  double direction_factor = segment.is_clockwise ? -1.0 : 1.0;

  std::vector<geometry_msgs::msg::Pose> poses;

  // Calculate curvature of previous segment (arc) considering rotation direction
  double start_curvature = (1.0 / segment.radius) * direction_factor;

  // Accurate calculation using numerical integration
  double current_x = start_pose.position.x;
  double current_y = start_pose.position.y;
  double current_psi = start_yaw;

  for (int i = 0; i < num_points; ++i) {
    // Create current point and add to poses
    geometry_msgs::msg::Pose pose;
    pose.position.x = current_x;
    pose.position.y = current_y;
    pose.position.z = 0.0;
    pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);
    poses.push_back(pose);

    // If not the last point, perform integration calculation to the next point
    if (i < num_points - 1) {
      // Infinitesimal interval to next point
      double ds = L / (num_points - 1);

      // Calculate curvature at current position (Exit Clothoid: linearly decrease from start
      // curvature to 0)
      double progress = static_cast<double>(i) / (num_points - 1);
      double current_curvature = start_curvature * (1.0 - progress);

      // Update coordinates using numerical integration
      current_x += std::cos(current_psi) * ds;
      current_y += std::sin(current_psi) * ds;
      current_psi += current_curvature * ds;
    }
  }

  // Create terminal state
  geometry_msgs::msg::Pose end_pose;
  end_pose.position.x = current_x;
  end_pose.position.y = current_y;
  end_pose.position.z = 0.0;  // This is temporarily set to 0.0. The z value will be overwritten
                              // from the lanelet when generating the final path.
  end_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(current_psi);

  return {poses, end_pose};
}

std::vector<geometry_msgs::msg::Point> generate_clothoid_path(
  const std::vector<ClothoidSegment> & segments, double point_interval,
  const geometry_msgs::msg::Pose & start_pose)
{
  // Early return if segments are empty
  if (segments.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("ClothoidPullOut"),
      "No clothoid segments provided to generate_clothoid_path");
    return {};
  }

  // Calculate theoretical arc length for each segment
  std::vector<double> theoretical_lengths;
  for (const auto & segment : segments) {
    if (
      segment.type == ClothoidSegment::CLOTHOID_ENTRY ||
      segment.type == ClothoidSegment::CLOTHOID_EXIT) {
      theoretical_lengths.push_back(segment.L);
    } else if (segment.type == ClothoidSegment::CIRCULAR_ARC) {
      theoretical_lengths.push_back(segment.radius * segment.angle);
    }
  }

  // Calculate number of points per segment from arc length and specified interval
  std::vector<int> segment_points;
  for (double length : theoretical_lengths) {
    int points = std::max(2, static_cast<int>(std::ceil(length / point_interval)) + 1);
    segment_points.push_back(points);
  }

  // Set initial state
  geometry_msgs::msg::Pose current_pose = start_pose;

  std::vector<geometry_msgs::msg::Point> all_points;

  for (size_t i = 0; i < segments.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> segment_poses_vec;
    geometry_msgs::msg::Pose end_pose;

    int num_points = segment_points[i];

    if (segments[i].type == ClothoidSegment::CLOTHOID_ENTRY) {
      auto result = generate_clothoid_entry_with_yaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CIRCULAR_ARC) {
      auto result = generate_circular_segment_with_yaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    } else if (segments[i].type == ClothoidSegment::CLOTHOID_EXIT) {
      auto result = generate_clothoid_exit_with_yaw(segments[i], current_pose, num_points);
      segment_poses_vec = result.first;
      end_pose = result.second;
    }

    // Combine avoiding duplicate points (extract coordinates only from Pose)
    size_t start_idx = (all_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < segment_poses_vec.size(); ++j) {
      all_points.push_back(segment_poses_vec[j].position);
    }

    current_pose = end_pose;
  }

  return all_points;
}

std::optional<std::vector<geometry_msgs::msg::Point>> convert_arc_to_clothoid(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, double point_interval)
{
  // Extract arc information
  double start_angle = arc_segment.calculateStartAngle();
  double end_angle = arc_segment.calculateEndAngle();

  double total_angle = std::abs(end_angle - start_angle);

  if (total_angle > M_PI) {
    total_angle = 2.0 * M_PI - total_angle;
  }

  double radius = arc_segment.radius;
  bool is_clockwise = arc_segment.is_clockwise;

  // Clothoid parameters
  double A = A_min;
  double L = L_min;
  double alpha_clothoid = (L * L) / (2.0 * A * A);  // Angle change of single clothoid

  RCLCPP_DEBUG(
    rclcpp::get_logger("ClothoidPullOut"),
    "Clothoid parameters: radius=%.3f, A_min=%.3f, L_min=%.3f, total_angle=%.3f°, "
    "alpha_clothoid=%.3f°",
    radius, A_min, L_min, total_angle * 180.0 / M_PI, alpha_clothoid * 180.0 / M_PI);

  std::vector<ClothoidSegment> segments;

  if (total_angle >= 2.0 * alpha_clothoid) {
    // Case A: CAC(A, L, θ)
    double theta_arc = total_angle - 2.0 * alpha_clothoid;

    // Entry clothoid
    ClothoidSegment entry(ClothoidSegment::CLOTHOID_ENTRY, A, L);
    entry.radius = radius;
    entry.is_clockwise = is_clockwise;
    entry.description = "Entry clothoid (κ: 0 → 1/R)";
    segments.push_back(entry);

    // Circular arc segment
    ClothoidSegment circular(ClothoidSegment::CIRCULAR_ARC);
    circular.radius = radius;
    circular.angle = theta_arc;
    circular.is_clockwise = is_clockwise;
    circular.description = "Circular arc (κ = 1/R = " + std::to_string(1.0 / radius) + ")";
    segments.push_back(circular);

    // Exit clothoid
    ClothoidSegment exit(ClothoidSegment::CLOTHOID_EXIT, A, L);
    exit.radius = radius;
    exit.is_clockwise = is_clockwise;
    exit.description = "Exit clothoid (κ: 1/R → 0)";
    segments.push_back(exit);
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Case B is not implemented. Please use Case A conditions.");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Current parameters: total_angle=%.3f°, alpha_clothoid=%.3f°, required: total_angle >= %.3f°",
      total_angle * 180.0 / M_PI, alpha_clothoid * 180.0 / M_PI,
      2.0 * alpha_clothoid * 180.0 / M_PI);

    // Return failure if Case B is not implemented
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "Clothoid conversion failed! Case B not implemented.");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Arc segment: radius=%.3f, center=(%.3f, %.3f), is_clockwise=%s", arc_segment.radius,
      arc_segment.center.x, arc_segment.center.y, arc_segment.is_clockwise ? "true" : "false");
    return std::nullopt;
  }

  // Generate clothoid path
  std::vector<geometry_msgs::msg::Point> clothoid_path =
    generate_clothoid_path(segments, point_interval, start_pose);

  // Return failure if generated path is empty
  if (clothoid_path.empty()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Clothoid conversion failed! Generated path is empty.");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Arc segment: radius=%.3f, center=(%.3f, %.3f), is_clockwise=%s", arc_segment.radius,
      arc_segment.center.x, arc_segment.center.y, arc_segment.is_clockwise ? "true" : "false");
    return std::nullopt;
  }

  return clothoid_path;
}

std::optional<std::vector<geometry_msgs::msg::Point>> convert_arc_to_clothoid_with_correction(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose,
  double initial_velocity, double wheel_base, double max_steer_angle_rate, double point_interval)
{
  // Calculate minimum radius (from arc_segment)
  const double minimum_radius = arc_segment.radius;

  // Calculate optimal clothoid parameters from vehicle parameters
  const double circular_steer_angle = std::atan(wheel_base / minimum_radius);
  const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
  const double L_min = initial_velocity * minimum_steer_time;
  const double A_min = std::sqrt(minimum_radius * L_min);

  RCLCPP_DEBUG(
    rclcpp::get_logger("ClothoidPullOut"),
    "Clothoid parameters: radius=%.3f, A_min=%.3f, L_min=%.3f, velocity=%.3f", minimum_radius,
    A_min, L_min, initial_velocity);

  // Execute original clothoid conversion
  auto clothoid_points_opt =
    convert_arc_to_clothoid(arc_segment, start_pose, A_min, L_min, point_interval);

  if (!clothoid_points_opt) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Clothoid conversion failed! Check parameters and arc segment validity.");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Arc segment: radius=%.3f, center=(%.3f, %.3f), is_clockwise=%s", arc_segment.radius,
      arc_segment.center.x, arc_segment.center.y, arc_segment.is_clockwise ? "true" : "false");
    return std::nullopt;
  }

  // Apply endpoint correction
  auto corrected_points =
    correct_clothoid_by_rigid_transform(*clothoid_points_opt, arc_segment, start_pose);

  return corrected_points;
}

std::optional<PathWithLaneId> create_path_with_lane_id_from_clothoid_paths(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & clothoid_paths, double velocity,
  double target_velocity, double acceleration, const lanelet::ConstLanelets & search_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  // Return nullopt if clothoid paths are empty
  if (clothoid_paths.empty()) {
    return std::nullopt;
  }

  // Combine all clothoid paths
  std::vector<geometry_msgs::msg::Point> all_clothoid_points;
  for (const auto & path : clothoid_paths) {
    // Process only if path is not empty
    if (path.empty()) {
      continue;
    }

    // Skip first point for paths other than the first (avoid duplication)
    // Is it okay to skip?
    size_t start_idx = (all_clothoid_points.empty()) ? 0 : 1;
    for (size_t j = start_idx; j < path.size(); ++j) {
      all_clothoid_points.push_back(path[j]);
    }
  }

  // Return nullopt if still empty after combination
  if (all_clothoid_points.empty()) {
    return std::nullopt;
  }

  // Create PathWithLaneId
  PathWithLaneId path_with_lane_id;
  path_with_lane_id.header = route_handler->getRouteHeader();

  // Convert each coordinate point to PathPointWithLaneId
  for (size_t i = 0; i < all_clothoid_points.size(); ++i) {
    PathPointWithLaneId path_point;

    // Set coordinates
    path_point.point.pose.position = all_clothoid_points[i];

    // Set z coordinate: get z value of closest point from lanelet information
    if (!search_lanes.empty()) {
      // Find closest lanelet
      lanelet::Lanelet closest_lanelet;
      if (lanelet::utils::query::getClosestLanelet(
            search_lanes, path_point.point.pose, &closest_lanelet)) {
        // Get z value of closest point from lanelet centerline
        const auto centerline = closest_lanelet.centerline();
        if (!centerline.empty()) {
          double min_distance = std::numeric_limits<double>::max();
          double closest_z = all_clothoid_points[i].z;  // Default is original z value

          for (const auto & point : centerline) {
            const double distance = std::hypot(
              point.x() - all_clothoid_points[i].x, point.y() - all_clothoid_points[i].y);
            if (distance < min_distance) {
              min_distance = distance;
              closest_z = point.z();
            }
          }
          path_point.point.pose.position.z = closest_z;
        }
      }
    }

    // Calculate velocity profile (accelerate with constant acceleration)
    double current_velocity;
    if (i == 0) {
      // First point uses initial velocity
      current_velocity = velocity;
    } else {
      // Calculate accumulated distance
      double accumulated_distance = 0.0;
      for (size_t j = 0; j < i; ++j) {
        const double dx = all_clothoid_points[j + 1].x - all_clothoid_points[j].x;
        const double dy = all_clothoid_points[j + 1].y - all_clothoid_points[j].y;
        accumulated_distance += std::sqrt(dx * dx + dy * dy);
      }

      // Uniform acceleration formula: v^2 = v0^2 + 2*a*s
      // But limit to not exceed target velocity
      double calculated_velocity =
        std::sqrt(velocity * velocity + 2.0 * acceleration * accumulated_distance);
      current_velocity = std::min(calculated_velocity, target_velocity);
    }

    // Set velocity
    path_point.point.longitudinal_velocity_mps = current_velocity;
    path_point.point.lateral_velocity_mps = 0.0;
    path_point.point.heading_rate_rps = 0.0;
    path_point.point.is_final = false;

    // Set lane ID
    std::vector<int64_t> previous_lane_ids;
    if (i > 0) {
      previous_lane_ids = path_with_lane_id.points[i - 1].lane_ids;
    }
    set_lane_ids_to_path_point(path_point, search_lanes, previous_lane_ids);

    path_with_lane_id.points.push_back(path_point);
  }

  return path_with_lane_id;
}

PathWithLaneId combine_path_with_centerline(
  const PathWithLaneId & clothoid_path, const PathWithLaneId & centerline_path,
  const geometry_msgs::msg::Pose & target_pose)
{
  // Return clothoid path as is if centerline path is empty
  if (centerline_path.points.empty()) {
    return clothoid_path;
  }

  // Find connection point from centerline_path at target_pose position
  const auto target_idx =
    autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);

  // Get centerline path from target_pose onwards
  if (target_idx < centerline_path.points.size()) {
    PathWithLaneId centerline_extension;
    centerline_extension.header = centerline_path.header;
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "target_pose: %.3f, %.3f", target_pose.position.x,
      target_pose.position.y);
    // Add points from target_pose onwards to centerline_extension
    // TODO(Sugahara): Check extension length
    for (size_t i = target_idx; i < centerline_path.points.size(); ++i) {
      centerline_extension.points.push_back(centerline_path.points[i]);
    }

    // If centerline extension exists, combine with existing path
    if (!centerline_extension.points.empty()) {
      // Combine avoiding duplicate points
      return utils::combinePath(clothoid_path, centerline_extension);
    }
  }
  return clothoid_path;
}

std::vector<geometry_msgs::msg::Pose> create_straight_path_to_end_pose(
  const geometry_msgs::msg::Pose & start_pose, double forward_distance, double backward_distance,
  double point_interval)
{
  // Calculate straight direction (yaw angle)
  const double start_yaw = tf2::getYaw(start_pose.orientation);

  // Store pose array
  std::vector<geometry_msgs::msg::Pose> poses;

  // 1. Generate backward path (generate from farthest backward point in order)
  if (backward_distance > 0.0) {
    const int backward_num_points = static_cast<int>(backward_distance / point_interval);

    for (int i = backward_num_points; i >= 1; --i) {
      geometry_msgs::msg::Pose pose;
      double distance = i * point_interval;
      pose.position.x = start_pose.position.x - distance * std::cos(start_yaw);
      pose.position.y = start_pose.position.y - distance * std::sin(start_yaw);
      pose.position.z = start_pose.position.z;
      pose.orientation = start_pose.orientation;
      poses.push_back(pose);
    }
  }

  // 2. Add start_pose
  poses.push_back(start_pose);

  // 3. Generate forward path
  if (forward_distance > 0.0) {
    // Calculate forward end pose
    geometry_msgs::msg::Pose forward_end_pose = start_pose;
    forward_end_pose.position.x = start_pose.position.x + forward_distance * std::cos(start_yaw);
    forward_end_pose.position.y = start_pose.position.y + forward_distance * std::sin(start_yaw);
    forward_end_pose.orientation = start_pose.orientation;

    // Calculate forward point count (exclude start_pose as it's already added)
    const int forward_num_points =
      std::max(1, static_cast<int>(std::ceil(forward_distance / point_interval)));

    // Recalculate actual interval (for equal spacing)
    const double actual_interval = forward_distance / forward_num_points;

    // Generate each forward point (after start_pose)
    for (int i = 1; i <= forward_num_points; ++i) {
      geometry_msgs::msg::Pose pose;

      if (i == forward_num_points) {
        // Final point (exactly forward_end_pose)
        pose = forward_end_pose;
      } else {
        // Intermediate point
        const double distance = i * actual_interval;
        pose.position.x = start_pose.position.x + distance * std::cos(start_yaw);
        pose.position.y = start_pose.position.y + distance * std::sin(start_yaw);
        pose.position.z = start_pose.position.z;
        pose.orientation = start_pose.orientation;
      }

      poses.push_back(pose);
    }
  }

  return poses;
}

double calc_necessary_longitudinal_distance(
  const double minimum_radius, const double initial_velocity, const double wheel_base,
  const double max_steer_angle_rate, const PathWithLaneId & centerline_path,
  const geometry_msgs::msg::Pose & start_pose)
{
  // TODO(Sugahara): This list of trial_distances is a temporary implementation. Define appropriate
  // candidate distances based on proper criteria.
  const std::vector<double> trial_distances = {
    0.5 * minimum_radius, 0.75 * minimum_radius, 1.0 * minimum_radius, 1.5 * minimum_radius,
    2.0 * minimum_radius, 3.0 * minimum_radius,  4.0 * minimum_radius, 5.0 * minimum_radius,
    6.0 * minimum_radius, 8.0 * minimum_radius,  10.0 * minimum_radius};

  // Results tracking
  std::vector<std::pair<double, double>> evaluation_results;
  evaluation_results.reserve(trial_distances.size());

  double best_distance = 0.0;
  double best_score = -1e9;  // Prioritize longer circular segments
  bool found_valid = false;
  int valid_results_count = 0;

  // Define lateral error threshold
  const double lateral_error_threshold = 0.5;  // 0.5 meters tolerance

  for (const double trial_distance : trial_distances) {
    // Get target pose using find_target_pose_along_path
    const geometry_msgs::msg::Pose target_pose =
      start_planner_utils::find_target_pose_along_path(centerline_path, start_pose, trial_distance);

    // Calculate relative pose information
    // TODO(Sugahara): If lateral_offset is positive here, straight path is fine.
    const auto relative_pose_info =
      start_planner_utils::calculate_relative_pose_in_vehicle_coordinate(start_pose, target_pose);

    // Generate circular path using calc_circular_path
    const auto circular_path_opt = calc_circular_path(
      start_pose, relative_pose_info.longitudinal_distance_vehicle,
      relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

    // Check if circular path generation failed
    if (!circular_path_opt) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("ClothoidPullOut"),
        "Trial distance: %.2f m - Circular path generation failed", trial_distance);
      continue;
    }

    const auto & circular_path = *circular_path_opt;

    const auto & arc1 = circular_path.segments[0];
    const auto & arc2 = circular_path.segments[1];

    // Calculate arc lengths using the calculateArcLength method
    const double arc1_length = arc1.calculateArcLength();
    const double arc2_length = arc2.calculateArcLength();

    // Calculate angle differences using calculateStartAngle and calculateEndAngle
    const double start_angle1 = arc1.calculateStartAngle();
    const double end_angle1 = arc1.calculateEndAngle();
    const double start_angle2 = arc2.calculateStartAngle();
    const double end_angle2 = arc2.calculateEndAngle();

    // Calculate angle differences with proper direction adjustment
    const double total_angle1 = std::abs(end_angle1 - start_angle1);

    const double total_angle2 = std::abs(end_angle2 - start_angle2);

    // Calculate clothoid parameters for each arc based on their respective radii
    // Arc1 clothoid parameters (based on arc1.radius)
    const double circular_steer_angle1 = std::atan(wheel_base / arc1.radius);
    const double minimum_steer_time1 = circular_steer_angle1 / max_steer_angle_rate;
    const double L_min1 = initial_velocity * minimum_steer_time1;
    const double A_min1 = std::sqrt(arc1.radius * L_min1);
    const double alpha_clothoid1 = (L_min1 * L_min1) / (2.0 * A_min1 * A_min1);

    // Arc2 clothoid parameters (based on arc2.radius)
    const double circular_steer_angle2 = std::atan(wheel_base / arc2.radius);
    const double minimum_steer_time2 = circular_steer_angle2 / max_steer_angle_rate;
    const double L_min2 = initial_velocity * minimum_steer_time2;
    const double A_min2 = std::sqrt(arc2.radius * L_min2);
    const double alpha_clothoid2 = (L_min2 * L_min2) / (2.0 * A_min2 * A_min2);

    // Check if arc lengths are sufficient for clothoid conversion
    const bool sufficient_for_clothoid1 = (total_angle1 >= 2.0 * alpha_clothoid1);
    const bool sufficient_for_clothoid2 = (total_angle2 >= 2.0 * alpha_clothoid2);

    // Calculate clothoid feasibility score for both arcs
    double clothoid_score1 = 0.0;
    double clothoid_score2 = 0.0;

    if (sufficient_for_clothoid1) {
      // CAC(A, L, θ) case: sufficient for entry + circular + exit clothoids
      const double circular_angle1 = total_angle1 - 2.0 * alpha_clothoid1;
      clothoid_score1 = circular_angle1;  // Prefer longer circular segments
    } else if (total_angle1 >= alpha_clothoid1) {
      // CA(A, L) or AC(A, L) case: only one clothoid segment
      clothoid_score1 = total_angle1 - alpha_clothoid1;
    } else {
      // Insufficient for any clothoid
      clothoid_score1 = -1.0;
    }

    if (sufficient_for_clothoid2) {
      // CAC(A, L, θ) case: sufficient for entry + circular + exit clothoids
      const double circular_angle2 = total_angle2 - 2.0 * alpha_clothoid2;
      clothoid_score2 = circular_angle2;  // Prefer longer circular segments
    } else if (total_angle2 >= alpha_clothoid2) {
      // CA(A, L) or AC(A, L) case: only one clothoid segment
      clothoid_score2 = total_angle2 - alpha_clothoid2;
    } else {
      // Insufficient for any clothoid
      clothoid_score2 = -1.0;
    }

    // Combined clothoid score (prioritize the worse case)
    const double combined_clothoid_score = std::min(clothoid_score1, clothoid_score2);

    // Calculate lateral error from the target lateral offset
    // Calculate difference between actual lateral distance and target value
    const double actual_lateral_offset = relative_pose_info.lateral_distance_vehicle;
    const double lateral_error = std::abs(actual_lateral_offset);
    const bool lateral_acceptable = (lateral_error <= lateral_error_threshold);

    // Store evaluation result for debugging
    evaluation_results.emplace_back(trial_distance, arc1_length);
    valid_results_count++;

    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "  Trial distance: %.2f m - Arc1 radius: %.3f m, Arc2 radius: %.3f m, Arc1 length: %.3f m, "
      "Arc2 length: %.3f m, Angle1: %.3f°, Angle2: %.3f°, Alpha1: %.3f°, Alpha2: %.3f°, "
      "Clothoid feasible: %s, Score: %.3f",
      trial_distance, arc1.radius, arc2.radius, arc1_length, arc2_length,
      total_angle1 * 180.0 / M_PI, total_angle2 * 180.0 / M_PI, alpha_clothoid1 * 180.0 / M_PI,
      alpha_clothoid2 * 180.0 / M_PI,
      (sufficient_for_clothoid1 && sufficient_for_clothoid2 ? "YES" : "NO"),
      combined_clothoid_score);

    if (lateral_acceptable) {
      if (combined_clothoid_score > best_score) {
        best_score = combined_clothoid_score;
        best_distance = trial_distance;
        found_valid = true;
      }
    } else if (!found_valid && combined_clothoid_score > best_score) {
      // If no acceptable solution found yet, select the best available
      best_score = combined_clothoid_score;
      best_distance = trial_distance;
    }
  }

  // Output selection results
  RCLCPP_DEBUG(rclcpp::get_logger("ClothoidPullOut"), "\n--- Selection Results ---");
  RCLCPP_DEBUG(rclcpp::get_logger("ClothoidPullOut"), "Valid results: %d", valid_results_count);
  RCLCPP_DEBUG(
    rclcpp::get_logger("ClothoidPullOut"), "Target lateral offset: %.3f", lateral_error_threshold);

  if (found_valid) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "Acceptable results (lateral error <= %.1f m): found",
      lateral_error_threshold);
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Selected result: Clothoid score = %.3f, Distance = %.3f m", best_score, best_distance);
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "No acceptable results found, using best available solution");
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Selected result: Clothoid score = %.3f, Distance = %.3f m", best_score, best_distance);
  }

  // Fallback if no valid solution found
  if (best_distance == 0.0) {
    // Use clothoid-based estimation
    // Get a target pose for relative pose calculation
    const geometry_msgs::msg::Pose fallback_target_pose =
      start_planner_utils::find_target_pose_along_path(
        centerline_path, start_pose, 4.0 * minimum_radius);
    const auto fallback_relative_pose_info =
      start_planner_utils::calculate_relative_pose_in_vehicle_coordinate(
        start_pose, fallback_target_pose);

    const double clothoid_based_distance = std::max(
      4.0 * minimum_radius,
      std::max(
        std::abs(fallback_relative_pose_info.lateral_distance_vehicle) * 3.0,
        std::abs(fallback_relative_pose_info.longitudinal_distance_vehicle) * 3.0));
    best_distance = clothoid_based_distance;
  }

  // TODO(Sugahara): The search logic for best_distance has not been sufficiently considered. Should
  // be improved.
  return best_distance;
}

std::optional<CompositeArcPath> calc_circular_path(
  const geometry_msgs::msg::Pose & start_pose, const double longitudinal_distance,
  const double lateral_distance, const double angle_diff, const double minimum_radius)
{
  const double PI = M_PI;

  // Calculate in relative coordinate system (origin at start point, X-axis as forward direction)
  // Start point: (0, 0, 0)
  // Goal point: (longitudinal_distance, lateral_distance, angle_diff)

  const double x_start_rel = 0.0;
  const double y_start_rel = 0.0;
  const double yaw_start_rel = 0.0;

  const double x_goal_rel = longitudinal_distance;
  const double y_goal_rel = lateral_distance;
  const double yaw_goal_rel = angle_diff;

  RCLCPP_DEBUG(
    rclcpp::get_logger("ClothoidPullOut"),
    "Relative coordinates - Start: (%.3f, %.3f), Goal: (%.3f, %.3f)", x_start_rel, y_start_rel,
    x_goal_rel, y_goal_rel);

  // Calculate starting arc center (assuming clockwise rotation)
  double C_rx_rel = x_start_rel + minimum_radius * std::sin(yaw_start_rel);
  double C_ry_rel = y_start_rel - minimum_radius * std::cos(yaw_start_rel);

  // Distance from goal point to starting arc center
  double dx_goal_rel = x_goal_rel - C_rx_rel;
  double dy_goal_rel = y_goal_rel - C_ry_rel;
  double d_goal_Cr_rel = std::sqrt(dx_goal_rel * dx_goal_rel + dy_goal_rel * dy_goal_rel);

  if (d_goal_Cr_rel < 1e-6) {
    RCLCPP_WARN(
      rclcpp::get_logger("ClothoidPullOut"),
      "Goal is too close to start arc center (distance: %.6f)", d_goal_Cr_rel);
    return std::nullopt;
  }

  // Calculate radius using Al-Kashi theorem
  double cos_term = dy_goal_rel / d_goal_Cr_rel;
  cos_term = std::max(-1.0, std::min(1.0, cos_term));

  // Adjust approach angle to goal (add π for reverse direction)
  double alpha = (yaw_goal_rel + PI) + std::acos(cos_term);

  double denominator = 2 * minimum_radius + 2 * d_goal_Cr_rel * std::cos(alpha);

  if (std::abs(denominator) < 1e-6) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "Denominator too small (denominator: %.6f)",
      denominator);
    return std::nullopt;
  }

  double R_goal = (d_goal_Cr_rel * d_goal_Cr_rel - minimum_radius * minimum_radius) / denominator;

  // Check physical feasibility
  if (R_goal < 0) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"), "Calculated radius is negative (R_goal: %.3f)",
      R_goal);
    return std::nullopt;
  }

  if (R_goal < minimum_radius) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ClothoidPullOut"),
      "Calculated radius is smaller than minimum (R_goal: %.3f < R_min: %.3f)", R_goal,
      minimum_radius);
    return std::nullopt;
  }

  // Calculate goal arc center (assuming counter-clockwise rotation)
  double C_lx_rel = x_goal_rel - R_goal * std::sin(yaw_goal_rel);
  double C_ly_rel = y_goal_rel + R_goal * std::cos(yaw_goal_rel);

  // Calculate tangent point
  double dx_centers = C_lx_rel - C_rx_rel;
  double dy_centers = C_ly_rel - C_ry_rel;
  double distance_centers = std::sqrt(dx_centers * dx_centers + dy_centers * dy_centers);

  double tangent_x_rel, tangent_y_rel;
  if (distance_centers < 1e-6) {
    tangent_x_rel = (C_rx_rel + C_lx_rel) / 2.0;
    tangent_y_rel = (C_ry_rel + C_ly_rel) / 2.0;
  } else {
    double ratio = minimum_radius / distance_centers;
    tangent_x_rel = C_rx_rel + ratio * dx_centers;
    tangent_y_rel = C_ry_rel + ratio * dy_centers;
  }

  // First arc (from start point to tangent point, clockwise)
  double start_angle1 = std::atan2(y_start_rel - C_ry_rel, x_start_rel - C_rx_rel);
  double end_angle1 = std::atan2(tangent_y_rel - C_ry_rel, tangent_x_rel - C_rx_rel);
  const double total_angle1 = std::abs(end_angle1 - start_angle1);

  // Adjust for clockwise direction
  if (total_angle1 > 0) {
    end_angle1 -= 2 * PI;
  }

  // Prepare for transformation to global coordinate system
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double cos_yaw = std::cos(start_yaw);
  const double sin_yaw = std::sin(start_yaw);

  // Create CompositeArcPath
  CompositeArcPath composite_path;

  // Create first arc segment
  ArcSegment arc1;
  arc1.radius = minimum_radius;
  arc1.is_clockwise = true;

  // Transform relative coordinate center to global coordinate system
  arc1.center.x = start_pose.position.x + C_rx_rel * cos_yaw - C_ry_rel * sin_yaw;
  arc1.center.y = start_pose.position.y + C_rx_rel * sin_yaw + C_ry_rel * cos_yaw;
  arc1.center.z = start_pose.position.z;

  // Set start and end poses
  arc1.start_pose = start_pose;

  // Calculate pose at tangent point (global coordinate system)
  geometry_msgs::msg::Pose tangent_pose;
  tangent_pose.position.x =
    start_pose.position.x + tangent_x_rel * cos_yaw - tangent_y_rel * sin_yaw;
  tangent_pose.position.y =
    start_pose.position.y + tangent_x_rel * sin_yaw + tangent_y_rel * cos_yaw;
  tangent_pose.position.z = start_pose.position.z;

  // Calculate orientation at tangent point (tangent direction of arc)
  double tangent_angle_global = end_angle1 + (arc1.is_clockwise ? -PI / 2 : PI / 2) + start_yaw;
  tangent_pose.orientation.x = 0.0;
  tangent_pose.orientation.y = 0.0;
  tangent_pose.orientation.z = std::sin(tangent_angle_global / 2.0);
  tangent_pose.orientation.w = std::cos(tangent_angle_global / 2.0);

  arc1.end_pose = tangent_pose;

  // Create second arc segment
  ArcSegment arc2;
  arc2.radius = R_goal;
  arc2.is_clockwise = false;

  // Transform relative coordinate center to global coordinate system
  arc2.center.x = start_pose.position.x + C_lx_rel * cos_yaw - C_ly_rel * sin_yaw;
  arc2.center.y = start_pose.position.y + C_lx_rel * sin_yaw + C_ly_rel * cos_yaw;
  arc2.center.z = start_pose.position.z;

  // Set start pose (tangent point) and end pose (goal point)
  arc2.start_pose = tangent_pose;

  // Calculate goal pose (global coordinate system)
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = start_pose.position.x + x_goal_rel * cos_yaw - y_goal_rel * sin_yaw;
  goal_pose.position.y = start_pose.position.y + x_goal_rel * sin_yaw + y_goal_rel * cos_yaw;
  goal_pose.position.z = start_pose.position.z;

  // Calculate orientation at goal point
  double goal_yaw_global = start_yaw + yaw_goal_rel;
  goal_pose.orientation.x = 0.0;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = std::sin(goal_yaw_global / 2.0);
  goal_pose.orientation.w = std::cos(goal_yaw_global / 2.0);

  arc2.end_pose = goal_pose;

  // Add segments
  composite_path.segments.push_back(arc1);
  composite_path.segments.push_back(arc2);

  // Check if circular path generation failed
  if (composite_path.segments.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger("ClothoidPullOut"), "Circular path generation failed (segments: %zu)",
      composite_path.segments.size());
    return std::nullopt;
  }

  return composite_path;
}

ClothoidPullOut::ClothoidPullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: PullOutPlannerBase{node, parameters, time_keeper}
{
  autoware::boundary_departure_checker::Param boundary_departure_checker_params;
  boundary_departure_checker_params.footprint_extra_margin =
    parameters.lane_departure_check_expansion_margin;
  boundary_departure_checker_ =
    std::make_shared<autoware::boundary_departure_checker::BoundaryDepartureChecker>(
      boundary_departure_checker_params, vehicle_info_, time_keeper_);
}

std::optional<PullOutPath> ClothoidPullOut::plan(
  const Pose & start_pose, const Pose & /*goal_pose*/,
  const std::shared_ptr<const PlannerData> & planner_data, PlannerDebugData & planner_debug_data)
{
  // =====================================================================
  // STEP 1: Parameter setting and initialization
  // =====================================================================
  const double initial_velocity = parameters_.clothoid_initial_velocity;
  const double acceleration = parameters_.clothoid_acceleration;
  const std::vector<double> max_steer_angles_deg = parameters_.clothoid_max_steer_angles_deg;

  // Convert degrees to radians from parameters
  std::vector<double> max_steer_angle;
  for (const auto & deg : max_steer_angles_deg) {
    max_steer_angle.push_back(deg2rad(deg));
  }

  const double max_steer_angle_rate_deg_per_sec =
    parameters_.clothoid_max_steer_angle_rate_deg_per_sec;
  const double max_steer_angle_rate = deg2rad(max_steer_angle_rate_deg_per_sec);
  // TODO(Sugahara): The forward straight section and backward path may not be necessary. If they
  // are not required, consider removing these variables after verification. If a backward path is
  // needed, avoid hardcoding and use the corresponding parameters from lane_following module
  // instead.
  const double backward_distance = 3.0;                      // backward distance [m]
  constexpr double initial_forward_straight_distance = 3.0;  // [m] straight section length (temp)

  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;
  const double wheel_base = common_parameters.vehicle_info.wheel_base_m;
  const double backward_path_length =
    planner_data->parameters.backward_path_length + parameters_.max_back_distance;

  // =====================================================================
  // STEP 2: Get lane information
  // =====================================================================
  // TODO(Sugahara): check if crossing lanes are not included
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // Get pull_out_lanes (including shoulder lane)
  const auto pull_out_lanes = getPullOutLanes(planner_data, backward_path_length);

  // Combine road_lanes and pull_out_lanes to include all lanes
  const auto all_lanes = utils::combineLanelets(road_lanes, pull_out_lanes);

  // =====================================================================
  // STEP 3: Generate centerline path
  // =====================================================================
  const auto row_centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  PathWithLaneId centerline_path =
    utils::resamplePathWithSpline(row_centerline_path, parameters_.center_line_path_interval);

  // =====================================================================
  // STEP 4: Generate straight path forward and backward
  // =====================================================================
  // Generate forward and backward straight path using create_straight_path_to_end_pose function
  auto straight_poses = create_straight_path_to_end_pose(
    start_pose, initial_forward_straight_distance, backward_distance,
    parameters_.center_line_path_interval);

  // Don't need to make it path_with_lane_id here
  // Generate PathPointWithLaneId from straight_poses
  std::vector<PathPointWithLaneId> straight_forward_points;
  for (size_t i = 0; i < straight_poses.size(); ++i) {
    PathPointWithLaneId pt;
    pt.point.pose = straight_poses[i];
    pt.point.longitudinal_velocity_mps = initial_velocity;
    pt.point.lateral_velocity_mps = 0.0;
    pt.point.heading_rate_rps = 0.0;
    pt.point.is_final = false;

    // Set lane ID
    std::vector<int64_t> previous_lane_ids;
    if (i > 0) {
      previous_lane_ids = straight_forward_points[i - 1].lane_ids;
    }
    set_lane_ids_to_path_point(pt, all_lanes, previous_lane_ids);

    straight_forward_points.push_back(pt);
  }

  const Pose straight_end_pose = straight_poses.back();

  // =====================================================================
  // STEP 5: Processing loop for each steering angle
  // =====================================================================
  for (const auto & steer_angle : max_steer_angle) {
    // ===================================================================
    // STEP 5-1: Generate arc path
    // ===================================================================
    // Calculate minimum radius based on the maximum steer angle
    const double minimum_radius = wheel_base / std::tan(steer_angle);

    const double longitudinal_distance = calc_necessary_longitudinal_distance(
      minimum_radius, initial_velocity, wheel_base, max_steer_angle_rate, centerline_path,
      start_pose);

    const Pose target_pose = start_planner_utils::find_target_pose_along_path(
      centerline_path, straight_end_pose, longitudinal_distance);

    // TODO(Sugahara): If lateral_offset is positive here, straight path is fine.
    const auto relative_pose_info =
      start_planner_utils::calculate_relative_pose_in_vehicle_coordinate(
        straight_end_pose, target_pose);

    // Generate circular path using calc_circular_path
    const auto circular_path_opt = calc_circular_path(
      straight_end_pose, relative_pose_info.longitudinal_distance_vehicle,
      relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

    // Check if circular path generation failed
    if (!circular_path_opt) {
      RCLCPP_INFO(
        rclcpp::get_logger("ClothoidPullOut"),
        "Circular path generation failed for steer angle %f deg. Relative pose info: %f, %f, %f. "
        "Continuing to next candidate.",
        rad2deg(steer_angle), relative_pose_info.longitudinal_distance_vehicle,
        relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff);
      planner_debug_data.conditions_evaluation.emplace_back("circular path generation failed");
      continue;
    }

    const auto & circular_path = *circular_path_opt;

    // ===================================================================
    // STEP 5-2: Generate clothoid path
    // ===================================================================
    geometry_msgs::msg::Pose current_segment_pose = straight_end_pose;
    std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

    // Process first segment (start segment)
    const auto & first_segment = circular_path.segments[0];

    // Clothoid conversion process for first segment
    const double first_minimum_radius = first_segment.radius;
    const double first_circular_steer_angle = std::atan(wheel_base / first_minimum_radius);
    const double first_minimum_steer_time = first_circular_steer_angle / max_steer_angle_rate;
    const double first_L_min = initial_velocity * first_minimum_steer_time;
    const double first_A_min = std::sqrt(first_minimum_radius * first_L_min);

    auto first_clothoid_points_opt = convert_arc_to_clothoid(
      first_segment, current_segment_pose, first_A_min, first_L_min,
      parameters_.center_line_path_interval);

    if (!first_clothoid_points_opt) {
      planner_debug_data.conditions_evaluation.emplace_back(
        "first segment clothoid conversion failed");
      continue;
    }

    // Apply endpoint correction for first segment
    auto first_corrected_points = correct_clothoid_by_rigid_transform(
      *first_clothoid_points_opt, first_segment, current_segment_pose);

    clothoid_paths.push_back(first_corrected_points);

    // Calculate pose at end of first segment (used as start pose for second segment)
    geometry_msgs::msg::Pose second_segment_start_pose;
    const auto & last_point_first = first_corrected_points.back();
    second_segment_start_pose.position = last_point_first;

    // Calculate heading direction at endpoint (from last two points)
    if (first_corrected_points.size() >= 2) {
      // TODO(Sugahara): Is this yaw calculation method correct?
      const auto & second_last_first = first_corrected_points[first_corrected_points.size() - 2];
      double dx = last_point_first.x - second_last_first.x;
      double dy = last_point_first.y - second_last_first.y;
      double heading = std::atan2(dy, dx);
      second_segment_start_pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    } else {
      second_segment_start_pose.orientation = current_segment_pose.orientation;
    }

    // Process second segment (end segment)
    const auto & second_segment = circular_path.segments[1];

    // Clothoid conversion process for second segment
    const double second_minimum_radius = second_segment.radius;
    const double second_circular_steer_angle = std::atan(wheel_base / second_minimum_radius);
    const double second_minimum_steer_time = second_circular_steer_angle / max_steer_angle_rate;
    const double second_L_min = initial_velocity * second_minimum_steer_time;
    const double second_A_min = std::sqrt(second_minimum_radius * second_L_min);

    auto second_clothoid_points_opt = convert_arc_to_clothoid(
      second_segment, second_segment_start_pose, second_A_min, second_L_min,
      parameters_.center_line_path_interval);

    if (!second_clothoid_points_opt) {
      planner_debug_data.conditions_evaluation.emplace_back(
        "second segment clothoid conversion failed");
      continue;
    }

    // Apply endpoint correction for second segment
    auto second_corrected_points = correct_clothoid_by_rigid_transform(
      *second_clothoid_points_opt, second_segment, second_segment_start_pose);

    clothoid_paths.push_back(second_corrected_points);

    // ===================================================================
    // STEP 5-4: Get target velocity and path combination/resampling
    // ===================================================================
    // Get target velocity (use velocity of point closest to target_pose from centerline_path)
    double target_velocity = initial_velocity;  // Default value
    if (!centerline_path.points.empty()) {
      const auto target_idx =
        autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);
      if (target_idx < centerline_path.points.size()) {
        target_velocity = centerline_path.points[target_idx].point.longitudinal_velocity_mps;
      }
    }

    // Combine clothoid path with centerline
    std::optional<PathWithLaneId> path_with_lane_id_opt =
      create_path_with_lane_id_from_clothoid_paths(
        clothoid_paths, initial_velocity, target_velocity, acceleration, all_lanes, route_handler);

    if (!path_with_lane_id_opt) {
      RCLCPP_WARN(
        rclcpp::get_logger("ClothoidPullOut"),
        "Failed to create clothoid path with lane ID for steer angle %.2f deg. Continuing to next "
        "candidate.",
        rad2deg(steer_angle));
      planner_debug_data.conditions_evaluation.emplace_back("clothoid path creation failed");
      continue;
    }

    const auto & path_with_lane_id = *path_with_lane_id_opt;

    // Combine with centerline path
    auto combined_path =
      combine_path_with_centerline(path_with_lane_id, centerline_path, target_pose);

    PathWithLaneId resampled_combined_path =
      utils::resamplePathWithSpline(combined_path, parameters_.center_line_path_interval);

    // ===================================================================
    // STEP 5-5: Create final path (combine with forward/backward straight paths, recalculate yaw
    // angles)
    // ===================================================================
    PathWithLaneId final_path;
    final_path.header = resampled_combined_path.header;
    final_path.points = straight_forward_points;  // Forward/backward straight path (integrated)

    // Add clothoid path + centerline extension path (skip duplicate points)
    if (!resampled_combined_path.points.empty()) {
      // Skip first point to avoid duplication
      for (size_t i = 1; i < resampled_combined_path.points.size(); ++i) {
        final_path.points.push_back(resampled_combined_path.points[i]);
      }
    }

    // Recalculate yaw angles based on coordinate information in final_path
    autoware::motion_utils::insertOrientation(final_path.points, true);

    // ===================================================================
    // STEP 5-6: Lane departure check and path validation
    // ===================================================================
    const auto lanelet_map_ptr = planner_data->route_handler->getLaneletMapPtr();

    std::vector<lanelet::Id> fused_id_start_to_end{};
    std::optional<autoware_utils::Polygon2d> fused_polygon_start_to_end = std::nullopt;

    // clothoid path is not separate but only one.
    auto & clothoid_path = final_path;

    // check lane_departure with path between pull_out_start to pull_out_end
    PathWithLaneId path_clothoid_start_to_end{};
    {
      const size_t pull_out_start_idx =
        autoware::motion_utils::findNearestIndex(clothoid_path.points, start_pose.position);
      const size_t pull_out_end_idx =
        autoware::motion_utils::findNearestIndex(clothoid_path.points, target_pose.position);

      path_clothoid_start_to_end.points.insert(
        path_clothoid_start_to_end.points.begin(),
        clothoid_path.points.begin() + pull_out_start_idx,
        clothoid_path.points.begin() + pull_out_end_idx + 1);
    }

    // check lane departure
    if (
      parameters_.check_clothoid_path_lane_departure &&
      boundary_departure_checker_->checkPathWillLeaveLane(
        lanelet_map_ptr, path_clothoid_start_to_end, fused_id_start_to_end,
        fused_polygon_start_to_end)) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("ClothoidPullOut"),
        "Lane departure detected for steer angle %.2f deg. Continuing to next candidate.",
        rad2deg(steer_angle));
      planner_debug_data.conditions_evaluation.emplace_back("lane departure");
      continue;
    }

    // crop backward path
    const size_t start_segment_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        clothoid_path.points, start_pose, common_parameters.ego_nearest_dist_threshold,
        common_parameters.ego_nearest_yaw_threshold);

    PathWithLaneId cropped_path;
    if (parameters_.check_clothoid_path_lane_departure) {
      std::vector<lanelet::Id> fused_id_crop_points{};
      std::optional<autoware_utils::Polygon2d> fused_polygon_crop_points = std::nullopt;
      cropped_path = boundary_departure_checker_->cropPointsOutsideOfLanes(
        lanelet_map_ptr, clothoid_path, start_segment_idx, fused_id_crop_points,
        fused_polygon_crop_points);
      if (cropped_path.points.empty()) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("ClothoidPullOut"),
          "Cropped path is empty for steer angle %.2f deg. Continuing to next candidate.",
          rad2deg(steer_angle));
        planner_debug_data.conditions_evaluation.emplace_back("cropped path is empty");
        continue;
      }
    } else {
      // If lane departure check is disabled, use the original path without cropping
      cropped_path = clothoid_path;
    }

    // check that the path is not cropped in excess and there is not excessive longitudinal
    // deviation between the first 2 points
    auto validate_cropped_path = [&](const auto & cropped_path) -> bool {
      if (cropped_path.points.size() < 2) return false;
      const double max_long_offset = parameters_.maximum_longitudinal_deviation;
      const size_t start_segment_idx_after_crop =
        autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
          cropped_path.points, start_pose);

      // if the start segment id after crop is not 0, then the cropping is not excessive
      if (start_segment_idx_after_crop != 0) return true;

      const auto long_offset_to_closest_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop, start_pose.position);
      const auto long_offset_to_next_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop + 1, start_pose.position);
      return std::abs(long_offset_to_closest_point - long_offset_to_next_point) < max_long_offset;
    };

    if (parameters_.check_clothoid_path_lane_departure && !validate_cropped_path(cropped_path)) {
      RCLCPP_INFO(
        rclcpp::get_logger("ClothoidPullOut"),
        "Cropped path is invalid for steer angle %.2f deg. Continuing to next candidate.",
        rad2deg(steer_angle));
      planner_debug_data.conditions_evaluation.emplace_back("cropped path is invalid");
      continue;
    }

    // Update the final path with cropped path
    clothoid_path.points = cropped_path.points;
    clothoid_path.header = planner_data->route_handler->getRouteHeader();

    // ===================================================================
    // STEP 5-7: Collision check
    // ===================================================================
    // Create PullOutPath for collision check
    PullOutPath temp_pull_out_path;
    temp_pull_out_path.partial_paths.push_back(clothoid_path);
    temp_pull_out_path.start_pose =
      clothoid_path.points.empty() ? start_pose : clothoid_path.points.front().point.pose;
    temp_pull_out_path.end_pose = target_pose;

    if (isPullOutPathCollided(
          temp_pull_out_path, planner_data, parameters_.shift_collision_check_distance_from_end)) {
      RCLCPP_INFO(
        rclcpp::get_logger("ClothoidPullOut"),
        "Collision detected for steer angle %.2f deg with margin %.2f m. Continuing to next "
        "candidate.",
        rad2deg(steer_angle), parameters_.shift_collision_check_distance_from_end);
      planner_debug_data.conditions_evaluation.emplace_back("collision");
      continue;
    }

    // ===================================================================
    // STEP 5-8: Return result on success
    // ===================================================================
    // Create final PullOutPath if validation succeeds
    PullOutPath pull_out_path;
    pull_out_path.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(initial_velocity, acceleration));
    pull_out_path.partial_paths.push_back(clothoid_path);  // Use validated and cropped path

    pull_out_path.start_pose =
      clothoid_path.points.empty() ? start_pose : clothoid_path.points.front().point.pose;
    pull_out_path.end_pose = target_pose;

    RCLCPP_INFO(
      rclcpp::get_logger("clothoid_pull_out"),
      "\n===========================================\n"
      "Successfully generated clothoid pull-out path with max steer angle %.2f deg.\n"
      "===========================================",
      rad2deg(steer_angle));

    planner_debug_data.conditions_evaluation.emplace_back("success");
    return pull_out_path;
  }

  // =====================================================================
  // STEP 6: Case when path could not be generated
  // =====================================================================
  planner_debug_data.conditions_evaluation.emplace_back("no path found");
  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
