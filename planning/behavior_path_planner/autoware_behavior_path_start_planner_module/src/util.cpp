// Copyright 2021 Tier IV, Inc.
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

#include "autoware/behavior_path_start_planner_module/util.hpp"

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/universe_utils/math/normalization.hpp"

#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using autoware::universe_utils::normalizeRadian;
using autoware_utils::deg2rad;

namespace autoware::behavior_path_planner::start_planner_utils
{
PathWithLaneId getBackwardPath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & shoulder_lanes,
  const Pose & current_pose, const Pose & backed_pose, const double velocity)
{
  const auto current_pose_arc_coords = lanelet::utils::getArcCoordinatesOnEgoCenterline(
    shoulder_lanes, current_pose, route_handler.getLaneletMapPtr());
  const auto backed_pose_arc_coords = lanelet::utils::getArcCoordinatesOnEgoCenterline(
    shoulder_lanes, backed_pose, route_handler.getLaneletMapPtr());

  const double s_start = backed_pose_arc_coords.length;
  const double s_end = current_pose_arc_coords.length;

  PathWithLaneId backward_path;
  {
    // forward center line path
    backward_path = route_handler.getCenterLinePath(shoulder_lanes, s_start, s_end, true);

    // If the returned path is empty, return an empty path
    if (backward_path.points.empty()) {
      return backward_path;
    }

    // backward center line path
    std::reverse(backward_path.points.begin(), backward_path.points.end());
    for (auto & p : backward_path.points) {
      p.point.longitudinal_velocity_mps = velocity;
    }
    backward_path.points.back().point.longitudinal_velocity_mps = 0.0;

    // lateral shift to current_pose
    const double lateral_distance_to_shoulder_center = current_pose_arc_coords.distance;
    for (size_t i = 0; i < backward_path.points.size(); ++i) {
      auto & p = backward_path.points.at(i).point.pose;
      p = autoware_utils::calc_offset_pose(p, 0, lateral_distance_to_shoulder_center, 0);
    }
  }

  return backward_path;
}

lanelet::ConstLanelets getPullOutLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length)
{
  const double & vehicle_width = planner_data->parameters.vehicle_width;
  const auto & route_handler = planner_data->route_handler;
  const auto start_pose = planner_data->route_handler->getOriginalStartPose();

  const auto current_shoulder_lane = route_handler->getPullOutStartLane(start_pose, vehicle_width);
  if (current_shoulder_lane) {
    // pull out from shoulder lane
    return route_handler->getShoulderLaneletSequence(*current_shoulder_lane, start_pose);
  }

  // pull out from road lane
  return utils::getExtendedCurrentLanes(
    planner_data, backward_length,
    /*forward_length*/ std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
}

std::optional<PathWithLaneId> extractCollisionCheckSection(
  const PullOutPath & path, const double collision_check_distance_from_end)
{
  PathWithLaneId full_path;
  for (const auto & partial_path : path.partial_paths) {
    full_path.points.insert(
      full_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  if (full_path.points.empty()) return std::nullopt;
  // Find the start index for collision check section based on the shift start pose
  const auto shift_start_idx =
    autoware::motion_utils::findNearestIndex(full_path.points, path.start_pose.position);

  // Find the end index for collision check section based on the end pose and collision check
  // distance
  const auto collision_check_end_idx = [&]() -> size_t {
    const auto end_pose_offset = autoware::motion_utils::calcLongitudinalOffsetPose(
      full_path.points, path.end_pose.position, collision_check_distance_from_end);

    return end_pose_offset
             ? autoware::motion_utils::findNearestIndex(full_path.points, end_pose_offset->position)
             : full_path.points.size() - 1;  // Use the last point if offset pose is not calculable
  }();

  // Extract the collision check section from the full path
  PathWithLaneId collision_check_section;
  if (shift_start_idx < collision_check_end_idx) {
    collision_check_section.points.assign(
      full_path.points.begin() + shift_start_idx,
      full_path.points.begin() + collision_check_end_idx + 1);
  }

  return collision_check_section;
}

namespace
{

struct PathEvaluationResult
{
  double distance;
  double arc1_length;
  double lateral_error;
  bool is_valid;

  PathEvaluationResult() : distance(0.0), arc1_length(0.0), lateral_error(0.0), is_valid(false) {}
  PathEvaluationResult(double d, double arc_len, double error, bool valid)
  : distance(d), arc1_length(arc_len), lateral_error(error), is_valid(valid)
  {
  }
};

}  // anonymous namespace

std::vector<double> calc_curvature_from_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  using autoware_utils::calc_curvature;

  std::vector<double> curvatures;

  if (trajectory.points.size() < 3) {
    // Cannot calculate curvature with less than 3 points
    curvatures.resize(trajectory.points.size(), 0.0);
    return curvatures;
  }

  curvatures.reserve(trajectory.points.size());

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    if (i == 0) {
      // First point: use next 2 points
      const auto & p1 = trajectory.points[0].pose.position;
      const auto & p2 = trajectory.points[1].pose.position;
      const auto & p3 = trajectory.points[2].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    } else if (i == trajectory.points.size() - 1) {
      // Last point: use previous 2 points
      const auto & p1 = trajectory.points[i - 2].pose.position;
      const auto & p2 = trajectory.points[i - 1].pose.position;
      const auto & p3 = trajectory.points[i].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    } else {
      // Middle points: use surrounding points
      const auto & p1 = trajectory.points[i - 1].pose.position;
      const auto & p2 = trajectory.points[i].pose.position;
      const auto & p3 = trajectory.points[i + 1].pose.position;
      curvatures.push_back(calc_curvature(p1, p2, p3));
    }
  }

  return curvatures;
}

std::vector<double> calc_curvature_from_points(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  using autoware_utils::calc_curvature;

  std::vector<double> curvatures;

  if (points.size() < 3) {
    // Cannot calculate curvature with less than 3 points
    curvatures.resize(points.size(), 0.0);
    return curvatures;
  }

  curvatures.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    try {
      if (i == 0) {
        // First point: use next 2 points
        const auto & p1 = points[0];
        const auto & p2 = points[1];
        const auto & p3 = points[2];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      } else if (i == points.size() - 1) {
        // Last point: use previous 2 points
        const auto & p1 = points[i - 2];
        const auto & p2 = points[i - 1];
        const auto & p3 = points[i];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      } else {
        // Middle points: use surrounding points
        const auto & p1 = points[i - 1];
        const auto & p2 = points[i];
        const auto & p3 = points[i + 1];
        curvatures.push_back(calc_curvature(p1, p2, p3));
      }
    } catch (const std::runtime_error & e) {
      // Set curvature to 0 if points are too close
      curvatures.push_back(0.0);
    }
  }

  return curvatures;
}

Pose find_target_pose_along_path(
  const PathWithLaneId & centerline_path, const Pose & start_pose,
  const double longitudinal_distance)
{
  Pose target_pose = start_pose;
  if (!centerline_path.points.empty()) {
    // Find the point on centerline path that is longitudinal_distance ahead
    const auto start_idx =
      autoware::motion_utils::findNearestIndex(centerline_path.points, start_pose.position);
    double accumulated_distance = 0.0;
    size_t target_idx = start_idx;

    for (size_t i = start_idx; i < centerline_path.points.size() - 1; ++i) {
      const double segment_distance = autoware_utils::calc_distance2d(
        centerline_path.points[i].point.pose.position,
        centerline_path.points[i + 1].point.pose.position);
      accumulated_distance += segment_distance;

      if (accumulated_distance >= longitudinal_distance) {
        target_idx = i + 1;
        break;
      }
    }

    if (target_idx < centerline_path.points.size()) {
      target_pose = centerline_path.points[target_idx].point.pose;
    }
  }

  return target_pose;
}

RelativePoseInfo calculate_relative_pose_in_vehicle_coordinate(
  const Pose & start_pose, const Pose & target_pose)
{
  const double dx = target_pose.position.x - start_pose.position.x;
  const double dy = target_pose.position.y - start_pose.position.y;
  const double start_yaw = tf2::getYaw(start_pose.orientation);
  const double target_yaw = tf2::getYaw(target_pose.orientation);

  // Transform to vehicle coordinate system (x: forward, y: left)
  const double longitudinal_distance_vehicle = dx * std::cos(start_yaw) + dy * std::sin(start_yaw);
  const double lateral_distance_vehicle = -dx * std::sin(start_yaw) + dy * std::cos(start_yaw);

  // Calculate angle difference
  double angle_diff = target_yaw - start_yaw;
  // Normalize angle to [-pi, pi]
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  return {longitudinal_distance_vehicle, lateral_distance_vehicle, angle_diff};
}

/**
 * @brief Get lane_ids for a given pose
 * Generic function to get lane_ids for a pose, based on implementations from other
 * behavior_path_planner modules
 * @param pose Target pose
 * @param candidate_lanes Target lane group for search
 * @param previous_lane_ids Previous point's lane_ids (for inheritance, optional)
 * @return Retrieved lane_ids
 */
std::vector<int64_t> get_lane_ids_from_pose(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & candidate_lanes,
  const std::vector<int64_t> & previous_lane_ids)
{
  std::vector<int64_t> lane_ids;

  // 1. First, find all lanes containing the pose
  bool found_containing_lane = false;
  for (const auto & lane : candidate_lanes) {
    if (lanelet::utils::isInLanelet(pose, lane)) {
      lane_ids.push_back(lane.id());
      found_containing_lane = true;
    }
  }

  // 2. Fallback processing when no containing lane is found
  if (!found_containing_lane) {
    // 2.1 Find the closest lane
    lanelet::Lanelet closest_lanelet{};
    if (lanelet::utils::query::getClosestLanelet(candidate_lanes, pose, &closest_lanelet)) {
      lane_ids = {closest_lanelet.id()};
    } else if (!previous_lane_ids.empty()) {
      // 2.2 If closest lane is not found, inherit lane_ids from previous point
      lane_ids = previous_lane_ids;
    } else if (!candidate_lanes.empty()) {
      // 2.3 Final fallback: use the first lane
      lane_ids.push_back(candidate_lanes.front().id());
    }
  }

  return lane_ids;
}

/**
 * @brief Set lane_ids to PathPointWithLaneId
 * @param point Target PathPointWithLaneId to set
 * @param road_lanes Target lane group for search
 * @param previous_lane_ids Previous point's lane_ids (for inheritance, optional)
 */
void set_lane_ids_to_path_point(
  PathPointWithLaneId & point, const lanelet::ConstLanelets & road_lanes,
  const std::vector<int64_t> & previous_lane_ids)
{
  point.lane_ids = get_lane_ids_from_pose(point.point.pose, road_lanes, previous_lane_ids);
}

/**
 * @brief Print detailed information of each point in PathWithLaneId
 * @param path Target PathWithLaneId
 * @param path_name Path name (for debugging)
 */
void print_path_with_lane_id_details(const PathWithLaneId & path, const std::string & path_name)
{
  std::cout << "=== " << path_name << " Details ===" << std::endl;
  std::cout << "Total points: " << path.points.size() << std::endl;

  double cumulative_distance = 0.0;

  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & point = path.points[i];
    const auto & pose = point.point.pose;
    const auto & position = pose.position;
    const auto & orientation = pose.orientation;

    // Calculate yaw angle
    const double yaw = tf2::getYaw(orientation);

    // Calculate distance from previous point
    double distance_from_prev = 0.0;
    if (i > 0) {
      const auto & prev_position = path.points[i - 1].point.pose.position;
      distance_from_prev = std::sqrt(
        std::pow(position.x - prev_position.x, 2) + std::pow(position.y - prev_position.y, 2));
      cumulative_distance += distance_from_prev;
    }

    // Convert lane_ids to string
    std::string lane_ids_str = "[";
    for (size_t j = 0; j < point.lane_ids.size(); ++j) {
      if (j > 0) lane_ids_str += ", ";
      lane_ids_str += std::to_string(point.lane_ids[j]);
    }
    lane_ids_str += "]";

    // Print information
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[" << std::setw(3) << i << "] "
              << "x=" << std::setw(10) << position.x << " "
              << "y=" << std::setw(10) << position.y << " "
              << "z=" << std::setw(10) << position.z << " "
              << "yaw=" << std::setw(8) << yaw << " rad "
              << "(" << std::setw(6) << yaw * 180.0 / M_PI << "°) "
              << "quat[" << std::setw(7) << orientation.x << ", " << std::setw(7) << orientation.y
              << ", " << std::setw(7) << orientation.z << ", " << std::setw(7) << orientation.w
              << "] "
              << "dist_prev=" << std::setw(8) << distance_from_prev << " "
              << "cumulative=" << std::setw(8) << cumulative_distance << " "
              << "lane_ids=" << lane_ids_str << std::endl;
  }

  std::cout << "Total path length: " << cumulative_distance << " m" << std::endl;
  std::cout << "=================================" << std::endl;
}

}  // namespace autoware::behavior_path_planner::start_planner_utils
