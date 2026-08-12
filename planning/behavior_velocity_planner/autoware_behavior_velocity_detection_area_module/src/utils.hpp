// Copyright 2024 Tier IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "scene.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::detection_area
{

using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

/// @brief get the extended stop line of the given detection area
/// @param [in] detection_area detection area
/// @param [in] left_bound left bound of ego path
/// @param [in] right_bound right bound of ego path
/// @return extended stop line
autoware_utils::LineString2d get_stop_line(
  const lanelet::autoware::DetectionArea & detection_area,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound);

/**
 * @brief Calculate the stop point on the path for a given stop line
 * @param path ego path
 * @param stop_line stop line
 * @param margin [m] margin to keep between the stop point and the stop line
 * @param vehicle_offset [m] offset of the vehicle front from the base link
 * @param lane_ids lane ids to consider (if empty, consider all lanes)
 * @return arc length of stop point along path (std::nullopt if not found)
 */
std::optional<double> get_stop_point(
  const Trajectory & path, const autoware_utils::LineString2d & stop_line, const double margin,
  const double vehicle_offset, const lanelet::Ids & lane_ids = {});

/// @brief get the extended stop line of the given detection area
/// @param [in] detection_area detection area
/// @param [in] path ego path
/// @return extended stop line
autoware_utils::LineString2d get_stop_line_geometry2d(
  const lanelet::autoware::DetectionArea & detection_area,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path);

/// @brief get the obstacle points found inside a detection area
/// @param [in] detection_areas detection area polygons
/// @param [in] points obstacle points
/// @return the first obstacle point found in each detection area
std::vector<geometry_msgs::msg::Point> get_obstacle_points(
  const lanelet::ConstPolygons3d & detection_areas, const pcl::PointCloud<pcl::PointXYZ> & points);

/// @brief return true if the stop state can be cleared
/// @details can be cleared if enough time passed since last detecting an obstacle
/// @param [in] last_obstacle_found_time pointer to the time when an obstacle was last detected
/// @param [in] now current time
/// @param [in] state_clear_time [s] minimum duration since last obstacle detection to clear the
/// stop state
/// @return true if the stop state can be cleared
bool can_clear_stop_state(
  const std::shared_ptr<const rclcpp::Time> & last_obstacle_found_time, const rclcpp::Time & now,
  const double state_clear_time);

/// @brief return true if distance to brake is enough
/// @param self_s current ego position in arc length
/// @param line_point_s arc length of stop point
/// @param pass_judge_line_distance braking distance
/// @param current_velocity current ego velocity
/// @return true if the distance to brake is enough
bool has_enough_braking_distance(
  const double self_s, const double line_point_s, const double pass_judge_line_distance,
  const double current_velocity);

/// @brief return true if distance to brake is enough
/// @param self_pose current ego pose
/// @param line_pose stop pose
/// @param pass_judge_line_distance braking distance
/// @param current_velocity current ego velocity
/// @return true if the distance to brake is enough
bool has_enough_braking_distance(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose,
  const double pass_judge_line_distance, const double current_velocity);

/// @brief return the feasible stop distance by max acceleration
/// @param [in] current_velocity current ego velocity
/// @param [in] max_acceleration max acceleration
/// @return the feasible stop distance by max acceleration
double feasible_stop_distance_by_max_acceleration(
  const double current_velocity, const double max_acceleration);

/// @brief check if any predicted objects are found inside detection areas
/// @param [in] detection_areas detection area polygons
/// @param [in] predicted_objects predicted objects
/// @param [in] target_filtering target filtering parameters
/// @return detected object if found, std::nullopt otherwise
std::optional<autoware_perception_msgs::msg::PredictedObject> get_detected_object(
  const lanelet::ConstPolygons3d & detection_areas,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const DetectionAreaModule::PlannerParam::TargetFiltering & target_filtering);

/// @brief check if the object classification matches the target filtering criteria
/// @param [in] classifications object classifications
/// @param [in] target_filtering target filtering parameters
/// @return true if the object matches the filtering criteria
bool is_target_object(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications,
  const DetectionAreaModule::PlannerParam::TargetFiltering & target_filtering);

/// @brief convert object classification label to lowercase string
/// @param [in] label object classification label
/// @return lowercase string representation (e.g., "car", "bus", "unknown")
std::string object_label_to_string(const uint8_t label);
}  // namespace autoware::behavior_velocity_planner::detection_area

#endif  // UTILS_HPP_
