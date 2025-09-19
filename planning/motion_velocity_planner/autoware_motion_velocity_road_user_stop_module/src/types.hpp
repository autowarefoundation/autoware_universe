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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "type_alias.hpp"

#include <autoware_utils_rclcpp/parameter.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

using autoware_utils_rclcpp::get_or_declare_parameter;
struct CommonParam
{
  double max_accel{};
  double min_accel{};
  double max_jerk{};
  double min_jerk{};
  double limit_max_accel{};
  double limit_min_accel{};
  double limit_max_jerk{};
  double limit_min_jerk{};

  CommonParam() = default;
  explicit CommonParam(rclcpp::Node & node)
  {
    max_accel = get_or_declare_parameter<double>(node, "normal.max_acc");
    min_accel = get_or_declare_parameter<double>(node, "normal.min_acc");
    max_jerk = get_or_declare_parameter<double>(node, "normal.max_jerk");
    min_jerk = get_or_declare_parameter<double>(node, "normal.min_jerk");
    limit_max_accel = get_or_declare_parameter<double>(node, "limit.max_acc");
    limit_min_accel = get_or_declare_parameter<double>(node, "limit.min_acc");
    limit_max_jerk = get_or_declare_parameter<double>(node, "limit.max_jerk");
    limit_min_jerk = get_or_declare_parameter<double>(node, "limit.min_jerk");
  }
};

struct StopObstacle
{
  StopObstacle(
    const UUID & arg_uuid, const rclcpp::Time & arg_stamp, const bool arg_is_opposite_traffic,
    const autoware_perception_msgs::msg::PredictedObject & arg_predicted_object,
    const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
    const Shape & arg_shape, const double arg_lon_velocity,
    const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    is_opposite_traffic(arg_is_opposite_traffic),
    original_object(arg_predicted_object),
    pose(arg_pose),
    velocity(arg_lon_velocity),
    shape(arg_shape),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(object_classification)
  {
  }
  UUID uuid;
  rclcpp::Time stamp;
  bool is_opposite_traffic;
  PredictedObject original_object;  // keep original object for reference
  geometry_msgs::msg::Pose pose;    // interpolated with the current stamp
  double velocity;                  // longitudinal velocity against ego's trajectory

  Shape shape;
  geometry_msgs::msg::Point collision_point;
  double dist_to_collide_on_decimated_traj;
  ObjectClassification classification;

  // for lost object tracking
  rclcpp::Time lost_time;  // time when object was lost
  bool is_lost = false;    // flag to indicate this is a lost object placeholder
};

struct RelevantLaneletData
{
  std::vector<autoware_utils_geometry::Polygon2d> polygons_for_vru;
  std::vector<autoware_utils_geometry::Polygon2d> polygons_for_opposing_traffic;
  lanelet::ConstLanelets ego_lanelets;
  lanelet::ConstLanelets opposite_lanelets;
};

struct TrackedObject
{
  std::string object_id;
  rclcpp::Time first_detected_time;
  rclcpp::Time last_detected_time;
  rclcpp::Time last_stop_obstacle_time;        // time when last detected as a stop obstacle
  std::deque<uint8_t> classification_history;  // ObjectClassification label values
  static constexpr size_t max_classification_history = 5;

  // for polygon expansion to reduce chattering
  bool was_inside_detection_area =
    false;  // flag to track if object was previously inside detection area
  double polygon_expansion_length = 0.0;  // expansion factor for object polygon when checking again

  void updateClassification(const uint8_t label)
  {
    classification_history.push_back(label);
    if (classification_history.size() > max_classification_history) {
      classification_history.pop_front();
    }
  }

  uint8_t getMostFrequentClassification() const
  {
    if (classification_history.empty()) {
      return ObjectClassification::UNKNOWN;
    }

    std::unordered_map<uint8_t, int> count_map;
    for (const auto & label : classification_history) {
      count_map[label]++;
    }

    auto max_it = std::max_element(
      count_map.begin(), count_map.end(),
      [](const auto & a, const auto & b) { return a.second < b.second; });

    return max_it->first;
  }
};

struct DebugData
{
  // Lanelet
  lanelet::ConstLanelets ego_lanelets;
  lanelet::ConstLanelets ego_lanelets_without_intersection;
  lanelet::ConstLanelets adjacent_lanelets;
  lanelet::ConstLanelets opposite_lanelets;

  // Lanelet polygons
  std::vector<autoware_utils_geometry::Polygon2d> intersection_polygons;
  std::vector<autoware_utils_geometry::Polygon2d> polygons_for_vru;
  std::vector<autoware_utils_geometry::Polygon2d> polygons_for_opposing_traffic;

  // Trajectory polygon with margins
  std::vector<autoware_utils_geometry::Polygon2d> trajectory_polygons;
  std::vector<PredictedObject> filtered_objects;
  std::vector<autoware_utils_geometry::Polygon2d> object_polygons;  // object polygons for debug
  std::optional<size_t> stop_index;
  std::optional<geometry_msgs::msg::Point> stop_point;  // for planning factor
  std::optional<PredictedObject> stop_target_object;    // object causing stop

  // Virtual wall marker
  MarkerArray stop_wall_marker;
};

}  // namespace autoware::motion_velocity_planner

#endif  // TYPES_HPP_
