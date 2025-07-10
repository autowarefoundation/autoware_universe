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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::motion_velocity_planner
{

struct RoadUserStopParameters
{
  // Detection parameters
  struct Detection
  {
    struct TargetObjectTypes
    {
      bool check_pedestrian;
      bool check_bicycle;
      bool check_motorcycle;
      bool check_unknown;
    } target_object_types;

    double search_length;
    bool adjacent_lane_check;
    double adjacent_lane_margin;  // maximum lateral distance from trajectory for adjacent lanes
    bool exclude_crosswalk_users;
    double crosswalk_margin;
    bool exclude_sidewalk_users;
    double min_detection_duration;
  } detection;

  // Stop decision parameters
  struct StopDecision
  {
    double stop_margin;
    double max_deceleration;
  } stop_decision;

  // In-place stop parameters
  struct InPlaceStop
  {
    bool enable;
    double wrong_way_angle_threshold;
    double min_speed_threshold;  // minimum speed to consider object as moving
    double deceleration;
  } in_place_stop;

  // Debug parameters
  struct Debug
  {
    bool publish_debug_markers;
    bool print_debug_info;
  } debug;

  RoadUserStopParameters() = default;

  explicit RoadUserStopParameters(rclcpp::Node & node, const std::string & module_name)
  {
    // Note: get_or_declare_parameter uses the full path including module name
    // For namespace "road_user_stop", parameters should be accessed as "road_user_stop.param"

    // Detection parameters - target object types
    detection.target_object_types.check_pedestrian = node.declare_parameter<bool>(
      module_name + ".detection.target_object_types.check_pedestrian", true);
    detection.target_object_types.check_bicycle = node.declare_parameter<bool>(
      module_name + ".detection.target_object_types.check_bicycle", true);
    detection.target_object_types.check_motorcycle = node.declare_parameter<bool>(
      module_name + ".detection.target_object_types.check_motorcycle", false);
    detection.target_object_types.check_unknown = node.declare_parameter<bool>(
      module_name + ".detection.target_object_types.check_unknown", true);

    // Detection parameters - search settings
    detection.search_length =
      node.declare_parameter<double>(module_name + ".detection.search_length", 50.0);
    detection.adjacent_lane_check =
      node.declare_parameter<bool>(module_name + ".detection.adjacent_lane_check", true);
    detection.adjacent_lane_margin =
      node.declare_parameter<double>(module_name + ".detection.adjacent_lane_margin", 10.0);
    detection.exclude_crosswalk_users =
      node.declare_parameter<bool>(module_name + ".detection.exclude_crosswalk_users", true);
    detection.crosswalk_margin =
      node.declare_parameter<double>(module_name + ".detection.crosswalk_margin", 1.0);
    detection.exclude_sidewalk_users =
      node.declare_parameter<bool>(module_name + ".detection.exclude_sidewalk_users", true);
    detection.min_detection_duration =
      node.declare_parameter<double>(module_name + ".detection.min_detection_duration", 0.5);

    // Stop decision parameters
    stop_decision.stop_margin =
      node.declare_parameter<double>(module_name + ".stop_decision.stop_margin", 5.0);
    stop_decision.max_deceleration =
      node.declare_parameter<double>(module_name + ".stop_decision.max_deceleration", 2.5);

    // In-place stop parameters
    in_place_stop.enable =
      node.declare_parameter<bool>(module_name + ".in_place_stop.enable", true);
    in_place_stop.wrong_way_angle_threshold = node.declare_parameter<double>(
      module_name + ".in_place_stop.wrong_way_angle_threshold", 150.0);
    in_place_stop.min_speed_threshold = node.declare_parameter<double>(
      module_name + ".in_place_stop.min_speed_threshold", 0.5);  // 0.5 m/s
    in_place_stop.deceleration =
      node.declare_parameter<double>(module_name + ".in_place_stop.deceleration", 1.0);

    // Debug parameters
    debug.publish_debug_markers =
      node.declare_parameter<bool>(module_name + ".debug.publish_debug_markers", false);
    debug.print_debug_info =
      node.declare_parameter<bool>(module_name + ".debug.print_debug_info", false);
  }
};

}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
