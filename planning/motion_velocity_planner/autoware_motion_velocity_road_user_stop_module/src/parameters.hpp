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
  // Option parameters
  struct Option
  {
    bool suppress_sudden_stop;
  } option;

  // Stop planning parameters
  struct StopPlanning
  {
    double stop_margin;
    double terminal_stop_margin;
    double min_behavior_stop_margin;

    double max_negative_velocity;
    double stop_margin_opposing_traffic;
    double effective_deceleration_opposing_traffic;

    double min_on_duration;
    double min_off_duration;
    double update_distance_th;

    double hold_stop_velocity_threshold;
    double hold_stop_distance_threshold;

    struct StopOnCurve
    {
      bool enable_approaching;
      double additional_stop_margin;
      double min_stop_margin;
    } stop_on_curve;

    // Common parameters for all object types
    double limit_min_acc;
    double sudden_object_acc_threshold;
    double sudden_object_dist_threshold;
    bool abandon_to_stop;
  } stop_planning;

  // Obstacle filtering parameters
  struct ObstacleFiltering
  {
    struct ObjectType
    {
      struct TargetTypes
      {
        bool pedestrian;
        bool bicycle;
        bool motorcycle;
        bool unknown;
      } target_types;
    } object_type;

    double adjacent_lane_margin;  // always check adjacent lanes

    bool exclude_crosswalk_users;
    double crosswalk_margin;
    bool exclude_sidewalk_users;

    struct WrongWayDetection
    {
      bool enable;
      double angle_threshold;
      double min_speed_threshold;
    } wrong_way_detection;

    double min_detection_duration;
  } obstacle_filtering;

  // Common parameters (from common_param.yaml)
  struct CommonParam
  {
    double limit_max_accel{1.0};
    double limit_min_accel{-2.5};
  } common_param;

  RoadUserStopParameters() = default;

  explicit RoadUserStopParameters(rclcpp::Node & node, const std::string & module_name)
  {
    // Option parameters
    option.suppress_sudden_stop =
      node.declare_parameter<bool>(module_name + ".option.suppress_sudden_stop", true);

    // Stop planning parameters
    stop_planning.stop_margin =
      node.declare_parameter<double>(module_name + ".stop_planning.stop_margin", 2.0);
    stop_planning.terminal_stop_margin =
      node.declare_parameter<double>(module_name + ".stop_planning.terminal_stop_margin", 3.0);
    stop_planning.min_behavior_stop_margin =
      node.declare_parameter<double>(module_name + ".stop_planning.min_behavior_stop_margin", 3.0);

    stop_planning.max_negative_velocity =
      node.declare_parameter<double>(module_name + ".stop_planning.max_negative_velocity", -0.5);
    stop_planning.stop_margin_opposing_traffic = node.declare_parameter<double>(
      module_name + ".stop_planning.stop_margin_opposing_traffic", 3.0);
    stop_planning.effective_deceleration_opposing_traffic = node.declare_parameter<double>(
      module_name + ".stop_planning.effective_deceleration_opposing_traffic", 4.0);

    stop_planning.min_on_duration =
      node.declare_parameter<double>(module_name + ".stop_planning.min_on_duration", 0.3);
    stop_planning.min_off_duration =
      node.declare_parameter<double>(module_name + ".stop_planning.min_off_duration", 1.0);
    stop_planning.update_distance_th =
      node.declare_parameter<double>(module_name + ".stop_planning.update_distance_th", 0.5);

    stop_planning.hold_stop_velocity_threshold = node.declare_parameter<double>(
      module_name + ".stop_planning.hold_stop_velocity_threshold", 0.01);
    stop_planning.hold_stop_distance_threshold = node.declare_parameter<double>(
      module_name + ".stop_planning.hold_stop_distance_threshold", 0.3);

    // Stop on curve parameters
    stop_planning.stop_on_curve.enable_approaching = node.declare_parameter<bool>(
      module_name + ".stop_planning.stop_on_curve.enable_approaching", true);
    stop_planning.stop_on_curve.additional_stop_margin = node.declare_parameter<double>(
      module_name + ".stop_planning.stop_on_curve.additional_stop_margin", 0.5);
    stop_planning.stop_on_curve.min_stop_margin = node.declare_parameter<double>(
      module_name + ".stop_planning.stop_on_curve.min_stop_margin", 1.0);

    // Common parameters for all object types
    stop_planning.limit_min_acc =
      node.declare_parameter<double>(module_name + ".stop_planning.limit_min_acc", -2.5);
    stop_planning.sudden_object_acc_threshold = node.declare_parameter<double>(
      module_name + ".stop_planning.sudden_object_acc_threshold", -1.0);
    stop_planning.sudden_object_dist_threshold = node.declare_parameter<double>(
      module_name + ".stop_planning.sudden_object_dist_threshold", 1000.0);
    stop_planning.abandon_to_stop =
      node.declare_parameter<bool>(module_name + ".stop_planning.abandon_to_stop", false);

    // Obstacle filtering parameters
    obstacle_filtering.object_type.target_types.pedestrian = node.declare_parameter<bool>(
      module_name + ".obstacle_filtering.object_type.target_types.pedestrian", true);
    obstacle_filtering.object_type.target_types.bicycle = node.declare_parameter<bool>(
      module_name + ".obstacle_filtering.object_type.target_types.bicycle", true);
    obstacle_filtering.object_type.target_types.motorcycle = node.declare_parameter<bool>(
      module_name + ".obstacle_filtering.object_type.target_types.motorcycle", true);
    obstacle_filtering.object_type.target_types.unknown = node.declare_parameter<bool>(
      module_name + ".obstacle_filtering.object_type.target_types.unknown", true);

    obstacle_filtering.adjacent_lane_margin =
      node.declare_parameter<double>(module_name + ".obstacle_filtering.adjacent_lane_margin", 3.0);

    obstacle_filtering.exclude_crosswalk_users = node.declare_parameter<bool>(
      module_name + ".obstacle_filtering.exclude_crosswalk_users", true);
    obstacle_filtering.crosswalk_margin =
      node.declare_parameter<double>(module_name + ".obstacle_filtering.crosswalk_margin", 1.0);
    obstacle_filtering.exclude_sidewalk_users = node.declare_parameter<bool>(
      module_name + ".obstacle_filtering.exclude_sidewalk_users", true);

    obstacle_filtering.wrong_way_detection.enable = node.declare_parameter<bool>(
      module_name + ".obstacle_filtering.wrong_way_detection.enable", true);
    obstacle_filtering.wrong_way_detection.angle_threshold = node.declare_parameter<double>(
      module_name + ".obstacle_filtering.wrong_way_detection.angle_threshold", 150.0);
    obstacle_filtering.wrong_way_detection.min_speed_threshold = node.declare_parameter<double>(
      module_name + ".obstacle_filtering.wrong_way_detection.min_speed_threshold", 0.5);

    obstacle_filtering.min_detection_duration = node.declare_parameter<double>(
      module_name + ".obstacle_filtering.min_detection_duration", 0.1);

    // Common parameters (these would normally come from common_param.yaml)
    common_param.limit_max_accel = node.declare_parameter<double>("common.limit.max_acc", 1.0);
    common_param.limit_min_accel = node.declare_parameter<double>("common.limit.min_acc", -2.5);
  }
};

}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
