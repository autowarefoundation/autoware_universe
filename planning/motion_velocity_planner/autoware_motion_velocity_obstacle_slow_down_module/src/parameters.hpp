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

#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_utils::get_or_declare_parameter;

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

struct ObstacleFilteringParam
{
  PointcloudObstacleFilteringParam pointcloud_obstacle_filtering_param;
  std::vector<uint8_t> object_types{};

  bool use_pointcloud{false};

  double min_lat_margin{};
  double max_lat_margin{};

  double lat_hysteresis_margin{};

  int successive_num_to_entry_slow_down_condition{};
  int successive_num_to_exit_slow_down_condition{};

  ObstacleFilteringParam() = default;
  explicit ObstacleFilteringParam(rclcpp::Node & node)
  {
    use_pointcloud = get_or_declare_parameter<bool>(
      node, "obstacle_slow_down.obstacle_filtering.object_type.pointcloud");
    object_types =
      utils::get_target_object_type(node, "obstacle_slow_down.obstacle_filtering.object_type.");
    min_lat_margin = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.obstacle_filtering.min_lat_margin");
    max_lat_margin = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.obstacle_filtering.max_lat_margin");
    lat_hysteresis_margin = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.obstacle_filtering.lat_hysteresis_margin");
    successive_num_to_entry_slow_down_condition = get_or_declare_parameter<int>(
      node, "obstacle_slow_down.obstacle_filtering.successive_num_to_entry_slow_down_condition");
    successive_num_to_exit_slow_down_condition = get_or_declare_parameter<int>(
      node, "obstacle_slow_down.obstacle_filtering.successive_num_to_exit_slow_down_condition");
  }
};

struct SlowDownPlanningParam
{
  double slow_down_min_acc{};
  double slow_down_min_jerk{};

  double lpf_gain_slow_down_vel{};
  double lpf_gain_lat_dist{};
  double lpf_gain_dist_to_slow_down{};

  double time_margin_on_target_velocity{};

  double moving_object_speed_threshold{};
  double moving_object_hysteresis_range{};

  std::vector<std::string> obstacle_labels{"default"};
  std::vector<std::string> obstacle_moving_classification{"static", "moving"};
  struct ObjectTypeSpecificParams
  {
    double min_lat_margin;
    double max_lat_margin;
    double min_ego_velocity;
    double max_ego_velocity;
  };
  std::unordered_map<uint8_t, std::string> object_types_maps = {
    {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
  std::unordered_map<std::string, ObjectTypeSpecificParams> object_type_specific_param_map;

  SlowDownPlanningParam() = default;
  explicit SlowDownPlanningParam(rclcpp::Node & node)
  {
    slow_down_min_acc = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.slow_down_min_acc");
    slow_down_min_jerk = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.slow_down_min_jerk");

    lpf_gain_slow_down_vel = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.lpf_gain_slow_down_vel");
    lpf_gain_lat_dist = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.lpf_gain_lat_dist");
    lpf_gain_dist_to_slow_down = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.lpf_gain_dist_to_slow_down");
    time_margin_on_target_velocity = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.time_margin_on_target_velocity");

    moving_object_speed_threshold = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.moving_object_speed_threshold");
    moving_object_hysteresis_range = get_or_declare_parameter<double>(
      node, "obstacle_slow_down.slow_down_planning.moving_object_hysteresis_range");

    const std::string param_prefix =
      "obstacle_slow_down.slow_down_planning.object_type_specified_params.";
    const auto object_types =
      get_or_declare_parameter<std::vector<std::string>>(node, param_prefix + "types");

    for (const auto & type_str : object_types) {
      for (const auto & movement_type : std::vector<std::string>{"moving", "static"}) {
        ObjectTypeSpecificParams param{
          get_or_declare_parameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".min_lat_margin"),
          get_or_declare_parameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".max_lat_margin"),
          get_or_declare_parameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".min_ego_velocity"),
          get_or_declare_parameter<double>(
            node, param_prefix + type_str + "." + movement_type + ".max_ego_velocity")};

        object_type_specific_param_map.emplace(type_str + "." + movement_type, param);
      }
    }
  }

  ObjectTypeSpecificParams get_object_param_by_label(
    const ObjectClassification label, const bool is_obstacle_moving) const
  {
    const auto type_str = object_types_maps.at(label.label);
    const std::string movement_type = is_obstacle_moving ? "moving" : "static";
    const std::string param_key = type_str + "." + movement_type;

    // First, search for parameters with the specified type
    const auto param_it = object_type_specific_param_map.find(param_key);
    if (param_it != object_type_specific_param_map.end()) {
      return param_it->second;
    }

    // If the specified type is not found, use default parameters
    const std::string default_key = "default." + movement_type;
    return object_type_specific_param_map.at(default_key);
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
