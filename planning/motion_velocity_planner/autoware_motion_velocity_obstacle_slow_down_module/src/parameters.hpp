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
#include <autoware_utils_rclcpp/parameter.hpp>

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

/// @brief Get the parameter defined for a specific object label, or the default value if it was
/// not specified
template <class T>
auto get_object_parameter(
  rclcpp::Node & node, const std::string & ns, const std::string & object_label,
  const std::string & param)
{
  using autoware_utils::get_or_declare_parameter;
  try {
    return get_or_declare_parameter<T>(node, ns + object_label + "." + param);
  } catch (const std::exception &) {
    return get_or_declare_parameter<T>(node, ns + "default." + param);
  }
}

struct VelocityInterpolationParam
{
  double min_lat_margin;
  double max_lat_margin;
  double min_ego_velocity;
  double max_ego_velocity;
};

static const std::unordered_map<uint8_t, std::string> object_types_maps = {
  {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
  {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
  {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
  {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
static const std::unordered_map<Side, std::string> side_to_string_map = {
  {Side::Left, "left"}, {Side::Right, "right"}};
static const std::unordered_map<Motion, std::string> motion_to_string_map = {
  {Motion::Moving, "moving"}, {Motion::Static, "static"}};

struct ObjectTypeSpecificParams
{
  std::array<
    std::array<VelocityInterpolationParam, static_cast<size_t>(Side::Count)>,
    static_cast<size_t>(Motion::Count)>
    params;

  VelocityInterpolationParam & get_param(const Side side, const Motion motion)
  {
    return params[static_cast<size_t>(side)][static_cast<size_t>(motion)];
  }

  [[nodiscard]] VelocityInterpolationParam get_param(const Side side, const Motion motion) const
  {
    return params[static_cast<size_t>(side)][static_cast<size_t>(motion)];
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

  std::unordered_map<std::string, ObjectTypeSpecificParams>
    object_type_specific_param_per_object_type;

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

    const auto to_param_name = [&](const Side s, const Motion m, const std::string & str) {
      return side_to_string_map.at(s) + "." + motion_to_string_map.at(m) + "." + str;
    };

    for (const auto & [_, type_str] : object_types_maps) {
      ObjectTypeSpecificParams param{};
      for (const auto side : {Side::Left, Side::Right}) {
        for (const auto motion : {Motion::Moving, Motion::Static}) {
          auto & p = param.get_param(side, motion);
          p.min_lat_margin = get_object_parameter<double>(
            node, param_prefix, type_str, to_param_name(side, motion, "min_lat_margin"));
          p.max_lat_margin = get_object_parameter<double>(
            node, param_prefix, type_str, to_param_name(side, motion, "max_lat_margin"));
          p.min_ego_velocity = get_object_parameter<double>(
            node, param_prefix, type_str, to_param_name(side, motion, "min_ego_velocity"));
          p.max_ego_velocity = get_object_parameter<double>(
            node, param_prefix, type_str, to_param_name(side, motion, "max_ego_velocity"));
        }
      }
      object_type_specific_param_per_object_type.emplace(type_str, param);
    }
  }

  VelocityInterpolationParam get_object_param(
    const ObjectClassification label, const Side side, const Motion motion) const
  {
    const auto & type_str = object_types_maps.at(label.label);
    return object_type_specific_param_per_object_type.at(type_str).get_param(side, motion);
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
