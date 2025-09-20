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

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/motion_velocity_planner_common/utils.hpp"
#include "autoware/object_recognition_utils/predicted_path_utils.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/ros/update_param.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <algorithm>
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
  std::vector<uint8_t> inside_object_types{};
  std::vector<uint8_t> outside_object_types{};
  std::vector<uint8_t> side_stopped_object_types{};
  // bool use_pointcloud{};

  double max_lat_margin{};

  double crossing_obstacle_velocity_threshold{};
  double crossing_obstacle_traj_angle_threshold{};

  double outside_obstacle_velocity_threshold{};
  double ego_obstacle_overlap_time_threshold{};
  double max_prediction_time_for_collision_check{};
  double max_lateral_time_margin{};
  int num_of_predicted_paths_for_outside_cruise_obstacle{};

  bool enable_yield{};
  double yield_lat_distance_threshold{};
  double max_lat_dist_between_obstacles{};
  double max_obstacles_collision_time{};
  double stopped_obstacle_velocity_threshold{};

  double obstacle_velocity_threshold_from_cruise{};
  double obstacle_velocity_threshold_to_cruise{};

  ObstacleFilteringParam() = default;
  explicit ObstacleFilteringParam(rclcpp::Node & node)
  {
    inside_object_types =
      utils::get_target_object_type(node, "obstacle_cruise.obstacle_filtering.object_type.inside.");
    outside_object_types = utils::get_target_object_type(
      node, "obstacle_cruise.obstacle_filtering.object_type.outside.");
    side_stopped_object_types = utils::get_target_object_type(
      node, "obstacle_cruise.obstacle_filtering.object_type.side_stopped.");
    // use_pointcloud = get_or_declare_parameter<bool>(
    //   node, "obstacle_cruise.obstacle_filtering.object_type.pointcloud");

    max_lat_margin =
      get_or_declare_parameter<double>(node, "obstacle_cruise.obstacle_filtering.max_lat_margin");

    crossing_obstacle_velocity_threshold = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.crossing_obstacle.obstacle_velocity_threshold");
    crossing_obstacle_traj_angle_threshold = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.crossing_obstacle.obstacle_traj_angle_threshold");

    outside_obstacle_velocity_threshold = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.outside_obstacle.obstacle_velocity_threshold");
    ego_obstacle_overlap_time_threshold = get_or_declare_parameter<double>(
      node,
      "obstacle_cruise.obstacle_filtering.outside_obstacle.ego_obstacle_overlap_time_threshold");
    max_prediction_time_for_collision_check = get_or_declare_parameter<double>(
      node,
      "obstacle_cruise.obstacle_filtering.outside_obstacle.max_prediction_time_for_collision_"
      "check");
    max_lateral_time_margin = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.outside_obstacle.max_lateral_time_margin");
    num_of_predicted_paths_for_outside_cruise_obstacle = get_or_declare_parameter<int>(
      node, "obstacle_cruise.obstacle_filtering.outside_obstacle.num_of_predicted_paths");
    enable_yield =
      get_or_declare_parameter<bool>(node, "obstacle_cruise.obstacle_filtering.yield.enable_yield");
    yield_lat_distance_threshold = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.yield.lat_distance_threshold");
    max_lat_dist_between_obstacles = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.yield.max_lat_dist_between_obstacles");
    max_obstacles_collision_time = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.yield.max_obstacles_collision_time");
    stopped_obstacle_velocity_threshold = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.yield.stopped_obstacle_velocity_threshold");
    obstacle_velocity_threshold_from_cruise = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.obstacle_velocity_threshold_from_cruise");
    obstacle_velocity_threshold_to_cruise = get_or_declare_parameter<double>(
      node, "obstacle_cruise.obstacle_filtering.obstacle_velocity_threshold_to_cruise");
  }
};

struct CruisePlanningParam
{
  double idling_time{};
  double min_ego_accel_for_rss{};
  double min_object_accel_for_rss{};
  double safe_distance_margin{};

  CruisePlanningParam() = default;
  explicit CruisePlanningParam(rclcpp::Node & node)
  {
    idling_time =
      get_or_declare_parameter<double>(node, "obstacle_cruise.cruise_planning.idling_time");
    min_ego_accel_for_rss = get_or_declare_parameter<double>(
      node, "obstacle_cruise.cruise_planning.min_ego_accel_for_rss");
    min_object_accel_for_rss = get_or_declare_parameter<double>(
      node, "obstacle_cruise.cruise_planning.min_object_accel_for_rss");
    safe_distance_margin = get_or_declare_parameter<double>(
      node, "obstacle_cruise.cruise_planning.safe_distance_margin");
  }
};
}  // namespace autoware::motion_velocity_planner

#endif  // PARAMETERS_HPP_
