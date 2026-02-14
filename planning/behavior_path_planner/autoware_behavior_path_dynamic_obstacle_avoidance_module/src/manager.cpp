// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_dynamic_obstacle_avoidance_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
void DynamicObstacleAvoidanceModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  DynamicAvoidanceParameters p{};

  {  // common
    const std::string ns = "dynamic_avoidance.common.";
    p.enable_debug_info = node->declare_parameter<bool>(ns + "enable_debug_info");
    p.use_hatched_road_markings = node->declare_parameter<bool>(ns + "use_hatched_road_markings");
  }

  {  // target object
    const std::string ns = "dynamic_avoidance.target_object.";
    p.avoid_car = node->declare_parameter<bool>(ns + "car");
    p.avoid_truck = node->declare_parameter<bool>(ns + "truck");
    p.avoid_bus = node->declare_parameter<bool>(ns + "bus");
    p.avoid_trailer = node->declare_parameter<bool>(ns + "trailer");
    p.avoid_unknown = node->declare_parameter<bool>(ns + "unknown");
    p.avoid_bicycle = node->declare_parameter<bool>(ns + "bicycle");
    p.avoid_motorcycle = node->declare_parameter<bool>(ns + "motorcycle");
    p.avoid_pedestrian = node->declare_parameter<bool>(ns + "pedestrian");
    p.successive_num_to_entry_dynamic_avoidance_condition =
      node->declare_parameter<int>(ns + "successive_num_to_entry_dynamic_avoidance_condition");
    p.successive_num_to_exit_dynamic_avoidance_condition =
      node->declare_parameter<int>(ns + "successive_num_to_exit_dynamic_avoidance_condition");

    p.min_obj_lat_offset_to_ego_path =
      node->declare_parameter<double>(ns + "min_obj_lat_offset_to_ego_path");
    p.max_obj_lat_offset_to_ego_path =
      node->declare_parameter<double>(ns + "max_obj_lat_offset_to_ego_path");
    p.max_front_object_ego_path_lat_cover_ratio =
      node->declare_parameter<double>(ns + "front_object.max_ego_path_lat_cover_ratio");
    p.min_front_object_vel = node->declare_parameter<double>(ns + "front_object.min_object_vel");

    p.max_stopped_object_vel =
      node->declare_parameter<double>(ns + "stopped_object.max_object_vel");
  }

  {  // drivable_area_generation
    const std::string ns = "dynamic_avoidance.drivable_area_generation.";
    p.lat_offset_from_obstacle = node->declare_parameter<double>(ns + "lat_offset_from_obstacle");
    p.margin_distance_around_pedestrian =
      node->declare_parameter<double>(ns + "margin_distance_around_pedestrian");
    p.max_lat_offset_to_avoid = node->declare_parameter<double>(ns + "max_lat_offset_to_avoid");
    p.max_time_for_lat_shift =
      node->declare_parameter<double>(ns + "max_time_for_object_lat_shift");
    p.lpf_gain_for_lat_avoid_to_offset =
      node->declare_parameter<double>(ns + "lpf_gain_for_lat_avoid_to_offset");
  }

  parameters_ = std::make_shared<DynamicAvoidanceParameters>(p);
}

void DynamicObstacleAvoidanceModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto & p = parameters_;

  {  // common
    const std::string ns = "dynamic_avoidance.common.";
    update_param<bool>(parameters, ns + "enable_debug_info", p->enable_debug_info);
    update_param<bool>(parameters, ns + "use_hatched_road_markings", p->use_hatched_road_markings);
  }

  {  // target object
    const std::string ns = "dynamic_avoidance.target_object.";

    update_param<bool>(parameters, ns + "car", p->avoid_car);
    update_param<bool>(parameters, ns + "truck", p->avoid_truck);
    update_param<bool>(parameters, ns + "bus", p->avoid_bus);
    update_param<bool>(parameters, ns + "trailer", p->avoid_trailer);
    update_param<bool>(parameters, ns + "unknown", p->avoid_unknown);
    update_param<bool>(parameters, ns + "bicycle", p->avoid_bicycle);
    update_param<bool>(parameters, ns + "motorcycle", p->avoid_motorcycle);
    update_param<bool>(parameters, ns + "pedestrian", p->avoid_pedestrian);

    update_param<int>(
      parameters, ns + "successive_num_to_entry_dynamic_avoidance_condition",
      p->successive_num_to_entry_dynamic_avoidance_condition);
    update_param<int>(
      parameters, ns + "successive_num_to_exit_dynamic_avoidance_condition",
      p->successive_num_to_exit_dynamic_avoidance_condition);

    update_param<double>(
      parameters, ns + "min_obj_lat_offset_to_ego_path", p->min_obj_lat_offset_to_ego_path);
    update_param<double>(
      parameters, ns + "max_obj_lat_offset_to_ego_path", p->max_obj_lat_offset_to_ego_path);
    update_param<double>(
      parameters, ns + "front_object.max_ego_path_lat_cover_ratio",
      p->max_front_object_ego_path_lat_cover_ratio);
    update_param<double>(parameters, ns + "front_object.min_object_vel", p->min_front_object_vel);

    update_param<double>(
      parameters, ns + "stopped_object.max_object_vel", p->max_stopped_object_vel);
  }

  {  // drivable_area_generation
    const std::string ns = "dynamic_avoidance.drivable_area_generation.";
    update_param<double>(parameters, ns + "lat_offset_from_obstacle", p->lat_offset_from_obstacle);
    update_param<double>(
      parameters, ns + "margin_distance_around_pedestrian", p->margin_distance_around_pedestrian);
    update_param<double>(parameters, ns + "max_lat_offset_to_avoid", p->max_lat_offset_to_avoid);
    update_param<double>(
      parameters, ns + "max_time_for_object_lat_shift", p->max_time_for_lat_shift);
    update_param<double>(
      parameters, ns + "lpf_gain_for_lat_avoid_to_offset", p->lpf_gain_for_lat_avoid_to_offset);
  }

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::DynamicObstacleAvoidanceModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
