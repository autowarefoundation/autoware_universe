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

#include "manager.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::get_or_declare_parameter;
RunOutModuleManager::RunOutModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  // Vehicle Parameters
  {
    const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
    auto & p = planner_param_.vehicle_param;
    p.base_to_front = vehicle_info.wheel_base_m + vehicle_info.front_overhang_m;
    p.base_to_rear = vehicle_info.rear_overhang_m;
    p.width = vehicle_info.vehicle_width_m;
    p.wheel_tread = vehicle_info.wheel_tread_m;
    p.right_overhang = vehicle_info.right_overhang_m;
    p.left_overhang = vehicle_info.left_overhang_m;
  }

  const std::string ns(RunOutModuleManager::getModuleName());

  {
    auto & p = planner_param_.smoother;
    // smoother parameters are already declared in behavior velocity node, so use get parameter
    p.start_jerk = node.get_parameter("backward.start_jerk").get_value<double>();
  }

  {
    auto & p = planner_param_.common;
    p.normal_min_jerk = get_or_declare_parameter<double>(node, "normal.min_jerk");
    p.normal_min_acc = get_or_declare_parameter<double>(node, "normal.min_acc");
    p.limit_min_jerk = get_or_declare_parameter<double>(node, "limit.min_jerk");
    p.limit_min_acc = get_or_declare_parameter<double>(node, "limit.min_acc");
  }

  {
    auto & p = planner_param_.run_out;
    p.detection_method = get_or_declare_parameter<std::string>(node, ns + ".detection_method");
    p.target_obstacle_types =
      get_or_declare_parameter<std::vector<std::string>>(node, ns + ".target_obstacle_types");
    p.use_partition_lanelet = get_or_declare_parameter<bool>(node, ns + ".use_partition_lanelet");
    p.use_ego_cut_line = get_or_declare_parameter<bool>(node, ns + ".use_ego_cut_line");
    p.exclude_obstacles_already_in_path =
      get_or_declare_parameter<bool>(node, ns + ".exclude_obstacles_already_in_path");
    p.suppress_on_crosswalk = get_or_declare_parameter<bool>(node, ns + ".suppress_on_crosswalk");
    p.specify_decel_jerk = get_or_declare_parameter<bool>(node, ns + ".specify_decel_jerk");
    p.stop_margin = get_or_declare_parameter<double>(node, ns + ".stop_margin");
    p.passing_margin = get_or_declare_parameter<double>(node, ns + ".passing_margin");
    p.deceleration_jerk = get_or_declare_parameter<double>(node, ns + ".deceleration_jerk");
    p.detection_distance = get_or_declare_parameter<double>(node, ns + ".detection_distance");
    p.detection_span = get_or_declare_parameter<double>(node, ns + ".detection_span");
    p.min_vel_ego_kmph = get_or_declare_parameter<double>(node, ns + ".min_vel_ego_kmph");
    p.ego_cut_line_length = get_or_declare_parameter<double>(node, ns + ".ego_cut_line_length");
    p.ego_footprint_extra_margin =
      get_or_declare_parameter<double>(node, ns + ".ego_footprint_extra_margin");
    p.keep_obstacle_on_path_time_threshold =
      get_or_declare_parameter<double>(node, ns + ".keep_obstacle_on_path_time_threshold");
    p.keep_stop_point_time = get_or_declare_parameter<double>(node, ns + ".keep_stop_point_time");
  }

  {
    auto & p = planner_param_.detection_area;
    const std::string ns_da = ns + ".detection_area";
    p.margin_ahead = get_or_declare_parameter<double>(node, ns_da + ".margin_ahead");
    p.margin_behind = get_or_declare_parameter<double>(node, ns_da + ".margin_behind");
  }

  {
    auto & p = planner_param_.mandatory_area;
    const std::string ns_da = ns + ".mandatory_area";
    p.decel_jerk = get_or_declare_parameter<double>(node, ns_da + ".decel_jerk");
  }

  {
    auto & p = planner_param_.dynamic_obstacle;
    const std::string ns_do = ns + ".dynamic_obstacle";
    p.use_mandatory_area = get_or_declare_parameter<bool>(node, ns_do + ".use_mandatory_area");
    p.assume_fixed_velocity =
      get_or_declare_parameter<bool>(node, ns_do + ".assume_fixed_velocity.enable");
    p.min_vel_kmph =
      get_or_declare_parameter<double>(node, ns_do + ".assume_fixed_velocity.min_vel_kmph");
    p.max_vel_kmph =
      get_or_declare_parameter<double>(node, ns_do + ".assume_fixed_velocity.max_vel_kmph");
    p.std_dev_multiplier = get_or_declare_parameter<double>(node, ns_do + ".std_dev_multiplier");
    p.diameter = get_or_declare_parameter<double>(node, ns_do + ".diameter");
    p.height = get_or_declare_parameter<double>(node, ns_do + ".height");
    p.max_prediction_time = get_or_declare_parameter<double>(node, ns_do + ".max_prediction_time");
    p.time_step = get_or_declare_parameter<double>(node, ns_do + ".time_step");
    p.points_interval = get_or_declare_parameter<double>(node, ns_do + ".points_interval");
  }

  {
    auto & p = planner_param_.approaching;
    const std::string ns_a = ns + ".approaching";
    p.enable = get_or_declare_parameter<bool>(node, ns_a + ".enable");
    p.margin = get_or_declare_parameter<double>(node, ns_a + ".margin");
    p.limit_vel_kmph = get_or_declare_parameter<double>(node, ns_a + ".limit_vel_kmph");

    const std::string ns_as = ns_a + ".state";
    p.state.stop_thresh = get_or_declare_parameter<double>(node, ns_as + ".stop_thresh");
    p.state.stop_time_thresh = get_or_declare_parameter<double>(node, ns_as + ".stop_time_thresh");
    p.state.disable_approach_dist =
      get_or_declare_parameter<double>(node, ns_as + ".disable_approach_dist");
    p.state.keep_approach_duration =
      get_or_declare_parameter<double>(node, ns_as + ".keep_approach_duration");
  }

  {
    auto & p = planner_param_.slow_down_limit;
    const std::string ns_m = ns + ".slow_down_limit";
    p.enable = get_or_declare_parameter<bool>(node, ns_m + ".enable");
    p.max_jerk = get_or_declare_parameter<double>(node, ns_m + ".max_jerk");
    p.max_acc = get_or_declare_parameter<double>(node, ns_m + ".max_acc");
  }

  {
    auto & p = planner_param_.ignore_momentary_detection;
    const std::string ns_param = ns + ".ignore_momentary_detection";
    p.enable = get_or_declare_parameter<bool>(node, ns_param + ".enable");
    p.time_threshold = get_or_declare_parameter<double>(node, ns_param + ".time_threshold");
  }

  debug_ptr_ = std::make_shared<RunOutDebug>(node);
  setDynamicObstacleCreator(node, debug_ptr_);
}

void RunOutModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) {
    return;
  }

  constexpr int64_t module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(
      std::make_shared<RunOutModule>(
        module_id, planner_data_, planner_param_, logger_.get_child("run_out_module"),
        std::move(dynamic_obstacle_creator_), debug_ptr_, clock_, time_keeper_,
        planning_factor_interface_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
RunOutModuleManager::getModuleExpiredFunction(
  [[maybe_unused]] const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  return []([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) -> bool {
    return false;
  };
}

void RunOutModuleManager::setDynamicObstacleCreator(
  rclcpp::Node & node, std::shared_ptr<RunOutDebug> & debug_ptr)
{
  using run_out_utils::DetectionMethod;

  const auto detection_method_enum = run_out_utils::toEnum(planner_param_.run_out.detection_method);
  switch (detection_method_enum) {
    case DetectionMethod::Object:
      dynamic_obstacle_creator_ = std::make_unique<DynamicObstacleCreatorForObject>(
        node, debug_ptr, planner_param_.dynamic_obstacle);
      break;

    case DetectionMethod::ObjectWithoutPath:
      dynamic_obstacle_creator_ = std::make_unique<DynamicObstacleCreatorForObjectWithoutPath>(
        node, debug_ptr, planner_param_.dynamic_obstacle);
      break;

    case DetectionMethod::Points:
      dynamic_obstacle_creator_ = std::make_unique<DynamicObstacleCreatorForPoints>(
        node, debug_ptr, planner_param_.dynamic_obstacle);
      break;

    default:
      RCLCPP_WARN_STREAM(logger_, "detection method is invalid. use default method (Object).");
      dynamic_obstacle_creator_ = std::make_unique<DynamicObstacleCreatorForObject>(
        node, debug_ptr, planner_param_.dynamic_obstacle);
      break;
  }
}
}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::RunOutModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
