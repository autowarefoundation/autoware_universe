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

#include "autoware/behavior_path_start_planner_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
void StartPlannerModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  StartPlannerParameters parameters = StartPlannerParameters::init(*node);
  // validation of parameters
  if (parameters.lateral_acceleration_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      node->get_logger().get_child(name()),
      "lateral_acceleration_sampling_num must be positive integer. Given parameter: "
        << parameters.lateral_acceleration_sampling_num << std::endl
        << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  parameters_ = std::make_shared<StartPlannerParameters>(parameters);
}

void StartPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto & p = parameters_;

  {
    const std::string ns = "start_planner.";
    update_param<double>(parameters, ns + "th_arrived_distance", p->th_arrived_distance);
    update_param<double>(parameters, ns + "th_stopped_velocity", p->th_stopped_velocity);
    update_param<double>(parameters, ns + "th_stopped_time", p->th_stopped_time);
    update_param<double>(
      parameters, ns + "prepare_time_before_start", p->prepare_time_before_start);
    update_param<double>(
      parameters, ns + "th_distance_to_middle_of_the_road", p->th_distance_to_middle_of_the_road);
    update_param<bool>(parameters, ns + "skip_rear_vehicle_check", p->skip_rear_vehicle_check);
    update_param<double>(
      parameters, ns + "extra_width_margin_for_rear_obstacle",
      p->extra_width_margin_for_rear_obstacle);
    update_param<std::vector<double>>(
      parameters, ns + "collision_check_margins", p->collision_check_margins);
    update_param<double>(
      parameters, ns + "collision_check_margin_from_front_object",
      p->collision_check_margin_from_front_object);
    update_param<double>(
      parameters, ns + "th_moving_object_velocity", p->th_moving_object_velocity);
    const std::string obj_types_ns = ns + "object_types_to_check_for_path_generation.";
    {
      update_param<bool>(
        parameters, obj_types_ns + "check_car",
        p->object_types_to_check_for_path_generation.check_car);
      update_param<bool>(
        parameters, obj_types_ns + "check_truck",
        p->object_types_to_check_for_path_generation.check_truck);
      update_param<bool>(
        parameters, obj_types_ns + "check_bus",
        p->object_types_to_check_for_path_generation.check_bus);
      update_param<bool>(
        parameters, obj_types_ns + "check_trailer",
        p->object_types_to_check_for_path_generation.check_trailer);
      update_param<bool>(
        parameters, obj_types_ns + "check_unknown",
        p->object_types_to_check_for_path_generation.check_unknown);
      update_param<bool>(
        parameters, obj_types_ns + "check_bicycle",
        p->object_types_to_check_for_path_generation.check_bicycle);
      update_param<bool>(
        parameters, obj_types_ns + "check_motorcycle",
        p->object_types_to_check_for_path_generation.check_motorcycle);
      update_param<bool>(
        parameters, obj_types_ns + "check_pedestrian",
        p->object_types_to_check_for_path_generation.check_pedestrian);
    }
    update_param<double>(
      parameters, ns + "center_line_path_interval", p->center_line_path_interval);
    update_param<bool>(parameters, ns + "enable_shift_pull_out", p->enable_shift_pull_out);
    update_param<double>(
      parameters, ns + "shift_collision_check_distance_from_end",
      p->shift_collision_check_distance_from_end);
    update_param<double>(
      parameters, ns + "minimum_shift_pull_out_distance", p->minimum_shift_pull_out_distance);
    update_param<int>(
      parameters, ns + "lateral_acceleration_sampling_num", p->lateral_acceleration_sampling_num);
    update_param<double>(parameters, ns + "lateral_jerk", p->lateral_jerk);
    update_param<double>(parameters, ns + "maximum_lateral_acc", p->maximum_lateral_acc);
    update_param<double>(parameters, ns + "minimum_lateral_acc", p->minimum_lateral_acc);
    update_param<double>(parameters, ns + "maximum_curvature", p->maximum_curvature);
    update_param<double>(
      parameters, ns + "end_pose_curvature_threshold", p->end_pose_curvature_threshold);
    update_param<double>(
      parameters, ns + "maximum_longitudinal_deviation", p->maximum_longitudinal_deviation);
    update_param<bool>(parameters, ns + "enable_geometric_pull_out", p->enable_geometric_pull_out);
    update_param<bool>(parameters, ns + "divide_pull_out_path", p->divide_pull_out_path);
    update_param<bool>(parameters, ns + "enable_clothoid_fallback", p->enable_clothoid_fallback);
    update_param<double>(
      parameters, ns + "arc_path_interval",
      p->parallel_parking_parameters.pull_out_arc_path_interval);
    update_param<double>(
      parameters, ns + "lane_departure_margin",
      p->parallel_parking_parameters.pull_out_lane_departure_margin);
    update_param<double>(
      parameters, ns + "lane_departure_check_expansion_margin",
      p->lane_departure_check_expansion_margin);
    update_param<double>(
      parameters, ns + "geometric_pull_out_max_steer_angle_margin_scale",
      p->parallel_parking_parameters.geometric_pull_out_max_steer_angle_margin_scale);

    // clothoid pull out
    update_param<double>(
      parameters, ns + "clothoid_initial_velocity", p->clothoid_initial_velocity);
    update_param<double>(parameters, ns + "clothoid_acceleration", p->clothoid_acceleration);
    update_param<std::vector<double>>(
      parameters, ns + "clothoid_max_steer_angles_deg", p->clothoid_max_steer_angles_deg);
    update_param<double>(
      parameters, ns + "clothoid_max_steer_angle_rate_deg_per_sec",
      p->clothoid_max_steer_angle_rate_deg_per_sec);
    update_param<bool>(
      parameters, ns + "check_clothoid_path_lane_departure", p->check_clothoid_path_lane_departure);

    // search start pose backward
    update_param<bool>(parameters, ns + "enable_back", p->enable_back);
    update_param<double>(parameters, ns + "backward_velocity", p->backward_velocity);
    update_param<double>(
      parameters, ns + "geometric_pull_out_velocity",
      p->parallel_parking_parameters.pull_out_velocity);
    update_param<double>(
      parameters, ns + "geometric_collision_check_distance_from_end",
      p->geometric_collision_check_distance_from_end);
    update_param<bool>(
      parameters, ns + "check_shift_path_lane_departure", p->check_shift_path_lane_departure);
    update_param<bool>(
      parameters, ns + "allow_check_shift_path_lane_departure_override",
      p->allow_check_shift_path_lane_departure_override);
    update_param<std::string>(parameters, ns + "search_priority", p->search_priority);
    update_param<double>(parameters, ns + "max_back_distance", p->max_back_distance);
    update_param<double>(
      parameters, ns + "backward_search_resolution", p->backward_search_resolution);
    update_param<double>(
      parameters, ns + "backward_path_update_duration", p->backward_path_update_duration);
    update_param<double>(
      parameters, ns + "ignore_distance_from_lane_end", p->ignore_distance_from_lane_end);
    update_param<double>(
      parameters, ns + "stop_condition.maximum_deceleration_for_stop",
      p->maximum_deceleration_for_stop);
    update_param<double>(
      parameters, ns + "stop_condition.maximum_jerk_for_stop", p->maximum_jerk_for_stop);
  }
  {
    const std::string ns = "start_planner.freespace_planner.";

    update_param<bool>(parameters, ns + "enable_freespace_planner", p->enable_freespace_planner);
    update_param<std::string>(
      parameters, ns + "freespace_planner_algorithm", p->freespace_planner_algorithm);
    update_param<double>(
      parameters, ns + "end_pose_search_start_distance", p->end_pose_search_start_distance);
    update_param<double>(
      parameters, ns + "end_pose_search_end_distance", p->end_pose_search_end_distance);
    update_param<double>(parameters, ns + "end_pose_search_interval", p->end_pose_search_interval);
    update_param<double>(parameters, ns + "velocity", p->freespace_planner_velocity);
    update_param<double>(parameters, ns + "vehicle_shape_margin", p->vehicle_shape_margin);
    update_param<double>(
      parameters, ns + "time_limit", p->freespace_planner_common_parameters.time_limit);
    update_param<double>(
      parameters, ns + "max_turning_ratio",
      p->freespace_planner_common_parameters.max_turning_ratio);
    update_param<int>(
      parameters, ns + "turning_steps", p->freespace_planner_common_parameters.turning_steps);
  }
  {
    const std::string ns = "start_planner.freespace_planner.search_configs.";

    update_param<int>(
      parameters, ns + "theta_size", p->freespace_planner_common_parameters.theta_size);
    update_param<double>(
      parameters, ns + "angle_goal_range", p->freespace_planner_common_parameters.angle_goal_range);
    update_param<double>(
      parameters, ns + "curve_weight", p->freespace_planner_common_parameters.curve_weight);
    update_param<double>(
      parameters, ns + "reverse_weight", p->freespace_planner_common_parameters.reverse_weight);
    update_param<double>(
      parameters, ns + "lateral_goal_range",
      p->freespace_planner_common_parameters.lateral_goal_range);
    update_param<double>(
      parameters, ns + "longitudinal_goal_range",
      p->freespace_planner_common_parameters.longitudinal_goal_range);
  }

  {
    const std::string ns = "start_planner.freespace_planner.costmap_configs.";

    update_param<int>(
      parameters, ns + "obstacle_threshold",
      p->freespace_planner_common_parameters.obstacle_threshold);
  }

  {
    const std::string ns = "start_planner.freespace_planner.astar.";

    update_param<std::string>(parameters, ns + "search_method", p->astar_parameters.search_method);
    update_param<bool>(parameters, ns + "use_back", p->astar_parameters.use_back);
    update_param<bool>(
      parameters, ns + "only_behind_solutions", p->astar_parameters.only_behind_solutions);
    update_param<double>(
      parameters, ns + "distance_heuristic_weight", p->astar_parameters.distance_heuristic_weight);
  }

  {
    const std::string ns = "start_planner.freespace_planner.rrtstar.";

    update_param<bool>(parameters, ns + "enable_update", p->rrt_star_parameters.enable_update);
    update_param<bool>(
      parameters, ns + "use_informed_sampling", p->rrt_star_parameters.use_informed_sampling);
    update_param<double>(
      parameters, ns + "max_planning_time", p->rrt_star_parameters.max_planning_time);
    update_param<double>(
      parameters, ns + "neighbor_radius", p->rrt_star_parameters.neighbor_radius);
    update_param<double>(parameters, ns + "margin", p->rrt_star_parameters.margin);
  }

  const std::string base_ns = "start_planner.path_safety_check.";
  const std::string ego_path_ns = base_ns + "ego_predicted_path.";

  {
    update_param<double>(
      parameters, ego_path_ns + "min_velocity", p->ego_predicted_path_params.min_velocity);
    update_param<double>(
      parameters, ego_path_ns + "min_acceleration", p->ego_predicted_path_params.acceleration);
    update_param<double>(
      parameters, ego_path_ns + "time_horizon_for_front_object",
      p->ego_predicted_path_params.time_horizon_for_front_object);
    update_param<double>(
      parameters, ego_path_ns + "time_horizon_for_rear_object",
      p->ego_predicted_path_params.time_horizon_for_rear_object);
    update_param<double>(
      parameters, ego_path_ns + "time_resolution", p->ego_predicted_path_params.time_resolution);
    update_param<double>(
      parameters, ego_path_ns + "delay_until_departure",
      p->ego_predicted_path_params.delay_until_departure);
  }

  const std::string obj_filter_ns = base_ns + "target_filtering.";
  {
    update_param<double>(
      parameters, obj_filter_ns + "safety_check_time_horizon",
      p->objects_filtering_params.safety_check_time_horizon);
    update_param<double>(
      parameters, obj_filter_ns + "safety_check_time_resolution",
      p->objects_filtering_params.safety_check_time_resolution);
    update_param<double>(
      parameters, obj_filter_ns + "object_check_forward_distance",
      p->objects_filtering_params.object_check_forward_distance);
    update_param<double>(
      parameters, obj_filter_ns + "object_check_backward_distance",
      p->objects_filtering_params.object_check_backward_distance);
    update_param<double>(
      parameters, obj_filter_ns + "ignore_object_velocity_threshold",
      p->objects_filtering_params.ignore_object_velocity_threshold);
    update_param<bool>(
      parameters, obj_filter_ns + "include_opposite_lane",
      p->objects_filtering_params.include_opposite_lane);
    update_param<bool>(
      parameters, obj_filter_ns + "invert_opposite_lane",
      p->objects_filtering_params.invert_opposite_lane);
    update_param<bool>(
      parameters, obj_filter_ns + "check_all_predicted_path",
      p->objects_filtering_params.check_all_predicted_path);
    update_param<bool>(
      parameters, obj_filter_ns + "use_all_predicted_path",
      p->objects_filtering_params.use_all_predicted_path);
    update_param<bool>(
      parameters, obj_filter_ns + "use_predicted_path_outside_lanelet",
      p->objects_filtering_params.use_predicted_path_outside_lanelet);
  }

  {
    const std::string obj_types_ns = obj_filter_ns + "object_types_to_check.";
    update_param<bool>(
      parameters, obj_types_ns + "check_car",
      p->objects_filtering_params.object_types_to_check.check_car);
    update_param<bool>(
      parameters, obj_types_ns + "check_truck",
      p->objects_filtering_params.object_types_to_check.check_truck);
    update_param<bool>(
      parameters, obj_types_ns + "check_bus",
      p->objects_filtering_params.object_types_to_check.check_bus);
    update_param<bool>(
      parameters, obj_types_ns + "check_trailer",
      p->objects_filtering_params.object_types_to_check.check_trailer);
    update_param<bool>(
      parameters, obj_types_ns + "check_unknown",
      p->objects_filtering_params.object_types_to_check.check_unknown);
    update_param<bool>(
      parameters, obj_types_ns + "check_bicycle",
      p->objects_filtering_params.object_types_to_check.check_bicycle);
    update_param<bool>(
      parameters, obj_types_ns + "check_motorcycle",
      p->objects_filtering_params.object_types_to_check.check_motorcycle);
    update_param<bool>(
      parameters, obj_types_ns + "check_pedestrian",
      p->objects_filtering_params.object_types_to_check.check_pedestrian);
  }

  const std::string obj_lane_ns = obj_filter_ns + "object_lane_configuration.";
  {
    update_param<bool>(
      parameters, obj_lane_ns + "check_current_lane",
      p->objects_filtering_params.object_lane_configuration.check_current_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_right_side_lane",
      p->objects_filtering_params.object_lane_configuration.check_right_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_left_side_lane",
      p->objects_filtering_params.object_lane_configuration.check_left_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_shoulder_lane",
      p->objects_filtering_params.object_lane_configuration.check_shoulder_lane);
    update_param<bool>(
      parameters, obj_lane_ns + "check_other_lane",
      p->objects_filtering_params.object_lane_configuration.check_other_lane);
  }

  const std::string safety_check_ns = base_ns + "safety_check_params.";
  {
    update_param<bool>(
      parameters, safety_check_ns + "enable_safety_check",
      p->safety_check_params.enable_safety_check);
    update_param<double>(
      parameters, safety_check_ns + "hysteresis_factor_expand_rate",
      p->safety_check_params.hysteresis_factor_expand_rate);
    update_param<double>(
      parameters, safety_check_ns + "backward_path_length",
      p->safety_check_params.backward_path_length);
    update_param<double>(
      parameters, safety_check_ns + "forward_path_length",
      p->safety_check_params.forward_path_length);
    update_param<bool>(
      parameters, safety_check_ns + "publish_debug_marker",
      p->safety_check_params.publish_debug_marker);
    update_param<double>(
      parameters, safety_check_ns + "collision_check_yaw_diff_threshold",
      p->safety_check_params.collision_check_yaw_diff_threshold);
  }

  {
    const std::string rss_ns = safety_check_ns + "rss_params.";
    update_param<double>(
      parameters, rss_ns + "rear_vehicle_reaction_time",
      p->safety_check_params.rss_params.rear_vehicle_reaction_time);
    update_param<double>(
      parameters, rss_ns + "rear_vehicle_safety_time_margin",
      p->safety_check_params.rss_params.rear_vehicle_safety_time_margin);
    update_param<double>(
      parameters, rss_ns + "lateral_distance_max_threshold",
      p->safety_check_params.rss_params.lateral_distance_max_threshold);
    update_param<double>(
      parameters, rss_ns + "longitudinal_distance_min_threshold",
      p->safety_check_params.rss_params.longitudinal_distance_min_threshold);
    update_param<double>(
      parameters, rss_ns + "longitudinal_velocity_delta_time",
      p->safety_check_params.rss_params.longitudinal_velocity_delta_time);
    update_param<std::string>(
      parameters, rss_ns + "extended_polygon_policy",
      p->safety_check_params.rss_params.extended_polygon_policy);
  }
  {
    const std::string surround_moving_obstacle_check_ns =
      "start_planner.surround_moving_obstacle_check.";
    update_param<double>(
      parameters, surround_moving_obstacle_check_ns + "search_radius", p->search_radius);
    update_param<double>(
      parameters, surround_moving_obstacle_check_ns + "th_moving_obstacle_velocity",
      p->th_moving_obstacle_velocity);

    // ObjectTypesToCheck
    {
      std::string obj_types_ns = surround_moving_obstacle_check_ns + "object_types_to_check.";
      update_param<bool>(
        parameters, obj_types_ns + "check_car",
        p->surround_moving_obstacles_type_to_check.check_car);
      update_param<bool>(
        parameters, obj_types_ns + "check_truck",
        p->surround_moving_obstacles_type_to_check.check_truck);
      update_param<bool>(
        parameters, obj_types_ns + "check_bus",
        p->surround_moving_obstacles_type_to_check.check_bus);
      update_param<bool>(
        parameters, obj_types_ns + "check_trailer",
        p->surround_moving_obstacles_type_to_check.check_trailer);
      update_param<bool>(
        parameters, obj_types_ns + "check_unknown",
        p->surround_moving_obstacles_type_to_check.check_unknown);
      update_param<bool>(
        parameters, obj_types_ns + "check_bicycle",
        p->surround_moving_obstacles_type_to_check.check_bicycle);
      update_param<bool>(
        parameters, obj_types_ns + "check_motorcycle",
        p->surround_moving_obstacles_type_to_check.check_motorcycle);
      update_param<bool>(
        parameters, obj_types_ns + "check_pedestrian",
        p->surround_moving_obstacles_type_to_check.check_pedestrian);
    }
  }

  {
    const std::string debug_ns = "start_planner.debug.";
    update_param<bool>(parameters, debug_ns + "print_debug_info", p->print_debug_info);
  }

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

bool StartPlannerModuleManager::isSimultaneousExecutableAsApprovedModule() const
{
  if (observers_.empty()) {
    return config_.enable_simultaneous_execution_as_approved_module;
  }

  const auto checker = [this](const SceneModuleObserver & observer) {
    if (observer.expired()) {
      return config_.enable_simultaneous_execution_as_approved_module;
    }

    const auto start_planner_ptr = std::dynamic_pointer_cast<StartPlannerModule>(observer.lock());

    // Currently simultaneous execution with other modules is not supported while backward driving
    if (!start_planner_ptr->isDrivingForward()) {
      return false;
    }

    // Other modules are not needed when freespace planning
    if (start_planner_ptr->isFreespacePlanning()) {
      return false;
    }

    return config_.enable_simultaneous_execution_as_approved_module;
  };

  return std::all_of(observers_.begin(), observers_.end(), checker);
}

bool StartPlannerModuleManager::isSimultaneousExecutableAsCandidateModule() const
{
  if (observers_.empty()) {
    return config_.enable_simultaneous_execution_as_candidate_module;
  }

  const auto checker = [this](const SceneModuleObserver & observer) {
    if (observer.expired()) {
      return config_.enable_simultaneous_execution_as_candidate_module;
    }

    const auto start_planner_ptr = std::dynamic_pointer_cast<StartPlannerModule>(observer.lock());

    // Currently simultaneous execution with other modules is not supported while backward driving
    if (start_planner_ptr->isDrivingForward()) {
      return false;
    }

    // Other modules are not needed when freespace planning
    if (start_planner_ptr->isFreespacePlanning()) {
      return false;
    }

    return config_.enable_simultaneous_execution_as_candidate_module;
  };

  return std::all_of(observers_.begin(), observers_.end(), checker);
}
}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::StartPlannerModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
