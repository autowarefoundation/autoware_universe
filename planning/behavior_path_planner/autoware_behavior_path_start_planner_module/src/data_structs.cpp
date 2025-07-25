// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_start_planner_module/data_structs.hpp"

#include "autoware/behavior_path_start_planner_module/manager.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::behavior_path_planner
{

StartPlannerParameters StartPlannerParameters::init(rclcpp::Node & node)
{
  using autoware_utils::get_or_declare_parameter;
  StartPlannerParameters p;
  {
    const std::string ns = "start_planner.";

    p.th_arrived_distance = get_or_declare_parameter<double>(node, ns + "th_arrived_distance");
    p.th_stopped_velocity = get_or_declare_parameter<double>(node, ns + "th_stopped_velocity");
    p.th_stopped_time = get_or_declare_parameter<double>(node, ns + "th_stopped_time");
    p.prepare_time_before_start =
      get_or_declare_parameter<double>(node, ns + "prepare_time_before_start");
    p.th_distance_to_middle_of_the_road =
      get_or_declare_parameter<double>(node, ns + "th_distance_to_middle_of_the_road");
    p.skip_rear_vehicle_check =
      get_or_declare_parameter<bool>(node, ns + "skip_rear_vehicle_check");
    p.extra_width_margin_for_rear_obstacle =
      get_or_declare_parameter<double>(node, ns + "extra_width_margin_for_rear_obstacle");
    p.collision_check_margins =
      get_or_declare_parameter<std::vector<double>>(node, ns + "collision_check_margins");
    p.collision_check_margin_from_front_object =
      get_or_declare_parameter<double>(node, ns + "collision_check_margin_from_front_object");
    p.th_moving_object_velocity =
      get_or_declare_parameter<double>(node, ns + "th_moving_object_velocity");
    p.center_line_path_interval =
      get_or_declare_parameter<double>(node, ns + "center_line_path_interval");
    // shift pull out
    p.enable_shift_pull_out = get_or_declare_parameter<bool>(node, ns + "enable_shift_pull_out");
    p.check_shift_path_lane_departure =
      get_or_declare_parameter<bool>(node, ns + "check_shift_path_lane_departure");
    p.allow_check_shift_path_lane_departure_override =
      get_or_declare_parameter<bool>(node, ns + "allow_check_shift_path_lane_departure_override");
    p.shift_collision_check_distance_from_end =
      get_or_declare_parameter<double>(node, ns + "shift_collision_check_distance_from_end");
    p.minimum_shift_pull_out_distance =
      get_or_declare_parameter<double>(node, ns + "minimum_shift_pull_out_distance");
    p.lateral_acceleration_sampling_num =
      get_or_declare_parameter<int>(node, ns + "lateral_acceleration_sampling_num");
    p.lateral_jerk = get_or_declare_parameter<double>(node, ns + "lateral_jerk");
    p.maximum_lateral_acc = get_or_declare_parameter<double>(node, ns + "maximum_lateral_acc");
    p.minimum_lateral_acc = get_or_declare_parameter<double>(node, ns + "minimum_lateral_acc");
    p.maximum_curvature = get_or_declare_parameter<double>(node, ns + "maximum_curvature");
    p.end_pose_curvature_threshold =
      get_or_declare_parameter<double>(node, ns + "end_pose_curvature_threshold");
    p.maximum_longitudinal_deviation =
      get_or_declare_parameter<double>(node, ns + "maximum_longitudinal_deviation");
    // geometric pull out
    p.enable_geometric_pull_out =
      get_or_declare_parameter<bool>(node, ns + "enable_geometric_pull_out");
    p.geometric_collision_check_distance_from_end =
      get_or_declare_parameter<double>(node, ns + "geometric_collision_check_distance_from_end");
    p.divide_pull_out_path = get_or_declare_parameter<bool>(node, ns + "divide_pull_out_path");
    p.enable_clothoid_fallback =
      get_or_declare_parameter<bool>(node, ns + "enable_clothoid_fallback");
    p.parallel_parking_parameters.pull_out_velocity =
      get_or_declare_parameter<double>(node, ns + "geometric_pull_out_velocity");
    p.parallel_parking_parameters.pull_out_arc_path_interval =
      get_or_declare_parameter<double>(node, ns + "arc_path_interval");
    p.parallel_parking_parameters.pull_out_lane_departure_margin =
      get_or_declare_parameter<double>(node, ns + "lane_departure_margin");
    p.lane_departure_check_expansion_margin =
      get_or_declare_parameter<double>(node, ns + "lane_departure_check_expansion_margin");
    p.parallel_parking_parameters.geometric_pull_out_max_steer_angle_margin_scale =
      get_or_declare_parameter<double>(
        node, ns + "geometric_pull_out_max_steer_angle_margin_scale");
    p.parallel_parking_parameters.center_line_path_interval =
      p.center_line_path_interval;  // for geometric parallel parking

    // clothoid pull out
    p.clothoid_initial_velocity =
      get_or_declare_parameter<double>(node, ns + "clothoid_initial_velocity");
    p.clothoid_acceleration = get_or_declare_parameter<double>(node, ns + "clothoid_acceleration");
    p.clothoid_max_steer_angles_deg =
      get_or_declare_parameter<std::vector<double>>(node, ns + "clothoid_max_steer_angles_deg");
    p.clothoid_max_steer_angle_rate_deg_per_sec =
      get_or_declare_parameter<double>(node, ns + "clothoid_max_steer_angle_rate_deg_per_sec");
    p.check_clothoid_path_lane_departure =
      get_or_declare_parameter<bool>(node, ns + "check_clothoid_path_lane_departure");

    // search start pose backward
    p.search_priority = get_or_declare_parameter<std::string>(
      node,
      ns + "search_priority");  // "efficient_path" or "short_back_distance"
    p.enable_back = get_or_declare_parameter<bool>(node, ns + "enable_back");
    p.backward_velocity = get_or_declare_parameter<double>(node, ns + "backward_velocity");
    p.max_back_distance = get_or_declare_parameter<double>(node, ns + "max_back_distance");
    p.backward_search_resolution =
      get_or_declare_parameter<double>(node, ns + "backward_search_resolution");
    p.backward_path_update_duration =
      get_or_declare_parameter<double>(node, ns + "backward_path_update_duration");
    p.ignore_distance_from_lane_end =
      get_or_declare_parameter<double>(node, ns + "ignore_distance_from_lane_end");
    // stop condition
    p.maximum_deceleration_for_stop =
      get_or_declare_parameter<double>(node, ns + "stop_condition.maximum_deceleration_for_stop");
    p.maximum_jerk_for_stop =
      get_or_declare_parameter<double>(node, ns + "stop_condition.maximum_jerk_for_stop");
  }
  {
    const std::string ns = "start_planner.object_types_to_check_for_path_generation.";
    p.object_types_to_check_for_path_generation.check_car =
      get_or_declare_parameter<bool>(node, ns + "check_car");
    p.object_types_to_check_for_path_generation.check_truck =
      get_or_declare_parameter<bool>(node, ns + "check_truck");
    p.object_types_to_check_for_path_generation.check_bus =
      get_or_declare_parameter<bool>(node, ns + "check_bus");
    p.object_types_to_check_for_path_generation.check_trailer =
      get_or_declare_parameter<bool>(node, ns + "check_trailer");
    p.object_types_to_check_for_path_generation.check_unknown =
      get_or_declare_parameter<bool>(node, ns + "check_unknown");
    p.object_types_to_check_for_path_generation.check_bicycle =
      get_or_declare_parameter<bool>(node, ns + "check_bicycle");
    p.object_types_to_check_for_path_generation.check_motorcycle =
      get_or_declare_parameter<bool>(node, ns + "check_motorcycle");
    p.object_types_to_check_for_path_generation.check_pedestrian =
      get_or_declare_parameter<bool>(node, ns + "check_pedestrian");
  }
  // freespace planner general params
  {
    const std::string ns = "start_planner.freespace_planner.";
    p.enable_freespace_planner =
      get_or_declare_parameter<bool>(node, ns + "enable_freespace_planner");
    p.freespace_planner_algorithm =
      get_or_declare_parameter<std::string>(node, ns + "freespace_planner_algorithm");
    p.end_pose_search_start_distance =
      get_or_declare_parameter<double>(node, ns + "end_pose_search_start_distance");
    p.end_pose_search_end_distance =
      get_or_declare_parameter<double>(node, ns + "end_pose_search_end_distance");
    p.end_pose_search_interval =
      get_or_declare_parameter<double>(node, ns + "end_pose_search_interval");
    p.freespace_planner_velocity = get_or_declare_parameter<double>(node, ns + "velocity");
    p.vehicle_shape_margin = get_or_declare_parameter<double>(node, ns + "vehicle_shape_margin");
    p.freespace_planner_common_parameters.time_limit =
      get_or_declare_parameter<double>(node, ns + "time_limit");
    p.freespace_planner_common_parameters.max_turning_ratio =
      get_or_declare_parameter<double>(node, ns + "max_turning_ratio");
    p.freespace_planner_common_parameters.turning_steps =
      get_or_declare_parameter<int>(node, ns + "turning_steps");
  }
  //  freespace planner search config
  {
    const std::string ns = "start_planner.freespace_planner.search_configs.";
    p.freespace_planner_common_parameters.theta_size =
      get_or_declare_parameter<int>(node, ns + "theta_size");
    p.freespace_planner_common_parameters.angle_goal_range =
      get_or_declare_parameter<double>(node, ns + "angle_goal_range");
    p.freespace_planner_common_parameters.curve_weight =
      get_or_declare_parameter<double>(node, ns + "curve_weight");
    p.freespace_planner_common_parameters.reverse_weight =
      get_or_declare_parameter<double>(node, ns + "reverse_weight");
    p.freespace_planner_common_parameters.lateral_goal_range =
      get_or_declare_parameter<double>(node, ns + "lateral_goal_range");
    p.freespace_planner_common_parameters.longitudinal_goal_range =
      get_or_declare_parameter<double>(node, ns + "longitudinal_goal_range");
  }
  //  freespace planner costmap configs
  {
    const std::string ns = "start_planner.freespace_planner.costmap_configs.";
    p.freespace_planner_common_parameters.obstacle_threshold =
      get_or_declare_parameter<int>(node, ns + "obstacle_threshold");
  }
  //  freespace planner astar
  {
    const std::string ns = "start_planner.freespace_planner.astar.";
    p.astar_parameters.search_method =
      get_or_declare_parameter<std::string>(node, ns + "search_method");
    p.astar_parameters.only_behind_solutions =
      get_or_declare_parameter<bool>(node, ns + "only_behind_solutions");
    p.astar_parameters.use_back = get_or_declare_parameter<bool>(node, ns + "use_back");
    p.astar_parameters.distance_heuristic_weight =
      get_or_declare_parameter<double>(node, ns + "distance_heuristic_weight");
  }
  //   freespace planner rrtstar
  {
    const std::string ns = "start_planner.freespace_planner.rrtstar.";
    p.rrt_star_parameters.enable_update =
      get_or_declare_parameter<bool>(node, ns + "enable_update");
    p.rrt_star_parameters.use_informed_sampling =
      get_or_declare_parameter<bool>(node, ns + "use_informed_sampling");
    p.rrt_star_parameters.max_planning_time =
      get_or_declare_parameter<double>(node, ns + "max_planning_time");
    p.rrt_star_parameters.neighbor_radius =
      get_or_declare_parameter<double>(node, ns + "neighbor_radius");
    p.rrt_star_parameters.margin = get_or_declare_parameter<double>(node, ns + "margin");
  }

  const std::string base_ns = "start_planner.path_safety_check.";
  // EgoPredictedPath
  {
    const std::string ego_path_ns = base_ns + "ego_predicted_path.";
    p.ego_predicted_path_params.min_velocity =
      get_or_declare_parameter<double>(node, ego_path_ns + "min_velocity");
    p.ego_predicted_path_params.acceleration =
      get_or_declare_parameter<double>(node, ego_path_ns + "min_acceleration");
    p.ego_predicted_path_params.time_horizon_for_front_object =
      get_or_declare_parameter<double>(node, ego_path_ns + "time_horizon_for_front_object");
    p.ego_predicted_path_params.time_horizon_for_rear_object =
      get_or_declare_parameter<double>(node, ego_path_ns + "time_horizon_for_rear_object");
    p.ego_predicted_path_params.time_resolution =
      get_or_declare_parameter<double>(node, ego_path_ns + "time_resolution");
    p.ego_predicted_path_params.delay_until_departure =
      get_or_declare_parameter<double>(node, ego_path_ns + "delay_until_departure");
  }
  // ObjectFilteringParams
  const std::string obj_filter_ns = base_ns + "target_filtering.";
  {
    p.objects_filtering_params.safety_check_time_horizon =
      get_or_declare_parameter<double>(node, obj_filter_ns + "safety_check_time_horizon");
    p.objects_filtering_params.safety_check_time_resolution =
      get_or_declare_parameter<double>(node, obj_filter_ns + "safety_check_time_resolution");
    p.objects_filtering_params.object_check_forward_distance =
      get_or_declare_parameter<double>(node, obj_filter_ns + "object_check_forward_distance");
    p.objects_filtering_params.object_check_backward_distance =
      get_or_declare_parameter<double>(node, obj_filter_ns + "object_check_backward_distance");
    p.objects_filtering_params.ignore_object_velocity_threshold =
      get_or_declare_parameter<double>(node, obj_filter_ns + "ignore_object_velocity_threshold");
    p.objects_filtering_params.include_opposite_lane =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "include_opposite_lane");
    p.objects_filtering_params.invert_opposite_lane =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "invert_opposite_lane");
    p.objects_filtering_params.check_all_predicted_path =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "check_all_predicted_path");
    p.objects_filtering_params.use_all_predicted_path =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "use_all_predicted_path");
    p.objects_filtering_params.use_predicted_path_outside_lanelet =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "use_predicted_path_outside_lanelet");
  }
  // ObjectTypesToCheck
  {
    const std::string obj_types_ns = obj_filter_ns + "object_types_to_check.";
    p.objects_filtering_params.object_types_to_check.check_car =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_car");
    p.objects_filtering_params.object_types_to_check.check_truck =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_truck");
    p.objects_filtering_params.object_types_to_check.check_bus =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_bus");
    p.objects_filtering_params.object_types_to_check.check_trailer =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_trailer");
    p.objects_filtering_params.object_types_to_check.check_unknown =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_unknown");
    p.objects_filtering_params.object_types_to_check.check_bicycle =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_bicycle");
    p.objects_filtering_params.object_types_to_check.check_motorcycle =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_motorcycle");
    p.objects_filtering_params.object_types_to_check.check_pedestrian =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_pedestrian");
  }
  // ObjectLaneConfiguration
  {
    const std::string obj_lane_ns = obj_filter_ns + "object_lane_configuration.";
    p.objects_filtering_params.object_lane_configuration.check_current_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_current_lane");
    p.objects_filtering_params.object_lane_configuration.check_right_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_right_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_left_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_left_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_shoulder_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_shoulder_lane");
    p.objects_filtering_params.object_lane_configuration.check_other_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_other_lane");
  }
  // SafetyCheckParams
  const std::string safety_check_ns = base_ns + "safety_check_params.";
  {
    p.safety_check_params.enable_safety_check =
      get_or_declare_parameter<bool>(node, safety_check_ns + "enable_safety_check");
    p.safety_check_params.hysteresis_factor_expand_rate =
      get_or_declare_parameter<double>(node, safety_check_ns + "hysteresis_factor_expand_rate");
    p.safety_check_params.backward_path_length =
      get_or_declare_parameter<double>(node, safety_check_ns + "backward_path_length");
    p.safety_check_params.forward_path_length =
      get_or_declare_parameter<double>(node, safety_check_ns + "forward_path_length");
    p.safety_check_params.publish_debug_marker =
      get_or_declare_parameter<bool>(node, safety_check_ns + "publish_debug_marker");
    p.safety_check_params.collision_check_yaw_diff_threshold = get_or_declare_parameter<double>(
      node, safety_check_ns + "collision_check_yaw_diff_threshold");
  }
  // RSSparams
  {
    const std::string rss_ns = safety_check_ns + "rss_params.";
    p.safety_check_params.rss_params.rear_vehicle_reaction_time =
      get_or_declare_parameter<double>(node, rss_ns + "rear_vehicle_reaction_time");
    p.safety_check_params.rss_params.rear_vehicle_safety_time_margin =
      get_or_declare_parameter<double>(node, rss_ns + "rear_vehicle_safety_time_margin");
    p.safety_check_params.rss_params.lateral_distance_max_threshold =
      get_or_declare_parameter<double>(node, rss_ns + "lateral_distance_max_threshold");
    p.safety_check_params.rss_params.longitudinal_distance_min_threshold =
      get_or_declare_parameter<double>(node, rss_ns + "longitudinal_distance_min_threshold");
    p.safety_check_params.rss_params.longitudinal_velocity_delta_time =
      get_or_declare_parameter<double>(node, rss_ns + "longitudinal_velocity_delta_time");
    p.safety_check_params.rss_params.extended_polygon_policy =
      get_or_declare_parameter<std::string>(node, rss_ns + "extended_polygon_policy");
  }
  // surround moving obstacle check
  {
    const std::string surround_moving_obstacle_check_ns =
      "start_planner.surround_moving_obstacle_check.";
    p.search_radius =
      get_or_declare_parameter<double>(node, surround_moving_obstacle_check_ns + "search_radius");
    p.th_moving_obstacle_velocity = get_or_declare_parameter<double>(
      node, surround_moving_obstacle_check_ns + "th_moving_obstacle_velocity");
    // ObjectTypesToCheck
    {
      const std::string obj_types_ns = surround_moving_obstacle_check_ns + "object_types_to_check.";
      p.surround_moving_obstacles_type_to_check.check_car =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_car");
      p.surround_moving_obstacles_type_to_check.check_truck =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_truck");
      p.surround_moving_obstacles_type_to_check.check_bus =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_bus");
      p.surround_moving_obstacles_type_to_check.check_trailer =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_trailer");
      p.surround_moving_obstacles_type_to_check.check_unknown =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_unknown");
      p.surround_moving_obstacles_type_to_check.check_bicycle =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_bicycle");
      p.surround_moving_obstacles_type_to_check.check_motorcycle =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_motorcycle");
      p.surround_moving_obstacles_type_to_check.check_pedestrian =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_pedestrian");
    }
  }

  // debug
  {
    const std::string debug_ns = "start_planner.debug.";
    p.print_debug_info = get_or_declare_parameter<bool>(node, debug_ns + "print_debug_info");
  }

  return p;
}
}  // namespace autoware::behavior_path_planner
