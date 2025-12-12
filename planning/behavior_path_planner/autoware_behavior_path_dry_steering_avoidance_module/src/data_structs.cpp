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

#include "autoware/behavior_path_dry_steering_avoidance_module/data_structs.hpp"

#include <autoware_utils/ros/parameter.hpp>

namespace autoware::behavior_path_planner
{

DrySteeringAvoidanceParameters DrySteeringAvoidanceParameters::init(rclcpp::Node & node)
{
  using autoware_utils::get_or_declare_parameter;
  DrySteeringAvoidanceParameters p;

  const std::string ns = "dry_steering_avoidance.";

  // activation condition
  p.th_stopped_velocity = get_or_declare_parameter<double>(node, ns + "th_stopped_velocity");
  p.th_stopped_time = get_or_declare_parameter<double>(node, ns + "th_stopped_time");
  p.th_static_object_velocity =
    get_or_declare_parameter<double>(node, ns + "th_static_object_velocity");

  // object detection
  p.object_search_forward_distance =
    get_or_declare_parameter<double>(node, ns + "object_search_forward_distance");

  // goal pose sampling
  p.avoidance_direction = get_or_declare_parameter<std::string>(node, ns + "avoidance_direction");
  p.longitudinal_search_min =
    get_or_declare_parameter<double>(node, ns + "longitudinal_search_min");
  p.longitudinal_search_max =
    get_or_declare_parameter<double>(node, ns + "longitudinal_search_max");
  p.longitudinal_search_interval =
    get_or_declare_parameter<double>(node, ns + "longitudinal_search_interval");
  p.lateral_search_min = get_or_declare_parameter<double>(node, ns + "lateral_search_min");
  p.lateral_search_max = get_or_declare_parameter<double>(node, ns + "lateral_search_max");
  p.lateral_search_interval =
    get_or_declare_parameter<double>(node, ns + "lateral_search_interval");

  // geometric pull out parameters
  p.parallel_parking_parameters.pull_out_velocity =
    get_or_declare_parameter<double>(node, ns + "geometric_pull_out_velocity");
  p.parallel_parking_parameters.pull_out_arc_path_interval =
    get_or_declare_parameter<double>(node, ns + "arc_path_interval");
  p.parallel_parking_parameters.pull_out_lane_departure_margin =
    get_or_declare_parameter<double>(node, ns + "lane_departure_margin");
  p.parallel_parking_parameters.center_line_path_interval =
    get_or_declare_parameter<double>(node, ns + "center_line_path_interval");
  p.parallel_parking_parameters.geometric_pull_out_max_steer_angle_margin_scale =
    get_or_declare_parameter<double>(node, ns + "geometric_pull_out_max_steer_angle_margin_scale");
  p.geometric_collision_check_distance_from_end =
    get_or_declare_parameter<double>(node, ns + "geometric_collision_check_distance_from_end");
  p.divide_pull_out_path = get_or_declare_parameter<bool>(node, ns + "divide_pull_out_path");
  p.lane_departure_check_expansion_margin =
    get_or_declare_parameter<double>(node, ns + "lane_departure_check_expansion_margin");

  // debug end pose offset parameters
  p.parallel_parking_parameters.debug_end_pose_longitudinal_offset =
    get_or_declare_parameter<double>(node, ns + "debug_end_pose_longitudinal_offset");
  p.parallel_parking_parameters.debug_end_pose_lateral_offset =
    get_or_declare_parameter<double>(node, ns + "debug_end_pose_lateral_offset");
  p.parallel_parking_parameters.center_line_path_extension =
    get_or_declare_parameter<double>(node, ns + "center_line_path_extension");

  return p;
}

}  // namespace autoware::behavior_path_planner
