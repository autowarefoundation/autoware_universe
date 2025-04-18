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

#include "autoware/lane_departure_checker/parameters.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <rclcpp/node.hpp>

#include <string>

namespace autoware::lane_departure_checker
{
using autoware_utils::get_or_declare_parameter;

Param init(rclcpp::Node & node)
{
  Param p;
  p.footprint_margin_scale = get_or_declare_parameter<double>(node, "footprint_margin_scale");
  p.footprint_extra_margin = get_or_declare_parameter<double>(node, "footprint_extra_margin");
  p.resample_interval = get_or_declare_parameter<double>(node, "resample_interval");
  p.max_deceleration = get_or_declare_parameter<double>(node, "max_deceleration");
  p.delay_time = get_or_declare_parameter<double>(node, "delay_time");
  p.min_braking_distance = get_or_declare_parameter<double>(node, "min_braking_distance");
  p.ego_nearest_dist_threshold =
    get_or_declare_parameter<double>(node, "ego_nearest_dist_threshold");
  p.ego_nearest_yaw_threshold = get_or_declare_parameter<double>(node, "ego_nearest_yaw_threshold");
  return p;
}

NodeParam NodeParam::init(rclcpp::Node & node)
{
  NodeParam p;
  p.will_out_of_lane_checker = get_or_declare_parameter<bool>(node, "will_out_of_lane_checker");
  p.out_of_lane_checker = get_or_declare_parameter<bool>(node, "out_of_lane_checker");
  p.boundary_departure_checker = get_or_declare_parameter<bool>(node, "boundary_departure_checker");
  p.update_rate = get_or_declare_parameter<double>(node, "update_rate");
  p.visualize_lanelet = get_or_declare_parameter<bool>(node, "visualize_lanelet");
  p.include_right_lanes = get_or_declare_parameter<bool>(node, "include_right_lanes");
  p.include_left_lanes = get_or_declare_parameter<bool>(node, "include_left_lanes");
  p.include_opposite_lanes = get_or_declare_parameter<bool>(node, "include_opposite_lanes");
  p.include_conflicting_lanes = get_or_declare_parameter<bool>(node, "include_conflicting_lanes");
  p.boundary_types_to_detect =
    get_or_declare_parameter<std::vector<std::string>>(node, "boundary_types_to_detect");
  return p;
}
}  // namespace autoware::lane_departure_checker
