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

#include "autoware/behavior_velocity_blind_spot_module/parameter.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <string>

namespace autoware::behavior_velocity_planner
{

PlannerParam PlannerParam::init(rclcpp::Node & node, const std::string & ns)
{
  using autoware_utils::get_or_declare_parameter;
  PlannerParam param;
  param.use_pass_judge_line = get_or_declare_parameter<bool>(node, ns + ".use_pass_judge_line");
  param.stop_line_margin = get_or_declare_parameter<double>(node, ns + ".stop_line_margin");
  param.backward_detection_length =
    get_or_declare_parameter<double>(node, ns + ".backward_detection_length");
  param.ignore_width_from_center_line =
    get_or_declare_parameter<double>(node, ns + ".ignore_width_from_center_line");
  param.adjacent_extend_width =
    get_or_declare_parameter<double>(node, ns + ".adjacent_extend_width");
  param.opposite_adjacent_extend_width =
    get_or_declare_parameter<double>(node, ns + ".opposite_adjacent_extend_width");
  param.max_future_movement_time =
    get_or_declare_parameter<double>(node, ns + ".max_future_movement_time");
  param.ttc_min = get_or_declare_parameter<double>(node, ns + ".ttc_min");
  param.ttc_max = get_or_declare_parameter<double>(node, ns + ".ttc_max");
  param.ttc_ego_minimal_velocity =
    get_or_declare_parameter<double>(node, ns + ".ttc_ego_minimal_velocity");
  return param;
}
}  // namespace autoware::behavior_velocity_planner
