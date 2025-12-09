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

#ifndef AUTOWARE__BEHAVIOR_PATH_DRY_STEERING_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DRY_STEERING_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::behavior_path_planner
{

struct DrySteeringAvoidanceParameters
{
  static DrySteeringAvoidanceParameters init(rclcpp::Node & node);

  // activation condition
  double th_stopped_velocity{0.01};
  double th_stopped_time{3.0};
  double th_static_object_velocity{0.5};

  // object detection
  double object_search_forward_distance{10.0};

  // goal pose sampling
  std::string avoidance_direction{"right"};
  double longitudinal_search_min{1.0};
  double longitudinal_search_max{15.0};
  double longitudinal_search_interval{1.0};
  double lateral_search_min{0.1};
  double lateral_search_max{5.0};
  double lateral_search_interval{0.5};

  // geometric pull out parameters
  ParallelParkingParameters parallel_parking_parameters{};
  double geometric_collision_check_distance_from_end{0.0};
  bool divide_pull_out_path{true};
  double lane_departure_check_expansion_margin{0.0};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DRY_STEERING_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
