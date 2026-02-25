// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__DATA_STRUCTS_HPP_

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

struct DirectionChangeParameters
{
  // Cusp detection parameters
  double cusp_detection_distance_threshold;
  double cusp_detection_angle_threshold_deg;

  // State transition parameters
  double cusp_detection_distance_start_approaching;  // [m] Distance to start approaching cusp
                                                     // (transition to APPROACHING_CUSP)
  double stop_velocity_threshold;  // [m/s] Velocity threshold to determine vehicle has stopped
  double th_stopped_time;  // [s] Duration velocity must stay below stop_velocity_threshold before
                           // direction switch at cusp

  // Direction change parameters
  double reverse_initial_speed;
  double reverse_speed_limit;

  // Path densification parameters
  double reverse_path_densify_max_yaw_step_deg;   // [deg] Maximum yaw angle step for reverse path
                                                  // densification
  double reverse_path_densify_max_distance_step;  // [m] Maximum distance step for reverse path
                                                  // densification

  // General parameters
  bool enable_cusp_detection;
  bool enable_reverse_following;
  bool publish_debug_marker;
  double th_arrived_distance;  // [m] If ego is within this distance of route goal, do not activate
                               // (avoid re-entry after completion)
};

struct DirectionChangeDebugData
{
  std::vector<geometry_msgs::msg::Point> cusp_points{};
  PathWithLaneId forward_path{};
  PathWithLaneId reverse_path{};
};

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__DATA_STRUCTS_HPP_
