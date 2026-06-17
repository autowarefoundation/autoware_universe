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
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

/// Map-fixed transition point between forward and reverse maneuver lanelets.
struct CuspPoint
{
  geometry_msgs::msg::Pose pose{};
  /// Index on tagged_lanelet_centerline_path where this cusp was detected.
  size_t tagged_centerline_index{0};
  bool visited{false};
};

/// How prefix (non-tagged) and suffix (non-tagged) lanes attach to the tagged centerline.
enum class ReferencePathAssemblyPhase {
  APPROACHING_TAGGED_AREA,
  INSIDE_TAGGED_CORRIDOR,
  EXITING_TAGGED_AREA,
};

inline const char * referencePathAssemblyPhaseToString(const ReferencePathAssemblyPhase phase)
{
  switch (phase) {
    case ReferencePathAssemblyPhase::APPROACHING_TAGGED_AREA:
      return "APPROACHING_TAGGED_AREA";
    case ReferencePathAssemblyPhase::INSIDE_TAGGED_CORRIDOR:
      return "INSIDE_TAGGED_CORRIDOR";
    case ReferencePathAssemblyPhase::EXITING_TAGGED_AREA:
      return "EXITING_TAGGED_AREA";
    default:
      return "UNKNOWN";
  }
}

/// Route-ordered lanelet groups and the stable centerline through direction_change tagged lanes.
struct DirectionChangeRouteContext
{
  std::vector<int64_t> route_lanelet_ids_ordered;
  std::vector<int64_t> tagged_lanelet_ids_ordered;
  std::vector<int64_t> prefix_lanelet_ids;
  std::vector<int64_t> suffix_lanelet_ids;
  PathWithLaneId tagged_lanelet_centerline_path;
  bool is_valid{false};

  DirectionChangeRouteContext()
  : tagged_lanelet_centerline_path(autoware_internal_planning_msgs::msg::PathWithLaneId())
  {
  }
};

/// Shared across module instances so completion survives SUCCESS / re-instantiation.
struct DirectionChangePersistentState
{
  bool maneuver_completed{false};
  unique_identifier_msgs::msg::UUID completed_route_uuid{};
};

struct DirectionChangeParameters
{
  // Cusp detection parameters
  double cusp_detection_distance_threshold;
  double cusp_detection_angle_threshold_deg;

  // State transition parameters
  double
    cusp_detection_distance_start_approaching;  // [m] Distance to zero terminal velocity at cusp
  double stop_velocity_threshold;  // [m/s] Velocity threshold to determine vehicle has stopped
  double th_stopped_time;  // [s] Duration velocity must stay below stop_velocity_threshold before
                           // direction switch at cusp

  // Goal lateral shift parameters (cubic polynomial blend toward route goal)
  bool enable_goal_lateral_shift{true};
  double max_allowed_yaw_deg{
    20.0};  // [deg] Max heading change rate limit for shift maneuver length

  // General parameters
  bool print_debug_info{false};
  double th_arrived_distance;  // [m] If ego is within this distance of route goal, do not activate
                               // (avoid re-entry after completion)
};

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__DATA_STRUCTS_HPP_
