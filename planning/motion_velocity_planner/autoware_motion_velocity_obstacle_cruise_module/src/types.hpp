// Copyright 2025 TIER IV, Inc.
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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "autoware/motion_velocity_planner_common/planner_data.hpp"
#include "autoware/motion_velocity_planner_common/polygon_utils.hpp"
#include "type_alias.hpp"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct CruiseObstacle
{
  CruiseObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_lon_velocity,
    const double arg_lat_velocity,
    const std::vector<polygon_utils::PointWithStamp> & arg_collision_points,
    bool arg_is_yield_obstacle = false)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_lon_velocity),
    lat_velocity(arg_lat_velocity),
    collision_points(arg_collision_points),
    is_yield_obstacle(arg_is_yield_obstacle)
  {
  }
  std::string uuid{};
  rclcpp::Time stamp{};
  geometry_msgs::msg::Pose pose{};  // interpolated with the current stamp
  double velocity{};                // longitudinal velocity against ego's trajectory
  double lat_velocity{};            // lateral velocity against ego's trajectory

  std::vector<polygon_utils::PointWithStamp> collision_points{};  // time-series collision points
  bool is_yield_obstacle{};
};

struct DebugData
{
  DebugData() = default;
  std::vector<std::shared_ptr<PlannerData::Object>> intentionally_ignored_obstacles{};
  std::vector<CruiseObstacle> obstacles_to_cruise{};
  std::vector<Polygon2d> decimated_traj_polys{};
  MarkerArray cruise_wall_marker{};
};

}  // namespace autoware::motion_velocity_planner

#endif  // TYPES_HPP_
