// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef UTIL_HPP_
#define UTIL_HPP_

#define EIGEN_MPL2_ONLY

#include <autoware/trajectory/path_point_with_lane_id.hpp>

#include <geometry_msgs/msg/point32.hpp>

#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::speed_bump
{

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;

// the status of intersection between path and speed bump
struct PolygonIntersection
{
  bool is_path_inside_of_polygon = false;  // true if path is completely inside the speed bump
                                           // polygon (no intersection point)
  std::optional<double> first_intersection_s = std::nullopt;
  std::optional<double> second_intersection_s = std::nullopt;
};

PolygonIntersection getPathIntersectionWithSpeedBumpPolygon(
  const autoware::experimental::trajectory::Trajectory<PathPointWithLaneId> & ego_path,
  const lanelet::BasicPolygon2d & polygon, const size_t max_num);

bool isNoRelation(const PolygonIntersection & status);

}  // namespace autoware::behavior_velocity_planner::speed_bump

namespace autoware::behavior_velocity_planner
{

using geometry_msgs::msg::Point32;

// returns y (speed) for y=mx+b
float calcSlowDownSpeed(const Point32 & p1, const Point32 & p2, const float speed_bump_height);

}  // namespace autoware::behavior_velocity_planner

#endif  // UTIL_HPP_
