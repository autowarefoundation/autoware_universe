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

#include "util.hpp"

#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::speed_bump
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;

PolygonIntersection getPathIntersectionWithSpeedBumpPolygon(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & ego_path,
  const lanelet::BasicPolygon2d & polygon, const size_t max_num)
{
  PolygonIntersection polygon_intersection;

  auto closed_polygon = polygon;
  if (closed_polygon.front() != closed_polygon.back()) {
    closed_polygon.push_back(closed_polygon.front());  // close polygon
  }
  auto intersects = experimental::trajectory::crossed(ego_path, closed_polygon);

  std::partial_sort(
    intersects.begin(), intersects.begin() + std::min(intersects.size(), max_num),
    intersects.end());
  intersects.resize(std::min(intersects.size(), max_num));

  const auto & p_last = ego_path.compute(ego_path.length()).point.pose.position;
  const auto & p_first = ego_path.compute(0).point.pose.position;
  const Point & last_path_point{p_last.x, p_last.y};
  const Point & first_path_point{p_first.x, p_first.y};

  const auto & is_first_path_point_inside_polygon = bg::within(first_path_point, polygon);
  const auto & is_last_path_point_inside_polygon = bg::within(last_path_point, polygon);

  // classify first and second intersection points
  if (intersects.empty()) {
    if (is_first_path_point_inside_polygon && is_last_path_point_inside_polygon) {
      polygon_intersection.is_path_inside_of_polygon = true;
    } else {
      // do nothing
    }
  } else if (intersects.size() == 1) {
    const auto & s = intersects.at(0);
    if (is_last_path_point_inside_polygon) {
      polygon_intersection.first_intersection_s = s;
    } else if (is_first_path_point_inside_polygon) {
      polygon_intersection.second_intersection_s = s;
    } else {
      // do nothing
    }
  } else if (intersects.size() == 2) {
    polygon_intersection.first_intersection_s = intersects.at(0);
    polygon_intersection.second_intersection_s = intersects.at(1);
  } else {
    // do nothing
  }

  return polygon_intersection;
}

bool isNoRelation(const PolygonIntersection & status)
{
  return !status.is_path_inside_of_polygon && !status.first_intersection_s &&
         !status.second_intersection_s;
}

}  // namespace autoware::behavior_velocity_planner::speed_bump

namespace autoware::behavior_velocity_planner
{

float calcSlowDownSpeed(const Point32 & p1, const Point32 & p2, const float speed_bump_height)
{
  const float m = (p1.y - p2.y) / (p1.x - p2.x);
  const float b = p1.y - (m * p1.x);

  // y=mx+b
  auto speed = m * speed_bump_height + b;

  return std::clamp(speed, p2.y, p1.y);
}

}  // namespace autoware::behavior_velocity_planner
