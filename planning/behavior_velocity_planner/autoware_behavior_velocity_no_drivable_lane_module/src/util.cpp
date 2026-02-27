// Copyright 2023 TIER IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <vector>

namespace autoware::behavior_velocity_planner::no_drivable_lane
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

no_drivable_lane::PolygonIntersection getPathIntersectionWithNoDrivableLanePolygon(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & ego_path,
  const lanelet::BasicPolygon2d & polygon, const size_t max_num)
{
  no_drivable_lane::PolygonIntersection path_no_drivable_lane_polygon_intersection;

  auto intersects = experimental::trajectory::crossed_with_polygon(ego_path, polygon);

  intersects.resize(std::min(intersects.size(), max_num));

  const auto & p_last = ego_path.compute(ego_path.length()).point.pose.position;
  const auto & p_first = ego_path.compute(0).point.pose.position;
  const Point & last_path_point{p_last.x, p_last.y};
  const Point & first_path_point{p_first.x, p_first.y};

  path_no_drivable_lane_polygon_intersection.is_first_path_point_inside_polygon =
    bg::within(first_path_point, polygon);
  auto const & is_last_path_point_inside_polygon = bg::within(last_path_point, polygon);

  if (intersects.empty()) {
    if (
      path_no_drivable_lane_polygon_intersection.is_first_path_point_inside_polygon &&
      is_last_path_point_inside_polygon) {
      path_no_drivable_lane_polygon_intersection.is_path_inside_of_polygon = true;
    } else {
      // do nothing
    }
  } else if (intersects.size() == 1) {
    const auto & s = intersects.at(0);
    if (is_last_path_point_inside_polygon) {
      path_no_drivable_lane_polygon_intersection.first_intersection_s = s;
    } else if (path_no_drivable_lane_polygon_intersection.is_first_path_point_inside_polygon) {
      path_no_drivable_lane_polygon_intersection.second_intersection_s = s;
    } else {
      // do nothing
    }
  } else if (intersects.size() == 2) {
    // classify first and second intersection points
    path_no_drivable_lane_polygon_intersection.first_intersection_s = intersects.at(0);
    path_no_drivable_lane_polygon_intersection.second_intersection_s = intersects.at(1);
  } else {
    // do nothing
  }

  return path_no_drivable_lane_polygon_intersection;
}
}  // namespace autoware::behavior_velocity_planner::no_drivable_lane
