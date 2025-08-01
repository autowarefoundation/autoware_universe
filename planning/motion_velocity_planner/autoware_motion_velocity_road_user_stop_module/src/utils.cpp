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

#include "utils.hpp"

#include <boost/geometry/algorithms/correct.hpp>

namespace autoware::motion_velocity_planner::road_user_stop::utils
{

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & poly)
{
  autoware_utils_geometry::Polygon2d polygon;
  auto & outer = polygon.outer();

  outer.reserve(poly.size());
  for (const auto & p : poly) {
    outer.emplace_back(p.x(), p.y());
  }
  boost::geometry::correct(polygon);
  return polygon;
}

autoware_utils_geometry::Polygon2d to_polygon_2d(const lanelet::BasicPolygon3d & poly)
{
  autoware_utils_geometry::Polygon2d polygon;
  auto & outer = polygon.outer();

  outer.reserve(poly.size());
  for (const auto & p : poly) {
    outer.emplace_back(p.x(), p.y());  // ignore z-coordinate
  }
  boost::geometry::correct(polygon);
  return polygon;
}

}  // namespace autoware::motion_velocity_planner::road_user_stop::utils
