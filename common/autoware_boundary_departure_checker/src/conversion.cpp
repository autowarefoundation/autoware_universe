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

#include "autoware/boundary_departure_checker/conversion.hpp"

namespace autoware::boundary_departure_checker::utils
{
lanelet::BasicPolygon2d to_basic_polygon_2d(const LinearRing2d & footprint_hull)
{
  lanelet::BasicPolygon2d basic_polygon;
  basic_polygon.reserve(footprint_hull.size());
  for (const auto & point : footprint_hull) {
    basic_polygon.emplace_back(point.x(), point.y());
  }
  return basic_polygon;
}

LineString2d to_linestring_2d(const Segment2d & segment)
{
  const auto & [fr, bk] = segment;
  return {fr, bk};
}

Point2d to_point_2d(const Eigen::Matrix<double, 3, 1> & ll_pt)
{
  return {ll_pt.x(), ll_pt.y()};
}

Segment2d to_segment_2d(
  const Eigen::Matrix<double, 3, 1> & ll_pt1, const Eigen::Matrix<double, 3, 1> & ll_pt2)
{
  return {to_point_2d(ll_pt1), to_point_2d(ll_pt2)};
}

Point to_geom_pt(const Point2d & point, const double z)
{
  return autoware_utils::to_msg(point.to_3d(z));
}

Polygon2d to_polygon_2d(const lanelet::BasicPolygon2d & poly)
{
  autoware_utils::Polygon2d polygon;
  auto & outer = polygon.outer();

  outer.reserve(poly.size());
  for (const auto & p : poly) {
    outer.emplace_back(p.x(), p.y());
  }
  boost::geometry::correct(polygon);
  return polygon;
}
}  // namespace autoware::boundary_departure_checker::utils
