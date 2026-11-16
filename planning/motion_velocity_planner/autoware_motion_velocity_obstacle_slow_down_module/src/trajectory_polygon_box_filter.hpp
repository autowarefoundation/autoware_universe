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

#ifndef TRAJECTORY_POLYGON_BOX_FILTER_HPP_
#define TRAJECTORY_POLYGON_BOX_FILTER_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <cmath>
#include <vector>

namespace autoware::motion_velocity_planner
{
// Bounding boxes of the trajectory polygons, stored in an R-tree.
// A point outside all the boxes is outside all the polygons, so it can be skipped before the
// lateral offset calculation without changing the result. The boxes are enlarged by `margin` as a
// safety margin against rounding (the points on the boundaries of the polygons are inside the boxes
// even without it).
class TrajectoryPolygonBoxFilter
{
public:
  explicit TrajectoryPolygonBoxFilter(
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons, const double margin = 1e-3)
  : rtree_(make_boxes(polygons, margin))
  {
  }

  // true if the point is outside all the boxes, so it can be skipped. A point with a NaN or
  // infinite coordinate is never skipped, so that it is handled in the same way as before the
  // pre-filtering.
  [[nodiscard]] bool is_outside_all_boxes(const double x, const double y) const
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return false;
    }
    return rtree_.qbegin(
             boost::geometry::index::intersects(autoware_utils_geometry::Point2d(x, y))) ==
           rtree_.qend();
  }

private:
  using Box2d = autoware_utils_geometry::Box2d;

  static std::vector<Box2d> make_boxes(
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons, const double margin)
  {
    namespace bg = boost::geometry;
    std::vector<Box2d> boxes;
    boxes.reserve(polygons.size());
    for (const auto & polygon : polygons) {
      auto box = bg::return_envelope<Box2d>(polygon);
      bg::set<bg::min_corner, 0>(box, bg::get<bg::min_corner, 0>(box) - margin);
      bg::set<bg::min_corner, 1>(box, bg::get<bg::min_corner, 1>(box) - margin);
      bg::set<bg::max_corner, 0>(box, bg::get<bg::max_corner, 0>(box) + margin);
      bg::set<bg::max_corner, 1>(box, bg::get<bg::max_corner, 1>(box) + margin);
      boxes.push_back(box);
    }
    return boxes;
  }

  boost::geometry::index::rtree<Box2d, boost::geometry::index::rstar<16>> rtree_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // TRAJECTORY_POLYGON_BOX_FILTER_HPP_
