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

#ifndef TRAJECTORY_POLYGON_FILTER_HPP_
#define TRAJECTORY_POLYGON_FILTER_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
// Checks if a point is inside any of the trajectory polygons.
//
// The bounding boxes of the polygons are stored in an R-tree together with the indices of the
// polygons. A point inside a polygon is also inside its bounding box, so only the polygons whose
// bounding boxes contain the point are checked instead of all the polygons. The result is the same
// as checking all the polygons (the distance from the point to some polygon is zero). The boxes are
// enlarged by `margin` as a safety margin against rounding (the points on the boundaries of the
// polygons are inside the boxes even without it).
class TrajectoryPolygonFilter
{
public:
  // `polygons` must outlive this object.
  explicit TrajectoryPolygonFilter(
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons, const double margin = 1e-3)
  : polygons_(polygons), rtree_(make_boxes(polygons, margin))
  {
  }
  // The polygons are held by reference, so temporaries are not accepted.
  TrajectoryPolygonFilter(
    std::vector<autoware_utils_geometry::Polygon2d> && polygons, double margin = 1e-3) = delete;

  // true if the point is inside (or on the boundary of) any of the polygons
  [[nodiscard]] bool is_in_any_polygon(const double x, const double y) const
  {
    namespace bg = boost::geometry;
    const autoware_utils_geometry::Point2d point(x, y);
    if (!std::isfinite(x) || !std::isfinite(y)) {
      // The R-tree cannot find such a point, so check all the polygons to give the same result as
      // before (the distance to a polygon can still be 0 for a point with a NaN coordinate).
      return std::any_of(polygons_.begin(), polygons_.end(), [&point](const auto & polygon) {
        return bg::distance(polygon, point) <= 0.0;
      });
    }
    for (auto it = rtree_.qbegin(bg::index::intersects(point)); it != rtree_.qend(); ++it) {
      if (bg::distance(polygons_[it->second], point) <= 0.0) {
        return true;
      }
    }
    return false;
  }

private:
  using Box2d = autoware_utils_geometry::Box2d;
  using BoxWithIndex = std::pair<Box2d, size_t>;

  static std::vector<BoxWithIndex> make_boxes(
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons, const double margin)
  {
    namespace bg = boost::geometry;
    std::vector<BoxWithIndex> boxes;
    boxes.reserve(polygons.size());
    for (size_t i = 0; i < polygons.size(); ++i) {
      auto box = bg::return_envelope<Box2d>(polygons[i]);
      bg::set<bg::min_corner, 0>(box, bg::get<bg::min_corner, 0>(box) - margin);
      bg::set<bg::min_corner, 1>(box, bg::get<bg::min_corner, 1>(box) - margin);
      bg::set<bg::max_corner, 0>(box, bg::get<bg::max_corner, 0>(box) + margin);
      bg::set<bg::max_corner, 1>(box, bg::get<bg::max_corner, 1>(box) + margin);
      boxes.emplace_back(box, i);
    }
    return boxes;
  }

  const std::vector<autoware_utils_geometry::Polygon2d> & polygons_;
  boost::geometry::index::rtree<BoxWithIndex, boost::geometry::index::rstar<16>> rtree_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // TRAJECTORY_POLYGON_FILTER_HPP_
