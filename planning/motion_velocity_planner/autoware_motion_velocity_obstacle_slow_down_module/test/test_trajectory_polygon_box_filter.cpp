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

#include "trajectory_polygon_box_filter.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace
{
using autoware::motion_velocity_planner::TrajectoryPolygonBoxFilter;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

// A rectangle centered at (cx, cy) and rotated by yaw, like one polygon of a trajectory.
Polygon2d make_rectangle(
  const double cx, const double cy, const double yaw, const double length, const double width)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Polygon2d polygon;
  for (const auto & [dx, dy] : std::vector<std::pair<double, double>>{
         {length / 2, width / 2},
         {-length / 2, width / 2},
         {-length / 2, -width / 2},
         {length / 2, -width / 2}}) {
    polygon.outer().emplace_back(cx + c * dx - s * dy, cy + s * dx + c * dy);
  }
  boost::geometry::correct(polygon);
  return polygon;
}

// Polygons along an arc, so that most of them are rotated and their bounding boxes are larger.
std::vector<Polygon2d> make_curved_trajectory_polygons()
{
  std::vector<Polygon2d> polygons;
  const double radius = 30.0;
  for (int i = 0; i < 40; ++i) {
    const double angle = i * 2.0 / radius;  // one polygon every 2 m
    polygons.push_back(
      make_rectangle(radius * std::sin(angle), radius * (1.0 - std::cos(angle)), angle, 2.5, 3.0));
  }
  return polygons;
}
}  // namespace

TEST(TrajectoryPolygonBoxFilter, KeepsPointsInsideAndOnTheBoundary)
{
  const TrajectoryPolygonBoxFilter filter({make_rectangle(1.0, 1.0, 0.0, 2.0, 2.0)});
  EXPECT_FALSE(filter.is_outside_all_boxes(1.0, 1.0));     // inside
  EXPECT_FALSE(filter.is_outside_all_boxes(2.0, 1.0));     // on an edge
  EXPECT_FALSE(filter.is_outside_all_boxes(2.0, 2.0));     // on a vertex
  EXPECT_FALSE(filter.is_outside_all_boxes(2.0005, 1.0));  // within the 1 mm margin
}

TEST(TrajectoryPolygonBoxFilter, RejectsPointsOutsideAllTheBoxes)
{
  const TrajectoryPolygonBoxFilter filter({make_rectangle(1.0, 1.0, 0.0, 2.0, 2.0)});
  EXPECT_TRUE(filter.is_outside_all_boxes(2.01, 1.0));
  EXPECT_TRUE(filter.is_outside_all_boxes(-5.0, -5.0));
}

TEST(TrajectoryPolygonBoxFilter, RejectsAllPointsWithoutPolygons)
{
  const TrajectoryPolygonBoxFilter filter({});
  EXPECT_TRUE(filter.is_outside_all_boxes(0.0, 0.0));
}

TEST(TrajectoryPolygonBoxFilter, NeverRejectsPointsWithNonFiniteCoordinates)
{
  // The pre-filter must not change how such points are handled, so it does not reject them.
  const TrajectoryPolygonBoxFilter filter({make_rectangle(1.0, 1.0, 0.0, 2.0, 2.0)});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(filter.is_outside_all_boxes(1.0, nan));
  EXPECT_FALSE(filter.is_outside_all_boxes(5.0, nan));
  EXPECT_FALSE(filter.is_outside_all_boxes(nan, 1.0));
  EXPECT_FALSE(filter.is_outside_all_boxes(nan, nan));
  EXPECT_FALSE(filter.is_outside_all_boxes(inf, 1.0));
  EXPECT_FALSE(filter.is_outside_all_boxes(1.0, -inf));
}

TEST(TrajectoryPolygonBoxFilter, KeepsEveryVertexOfRotatedPolygons)
{
  const auto polygons = make_curved_trajectory_polygons();
  const TrajectoryPolygonBoxFilter filter(polygons);
  for (const auto & polygon : polygons) {
    for (const auto & p : polygon.outer()) {
      EXPECT_FALSE(filter.is_outside_all_boxes(p.x(), p.y()));
    }
  }
}

// The pre-filter must not change the output: a point it rejects must also be rejected by the
// precise filtering, which keeps only the points whose distance to some polygon is zero.
TEST(TrajectoryPolygonBoxFilter, NeverRejectsAPointThatThePreciseFilteringKeeps)
{
  const auto polygons = make_curved_trajectory_polygons();
  const TrajectoryPolygonBoxFilter filter(polygons);
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> x_dist(-10.0, 90.0);
  std::uniform_real_distribution<double> y_dist(-10.0, 60.0);
  int num_rejected = 0;
  int num_kept_in_box_but_outside_polygons = 0;
  for (int i = 0; i < 20000; ++i) {
    const Point2d point(x_dist(rng), y_dist(rng));
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & polygon : polygons) {
      min_dist = std::min(min_dist, boost::geometry::distance(point, polygon));
    }
    if (filter.is_outside_all_boxes(point.x(), point.y())) {
      ++num_rejected;
      EXPECT_GT(min_dist, 0.0) << "rejected a point at (" << point.x() << ", " << point.y() << ")";
    } else if (min_dist > 0.0) {
      ++num_kept_in_box_but_outside_polygons;
    }
  }
  // both cases actually occur with these polygons
  EXPECT_GT(num_rejected, 0);
  EXPECT_GT(num_kept_in_box_but_outside_polygons, 0);
}
