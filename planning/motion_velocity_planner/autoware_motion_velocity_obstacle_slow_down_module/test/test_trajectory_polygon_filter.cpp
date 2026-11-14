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

#include "trajectory_polygon_filter.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
using autoware::motion_velocity_planner::TrajectoryPolygonFilter;
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

// Polygons along an arc, so that most of them are rotated, their bounding boxes are larger than
// themselves, and the neighboring polygons overlap.
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

// What the filter replaces: the distance from the point to the nearest of all the polygons is zero.
bool is_in_any_polygon_by_checking_all(const std::vector<Polygon2d> & polygons, const Point2d & p)
{
  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto & polygon : polygons) {
    min_dist = std::min(min_dist, boost::geometry::distance(polygon, p));
  }
  return !(min_dist > 0.0);
}

// The filter holds the polygons by reference, so it must not accept temporaries.
static_assert(
  !std::is_constructible_v<TrajectoryPolygonFilter, std::vector<Polygon2d>>,
  "a temporary vector of polygons must be rejected");
}  // namespace

TEST(TrajectoryPolygonFilter, KeepsPointsInsideAndOnTheBoundary)
{
  const std::vector<Polygon2d> polygons{make_rectangle(1.0, 1.0, 0.0, 2.0, 2.0)};
  const TrajectoryPolygonFilter filter(polygons);
  EXPECT_TRUE(filter.is_in_any_polygon(1.0, 1.0));  // inside
  EXPECT_TRUE(filter.is_in_any_polygon(2.0, 1.0));  // on an edge
  EXPECT_TRUE(filter.is_in_any_polygon(2.0, 2.0));  // on a vertex
}

TEST(TrajectoryPolygonFilter, RejectsPointsOutsideThePolygons)
{
  const std::vector<Polygon2d> polygons{make_rectangle(1.0, 1.0, 0.0, 2.0, 2.0)};
  const TrajectoryPolygonFilter filter(polygons);
  EXPECT_FALSE(filter.is_in_any_polygon(2.0005, 1.0));  // inside the 1 mm margin of the box
  EXPECT_FALSE(filter.is_in_any_polygon(2.01, 1.0));
  EXPECT_FALSE(filter.is_in_any_polygon(-5.0, -5.0));
}

TEST(TrajectoryPolygonFilter, RejectsPointsInTheBoundingBoxButOutsideOfTheRotatedPolygon)
{
  // The polygon is a square rotated by 45 degrees. The corner of its bounding box is outside of it.
  const std::vector<Polygon2d> polygons{make_rectangle(0.0, 0.0, M_PI / 4, 2.0, 2.0)};
  const TrajectoryPolygonFilter filter(polygons);
  EXPECT_TRUE(filter.is_in_any_polygon(0.0, 0.0));
  EXPECT_TRUE(filter.is_in_any_polygon(0.9, 0.4));
  EXPECT_FALSE(filter.is_in_any_polygon(1.3, 1.3));
  EXPECT_FALSE(filter.is_in_any_polygon(-1.3, 1.3));
}

TEST(TrajectoryPolygonFilter, RejectsAllPointsWithoutPolygons)
{
  const std::vector<Polygon2d> polygons;
  const TrajectoryPolygonFilter filter(polygons);
  EXPECT_FALSE(filter.is_in_any_polygon(0.0, 0.0));
}

TEST(TrajectoryPolygonFilter, KeepsEveryVertexOfRotatedPolygons)
{
  const auto polygons = make_curved_trajectory_polygons();
  const TrajectoryPolygonFilter filter(polygons);
  for (const auto & polygon : polygons) {
    for (const auto & p : polygon.outer()) {
      EXPECT_TRUE(filter.is_in_any_polygon(p.x(), p.y()));
    }
  }
}

// The filter must give the same result as checking all the polygons.
TEST(TrajectoryPolygonFilter, SameAsCheckingAllThePolygons)
{
  const auto polygons = make_curved_trajectory_polygons();
  const TrajectoryPolygonFilter filter(polygons);

  std::vector<Point2d> points;
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> x_dist(-10.0, 90.0);
  std::uniform_real_distribution<double> y_dist(-10.0, 60.0);
  for (int i = 0; i < 20000; ++i) {
    points.emplace_back(x_dist(rng), y_dist(rng));
  }
  // points with NaN or infinite coordinates (checked against all the polygons as before)
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double v : {0.0, 10.0, 30.0, 45.5, 1000.0}) {
    points.emplace_back(v, nan);
    points.emplace_back(nan, v);
    points.emplace_back(v, inf);
    points.emplace_back(-inf, v);
  }
  points.emplace_back(nan, nan);
  // the vertices and the middle points of the edges are on the boundaries of the polygons
  for (const auto & polygon : polygons) {
    const auto & ring = polygon.outer();
    for (size_t i = 0; i + 1 < ring.size(); ++i) {
      points.emplace_back(ring[i]);
      points.emplace_back((ring[i].x() + ring[i + 1].x()) / 2, (ring[i].y() + ring[i + 1].y()) / 2);
    }
  }

  int num_inside = 0;
  int num_outside = 0;
  for (const auto & point : points) {
    const bool expected = is_in_any_polygon_by_checking_all(polygons, point);
    EXPECT_EQ(filter.is_in_any_polygon(point.x(), point.y()), expected)
      << "at (" << point.x() << ", " << point.y() << ")";
    (expected ? num_inside : num_outside)++;
  }
  // both cases actually occur with these polygons
  EXPECT_GT(num_inside, 0);
  EXPECT_GT(num_outside, 0);
}
