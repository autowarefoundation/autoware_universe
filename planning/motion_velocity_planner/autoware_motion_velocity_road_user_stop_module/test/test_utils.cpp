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

#include "../src/utils.hpp"

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

#include <gtest/gtest.h>

namespace autoware::motion_velocity_planner::road_user_stop::utils
{
namespace
{

class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // create a simple square polygon in 2D
    basic_polygon_2d_.emplace_back(0.0, 0.0);
    basic_polygon_2d_.emplace_back(1.0, 0.0);
    basic_polygon_2d_.emplace_back(1.0, 1.0);
    basic_polygon_2d_.emplace_back(0.0, 1.0);

    // create a simple square polygon in 3D
    basic_polygon_3d_.emplace_back(0.0, 0.0, 0.0);
    basic_polygon_3d_.emplace_back(1.0, 0.0, 0.5);
    basic_polygon_3d_.emplace_back(1.0, 1.0, 1.0);
    basic_polygon_3d_.emplace_back(0.0, 1.0, 0.5);

    // create a triangle polygon
    triangle_polygon_2d_.emplace_back(0.0, 0.0);
    triangle_polygon_2d_.emplace_back(2.0, 0.0);
    triangle_polygon_2d_.emplace_back(1.0, 2.0);
  }

  lanelet::BasicPolygon2d basic_polygon_2d_;
  lanelet::BasicPolygon3d basic_polygon_3d_;
  lanelet::BasicPolygon2d triangle_polygon_2d_;
};

TEST_F(UtilsTest, ToPolygon2dFromBasicPolygon2d)
{
  // test conversion from BasicPolygon2d
  const auto result = to_polygon_2d(basic_polygon_2d_);

  // check that the polygon is valid
  EXPECT_TRUE(boost::geometry::is_valid(result));

  // check that the number of points is correct (should be 5 after correction - closed polygon)
  EXPECT_EQ(result.outer().size(), 5U);

  // check that all original points are present (order might be different after
  // boost::geometry::correct)
  bool has_00 = false, has_10 = false, has_11 = false, has_01 = false;
  for (size_t i = 0; i < 4; ++i) {
    const auto & p = result.outer()[i];
    if (p.x() == 0.0 && p.y() == 0.0)
      has_00 = true;
    else if (p.x() == 1.0 && p.y() == 0.0)
      has_10 = true;
    else if (p.x() == 1.0 && p.y() == 1.0)
      has_11 = true;
    else if (p.x() == 0.0 && p.y() == 1.0)
      has_01 = true;
  }
  EXPECT_TRUE(has_00);
  EXPECT_TRUE(has_10);
  EXPECT_TRUE(has_11);
  EXPECT_TRUE(has_01);

  // check that the polygon is closed
  EXPECT_DOUBLE_EQ(result.outer()[4].x(), result.outer()[0].x());
  EXPECT_DOUBLE_EQ(result.outer()[4].y(), result.outer()[0].y());

  // check area (should be 1.0 for a unit square)
  EXPECT_DOUBLE_EQ(boost::geometry::area(result), 1.0);
}

TEST_F(UtilsTest, ToPolygon2dFromBasicPolygon3d)
{
  // test conversion from BasicPolygon3d
  const auto result = to_polygon_2d(basic_polygon_3d_);

  // check that the polygon is valid
  EXPECT_TRUE(boost::geometry::is_valid(result));

  // check that the number of points is correct (should be 5 after correction - closed polygon)
  EXPECT_EQ(result.outer().size(), 5U);

  // check that all original points are present (order might be different after
  // boost::geometry::correct)
  bool has_00 = false, has_10 = false, has_11 = false, has_01 = false;
  for (size_t i = 0; i < 4; ++i) {
    const auto & p = result.outer()[i];
    if (p.x() == 0.0 && p.y() == 0.0)
      has_00 = true;
    else if (p.x() == 1.0 && p.y() == 0.0)
      has_10 = true;
    else if (p.x() == 1.0 && p.y() == 1.0)
      has_11 = true;
    else if (p.x() == 0.0 && p.y() == 1.0)
      has_01 = true;
  }
  EXPECT_TRUE(has_00);
  EXPECT_TRUE(has_10);
  EXPECT_TRUE(has_11);
  EXPECT_TRUE(has_01);

  // check that the polygon is closed
  EXPECT_DOUBLE_EQ(result.outer()[4].x(), result.outer()[0].x());
  EXPECT_DOUBLE_EQ(result.outer()[4].y(), result.outer()[0].y());

  // check area (should be 1.0 for a unit square, z-coordinate doesn't affect 2D area)
  EXPECT_DOUBLE_EQ(boost::geometry::area(result), 1.0);
}

TEST_F(UtilsTest, ToPolygon2dTriangle)
{
  // test conversion of a triangle
  const auto result = to_polygon_2d(triangle_polygon_2d_);

  // check that the polygon is valid
  EXPECT_TRUE(boost::geometry::is_valid(result));

  // check that the number of points is correct (should be 4 after correction - closed polygon)
  EXPECT_EQ(result.outer().size(), 4U);

  // check area (should be 2.0 for this triangle)
  EXPECT_DOUBLE_EQ(boost::geometry::area(result), 2.0);
}

TEST_F(UtilsTest, ToPolygon2dEmptyPolygon)
{
  // test conversion of an empty polygon
  lanelet::BasicPolygon2d empty_polygon;
  const auto result = to_polygon_2d(empty_polygon);

  // check that the result is empty
  EXPECT_TRUE(result.outer().empty());
}

TEST_F(UtilsTest, ToPolygon2dSinglePoint)
{
  // test conversion of a polygon with a single point
  lanelet::BasicPolygon2d single_point_polygon;
  single_point_polygon.emplace_back(1.0, 2.0);

  const auto result = to_polygon_2d(single_point_polygon);

  // single point polygon is degenerate and boost::geometry::correct may handle it differently
  // just verify that the conversion doesn't crash and returns something
  EXPECT_EQ(result.outer().size(), 1U);
}

TEST_F(UtilsTest, ToPolygon2dCounterClockwise)
{
  // test that boost::geometry::correct properly handles counter-clockwise polygons
  lanelet::BasicPolygon2d ccw_polygon;
  ccw_polygon.emplace_back(0.0, 0.0);
  ccw_polygon.emplace_back(0.0, 1.0);
  ccw_polygon.emplace_back(1.0, 1.0);
  ccw_polygon.emplace_back(1.0, 0.0);

  const auto result = to_polygon_2d(ccw_polygon);

  // check that the polygon is valid
  EXPECT_TRUE(boost::geometry::is_valid(result));

  // check area (should be positive 1.0 after correction)
  EXPECT_DOUBLE_EQ(boost::geometry::area(result), 1.0);
}

TEST_F(UtilsTest, ToPolygon2dLargePolygon)
{
  // test conversion of a larger polygon
  lanelet::BasicPolygon2d large_polygon;
  const size_t num_points = 100;
  const double radius = 10.0;

  for (size_t i = 0; i < num_points; ++i) {
    const double angle = 2.0 * M_PI * i / num_points;
    large_polygon.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
  }

  const auto result = to_polygon_2d(large_polygon);

  // check that the polygon is valid
  EXPECT_TRUE(boost::geometry::is_valid(result));

  // check that the number of points is correct (should be num_points + 1 after correction)
  EXPECT_EQ(result.outer().size(), num_points + 1);

  // check approximate area (should be close to pi * radius^2)
  const double expected_area = M_PI * radius * radius;
  EXPECT_NEAR(boost::geometry::area(result), expected_area, expected_area * 0.01);  // 1% tolerance
}

}  // namespace
}  // namespace autoware::motion_velocity_planner::road_user_stop::utils
