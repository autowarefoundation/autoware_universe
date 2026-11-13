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

#include "../src/footprint.hpp"
#include "../src/types.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <gtest/gtest.h>

namespace autoware::motion_velocity_planner::out_of_lane
{

TEST(TestFootprint, BaseFootprintDimensionsAndArea)
{
  PlannerParam p;
  p.front_offset = 3.79;
  p.rear_offset = -1.10;
  p.left_offset = 0.948;
  p.right_offset = -0.948;
  p.extra_front_offset = 0.0;
  p.extra_rear_offset = 0.0;
  p.extra_left_offset = 0.0;
  p.extra_right_offset = 0.0;

  const auto footprint = make_base_footprint(p, false);

  ASSERT_EQ(footprint.outer().size(), 5u);

  // Footprint must not collapse into a zero-width line segment (Issue #13536)
  const double expected_length = p.front_offset - p.rear_offset;
  const double expected_width = p.left_offset - p.right_offset;
  const double expected_area = expected_length * expected_width;

  const double calculated_area = std::abs(boost::geometry::area(footprint));
  EXPECT_NEAR(calculated_area, expected_area, 1e-6);
  EXPECT_GT(calculated_area, 1.0);

  boost::geometry::model::box<autoware_utils::Point2d> box;
  boost::geometry::envelope(footprint, box);
  EXPECT_NEAR(box.min_corner().x(), p.rear_offset, 1e-6);
  EXPECT_NEAR(box.max_corner().x(), p.front_offset, 1e-6);
  EXPECT_NEAR(box.min_corner().y(), p.right_offset, 1e-6);
  EXPECT_NEAR(box.max_corner().y(), p.left_offset, 1e-6);
}

TEST(TestFootprint, BaseFootprintWithExtraOffsets)
{
  PlannerParam p;
  p.front_offset = 3.79;
  p.rear_offset = -1.10;
  p.left_offset = 0.948;
  p.right_offset = -0.948;
  p.extra_front_offset = 0.5;
  p.extra_rear_offset = 0.3;
  p.extra_left_offset = 0.2;
  p.extra_right_offset = 0.1;

  const auto footprint = make_base_footprint(p, false);

  ASSERT_EQ(footprint.outer().size(), 5u);

  const double expected_min_x = p.rear_offset - p.extra_rear_offset;
  const double expected_max_x = p.front_offset + p.extra_front_offset;
  const double expected_min_y = p.right_offset - p.extra_right_offset;
  const double expected_max_y = p.left_offset + p.extra_left_offset;
  const double expected_area =
    (expected_max_x - expected_min_x) * (expected_max_y - expected_min_y);

  const double calculated_area = std::abs(boost::geometry::area(footprint));
  EXPECT_NEAR(calculated_area, expected_area, 1e-6);

  boost::geometry::model::box<autoware_utils::Point2d> box;
  boost::geometry::envelope(footprint, box);
  EXPECT_NEAR(box.min_corner().x(), expected_min_x, 1e-6);
  EXPECT_NEAR(box.max_corner().x(), expected_max_x, 1e-6);
  EXPECT_NEAR(box.min_corner().y(), expected_min_y, 1e-6);
  EXPECT_NEAR(box.max_corner().y(), expected_max_y, 1e-6);
}

TEST(TestFootprint, BaseFootprintIgnoreOffset)
{
  PlannerParam p;
  p.front_offset = 3.79;
  p.rear_offset = -1.10;
  p.left_offset = 0.948;
  p.right_offset = -0.948;
  p.extra_front_offset = 0.5;
  p.extra_rear_offset = 0.3;
  p.extra_left_offset = 0.2;
  p.extra_right_offset = 0.1;

  const auto footprint_ignored = make_base_footprint(p, true);

  boost::geometry::model::box<autoware_utils::Point2d> box;
  boost::geometry::envelope(footprint_ignored, box);
  EXPECT_NEAR(box.min_corner().x(), p.rear_offset, 1e-6);
  EXPECT_NEAR(box.max_corner().x(), p.front_offset, 1e-6);
  EXPECT_NEAR(box.min_corner().y(), p.right_offset, 1e-6);
  EXPECT_NEAR(box.max_corner().y(), p.left_offset, 1e-6);
}

TEST(TestFootprint, CurrentEgoFootprintAndLaneletOverlap)
{
  PlannerParam p;
  p.front_offset = 3.79;
  p.rear_offset = -1.10;
  p.left_offset = 0.948;
  p.right_offset = -0.948;
  p.extra_front_offset = 0.0;
  p.extra_rear_offset = 0.0;
  p.extra_left_offset = 0.0;
  p.extra_right_offset = 0.0;

  EgoData ego_data;
  ego_data.pose.position.x = 10.0;
  ego_data.pose.position.y = 5.0;
  ego_data.pose.orientation.w = 1.0;

  const auto ego_footprint = calculate_current_ego_footprint(ego_data, p, false);

  autoware_utils::Polygon2d bg_ego_footprint;
  boost::geometry::convert(ego_footprint, bg_ego_footprint);

  boost::geometry::model::box<autoware_utils::Point2d> box;
  boost::geometry::envelope(bg_ego_footprint, box);
  EXPECT_NEAR(box.min_corner().x(), 10.0 - 1.10, 1e-6);
  EXPECT_NEAR(box.max_corner().x(), 10.0 + 3.79, 1e-6);
  EXPECT_NEAR(box.min_corner().y(), 5.0 - 0.948, 1e-6);
  EXPECT_NEAR(box.max_corner().y(), 5.0 + 0.948, 1e-6);

  // Verify overlap with an adjacent lanelet polygon on the right side: [8.0, 15.0] x [3.0, 4.5]
  // Since ego right side extends down to 5.0 - 0.948 = 4.052, it penetrates the region [3.0, 4.5]
  autoware_utils::Polygon2d adjacent_lanelet;
  adjacent_lanelet.outer() = {{8.0, 3.0}, {15.0, 3.0}, {15.0, 4.5}, {8.0, 4.5}, {8.0, 3.0}};
  EXPECT_TRUE(boost::geometry::intersects(bg_ego_footprint, adjacent_lanelet));
}

}  // namespace autoware::motion_velocity_planner::out_of_lane
