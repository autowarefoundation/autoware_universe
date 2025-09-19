// Copyright 2022 TIER IV, Inc.
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
#include "autoware_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

using autoware_utils::create_point;
using autoware_utils::create_quaternion;

PathWithLaneId generatePath(
  const geometry_msgs::msg::Pose & pose,
  const std::optional<double> & bound_y_offset = std::nullopt)
{
  constexpr double interval_distance = 1.0;

  PathWithLaneId traj;
  for (double s = 0.0; s <= 10.0 * interval_distance; s += interval_distance) {
    PathPointWithLaneId p;
    p.point.pose = pose;
    p.point.pose.position.x += s;
    traj.points.push_back(p);
  }

  if (bound_y_offset) {
    traj.left_bound = {
      geometry_msgs::msg::Point{}.set__x(pose.position.x).set__y(pose.position.y + *bound_y_offset),
      geometry_msgs::msg::Point{}
        .set__x(pose.position.x + 10.0)
        .set__y(pose.position.y + *bound_y_offset)};
    traj.right_bound = {
      geometry_msgs::msg::Point{}.set__x(pose.position.x).set__y(pose.position.y - *bound_y_offset),
      geometry_msgs::msg::Point{}
        .set__x(pose.position.x + 10.0)
        .set__y(pose.position.y - *bound_y_offset)};
  }

  return traj;
}

TEST(BehaviorTrafficLightModuleUtilsTest, getOffsetPoint)
{
  constexpr double length = 2.0;
  Point2d p1 = {0.0, 0.0};
  Point2d p2 = {1.0, 1.0};

  const auto output = getOffsetPoint(p1, p2, length);

  EXPECT_DOUBLE_EQ(output.x(), 1.41421356237309505);
  EXPECT_DOUBLE_EQ(output.y(), 1.41421356237309505);
}

TEST(BehaviorTrafficLightModuleUtilsTest, findNearestCollisionPoint)
{
  {
    Point2d p = {0.5, 0.5};
    LineString2d line1 = {{-1.0, 0.0}, {1.0, 0.0}};
    LineString2d line2 = {{0.0, -1.0}, {0.0, 1.0}};

    const auto output = findNearestCollisionPoint(line1, line2, p);

    EXPECT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output.value().x(), 0.0);
    EXPECT_DOUBLE_EQ(output.value().y(), 0.0);
  }

  {
    Point2d p = {0.5, 0.5};
    LineString2d line1 = {{-1.0, -1.0}, {-1.0, 1.0}};
    LineString2d line2 = {{1.0, -1.0}, {1.0, 1.0}};

    const auto output = findNearestCollisionPoint(line1, line2, p);

    EXPECT_FALSE(output.has_value());
  }
}

TEST(BehaviorTrafficLightModuleUtilsTest, createTargetPoint)
{
  const auto pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                      .position(create_point(0.0, 0.0, 0.0))
                      .orientation(create_quaternion(0.0, 0.0, 0.0, 1.0));
  const auto path = generatePath(pose);

  {
    constexpr double offset = 1.75;
    LineString2d line = {{5.5, -1.0}, {5.5, 1.0}};
    const auto output = createTargetPoint(PathWithLaneId{}, line, offset);

    EXPECT_FALSE(output.has_value());
  }

  {
    constexpr double offset = 1.75;
    LineString2d line = {{5.5, -1.0}, {5.5, 1.0}};
    const auto output = createTargetPoint(path, line, offset);

    EXPECT_TRUE(output.has_value());
    EXPECT_EQ(output.value().first, size_t(4));
    EXPECT_DOUBLE_EQ(output.value().second.x(), 3.75);
    EXPECT_DOUBLE_EQ(output.value().second.y(), 0.0);
  }

  {
    constexpr double offset = -1.75;
    LineString2d line = {{5.5, -1.0}, {5.5, 1.0}};
    const auto output = createTargetPoint(path, line, offset);

    EXPECT_TRUE(output.has_value());
    EXPECT_EQ(output.value().first, size_t(8));
    EXPECT_DOUBLE_EQ(output.value().second.x(), 7.25);
    EXPECT_DOUBLE_EQ(output.value().second.y(), 0.0);
  }

  {
    constexpr double offset = 1.75;
    LineString2d line = {{5.5, 2.0}, {5.5, 1.0}};
    const auto output = createTargetPoint(path, line, offset);

    EXPECT_FALSE(output.has_value());
  }
}

TEST(BehaviorTrafficLightModuleUtilsTest, calcStopPointAndInsertIndex)
{
  const auto pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                      .position(create_point(0.0, 0.0, 0.0))
                      .orientation(create_quaternion(0.0, 0.0, 0.0, 1.0));
  constexpr double offset = 1.75;

  {  // Path is empty
    lanelet::Points3d basic_line;
    basic_line.emplace_back(lanelet::InvalId, 5.5, -1.0, 0.0);
    basic_line.emplace_back(lanelet::InvalId, 5.5, 1.0, 0.0);

    const auto line = lanelet::LineString3d(lanelet::InvalId, basic_line);
    const auto output = calcStopPointAndInsertIndex(PathWithLaneId{}, line, offset);

    EXPECT_FALSE(output.has_value());
  }

  {  // Normal case
    lanelet::Points3d basic_line;
    basic_line.emplace_back(lanelet::InvalId, 5.5, -1.0, 0.0);
    basic_line.emplace_back(lanelet::InvalId, 5.5, 1.0, 0.0);

    const auto line = lanelet::LineString3d(lanelet::InvalId, basic_line);
    const auto output = calcStopPointAndInsertIndex(generatePath(pose, 1.0), line, offset);

    EXPECT_TRUE(output.has_value());
    EXPECT_EQ(output.value().first, size_t(4));
    EXPECT_DOUBLE_EQ(output.value().second.x(), 3.75);
    EXPECT_DOUBLE_EQ(output.value().second.y(), 0.0);
  }

  {  // Stop line does not intersect path bound
    lanelet::Points3d basic_line;
    basic_line.emplace_back(lanelet::InvalId, 5.5, 0.5, 0.0);
    basic_line.emplace_back(lanelet::InvalId, 4.5, 0.5, 0.0);

    const auto line = lanelet::LineString3d(lanelet::InvalId, basic_line);
    const auto output = calcStopPointAndInsertIndex(generatePath(pose, 1.0), line, offset);

    EXPECT_FALSE(output.has_value());
  }
}

}  // namespace autoware::behavior_velocity_planner
