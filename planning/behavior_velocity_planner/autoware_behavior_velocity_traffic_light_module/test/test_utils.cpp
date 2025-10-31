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

#include <autoware/trajectory/utils/pretty_build.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;

using autoware_utils::create_point;
using autoware_utils::create_quaternion;

std::tuple<
  Trajectory, std::vector<geometry_msgs::msg::Point>, std::vector<geometry_msgs::msg::Point>>
generatePath(const geometry_msgs::msg::Pose & pose, const double bound_y_offset = 0.0)
{
  constexpr double interval_distance = 1.0;

  std::vector<PathPointWithLaneId> path_points;
  for (double s = 0.0; s <= 10.0 * interval_distance; s += interval_distance) {
    PathPointWithLaneId p;
    p.point.pose = pose;
    p.point.pose.position.x += s;
    path_points.push_back(p);
  }

  const std::vector<geometry_msgs::msg::Point> left_bound = {
    geometry_msgs::msg::Point{}.set__x(pose.position.x).set__y(pose.position.y + bound_y_offset),
    geometry_msgs::msg::Point{}
      .set__x(pose.position.x + 10.0)
      .set__y(pose.position.y + bound_y_offset)};
  const std::vector<geometry_msgs::msg::Point> right_bound = {
    geometry_msgs::msg::Point{}.set__x(pose.position.x).set__y(pose.position.y - bound_y_offset),
    geometry_msgs::msg::Point{}
      .set__x(pose.position.x + 10.0)
      .set__y(pose.position.y - bound_y_offset)};

  return {*experimental::trajectory::pretty_build(path_points), left_bound, right_bound};
}

TEST(BehaviorTrafficLightModuleUtilsTest, calcStopPoint)
{
  const auto pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                      .position(create_point(0.0, 0.0, 0.0))
                      .orientation(create_quaternion(0.0, 0.0, 0.0, 1.0));
  constexpr double offset = 1.75;

  {  // Normal case
    lanelet::Points3d basic_line;
    basic_line.emplace_back(lanelet::InvalId, 5.5, -1.0, 0.0);
    basic_line.emplace_back(lanelet::InvalId, 5.5, 1.0, 0.0);

    const auto [path, left_bound, right_bound] = generatePath(pose, 1.0);
    const auto line = lanelet::LineString3d(lanelet::InvalId, basic_line);
    const auto output = calcStopPoint(path, left_bound, right_bound, line, offset);

    EXPECT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output.value(), 3.75);
  }

  {  // Stop line does not intersect path bound
    lanelet::Points3d basic_line;
    basic_line.emplace_back(lanelet::InvalId, 5.5, 0.5, 0.0);
    basic_line.emplace_back(lanelet::InvalId, 4.5, 0.5, 0.0);

    const auto [path, left_bound, right_bound] = generatePath(pose, 1.0);
    const auto line = lanelet::LineString3d(lanelet::InvalId, basic_line);
    const auto output = calcStopPoint(path, left_bound, right_bound, line, offset);

    EXPECT_FALSE(output.has_value());
  }
}

}  // namespace autoware::behavior_velocity_planner
