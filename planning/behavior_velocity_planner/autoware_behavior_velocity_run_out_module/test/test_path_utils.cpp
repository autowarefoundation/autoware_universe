// Copyright 2024 TIER IV, Inc.
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

#include "path_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

using autoware::behavior_velocity_planner::run_out_utils::findLongitudinalNearestPoint;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;

class TestPathUtils : public ::testing::Test
{
  void SetUp() override {}
};

TEST_F(TestPathUtils, testFindLongitudinalNearestPoint)
{
  const auto path =
    autoware::test_utils::generateTrajectory<PathWithLaneId>(10, 1.0, 1.0, 0.0, 0.0);
  const auto p_src = path.points.front();
  const auto p_dst = path.points.back();
  const auto p_med = path.points.at(path.points.size() / 2);

  const auto geom_p_src = autoware_utils::create_point(
    p_src.point.pose.position.x, p_src.point.pose.position.y, p_src.point.pose.position.z);
  const auto geom_p_dst = autoware_utils::create_point(
    p_dst.point.pose.position.x, p_dst.point.pose.position.y, p_dst.point.pose.position.z);
  const auto geom_p_med = autoware_utils::create_point(
    p_med.point.pose.position.x, p_med.point.pose.position.y, p_med.point.pose.position.z);
  std::vector<Point> dst_points{geom_p_src, geom_p_dst, geom_p_med};
  const auto closest_point_src = findLongitudinalNearestPoint(path.points, geom_p_src, dst_points);
  const auto closest_point_dst = findLongitudinalNearestPoint(path.points, geom_p_dst, dst_points);
  const auto closest_point_med = findLongitudinalNearestPoint(path.points, geom_p_med, dst_points);

  EXPECT_DOUBLE_EQ(
    autoware_utils::calc_distance3d(closest_point_src, geom_p_src),
    autoware_utils::calc_distance3d(geom_p_src, geom_p_src));
  EXPECT_DOUBLE_EQ(
    autoware_utils::calc_distance3d(closest_point_dst, geom_p_dst),
    autoware_utils::calc_distance3d(geom_p_src, geom_p_dst));
  EXPECT_DOUBLE_EQ(
    autoware_utils::calc_distance3d(closest_point_med, geom_p_med),
    autoware_utils::calc_distance3d(geom_p_src, geom_p_med));
}
