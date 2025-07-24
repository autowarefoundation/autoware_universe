// Copyright 2024 Tier IV, Inc.
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

#include <gtest/gtest.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <rcl/time.h>

#include <cstddef>
#include <memory>

TEST(TestUtils, getStopLine)
{
  using autoware::behavior_velocity_planner::detection_area::get_stop_line_geometry2d;
  lanelet::LineString3d line;
  line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, -1.0));
  line.push_back(lanelet::Point3d(lanelet::InvalId, 0.0, 1.0));
  lanelet::Polygons3d detection_areas;
  lanelet::Polygon3d area;
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, -1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -1.0));
  detection_areas.push_back(area);
  auto detection_area =
    lanelet::autoware::DetectionArea::make(lanelet::InvalId, {}, detection_areas, line);
  {
    const double extend_length = 0.0;
    const auto stop_line = get_stop_line_geometry2d(*detection_area, extend_length);
    ASSERT_EQ(stop_line.size(), 2UL);
    EXPECT_EQ(stop_line[0].x(), line[0].x());
    EXPECT_EQ(stop_line[0].y(), line[0].y());
    EXPECT_EQ(stop_line[1].x(), line[1].x());
    EXPECT_EQ(stop_line[1].y(), line[1].y());
  }
  // extended line
  for (auto extend_length = -2.0; extend_length < 2.0; extend_length += 0.1) {
    const auto stop_line = get_stop_line_geometry2d(*detection_area, extend_length);
    ASSERT_EQ(stop_line.size(), 2UL);
    EXPECT_EQ(stop_line[0].x(), line[0].x());
    EXPECT_EQ(stop_line[0].y(), line[0].y() - extend_length);
    EXPECT_EQ(stop_line[1].x(), line[1].x());
    EXPECT_EQ(stop_line[1].y(), line[1].y() + extend_length);
  }
}

TEST(TestUtils, getObstaclePoints)
{
  using autoware::behavior_velocity_planner::detection_area::get_obstacle_points;
  lanelet::ConstPolygons3d detection_areas;
  lanelet::Polygon3d area;
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, -1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 1.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, 1.0));
  area.push_back(lanelet::Point3d(lanelet::InvalId, 3.0, -1.0));
  detection_areas.push_back(area);
  pcl::PointCloud<pcl::PointXYZ> points;
  // empty points
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    EXPECT_TRUE(obstacle_points.empty());
  }
  // add points outside the detection area
  points.emplace_back(0.0, 0.0, 0.0);
  points.emplace_back(4.0, 4.0, 0.0);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    EXPECT_TRUE(obstacle_points.empty());
  }
  // add point on the edge of the detection area (will not be found)
  points.emplace_back(1.0, 1.0, 0.0);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    EXPECT_TRUE(obstacle_points.empty());
  }
  // add point inside the detection area (will be found)
  points.emplace_back(2.0, 0.0, 0.0);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    ASSERT_EQ(obstacle_points.size(), 1UL);
    EXPECT_EQ(obstacle_points[0].x, points[3].x);
    EXPECT_EQ(obstacle_points[0].y, points[3].y);
  }
  // add a detection area that covers all points
  lanelet::Polygon3d full_area;
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, -10.0, -10.0));
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, -10.0, 10.0));
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, 10.0, 10.0));
  full_area.push_back(lanelet::Point3d(lanelet::InvalId, 10.0, -10.0));
  detection_areas.push_back(full_area);
  {
    const auto obstacle_points = get_obstacle_points(detection_areas, points);
    ASSERT_EQ(obstacle_points.size(), 2UL);  // only the 1st point found for each area are returned
    EXPECT_EQ(obstacle_points[0].x, points[3].x);
    EXPECT_EQ(obstacle_points[0].y, points[3].y);
    EXPECT_EQ(obstacle_points[1].x, points[0].x);
    EXPECT_EQ(obstacle_points[1].y, points[0].y);
  }
}

TEST(TestUtils, canClearStopState)
{
  using autoware::behavior_velocity_planner::detection_area::can_clear_stop_state;
  std::shared_ptr<const rclcpp::Time> last_obstacle_found_time = nullptr;
  // can clear if we never found an obstacle
  for (auto now_s = 0; now_s <= 10; now_s += 1) {
    for (auto now_ns = 0; now_ns <= 1e9; now_ns += 1e8) {
      for (double state_clear_time = 0.0; state_clear_time <= 10.0; state_clear_time += 0.1) {
        const auto can_clear = can_clear_stop_state(
          last_obstacle_found_time, rclcpp::Time(now_s, now_ns, RCL_CLOCK_UNINITIALIZED),
          state_clear_time);
        EXPECT_TRUE(can_clear);
      }
    }
  }
  last_obstacle_found_time = std::make_shared<rclcpp::Time>(1.0, 0.0);
  const auto state_clear_time = 1.0;
  // special case for negative time difference which may occur with simulated time
  EXPECT_TRUE(can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(0, 0), state_clear_time));
  EXPECT_TRUE(
    can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(0, 9 * 1e8), state_clear_time));
  // cannot clear before the time has passed since the obstacle disappeared
  EXPECT_FALSE(
    can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(1, 1), state_clear_time));
  EXPECT_FALSE(
    can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(1, 1e9 - 1), state_clear_time));
  // can clear after the time has passed
  EXPECT_TRUE(can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(2, 1), state_clear_time));
  EXPECT_TRUE(
    can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(100, 0), state_clear_time));
  // negative time
  const auto negative_state_clear_time = -1.0;
  EXPECT_TRUE(
    can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(0, 0), negative_state_clear_time));
  EXPECT_TRUE(
    can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(1, 0), negative_state_clear_time));
  EXPECT_TRUE(
    can_clear_stop_state(last_obstacle_found_time, rclcpp::Time(2, 0), negative_state_clear_time));
}

TEST(TestUtils, hasEnoughBrakingDistance)
{
  using autoware::behavior_velocity_planner::detection_area::has_enough_braking_distance;
  // prepare a stop pose 10m away from the self pose
  geometry_msgs::msg::Pose self_pose;
  self_pose.position.x = 0.0;
  self_pose.position.y = 0.0;
  geometry_msgs::msg::Pose line_pose;
  line_pose.position.x = 10.0;
  line_pose.position.y = 0.0;
  // can always brake at zero velocity
  for (auto pass_judge_line_distance = 0.0; pass_judge_line_distance <= 20.0;
       pass_judge_line_distance += 0.1) {
    double current_velocity = 0.0;
    EXPECT_TRUE(has_enough_braking_distance(
      self_pose, line_pose, pass_judge_line_distance, current_velocity));
  }
  // if velocity is not zero, can brake if the pass judge line distance is lower than 10m
  const double current_velocity = 5.0;
  for (auto pass_judge_line_distance = 0.0; pass_judge_line_distance < 10.0;
       pass_judge_line_distance += 0.1) {
    EXPECT_TRUE(has_enough_braking_distance(
      self_pose, line_pose, pass_judge_line_distance, current_velocity));
  }
  for (auto pass_judge_line_distance = 10.0; pass_judge_line_distance <= 20.0;
       pass_judge_line_distance += 0.1) {
    EXPECT_FALSE(has_enough_braking_distance(
      self_pose, line_pose, pass_judge_line_distance, current_velocity));
  }
}
