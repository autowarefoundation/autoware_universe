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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_extender_utils.hpp"
#include "test_utils.hpp"

#include <gtest/gtest.h>
#include <tf2/utils.h>

#include <cmath>
#include <vector>

using autoware::trajectory_optimizer::plugin::trajectory_extender_utils::
  add_ego_state_to_trajectory;
using autoware::trajectory_optimizer::plugin::trajectory_extender_utils::
  expand_trajectory_with_ego_history;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using trajectory_optimizer_test_utils::create_odometry;
using trajectory_optimizer_test_utils::create_point;
using trajectory_optimizer_test_utils::create_point_with_yaw;

class ExtenderUtilsTest : public ::testing::Test
{
};

// Tests for add_ego_state_to_trajectory
TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  const auto odom = create_odometry(1.0, 2.0, 0.0, 5.0);

  add_ego_state_to_trajectory(points, odom, 1.0, 1.0, 5.0);

  ASSERT_EQ(points.size(), 1);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(points[0].pose.position.y, 2.0);
  EXPECT_FLOAT_EQ(points[0].longitudinal_velocity_mps, 5.0f);
}

TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_SmallChange_Ignored)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));

  // Very small change (< epsilon = 1e-2)
  const auto odom = create_odometry(0.001, 0.001, 0.0, 1.0);

  add_ego_state_to_trajectory(points, odom, 1.0, 1.0, 5.0);

  // Should not add new point
  EXPECT_EQ(points.size(), 1);
}

TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_LargeChange_ResetTrajectory)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));
  points.push_back(create_point(1.0, 0.0, 1.0f));

  // Large change exceeds thresholds
  const auto odom = create_odometry(10.0, 10.0, M_PI, 5.0);

  add_ego_state_to_trajectory(points, odom, 1.0, 1.0, 5.0);

  // Should reset trajectory to just ego state
  ASSERT_EQ(points.size(), 1);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, 10.0);
  EXPECT_DOUBLE_EQ(points[0].pose.position.y, 10.0);
}

TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_NormalAddition)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));

  // Moderate change - within thresholds
  const auto odom = create_odometry(0.5, 0.0, 0.0, 1.5);

  add_ego_state_to_trajectory(points, odom, 1.0, 1.0, 5.0);

  // Should add ego state
  ASSERT_EQ(points.size(), 2);
  EXPECT_DOUBLE_EQ(points[1].pose.position.x, 0.5);
  EXPECT_FLOAT_EQ(points[1].longitudinal_velocity_mps, 1.5f);
}

TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_ClipsWhenExceedingLength)
{
  std::vector<TrajectoryPoint> points;
  // Create a long history
  for (int i = 0; i < 20; ++i) {
    points.push_back(create_point(static_cast<double>(i) * 0.5, 0.0, 1.0f));
  }

  const auto odom = create_odometry(10.0, 0.0, 0.0, 1.0);
  const double backward_extension = 3.0;  // Only keep 3m of history

  add_ego_state_to_trajectory(points, odom, 1.0, 1.0, backward_extension);

  // Should clip to maintain backward extension limit
  EXPECT_GT(points.size(), 1);

  // Calculate total length
  double total_length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    total_length += std::hypot(
      points[i].pose.position.x - points[i - 1].pose.position.x,
      points[i].pose.position.y - points[i - 1].pose.position.y);
  }
  EXPECT_LE(total_length, backward_extension + 1.0);  // Some tolerance
}

// Tests for expand_trajectory_with_ego_history
TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_EmptyHistory)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));

  std::vector<TrajectoryPoint> empty_history;
  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  expand_trajectory_with_ego_history(points, empty_history, odom);

  // Should remain unchanged
  EXPECT_EQ(points.size(), 2);
}

TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  std::vector<TrajectoryPoint> history;
  history.push_back(create_point(-2.0, 0.0));
  history.push_back(create_point(-1.0, 0.0));

  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  expand_trajectory_with_ego_history(points, history, odom);

  // Should copy all history
  ASSERT_EQ(points.size(), 2);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, -2.0);
  EXPECT_DOUBLE_EQ(points[1].pose.position.x, -1.0);
}

TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_PrependsValidPoints)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));

  std::vector<TrajectoryPoint> history;
  history.push_back(create_point(-2.0, 0.0, 1.0f));
  history.push_back(create_point(-1.0, 0.0, 1.0f));
  history.push_back(create_point(-0.5, 0.0, 1.0f));

  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  const size_t original_size = points.size();
  expand_trajectory_with_ego_history(points, history, odom);

  // Should have added some history points
  EXPECT_GT(points.size(), original_size);

  // First point should be from history (negative x)
  EXPECT_LT(points[0].pose.position.x, 0.0);
}

TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_FiltersDuplicates)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));

  std::vector<TrajectoryPoint> history;
  // Add a point that's very close to existing trajectory point
  history.push_back(create_point(-1.0, 0.0));
  history.push_back(create_point(0.05, 0.0));  // Within 0.1m of first trajectory point

  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  expand_trajectory_with_ego_history(points, history, odom);

  // Should filter out the duplicate-ish point
  // Exact count depends on implementation, but should not add all history points
  EXPECT_LE(points.size(), 4);
}

TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_SetsVelocity)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 10.0f));  // First point has 10 m/s

  std::vector<TrajectoryPoint> history;
  history.push_back(create_point(-2.0, 0.0, 5.0f));  // History has 5 m/s
  history.push_back(create_point(-1.0, 0.0, 6.0f));

  const auto odom = create_odometry(0.0, 0.0, 0.0, 1.0);

  expand_trajectory_with_ego_history(points, history, odom);

  // First point should have velocity from original trajectory's first point
  EXPECT_FLOAT_EQ(points[0].longitudinal_velocity_mps, 10.0f);
}

TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_FiltersPointsAheadOfTrajectory)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));

  std::vector<TrajectoryPoint> history;
  history.push_back(create_point(-1.0, 0.0));  // Behind trajectory
  history.push_back(create_point(0.5, 0.0));   // Ahead of first point (should be filtered)
  history.push_back(create_point(2.0, 0.0));   // Way ahead (should be filtered)

  const auto odom = create_odometry(-0.5, 0.0, 0.0, 1.0);

  expand_trajectory_with_ego_history(points, history, odom);

  // Should not add points ahead of the trajectory
  // Only the -1.0 point might be added depending on arc length calculation
  EXPECT_LT(points.size(), 5);
}

TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_YawThreshold)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point_with_yaw(0.0, 0.0, 0.0, 1.0f));

  // Same position, but large yaw difference
  const auto odom = create_odometry(0.0, 0.0, M_PI, 1.0);
  const double yaw_threshold = M_PI / 4.0;  // 45 degrees

  add_ego_state_to_trajectory(points, odom, 1.0, yaw_threshold, 5.0);

  // Yaw difference (M_PI) exceeds threshold, should reset trajectory
  ASSERT_EQ(points.size(), 1);

  tf2::Quaternion q;
  tf2::fromMsg(points[0].pose.orientation, q);
  EXPECT_NEAR(tf2::getYaw(q), M_PI, 1e-6);
}

TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_DistanceThreshold)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));

  // Large distance, small yaw
  const auto odom = create_odometry(5.0, 0.0, 0.0, 1.0);
  const double dist_threshold = 2.0;

  add_ego_state_to_trajectory(points, odom, dist_threshold, 1.0, 10.0);

  // Distance (5.0) exceeds threshold (2.0), should reset trajectory
  ASSERT_EQ(points.size(), 1);
  EXPECT_DOUBLE_EQ(points[0].pose.position.x, 5.0);
}

TEST_F(ExtenderUtilsTest, AddEgoStateToTrajectory_BackwardExtensionLimit)
{
  std::vector<TrajectoryPoint> points;

  const auto odom1 = create_odometry(0.0, 0.0, 0.0, 1.0);
  add_ego_state_to_trajectory(points, odom1, 2.0, 1.0, 2.0);

  const auto odom2 = create_odometry(1.0, 0.0, 0.0, 1.0);
  add_ego_state_to_trajectory(points, odom2, 2.0, 1.0, 2.0);

  const auto odom3 = create_odometry(2.0, 0.0, 0.0, 1.0);
  add_ego_state_to_trajectory(points, odom3, 2.0, 1.0, 2.0);

  const auto odom4 = create_odometry(3.0, 0.0, 0.0, 1.0);
  add_ego_state_to_trajectory(points, odom4, 2.0, 1.0, 2.0);

  // With backward_extension = 2.0, should clip older points
  // Calculate total length
  double total_length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    total_length += std::hypot(
      points[i].pose.position.x - points[i - 1].pose.position.x,
      points[i].pose.position.y - points[i - 1].pose.position.y);
  }

  EXPECT_LE(total_length, 2.5);  // Should be around 2.0 with some tolerance
}

TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_MaintainsTrajectorySize)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0));
  points.push_back(create_point(1.0, 0.0));

  std::vector<TrajectoryPoint> history;
  history.push_back(create_point(-1.0, 0.0));

  const auto odom = create_odometry(-0.5, 0.0, 0.0, 1.0);

  const size_t original_size = points.size();
  expand_trajectory_with_ego_history(points, history, odom);

  // Should add at least the history point
  EXPECT_GE(points.size(), original_size);
}

TEST_F(ExtenderUtilsTest, ExpandTrajectoryWithEgoHistory_PreservesOriginalTrajectory)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 5.0f));
  points.push_back(create_point(1.0, 0.0, 5.0f));

  const double first_x = points[0].pose.position.x;
  const double second_x = points[1].pose.position.x;

  std::vector<TrajectoryPoint> history;
  history.push_back(create_point(-1.0, 0.0, 1.0f));

  const auto odom = create_odometry(-0.5, 0.0, 0.0, 1.0);

  expand_trajectory_with_ego_history(points, history, odom);

  // Original trajectory points should still be present (may be shifted in index)
  bool found_first = false;
  bool found_second = false;
  for (const auto & pt : points) {
    if (std::abs(pt.pose.position.x - first_x) < 1e-6) found_first = true;
    if (std::abs(pt.pose.position.x - second_x) < 1e-6) found_second = true;
  }
  EXPECT_TRUE(found_first);
  EXPECT_TRUE(found_second);
}
