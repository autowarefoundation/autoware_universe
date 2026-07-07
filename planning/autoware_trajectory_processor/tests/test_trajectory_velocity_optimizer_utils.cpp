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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.hpp"
#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::trajectory_optimizer::plugin::trajectory_velocity_optimizer_utils::clamp_velocities;
using autoware::trajectory_optimizer::plugin::trajectory_velocity_optimizer_utils::set_max_velocity;
using autoware_planning_msgs::msg::TrajectoryPoint;
using trajectory_optimizer_test_utils::create_sample_trajectory;

class VelocityOptimizerUtilsTest : public ::testing::Test
{
};

// Tests for clamp_velocities
TEST_F(VelocityOptimizerUtilsTest, ClampVelocities)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory();
  clamp_velocities(points, 2.0f, 0.5f);
  for (const auto & point : points) {
    ASSERT_GE(point.longitudinal_velocity_mps, 2.0f);
    ASSERT_GE(point.acceleration_mps2, 0.5f);
  }
}

TEST_F(VelocityOptimizerUtilsTest, ClampVelocities_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  clamp_velocities(points, 2.0f, 0.5f);
  EXPECT_TRUE(points.empty());
}

TEST_F(VelocityOptimizerUtilsTest, ClampVelocities_AlreadyAboveMinimum)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory();
  // Set all to high values
  for (auto & point : points) {
    point.longitudinal_velocity_mps = 10.0f;
    point.acceleration_mps2 = 5.0f;
  }

  clamp_velocities(points, 2.0f, 0.5f);

  // Should remain unchanged
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 10.0f);
    EXPECT_FLOAT_EQ(point.acceleration_mps2, 5.0f);
  }
}

// Tests for set_max_velocity
TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_BasicCapping)
{
  // Create trajectory with varying velocities
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 5.0f + i;  // [5, 6, 7, 8, 9]
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 7.0f;
  set_max_velocity(points, max_velocity);

  // Check all velocities are capped
  for (const auto & point : points) {
    ASSERT_LE(point.longitudinal_velocity_mps, max_velocity);
  }

  // Check specific values
  EXPECT_FLOAT_EQ(points[0].longitudinal_velocity_mps, 5.0f);  // Below max, unchanged
  EXPECT_FLOAT_EQ(points[1].longitudinal_velocity_mps, 6.0f);  // Below max, unchanged
  EXPECT_FLOAT_EQ(points[2].longitudinal_velocity_mps, 7.0f);  // At max, unchanged
  EXPECT_FLOAT_EQ(points[3].longitudinal_velocity_mps, 7.0f);  // Capped from 8
  EXPECT_FLOAT_EQ(points[4].longitudinal_velocity_mps, 7.0f);  // Capped from 9
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_InteriorAccelerationsZero)
{
  // Create trajectory with segment of constant velocity after capping
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 10.0f;  // All exceed max
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  // All interior points should have zero acceleration (constant velocity)
  // Points 0-3 should have zero acceleration (transition to next point at same velocity)
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(points[i].acceleration_mps2, 0.0f)
      << "Interior point " << i << " should have zero acceleration";
  }

  // Last point always has zero acceleration
  EXPECT_FLOAT_EQ(points[4].acceleration_mps2, 0.0f);
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_BoundaryAccelerations)
{
  // Create trajectory: [low, HIGH, HIGH, HIGH, low]
  std::vector<TrajectoryPoint> points;
  std::vector<float> velocities = {3.0f, 10.0f, 10.0f, 10.0f, 4.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;  // Will be used to compute dt
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  // After capping: [3.0, 5.0, 5.0, 5.0, 4.0]
  // Segment is [1, 3]

  // Point 0 (before segment): acceleration should be recalculated for transition INTO segment
  // a[0] = (v[1] - v[0]) / dt[0] = (5.0 - 3.0) / dt[0] > 0
  EXPECT_GT(points[0].acceleration_mps2, 0.0f)
    << "Point before segment should have positive acceleration";

  // Points 1-2 (interior): should have zero acceleration
  EXPECT_FLOAT_EQ(points[1].acceleration_mps2, 0.0f);
  EXPECT_FLOAT_EQ(points[2].acceleration_mps2, 0.0f);

  // Point 3 (end of segment): acceleration for transition OUT of segment
  // a[3] = (v[4] - v[3]) / dt[3] = (4.0 - 5.0) / dt[3] < 0
  EXPECT_LT(points[3].acceleration_mps2, 0.0f)
    << "Last point of segment should have negative acceleration";

  // Last point always zero
  EXPECT_FLOAT_EQ(points[4].acceleration_mps2, 0.0f);
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_SegmentAtStart)
{
  // Create trajectory starting with offending segment: [HIGH, HIGH, low, low]
  std::vector<TrajectoryPoint> points;
  std::vector<float> velocities = {10.0f, 10.0f, 3.0f, 3.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  // After capping: [5.0, 5.0, 3.0, 3.0]
  // Segment is [0, 1]

  // Point 0: interior of segment, should have zero acceleration
  EXPECT_FLOAT_EQ(points[0].acceleration_mps2, 0.0f);

  // Point 1: end of segment, should have negative acceleration (transition to lower velocity)
  EXPECT_LT(points[1].acceleration_mps2, 0.0f);

  // Last point always zero
  EXPECT_FLOAT_EQ(points[3].acceleration_mps2, 0.0f);
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_SegmentAtEnd)
{
  // Create trajectory ending with offending segment: [low, low, HIGH, HIGH]
  std::vector<TrajectoryPoint> points;
  std::vector<float> velocities = {3.0f, 3.0f, 10.0f, 10.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  // After capping: [3.0, 3.0, 5.0, 5.0]
  // Segment is [2, 3]

  // Point 1: before segment, should have positive acceleration
  EXPECT_GT(points[1].acceleration_mps2, 0.0f);

  // Point 2: interior of segment, should have zero acceleration
  EXPECT_FLOAT_EQ(points[2].acceleration_mps2, 0.0f);

  // Point 3: last point of trajectory, must have zero acceleration
  EXPECT_FLOAT_EQ(points[3].acceleration_mps2, 0.0f);
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_SinglePointSegment)
{
  // Create trajectory with single offending point: [low, HIGH, low]
  std::vector<TrajectoryPoint> points;
  std::vector<float> velocities = {3.0f, 10.0f, 3.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  // After capping: [3.0, 5.0, 3.0]
  // Segment is [1, 1]

  EXPECT_FLOAT_EQ(points[1].longitudinal_velocity_mps, 5.0f);

  // Point 0: transition into segment, positive acceleration
  EXPECT_GT(points[0].acceleration_mps2, 0.0f);

  // Point 1: transition out of segment, negative acceleration
  EXPECT_LT(points[1].acceleration_mps2, 0.0f);

  // Last point always zero
  EXPECT_FLOAT_EQ(points[2].acceleration_mps2, 0.0f);
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_EntireTrajectoryOffending)
{
  // All points exceed max velocity
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 10.0f;
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  // All velocities should be capped
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, max_velocity);
  }

  // All accelerations should be zero (constant velocity throughout)
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.acceleration_mps2, 0.0f);
  }
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_NoOffendingSegments)
{
  // All points below max velocity
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = 3.0f;
    point.acceleration_mps2 = 0.5f;
    points.push_back(point);
  }

  const float max_velocity = 10.0f;
  set_max_velocity(points, max_velocity);

  // Velocities should remain unchanged
  for (const auto & point : points) {
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 3.0f);
  }

  // Last point should have zero acceleration
  EXPECT_FLOAT_EQ(points.back().acceleration_mps2, 0.0f);
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_MultipleSegments)
{
  // Create trajectory with multiple offending segments: [HIGH, HIGH, low, HIGH, HIGH, low]
  std::vector<TrajectoryPoint> points;
  std::vector<float> velocities = {10.0f, 10.0f, 3.0f, 10.0f, 10.0f, 3.0f};

  for (size_t i = 0; i < velocities.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 1.0;
    point.pose.position.y = 0.0;
    point.longitudinal_velocity_mps = velocities[i];
    point.acceleration_mps2 = 1.0f;
    points.push_back(point);
  }

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  // After capping: [5.0, 5.0, 3.0, 5.0, 5.0, 3.0]
  // Segments are [0, 1] and [3, 4]

  // First segment [0, 1]
  EXPECT_FLOAT_EQ(points[0].acceleration_mps2, 0.0f);  // Interior
  EXPECT_LT(points[1].acceleration_mps2, 0.0f);        // Transition out

  // Between segments
  EXPECT_GT(points[2].acceleration_mps2, 0.0f);  // Transition into second segment

  // Second segment [3, 4]
  EXPECT_FLOAT_EQ(points[3].acceleration_mps2, 0.0f);  // Interior
  EXPECT_LT(points[4].acceleration_mps2, 0.0f);        // Transition out

  // Last point always zero
  EXPECT_FLOAT_EQ(points[5].acceleration_mps2, 0.0f);
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  set_max_velocity(points, 5.0f);
  EXPECT_TRUE(points.empty());
}

TEST_F(VelocityOptimizerUtilsTest, SetMaxVelocity_SinglePointTrajectory)
{
  std::vector<TrajectoryPoint> points;
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 10.0f;
  point.acceleration_mps2 = 0.5f;
  points.push_back(point);

  const float max_velocity = 5.0f;
  set_max_velocity(points, max_velocity);

  EXPECT_FLOAT_EQ(points[0].longitudinal_velocity_mps, max_velocity);
  EXPECT_FLOAT_EQ(points[0].acceleration_mps2, 0.0f);
}
