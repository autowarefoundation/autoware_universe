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

#include "autoware/diffusion_planner/utils/planning_factor_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::diffusion_planner::test
{

using autoware_planning_msgs::msg::TrajectoryPoint;

namespace
{

TrajectoryPoint make_point(
  const double time_from_start_sec, const double x, const float velocity, const float accel)
{
  TrajectoryPoint p;
  p.time_from_start.sec = static_cast<int32_t>(time_from_start_sec);
  p.time_from_start.nanosec =
    static_cast<uint32_t>((time_from_start_sec - static_cast<int32_t>(time_from_start_sec)) * 1e9);
  p.pose.position.x = x;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.0;
  p.longitudinal_velocity_mps = velocity;
  p.acceleration_mps2 = accel;
  return p;
}

}  // namespace

class PlanningFactorUtilsTest : public ::testing::Test
{
protected:
  PlanningFactorDetectionConfig config_;

  void SetUp() override
  {
    config_.stop_velocity_threshold = 0.1;
    config_.stop_keep_duration_threshold = 1.0;
    config_.slowdown_accel_threshold = -0.3;
  }
};

TEST_F(PlanningFactorUtilsTest, EmptyPoints)
{
  std::vector<TrajectoryPoint> points;
  const auto result = detect_planning_factors(points, config_);
  EXPECT_FALSE(result.stop.has_value());
  EXPECT_FALSE(result.slowdown.has_value());
}

TEST_F(PlanningFactorUtilsTest, NoStopNoSlowdown)
{
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < 10; ++i) {
    points.push_back(make_point(i * 0.5, i * 1.0, 5.0f, 0.0f));
  }
  const auto result = detect_planning_factors(points, config_);
  EXPECT_FALSE(result.stop.has_value());
  EXPECT_FALSE(result.slowdown.has_value());
}

TEST_F(PlanningFactorUtilsTest, StopDetected)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_point(0.0, 0.0, 3.0f, 0.0f));
  points.push_back(make_point(0.5, 1.0, 2.0f, -0.2f));
  points.push_back(make_point(1.0, 2.0, 1.0f, -0.2f));
  points.push_back(make_point(1.5, 3.0, 0.05f, -0.1f));  // below threshold
  points.push_back(make_point(2.0, 4.0, 0.0f, 0.0f));
  points.push_back(make_point(2.5, 5.0, 0.0f, 0.0f));
  points.push_back(make_point(3.0, 6.0, 0.0f, 0.0f));

  const auto result = detect_planning_factors(points, config_);
  ASSERT_TRUE(result.stop.has_value());
  EXPECT_DOUBLE_EQ(result.stop->ego_pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.stop->stop_pose.position.x, 3.0);
  EXPECT_FALSE(result.slowdown.has_value());
}

TEST_F(PlanningFactorUtilsTest, StopInvalidatedByVelocityResume)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_point(0.0, 0.0, 3.0f, 0.0f));
  points.push_back(make_point(0.5, 1.0, 0.05f, -0.1f));  // below threshold
  points.push_back(make_point(0.8, 2.0, 0.5f, 0.1f));    // above threshold, within 1.0s
  points.push_back(make_point(1.0, 3.0, 2.0f, 0.1f));

  const auto result = detect_planning_factors(points, config_);
  EXPECT_FALSE(result.stop.has_value());
}

TEST_F(PlanningFactorUtilsTest, SlowdownDetected)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_point(0.0, 0.0, 5.0f, 0.0f));
  points.push_back(make_point(0.5, 1.0, 4.5f, -0.1f));
  points.push_back(make_point(1.0, 2.0, 3.5f, -0.5f));  // below slowdown threshold
  points.push_back(make_point(1.5, 3.0, 2.5f, -0.5f));  // below slowdown threshold
  points.push_back(make_point(2.0, 4.0, 2.0f, -0.1f));  // above threshold -> end
  points.push_back(make_point(2.5, 5.0, 2.0f, 0.0f));

  const auto result = detect_planning_factors(points, config_);
  EXPECT_FALSE(result.stop.has_value());
  ASSERT_TRUE(result.slowdown.has_value());
  EXPECT_DOUBLE_EQ(result.slowdown->ego_pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.slowdown->start_pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(result.slowdown->end_pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(result.slowdown->start_velocity, 3.5);
  EXPECT_DOUBLE_EQ(result.slowdown->end_velocity, 2.0);
}

TEST_F(PlanningFactorUtilsTest, BothStopAndSlowdown)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_point(0.0, 0.0, 5.0f, 0.0f));
  points.push_back(make_point(0.5, 1.0, 3.0f, -0.5f));  // slowdown start
  points.push_back(make_point(1.0, 2.0, 1.0f, -0.5f));
  points.push_back(make_point(1.5, 3.0, 0.5f, -0.1f));    // slowdown end
  points.push_back(make_point(2.0, 4.0, 0.05f, -0.05f));  // stop detected
  points.push_back(make_point(2.5, 5.0, 0.0f, 0.0f));
  points.push_back(make_point(3.0, 6.0, 0.0f, 0.0f));
  points.push_back(make_point(3.5, 7.0, 0.0f, 0.0f));

  const auto result = detect_planning_factors(points, config_);

  ASSERT_TRUE(result.stop.has_value());
  EXPECT_DOUBLE_EQ(result.stop->stop_pose.position.x, 4.0);

  ASSERT_TRUE(result.slowdown.has_value());
  EXPECT_DOUBLE_EQ(result.slowdown->start_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(result.slowdown->end_pose.position.x, 3.0);
}

TEST_F(PlanningFactorUtilsTest, SlowdownWithoutEnd)
{
  // Deceleration starts but never ends
  std::vector<TrajectoryPoint> points;
  points.push_back(make_point(0.0, 0.0, 5.0f, 0.0f));
  points.push_back(make_point(0.5, 1.0, 3.0f, -0.5f));
  points.push_back(make_point(1.0, 2.0, 1.0f, -0.5f));
  points.push_back(make_point(1.5, 3.0, 0.5f, -0.5f));

  const auto result = detect_planning_factors(points, config_);
  ASSERT_TRUE(result.slowdown.has_value());
  EXPECT_DOUBLE_EQ(result.slowdown->start_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(result.slowdown->end_pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(result.slowdown->start_velocity, 3.0);
  EXPECT_DOUBLE_EQ(result.slowdown->end_velocity, 0.5);
}

TEST_F(PlanningFactorUtilsTest, SingleStoppedPoint)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(make_point(0.0, 0.0, 0.0f, 0.0f));

  const auto result = detect_planning_factors(points, config_);
  ASSERT_TRUE(result.stop.has_value());
  EXPECT_DOUBLE_EQ(result.stop->ego_pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(result.stop->stop_pose.position.x, 0.0);
}

}  // namespace autoware::diffusion_planner::test
