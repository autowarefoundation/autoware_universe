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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_mpt_optimizer_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils::
  calculate_acceleration_from_velocity_and_distance;
using autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils::
  calculate_corridor_width;
using autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils::
  calculate_curvature_at_point;
using autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils::
  calculate_time_interval;
using autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils::generate_bounds;
using autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils::
  recalculate_trajectory_dynamics;
using autoware_planning_msgs::msg::TrajectoryPoint;

class MPTOptimizerUtilsTest : public ::testing::Test
{
protected:
  static TrajectoryPoint create_point(double x, double y, float velocity, float acceleration = 0.0f)
  {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = velocity;
    point.acceleration_mps2 = acceleration;
    return point;
  }
};

TEST_F(MPTOptimizerUtilsTest, CalculateAcceleration_Accelerating)
{
  auto p_curr = create_point(0.0, 0.0, 5.0f);
  auto p_next = create_point(10.0, 0.0, 10.0f);

  const double acc = calculate_acceleration_from_velocity_and_distance(p_curr, p_next);

  // a = (v² - v₀²) / (2s) = (100 - 25) / 20 = 3.75 m/s²
  EXPECT_NEAR(acc, 3.75, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, CalculateAcceleration_Decelerating)
{
  auto p_curr = create_point(0.0, 0.0, 10.0f);
  auto p_next = create_point(10.0, 0.0, 5.0f);

  const double acc = calculate_acceleration_from_velocity_and_distance(p_curr, p_next);

  // a = (25 - 100) / 20 = -3.75 m/s²
  EXPECT_NEAR(acc, -3.75, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, CalculateAcceleration_ConstantVelocity)
{
  auto p_curr = create_point(0.0, 0.0, 10.0f);
  auto p_next = create_point(10.0, 0.0, 10.0f);

  const double acc = calculate_acceleration_from_velocity_and_distance(p_curr, p_next);

  EXPECT_NEAR(acc, 0.0, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, CalculateAcceleration_ZeroDistance)
{
  auto p_curr = create_point(0.0, 0.0, 5.0f);
  auto p_next = create_point(0.0, 0.0, 10.0f);

  const double acc = calculate_acceleration_from_velocity_and_distance(p_curr, p_next);

  EXPECT_EQ(acc, 0.0);
}

TEST_F(MPTOptimizerUtilsTest, CalculateTimeInterval_ConstantVelocity)
{
  auto p_curr = create_point(0.0, 0.0, 10.0f);
  auto p_next = create_point(5.0, 0.0, 10.0f);

  const double dt = calculate_time_interval(10.0, 0.0, p_curr, p_next);

  // t = s / v = 5 / 10 = 0.5 s
  EXPECT_NEAR(dt, 0.5, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, CalculateTimeInterval_WithAcceleration)
{
  auto p_curr = create_point(0.0, 0.0, 5.0f);
  auto p_next = create_point(10.0, 0.0, 10.0f);

  const double dt = calculate_time_interval(5.0, 3.75, p_curr, p_next);

  // t = (v - v₀) / a = (10 - 5) / 3.75 = 1.333... s
  EXPECT_NEAR(dt, 5.0 / 3.75, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, CalculateTimeInterval_NearZeroVelocity)
{
  auto p_curr = create_point(0.0, 0.0, 0.0001f);
  auto p_next = create_point(1.0, 0.0, 0.0001f);

  const double dt = calculate_time_interval(0.0001, 0.0, p_curr, p_next);

  EXPECT_EQ(dt, 0.1);
}

TEST_F(MPTOptimizerUtilsTest, CalculateCurvature_StraightLine)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));
  points.push_back(create_point(1.0, 0.0, 1.0f));
  points.push_back(create_point(2.0, 0.0, 1.0f));

  const double curvature = calculate_curvature_at_point(points, 1);

  EXPECT_NEAR(curvature, 0.0, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, CalculateCurvature_LeftTurn)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));
  points.push_back(create_point(1.0, 0.0, 1.0f));
  points.push_back(create_point(1.0, 1.0, 1.0f));

  const double curvature = calculate_curvature_at_point(points, 1);

  EXPECT_GT(curvature, 0.0);
}

TEST_F(MPTOptimizerUtilsTest, CalculateCurvature_EdgeCases)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));
  points.push_back(create_point(1.0, 0.0, 1.0f));

  EXPECT_EQ(calculate_curvature_at_point(points, 0), 0.0);

  points.push_back(create_point(2.0, 0.0, 1.0f));

  EXPECT_EQ(calculate_curvature_at_point(points, 0), 0.0);
  EXPECT_EQ(calculate_curvature_at_point(points, 2), 0.0);
}

TEST_F(MPTOptimizerUtilsTest, CalculateCorridorWidth_BaseOnly)
{
  const double width = calculate_corridor_width(0.0, 10.0, 3.5, 0.0, 0.0);

  EXPECT_EQ(width, 3.5);
}

TEST_F(MPTOptimizerUtilsTest, CalculateCorridorWidth_WithCurvature)
{
  const double curvature = 0.1;
  const double width = calculate_corridor_width(curvature, 10.0, 3.5, 0.5, 0.0);

  // width = 3.5 + 0.5 * 0.1 = 3.55
  EXPECT_NEAR(width, 3.55, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, CalculateCorridorWidth_LowSpeed)
{
  const double width = calculate_corridor_width(0.0, 0.0, 3.5, 0.0, 0.3);

  // At v=0, velocity_factor = (15-0)/15 = 1.0, addition = 0.3
  // width = 3.5 + 0.3 = 3.8
  EXPECT_NEAR(width, 3.8, 1e-6);
}

TEST_F(MPTOptimizerUtilsTest, GenerateBounds_Basic)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(3);
  for (int i = 0; i < 3; ++i) {
    points.push_back(create_point(static_cast<double>(i), 0.0, 1.0f));
  }

  // Use vehicle_width = 1.0, min_clearance = 0.0, corridor = 2.0
  // Expected width = max(2.0, 1.0 + 0.0) = 2.0
  const auto bounds = generate_bounds(points, 2.0, false, 0.0, 0.0, 0.0, 1.0);

  EXPECT_EQ(bounds.left_bound.size(), points.size());
  EXPECT_EQ(bounds.right_bound.size(), points.size());

  // For straight line along x-axis, left bounds should be at y = +2.0, right at y = -2.0
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_NEAR(bounds.left_bound[i].y, 2.0, 1e-4);
    EXPECT_NEAR(bounds.right_bound[i].y, -2.0, 1e-4);
  }
}

TEST_F(MPTOptimizerUtilsTest, GenerateBounds_MinimumClearance)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));

  const double vehicle_width = 3.0;
  const double min_clearance = 0.5;
  const double small_corridor = 1.0;  // Less than required minimum

  const auto bounds =
    generate_bounds(points, small_corridor, false, 0.0, 0.0, min_clearance, vehicle_width);

  // Corridor width is per-side offset from centerline
  // Required: vehicle_width/2 + min_clearance = 1.5 + 0.5 = 2.0
  const double actual_width = std::abs(bounds.left_bound[0].y);
  const double expected_min = (vehicle_width / 2.0) + min_clearance;
  EXPECT_GE(actual_width, expected_min);
  EXPECT_NEAR(actual_width, 2.0, 1e-6);  // Should be clamped to minimum
}

TEST_F(MPTOptimizerUtilsTest, RecalculateTrajectoryDynamics_TimeProgression)
{
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    auto point = create_point(static_cast<double>(i * 10), 0.0, 10.0f);
    point.time_from_start.sec = i;
    point.time_from_start.nanosec = 0;
    points.push_back(point);
  }

  recalculate_trajectory_dynamics(points, 1);

  EXPECT_EQ(points[0].time_from_start.sec, 0);
  EXPECT_EQ(points[0].time_from_start.nanosec, 0u);

  for (size_t i = 1; i < points.size(); ++i) {
    const double t_curr = static_cast<double>(points[i].time_from_start.sec) +
                          static_cast<double>(points[i].time_from_start.nanosec) * 1e-9;
    const double t_prev = static_cast<double>(points[i - 1].time_from_start.sec) +
                          static_cast<double>(points[i - 1].time_from_start.nanosec) * 1e-9;
    EXPECT_GT(t_curr, t_prev);
  }
}

TEST_F(MPTOptimizerUtilsTest, RecalculateTrajectoryDynamics_LastPointZeroAccel)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(5);
  for (int i = 0; i < 5; ++i) {
    points.push_back(create_point(static_cast<double>(i), 0.0, 10.0f, 5.0f));
  }

  recalculate_trajectory_dynamics(points, 1);

  EXPECT_EQ(points.back().acceleration_mps2, 0.0f);
}

TEST_F(MPTOptimizerUtilsTest, RecalculateTrajectoryDynamics_ConstantVelocity)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(5);
  for (int i = 0; i < 5; ++i) {
    points.push_back(create_point(static_cast<double>(i * 10), 0.0, 10.0f));
  }

  recalculate_trajectory_dynamics(points, 1);

  for (auto & point : points) {
    EXPECT_NEAR(point.acceleration_mps2, 0.0f, 1e-4);
  }
}

TEST_F(MPTOptimizerUtilsTest, RecalculateTrajectoryDynamics_MovingAverage)
{
  std::vector<TrajectoryPoint> points;
  std::vector<float> velocities = {5.0f, 6.0f, 8.0f, 11.0f, 15.0f};
  for (size_t i = 0; i < velocities.size(); ++i) {
    points.push_back(create_point(static_cast<double>(i * 10), 0.0, velocities[i]));
  }

  recalculate_trajectory_dynamics(points, 1);
  std::vector<float> original_accel;
  original_accel.reserve(points.size());
  for (const auto & p : points) {
    original_accel.push_back(p.acceleration_mps2);
  }

  points.clear();
  for (size_t i = 0; i < velocities.size(); ++i) {
    points.push_back(create_point(static_cast<double>(i * 10), 0.0, velocities[i]));
  }

  recalculate_trajectory_dynamics(points, 3);

  bool has_smoothing = false;
  for (size_t i = 2; i < points.size() - 1; ++i) {
    if (std::abs(points[i].acceleration_mps2 - original_accel[i]) > 1e-3) {
      has_smoothing = true;
      break;
    }
  }
  EXPECT_TRUE(has_smoothing);
  EXPECT_EQ(points.back().acceleration_mps2, 0.0f);
}

TEST_F(MPTOptimizerUtilsTest, RecalculateTrajectoryDynamics_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;
  recalculate_trajectory_dynamics(points, 5);
  EXPECT_TRUE(points.empty());
}

TEST_F(MPTOptimizerUtilsTest, RecalculateTrajectoryDynamics_SinglePoint)
{
  std::vector<TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 10.0f, 5.0f));

  recalculate_trajectory_dynamics(points, 5);

  EXPECT_EQ(points[0].time_from_start.sec, 0);
  EXPECT_EQ(points[0].time_from_start.nanosec, 0u);
  EXPECT_EQ(points[0].acceleration_mps2, 0.0f);
}
