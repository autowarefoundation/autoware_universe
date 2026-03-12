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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.hpp"
#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <vector>

using autoware::trajectory_optimizer::plugin::ContinuousJerkSmoother;
using autoware::trajectory_optimizer::plugin::ContinuousJerkSmootherParams;
using autoware_planning_msgs::msg::TrajectoryPoint;
using trajectory_optimizer_test_utils::create_point;

class ContinuousJerkSmootherTest : public ::testing::Test
{
protected:
  ContinuousJerkSmootherParams get_default_params() const
  {
    ContinuousJerkSmootherParams params;
    // QP optimization weights from config file
    params.jerk_weight = 30.0;
    params.over_v_weight = 3000.0;
    params.over_a_weight = 30.0;
    params.over_j_weight = 10.0;
    params.velocity_tracking_weight = 1.0;
    params.accel_tracking_weight = 300.0;
    // Kinematic limits from common namespace (limit.*)
    params.max_accel = 2.0;
    params.min_decel = -3.0;
    params.max_jerk = 1.5;
    params.min_jerk = -1.5;
    return params;
  }

  std::vector<TrajectoryPoint> create_linear_velocity_trajectory(
    double v_start_mps, double v_end_mps, size_t num_points)
  {
    std::vector<TrajectoryPoint> points;
    points.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
      const double velocity = v_start_mps + t * (v_end_mps - v_start_mps);
      const double x = static_cast<double>(i) * 1.0;  // 1m spacing
      points.push_back(create_point(x, 0.0, static_cast<float>(velocity), 0.0f));
    }
    return points;
  }

  std::vector<TrajectoryPoint> create_constant_velocity_trajectory(
    double velocity_mps, size_t num_points)
  {
    return create_linear_velocity_trajectory(velocity_mps, velocity_mps, num_points);
  }
};

// Test Case 1: Normal driving without velocity constraint - verify velocity tracking works
TEST_F(ContinuousJerkSmootherTest, NormalDrivingNoConstraint)
{
  const size_t num_points = 80;
  const double v_start = 30.0 / 3.6;  // 30 km/h -> 8.333 m/s
  const double v_end = 40.0 / 3.6;    // 40 km/h -> 11.111 m/s

  auto input = create_linear_velocity_trajectory(v_start, v_end, num_points);
  std::vector<TrajectoryPoint> output;

  ContinuousJerkSmoother smoother(get_default_params());
  const bool success = smoother.apply(input, output, {});

  ASSERT_TRUE(success) << "Optimization should succeed";
  ASSERT_EQ(output.size(), input.size());

  // Verify velocity tracking: output should be close to input
  // With velocity_tracking_weight=1.0 and high accel_tracking_weight=300.0,
  // the output should closely track the input velocity profile
  double max_velocity_diff = 0.0;
  for (size_t i = 0; i < output.size(); ++i) {
    const double diff =
      std::abs(output[i].longitudinal_velocity_mps - input[i].longitudinal_velocity_mps);
    max_velocity_diff = std::max(max_velocity_diff, diff);
  }

  // With high accel_tracking_weight, velocities should closely track input
  EXPECT_LT(max_velocity_diff, 2.0) << "Velocity tracking should produce close results";

  // Verify all velocities are positive
  for (size_t i = 0; i < output.size(); ++i) {
    EXPECT_GE(output[i].longitudinal_velocity_mps, 0.0)
      << "Velocity should be non-negative at point " << i;
  }
}

// Test Case 2: Normal driving with max velocity cap (soft constraint)
TEST_F(ContinuousJerkSmootherTest, NormalDrivingWithMaxVelocityCap)
{
  const size_t num_points = 80;
  const double v_start = 30.0 / 3.6;  // 30 km/h -> 8.333 m/s
  const double v_end = 40.0 / 3.6;    // 40 km/h -> 11.111 m/s
  const double v_max = 35.0 / 3.6;    // 35 km/h -> 9.722 m/s

  auto input = create_linear_velocity_trajectory(v_start, v_end, num_points);
  std::vector<TrajectoryPoint> output;

  // Apply max velocity constraint (soft)
  std::vector<double> max_velocity_per_point(num_points, v_max);

  ContinuousJerkSmoother smoother(get_default_params());
  const bool success = smoother.apply(input, output, max_velocity_per_point);

  ASSERT_TRUE(success) << "Optimization should succeed";
  ASSERT_EQ(output.size(), input.size());

  // Soft constraint: velocities should be roughly capped but may slightly exceed
  // due to slack variables in the QP formulation
  size_t violations = 0;
  for (size_t i = 0; i < output.size(); ++i) {
    if (output[i].longitudinal_velocity_mps > v_max * 1.1) {  // 10% tolerance
      ++violations;
    }
  }

  // Most points should respect the constraint (soft)
  EXPECT_LT(violations, output.size() / 2) << "Most velocities should respect the soft constraint";
}

// Test Case 3: Deceleration from 15 km/h to 0 (stop)
TEST_F(ContinuousJerkSmootherTest, DecelerationToStop)
{
  const size_t num_points = 80;
  const double v_start = 15.0 / 3.6;  // 15 km/h -> 4.167 m/s
  const double v_end = 0.0;           // 0 m/s (stop)

  auto input = create_linear_velocity_trajectory(v_start, v_end, num_points);
  std::vector<TrajectoryPoint> output;

  ContinuousJerkSmoother smoother(get_default_params());
  const bool success = smoother.apply(input, output, {});

  ASSERT_TRUE(success) << "Optimization should succeed";
  ASSERT_EQ(output.size(), input.size());

  // Verify output decelerates towards zero at the end
  const double last_quarter_start = output.size() * 0.75;
  double sum_velocity_last_quarter = 0.0;
  for (size_t i = static_cast<size_t>(last_quarter_start); i < output.size(); ++i) {
    sum_velocity_last_quarter += output[i].longitudinal_velocity_mps;
  }
  const double avg_velocity_last_quarter =
    sum_velocity_last_quarter / (output.size() - last_quarter_start);

  // Last quarter should have low velocity (decelerating to stop)
  EXPECT_LT(avg_velocity_last_quarter, v_start * 0.5)
    << "Last quarter should have significantly reduced velocity";

  // Verify velocities are non-negative
  for (size_t i = 0; i < output.size(); ++i) {
    EXPECT_GE(output[i].longitudinal_velocity_mps, 0.0)
      << "Velocity should be non-negative at point " << i;
  }
}

// Test Case 4: Driving with local low speed constraint in middle
TEST_F(ContinuousJerkSmootherTest, LocalLowSpeedConstraint)
{
  const size_t num_points = 80;
  const double v_constant = 25.0 / 3.6;  // 25 km/h -> 6.944 m/s
  const double v_low = 15.0 / 3.6;       // 15 km/h -> 4.167 m/s

  // Test parameters for LocalLowSpeedConstraint test case
  static constexpr size_t kConstraintStartIndex = 20;
  static constexpr size_t kConstraintEndIndex = 60;
  static constexpr size_t kConstraintNumPoints = kConstraintEndIndex - kConstraintStartIndex;

  auto input = create_constant_velocity_trajectory(v_constant, num_points);
  std::vector<TrajectoryPoint> output;

  // Apply low speed constraint in middle section (indices 20-60)
  std::vector<double> max_velocity_per_point(num_points, v_constant);
  for (size_t i = kConstraintStartIndex; i < kConstraintEndIndex; ++i) {
    max_velocity_per_point[i] = v_low;
  }

  ContinuousJerkSmoother smoother(get_default_params());
  const bool success = smoother.apply(input, output, max_velocity_per_point);

  ASSERT_TRUE(success) << "Optimization should succeed";
  ASSERT_EQ(output.size(), input.size());

  // Check that velocities are reduced in the constrained middle section
  double avg_middle_velocity = 0.0;
  for (size_t i = kConstraintStartIndex; i < kConstraintEndIndex; ++i) {
    avg_middle_velocity += output[i].longitudinal_velocity_mps;
  }
  avg_middle_velocity /= kConstraintNumPoints;

  // Soft constraint: middle section should have lower velocity than outside
  // but may not strictly enforce the limit
  EXPECT_LT(avg_middle_velocity, v_constant)
    << "Middle section should have reduced velocity due to constraint";
}
