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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_spline_smoother_utils.hpp"
#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::trajectory_optimizer::plugin::trajectory_spline_smoother_utils::apply_spline;
using autoware_planning_msgs::msg::TrajectoryPoint;
using trajectory_optimizer_test_utils::create_sample_trajectory;

class SplineSmootherUtilsTest : public ::testing::Test
{
};

TEST_F(SplineSmootherUtilsTest, ApplySpline_BasicInterpolation)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory(1.0, 0.0, 10);
  const size_t original_size = points.size();

  const double interpolation_resolution_m = 0.5;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = false;

  apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);

  // Should have more points after interpolation
  EXPECT_GT(points.size(), original_size);
  EXPECT_GE(points.size(), 2);
}

TEST_F(SplineSmootherUtilsTest, ApplySpline_PreserveOrientation)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory(1.0, 0.0, 10);

  const double interpolation_resolution_m = 0.1;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = true;

  apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);

  ASSERT_GE(points.size(), 2);
  // Check all points have valid orientations
  for (const auto & point : points) {
    EXPECT_TRUE(std::isfinite(point.pose.orientation.w));
    EXPECT_TRUE(std::isfinite(point.pose.orientation.x));
    EXPECT_TRUE(std::isfinite(point.pose.orientation.y));
    EXPECT_TRUE(std::isfinite(point.pose.orientation.z));
  }
}

TEST_F(SplineSmootherUtilsTest, ApplySpline_TooFewPoints)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory(1.0, 0.0, 3);
  const size_t original_size = points.size();

  const double interpolation_resolution_m = 0.1;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = false;

  apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);

  // Should be unchanged (minimum is 5 points for Akima spline)
  EXPECT_EQ(points.size(), original_size);
}

TEST_F(SplineSmootherUtilsTest, ApplySpline_MinimumPoints)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory(1.0, 0.0, 5);

  const double interpolation_resolution_m = 0.5;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = false;

  apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);

  // Should successfully interpolate with exactly 5 points
  EXPECT_GE(points.size(), 2);
}

TEST_F(SplineSmootherUtilsTest, ApplySpline_SmallResolution)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory(2.0, 0.0, 6);

  const double interpolation_resolution_m = 0.05;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = false;

  apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);

  // Small resolution should create many points
  EXPECT_GT(points.size(), 20);
}

TEST_F(SplineSmootherUtilsTest, ApplySpline_LargeResolution)
{
  std::vector<TrajectoryPoint> points = create_sample_trajectory(1.0, 0.0, 10);

  const double interpolation_resolution_m = 5.0;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = false;

  apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);

  // Large resolution should create fewer points
  EXPECT_GE(points.size(), 2);
  EXPECT_LT(points.size(), 10);
}

TEST_F(SplineSmootherUtilsTest, ApplySpline_EmptyTrajectory)
{
  std::vector<TrajectoryPoint> points;

  const double interpolation_resolution_m = 0.1;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = false;

  apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);

  // Should remain empty
  EXPECT_TRUE(points.empty());
}
