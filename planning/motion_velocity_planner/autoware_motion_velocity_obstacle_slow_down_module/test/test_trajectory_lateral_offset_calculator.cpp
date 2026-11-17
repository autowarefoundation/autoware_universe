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

#include "trajectory_lateral_offset_calculator.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace
{
using autoware::motion_velocity_planner::TrajectoryLateralOffsetCalculator;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;

TrajectoryPoint make_trajectory_point(const double x, const double y)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  return point;
}

Point make_point(const double x, const double y)
{
  Point point;
  point.x = x;
  point.y = y;
  return point;
}

// A trajectory along an arc, one point every 1 m.
std::vector<TrajectoryPoint> make_curved_trajectory()
{
  std::vector<TrajectoryPoint> traj_points;
  const double radius = 30.0;
  for (int i = 0; i < 80; ++i) {
    const double angle = i / radius;
    traj_points.push_back(
      make_trajectory_point(radius * std::sin(angle), radius * (1.0 - std::cos(angle))));
  }
  return traj_points;
}

// The same trajectory, but some points are repeated (e.g. at the start, in the middle and at the
// end), as they can be in the trajectories from the planner.
std::vector<TrajectoryPoint> with_overlapping_points(const std::vector<TrajectoryPoint> & original)
{
  std::vector<TrajectoryPoint> traj_points;
  for (size_t i = 0; i < original.size(); ++i) {
    traj_points.push_back(original[i]);
    if (i == 0 || i == original.size() - 1 || i % 7 == 3) {
      traj_points.push_back(original[i]);
    }
    if (i % 11 == 5) {
      traj_points.push_back(original[i]);
      traj_points.push_back(original[i]);
    }
  }
  return traj_points;
}

// true if the two values are the same, including NaN, bit for bit
bool is_identical(const double a, const double b)
{
  return std::memcmp(&a, &b, sizeof(double)) == 0 || (std::isnan(a) && std::isnan(b));
}

void expect_same_as_calc_lateral_offset(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Point> & points)
{
  const TrajectoryLateralOffsetCalculator calculator(traj_points);
  for (const auto & point : points) {
    const double expected = autoware::motion_utils::calcLateralOffset(traj_points, point);
    const double actual = calculator.calc(point);
    EXPECT_TRUE(is_identical(expected, actual))
      << "at (" << point.x << ", " << point.y << "): expected " << expected << ", actual "
      << actual;
  }
}

std::vector<Point> make_random_points(const int num_points)
{
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> x_dist(-10.0, 90.0);
  std::uniform_real_distribution<double> y_dist(-30.0, 60.0);
  std::vector<Point> points;
  for (int i = 0; i < num_points; ++i) {
    points.push_back(make_point(x_dist(rng), y_dist(rng)));
  }
  return points;
}
}  // namespace

TEST(TrajectoryLateralOffsetCalculator, SameAsCalcLateralOffsetOnTrajectoryWithoutOverlap)
{
  expect_same_as_calc_lateral_offset(make_curved_trajectory(), make_random_points(5000));
}

TEST(TrajectoryLateralOffsetCalculator, SameAsCalcLateralOffsetOnTrajectoryWithOverlappingPoints)
{
  const auto traj_points = with_overlapping_points(make_curved_trajectory());
  ASSERT_GT(traj_points.size(), make_curved_trajectory().size());
  expect_same_as_calc_lateral_offset(traj_points, make_random_points(5000));
}

TEST(TrajectoryLateralOffsetCalculator, SameAsCalcLateralOffsetAtThePointsOfTheTrajectory)
{
  const auto traj_points = with_overlapping_points(make_curved_trajectory());
  std::vector<Point> points;
  for (const auto & traj_point : traj_points) {
    points.push_back(traj_point.pose.position);
  }
  expect_same_as_calc_lateral_offset(traj_points, points);
}

TEST(TrajectoryLateralOffsetCalculator, SameAsCalcLateralOffsetOnTwoPointsTrajectory)
{
  const std::vector<TrajectoryPoint> traj_points{
    make_trajectory_point(0.0, 0.0), make_trajectory_point(10.0, 0.0)};
  expect_same_as_calc_lateral_offset(traj_points, make_random_points(100));
  // positive on the left side of the trajectory, negative on the right side
  const TrajectoryLateralOffsetCalculator calculator(traj_points);
  EXPECT_DOUBLE_EQ(calculator.calc(make_point(5.0, 2.0)), 2.0);
  EXPECT_DOUBLE_EQ(calculator.calc(make_point(5.0, -3.0)), -3.0);
}

TEST(TrajectoryLateralOffsetCalculator, ReturnsNanLikeCalcLateralOffsetWithoutASegment)
{
  const auto point = make_point(1.0, 1.0);

  const std::vector<TrajectoryPoint> empty;
  EXPECT_TRUE(std::isnan(TrajectoryLateralOffsetCalculator(empty).calc(point)));
  EXPECT_TRUE(std::isnan(autoware::motion_utils::calcLateralOffset(empty, point)));

  const std::vector<TrajectoryPoint> one_point{make_trajectory_point(0.0, 0.0)};
  EXPECT_TRUE(std::isnan(TrajectoryLateralOffsetCalculator(one_point).calc(point)));
  EXPECT_TRUE(std::isnan(autoware::motion_utils::calcLateralOffset(one_point, point)));

  // all the points overlap, so there is no segment
  const std::vector<TrajectoryPoint> same_points(5, make_trajectory_point(2.0, 3.0));
  EXPECT_TRUE(std::isnan(TrajectoryLateralOffsetCalculator(same_points).calc(point)));
  EXPECT_TRUE(std::isnan(autoware::motion_utils::calcLateralOffset(same_points, point)));
}
