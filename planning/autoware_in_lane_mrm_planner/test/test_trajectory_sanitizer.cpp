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

#include "trajectory_sanitizer.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::in_lane_mrm_planner
{
namespace
{

TrajectoryPoints make_straight_trajectory(
  const size_t num_points, const double spacing, const float velocity)
{
  TrajectoryPoints points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = spacing * static_cast<double>(i);
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    point.longitudinal_velocity_mps = velocity;
    point.acceleration_mps2 = 0.0F;
    points.push_back(point);
  }
  return points;
}

}  // namespace

TEST(TrajectorySanitizerTest, RemovesOverlappingPointKeepingStopVelocity)
{
  auto points = make_straight_trajectory(5, 0.5, 2.0F);
  auto duplicated_stop = points.back();
  duplicated_stop.longitudinal_velocity_mps = 0.0F;
  // Duplicated terminal stop point, as published in the 2026-06-30 in-lane MRM incident.
  points.push_back(duplicated_stop);

  const auto removed = remove_overlap_points(points, 1e-3);

  EXPECT_EQ(removed, 1U);
  ASSERT_EQ(points.size(), 5U);
  // The stop (v=0) carried by the removed duplicate must not be lost.
  EXPECT_FLOAT_EQ(points.back().longitudinal_velocity_mps, 0.0F);
  for (size_t i = 1; i < points.size(); ++i) {
    EXPECT_GE(autoware_utils::calc_distance2d(points.at(i - 1), points.at(i)), 1e-3)
      << "overlapping points at index " << i - 1 << " and " << i;
  }
}

TEST(TrajectorySanitizerTest, RemovesDuplicateFromIncidentTrajectory)
{
  // Verbatim trajectory published at 2026-06-30 11:44:25.408 that crashed the MRM trajectory
  // follower: the terminal point is a bitwise duplicate of its predecessor (velocity 0 vs 1.44),
  // which broke the strictly-increasing arc-length assumption of MPC spline resampling.
  struct RawPoint
  {
    double x;
    double y;
    double qz;
    double qw;
    float v;
  };
  const RawPoint raw_points[] = {
    {89149.32953892737, 42424.34582301043, -0.946302462679479, 0.3232826149466643, 1.439218F},
    {89149.2504412572, 42424.2846383835, -0.9463024626799119, 0.3232826149453973, 1.439218F},
    {89149.17134358702, 42424.22345375656, -0.9463024626798169, 0.3232826149456753, 1.439218F},
    {89149.09224591685, 42424.16226912963, -0.9463024626804217, 0.3232826149439048, 1.439218F},
    {89149.01314824667, 42424.1010845027, -0.9463024626788296, 0.3232826149485653, 1.439218F},
    {89148.9340505765, 42424.03989987577, -0.946302462679479, 0.3232826149466643, 1.439218F},
    {89148.85495290632, 42423.97871524883, -0.9463024626791807, 0.3232826149475374, 1.439218F},
    {89148.77585523615, 42423.9175306219, -0.94630246268038, 0.3232826149440269, 1.439218F},
    {89148.69675756597, 42423.85634599496, -0.9463024626802341, 0.32328261494445404, 1.439218F},
    {89148.6176598958, 42423.795161368034, -0.9463024626755827, 0.3232826149580693, 1.439218F},
    {89148.53856222563, 42423.7339767411, -0.946302462679479, 0.3232826149466643, 1.439218F},
    {89148.53856222563, 42423.7339767411, -0.946302462679479, 0.3232826149466643, 0.0F},
  };

  TrajectoryPoints points;
  for (const auto & raw : raw_points) {
    TrajectoryPoint point;
    point.pose.position.x = raw.x;
    point.pose.position.y = raw.y;
    point.pose.orientation.z = raw.qz;
    point.pose.orientation.w = raw.qw;
    point.longitudinal_velocity_mps = raw.v;
    points.push_back(point);
  }

  const auto removed = remove_overlap_points(points, 1e-3);

  EXPECT_EQ(removed, 1U);
  ASSERT_EQ(points.size(), 11U);
  // The stop carried by the removed duplicate must survive on the terminal point.
  EXPECT_FLOAT_EQ(points.back().longitudinal_velocity_mps, 0.0F);
  for (size_t i = 1; i < points.size(); ++i) {
    EXPECT_GE(autoware_utils::calc_distance2d(points.at(i - 1), points.at(i)), 1e-3);
  }
}

TEST(TrajectorySanitizerTest, LeavesWellSpacedTrajectoryUntouched)
{
  auto points = make_straight_trajectory(5, 0.5, 2.0F);
  const auto expected = points;

  const auto removed = remove_overlap_points(points, 1e-3);

  EXPECT_EQ(removed, 0U);
  ASSERT_EQ(points.size(), expected.size());
  for (size_t i = 0; i < points.size(); ++i) {
    EXPECT_DOUBLE_EQ(points.at(i).pose.position.x, expected.at(i).pose.position.x);
    EXPECT_FLOAT_EQ(
      points.at(i).longitudinal_velocity_mps, expected.at(i).longitudinal_velocity_mps);
  }
}

}  // namespace autoware::in_lane_mrm_planner
