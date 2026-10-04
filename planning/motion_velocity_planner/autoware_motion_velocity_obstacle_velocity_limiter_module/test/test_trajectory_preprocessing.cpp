// Copyright 2026 The Autoware Contributors
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

#include "../src/trajectory_preprocessing.hpp"
#include "../src/types.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using autoware::motion_velocity_planner::obstacle_velocity_limiter::calculateSteeringAngles;
using autoware::motion_velocity_planner::obstacle_velocity_limiter::TrajectoryPoint;
using autoware::motion_velocity_planner::obstacle_velocity_limiter::TrajectoryPoints;

constexpr auto WHEEL_BASE = 2.79;

/// @brief generate a trajectory with a constant curvature, a constant velocity, and the given
/// initial heading, with points spaced by the given arc length
TrajectoryPoints generateConstantCurvatureTrajectory(
  const double initial_heading, const double curvature, const double velocity,
  const double arc_length, const size_t nb_points)
{
  TrajectoryPoints trajectory;
  double x = 0.0;
  double y = 0.0;
  double heading = initial_heading;
  for (size_t i = 0; i < nb_points; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = autoware_utils::create_quaternion_from_yaw(heading);
    p.longitudinal_velocity_mps = static_cast<float>(velocity);
    trajectory.push_back(p);
    // move along the chord of the arc so that the distance between points equals arc_length
    const auto d_heading = curvature * arc_length;
    x += arc_length * std::cos(heading + d_heading / 2.0);
    y += arc_length * std::sin(heading + d_heading / 2.0);
    heading += d_heading;
  }
  return trajectory;
}
}  // namespace

TEST(TestTrajectoryPreprocessing, calculateSteeringAnglesConstantCurvature)
{
  constexpr auto curvature = 0.05;
  auto trajectory = generateConstantCurvatureTrajectory(0.0, curvature, 5.0, 1.0, 10);
  calculateSteeringAngles(trajectory, WHEEL_BASE);
  const auto expected_steering = std::atan(WHEEL_BASE * curvature);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_NEAR(trajectory[i].front_wheel_angle_rad, expected_steering, 1e-3) << "index: " << i;
  }
}

// The heading jumps from +pi to -pi when the trajectory crosses the heading of pi. The heading
// difference must be normalized, otherwise the steering angle becomes close to -pi/2 at that point.
TEST(TestTrajectoryPreprocessing, calculateSteeringAnglesHeadingCrossingPi)
{
  constexpr auto curvature = 0.001;
  auto trajectory = generateConstantCurvatureTrajectory(M_PI - 0.0045, curvature, 5.0, 1.0, 10);
  calculateSteeringAngles(trajectory, WHEEL_BASE);
  const auto expected_steering = std::atan(WHEEL_BASE * curvature);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_NEAR(trajectory[i].front_wheel_angle_rad, expected_steering, 1e-4) << "index: " << i;
  }
}
