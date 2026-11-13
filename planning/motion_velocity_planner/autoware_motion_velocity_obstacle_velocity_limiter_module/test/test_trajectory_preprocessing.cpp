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

// The steering angle only depends on the geometry of the trajectory, not on its velocity profile.
TEST(TestTrajectoryPreprocessing, calculateSteeringAnglesVaryingVelocity)
{
  constexpr auto curvature = 0.05;
  auto trajectory = generateConstantCurvatureTrajectory(0.0, curvature, 0.0, 1.0, 10);
  for (size_t i = 0; i < trajectory.size(); ++i) {
    trajectory[i].longitudinal_velocity_mps = static_cast<float>(10.0 - static_cast<double>(i));
  }
  calculateSteeringAngles(trajectory, WHEEL_BASE);
  const auto expected_steering = std::atan(WHEEL_BASE * curvature);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_NEAR(trajectory[i].front_wheel_angle_rad, expected_steering, 1e-3) << "index: " << i;
  }
}

// Points with a zero velocity (e.g., after a stop point) must not produce NaN steering angles.
TEST(TestTrajectoryPreprocessing, calculateSteeringAnglesZeroVelocity)
{
  constexpr auto curvature = 0.05;
  auto trajectory = generateConstantCurvatureTrajectory(0.0, curvature, 0.0, 1.0, 10);
  calculateSteeringAngles(trajectory, WHEEL_BASE);
  const auto expected_steering = std::atan(WHEEL_BASE * curvature);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_FALSE(std::isnan(trajectory[i].front_wheel_angle_rad)) << "index: " << i;
    EXPECT_NEAR(trajectory[i].front_wheel_angle_rad, expected_steering, 1e-3) << "index: " << i;
  }
}

// A duplicated point (zero-length segment) keeps the steering angle of the previous point.
TEST(TestTrajectoryPreprocessing, calculateSteeringAnglesDuplicatedPoint)
{
  constexpr auto curvature = 0.05;
  auto trajectory = generateConstantCurvatureTrajectory(0.0, curvature, 5.0, 1.0, 10);
  trajectory.insert(trajectory.begin() + 5, trajectory[4]);
  calculateSteeringAngles(trajectory, WHEEL_BASE);
  const auto expected_steering = std::atan(WHEEL_BASE * curvature);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_NEAR(trajectory[i].front_wheel_angle_rad, expected_steering, 1e-3) << "index: " << i;
  }
}

// A heading change at a duplicated point is accounted for in the next segment instead of
// producing a steering angle of +-pi/2.
TEST(TestTrajectoryPreprocessing, calculateSteeringAnglesDuplicatedPointWithHeadingChange)
{
  constexpr auto d_heading = 0.1;
  auto trajectory = generateConstantCurvatureTrajectory(0.0, 0.0, 5.0, 1.0, 3);
  // insert a point at the same position as the first point, rotated by d_heading
  auto rotated_point = trajectory[0];
  rotated_point.pose.orientation = autoware_utils::create_quaternion_from_yaw(d_heading);
  trajectory.insert(trajectory.begin() + 1, rotated_point);
  // the following points also keep the rotated heading
  for (size_t i = 2; i < trajectory.size(); ++i) {
    trajectory[i].pose.orientation = autoware_utils::create_quaternion_from_yaw(d_heading);
  }
  trajectory[0].front_wheel_angle_rad = 0.0f;
  calculateSteeringAngles(trajectory, WHEEL_BASE);
  // duplicated point: previous steering angle is kept
  EXPECT_NEAR(trajectory[1].front_wheel_angle_rad, 0.0, 1e-6);
  // next segment (length 1.0): the heading change happened over that segment
  EXPECT_NEAR(trajectory[2].front_wheel_angle_rad, std::atan(WHEEL_BASE * d_heading / 1.0), 1e-6);
  // after that, the heading is constant
  EXPECT_NEAR(trajectory[3].front_wheel_angle_rad, 0.0, 1e-6);
}
