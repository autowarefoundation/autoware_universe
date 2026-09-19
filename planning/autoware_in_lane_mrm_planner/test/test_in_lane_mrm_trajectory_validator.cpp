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

#include "in_lane_mrm_trajectory_validator.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace autoware::in_lane_mrm_planner
{
namespace
{

Params make_default_params()
{
  Params params;
  params.trajectory_validator.enable = true;
  params.trajectory_validator.min_point_count = 3;
  return params;
}

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

TEST(InLaneMrmTrajectoryValidatorTest, AcceptsUniformLowSpeedTrajectory)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  const auto points = make_straight_trajectory(10, 1.0, 0.0F);
  EXPECT_TRUE(validator.validate(points).ok);
}

TEST(InLaneMrmTrajectoryValidatorTest, AcceptsMrmStopFromSpeedWithLongZeroTail)
{
  // Regression: a legitimate MRM stop from speed decelerates to zero quickly,
  // leaving a long zero-velocity tail (front fast, >=80% of points stopped).
  // The removed hazardous_step check used to drop this; it must now pass.
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  auto points = make_straight_trajectory(10, 1.0, 0.0F);
  points.front().longitudinal_velocity_mps = 3.0F;
  EXPECT_TRUE(validator.validate(points).ok);
}

TEST(InLaneMrmTrajectoryValidatorTest, AcceptsNonZeroProfile)
{
  // Regression: removed standstill_mismatch check. A nonzero profile is fine;
  // this is the publish gate, not a longitudinal-feasibility judge.
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  const auto points = make_straight_trajectory(10, 1.0, 5.0F);
  EXPECT_TRUE(validator.validate(points).ok);
}

TEST(InLaneMrmTrajectoryValidatorTest, RejectsInsufficientPointCount)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  const auto points = make_straight_trajectory(2, 1.0, 0.0F);
  const auto result = validator.validate(points);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.code, InLaneMrmTrajectoryValidator::FailureCode::INSUFFICIENT_POINT_COUNT);
}

TEST(InLaneMrmTrajectoryValidatorTest, RejectsEmptyTrajectory)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  const TrajectoryPoints points;
  const auto result = validator.validate(points);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.code, InLaneMrmTrajectoryValidator::FailureCode::INSUFFICIENT_POINT_COUNT);
}

TEST(InLaneMrmTrajectoryValidatorTest, RejectsNonFiniteValues)
{
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  auto points = make_straight_trajectory(5, 1.0, 0.0F);
  points.at(2).pose.position.x = std::numeric_limits<double>::quiet_NaN();
  const auto result = validator.validate(points);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.code, InLaneMrmTrajectoryValidator::FailureCode::NON_FINITE_VALUES);
}

TEST(InLaneMrmTrajectoryValidatorTest, AcceptsExactlyMinPointCount)
{
  // Boundary: a trajectory with exactly min_point_count points must pass.
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  const auto points = make_straight_trajectory(3, 1.0, 0.0F);
  EXPECT_TRUE(validator.validate(points).ok);
}

TEST(InLaneMrmTrajectoryValidatorTest, RejectsNonFiniteVelocity)
{
  // has_finite_values guards more than position; an infinite velocity must be rejected.
  const InLaneMrmTrajectoryValidator validator(make_default_params());
  auto points = make_straight_trajectory(5, 1.0, 0.0F);
  points.at(2).longitudinal_velocity_mps = std::numeric_limits<float>::infinity();
  const auto result = validator.validate(points);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.code, InLaneMrmTrajectoryValidator::FailureCode::NON_FINITE_VALUES);
}

}  // namespace autoware::in_lane_mrm_planner
