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

#include "utils.hpp"

#include <autoware/speed_scale_corrector/types.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace autoware::speed_scale_corrector
{

class UtilsTest : public ::testing::Test
{
protected:
  static TimestampedImu create_imu(double sec, double angular_velocity_z)
  {
    return {sec, angular_velocity_z};
  }

  static TimestampedVelocity create_velocity(double sec, double velocity)
  {
    return {sec, velocity};
  }
};

TEST_F(UtilsTest, FindNearestImuReturnsClosestSample)
{
  const std::vector<TimestampedImu> imus = {
    create_imu(0.0, 0.1), create_imu(0.2, 0.3), create_imu(0.5, 0.7)};

  const auto result = find_nearest_imu(imus, 0.21);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->angular_velocity_z, 0.3, 1e-9);
  EXPECT_NEAR(result->stamp_diff, 0.01, 1e-9);
}

TEST_F(UtilsTest, FindNearestImuReturnsNulloptForEmptyInput)
{
  const std::vector<TimestampedImu> imus;
  const auto result = find_nearest_imu(imus, 1.0);

  EXPECT_FALSE(result.has_value());
}

TEST_F(UtilsTest, FindNearestVelocityReportReturnsClosestSample)
{
  const std::vector<TimestampedVelocity> velocity_reports = {
    create_velocity(0.0, 1.0), create_velocity(0.2, 3.0), create_velocity(0.5, 7.0)};

  const auto result = find_nearest_velocity_report(velocity_reports, 0.21);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 3.0, 1e-9);
  EXPECT_NEAR(result->stamp_diff, 0.01, 1e-9);
}

TEST_F(UtilsTest, FindNearestVelocityReportReturnsNulloptForEmptyInput)
{
  const std::vector<TimestampedVelocity> velocity_reports;
  const auto result = find_nearest_velocity_report(velocity_reports, 1.0);

  EXPECT_FALSE(result.has_value());
}

TEST_F(UtilsTest, CalcOdometryVelocityFromPoseDifference)
{
  const TimestampedPose pose_a{0.0, 0.0, 0.0, 0.0};
  const TimestampedPose pose_b{0.1, 1.0, 0.0, 0.0};

  EXPECT_NEAR(calc_odometry_velocity(pose_a, pose_b), 10.0, 1e-9);
}

}  // namespace autoware::speed_scale_corrector
