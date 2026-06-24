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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::speed_scale_corrector
{

class UtilsTest : public ::testing::Test
{
protected:
  static Imu create_imu_msg(double sec, double angular_velocity_z)
  {
    Imu msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.angular_velocity.z = angular_velocity_z;
    return msg;
  }
};

TEST_F(UtilsTest, FindNearestImuReturnsClosestSample)
{
  const std::vector<Imu> imus = {
    create_imu_msg(0.0, 0.1), create_imu_msg(0.2, 0.3), create_imu_msg(0.5, 0.7)};

  const rclcpp::Time target_time(0, 210000000, RCL_ROS_TIME);
  const auto result = find_nearest_imu(imus, target_time);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->angular_velocity_z, 0.3, 1e-9);
  EXPECT_NEAR(result->stamp_diff, 0.01, 1e-9);
}

TEST_F(UtilsTest, FindNearestImuReturnsNulloptForEmptyInput)
{
  const std::vector<Imu> imus;
  const auto result = find_nearest_imu(imus, rclcpp::Time(1, 0));

  EXPECT_FALSE(result.has_value());
}

}  // namespace autoware::speed_scale_corrector
