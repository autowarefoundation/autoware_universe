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

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>

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

  static VelocityReport create_velocity_msg(double sec, double velocity)
  {
    VelocityReport msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.longitudinal_velocity = static_cast<float>(velocity);
    return msg;
  }

  static PoseStamped create_pose_msg(double sec, double x, double y)
  {
    PoseStamped msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0.0;
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

TEST_F(UtilsTest, FindNearestVelocityReportReturnsClosestSample)
{
  const std::vector<VelocityReport> velocity_reports = {
    create_velocity_msg(0.0, 1.0), create_velocity_msg(0.2, 3.0), create_velocity_msg(0.5, 7.0)};

  const rclcpp::Time target_time(0, 210000000, RCL_ROS_TIME);
  const auto result = find_nearest_velocity_report(velocity_reports, target_time);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->longitudinal_velocity, 3.0, 1e-9);
  EXPECT_NEAR(result->stamp_diff, 0.01, 1e-9);
}

TEST_F(UtilsTest, FindNearestVelocityReportReturnsNulloptForEmptyInput)
{
  const std::vector<VelocityReport> velocity_reports;
  const auto result = find_nearest_velocity_report(velocity_reports, rclcpp::Time(1, 0));

  EXPECT_FALSE(result.has_value());
}

TEST_F(UtilsTest, CalcOdometryVelocityFromPoseDifference)
{
  const PoseStamped pose_a = create_pose_msg(0.0, 0.0, 0.0);
  const PoseStamped pose_b = create_pose_msg(0.1, 1.0, 0.0);

  EXPECT_NEAR(calc_odometry_velocity(pose_a, pose_b), 10.0, 1e-9);
}

}  // namespace autoware::speed_scale_corrector
