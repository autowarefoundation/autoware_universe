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

#include "autoware/steer_offset_estimator/utils.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::steer_offset_estimator::utils
{

class TestUtils : public ::testing::Test
{
protected:
  // Helper function to create a steering report
  static autoware_vehicle_msgs::msg::SteeringReport create_steering_report(float angle)
  {
    autoware_vehicle_msgs::msg::SteeringReport steer;
    steer.steering_tire_angle = angle;
    return steer;
  }

  // Helper function to create a pose
  static geometry_msgs::msg::PoseStamped create_pose(
    double x, double y, double z, double roll, double pitch, double yaw, double timestamp_sec)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp.sec = static_cast<int32_t>(timestamp_sec);
    pose.header.stamp.nanosec =
      static_cast<uint32_t>((timestamp_sec - pose.header.stamp.sec) * 1e9);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    return pose;
  }
};

TEST_F(TestUtils, ComputeRelativeRotationVectorIdentical)
{
  tf2::Quaternion q1(0.0, 0.0, 0.0, 1.0);  // Identity quaternion
  tf2::Quaternion q2(0.0, 0.0, 0.0, 1.0);  // Identity quaternion

  geometry_msgs::msg::Vector3 result = compute_relative_rotation_vector(q1, q2);

  EXPECT_NEAR(result.x, 0.0, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
  EXPECT_NEAR(result.z, 0.0, 1e-6);
}

TEST_F(TestUtils, ComputeRelativeRotationVectorZRotation)
{
  tf2::Quaternion q1;
  q1.setRPY(0.0, 0.0, 0.0);  // No rotation

  tf2::Quaternion q2;
  q2.setRPY(0.0, 0.0, M_PI / 4);  // 45 degree rotation around Z-axis

  geometry_msgs::msg::Vector3 result = compute_relative_rotation_vector(q1, q2);

  EXPECT_NEAR(result.x, 0.0, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
  EXPECT_NEAR(result.z, M_PI / 4, 1e-6);
}

TEST_F(TestUtils, CalcTwistFromPoseIdentical)
{
  auto pose_a = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  auto pose_b = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);  // Identical pose

  geometry_msgs::msg::Twist result = calc_twist_from_pose(pose_a, pose_b);

  EXPECT_EQ(result.linear.x, 0.0);
  EXPECT_EQ(result.linear.y, 0.0);
  EXPECT_EQ(result.linear.z, 0.0);
  EXPECT_EQ(result.angular.x, 0.0);
  EXPECT_EQ(result.angular.y, 0.0);
  EXPECT_EQ(result.angular.z, 0.0);
}

TEST_F(TestUtils, CalcTwistFromPoseLinearMotion)
{
  auto pose_a = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  auto pose_b = create_pose(3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0);  // Move 3m in 1 second

  geometry_msgs::msg::Twist result = calc_twist_from_pose(pose_a, pose_b);

  EXPECT_NEAR(result.linear.x, 3.0, 1e-6);  // 3 m/s
  EXPECT_EQ(result.linear.y, 0.0);
  EXPECT_EQ(result.linear.z, 0.0);
  EXPECT_NEAR(result.angular.x, 0.0, 1e-6);
  EXPECT_NEAR(result.angular.y, 0.0, 1e-6);
  EXPECT_NEAR(result.angular.z, 0.0, 1e-6);
}

TEST_F(TestUtils, CalcTwistFromPoseAngularMotion)
{
  auto pose_a = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  auto pose_b = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2, 2.0);  // 90 degree turn in 1 second

  geometry_msgs::msg::Twist result = calc_twist_from_pose(pose_a, pose_b);

  EXPECT_EQ(result.linear.x, 0.0);
  EXPECT_EQ(result.linear.y, 0.0);
  EXPECT_EQ(result.linear.z, 0.0);
  EXPECT_NEAR(result.angular.x, 0.0, 1e-6);
  EXPECT_NEAR(result.angular.y, 0.0, 1e-6);
  EXPECT_NEAR(result.angular.z, M_PI / 2, 1e-6);  // π/2 rad/s
}

TEST_F(TestUtils, CalcTwistFromPoseCombinedMotion)
{
  auto pose_a = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  auto pose_b = create_pose(2.0, 2.0, 0.0, 0.0, 0.0, M_PI / 4, 2.0);  // Move and turn in 1 second

  geometry_msgs::msg::Twist result = calc_twist_from_pose(pose_a, pose_b);

  double expected_linear_velocity = std::sqrt(2.0 * 2.0 + 2.0 * 2.0);  // sqrt(8) ≈ 2.83
  EXPECT_NEAR(result.linear.x, expected_linear_velocity, 1e-6);
  EXPECT_EQ(result.linear.y, 0.0);
  EXPECT_EQ(result.linear.z, 0.0);
  EXPECT_NEAR(result.angular.x, 0.0, 1e-6);
  EXPECT_NEAR(result.angular.y, 0.0, 1e-6);
  EXPECT_NEAR(result.angular.z, M_PI / 4, 1e-6);  // π/4 rad/s
}

TEST_F(TestUtils, CalcTwistFromPoseZeroTimeDifference)
{
  auto pose_a = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  auto pose_b = create_pose(3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);  // Same timestamp

  geometry_msgs::msg::Twist result = calc_twist_from_pose(pose_a, pose_b);

  // Should return zero twist when time difference is zero
  EXPECT_EQ(result.linear.x, 0.0);
  EXPECT_EQ(result.linear.y, 0.0);
  EXPECT_EQ(result.linear.z, 0.0);
  EXPECT_EQ(result.angular.x, 0.0);
  EXPECT_EQ(result.angular.y, 0.0);
  EXPECT_EQ(result.angular.z, 0.0);
}

}  // namespace autoware::steer_offset_estimator::utils
