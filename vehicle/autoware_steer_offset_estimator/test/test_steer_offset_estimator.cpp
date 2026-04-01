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

#include "autoware/steer_offset_estimator/steer_offset_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::steer_offset_estimator
{

class TestSteerOffsetEstimator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Set up test parameters
    params_.initial_covariance = 1.0;
    params_.initial_offset = 0.0;
    params_.wheel_base = 2.5;
    params_.min_velocity = 2.0;
    params_.max_steer = 0.5;

    estimator_ = std::make_unique<SteerOffsetEstimator>(params_);
  }

  SteerOffsetEstimatorParameters params_;
  std::unique_ptr<SteerOffsetEstimator> estimator_;

  // Helper function to create a pose
  static geometry_msgs::msg::PoseStamped create_pose(
    double x, double y, double yaw, double timestamp_sec)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp.sec = static_cast<int32_t>(timestamp_sec);
    pose.header.stamp.nanosec =
      static_cast<uint32_t>((timestamp_sec - pose.header.stamp.sec) * 1e9);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);

    return pose;
  }

  // Helper function to create a steering report
  static autoware_vehicle_msgs::msg::SteeringReport create_steering_report(
    float angle, double timestamp_sec)
  {
    autoware_vehicle_msgs::msg::SteeringReport steer;
    steer.steering_tire_angle = angle;
    steer.stamp.sec = static_cast<int32_t>(timestamp_sec);
    steer.stamp.nanosec = static_cast<uint32_t>((timestamp_sec - steer.stamp.sec) * 1e9);
    return steer;
  }
};

TEST_F(TestSteerOffsetEstimator, Constructor)
{
  EXPECT_EQ(estimator_->get_parameters().initial_covariance, params_.initial_covariance);
  EXPECT_EQ(estimator_->get_parameters().initial_offset, params_.initial_offset);
  EXPECT_EQ(estimator_->get_parameters().wheel_base, params_.wheel_base);
  EXPECT_EQ(estimator_->get_parameters().min_velocity, params_.min_velocity);
  EXPECT_EQ(estimator_->get_parameters().max_steer, params_.max_steer);
}

TEST_F(TestSteerOffsetEstimator, UpdateWithEmptyPoses)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  std::vector<autoware_vehicle_msgs::msg::SteeringReport> steers = {
    create_steering_report(0.1, 1.0)};

  auto result = estimator_->update(poses, steers);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().reason, "poses is empty");
}

TEST_F(TestSteerOffsetEstimator, UpdateWithEmptySteers)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses = {create_pose(0.0, 0.0, 0.0, 1.0)};
  std::vector<autoware_vehicle_msgs::msg::SteeringReport> steers;

  auto result = estimator_->update(poses, steers);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().reason, "steers is empty");
}

TEST_F(TestSteerOffsetEstimator, UpdateWithLowVelocity)
{
  // Create poses with very small movement (low velocity)
  std::vector<geometry_msgs::msg::PoseStamped> poses = {
    create_pose(0.0, 0.0, 0.0, 1.0), create_pose(0.1, 0.0, 0.0, 2.0)  // 0.1 m/s velocity
  };
  std::vector<autoware_vehicle_msgs::msg::SteeringReport> steers = {
    create_steering_report(0.1, 2.0)};

  auto result = estimator_->update(poses, steers);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().reason, "velocity is too low");
}

TEST_F(TestSteerOffsetEstimator, UpdateWithLargeSteeringAngle)
{
  // Create poses with sufficient velocity
  std::vector<geometry_msgs::msg::PoseStamped> poses = {
    create_pose(0.0, 0.0, 0.0, 1.0), create_pose(3.0, 0.0, 0.0, 2.0)  // 3.0 m/s velocity
  };
  std::vector<autoware_vehicle_msgs::msg::SteeringReport> steers = {
    create_steering_report(0.6, 2.0)};  // > max_steer

  auto result = estimator_->update(poses, steers);
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().reason, "steering angle is too large");
}

TEST_F(TestSteerOffsetEstimator, UpdateWithValidData)
{
  // Create poses with sufficient velocity and turning motion
  std::vector<geometry_msgs::msg::PoseStamped> poses = {
    create_pose(0.0, 0.0, 0.0, 1.0),
    create_pose(3.0, 0.0, 0.1, 2.0)  // Moving forward with slight turn
  };
  std::vector<autoware_vehicle_msgs::msg::SteeringReport> steers = {
    create_steering_report(0.2, 2.0)};

  auto result = estimator_->update(poses, steers);
  EXPECT_TRUE(result.has_value());
  EXPECT_NE(result.value().covariance, 0.0);
}

TEST_F(TestSteerOffsetEstimator, MultipleUpdates)
{
  // Perform multiple updates to test estimation convergence
  for (int i = 0; i < 5; ++i) {
    std::vector<geometry_msgs::msg::PoseStamped> poses = {
      create_pose(
        static_cast<double>(i * 3), 0.0, static_cast<double>(i * 0.1), static_cast<double>(i + 1)),
      create_pose(
        static_cast<double>((i + 1) * 3), 0.0, static_cast<double>((i + 1) * 0.1),
        static_cast<double>(i + 2))};
    std::vector<autoware_vehicle_msgs::msg::SteeringReport> steers = {
      create_steering_report(0.2, static_cast<double>(i + 2))};

    auto result = estimator_->update(poses, steers);
    if (i > 0) {  // Skip first iteration as it needs previous pose
      EXPECT_TRUE(result.has_value());
    }
  }
}

}  // namespace autoware::steer_offset_estimator
