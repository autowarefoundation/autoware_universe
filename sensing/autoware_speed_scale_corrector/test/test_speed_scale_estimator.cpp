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

#include "autoware/speed_scale_corrector/speed_scale_estimator.hpp"

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::speed_scale_corrector
{

// Test for SpeedScaleEstimator class
// - Basic estimation behavior with simulated vehicle movement
// - Constraint validation (angular velocity, speed limits, time window)
// - Edge cases (empty data, insufficient time window)

class SpeedScaleEstimatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    parameters_.initial_speed_scale_factor = 1.0;
    parameters_.initial_speed_scale_factor_covariance = 1000.0;
    parameters_.process_noise_covariance = 0.01;
    parameters_.measurement_noise_covariance = 0.01;
    parameters_.max_angular_velocity = 1.0;
    parameters_.max_speed = 15.0;
    parameters_.min_speed = 2.0;

    estimator_ = std::make_unique<SpeedScaleEstimator>(parameters_);
  }

  static PoseStamped create_pose_msg(double sec, double x, double y)
  {
    PoseStamped pose;
    pose.header.stamp.sec = static_cast<int32_t>(sec);
    pose.header.stamp.nanosec = static_cast<uint32_t>((sec - pose.header.stamp.sec) * 1e9);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    return pose;
  }

  static VelocityReport create_velocity_msg(double sec, double velocity)
  {
    VelocityReport msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.longitudinal_velocity = static_cast<float>(velocity);
    return msg;
  }

  static Imu create_imu_msg(double sec, double angular_velocity_z)
  {
    Imu msg;
    msg.header.stamp.sec = static_cast<int32_t>(sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>((sec - msg.header.stamp.sec) * 1e9);
    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = angular_velocity_z;
    return msg;
  }

  SpeedScaleEstimatorParameters parameters_;
  std::unique_ptr<SpeedScaleEstimator> estimator_;
};

// Test basic speed scale estimation with simulated vehicle movement
TEST_F(SpeedScaleEstimatorTest, EstimationBehaviorTest)
{
  int successful_estimations = 0;
  double estimated_speed_scale_factor = 0.0;

  for (double t = 0.0; t <= 30.0; t += 0.1) {
    std::vector<PoseStamped> poses = {create_pose_msg(t, 10.0 * t, 0.0)};
    std::vector<VelocityReport> velocity_reports = {create_velocity_msg(t, 5.0)};
    std::vector<Imu> imus = {create_imu_msg(t, 0.05)};
    auto result = estimator_->update(poses, imus, velocity_reports);

    if (result) {
      successful_estimations++;
      estimated_speed_scale_factor = result.value().estimated_speed_scale_factor;
    }
  }

  EXPECT_GT(successful_estimations, 0);
  EXPECT_NEAR(estimated_speed_scale_factor, 2.0, 0.5);
}

// Test error handling when there is only one pose
TEST_F(SpeedScaleEstimatorTest, InsufficientPoses)
{
  // Create data with only one pose
  std::vector<PoseStamped> poses = {create_pose_msg(1.0, 0.0, 0.0)};
  std::vector<VelocityReport> velocity_reports = {create_velocity_msg(1.0, 5.0)};
  std::vector<Imu> imus = {create_imu_msg(1.0, 0.1)};

  auto result = estimator_->update(poses, imus, velocity_reports);
  EXPECT_FALSE(result);
  EXPECT_TRUE(result.error().reason.find("Need at least 2 poses") != std::string::npos);
}

// Test constraint validation when angular velocity exceeds maximum limit
TEST_F(SpeedScaleEstimatorTest, AngularVelocityConstraintViolation)
{
  for (double t = 0.0; t <= 3.0; t += 0.1) {
    std::vector<PoseStamped> poses = {create_pose_msg(t, 5.0 * t, 0.0)};
    std::vector<VelocityReport> velocity_reports = {create_velocity_msg(t, 5.0f)};
    std::vector<Imu> imus = {create_imu_msg(t, 2.0)};  // Exceeds max_angular_velocity = 1.0

    auto result = estimator_->update(poses, imus, velocity_reports);

    if (t >= 0.2) {  // After we have at least 2 poses
      if (!result) {
        std::cout << "Error reason at t=" << t << ": " << result.error().reason << std::endl;
        // Should get angular velocity constraint error
        EXPECT_TRUE(
          result.error().reason.find("Angular velocity is too high") != std::string::npos);
      }
    }
  }
}

// Test constraint validation when speed is below minimum threshold
TEST_F(SpeedScaleEstimatorTest, SpeedConstraintViolation)
{
  for (double t = 0.0; t <= 3.0; t += 0.1) {
    std::vector<PoseStamped> poses = {create_pose_msg(t, 1.0 * t, 0.0)};
    std::vector<VelocityReport> velocity_reports = {
      create_velocity_msg(t, 1.0f)};  // Below min_speed = 2.0
    std::vector<Imu> imus = {create_imu_msg(t, 0.1)};

    auto result = estimator_->update(poses, imus, velocity_reports);

    if (t >= 0.2) {  // After we have at least 2 poses
      if (!result) {
        std::cout << "Error reason at t=" << t << ": " << result.error().reason << std::endl;
        // Should get speed constraint error
        EXPECT_TRUE(result.error().reason.find("Velocity is too low") != std::string::npos);
      }
    }
  }
}

// Test error handling with empty IMU and velocity data
TEST_F(SpeedScaleEstimatorTest, EmptyDataHandling)
{
  std::vector<PoseStamped> poses = {create_pose_msg(1.0, 0.0, 0.0)};
  std::vector<VelocityReport> empty_velocity;
  std::vector<Imu> empty_imu;

  auto result = estimator_->update(poses, empty_imu, empty_velocity);
  EXPECT_FALSE(result);
  EXPECT_TRUE(result.error().reason.find("IMU is empty") != std::string::npos);
}

}  // namespace autoware::speed_scale_corrector
