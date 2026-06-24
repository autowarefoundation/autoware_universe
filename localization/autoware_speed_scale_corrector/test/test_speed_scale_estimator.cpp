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

#include "speed_scale_estimator.hpp"

#include <autoware/speed_scale_corrector/types.hpp>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace autoware::speed_scale_corrector
{

class SpeedScaleEstimatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    parameters_.update_interval = 0.1;
    parameters_.initial_speed_scale_factor = 1.0;
    parameters_.initial_speed_scale_factor_covariance = 1000.0;
    parameters_.process_noise_covariance = 0.01;
    parameters_.measurement_noise_covariance = 0.01;
    parameters_.max_angular_velocity = 0.0105;  // ~0.6 deg/s
    parameters_.max_speed = 17.0;
    parameters_.min_speed = 6.0;

    estimator_ = std::make_unique<SpeedScaleEstimator>(parameters_);
  }

  static TimestampedPose create_pose(double sec, double x, double y)
  {
    return {sec, x, y, 0.0};
  }

  static TimestampedVelocity create_velocity(double sec, double velocity)
  {
    return {sec, velocity};
  }

  static TimestampedImu create_imu(double sec, double angular_velocity_z)
  {
    return {sec, angular_velocity_z};
  }

  SpeedScaleEstimatorParameters parameters_;
  std::unique_ptr<SpeedScaleEstimator> estimator_;
};

TEST_F(SpeedScaleEstimatorTest, EstimationBehaviorTest)
{
  int successful_estimations = 0;
  double estimated_speed_scale_factor = 0.0;

  for (double t = 0.0; t <= 30.0; t += 0.1) {
    std::vector<TimestampedPose> poses = {create_pose(t, 10.0 * t, 0.0)};
    std::vector<TimestampedVelocity> velocity_reports = {create_velocity(t, 5.0)};
    std::vector<TimestampedImu> imus = {create_imu(t, 0.005)};
    auto result = estimator_->update(poses, imus, velocity_reports);

    if (result) {
      successful_estimations++;
      estimated_speed_scale_factor = result.value().estimated_speed_scale_factor;
    }
  }

  EXPECT_GT(successful_estimations, 0);
  EXPECT_NEAR(estimated_speed_scale_factor, 2.0, 0.5);
}

TEST_F(SpeedScaleEstimatorTest, InsufficientPoses)
{
  std::vector<TimestampedPose> poses = {create_pose(1.0, 0.0, 0.0)};
  std::vector<TimestampedVelocity> velocity_reports = {create_velocity(1.0, 5.0)};
  std::vector<TimestampedImu> imus = {create_imu(1.0, 0.1)};

  auto result = estimator_->update(poses, imus, velocity_reports);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::WaitingForNextPose);
}

TEST_F(SpeedScaleEstimatorTest, AngularVelocityConstraintViolation)
{
  std::vector<TimestampedPose> poses_t0 = {create_pose(0.0, 0.0, 0.0)};
  std::vector<TimestampedVelocity> velocity_reports_t0 = {create_velocity(0.0, 5.0)};
  std::vector<TimestampedImu> imus_t0 = {create_imu(0.0, 0.0)};
  (void)estimator_->update(poses_t0, imus_t0, velocity_reports_t0);

  std::vector<TimestampedPose> poses = {create_pose(0.1, 0.5, 0.0)};
  std::vector<TimestampedVelocity> velocity_reports = {create_velocity(0.1, 5.0)};
  std::vector<TimestampedImu> imus = {create_imu(0.1, 0.05)};

  const auto result = estimator_->update(poses, imus, velocity_reports);

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::AngularVelocityTooHigh);
}

TEST_F(SpeedScaleEstimatorTest, SpeedConstraintViolation)
{
  std::vector<TimestampedPose> poses_t0 = {create_pose(0.0, 0.0, 0.0)};
  std::vector<TimestampedVelocity> velocity_reports_t0 = {create_velocity(0.0, 5.0)};
  std::vector<TimestampedImu> imus_t0 = {create_imu(0.0, 0.0)};
  (void)estimator_->update(poses_t0, imus_t0, velocity_reports_t0);

  std::vector<TimestampedPose> poses = {create_pose(0.1, 0.1, 0.0)};
  std::vector<TimestampedVelocity> velocity_reports = {create_velocity(0.1, 1.0)};
  std::vector<TimestampedImu> imus = {create_imu(0.1, 0.005)};

  const auto result = estimator_->update(poses, imus, velocity_reports);

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::VelocityTooLow);
}

TEST_F(SpeedScaleEstimatorTest, VelocityReportTimestampMismatch)
{
  std::vector<TimestampedPose> poses_t0 = {create_pose(0.0, 0.0, 0.0)};
  std::vector<TimestampedVelocity> velocity_reports_t0 = {create_velocity(0.0, 5.0)};
  std::vector<TimestampedImu> imus_t0 = {create_imu(0.0, 0.0)};
  (void)estimator_->update(poses_t0, imus_t0, velocity_reports_t0);

  std::vector<TimestampedPose> poses = {create_pose(0.1, 1.0, 0.0)};
  std::vector<TimestampedVelocity> velocity_reports = {create_velocity(0.5, 5.0)};
  std::vector<TimestampedImu> imus = {create_imu(0.1, 0.0)};

  const auto result = estimator_->update(poses, imus, velocity_reports);

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::VelocityReportTimestampMismatch);
}

TEST_F(SpeedScaleEstimatorTest, EmptyDataHandling)
{
  std::vector<TimestampedPose> poses = {create_pose(1.0, 0.0, 0.0)};
  std::vector<TimestampedVelocity> empty_velocity;
  std::vector<TimestampedImu> empty_imu;

  auto result = estimator_->update(poses, empty_imu, empty_velocity);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error().reason, UpdateFailureReason::ImuEmpty);
}

}  // namespace autoware::speed_scale_corrector
