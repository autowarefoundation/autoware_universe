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

#include "speed_scale_estimator_processor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::speed_scale_estimator
{

class SpeedScaleEstimatorProcessorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    parameters_.update_interval = 0.1;
    parameters_.max_pose_lag = 0.5;
    parameters_.max_stamp_lag = 0.2;
    parameters_.sensor_buffer_duration = 1.0;
    parameters_.initial_speed_scale_factor = 1.0;
    parameters_.initial_speed_scale_factor_covariance = 1000.0;
    parameters_.process_noise_covariance = 0.01;
    parameters_.measurement_noise_covariance = 0.01;
    parameters_.max_angular_velocity = 0.0105;
    parameters_.max_speed = 17.0;
    parameters_.min_speed = 6.0;

    processor_ = std::make_unique<SpeedScaleEstimatorProcessor>(parameters_);
  }

  static PoseStamped::SharedPtr create_pose_msg(double sec, double x, double y)
  {
    auto msg = std::make_shared<PoseStamped>();
    msg->header.stamp.sec = static_cast<int32_t>(sec);
    msg->header.stamp.nanosec = static_cast<uint32_t>((sec - msg->header.stamp.sec) * 1e9);
    msg->pose.position.x = x;
    msg->pose.position.y = y;
    msg->pose.position.z = 0.0;
    return msg;
  }

  static VelocityReport::SharedPtr create_velocity_msg(double sec, double velocity)
  {
    auto msg = std::make_shared<VelocityReport>();
    msg->header.stamp.sec = static_cast<int32_t>(sec);
    msg->header.stamp.nanosec = static_cast<uint32_t>((sec - msg->header.stamp.sec) * 1e9);
    msg->longitudinal_velocity = static_cast<float>(velocity);
    return msg;
  }

  static Imu::SharedPtr create_imu_msg(double sec, double angular_velocity_z)
  {
    auto msg = std::make_shared<Imu>();
    msg->header.stamp.sec = static_cast<int32_t>(sec);
    msg->header.stamp.nanosec = static_cast<uint32_t>((sec - msg->header.stamp.sec) * 1e9);
    msg->angular_velocity.z = angular_velocity_z;
    return msg;
  }

  SpeedScaleEstimatorParameters parameters_;
  std::unique_ptr<SpeedScaleEstimatorProcessor> processor_;
};

TEST_F(SpeedScaleEstimatorProcessorTest, ProcessReturnsWaitingForNextPoseOnFirstUpdate)
{
  const std::vector<PoseStamped::ConstSharedPtr> poses = {create_pose_msg(0.0, 0.0, 0.0)};
  const std::vector<Imu::ConstSharedPtr> imus = {create_imu_msg(0.0, 0.0)};
  const std::vector<VelocityReport::ConstSharedPtr> velocity_reports = {
    create_velocity_msg(0.0, 5.0)};

  const auto result = processor_->process(poses, imus, velocity_reports);

  EXPECT_FALSE(result.updated);
  ASSERT_FALSE(result.estimation_result);
  EXPECT_EQ(result.estimation_result.error().reason, UpdateFailureReason::WaitingForNextPose);
}

TEST_F(SpeedScaleEstimatorProcessorTest, MakeDebugInfoContainsFailureReason)
{
  SpeedScaleEstimatorProcessResult result;
  result.updated = false;
  result.estimation_result =
    tl::make_unexpected(SpeedScaleEstimatorNotUpdated{UpdateFailureReason::PoseEmpty, {}, 1.0});

  const auto debug_info = SpeedScaleEstimatorProcessor::make_debug_info(result, rclcpp::Time(1, 0));

  EXPECT_EQ(debug_info.stamp.sec, 1);
  EXPECT_NE(debug_info.data.find("Pose is empty"), std::string::npos);
  EXPECT_NE(debug_info.data.find("1"), std::string::npos);
}

TEST_F(SpeedScaleEstimatorProcessorTest, MakeScaleFactorMsgUsesUpdatedValue)
{
  SpeedScaleEstimatorUpdated updated;
  updated.estimated_speed_scale_factor = 1.25;

  const auto msg = SpeedScaleEstimatorProcessor::make_scale_factor_msg(updated, rclcpp::Time(2, 0));

  EXPECT_EQ(msg.stamp.sec, 2);
  EXPECT_FLOAT_EQ(msg.data, 1.25f);
}

}  // namespace autoware::speed_scale_estimator
