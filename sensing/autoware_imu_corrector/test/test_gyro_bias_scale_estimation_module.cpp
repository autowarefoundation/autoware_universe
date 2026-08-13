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

#include "autoware/imu_corrector/gyro_bias_estimator.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <iostream>

namespace autoware::imu_corrector
{
class GyroBiasEstimatorTest : public ::testing::Test
{
protected:
  rclcpp::NodeOptions options;
  GyroBiasEstimator module_;

  rclcpp::NodeOptions getNodeOptionsWithDefaultParams()
  {
    rclcpp::NodeOptions node_options;

    node_options.append_parameter_override("gyro_bias_threshold", 0.009);
    node_options.append_parameter_override("angular_velocity_offset_x", 0.0);
    node_options.append_parameter_override("angular_velocity_offset_y", 0.0);
    node_options.append_parameter_override("angular_velocity_offset_z", 0.0);
    node_options.append_parameter_override("timer_callback_interval_sec", 0.1);
    node_options.append_parameter_override("diagnostics_updater_interval_sec", 0.1);
    node_options.append_parameter_override("straight_motion_ang_vel_upper_limit", 0.015);
    node_options.append_parameter_override("estimate_scale_init", 1.0);
    node_options.append_parameter_override("min_allowed_scale", 0.9);
    node_options.append_parameter_override("max_allowed_scale", 1.1);
    node_options.append_parameter_override("alpha", 0.99);
    node_options.append_parameter_override("alpha_ndt_rate", 0.6);
    node_options.append_parameter_override("threshold_to_estimate_scale", 0.1);
    node_options.append_parameter_override("percentage_scale_rate_allow_correct", 0.04);
    node_options.append_parameter_override("alpha_gyro", 0.0);
    node_options.append_parameter_override("delay_gyro_ms", 170);
    node_options.append_parameter_override("buffer_size_gyro", 200);
    node_options.append_parameter_override("samples_filter_pose_rate", 2);
    node_options.append_parameter_override("samples_filter_gyro_rate", 6);
    // EKF rate
    node_options.append_parameter_override("ekf_rate.max_variance_p", 1e-8);
    node_options.append_parameter_override("ekf_rate.variance_p_after", 5e-9);
    node_options.append_parameter_override("ekf_rate.process_noise_q", 1e-14);
    node_options.append_parameter_override("ekf_rate.process_noise_q_after", 1e-15);
    node_options.append_parameter_override("ekf_rate.measurement_noise_r", 0.00007);
    node_options.append_parameter_override("ekf_rate.measurement_noise_r_after", 0.00005);
    node_options.append_parameter_override("ekf_rate.samples_to_init", 120);
    node_options.append_parameter_override("ekf_rate.min_covariance", 1e-11);

    // EKF angle
    node_options.append_parameter_override("ekf_angle.process_noise_q_angle", 1e-12);
    node_options.append_parameter_override("ekf_angle.variance_p_angle", 5e-9);
    node_options.append_parameter_override("ekf_angle.measurement_noise_r_angle", 0.00005);
    node_options.append_parameter_override("ekf_angle.min_covariance_angle", 1e-11);
    node_options.append_parameter_override("ekf_angle.decay_coefficient", 0.99999998);
    node_options.append_parameter_override("ekf_angle.samples_in_bounds_to_init", 20);
    node_options.append_parameter_override("ekf_angle.max_reinitialization_retries", 5);

    // IMU scale injection
    node_options.append_parameter_override("scale_imu_injection.modify_imu_scale", false);
    node_options.append_parameter_override("scale_imu_injection.scale_on_purpose", 1.0);
    node_options.append_parameter_override("scale_imu_injection.bias_on_purpose", 0.0);
    node_options.append_parameter_override("scale_imu_injection.drift_scale", 0.0);
    node_options.append_parameter_override("scale_imu_injection.drift_bias", 0.0);

    return node_options;
  }

  GyroBiasEstimatorTest() : module_(getNodeOptionsWithDefaultParams()) {}
};

TEST_F(GyroBiasEstimatorTest, ModifyImuAppliesBiasAndScale)
{
  sensor_msgs::msg::Imu imu_in;
  GyroBiasEstimator::ScaleImuSignal scale_imu{};
  rclcpp::Time time(100);

  imu_in.header.frame_id = "base_link";
  imu_in.angular_velocity.x = 1.0;
  imu_in.angular_velocity.y = 2.0;
  imu_in.angular_velocity.z = 3.0;

  scale_imu.scale_final_ = 1.0;
  scale_imu.bias_final_ = 0.0;
  scale_imu.drift_scale_ = 0.1;
  scale_imu.drift_bias_ = 0.5;

  auto imu_out = module_.modify_imu(imu_in, scale_imu, time);

  double expected_scale = 1.0 + 0.1;
  double expected_bias = 0.0 + 0.5;

  EXPECT_EQ(imu_out.header.stamp, time);
  EXPECT_EQ(imu_out.header.frame_id, imu_in.header.frame_id);

  EXPECT_DOUBLE_EQ(imu_out.angular_velocity.x, expected_scale * 1.0 + expected_bias);
  EXPECT_DOUBLE_EQ(imu_out.angular_velocity.y, expected_scale * 2.0 + expected_bias);
  EXPECT_DOUBLE_EQ(imu_out.angular_velocity.z, expected_scale * 3.0 + expected_bias);
}
TEST_F(GyroBiasEstimatorTest, ExtractYawFromPoseReturnsCorrectYaw)
{
  geometry_msgs::msg::Quaternion quat_msg;
  tf2::Quaternion tf_quat;

  double expected_yaw = M_PI / 4.0;
  tf_quat.setRPY(0, 0, expected_yaw);

  quat_msg.x = tf_quat.x();
  quat_msg.y = tf_quat.y();
  quat_msg.z = tf_quat.z();
  quat_msg.w = tf_quat.w();

  tf2::Quaternion quat_out;
  double yaw_extracted = module_.extract_yaw_from_pose(quat_msg, quat_out);

  EXPECT_NEAR(expected_yaw, yaw_extracted, 1e-6);
}

TEST_F(GyroBiasEstimatorTest, ShouldSkipUpdateReturnsTrueWhenRateSmall)
{
  double gyro_yaw_rate = 0.001;  // smaller than threshold
  bool result = module_.should_skip_update(gyro_yaw_rate);

  EXPECT_TRUE(result);
}

TEST_F(GyroBiasEstimatorTest, ShouldSkipUpdateReturnsFalseWhenRateHigh)
{
  double gyro_yaw_rate = 0.2;  // bigger than threshold
  bool result = module_.should_skip_update(gyro_yaw_rate);

  EXPECT_FALSE(result);
}

TEST_F(GyroBiasEstimatorTest, PositiveAngleEKFConvergence)
{
  GyroBiasEstimator::EKFEstimateScaleAngleVars ekf_angle{};
  ekf_angle.x_state_ << Eigen::Vector2d(1.02, 1.00);
  ekf_angle.max_variance_p_angle_ = 5e-8;
  ekf_angle.min_covariance_angle_ = 1e-6;
  ekf_angle.p_angle_ << ekf_angle.max_variance_p_angle_, 0, 0, ekf_angle.max_variance_p_angle_;
  ekf_angle.q_angle_ << 0, 0, 0, 4e-11;
  double noise = 0.0005;
  ekf_angle.r_angle_ << noise * noise;
  ekf_angle.estimated_scale_angle_ = 1.0;
  ekf_angle.filtered_scale_angle_ = 1.0;
  ekf_angle.decay_coefficient_ = 0.99999998;

  double true_yaw = 0.10;  // Yaw angle
  double factor_scale = 1.04;
  int n_iter = 1000;
  double gyro_rate = 0.1;
  double angle_gyro = true_yaw;

  for (int i = 0; i < n_iter; ++i) {
    angle_gyro = factor_scale * true_yaw;
    ekf_angle.x_state_(0) = angle_gyro;
    ekf_angle.x_state_(1) = (ekf_angle.x_state_(1) * ekf_angle.decay_coefficient_);
    Eigen::Matrix2d f_matrix;
    f_matrix << 1, gyro_rate, 0, ekf_angle.decay_coefficient_;
    ekf_angle.p_angle_ = f_matrix * ekf_angle.p_angle_ * f_matrix.transpose() + ekf_angle.q_angle_;
    if (i % 2 == 0) {
      module_.update_angle_ekf(true_yaw, ekf_angle);
      angle_gyro = true_yaw;
    }
  }
  std::cout << " Converged: " << 1 / ekf_angle.x_state_(1) << std::endl;

  // After many iterations, the state should converge to true_yaw
  EXPECT_NEAR(1 / ekf_angle.x_state_(1), factor_scale, 7e-3);
}

TEST_F(GyroBiasEstimatorTest, NegativeAngleEKFConvergence)
{
  GyroBiasEstimator::EKFEstimateScaleAngleVars ekf_angle{};
  ekf_angle.x_state_ << Eigen::Vector2d(1.02, 1.00);
  ekf_angle.max_variance_p_angle_ = 5e-8;
  ekf_angle.min_covariance_angle_ = 1e-6;
  ekf_angle.p_angle_ << ekf_angle.max_variance_p_angle_, 0, 0, ekf_angle.max_variance_p_angle_;
  ekf_angle.q_angle_ << 0, 0, 0, 4e-11;
  double noise = 0.0005;
  ekf_angle.r_angle_ << noise * noise;
  ekf_angle.estimated_scale_angle_ = 1.0;
  ekf_angle.filtered_scale_angle_ = 1.0;
  ekf_angle.decay_coefficient_ = 0.99999998;

  double true_yaw = 0.10;  // Yaw angle
  double factor_scale = 0.96;
  int n_iter = 1000;
  double gyro_rate = 0.1;
  double angle_gyro = true_yaw;

  for (int i = 0; i < n_iter; ++i) {
    angle_gyro = factor_scale * true_yaw;
    ekf_angle.x_state_(0) = angle_gyro;
    ekf_angle.x_state_(1) = (ekf_angle.x_state_(1) * ekf_angle.decay_coefficient_);
    Eigen::Matrix2d f_matrix;
    f_matrix << 1, gyro_rate, 0, ekf_angle.decay_coefficient_;
    ekf_angle.p_angle_ = f_matrix * ekf_angle.p_angle_ * f_matrix.transpose() + ekf_angle.q_angle_;
    if (i % 2 == 0) {
      module_.update_angle_ekf(true_yaw, ekf_angle);
      angle_gyro = true_yaw;
    }
  }
  std::cout << " Converged: " << 1 / ekf_angle.x_state_(1) << std::endl;

  // After many iterations, the state should converge to true_yaw
  EXPECT_NEAR(1 / ekf_angle.x_state_(1), factor_scale, 7e-3);
}
}  // namespace autoware::imu_corrector
