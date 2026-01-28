// Copyright 2020 Tier IV, Inc.
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

#include "autoware/imu_corrector/imu_corrector_core.hpp"

#include <memory>
#include <string>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <algorithm>

std::array<double, 9> transform_covariance(const std::array<double, 9> & cov)
{
  using COV_IDX = autoware_utils::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = 0.0;
  max_cov = std::max(max_cov, cov[COV_IDX::X_X]);
  max_cov = std::max(max_cov, cov[COV_IDX::Y_Y]);
  max_cov = std::max(max_cov, cov[COV_IDX::Z_Z]);

  std::array<double, 9> cov_transformed{};
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

geometry_msgs::msg::Vector3 transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
}

namespace autoware::imu_corrector
{
ImuCorrector::ImuCorrector(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_corrector", options),
  output_frame_(declare_parameter<std::string>("base_link", "base_link"))
{
  transform_listener_ = std::make_shared<autoware_utils::TransformListener>(this);

  angular_velocity_offset_x_imu_link_ = declare_parameter<double>("angular_velocity_offset_x", 0.0);
  angular_velocity_offset_y_imu_link_ = declare_parameter<double>("angular_velocity_offset_y", 0.0);
  angular_velocity_offset_z_imu_link_ = declare_parameter<double>("angular_velocity_offset_z", 0.0);

  angular_velocity_stddev_xx_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_xx", 0.03);
  angular_velocity_stddev_yy_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_yy", 0.03);
  angular_velocity_stddev_zz_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_zz", 0.03);

  accel_stddev_imu_link_ = declare_parameter<double>("acceleration_stddev", 10000.0);

  correct_for_static_bias_ =
    declare_parameter<bool>("on_off_correction.correct_for_static_bias", true);
  correct_for_dynamic_bias_ =
    declare_parameter<bool>("on_off_correction.correct_for_dynamic_bias", false);
  correct_for_scale_ = declare_parameter<bool>("on_off_correction.correct_for_scale", false);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "input", rclcpp::QoS{1}, std::bind(&ImuCorrector::callback_imu, this, std::placeholders::_1));
  gyro_bias_sub_ = create_subscription<Vector3Stamped>(
    "gyro_bias_input", rclcpp::SensorDataQoS(),
    std::bind(&ImuCorrector::callback_bias, this, std::placeholders::_1));
  gyro_scale_sub_ = create_subscription<Vector3Stamped>(
    "gyro_scale_input", rclcpp::SensorDataQoS(),
    std::bind(&ImuCorrector::callback_scale, this, std::placeholders::_1));
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
  gyro_scale_.vector.x = 1.0;
  gyro_scale_.vector.y = 1.0;
  gyro_scale_.vector.z = 1.0;

  RCLCPP_INFO(
    this->get_logger(), "correct_for_static_bias: %s", correct_for_static_bias_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "correct_for_dynamic_bias: %s",
    correct_for_dynamic_bias_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "correct_for_scale: %s", correct_for_scale_ ? "true" : "false");

  if (correct_for_static_bias_ && correct_for_dynamic_bias_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Both static and dynamic gyro bias correction are enabled."
      "Disabling static bias correction.");
    correct_for_static_bias_ = false;
  }
}

void ImuCorrector::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg = *imu_msg_ptr;

  if (
    gyro_scale_.vector.x == 0.0 || gyro_scale_.vector.y == 0.0 || gyro_scale_.vector.z == 0.0 ||
    std::isnan(gyro_scale_.vector.x) || std::isnan(gyro_scale_.vector.y) ||
    std::isnan(gyro_scale_.vector.z) || std::isinf(gyro_scale_.vector.x) ||
    std::isinf(gyro_scale_.vector.y) || std::isinf(gyro_scale_.vector.z)) {
    RCLCPP_ERROR(this->get_logger(), "Gyro scale is zero, not correcting imu.");
    gyro_scale_.vector.x = 1.0;
    gyro_scale_.vector.y = 1.0;
    gyro_scale_.vector.z = 1.0;
  }

  if (correct_for_static_bias_) {
    imu_msg.angular_velocity.x -= angular_velocity_offset_x_imu_link_;
    imu_msg.angular_velocity.y -= angular_velocity_offset_y_imu_link_;
    imu_msg.angular_velocity.z -= angular_velocity_offset_z_imu_link_;
  }

  if (correct_for_dynamic_bias_) {
    imu_msg.angular_velocity.x -= gyro_bias_.vector.x;
    imu_msg.angular_velocity.y -= gyro_bias_.vector.y;
    imu_msg.angular_velocity.z -= gyro_bias_.vector.z;
  }

  if (correct_for_scale_) {
    imu_msg.angular_velocity.x /= gyro_scale_.vector.x;
    imu_msg.angular_velocity.y /= gyro_scale_.vector.y;
    imu_msg.angular_velocity.z /= gyro_scale_.vector.z;
  }

  imu_msg.angular_velocity_covariance[COV_IDX::X_X] =
    angular_velocity_stddev_xx_imu_link_ * angular_velocity_stddev_xx_imu_link_;
  imu_msg.angular_velocity_covariance[COV_IDX::Y_Y] =
    angular_velocity_stddev_yy_imu_link_ * angular_velocity_stddev_yy_imu_link_;
  imu_msg.angular_velocity_covariance[COV_IDX::Z_Z] =
    angular_velocity_stddev_zz_imu_link_ * angular_velocity_stddev_zz_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::X_X] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::Y_Y] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::Z_Z] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->get_latest_transform(imu_msg.header.frame_id, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_msg.header.frame_id).c_str());
    return;
  }

  sensor_msgs::msg::Imu imu_msg_base_link;
  imu_msg_base_link.header.stamp = imu_msg_ptr->header.stamp;
  imu_msg_base_link.header.frame_id = output_frame_;
  imu_msg_base_link.linear_acceleration =
    transform_vector3(imu_msg.linear_acceleration, *tf_imu2base_ptr);
  imu_msg_base_link.linear_acceleration_covariance =
    transform_covariance(imu_msg.linear_acceleration_covariance);
  imu_msg_base_link.angular_velocity =
    transform_vector3(imu_msg.angular_velocity, *tf_imu2base_ptr);
  imu_msg_base_link.angular_velocity_covariance =
    transform_covariance(imu_msg.angular_velocity_covariance);

  imu_pub_->publish(imu_msg_base_link);
}

void ImuCorrector::callback_bias(const Vector3Stamped::ConstSharedPtr bias_msg_ptr)
{
  // update gyro bias
  gyro_bias_ = *bias_msg_ptr;
}

void ImuCorrector::callback_scale(const Vector3Stamped::ConstSharedPtr scale_msg_ptr)
{
  // update gyro scale
  gyro_scale_ = *scale_msg_ptr;
}

}  // namespace autoware::imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::imu_corrector::ImuCorrector)
