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
#ifndef AUTOWARE__IMU_CORRECTOR__IMU_CORRECTOR_CORE_HPP_
#define AUTOWARE__IMU_CORRECTOR__IMU_CORRECTOR_CORE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware_utils/ros/msg_covariance.hpp>
#include <autoware_utils/ros/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware::imu_corrector
{
class ImuCorrector : public autoware::agnocast_wrapper::Node
{
  using COV_IDX = autoware_utils::xyz_covariance_index::XYZ_COV_IDX;
  using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;

public:
  explicit ImuCorrector(const rclcpp::NodeOptions & options);

private:
  void callback_imu(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Imu) & imu_msg_ptr);
  void callback_bias(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Vector3Stamped) & bias_msg_ptr);
  void callback_scale(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Vector3Stamped) & scale_msg_ptr);

  AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::Imu) imu_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(Vector3Stamped) gyro_bias_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(Vector3Stamped) gyro_scale_sub_;

  AUTOWARE_PUBLISHER_PTR(sensor_msgs::msg::Imu) imu_pub_;

  double angular_velocity_offset_x_imu_link_;
  double angular_velocity_offset_y_imu_link_;
  double angular_velocity_offset_z_imu_link_;

  double angular_velocity_stddev_xx_imu_link_;
  double angular_velocity_stddev_yy_imu_link_;
  double angular_velocity_stddev_zz_imu_link_;

  bool correct_for_static_bias_;
  bool correct_for_dynamic_bias_;
  bool correct_for_scale_;

  Vector3Stamped gyro_bias_;
  Vector3Stamped gyro_scale_;

  double accel_stddev_imu_link_;

  using TfListener = autoware_utils::TransformListenerT<
    autoware::agnocast_wrapper::Node, autoware::agnocast_wrapper::Buffer,
    autoware::agnocast_wrapper::TransformListener>;
  std::shared_ptr<TfListener> transform_listener_;

  std::string output_frame_;
};
}  // namespace autoware::imu_corrector

#endif  // AUTOWARE__IMU_CORRECTOR__IMU_CORRECTOR_CORE_HPP_
