// Copyright 2025 TIER IV.
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

#include "autoware/tensorrt_vad/coordinate_transformer.hpp"
#include <tf2/exceptions.h>

namespace autoware::tensorrt_vad::vad_interface {

CoordinateTransformer::CoordinateTransformer(
  const Eigen::Matrix4f& vad2base, 
  const Eigen::Matrix4f& base2vad, 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  : vad2base_(vad2base), base2vad_(base2vad), tf_buffer_(tf_buffer)
{
}

std::tuple<float, float, float> CoordinateTransformer::vad2aw_xyz(
  float vad_x, float vad_y, float vad_z) const
{
  // Convert VAD base_link coordinates [x, y, z] to Autoware(base_link) coordinates
  Eigen::Vector4f vad_xyz(vad_x, vad_y, vad_z, 1.0f);
  Eigen::Vector4f aw_xyz = vad2base_ * vad_xyz;
  return {aw_xyz[0], aw_xyz[1], aw_xyz[2]};
}

std::tuple<float, float, float> CoordinateTransformer::aw2vad_xyz(
  float aw_x, float aw_y, float aw_z) const
{
  // Convert Autoware(base_link) coordinates [x, y, z] to VAD base_link coordinates
  Eigen::Vector4f aw_xyz(aw_x, aw_y, aw_z, 1.0f);
  Eigen::Vector4f vad_xyz = base2vad_ * aw_xyz;
  return {vad_xyz[0], vad_xyz[1], vad_xyz[2]};
}

Eigen::Quaternionf CoordinateTransformer::aw2vad_quaternion(const Eigen::Quaternionf& q_aw) const
{
  // base2vad_ rotation part to quaternion conversion
  Eigen::Matrix3f rot = base2vad_.block<3,3>(0,0);
  Eigen::Quaternionf q_v2a(rot); // base_link→vad rotation
  Eigen::Quaternionf q_v2a_inv = q_v2a.conjugate(); // For unit quaternion, inverse = conjugate
  // q_vad = q_v2a * q_aw * q_v2a_inv
  return q_v2a * q_aw * q_v2a_inv;
}

std::optional<Eigen::Matrix4f> CoordinateTransformer::lookup_base2cam(const std::string& source_frame) const
{
  std::string target_frame = "base_link";

  try {
    geometry_msgs::msg::TransformStamped lookup_result =
        tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    
    // Translation
    transform_matrix(0, 3) = lookup_result.transform.translation.x;
    transform_matrix(1, 3) = lookup_result.transform.translation.y;
    transform_matrix(2, 3) = lookup_result.transform.translation.z;
    
    // Rotation (quaternion to rotation matrix conversion)
    Eigen::Quaternionf q(
        lookup_result.transform.rotation.w,
        lookup_result.transform.rotation.x,
        lookup_result.transform.rotation.y,
        lookup_result.transform.rotation.z);
    transform_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();

    // calculate the inverse transformation
    Eigen::Matrix4f transform_matrix_inverse = transform_matrix.inverse();

    return transform_matrix_inverse;

  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, 
                 "Failed to get TF transformation: %s -> %s. Reason: %s",
                 source_frame.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

} // namespace autoware::tensorrt_vad::vad_interface
