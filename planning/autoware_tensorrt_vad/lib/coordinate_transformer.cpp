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

namespace autoware::tensorrt_vad::vad_interface
{

CoordinateTransformer::CoordinateTransformer(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: tf_buffer_(tf_buffer)
{
}

std::optional<Eigen::Matrix4f> CoordinateTransformer::lookup_base2cam(
  const std::string & source_frame) const
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
      lookup_result.transform.rotation.w, lookup_result.transform.rotation.x,
      lookup_result.transform.rotation.y, lookup_result.transform.rotation.z);
    transform_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();

    // calculate the inverse transformation
    Eigen::Matrix4f transform_matrix_inverse = transform_matrix.inverse();

    return transform_matrix_inverse;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000,
      "Failed to get TF transformation: %s -> %s. Reason: %s", source_frame.c_str(),
      target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

}  // namespace autoware::tensorrt_vad::vad_interface
