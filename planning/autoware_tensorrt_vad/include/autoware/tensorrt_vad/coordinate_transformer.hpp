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

#ifndef AUTOWARE_TENSORRT_VAD_COORDINATE_TRANSFORMER_HPP_
#define AUTOWARE_TENSORRT_VAD_COORDINATE_TRANSFORMER_HPP_

#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <string>
#include <tuple>

namespace autoware::tensorrt_vad::vad_interface {

/**
 * @brief CoordinateTransformer handles coordinate system transformations and TF lookups
 * 
 * This class manages:
 * - VAD coordinate system to Autoware coordinate system transformations
 * - TF buffer for dynamic coordinate transformations
 * - Transformation matrices (vad2base, base2vad)
 */
class CoordinateTransformer {
public:
  /**
   * @brief Constructor
   * @param vad2base Transformation matrix from VAD coordinate to base_link coordinate
   * @param base2vad Transformation matrix from base_link coordinate to VAD coordinate
   * @param tf_buffer Shared pointer to TF buffer for dynamic transformations
   */
  CoordinateTransformer(
    const Eigen::Matrix4f& vad2base, 
    const Eigen::Matrix4f& base2vad, 
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief Convert VAD coordinates to Autoware coordinates
   * @param vad_x X coordinate in VAD system
   * @param vad_y Y coordinate in VAD system
   * @param vad_z Z coordinate in VAD system
   * @return Tuple of (aw_x, aw_y, aw_z) in Autoware coordinate system
   */
  std::tuple<float, float, float> vad2aw_xyz(float vad_x, float vad_y, float vad_z) const;

  /**
   * @brief Convert Autoware coordinates to VAD coordinates
   * @param aw_x X coordinate in Autoware system
   * @param aw_y Y coordinate in Autoware system
   * @param aw_z Z coordinate in Autoware system
   * @return Tuple of (vad_x, vad_y, vad_z) in VAD coordinate system
   */
  std::tuple<float, float, float> aw2vad_xyz(float aw_x, float aw_y, float aw_z) const;

  /**
   * @brief Convert Autoware quaternion to VAD quaternion
   * @param q_aw Quaternion in Autoware coordinate system
   * @return Quaternion in VAD coordinate system
   */
  Eigen::Quaternionf aw2vad_quaternion(const Eigen::Quaternionf& q_aw) const;

  /**
   * @brief Lookup transformation from base_link to camera frame
   * @param source_frame Camera frame name (e.g., "camera0", "camera1")
   * @return Optional transformation matrix from base_link to camera
   */
  std::optional<Eigen::Matrix4f> lookup_base2cam(const std::string& source_frame) const;

private:
  const Eigen::Matrix4f vad2base_;  ///< Transformation matrix from VAD to base_link
  const Eigen::Matrix4f base2vad_;  ///< Transformation matrix from base_link to VAD
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  ///< TF buffer for dynamic transformations
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_COORDINATE_TRANSFORMER_HPP_
