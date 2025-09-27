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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__DATA_TYPE_EIGEN_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__DATA_TYPE_EIGEN_HPP_

#include "autoware/calibration_status_classifier/data_type.hpp"

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{

/**
 * @brief Structure containing camera and LiDAR calibration information
 *
 * Contains all intrinsic and extrinsic parameters needed for LiDAR-camera calibration:
 * - Camera intrinsics: image dimensions, distortion coefficients, calibration matrices
 * - Transformation from camera to LiDAR coordinate frame
 * - Frame IDs for both sensors
 * - Flag indicating whether image undistortion is needed
 *
 * All matrices are stored in row-major order for compatibility with external libraries.
 */
struct CameraLidarInfo
{
  std::size_t height;
  std::size_t width;
  std::vector<double> d{std::vector<double>(dist_coeffs_size, 0.0)};
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> k{
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity()};
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> r{
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity()};
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> p{
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor>::Zero()};
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tf_camera_to_lidar{
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor>::Identity()};
  std::string lidar_frame_id;
  std::string camera_frame_id;
  bool to_undistort;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__DATA_TYPE_EIGEN_HPP_
