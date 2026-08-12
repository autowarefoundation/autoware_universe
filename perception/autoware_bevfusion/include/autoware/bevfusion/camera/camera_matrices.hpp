// Copyright 2026 TIER IV
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

#ifndef AUTOWARE__BEVFUSION__CAMERA__CAMERA_MATRICES_HPP_
#define AUTOWARE__BEVFUSION__CAMERA__CAMERA_MATRICES_HPP_

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

namespace autoware::bevfusion
{
using autoware::cuda_utils::CudaUniquePtr;

// Helper struct for camera matrices
struct CameraMatrices
{
  cv::Mat K;
  cv::Mat D;
  cv::Mat P;
  int map_width;
  int map_height;
  bool matrices_ready{false};

  // GPU Memory for undistortion maps and they only need to be computed once for each camera
  CudaUniquePtr<float[]> undistorted_map_x_d_{nullptr};
  CudaUniquePtr<float[]> undistorted_map_y_d_{nullptr};

  CameraMatrices();
  void update_camera_matrices(const sensor_msgs::msg::CameraInfo & camera_info);
  void compute_undistorted_map_x_y(cudaStream_t stream);
};
}  // namespace autoware::bevfusion
#endif  // AUTOWARE__BEVFUSION__CAMERA__CAMERA_MATRICES_HPP_
