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

#include "autoware/bevfusion/camera/camera_matrices.hpp"

#include <opencv2/opencv.hpp>

namespace autoware::bevfusion
{

CameraMatrices::CameraMatrices()
{
  K = cv::Mat::zeros(3, 3, CV_64F);
  D = cv::Mat::zeros(1, 5, CV_64F);
  P = cv::Mat::zeros(3, 4, CV_64F);
  map_width = 0;
  map_height = 0;

  matrices_ready = false;
}

void CameraMatrices::update_camera_matrices(const sensor_msgs::msg::CameraInfo & camera_info)
{
  // Create camera matrix K from camera_info
  K =
    (cv::Mat_<double>(3, 3) << camera_info.k[0], camera_info.k[1], camera_info.k[2],
     camera_info.k[3], camera_info.k[4], camera_info.k[5], camera_info.k[6], camera_info.k[7],
     camera_info.k[8]);

  // Create distortion coefficients matrix D from camera_info
  const auto & d_vec = camera_info.d;
  D = cv::Mat(1, static_cast<int>(d_vec.size()), CV_64F);
  for (size_t i = 0; i < d_vec.size(); ++i) {
    D.at<double>(0, static_cast<int>(i)) = d_vec[i];
  }

  // Create projection matrix P from camera_info (first 3x3 part)
  P =
    (cv::Mat_<double>(3, 3) << camera_info.p[0], camera_info.p[1], camera_info.p[2],
     camera_info.p[4], camera_info.p[5], camera_info.p[6], camera_info.p[8], camera_info.p[9],
     camera_info.p[10]);

  // Use full resolution for both input and output (no downsampling)
  map_width = camera_info.width;
  map_height = camera_info.height;
}

void CameraMatrices::compute_undistorted_map_x_y(cudaStream_t stream)
{
  undistorted_map_x_d_ = autoware::cuda_utils::make_unique<float[]>(map_width * map_height);
  undistorted_map_y_d_ = autoware::cuda_utils::make_unique<float[]>(map_width * map_height);

  // Compute undistortion maps for full resolution
  cv::Mat undistort_map_x, undistort_map_y;
  cv::initUndistortRectifyMap(
    K, D, cv::Mat(), P, cv::Size(map_width, map_height), CV_32FC1, undistort_map_x,
    undistort_map_y);

  // Always sync the data to the GPU since it only runs once for every camera
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    undistorted_map_x_d_.get(), undistort_map_x.data, map_width * map_height * sizeof(float),
    cudaMemcpyHostToDevice, stream));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    undistorted_map_y_d_.get(), undistort_map_y.data, map_width * map_height * sizeof(float),
    cudaMemcpyHostToDevice, stream));

  matrices_ready = true;
}

}  // namespace autoware::bevfusion
