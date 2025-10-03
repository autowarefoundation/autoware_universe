// Copyright 2025 TIER IV
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

#ifndef AUTOWARE__CAMERA_STREAMPETR__NETWORK__CAMERA_DATA_STORE_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__NETWORK__CAMERA_DATA_STORE_HPP_

#include "autoware/camera_streampetr/cuda_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
namespace autoware::camera_streampetr
{

constexpr float MAX_ALLOWED_CAMERA_TIME_DIFF =
  600;  // 10 minutes. Used to keep the timestamp within limit to prevent overflow in fp16 mode.

class CameraDataStore
{
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Tensor = cuda::Tensor;

public:
  CameraDataStore(
    rclcpp::Node * node, const int rois_number, const int image_height, const int image_width,
    const int anchor_camera_id, const bool is_distorted_image);
  ~CameraDataStore();
  void update_camera_image(
    const int camera_id, const Image::ConstSharedPtr & input_camera_image_msg);
  void update_camera_info(
    const int camera_id, const CameraInfo::ConstSharedPtr & input_camera_info_msg);
  bool check_if_all_camera_image_received() const;
  bool check_if_all_camera_info_received() const;
  float check_if_all_images_synced() const;
  float get_preprocess_time_ms() const;
  std::vector<float> get_camera_info_vector() const;
  std::shared_ptr<cuda::Tensor> get_image_input() const;

  std::vector<float> get_image_shape() const;
  float get_timestamp();
  std::vector<std::string> get_camera_link_names() const;
  void restart();
  void freeze_updates();
  void unfreeze_updates();

private:
  struct ImageProcessingParams
  {
    int original_height;
    int original_width;
    float resize;
    int newW;
    int newH;
    int crop_h;
    int crop_w;
    int start_x;
    int start_y;
    int camera_offset;
  };

  // Helper methods for update_camera_image
  ImageProcessingParams calculate_image_processing_params(
    const int camera_id, const Image::ConstSharedPtr & input_camera_image_msg) const;
  std::unique_ptr<Tensor> process_distorted_image(
    const int camera_id, const Image::ConstSharedPtr & input_camera_image_msg,
    ImageProcessingParams & params);
  std::unique_ptr<Tensor> process_regular_image(
    const Image::ConstSharedPtr & input_camera_image_msg, const ImageProcessingParams & params,
    const int camera_id);
  void update_metadata_and_timing(
    const int camera_id, const Image::ConstSharedPtr & input_camera_image_msg,
    const std::chrono::high_resolution_clock::time_point & start_time);
  void compute_undistortion_maps(const int camera_id);

  const size_t rois_number_;
  const int image_height_;
  const int image_width_;
  const int anchor_camera_id_;
  double start_timestamp_;
  float preprocess_time_ms_;
  const bool is_distorted_image_;

  rclcpp::Logger logger_;
  std::vector<CameraInfo::ConstSharedPtr> camera_info_list_;
  std::shared_ptr<Tensor> image_input_;
  std::shared_ptr<Tensor> image_input_mean_;
  std::shared_ptr<Tensor> image_input_std_;
  std::vector<double> camera_image_timestamp_;
  std::vector<std::string> camera_link_names_;
  std::vector<cudaStream_t> streams_;

  // GPU memory for undistortion maps
  std::vector<std::shared_ptr<Tensor>> undistort_map_x_gpu_;
  std::vector<std::shared_ptr<Tensor>> undistort_map_y_gpu_;
  std::vector<bool> undistortion_maps_computed_;

  // multithreading variables
  mutable std::mutex freeze_mutex_;
  mutable std::condition_variable freeze_cv_;
  bool is_frozen_;
  int active_updates_;
};

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__NETWORK__CAMERA_DATA_STORE_HPP_
