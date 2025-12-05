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

#include "../src/input_converter/image_converter.hpp"

#include <opencv2/opencv.hpp>

#include <optional>
#include <stdexcept>
#include <vector>

namespace autoware::tensorrt_vad::vad_interface
{

namespace
{
const rclcpp::Logger & vad_logger()
{
  static const rclcpp::Logger logger = rclcpp::get_logger("autoware_tensorrt_vad");
  return logger;
}

const rclcpp::Clock::SharedPtr & vad_clock()
{
  static const rclcpp::Clock::SharedPtr clock = rclcpp::Clock::make_shared();
  return clock;
}

std::optional<cv::Mat> to_bgr_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, const int32_t camera_idx)
{
  if (image_msg->encoding == "bgr8") {
    return cv::Mat(
      image_msg->height, image_msg->width, CV_8UC3, const_cast<uint8_t *>(image_msg->data.data()),
      image_msg->step);
  }

  if (image_msg->encoding == "bgra8") {
    cv::Mat bgra_img(
      image_msg->height, image_msg->width, CV_8UC4, const_cast<uint8_t *>(image_msg->data.data()),
      image_msg->step);
    cv::Mat bgr_img;
    cv::cvtColor(bgra_img, bgr_img, cv::COLOR_BGRA2BGR);
    return bgr_img;
  }

  RCLCPP_ERROR_THROTTLE(
    vad_logger(), *vad_clock(), 5000, "Unsupported image encoding: %s for camera %d",
    image_msg->encoding.c_str(), camera_idx);
  return std::nullopt;
}
}  // namespace

InputImageConverter::InputImageConverter(
  const CoordinateTransformer & coordinate_transformer, const VadInterfaceConfig & config)
: Converter(coordinate_transformer, config)
{
}

CameraImagesData InputImageConverter::process_image(
  const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images) const
{
  std::vector<cv::Mat> processed_images;
  const int32_t num_cameras = static_cast<int32_t>(images.size());
  processed_images.resize(num_cameras);

  // Process each camera image (CARLA: identity mapping, camera order matches VAD training
  // order)
  for (int32_t camera_idx = 0; camera_idx < num_cameras; ++camera_idx) {
    const auto & image_msg = images[camera_idx];

    // Skip if image is not available
    if (!image_msg) {
      RCLCPP_WARN_THROTTLE(
        vad_logger(), *vad_clock(), 5000, "Image for camera %d is null, skipping", camera_idx);
      continue;
    }

    const auto bgr_img_opt = to_bgr_image(image_msg, camera_idx);
    if (!bgr_img_opt.has_value()) {
      continue;
    }
    const cv::Mat & bgr_img = bgr_img_opt.value();

    if (bgr_img.empty()) {
      RCLCPP_ERROR_THROTTLE(
        vad_logger(), *vad_clock(), 5000, "Failed to decode image data: %d", camera_idx);
      continue;
    }

    // Clone the data to ensure memory continuity
    cv::Mat processed_img = bgr_img.clone();

    // Store directly (identity mapping for CARLA)
    processed_images[camera_idx] = processed_img;
  }

  return processed_images;
}

}  // namespace autoware::tensorrt_vad::vad_interface
