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

#ifndef AUTOWARE__CAMERA_STREAMPETR__IMAGE_UTILS_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__IMAGE_UTILS_HPP_

#include <opencv2/opencv.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::camera_streampetr
{

void decompress_image(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & input_compressed_image_msg,
  cv_bridge::CvImagePtr & cv_ptr, const int decompression_downsample);

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__IMAGE_UTILS_HPP_
