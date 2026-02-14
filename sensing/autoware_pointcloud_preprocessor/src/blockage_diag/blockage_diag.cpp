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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_detection.hpp"

#include <opencv2/imgproc.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

cv::Mat quantize_to_8u(const cv::Mat & image_16u)
{
  assert(image_16u.type() == CV_16UC1);
  auto dimensions = image_16u.size();

  cv::Mat image_8u(dimensions, CV_8UC1, cv::Scalar(0));
  // UINT16_MAX = 65535, UINT8_MAX = 255, so downscale by ceil(65535 / 255) = 256.
  image_16u.convertTo(image_8u, CV_8UC1, 1.0 / 256);
  return image_8u;
}

cv::Mat make_no_return_mask(const cv::Mat & depth_image)
{
  assert(depth_image.type() == CV_8UC1);
  auto dimensions = depth_image.size();

  cv::Mat no_return_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::inRange(depth_image, 0, 1, no_return_mask);

  return no_return_mask;
}

std::pair<cv::Mat, cv::Mat> segment_into_ground_and_sky(
  const cv::Mat & mask, int horizontal_ring_id)
{
  assert(mask.type() == CV_8UC1);
  auto dimensions = mask.size();

  cv::Mat sky_mask;
  mask(cv::Rect(0, 0, dimensions.width, horizontal_ring_id)).copyTo(sky_mask);

  cv::Mat ground_mask;
  mask(cv::Rect(0, horizontal_ring_id, dimensions.width, dimensions.height - horizontal_ring_id))
    .copyTo(ground_mask);

  return {ground_mask, sky_mask};
}

void validate_pointcloud_fields(const sensor_msgs::msg::PointCloud2 & input)
{
  std::vector<std::string> required_fields = {"channel", "azimuth", "distance"};

  for (const auto & field : input.fields) {
    auto it = std::find(required_fields.begin(), required_fields.end(), field.name);
    if (it != required_fields.end()) {
      required_fields.erase(it);
    }
  }

  bool has_all_required_fields = required_fields.empty();
  if (has_all_required_fields) {
    return;
  }

  std::string error_msg = "PointCloud2 missing required fields:";
  for (const auto & missing_field : required_fields) {
    error_msg += " " + missing_field;
  }
  throw std::runtime_error(error_msg);
}

}  // namespace autoware::pointcloud_preprocessor
