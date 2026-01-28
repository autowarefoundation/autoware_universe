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

#include <opencv2/imgproc.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

MultiFrameDetectionAggregator::MultiFrameDetectionAggregator(
  const MultiFrameDetectionAggregatorConfig & config)
: frame_count_(0), buffering_interval_(config.buffering_interval)
{
  mask_buffer_.set_capacity(config.buffering_frames);
}

cv::Mat MultiFrameDetectionAggregator::update(const cv::Mat & mask)
{
  if (buffering_interval_ == 0) {
    return mask.clone();
  }

  assert(mask.type() == CV_8UC1);
  auto dimensions = mask.size();

  cv::Mat time_series_result(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat time_series_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat binarized_mask(dimensions, CV_8UC1, cv::Scalar(0));

  binarized_mask = mask / 255;
  if (frame_count_ >= buffering_interval_) {
    mask_buffer_.push_back(binarized_mask);
    frame_count_ = 0;
  } else {
    frame_count_++;
  }

  for (const auto & binary_mask : mask_buffer_) {
    time_series_mask += binary_mask;
  }

  cv::inRange(time_series_mask, mask_buffer_.size() - 1, mask_buffer_.size(), time_series_result);

  return time_series_result;
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
