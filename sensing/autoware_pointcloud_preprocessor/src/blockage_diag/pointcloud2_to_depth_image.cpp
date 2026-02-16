// Copyright 2026 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/blockage_diag/pointcloud2_to_depth_image.hpp"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::pointcloud_preprocessor
{

namespace pointcloud2_to_depth_image
{

// PointCloud2ToDepthImage class implementation
PointCloud2ToDepthImage::PointCloud2ToDepthImage(const ConverterConfig & config) : config_(config)
{
}

std::optional<int> PointCloud2ToDepthImage::get_horizontal_bin(double azimuth_deg) const
{
  double min_deg = config_.horizontal.angle_range_min_deg;
  double max_deg = config_.horizontal.angle_range_max_deg;
  bool fov_wraps_around = (min_deg > max_deg);
  if (fov_wraps_around) {
    azimuth_deg += 360.0;
    max_deg += 360.0;
  }

  bool azimuth_is_in_fov = ((azimuth_deg > min_deg) && (azimuth_deg <= max_deg));
  if (!azimuth_is_in_fov) {
    return std::nullopt;
  }

  return {static_cast<int>((azimuth_deg - min_deg) / config_.horizontal.horizontal_resolution)};
}

std::optional<int> PointCloud2ToDepthImage::get_vertical_bin(uint16_t channel) const
{
  if (channel >= config_.vertical.vertical_bins) {
    return std::nullopt;
  }

  if (config_.vertical.is_channel_order_top2down) {
    return {channel};
  }

  return {config_.vertical.vertical_bins - channel - 1};
}

cv::Mat PointCloud2ToDepthImage::make_normalized_depth_image(
  const sensor_msgs::msg::PointCloud2 & input) const
{
  // Calculate dimensions
  auto horizontal_bins = get_horizontal_bin(config_.horizontal.angle_range_max_deg);
  if (!horizontal_bins) {
    throw std::logic_error("Horizontal bin is not valid");
  }

  cv::Size dimensions(*horizontal_bins, config_.vertical.vertical_bins);
  cv::Mat depth_image(dimensions, CV_16UC1, cv::Scalar(0));

  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_channel(input, "channel");
  sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(input, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> iter_distance(input, "distance");

  for (; iter_channel != iter_channel.end(); ++iter_channel, ++iter_azimuth, ++iter_distance) {
    uint16_t channel = *iter_channel;
    float azimuth = *iter_azimuth;
    float distance = *iter_distance;

    auto vertical_bin = get_vertical_bin(channel);
    if (!vertical_bin) {
      throw std::runtime_error("Vertical bin is not valid");
    }

    double azimuth_deg = azimuth * (180.0 / M_PI);
    auto horizontal_bin = get_horizontal_bin(azimuth_deg);
    if (!horizontal_bin) {
      continue;
    }

    // Max distance is mapped to 0, zero-distance is mapped to UINT16_MAX.
    uint16_t normalized_depth =
      UINT16_MAX * (1.0 - std::min(distance / config_.max_distance_range, 1.0));
    depth_image.at<uint16_t>(*vertical_bin, *horizontal_bin) = normalized_depth;
  }

  return depth_image;
}

}  // namespace pointcloud2_to_depth_image

}  // namespace autoware::pointcloud_preprocessor
