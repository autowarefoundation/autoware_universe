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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__POINTCLOUD2_TO_DEPTH_IMAGE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__POINTCLOUD2_TO_DEPTH_IMAGE_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <optional>

namespace autoware::pointcloud_preprocessor
{

namespace pointcloud2_to_depth_image
{

struct HorizontalConfig
{
  double angle_range_min_deg;
  double angle_range_max_deg;
  double horizontal_resolution;
};

struct VerticalConfig
{
  int vertical_bins;
  bool is_channel_order_top2down;
};

struct ConverterConfig
{
  HorizontalConfig horizontal;
  VerticalConfig vertical;
  double max_distance_range;
};

/**
 * @brief Class to convert PointCloud2 to normalized depth image.
 *
 * This class processes point cloud data and generates a downsampled depth image.
 * The configuration parameters are set during construction and remain constant.
 */
class PointCloud2ToDepthImage
{
public:
  /**
   * @brief Construct a new PointCloud2ToDepthImage object.
   *
   * @param config The configuration for the depth image conversion.
   */
  explicit PointCloud2ToDepthImage(const ConverterConfig & config);

  /**
   * @brief Make a downsampled depth image from the input point cloud, normalized to 0-65535.
   *
   * Close depth values are mapped to higher values, far depth values are mapped to lower values.
   * The `max_distance_range` is mapped to 0, and a LiDAR distance of 0 is mapped to UINT16_MAX.
   *
   * @param input The input point cloud.
   * @return cv::Mat The normalized depth image. The data type is `CV_16UC1`.
   */
  cv::Mat make_normalized_depth_image(const sensor_msgs::msg::PointCloud2 & input) const;

private:
  /**
   * @brief Get the horizontal bin index of the given azimuth, if within the FoV.
   *
   * If the FoV wraps around, the azimuth is adjusted to be within the FoV.
   * The bin is calculated as `(azimuth_deg - min_deg) / horizontal_resolution_` and any
   * azimuth for which `min_deg < azimuth_deg <= max_deg` is valid.
   *
   * @param azimuth_deg The azimuth to get the bin index for.
   * @return std::optional<int> The bin index if valid, otherwise `std::nullopt`.
   */
  std::optional<int> get_horizontal_bin(double azimuth_deg) const;

  /**
   * @brief Get the vertical bin index of the given channel, if within the FoV.
   *
   * Vertical bins and channels are usually equivalent, apart from the 0-based index of bins.
   * If `is_channel_order_top2down_` is `false`, the bin order is reversed compared to the channel
   * order.
   *
   * @param channel The channel to get the bin index for.
   * @return std::optional<int> The bin index if valid, otherwise `std::nullopt`.
   */
  std::optional<int> get_vertical_bin(uint16_t channel) const;

  ConverterConfig config_;
};

}  // namespace pointcloud2_to_depth_image

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__POINTCLOUD2_TO_DEPTH_IMAGE_HPP_
