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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_

#include <opencv2/core/mat.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/circular_buffer.hpp>

#include <vector>

namespace autoware::pointcloud_preprocessor
{

struct BlockageDetectionConfig
{
  float blockage_ratio_threshold;
  int blockage_kernel;
  int blockage_count_threshold;
};

struct BlockageAreaResult
{
  float blockage_ratio = -1.0f;
  int blockage_count = 0;
  float blockage_start_deg = 0.0f;
  float blockage_end_deg = 0.0f;
};

struct BlockageDetectionResult
{
  BlockageAreaResult ground;
  BlockageAreaResult sky;
};

struct DetectionVisualizeData
{
  int frame_count = 0;
  int buffering_interval = 0;
  boost::circular_buffer<cv::Mat> mask_buffer{1};
};

struct DustDetectionConfig
{
  float dust_ratio_threshold;
  int dust_kernel_size;
  int dust_count_threshold;
};

struct DustDetectionResult
{
  float ground_dust_ratio = -1.0f;
  int dust_frame_count = 0;
};

/**
 * @brief Validate that the PointCloud2 message has required fields for blockage diagnosis.
 *
 * @param input The input point cloud.
 * @throws std::runtime_error if any required field is missing.
 */
void validate_pointcloud_fields(const sensor_msgs::msg::PointCloud2 & input);

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_
