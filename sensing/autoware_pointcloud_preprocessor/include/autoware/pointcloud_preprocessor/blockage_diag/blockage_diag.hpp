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
#include <string>
#include <utility>

namespace autoware::pointcloud_preprocessor
{

enum DiagnosticLevel { OK, WARN, ERROR, STALE };

struct DiagnosticAdditionalData
{
  std::string key;
  std::string value;
};

struct DiagnosticOutput
{
  DiagnosticLevel level;
  std::string message;
  std::vector<DiagnosticAdditionalData> additional_data;
};

struct MultiFrameDetectionAggregatorConfig
{
  int buffering_frames;    // Number of frames to buffer
  int buffering_interval;  // Interval between frames to buffer
};

/**
 * @brief A class to accumulate and aggregate detection masks over multiple frames.
 */
class MultiFrameDetectionAggregator
{
public:
  /**
   * @brief Constructor.
   * @param config Configuration for multi-frame detection visualization.
   */
  explicit MultiFrameDetectionAggregator(const MultiFrameDetectionAggregatorConfig & config);

  /**
   * @brief Update the time series mask with the current frame's mask.
   * @param mask The current mask to add. The data type is `CV_8UC1`.
   * @return cv::Mat The aggregated multi-frame result. The data type is `CV_8UC1`.
   */
  cv::Mat update(const cv::Mat & mask);

private:
  int frame_count_;
  int buffering_interval_;
  boost::circular_buffer<cv::Mat> mask_buffer_;
};

struct DustDetectionConfig
{
  float dust_ratio_threshold;
  int dust_kernel_size;
  int dust_count_threshold;
  int horizontal_ring_id;
};

struct DustDetectionResult
{
  float ground_dust_ratio = -1.0f;
  int dust_frame_count = 0;
  cv::Mat dust_mask;
};

/**
 * @brief A class to detect dust in point cloud data.
 */
class DustDetector
{
public:
  /**
   * @brief Constructor.
   * @param config Configuration for dust detection.
   */
  explicit DustDetector(const DustDetectionConfig & config);

  /**
   * @brief Compute dust diagnostics from a depth image.
   * @param depth_image_16u The input depth image. The data type is `CV_16UC1`.
   * @return DustDetectionResult The dust detection result.
   */
  DustDetectionResult compute_dust_diagnostics(const cv::Mat & depth_image_16u);

  /**
   * @brief Get diagnostic output for dust detection.
   * @return DiagnosticOutput The diagnostic output.
   */
  DiagnosticOutput get_dust_diagnostics_output() const;

private:
  DustDetectionConfig config_;
  DustDetectionResult result_;
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
