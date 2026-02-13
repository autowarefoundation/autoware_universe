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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DETECTION_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DETECTION_HPP_

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include <opencv2/core/mat.hpp>

#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

struct BlockageDetectionConfig
{
  float blockage_ratio_threshold;
  int blockage_kernel;
  int blockage_count_threshold;
  int horizontal_ring_id;
  double horizontal_resolution;
  double angle_range_min_deg;
  double angle_range_max_deg;
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
  cv::Mat blockage_mask;
};

/**
 * @brief A class to detect blockage in point cloud data.
 */
class BlockageDetector
{
public:
  /**
   * @brief Constructor.
   * @param config Configuration for blockage detection.
   */
  explicit BlockageDetector(const BlockageDetectionConfig & config);

  /**
   * @brief Compute blockage diagnostics from a depth image.
   * @param depth_image_16u The input depth image. The data type is `CV_16UC1`.
   * @return BlockageDetectionResult The blockage detection result.
   */
  BlockageDetectionResult compute_blockage_diagnostics(const cv::Mat & depth_image_16u);

  /**
   * @brief Get diagnostic output for blockage detection.
   * @return DiagnosticOutput The diagnostic output.
   */
  DiagnosticOutput get_blockage_diagnostics_output() const;

private:
  /**
   * @brief Make a binary, cleaned blockage mask from the input no-return mask.
   *
   * @param no_return_mask A mask where 255 is no-return and 0 is return.
   * @return cv::Mat The blockage mask. The data type is `CV_8UC1`.
   */
  cv::Mat make_blockage_mask(const cv::Mat & no_return_mask) const;

  /**
   * @brief Get the ratio of non-zero pixels in a given mask.
   *
   * @param mask The input mask. The data type is `CV_8UC1`.
   * @return float The ratio of non-zero pixels.
   */
  static float get_nonzero_ratio(const cv::Mat & mask);

  /**
   * @brief Update the blockage info for a specific area (ground or sky).
   *
   * @param blockage_mask The blockage mask. The data type is `CV_8UC1`.
   * @param area_result Reference to the BlockageAreaResult to update.
   */
  void update_blockage_info(const cv::Mat & blockage_mask, BlockageAreaResult & area_result);

  BlockageDetectionConfig config_;
  BlockageDetectionResult result_;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DETECTION_HPP_
