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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__DUST_DETECTION_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__DUST_DETECTION_HPP_

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include <opencv2/core/mat.hpp>

namespace autoware::pointcloud_preprocessor
{

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

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__DUST_DETECTION_HPP_
