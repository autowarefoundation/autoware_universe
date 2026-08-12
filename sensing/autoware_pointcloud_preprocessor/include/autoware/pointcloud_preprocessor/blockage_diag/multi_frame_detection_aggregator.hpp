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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__MULTI_FRAME_DETECTION_AGGREGATOR_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__MULTI_FRAME_DETECTION_AGGREGATOR_HPP_

#include <opencv2/core/mat.hpp>

#include <boost/circular_buffer.hpp>

namespace autoware::pointcloud_preprocessor
{

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

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__MULTI_FRAME_DETECTION_AGGREGATOR_HPP_
