// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__CAMERA_STREAMPETR__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__POSTPROCESS__POSTPROCESS_KERNEL_HPP_

#include "autoware/camera_streampetr/utils.hpp"

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <vector>

namespace autoware::camera_streampetr
{

class PostProcessingConfig
{
public:
  PostProcessingConfig(
    const int32_t num_classes, const float circle_nms_dist_threshold, const float score_threshold,
    const std::vector<double> & yaw_norm_thresholds, const int32_t num_proposals,
    const std::vector<float> & detection_range)
  : num_classes_(num_classes),
    num_proposals_(num_proposals),
    score_threshold_(score_threshold),
    circle_nms_dist_threshold_(circle_nms_dist_threshold),
    detection_range_(detection_range)
  {
    yaw_norm_thresholds_ =
      std::vector<float>(yaw_norm_thresholds.begin(), yaw_norm_thresholds.end());
    for (auto & yaw_norm_threshold : yaw_norm_thresholds_) {
      yaw_norm_threshold =
        (yaw_norm_threshold >= 0.0 && yaw_norm_threshold < 1.0) ? yaw_norm_threshold : 0.0;
    }
  }

  ///// NETWORK PARAMETERS /////
  int32_t num_classes_{5};
  int32_t num_proposals_{5400};

  // Post processing parameters
  float score_threshold_{0.1};
  float circle_nms_dist_threshold_{0.5};
  std::vector<float> yaw_norm_thresholds_{0.3, 0.3, 0.3, 0.3, 0.0};
  std::vector<float> detection_range_{-61.2, -61.2, -10.0, 61.2, 61.2, 10.0};
};

class PostprocessCuda
{
public:
  explicit PostprocessCuda(const PostProcessingConfig & config, cudaStream_t & stream);

  cudaError_t generateDetectedBoxes3D_launch(
    const float * cls_output, const float * box_output, std::vector<Box3D> & det_boxes3d,
    cudaStream_t stream);

private:
  PostProcessingConfig config_;
  cudaStream_t stream_;
  cudaStream_t stream_event_;
  cudaEvent_t start_, stop_;
};

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
