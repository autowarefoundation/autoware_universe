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

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <vector>

namespace autoware::camera_streampetr
{

class PostProcessingConfig
{
public:
  PostProcessingConfig(
    const int32_t num_classes, const float circle_nms_dist_threshold,
    const std::vector<double> & score_thresholds, const std::vector<double> & yaw_norm_thresholds,
    const int32_t num_proposals, const std::vector<float> & detection_range)
  : num_classes_(num_classes),
    num_proposals_(num_proposals),
    circle_nms_dist_threshold_(circle_nms_dist_threshold),
    detection_range_(detection_range)
  {
    if (static_cast<int32_t>(yaw_norm_thresholds.size()) != num_classes) {
      throw std::invalid_argument(
        "yaw_norm_thresholds size (" + std::to_string(yaw_norm_thresholds.size()) +
        ") must equal num_classes (" + std::to_string(num_classes) + ")");
    }
    if (static_cast<int32_t>(score_thresholds.size()) != num_classes) {
      throw std::invalid_argument(
        "score_thresholds size (" + std::to_string(score_thresholds.size()) +
        ") must equal num_classes (" + std::to_string(num_classes) + ")");
    }

    yaw_norm_thresholds_ =
      std::vector<float>(yaw_norm_thresholds.begin(), yaw_norm_thresholds.end());
    for (auto & yaw_norm_threshold : yaw_norm_thresholds_) {
      yaw_norm_threshold =
        (yaw_norm_threshold >= 0.0 && yaw_norm_threshold < 1.0) ? yaw_norm_threshold : 0.0;
    }
    score_thresholds_ = std::vector<float>(score_thresholds.begin(), score_thresholds.end());
  }

  ///// NETWORK PARAMETERS /////
  int32_t num_classes_{5};
  int32_t num_proposals_{5400};

  // Post processing parameters
  std::vector<float> score_thresholds_{0.1, 0.1, 0.1, 0.1, 0.1};
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

  // Pre-allocated device arrays to avoid repeated allocations
  autoware::cuda_utils::CudaUniquePtr<float[]> yaw_norm_thresholds_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> score_thresholds_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> detection_range_d_;
  autoware::cuda_utils::CudaUniquePtr<Box3D[]> boxes3d_d_;
};

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
