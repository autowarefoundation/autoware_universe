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

#ifndef AUTOWARE__BEVFUSION__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__BEVFUSION__POSTPROCESS__POSTPROCESS_KERNEL_HPP_

#include "autoware/bevfusion/bevfusion_config.hpp"
#include "autoware/bevfusion/postprocess/circle_nms_kernel.hpp"
#include "autoware/bevfusion/utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <memory>
#include <vector>

namespace autoware::bevfusion
{
using autoware::cuda_utils::CudaUniquePtr;

class PostprocessCuda
{
public:
  explicit PostprocessCuda(const BEVFusionConfig & config, cudaStream_t stream);

  cudaError_t generateDetectedBoxes3D_launch(
    const std::int64_t * label_pred_output, const float * bbox_pred_output,
    const float * score_output, std::vector<Box3D> & det_boxes3d, cudaStream_t stream);

private:
  BEVFusionConfig config_;
  cudaStream_t stream_;

  // For distance-based and class-based score thresholding
  CudaUniquePtr<float[]> distance_bin_upper_limits_d_ptr_{nullptr};
  CudaUniquePtr<float[]> score_thresholds_d_ptr_{nullptr};
  CudaUniquePtr<float[]> yaw_norm_thresholds_d_ptr_{nullptr};

  // For argsort
  // BBoxes score is used as a key to sort BBoxes in descending order.
  CudaUniquePtr<float[]> bboxes_score_d_ptr_{nullptr};
  CudaUniquePtr<float[]> sorted_bboxes_score_d_ptr_{nullptr};
  float highest_bbox_score_h_ =
    0.f;  // To save the highest score from bboxes, used to check if any valid bboxes

  // Memory to save sorted Bboxes
  CudaUniquePtr<Box3D[]> bboxes_d_ptr_{nullptr};
  CudaUniquePtr<Box3D[]> sorted_bboxes_d_ptr_{nullptr};

  std::size_t sort_workspace_size_ = 0;
  CudaUniquePtr<std::uint8_t[]> sort_workspace_d_{nullptr};
  std::size_t flagged_workspace_size_ = 0;
  CudaUniquePtr<std::uint8_t[]> flagged_workspace_d_{nullptr};

  // For Circle NMS
  std::unique_ptr<CircleNMS> circle_nms_ptr_{nullptr};
  CudaUniquePtr<std::size_t[]> num_selector_d_ptr_{nullptr};  // To save the number of selection
  CudaUniquePtr<Box3D[]> filtered_bboxes_d_ptr_{
    nullptr};  // To save bboxes after filtering by suppression masks.
  CudaUniquePtr<std::uint8_t[]> final_keep_mask_d_{
    nullptr};  // To save the final keep mask after NMS
  std::size_t num_final_det_boxes3d = 0;
};

}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
