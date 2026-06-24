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
#include "autoware/bevfusion/utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/thrust_workspace_allocator.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include <cstdint>
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
  // Distance-based NMS over the first num_boxes3d entries of det_boxes3d_d_,
  // writing keep_mask_d_; reuses the pre-allocated scratch below.
  std::size_t circleNMS(std::size_t num_boxes3d, float distance_threshold, cudaStream_t stream);

  BEVFusionConfig config_;
  cudaStream_t stream_;

  // For distance-based and class-based score thresholding
  CudaUniquePtr<float[]> distance_bin_upper_limits_d_ptr_{nullptr};
  CudaUniquePtr<float[]> score_thresholds_d_ptr_{nullptr};

  // Thrust temporary workspace + persistent scratch, allocated once so the
  // per-frame reductions/sort/NMS perform no device or pinned-host allocation
  // and run on the node's stream (no default-stream barriers).
  autoware::cuda_utils::ThrustWorkspace thrust_workspace_;
  thrust::device_vector<Box3D> boxes3d_d_;
  thrust::device_vector<float> yaw_norm_thresholds_d_;
  thrust::device_vector<Box3D> det_boxes3d_d_;
  thrust::device_vector<bool> keep_mask_d_;
  thrust::device_vector<Box3D> final_det_boxes3d_d_;
  thrust::device_vector<std::uint64_t> nms_mask_d_;
  thrust::host_vector<std::uint64_t> nms_mask_h_;
  thrust::host_vector<bool> keep_mask_h_;
  std::vector<std::uint64_t> nms_remv_h_;
};

}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
