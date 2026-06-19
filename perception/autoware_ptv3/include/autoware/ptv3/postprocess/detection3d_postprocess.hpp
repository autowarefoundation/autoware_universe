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

#ifndef AUTOWARE__PTV3__POSTPROCESS__DETECTION3D_POSTPROCESS_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__DETECTION3D_POSTPROCESS_HPP_

#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>

#include <cstdint>

namespace autoware::ptv3
{

/**
 * @brief GPU postprocessor for the Detection3D head.
 *
 * Decodes proposals, compacts kept boxes, and sorts them by score on GPU.
 */
class Detection3DPostprocess
{
public:
  /**
   * @brief Allocate decode buffers and copy threshold tables to the device.
   *
   * @param config Runtime configuration for detection thresholds and grid sizes.
   * @param stream CUDA stream used for all kernel launches.
   */
  Detection3DPostprocess(const PTv3Config & config, cudaStream_t stream);

  /**
   * @brief Decode proposals, compact kept boxes, and sort by score.
   *
   * @param query_heatmap_score_d Device pointer to query heatmap scores.
   * @param query_labels_d        Device pointer to query class labels.
   * @param heatmap_d             Device pointer to per-query heatmap tensor.
   * @param center_d              Device pointer to center regression tensor.
   * @param height_d              Device pointer to height regression tensor.
   * @param dim_d                 Device pointer to dimension regression tensor.
   * @param rot_d                 Device pointer to rotation regression tensor.
   * @param vel_d                 Device pointer to velocity regression tensor.
   * @param stream                CUDA stream for this call.
   * @return cudaSuccess on success, or the last CUDA error.
   */
  cudaError_t process(
    const float * query_heatmap_score_d, const std::int64_t * query_labels_d,
    const float * heatmap_d, const float * center_d, const float * height_d, const float * dim_d,
    const float * rot_d, const float * vel_d, cudaStream_t stream);

  /**
   * @brief Return device pointer to the score-sorted boxes.
   *
   * Valid until the next call to process().
   *
   * @return Const device pointer to Box3D array with numBoxes() elements.
   */
  [[nodiscard]] const Box3D * deviceBoxes() const
  {
    return thrust::raw_pointer_cast(passing_boxes_d_.data());
  }

  /**
   * @brief Return the number of boxes from the last process() call.
   *
   * @return Number of decoded boxes kept after filtering.
   */
  [[nodiscard]] std::size_t numBoxes() const { return num_boxes_; }

private:
  PTv3Config config_;

  thrust::device_vector<Box3D> raw_boxes_d_;
  thrust::device_vector<Box3D> passing_boxes_d_;
  thrust::device_vector<float> yaw_norm_thresholds_d_;

  autoware::cuda_utils::CudaUniquePtr<float[]> post_center_range_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> score_thresholds_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> dist_bin_limits_d_{nullptr};

  std::size_t num_boxes_{0};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__DETECTION3D_POSTPROCESS_HPP_
