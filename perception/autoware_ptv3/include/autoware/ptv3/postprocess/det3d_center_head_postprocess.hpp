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

#ifndef AUTOWARE__PTV3__POSTPROCESS__DET3D_CENTER_HEAD_POSTPROCESS_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__DET3D_CENTER_HEAD_POSTPROCESS_HPP_

#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>

namespace autoware::ptv3
{

/**
 * @brief GPU postprocessor for the CenterHead 3D detection head.
 *
 * Decodes BEV feature maps into a score-filtered and score-sorted Box3D buffer on device.
 */
class Det3dCenterHeadPostprocess
{
public:
  /**
   * @brief Allocate decode buffers and copy threshold tables to the device.
   *
   * @param config Runtime configuration for detection thresholds and grid sizes.
   * @param stream CUDA stream used for all kernel launches.
   */
  Det3dCenterHeadPostprocess(const PTv3Config & config, cudaStream_t stream);

  /**
   * @brief Decode BEV feature maps and apply score and yaw filters.
   *
   * After return, num_boxes() is the number of boxes available at device_boxes().
   *
   * @param out_heatmap  Device pointer to heatmap tensor [det_grid x num_classes].
   * @param out_reg      Device pointer to regression tensor [det_grid x reg_size].
   * @param out_height   Device pointer to height tensor [det_grid x height_size].
   * @param out_dim      Device pointer to dimension tensor [det_grid x dim_size].
   * @param out_rot      Device pointer to rotation tensor [det_grid x rot_size].
   * @param out_vel      Device pointer to velocity tensor [det_grid x vel_size].
   * @param stream       CUDA stream for this call.
   * @return cudaSuccess on success, or the last CUDA error.
   */
  template <typename FloatType>
  cudaError_t process(
    const FloatType * out_heatmap, const FloatType * out_reg, const FloatType * out_height,
    const FloatType * out_dim, const FloatType * out_rot, const FloatType * out_vel,
    cudaStream_t stream);

  /**
   * @brief Return device pointer to the score-sorted boxes.
   *
   * Valid until the next call to process().
   *
   * @return Const device pointer to Box3D array with num_boxes() elements.
   */
  [[nodiscard]] const Box3D * device_boxes() const;

  /**
   * @brief Return the number of boxes from the last process() call.
   *
   * @return Number of decoded boxes kept after filtering.
   */
  [[nodiscard]] std::size_t num_boxes() const { return num_boxes_; }

private:
  PTv3Config config_;
  std::size_t max_boxes_;
  std::size_t num_boxes_{0};

  thrust::device_vector<Box3D> raw_boxes_d_;
  thrust::device_vector<Box3D> passing_boxes_d_;
  thrust::device_vector<float> yaw_norm_thresholds_d_;

  autoware::cuda_utils::CudaUniquePtr<float[]> distance_bin_upper_limits_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> score_thresholds_d_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__DET3D_CENTER_HEAD_POSTPROCESS_HPP_
