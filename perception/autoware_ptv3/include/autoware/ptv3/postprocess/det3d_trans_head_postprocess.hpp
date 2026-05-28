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

#ifndef AUTOWARE__PTV3__POSTPROCESS__DET3D_TRANS_HEAD_POSTPROCESS_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__DET3D_TRANS_HEAD_POSTPROCESS_HPP_

#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>

namespace autoware::ptv3
{

/**
 * @brief Launch the TransHead proposal decoder.
 *
 * Each proposal writes to its matching slot in out_boxes. Rejected proposals get score 0 and are
 * compacted away by Det3dTransHeadPostprocess::process().
 *
 * @param query_heatmap_score_d  Device pointer to query heatmap scores [num_proposals x
 *                               num_classes].
 * @param query_labels_d         Device pointer to query class labels [num_proposals].
 * @param heatmap_d              Device pointer to dense heatmap tensor.
 * @param center_d               Device pointer to center regression tensor.
 * @param height_d               Device pointer to height regression tensor.
 * @param dim_d                  Device pointer to dimension regression tensor.
 * @param rot_d                  Device pointer to rotation regression tensor.
 * @param vel_d                  Device pointer to velocity regression tensor.
 * @param num_proposals          Number of query proposals.
 * @param num_classes            Number of detection classes.
 * @param bbox_downsample_factor Spatial downsampling factor for BEV grid.
 * @param bbox_voxel_x_size      BEV voxel size in x direction.
 * @param bbox_voxel_y_size      BEV voxel size in y direction.
 * @param min_x_range            Minimum x coordinate of the point cloud range.
 * @param min_y_range            Minimum y coordinate of the point cloud range.
 * @param post_center_range_d    Device pointer to 6-element post-center filter range.
 * @param score_thresholds_d     Device pointer to per-class per-distance-bin score thresholds.
 * @param dist_bin_limits_d      Device pointer to distance bin upper limits.
 * @param num_dist_bins          Number of distance bins.
 * @param yaw_norm_thresholds_d  Device pointer to per-class yaw normalization thresholds.
 * @param has_twist              Whether velocity tensor is present.
 * @param out_boxes_d            Output: device pointer to Box3D array [num_proposals].
 * @param stream                 CUDA stream for this call.
 */
template <typename FloatType, typename IntType>
void launch_decode_trans_head_to_boxes3d(
  const FloatType * query_heatmap_score_d, const IntType * query_labels_d,
  const FloatType * heatmap_d, const FloatType * center_d, const FloatType * height_d,
  const FloatType * dim_d, const FloatType * rot_d, const FloatType * vel_d, int num_proposals,
  int num_classes, float bbox_downsample_factor, float bbox_voxel_x_size, float bbox_voxel_y_size,
  float min_x_range, float min_y_range, const float * post_center_range_d,
  const float * score_thresholds_d, const float * dist_bin_limits_d, int num_dist_bins,
  const float * yaw_norm_thresholds_d, bool has_twist, Box3D * out_boxes_d, cudaStream_t stream);

/**
 * @brief GPU postprocessor for the TransHead 3D detection head.
 *
 * Decodes proposals, compacts kept boxes, and sorts them by score on GPU.
 */
class Det3dTransHeadPostprocess
{
public:
  /**
   * @brief Allocate decode buffers and copy threshold tables to the device.
   *
   * @param config Runtime configuration for detection thresholds and grid sizes.
   * @param stream CUDA stream used for all kernel launches.
   */
  Det3dTransHeadPostprocess(const PTv3Config & config, cudaStream_t stream);

  /**
   * @brief Decode proposals, compact kept boxes, and sort by score.
   *
   * FloatType is float or __half. IntType is int32_t or int64_t.
   *
   * @param query_heatmap_score_d Device pointer to query heatmap scores.
   * @param query_labels_d        Device pointer to query class labels.
   * @param heatmap_d             Device pointer to dense heatmap tensor.
   * @param center_d              Device pointer to center regression tensor.
   * @param height_d              Device pointer to height regression tensor.
   * @param dim_d                 Device pointer to dimension regression tensor.
   * @param rot_d                 Device pointer to rotation regression tensor.
   * @param vel_d                 Device pointer to velocity regression tensor.
   * @param stream                CUDA stream for this call.
   * @return cudaSuccess on success, or the last CUDA error.
   */
  template <typename FloatType, typename IntType>
  cudaError_t process(
    const FloatType * query_heatmap_score_d, const IntType * query_labels_d,
    const FloatType * heatmap_d, const FloatType * center_d, const FloatType * height_d,
    const FloatType * dim_d, const FloatType * rot_d, const FloatType * vel_d, cudaStream_t stream);

  /**
   * @brief Return device pointer to the score-sorted boxes.
   *
   * Valid until the next call to process().
   *
   * @return Const device pointer to Box3D array with num_boxes() elements.
   */
  [[nodiscard]] const Box3D * device_boxes() const
  {
    return thrust::raw_pointer_cast(passing_boxes_d_.data());
  }

  /**
   * @brief Return the number of boxes from the last process() call.
   *
   * @return Number of decoded boxes kept after filtering.
   */
  [[nodiscard]] std::size_t num_boxes() const { return num_boxes_; }

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

#endif  // AUTOWARE__PTV3__POSTPROCESS__DET3D_TRANS_HEAD_POSTPROCESS_HPP_
