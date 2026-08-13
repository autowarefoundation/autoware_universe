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

#include "autoware/bevfusion/postprocess/circle_nms_kernel.hpp"
#include "autoware/bevfusion/postprocess/postprocess_kernel.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>
#include <cub/cub.cuh>
#include <cub/device/device_radix_sort.cuh>

#include <cstdint>

namespace autoware::bevfusion
{

__global__ void generateBoxes3D_kernel(
  const std::int64_t * __restrict__ label_pred_output, const float * __restrict__ bbox_pred_output,
  const float * __restrict__ score_output, const float voxel_size_x, const float voxel_size_y,
  const float min_x_range, const float min_y_range, const int num_proposals,
  const float out_size_factor, const float * __restrict__ yaw_norm_thresholds, const int class_size,
  const float * distance_bin_upper_limits, const float * score_thresholds,
  const std::size_t num_distance_bin_upper_limits, Box3D * __restrict__ det_boxes3d,
  float * __restrict__ bboxes_score)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= num_proposals) {
    return;
  }

  const float yaw_sin = bbox_pred_output[6 * num_proposals + point_idx];
  const float yaw_cos = bbox_pred_output[7 * num_proposals + point_idx];
  const float yaw_norm = sqrtf(yaw_sin * yaw_sin + yaw_cos * yaw_cos);
  const int label = static_cast<int>(label_pred_output[point_idx]);

  det_boxes3d[point_idx].label = label;
  det_boxes3d[point_idx].score =
    yaw_norm >= yaw_norm_thresholds[label] ? score_output[point_idx] : 0.f;

  // Set the score for sorting, so that it doesn't need to launch another kernel
  bboxes_score[point_idx] = det_boxes3d[point_idx].score;

  // Stop processing if the score is 0
  if (det_boxes3d[point_idx].score == 0.f) {
    return;
  }

  const float x =
    bbox_pred_output[0 * num_proposals + point_idx] * out_size_factor * voxel_size_x + min_x_range;
  const float y =
    bbox_pred_output[1 * num_proposals + point_idx] * out_size_factor * voxel_size_y + min_y_range;
  const float radial_distance = x * x + y * y;

  // Loop through distance_bin_upper_limits to decide the distance bucket index, and since
  // upper_bounds is sorted in ascending order, the first one that is greater than the radial
  // distance is the distance bucket index. Note that the distance_bin_upper_limits is already
  // squared, so we don't need to square the radial distance.
  int distance_bucket_index = -1;
  for (int i = 0; i < num_distance_bin_upper_limits; i++) {
    if (radial_distance < distance_bin_upper_limits[i]) {
      distance_bucket_index = i;
      break;
    }
  }

  // If the radial distance is greater than the last distance_bin_upper_limit, which is out of bound
  // and then we set the score to 0, and stop processing
  if (distance_bucket_index == -1) {
    det_boxes3d[point_idx].score = 0.f;
    // Set the score for sorting, so that it doesn't need to launch another kernel
    bboxes_score[point_idx] = 0.f;
    return;
  }

  // Index = distance_bucket_index * class_size + label since row = num of distance buckets and
  // column = num of classes
  const float class_score_threshold = score_thresholds[distance_bucket_index * class_size + label];
  // If the score is less than the class score threshold, then we set the score to 0, and stop
  // processing
  if (det_boxes3d[point_idx].score < class_score_threshold) {
    det_boxes3d[point_idx].score = 0.f;
    // Set the score for sorting, so that it doesn't need to launch another kernel
    bboxes_score[point_idx] = 0.f;
    return;
  }

  det_boxes3d[point_idx].x = x;
  det_boxes3d[point_idx].y = y;
  det_boxes3d[point_idx].z = bbox_pred_output[2 * num_proposals + point_idx];
  det_boxes3d[point_idx].length = expf(bbox_pred_output[3 * num_proposals + point_idx]);
  det_boxes3d[point_idx].width = expf(bbox_pred_output[4 * num_proposals + point_idx]);
  det_boxes3d[point_idx].height = expf(bbox_pred_output[5 * num_proposals + point_idx]);
  det_boxes3d[point_idx].yaw = atan2f(yaw_sin, yaw_cos);
  det_boxes3d[point_idx].vx = bbox_pred_output[8 * num_proposals + point_idx];
  det_boxes3d[point_idx].vy = bbox_pred_output[9 * num_proposals + point_idx];
}

PostprocessCuda::PostprocessCuda(const BEVFusionConfig & config, cudaStream_t stream)
: config_(config), stream_(stream)
{
  // Allocate memory for score thresholds on device using cuda::make_unique
  score_thresholds_d_ptr_ =
    autoware::cuda_utils::make_unique<float[]>(config_.score_thresholds_.size());
  distance_bin_upper_limits_d_ptr_ =
    autoware::cuda_utils::make_unique<float[]>(config_.distance_bin_upper_limits_.size());
  yaw_norm_thresholds_d_ptr_ =
    autoware::cuda_utils::make_unique<float[]>(config_.yaw_norm_thresholds_.size());

  // Move from host to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    score_thresholds_d_ptr_.get(), config_.score_thresholds_.data(),
    config_.score_thresholds_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    distance_bin_upper_limits_d_ptr_.get(), config_.distance_bin_upper_limits_.data(),
    config_.distance_bin_upper_limits_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    yaw_norm_thresholds_d_ptr_.get(), config_.yaw_norm_thresholds_.data(),
    config_.yaw_norm_thresholds_.size() * sizeof(float), cudaMemcpyHostToDevice, stream_));

  // Allocate memory for sorted Bboxes on device using cuda::make_unique
  bboxes_score_d_ptr_ = autoware::cuda_utils::make_unique<float[]>(config_.num_proposals_);
  sorted_bboxes_score_d_ptr_ = autoware::cuda_utils::make_unique<float[]>(config_.num_proposals_);
  bboxes_d_ptr_ = autoware::cuda_utils::make_unique<Box3D[]>(config_.num_proposals_);
  sorted_bboxes_d_ptr_ = autoware::cuda_utils::make_unique<Box3D[]>(config_.num_proposals_);

  // Initialize device memory for bbox scores and indices for argsort
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    bboxes_score_d_ptr_.get(), 0.f, config_.num_proposals_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    sorted_bboxes_score_d_ptr_.get(), 0.f, config_.num_proposals_ * sizeof(float), stream_));

  // MemsetAsync to zero for bboxes
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(bboxes_d_ptr_.get(), 0, config_.num_proposals_ * sizeof(Box3D), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    sorted_bboxes_d_ptr_.get(), 0, config_.num_proposals_ * sizeof(Box3D), stream_));

  CHECK_CUDA_ERROR(
    cub::DeviceRadixSort::SortPairsDescending(
      nullptr, sort_workspace_size_,
      static_cast<float *>(nullptr),  // KeyT = float
      static_cast<float *>(nullptr),  // KeyT = float
      static_cast<Box3D *>(nullptr),  // ValueT = Box3D
      static_cast<Box3D *>(nullptr),  // ValueT = Box3D
      config_.num_proposals_, 0, sizeof(float) * 8, stream_));
  sort_workspace_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(sort_workspace_size_);

  // Initialize CircleNMS
  circle_nms_ptr_ = std::make_unique<CircleNMS>(config_, stream_);
  filtered_bboxes_d_ptr_ = autoware::cuda_utils::make_unique<Box3D[]>(config_.num_proposals_);
  final_keep_mask_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(config_.num_proposals_);
  num_selector_d_ptr_ = autoware::cuda_utils::make_unique<std::size_t[]>(1);

  CHECK_CUDA_ERROR(cudaMemsetAsync(
    filtered_bboxes_d_ptr_.get(), 0, config_.num_proposals_ * sizeof(Box3D), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    final_keep_mask_d_.get(), 0, config_.num_proposals_ * sizeof(std::uint8_t), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(num_selector_d_ptr_.get(), 0, sizeof(std::size_t), stream_));

  // Flagged workspace for cub::DeviceSelect::Flagged
  CHECK_CUDA_ERROR(
    cub::DeviceSelect::Flagged(
      nullptr, flagged_workspace_size_,
      static_cast<Box3D *>(nullptr),         // InputIteratorT
      static_cast<std::uint8_t *>(nullptr),  // FlagIterator
      static_cast<Box3D *>(nullptr),         // OutputIteratorT
      static_cast<std::size_t *>(nullptr),   // NumSelectedIteratorT
      config_.num_proposals_, stream_));
  flagged_workspace_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(flagged_workspace_size_);
}

// cspell: ignore divup
cudaError_t PostprocessCuda::generateDetectedBoxes3D_launch(
  const std::int64_t * label_pred_output, const float * bbox_pred_output,
  const float * score_output, std::vector<Box3D> & det_boxes3d, cudaStream_t stream)
{
  dim3 threads = {config_.threads_per_block_};
  dim3 blocks = {divup(config_.num_proposals_, threads.x)};

  // Do not need to reset the bboxes_d_ptr_ and bboxes_score_d_ptr_ to zero,
  // since it's going to overwrite them in the kernel
  generateBoxes3D_kernel<<<blocks, threads, 0, stream>>>(
    label_pred_output, bbox_pred_output, score_output, config_.voxel_x_size_, config_.voxel_y_size_,
    config_.min_x_range_, config_.min_y_range_, config_.num_proposals_, config_.out_size_factor_,
    yaw_norm_thresholds_d_ptr_.get(), config_.num_classes_, distance_bin_upper_limits_d_ptr_.get(),
    score_thresholds_d_ptr_.get(), config_.distance_bin_upper_limits_.size(), bboxes_d_ptr_.get(),
    bboxes_score_d_ptr_.get());

  // Sort the bboxes by score in descending order using cub::DeviceRadixSort
  std::size_t sort_workspace_size = sort_workspace_size_;
  CHECK_CUDA_ERROR(
    cub::DeviceRadixSort::SortPairsDescending(
      sort_workspace_d_.get(), sort_workspace_size, bboxes_score_d_ptr_.get(),
      sorted_bboxes_score_d_ptr_.get(), bboxes_d_ptr_.get(), sorted_bboxes_d_ptr_.get(),
      config_.num_proposals_, 0, sizeof(float) * 8, stream));

  // Get the highest score from the sorted bboxes, which is the first element in the
  // sorted_bboxes_score_d_ptr_
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &highest_bbox_score_h_, sorted_bboxes_score_d_ptr_.get(), sizeof(float), cudaMemcpyDeviceToHost,
    stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  if (highest_bbox_score_h_ == 0.f) {
    // No valid bboxes, return empty vector
    det_boxes3d.resize(0);
    return cudaGetLastError();
  }

  // suppress by NMS
  std::vector<std::uint8_t> final_keep_mask_h =
    circle_nms_ptr_->circleNMS(sorted_bboxes_d_ptr_.get(), stream);
  // Copy final_keep_mask_h to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    final_keep_mask_d_.get(), final_keep_mask_h.data(),
    config_.num_proposals_ * sizeof(std::uint8_t), cudaMemcpyHostToDevice, stream));

  // Use cub::DeviceSelect::Flagged to select the bboxes that are kept after NMS
  std::size_t flagged_workspace_size = flagged_workspace_size_;
  CHECK_CUDA_ERROR(
    cub::DeviceSelect::Flagged(
      flagged_workspace_d_.get(), flagged_workspace_size, sorted_bboxes_d_ptr_.get(),
      final_keep_mask_d_.get(), filtered_bboxes_d_ptr_.get(), num_selector_d_ptr_.get(),
      config_.num_proposals_, stream));

  // Copy the number of selected bboxes from device to host
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &num_final_det_boxes3d, num_selector_d_ptr_.get(), sizeof(std::size_t), cudaMemcpyDeviceToHost,
    stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));

  // Resize the vector to the number of selected bboxes
  det_boxes3d.resize(num_final_det_boxes3d);
  // If no bboxes are kept after NMS, return empty vector
  if (num_final_det_boxes3d == 0) {
    return cudaGetLastError();
  }

  // memcpy device to host
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    det_boxes3d.data(), filtered_bboxes_d_ptr_.get(), num_final_det_boxes3d * sizeof(Box3D),
    cudaMemcpyDeviceToHost, stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  return cudaGetLastError();
}

}  // namespace autoware::bevfusion
