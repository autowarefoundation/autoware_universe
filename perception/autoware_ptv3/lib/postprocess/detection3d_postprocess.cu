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

#include "autoware/ptv3/postprocess/detection3d_postprocess.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>
#include <thrust/copy.h>
#include <thrust/count.h>
#include <thrust/execution_policy.h>
#include <thrust/sort.h>

#include <cstdint>

namespace autoware::ptv3
{

namespace
{
constexpr int k_block_size = 256;

struct IsScoreNonZero
{
  __device__ bool operator()(const Box3D & box) const { return box.score > 0.0f; }
};

struct IsScoreGreaterThan
{
  __device__ bool operator()(const Box3D & lhs, const Box3D & rhs) const
  {
    return lhs.score > rhs.score;
  }
};

// Decode one Detection3D proposal per thread into a Box3D. Rejected proposals keep score 0 and are
// compacted away by process(). All boundary tensors are pinned to float32 and integer types
// (int32/int64) via NetworkIO, so no runtime dtype dispatch is needed. The `heatmap` input is the
// per-query class logit
__global__ void decodeDetection3DToBoxesKernel(
  const float * __restrict__ query_heatmap_score, const std::int64_t * __restrict__ query_labels,
  const float * __restrict__ heatmap, const float * __restrict__ center,
  const float * __restrict__ height, const float * __restrict__ dim, const float * __restrict__ rot,
  const float * __restrict__ vel, int num_proposals, int num_classes, float bbox_voxel_x_size,
  float bbox_voxel_y_size, float min_x_range, float min_y_range,
  const float * __restrict__ post_center_range, const float * __restrict__ score_thresholds,
  const float * __restrict__ dist_bin_limits, int num_dist_bins,
  const float * __restrict__ yaw_norm_thresholds, bool has_twist, Box3D * __restrict__ out_boxes)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_proposals) {
    return;
  }

  out_boxes[i] = Box3D{};

  const int label = static_cast<int>(query_labels[i]);
  if (label < 0 || label >= num_classes) {
    return;
  }

  const int class_offset = label * num_proposals;
  auto sigmoid = [](float logit) { return 1.0f / (1.0f + expf(-logit)); };
  const float score = sigmoid(heatmap[class_offset + i]) * query_heatmap_score[class_offset + i];

  const float x = center[i] * bbox_voxel_x_size + min_x_range;
  const float y = center[num_proposals + i] * bbox_voxel_y_size + min_y_range;
  const float box_h = expf(dim[2 * num_proposals + i]);
  const float z = height[i] - box_h * 0.5f;

  if (
    x < post_center_range[0] || y < post_center_range[1] || z < post_center_range[2] ||
    x > post_center_range[3] || y > post_center_range[4] || z > post_center_range[5]) {
    return;
  }

  const float radial = sqrtf(x * x + y * y);
  float threshold = 1.0e9f;
  for (int d = 0; d < num_dist_bins; ++d) {
    if (radial < dist_bin_limits[d]) {
      threshold = score_thresholds[d * num_classes + label];
      break;
    }
  }
  if (score < threshold) {
    return;
  }

  const float yaw_sin = rot[i];
  const float yaw_cos = rot[num_proposals + i];
  const float yaw_norm = sqrtf(yaw_sin * yaw_sin + yaw_cos * yaw_cos);
  if (yaw_norm < yaw_norm_thresholds[label]) {
    return;
  }

  const float length = expf(dim[i]);
  const float width = expf(dim[num_proposals + i]);

  out_boxes[i].label = label;
  out_boxes[i].score = score;
  out_boxes[i].x = x;
  out_boxes[i].y = y;
  out_boxes[i].z = z;
  out_boxes[i].length = length;
  out_boxes[i].width = width;
  out_boxes[i].height = box_h;
  out_boxes[i].yaw = atan2f(yaw_sin, yaw_cos);
  out_boxes[i].vel_x = has_twist ? vel[i] : 0.0F;
  out_boxes[i].vel_y = has_twist ? vel[num_proposals + i] : 0.0F;
}

void launchDecodeDetection3DToBoxes(
  const float * query_heatmap_score_d, const std::int64_t * query_labels_d, const float * heatmap_d,
  const float * center_d, const float * height_d, const float * dim_d, const float * rot_d,
  const float * vel_d, int num_proposals, int num_classes, float bbox_voxel_x_size,
  float bbox_voxel_y_size, float min_x_range, float min_y_range, const float * post_center_range_d,
  const float * score_thresholds_d, const float * dist_bin_limits_d, int num_dist_bins,
  const float * yaw_norm_thresholds_d, bool has_twist, Box3D * out_boxes_d, cudaStream_t stream)
{
  const int grid = (num_proposals + k_block_size - 1) / k_block_size;
  decodeDetection3DToBoxesKernel<<<grid, k_block_size, 0, stream>>>(
    query_heatmap_score_d, query_labels_d, heatmap_d, center_d, height_d, dim_d, rot_d, vel_d,
    num_proposals, num_classes, bbox_voxel_x_size, bbox_voxel_y_size, min_x_range, min_y_range,
    post_center_range_d, score_thresholds_d, dist_bin_limits_d, num_dist_bins,
    yaw_norm_thresholds_d, has_twist, out_boxes_d);
}
}  // namespace

Detection3DPostprocess::Detection3DPostprocess(const PTv3Config & config, cudaStream_t stream)
: config_(config),
  raw_boxes_d_(config.num_proposals_),
  passing_boxes_d_(config.num_proposals_),
  yaw_norm_thresholds_d_(config.yaw_norm_thresholds_.begin(), config.yaw_norm_thresholds_.end())
{
  post_center_range_d_ = autoware::cuda_utils::make_unique<float[]>(6);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    post_center_range_d_.get(), config_.post_center_range_.data(), 6 * sizeof(float),
    cudaMemcpyHostToDevice, stream));

  score_thresholds_d_ =
    autoware::cuda_utils::make_unique<float[]>(config_.detection_score_thresholds_.size());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    score_thresholds_d_.get(), config_.detection_score_thresholds_.data(),
    config_.detection_score_thresholds_.size() * sizeof(float), cudaMemcpyHostToDevice, stream));

  dist_bin_limits_d_ =
    autoware::cuda_utils::make_unique<float[]>(config_.distance_bin_upper_limits_.size());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    dist_bin_limits_d_.get(), config_.distance_bin_upper_limits_.data(),
    config_.distance_bin_upper_limits_.size() * sizeof(float), cudaMemcpyHostToDevice, stream));
}

cudaError_t Detection3DPostprocess::process(
  const float * query_heatmap_score_d, const std::int64_t * query_labels_d, const float * heatmap_d,
  const float * center_d, const float * height_d, const float * dim_d, const float * rot_d,
  const float * vel_d, cudaStream_t stream)
{
  num_boxes_ = 0;

  launchDecodeDetection3DToBoxes(
    query_heatmap_score_d, query_labels_d, heatmap_d, center_d, height_d, dim_d, rot_d, vel_d,
    static_cast<int>(config_.num_proposals_),
    static_cast<int>(config_.detection_class_names_.size()), config_.bbox_voxel_x_size_,
    config_.bbox_voxel_y_size_, config_.min_x_range_, config_.min_y_range_,
    post_center_range_d_.get(), score_thresholds_d_.get(), dist_bin_limits_d_.get(),
    static_cast<int>(config_.distance_bin_upper_limits_.size()),
    thrust::raw_pointer_cast(yaw_norm_thresholds_d_.data()), config_.has_twist_,
    thrust::raw_pointer_cast(raw_boxes_d_.data()), stream);

  const auto policy = thrust::cuda::par.on(stream);
  const auto passing_end = thrust::copy_if(
    policy, raw_boxes_d_.begin(), raw_boxes_d_.end(), passing_boxes_d_.begin(), IsScoreNonZero{});
  const auto num_passing = static_cast<std::size_t>(passing_end - passing_boxes_d_.begin());

  if (num_passing == 0) {
    return cudaGetLastError();
  }

  thrust::sort(policy, passing_boxes_d_.begin(), passing_end, IsScoreGreaterThan{});

  num_boxes_ = static_cast<std::size_t>(num_passing);
  return cudaGetLastError();
}

}  // namespace autoware::ptv3
