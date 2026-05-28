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

#include "autoware/ptv3/postprocess/det3d_centerhead_postprocess.hpp"

#include <cuda_fp16.h>
#include <thrust/copy.h>
#include <thrust/count.h>
#include <thrust/execution_policy.h>
#include <thrust/sort.h>

namespace
{
constexpr std::size_t k_threads_per_block = 32;
}  // namespace

namespace autoware::ptv3
{

struct IsScoreKeep
{
  __device__ bool operator()(const Box3D & box) const { return box.score > 0.0F; }
};

struct ScoreGreater
{
  __device__ bool operator()(const Box3D & lhs, const Box3D & rhs) const
  {
    return lhs.score > rhs.score;
  }
};

__device__ inline float sigmoid(float x)
{
  return 1.0F / (1.0F + expf(-x));
}

template <typename FloatType>
__global__ void generate_boxes3d_kernel(
  const FloatType * __restrict__ out_heatmap, const FloatType * __restrict__ out_reg,
  const FloatType * __restrict__ out_height, const FloatType * __restrict__ out_dim,
  const FloatType * __restrict__ out_rot, const FloatType * __restrict__ out_vel,
  float voxel_size_x, float voxel_size_y, float range_min_x, float range_min_y,
  std::size_t det_grid_size_x, std::size_t det_grid_size_y, std::size_t bbox_downsample_factor,
  int class_size, const float * __restrict__ distance_bin_upper_limits,
  const float * __restrict__ score_thresholds, std::size_t num_distance_bins, bool has_twist,
  bool has_variance, const float * __restrict__ yaw_norm_thresholds,
  Box3D * __restrict__ det_boxes3d)
{
  const auto yi = blockIdx.x * k_threads_per_block + threadIdx.x;
  const auto xi = blockIdx.y * k_threads_per_block + threadIdx.y;
  const auto idx = det_grid_size_x * yi + xi;
  const auto det_grid_size = det_grid_size_y * det_grid_size_x;

  if (yi >= det_grid_size_y || xi >= det_grid_size_x) {
    return;
  }
  det_boxes3d[idx] = Box3D{};

  int label = -1;
  float max_score = -1.0F;
  for (int class_index = 0; class_index < class_size; ++class_index) {
    const float score = sigmoid(static_cast<float>(out_heatmap[det_grid_size * class_index + idx]));
    if (score > max_score) {
      label = class_index;
      max_score = score;
    }
  }

  if (label == -1) {
    return;
  }

  const float offset_x = static_cast<float>(out_reg[det_grid_size * 0 + idx]);
  const float offset_y = static_cast<float>(out_reg[det_grid_size * 1 + idx]);
  const float x = voxel_size_x * bbox_downsample_factor * (xi + offset_x) + range_min_x;
  const float y = voxel_size_y * bbox_downsample_factor * (yi + offset_y) + range_min_y;
  const float radial_distance = sqrtf(x * x + y * y);

  int distance_bucket_index = -1;
  for (std::size_t i = 0; i < num_distance_bins; ++i) {
    if (radial_distance < distance_bin_upper_limits[i]) {
      distance_bucket_index = static_cast<int>(i);
      break;
    }
  }
  if (distance_bucket_index == -1) {
    return;
  }

  const float class_score_threshold = score_thresholds[distance_bucket_index * class_size + label];
  if (max_score < class_score_threshold) {
    return;
  }

  const float z = static_cast<float>(out_height[idx]);
  const float w = static_cast<float>(out_dim[det_grid_size * 0 + idx]);
  const float l = static_cast<float>(out_dim[det_grid_size * 1 + idx]);
  const float h = static_cast<float>(out_dim[det_grid_size * 2 + idx]);
  const float yaw_sin = static_cast<float>(out_rot[det_grid_size * 0 + idx]);
  const float yaw_cos = static_cast<float>(out_rot[det_grid_size * 1 + idx]);
  const float yaw_norm = sqrtf(yaw_sin * yaw_sin + yaw_cos * yaw_cos);
  const float vel_x = has_twist ? static_cast<float>(out_vel[det_grid_size * 0 + idx]) : 0.0F;
  const float vel_y = has_twist ? static_cast<float>(out_vel[det_grid_size * 1 + idx]) : 0.0F;

  det_boxes3d[idx].label = label;
  det_boxes3d[idx].score = yaw_norm >= yaw_norm_thresholds[label] ? max_score : 0.0F;
  if (det_boxes3d[idx].score == 0.0F) {
    return;
  }

  det_boxes3d[idx].x = x;
  det_boxes3d[idx].y = y;
  det_boxes3d[idx].z = z;
  det_boxes3d[idx].length = expf(l);
  det_boxes3d[idx].width = expf(w);
  det_boxes3d[idx].height = expf(h);
  det_boxes3d[idx].yaw = atan2f(yaw_sin, yaw_cos);
  det_boxes3d[idx].vel_x = vel_x;
  det_boxes3d[idx].vel_y = vel_y;

  if (has_variance) {
    const float offset_x_variance = static_cast<float>(out_reg[det_grid_size * 2 + idx]);
    const float offset_y_variance = static_cast<float>(out_reg[det_grid_size * 3 + idx]);
    const float z_variance = static_cast<float>(out_height[det_grid_size * 1 + idx]);
    const float w_variance = static_cast<float>(out_dim[det_grid_size * 3 + idx]);
    const float l_variance = static_cast<float>(out_dim[det_grid_size * 4 + idx]);
    const float h_variance = static_cast<float>(out_dim[det_grid_size * 5 + idx]);
    const float yaw_sin_log_variance = static_cast<float>(out_rot[det_grid_size * 2 + idx]);
    const float yaw_cos_log_variance = static_cast<float>(out_rot[det_grid_size * 3 + idx]);
    const float vel_x_variance =
      has_twist ? static_cast<float>(out_vel[det_grid_size * 2 + idx]) : 0.0F;
    const float vel_y_variance =
      has_twist ? static_cast<float>(out_vel[det_grid_size * 3 + idx]) : 0.0F;

    det_boxes3d[idx].x_variance = voxel_size_x * bbox_downsample_factor * expf(offset_x_variance);
    det_boxes3d[idx].y_variance = voxel_size_y * bbox_downsample_factor * expf(offset_y_variance);
    det_boxes3d[idx].z_variance = expf(z_variance);
    det_boxes3d[idx].length_variance = expf(l_variance);
    det_boxes3d[idx].width_variance = expf(w_variance);
    det_boxes3d[idx].height_variance = expf(h_variance);

    const float yaw_sin_sq = yaw_sin * yaw_sin;
    const float yaw_cos_sq = yaw_cos * yaw_cos;
    const float yaw_norm_sq = (yaw_sin_sq + yaw_cos_sq) * (yaw_sin_sq + yaw_cos_sq);
    det_boxes3d[idx].yaw_variance =
      (yaw_cos_sq * expf(yaw_sin_log_variance) + yaw_sin_sq * expf(yaw_cos_log_variance)) /
      yaw_norm_sq;
    det_boxes3d[idx].vel_x_variance = has_twist ? expf(vel_x_variance) : 0.0F;
    det_boxes3d[idx].vel_y_variance = has_twist ? expf(vel_y_variance) : 0.0F;
  }
}

Det3dCenterHeadPostprocess::Det3dCenterHeadPostprocess(
  const PTv3Config & config, cudaStream_t stream)
: config_(config),
  max_boxes_(config.det_grid_x_size_ * config.det_grid_y_size_),
  raw_boxes_d_(max_boxes_),
  passing_boxes_d_(max_boxes_),
  yaw_norm_thresholds_d_(config.yaw_norm_thresholds_.begin(), config.yaw_norm_thresholds_.end())
{
  distance_bin_upper_limits_d_ =
    autoware::cuda_utils::make_unique<float[]>(config_.distance_bin_upper_limits_.size());
  score_thresholds_d_ =
    autoware::cuda_utils::make_unique<float[]>(config_.detection_score_thresholds_.size());

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    distance_bin_upper_limits_d_.get(), config_.distance_bin_upper_limits_.data(),
    config_.distance_bin_upper_limits_.size() * sizeof(float), cudaMemcpyHostToDevice, stream));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    score_thresholds_d_.get(), config_.detection_score_thresholds_.data(),
    config_.detection_score_thresholds_.size() * sizeof(float), cudaMemcpyHostToDevice, stream));
}

template <typename FloatType>
cudaError_t Det3dCenterHeadPostprocess::process(
  const FloatType * out_heatmap, const FloatType * out_reg, const FloatType * out_height,
  const FloatType * out_dim, const FloatType * out_rot, const FloatType * out_vel,
  cudaStream_t stream)
{
  num_boxes_ = 0;

  dim3 blocks(
    divup(config_.det_grid_y_size_, k_threads_per_block),
    divup(config_.det_grid_x_size_, k_threads_per_block));
  dim3 threads(k_threads_per_block, k_threads_per_block);

  generate_boxes3d_kernel<FloatType><<<blocks, threads, 0, stream>>>(
    out_heatmap, out_reg, out_height, out_dim, out_rot, out_vel, config_.bbox_voxel_x_size_,
    config_.bbox_voxel_y_size_, config_.min_x_range_, config_.min_y_range_,
    config_.det_grid_x_size_, config_.det_grid_y_size_, config_.bbox_downsample_factor_,
    static_cast<int>(config_.detection_class_names_.size()), distance_bin_upper_limits_d_.get(),
    score_thresholds_d_.get(), config_.distance_bin_upper_limits_.size(), config_.has_twist_,
    config_.has_variance_, thrust::raw_pointer_cast(yaw_norm_thresholds_d_.data()),
    thrust::raw_pointer_cast(raw_boxes_d_.data()));

  const auto policy = thrust::cuda::par.on(stream);

  const auto num_passing =
    thrust::count_if(policy, raw_boxes_d_.begin(), raw_boxes_d_.end(), IsScoreKeep());
  if (num_passing == 0) {
    return cudaGetLastError();
  }

  const auto passing_end = thrust::copy_if(
    policy, raw_boxes_d_.begin(), raw_boxes_d_.end(), passing_boxes_d_.begin(), IsScoreKeep());
  thrust::sort(policy, passing_boxes_d_.begin(), passing_end, ScoreGreater());

  num_boxes_ = static_cast<std::size_t>(num_passing);
  return cudaGetLastError();
}

const Box3D * Det3dCenterHeadPostprocess::device_boxes() const
{
  return thrust::raw_pointer_cast(passing_boxes_d_.data());
}

template cudaError_t Det3dCenterHeadPostprocess::process<float>(
  const float *, const float *, const float *, const float *, const float *, const float *,
  cudaStream_t);
template cudaError_t Det3dCenterHeadPostprocess::process<__half>(
  const __half *, const __half *, const __half *, const __half *, const __half *, const __half *,
  cudaStream_t);

}  // namespace autoware::ptv3
