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

// Modified from
// https://github.com/open-mmlab/OpenPCDet/blob/master/pcdet/ops/iou3d_nms/src/iou3d_nms_kernel.cu

/*
3D IoU Calculation and Rotated NMS(modified from 2D NMS written by others)
Written by Shaoshuai Shi
All Rights Reserved 2019-2020.
*/

#include "autoware/bevfusion/postprocess/circle_nms_kernel.hpp"
#include "autoware/bevfusion/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <thrust/host_vector.h>

#include <cstddef>

namespace
{
const std::size_t THREADS_PER_BLOCK_NMS = 16;
}  // namespace

namespace autoware::bevfusion
{

__device__ inline float dist2dPow(const Box3D * a, const Box3D * b)
{
  return powf(a->x - b->x, 2) + powf(a->y - b->y, 2);
}

// cspell: ignore divup
__global__ void circleNMS_Kernel(
  const Box3D * __restrict__ boxes, const std::size_t num_boxes3d, const std::size_t col_blocks,
  const float dist2d_pow_threshold, std::uint64_t * __restrict__ mask)
{
  // params: boxes (N,)
  // params: mask (N, divup(N/THREADS_PER_BLOCK_NMS))

  const auto row_start = blockIdx.y;
  const auto col_start = blockIdx.x;

  if (row_start > col_start) return;

  const std::size_t row_size =
    fminf(num_boxes3d - row_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);
  const std::size_t col_size =
    fminf(num_boxes3d - col_start * THREADS_PER_BLOCK_NMS, THREADS_PER_BLOCK_NMS);

  __shared__ Box3D block_boxes[THREADS_PER_BLOCK_NMS];

  if (threadIdx.x < col_size) {
    block_boxes[threadIdx.x] = boxes[THREADS_PER_BLOCK_NMS * col_start + threadIdx.x];
  }
  __syncthreads();

  if (threadIdx.x < row_size) {
    const std::size_t cur_box_idx = THREADS_PER_BLOCK_NMS * row_start + threadIdx.x;
    const Box3D * cur_box = boxes + cur_box_idx;

    std::uint64_t t = 0;
    std::size_t start = 0;
    if (row_start == col_start) {
      start = threadIdx.x + 1;
    }
    for (std::size_t i = start; i < col_size; i++) {
      // If the score is 0 (invalid) or the distance is less than the threshold, set the
      // corresponding bit in t
      if (
        (block_boxes[i].score == 0.f) ||
        dist2dPow(cur_box, &block_boxes[i]) < dist2d_pow_threshold) {
        t |= 1ULL << i;
      }
    }
    mask[cur_box_idx * col_blocks + col_start] = t;
  }
}

cudaError_t circleNMS_launch(
  Box3D * boxes3d, const std::size_t num_boxes3d, std::size_t col_blocks,
  const float distance_threshold, std::uint64_t * mask, cudaStream_t stream)
{
  const float dist2d_pow_thres = powf(distance_threshold, 2);

  dim3 blocks(col_blocks, col_blocks);
  dim3 threads(THREADS_PER_BLOCK_NMS);
  circleNMS_Kernel<<<blocks, threads, 0, stream>>>(
    boxes3d, num_boxes3d, col_blocks, dist2d_pow_thres, mask);

  return cudaGetLastError();
}

CircleNMS::CircleNMS(const BEVFusionConfig & config, cudaStream_t stream)
: config_(config), stream_(stream)
{
  // Allocate memory for NMS
  col_blocks_ = divup(config_.num_proposals_, THREADS_PER_BLOCK_NMS);

  // Final keep mask for NMS
  final_keep_mask_d_ =
    autoware::cuda_utils::make_unique<std::uint64_t[]>(config_.num_proposals_ * col_blocks_);
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    final_keep_mask_d_.get(), 0, config_.num_proposals_ * col_blocks_ * sizeof(std::uint64_t),
    stream_));

  final_keep_mask_h_.resize(config_.num_proposals_ * col_blocks_);
}

std::vector<std::uint8_t> CircleNMS::circleNMS(
  Box3D * descending_sorted_bboxes, cudaStream_t stream)
{
  CHECK_CUDA_ERROR(circleNMS_launch(
    descending_sorted_bboxes, config_.num_proposals_, col_blocks_,
    config_.circle_nms_dist_threshold_, final_keep_mask_d_.get(), stream));

  // memcpy device to host
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    final_keep_mask_h_.data(), final_keep_mask_d_.get(),
    config_.num_proposals_ * col_blocks_ * sizeof(std::uint64_t), cudaMemcpyDeviceToHost, stream));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  // generate keep_mask
  std::vector<std::uint64_t> remv_h(col_blocks_);
  std::vector<std::uint8_t> keep_mask_h(config_.num_proposals_);
  std::size_t num_to_keep = 0;
  for (std::size_t i = 0; i < config_.num_proposals_; i++) {
    auto nblock = i / THREADS_PER_BLOCK_NMS;
    auto inblock = i % THREADS_PER_BLOCK_NMS;

    if (!(remv_h[nblock] & (1ULL << inblock))) {
      keep_mask_h[i] = true;
      num_to_keep++;
      std::uint64_t * p = &final_keep_mask_h_[0] + i * col_blocks_;
      for (std::size_t j = nblock; j < col_blocks_; j++) {
        remv_h[j] |= p[j];
      }
    } else {
      keep_mask_h[i] = false;
    }
  }

  return keep_mask_h;
}

}  // namespace autoware::bevfusion
