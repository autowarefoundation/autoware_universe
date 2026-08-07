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
#include "autoware/bevfusion/postprocess/postprocess_kernel.hpp"
#include "autoware/bevfusion/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <thrust/copy.h>
#include <thrust/host_vector.h>

#include <cstddef>
#include <vector>

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
      if (dist2dPow(cur_box, block_boxes + i) < dist2d_pow_threshold) {
        t |= 1ULL << i;
      }
    }
    mask[cur_box_idx * col_blocks + col_start] = t;
  }
}

cudaError_t circleNMS_launch(
  const thrust::device_vector<Box3D> & boxes3d, const std::size_t num_boxes3d,
  std::size_t col_blocks, const float distance_threshold,
  thrust::device_vector<std::uint64_t> & mask, cudaStream_t stream)
{
  const float dist2d_pow_thres = powf(distance_threshold, 2);

  dim3 blocks(col_blocks, col_blocks);
  dim3 threads(THREADS_PER_BLOCK_NMS);
  circleNMS_Kernel<<<blocks, threads, 0, stream>>>(
    thrust::raw_pointer_cast(boxes3d.data()), num_boxes3d, col_blocks, dist2d_pow_thres,
    thrust::raw_pointer_cast(mask.data()));

  return cudaGetLastError();
}

std::size_t PostprocessCuda::circleNMS(
  std::size_t num_boxes3d, const float distance_threshold, cudaStream_t stream)
{
  const auto col_blocks = divup(num_boxes3d, THREADS_PER_BLOCK_NMS);

  // Reserve scratch to the worst case once; per-call resizes never reallocate.
  const std::size_t max_boxes = static_cast<std::size_t>(config_.num_proposals_);
  const std::size_t max_col_blocks = divup(max_boxes, THREADS_PER_BLOCK_NMS);
  nms_mask_d_.reserve(max_boxes * max_col_blocks);
  nms_mask_h_.reserve(max_boxes * max_col_blocks);
  keep_mask_h_.reserve(max_boxes);
  nms_remv_h_.reserve(max_col_blocks);

  nms_mask_d_.resize(num_boxes3d * col_blocks);
  CHECK_CUDA_ERROR(circleNMS_launch(
    det_boxes3d_d_, num_boxes3d, col_blocks, distance_threshold, nms_mask_d_, stream));

  // copy the suppression mask to host on the node stream (stream-local sync)
  nms_mask_h_.resize(nms_mask_d_.size());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(nms_mask_h_.data()), thrust::raw_pointer_cast(nms_mask_d_.data()),
    nms_mask_d_.size() * sizeof(std::uint64_t), cudaMemcpyDeviceToHost, stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));

  // generate keep_mask on host
  nms_remv_h_.assign(col_blocks, 0);
  keep_mask_h_.resize(num_boxes3d);
  std::size_t num_to_keep = 0;
  for (std::size_t i = 0; i < num_boxes3d; i++) {
    auto nblock = i / THREADS_PER_BLOCK_NMS;
    auto inblock = i % THREADS_PER_BLOCK_NMS;

    if (!(nms_remv_h_[nblock] & (1ULL << inblock))) {
      keep_mask_h_[i] = true;
      num_to_keep++;
      std::uint64_t * p = &nms_mask_h_[0] + i * col_blocks;
      for (std::size_t j = nblock; j < col_blocks; j++) {
        nms_remv_h_[j] |= p[j];
      }
    } else {
      keep_mask_h_[i] = false;
    }
  }

  // memcpy host to device on the node stream (ordered before the copy_if that reads it)
  keep_mask_d_.resize(num_boxes3d);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(keep_mask_d_.data()), thrust::raw_pointer_cast(keep_mask_h_.data()),
    num_boxes3d * sizeof(bool), cudaMemcpyHostToDevice, stream));

  return num_to_keep;
}

}  // namespace autoware::bevfusion
