// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/lidar_frnet/point_type.hpp"
#include "autoware/lidar_frnet/postprocess_kernel.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cuda_runtime.h>

#include <cstdint>

namespace autoware::lidar_frnet
{
__constant__ uint32_t const_num_classes;
__constant__ uint32_t const_num_classes_excluded;
__constant__ float const_score_threshold;
__constant__ float const_palette[64];
__constant__ uint32_t const_excluded_class_idxs[64];

PostprocessCuda::PostprocessCuda(const utils::PostprocessingParams & params, cudaStream_t stream)
: stream_(stream)
{
  auto num_classes = params.palette.size();
  auto num_classes_excluded = params.excluded_class_idxs.size();
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_num_classes, &num_classes, sizeof(uint32_t), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_num_classes_excluded, &num_classes_excluded, sizeof(uint32_t), 0,
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_score_threshold, &params.score_threshold, sizeof(float), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_palette, params.palette.data(), num_classes * sizeof(float), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_excluded_class_idxs, params.excluded_class_idxs.data(),
    num_classes_excluded * sizeof(uint32_t), 0, cudaMemcpyHostToDevice));
}

__global__ void fillCloud_kernel(
  const InputPointType * cloud, const float * seg_logit, const uint32_t num_points,
  const bool active_comm_seg, const bool active_comm_viz, const bool active_comm_filtered,
  uint32_t * output_num_points_filtered, OutputSegmentationPointType * output_cloud_seg,
  OutputVisualizationPointType * output_cloud_viz, InputPointType * output_cloud_filtered)
{
  uint32_t point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= num_points) {
    return;
  }

  const auto & point = cloud[point_idx];
  const uint32_t pred_idx = point_idx * const_num_classes;

  float best_score = -1e9;
  uint32_t class_id = const_num_classes - 1;

  // Find the best score and class_id
  for (uint32_t i = 0; i < const_num_classes; i++) {
    float score = seg_logit[pred_idx + i];
    if (score > best_score) {
      best_score = score;
      class_id = i;
    }
  }

  // Assign points to selected outputs only
  if (active_comm_seg) {
    output_cloud_seg[point_idx].x = point.x;
    output_cloud_seg[point_idx].y = point.y;
    output_cloud_seg[point_idx].z = point.z;
    output_cloud_seg[point_idx].class_id = best_score >= const_score_threshold
                                             ? static_cast<uint8_t>(class_id)
                                             : static_cast<uint8_t>(const_num_classes - 1);
  }

  if (active_comm_viz) {
    output_cloud_viz[point_idx].x = point.x;
    output_cloud_viz[point_idx].y = point.y;
    output_cloud_viz[point_idx].z = point.z;
    output_cloud_viz[point_idx].rgb = best_score >= const_score_threshold
                                        ? const_palette[class_id]
                                        : const_palette[const_num_classes - 1];
  }

  if (active_comm_filtered) {
    // Check if the class is excluded
    bool excluded = false;
    for (uint32_t i = 0; i < const_num_classes_excluded; i++) {
      if (const_excluded_class_idxs[i] == class_id && best_score >= const_score_threshold) {
        excluded = true;
        break;
      }
    }

    // Process non-excluded points
    if (!excluded) {
      const uint32_t append_idx = atomicAdd(output_num_points_filtered, 1);
      output_cloud_filtered[append_idx] = point;
    }
  }
}

cudaError_t PostprocessCuda::fillCloud_launch(
  const InputPointType * cloud, const float * seg_logit, const int32_t input_num_points,
  const utils::ActiveComm & active_comm, uint32_t * output_num_points_filtered,
  OutputSegmentationPointType * output_cloud_seg, OutputVisualizationPointType * output_cloud_viz,
  InputPointType * output_cloud_filtered)
{
  dim3 block(utils::divup(input_num_points, utils::kernel_1d_size));
  dim3 threads(utils::kernel_1d_size);

  fillCloud_kernel<<<block, threads, 0, stream_>>>(
    cloud, seg_logit, input_num_points, active_comm.seg, active_comm.viz, active_comm.filtered,
    output_num_points_filtered, output_cloud_seg, output_cloud_viz, output_cloud_filtered);

  return cudaGetLastError();
}

}  // namespace autoware::lidar_frnet
