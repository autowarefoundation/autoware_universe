// Copyright 2025 TIER IV.
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

#include "autoware/tensorrt_vad/networks/postprocess/cuda_utils.hpp"
#include "autoware/tensorrt_vad/networks/postprocess/map_postprocess_kernel.hpp"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <cmath>

namespace autoware::tensorrt_vad
{

__device__ inline void denormalize_2d_pts_cuda(
  float * output_pt, const float * input_pt, const float * detection_range)
{
  // x = normalized_x * (x_max[3] - x_min[0]) + x_min[0]
  output_pt[0] = input_pt[0] * (detection_range[3] - detection_range[0]) + detection_range[0];
  // y = normalized_y * (y_max[4] - y_min[1]) + y_min[1]
  output_pt[1] = input_pt[1] * (detection_range[4] - detection_range[1]) + detection_range[1];
}

/**
 * @brief CUDA kernel for map postprocessing
 * Each thread processes one query (polyline)
 */
__global__ void map_postprocess_kernel(
  const float * map_cls_preds_flat, const float * map_pts_preds_flat, float * d_output_cls_scores,
  float * d_output_points, int32_t * d_output_valid_flags, int32_t * d_output_max_class_indices,
  MapPostprocessConfig config)
{
  const int32_t query_idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (query_idx >= config.map_num_queries) {
    return;
  }

  // Use final decoder layer only
  const int32_t final_layer_idx = config.num_decoder_layers - 1;
  const int32_t cls_layer_size = config.map_num_queries * config.map_num_classes;
  const int32_t pts_layer_size = config.map_num_queries * config.map_points_per_polylines * 2;
  const int32_t cls_final_layer_offset = final_layer_idx * cls_layer_size;
  const int32_t pts_final_layer_offset = final_layer_idx * pts_layer_size;

  // Process classification scores
  float max_score = 0.0f;
  int32_t max_class_idx = 0;

  // Since map_num_classes is around 3, it's fine to use a for loop within the thread
  for (int32_t c = 0; c < config.map_num_classes; ++c) {
    const int32_t cls_flat_idx = cls_final_layer_offset + query_idx * config.map_num_classes + c;
    const float score = cuda_utils::sigmoid_cuda(map_cls_preds_flat[cls_flat_idx]);
    d_output_cls_scores[query_idx * config.map_num_classes + c] = score;

    if (score > max_score) {
      max_score = score;
      max_class_idx = c;
    }
  }

  // Check if max score meets threshold for the predicted class
  bool is_valid = false;
  if (max_class_idx < config.map_class_count) {
    const float threshold = config.map_confidence_thresholds_array[max_class_idx];
    is_valid = (max_score >= threshold);
  }

  d_output_valid_flags[query_idx] = is_valid ? 1 : 0;

  // Store the max class index for each query
  d_output_max_class_indices[query_idx] = max_class_idx;

  // Process points (denormalize)
  // Since map_points_per_polylines is around 20, it's fine to use a for loop within the thread
  for (int32_t p = 0; p < config.map_points_per_polylines; ++p) {
    float normalized_pt[2];
    float denormalized_pt[2];

    for (int32_t d = 0; d < 2; ++d) {
      const int32_t pts_flat_idx =
        pts_final_layer_offset + query_idx * config.map_points_per_polylines * 2 + p * 2 + d;
      normalized_pt[d] = map_pts_preds_flat[pts_flat_idx];
    }

    denormalize_2d_pts_cuda(denormalized_pt, normalized_pt, config.detection_range_array);

    const int32_t output_idx = query_idx * config.map_points_per_polylines * 2 + p * 2;
    d_output_points[output_idx] = denormalized_pt[0];
    d_output_points[output_idx + 1] = denormalized_pt[1];
  }
}

cudaError_t launch_map_postprocess_kernel(
  const float * map_cls_preds_flat, const float * map_pts_preds_flat, float * d_output_cls_scores,
  float * d_output_points, int32_t * d_output_valid_flags, int32_t * d_output_max_class_indices,
  const MapPostprocessConfig & config, cudaStream_t stream)
{
  const int32_t threads_per_block = 256;
  const int32_t blocks_per_grid =
    (config.map_num_queries + threads_per_block - 1) / threads_per_block;

  map_postprocess_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    map_cls_preds_flat, map_pts_preds_flat, d_output_cls_scores, d_output_points,
    d_output_valid_flags, d_output_max_class_indices, config);

  return cudaGetLastError();
}

}  // namespace autoware::tensorrt_vad
