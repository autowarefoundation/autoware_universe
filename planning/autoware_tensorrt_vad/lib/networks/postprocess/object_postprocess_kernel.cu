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
#include "autoware/tensorrt_vad/networks/postprocess/object_postprocess_kernel.hpp"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <cfloat>
#include <cmath>

namespace autoware::tensorrt_vad
{

// Namespace for physical bounds in postprocessing
namespace physical_limits
{
constexpr float MAX_CLASSIFICATION_INPUT = 100.0f;  // Maximum reasonable classification logit
constexpr float MAX_BBOX_EXP_INPUT = 10.0f;         // Maximum bbox exp input (e^10 ≈ 22000)
constexpr float MIN_BBOX_DIMENSION = 0.01f;         // Minimum realistic bbox dimension (1cm)
constexpr float MAX_BBOX_DIMENSION = 1000.0f;       // Maximum realistic bbox dimension (1km)
constexpr float MAX_COORDINATE_VALUE = 10000.0f;    // Maximum reasonable coordinate value (10km)
}  // namespace physical_limits

/**
 * @brief CUDA kernel for object postprocessing
 * Each thread processes one object query
 */
__global__ void object_postprocess_kernel(
  const float * all_cls_scores_flat, const float * all_traj_preds_flat,
  const float * all_traj_cls_scores_flat, const float * all_bbox_preds_flat,
  float * d_output_cls_scores, float * d_output_bbox_preds, float * d_output_trajectories,
  float * d_output_traj_scores, int32_t * d_output_valid_flags,
  int32_t * d_output_max_class_indices, ObjectPostprocessConfig config)
{
  const int32_t obj_idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (obj_idx >= config.prediction_num_queries) {
    return;
  }

  // Use final decoder layer only
  const int32_t final_layer_idx = config.num_decoder_layers - 1;

  // Calculate layer offsets
  const int32_t cls_layer_size = config.prediction_num_queries * config.prediction_num_classes;
  const int32_t bbox_layer_size = config.prediction_num_queries * config.prediction_bbox_pred_dim;
  const int32_t traj_layer_size = config.prediction_num_queries *
                                  config.prediction_trajectory_modes * config.prediction_timesteps *
                                  2;
  const int32_t traj_cls_layer_size =
    config.prediction_num_queries * config.prediction_trajectory_modes;

  const int32_t cls_final_layer_offset = final_layer_idx * cls_layer_size;
  const int32_t bbox_final_layer_offset = final_layer_idx * bbox_layer_size;
  const int32_t traj_final_layer_offset = final_layer_idx * traj_layer_size;
  const int32_t traj_cls_final_layer_offset = final_layer_idx * traj_cls_layer_size;

  // Process classification scores (apply sigmoid with safety checks)
  float max_score = 0.0f;
  int32_t max_class_idx = 0;

  for (int32_t c = 0; c < config.prediction_num_classes; ++c) {
    const int32_t cls_flat_idx =
      cls_final_layer_offset + obj_idx * config.prediction_num_classes + c;

    // Safety check: ensure we don't read invalid memory
    float raw_score = all_cls_scores_flat[cls_flat_idx];

    // Clamp input to prevent extreme values using namespace constant
    raw_score = fmaxf(
      -physical_limits::MAX_CLASSIFICATION_INPUT,
      fminf(physical_limits::MAX_CLASSIFICATION_INPUT, raw_score));

    const float score = autoware::tensorrt_vad::cuda_utils::sigmoid_cuda(raw_score);
    d_output_cls_scores[obj_idx * config.prediction_num_classes + c] = score;

    if (score > max_score) {
      max_score = score;
      max_class_idx = c;
    }
  }

  // Check if max score meets threshold for the predicted class
  bool is_valid = false;
  if (max_class_idx < config.bbox_class_count && max_class_idx < 16) {
    const float threshold = config.object_confidence_thresholds_array[max_class_idx];
    is_valid = (max_score >= threshold);
  }

  d_output_valid_flags[obj_idx] = is_valid ? 1 : 0;

  // Store the max class index for each object
  d_output_max_class_indices[obj_idx] = max_class_idx;

  // Process bbox predictions (apply exp to w, l, h with safety checks)
  for (int32_t i = 0; i < config.prediction_bbox_pred_dim; ++i) {
    const int32_t bbox_flat_idx =
      bbox_final_layer_offset + obj_idx * config.prediction_bbox_pred_dim + i;
    float bbox_value = all_bbox_preds_flat[bbox_flat_idx];

    // Apply exp transformation to w(2), l(3), h(5)
    if (i == 2 || i == 3 || i == 5) {
      // Clamp input to prevent extreme values before exp using namespace constant
      bbox_value = fmaxf(
        -physical_limits::MAX_BBOX_EXP_INPUT,
        fminf(physical_limits::MAX_BBOX_EXP_INPUT, bbox_value));
      bbox_value = cuda_utils::exp_cuda(bbox_value);

      // Additional safety: ensure reasonable bbox dimensions using namespace constants
      bbox_value = fmaxf(
        physical_limits::MIN_BBOX_DIMENSION,
        fminf(physical_limits::MAX_BBOX_DIMENSION, bbox_value));
    }

    d_output_bbox_preds[obj_idx * config.prediction_bbox_pred_dim + i] = bbox_value;
  }

  // Process trajectory predictions with safety checks
  for (int32_t mode = 0; mode < config.prediction_trajectory_modes; ++mode) {
    // Process trajectory classification scores (apply sigmoid with safety)
    const int32_t traj_cls_flat_idx =
      traj_cls_final_layer_offset + obj_idx * config.prediction_trajectory_modes + mode;
    float raw_traj_score = all_traj_cls_scores_flat[traj_cls_flat_idx];

    // Clamp input to prevent extreme values using namespace constant
    raw_traj_score = fmaxf(
      -physical_limits::MAX_CLASSIFICATION_INPUT,
      fminf(physical_limits::MAX_CLASSIFICATION_INPUT, raw_traj_score));

    const float traj_score = cuda_utils::sigmoid_cuda(raw_traj_score);
    d_output_traj_scores[obj_idx * config.prediction_trajectory_modes + mode] = traj_score;

    // Process trajectory points
    for (int32_t ts = 0; ts < config.prediction_timesteps; ++ts) {
      for (int32_t d = 0; d < 2; ++d) {  // x, y coordinates
        const int32_t traj_flat_idx =
          traj_final_layer_offset +
          obj_idx * config.prediction_trajectory_modes * config.prediction_timesteps * 2 +
          mode * config.prediction_timesteps * 2 + ts * 2 + d;

        const int32_t output_idx =
          obj_idx * config.prediction_trajectory_modes * config.prediction_timesteps * 2 +
          mode * config.prediction_timesteps * 2 + ts * 2 + d;

        float traj_value = all_traj_preds_flat[traj_flat_idx];

        // Apply reasonable bounds to trajectory coordinates using namespace constant
        traj_value = fmaxf(
          -physical_limits::MAX_COORDINATE_VALUE,
          fminf(physical_limits::MAX_COORDINATE_VALUE, traj_value));

        d_output_trajectories[output_idx] = traj_value;
      }
    }

    // Apply cumulative sum to build trajectory in 3D space (like CPU implementation)
    // This is critical for accurate trajectory representation
    for (int32_t ts = 1; ts < config.prediction_timesteps; ++ts) {
      for (int32_t d = 0; d < 2; ++d) {  // x, y coordinates
        const int32_t current_idx =
          obj_idx * config.prediction_trajectory_modes * config.prediction_timesteps * 2 +
          mode * config.prediction_timesteps * 2 + ts * 2 + d;
        const int32_t prev_idx =
          obj_idx * config.prediction_trajectory_modes * config.prediction_timesteps * 2 +
          mode * config.prediction_timesteps * 2 + (ts - 1) * 2 + d;

        // Cumulative sum: current_position += previous_position
        d_output_trajectories[current_idx] += d_output_trajectories[prev_idx];
      }
    }
  }
}

cudaError_t launch_object_postprocess_kernel(
  const float * all_cls_scores_flat, const float * all_traj_preds_flat,
  const float * all_traj_cls_scores_flat, const float * all_bbox_preds_flat,
  float * d_output_cls_scores, float * d_output_bbox_preds, float * d_output_trajectories,
  float * d_output_traj_scores, int32_t * d_output_valid_flags,
  int32_t * d_output_max_class_indices, const ObjectPostprocessConfig & config, cudaStream_t stream)
{
  const int32_t threads_per_block = 256;
  const int32_t blocks_per_grid =
    (config.prediction_num_queries + threads_per_block - 1) / threads_per_block;

  object_postprocess_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    all_cls_scores_flat, all_traj_preds_flat, all_traj_cls_scores_flat, all_bbox_preds_flat,
    d_output_cls_scores, d_output_bbox_preds, d_output_trajectories, d_output_traj_scores,
    d_output_valid_flags, d_output_max_class_indices, config);

  return cudaGetLastError();
}

}  // namespace autoware::tensorrt_vad
