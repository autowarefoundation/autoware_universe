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

#ifndef AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_KERNEL_HPP_

#include <cuda_runtime.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

/**
 * @struct ObjectPostprocessConfig
 * @brief Configuration parameters required for object CUDA postprocessing
 */
struct ObjectPostprocessConfig
{
  // VAD configuration parameters
  int32_t prediction_num_queries;       // Number of detection queries (maximum objects)
  int32_t prediction_num_classes;       // Number of object classes
  int32_t prediction_bbox_pred_dim;     // Bounding box prediction dimension
  int32_t prediction_trajectory_modes;  // Number of trajectory prediction modes per object
  int32_t prediction_timesteps;         // Number of prediction timesteps
  int32_t num_decoder_layers;           // Number of decoder layers

  // Class names and confidence thresholds
  std::vector<std::string> bbox_class_names;
  std::map<std::string, float> object_confidence_thresholds;

  // Additional members for CUDA kernel (computed at runtime)
  int32_t bbox_class_count;
  float object_confidence_thresholds_array[16];  // Configurable max classes (adjustable)

  // Helper method to prepare kernel data
  void prepare_for_kernel()
  {
    bbox_class_count = static_cast<int32_t>(bbox_class_names.size());

    // Initialize confidence thresholds array
    for (int32_t i = 0; i < 16; ++i) {
      object_confidence_thresholds_array[i] = 0.0f;
    }

    // Copy confidence thresholds to flat array
    for (int32_t i = 0; i < static_cast<int32_t>(bbox_class_names.size()) && i < 16; ++i) {
      const std::string & class_name = bbox_class_names.at(i);
      auto it = object_confidence_thresholds.find(class_name);
      if (it != object_confidence_thresholds.end()) {
        object_confidence_thresholds_array[i] = it->second;
      }
    }
  }
};

/**
 * @brief Launch CUDA kernel to postprocess object predictions
 * @param all_cls_scores_flat Flat array of object classification scores
 * @param all_traj_preds_flat Flat array of trajectory predictions
 * @param all_traj_cls_scores_flat Flat array of trajectory classification scores
 * @param all_bbox_preds_flat Flat array of bounding box predictions
 * @param d_output_cls_scores Device buffer for output classification scores [num_queries,
 * num_classes]
 * @param d_output_bbox_preds Device buffer for output bbox predictions [num_queries, bbox_pred_dim]
 * @param d_output_trajectories Device buffer for output trajectories [num_queries, traj_modes,
 * timesteps, 2]
 * @param d_output_traj_scores Device buffer for output trajectory scores [num_queries, traj_modes]
 * @param d_output_valid_flags Device buffer for valid object flags [num_queries]
 * @param d_output_max_class_indices Device buffer for max class indices [num_queries]
 * @param config Postprocessing configuration parameters
 * @param stream CUDA stream for asynchronous execution
 * @return cudaError_t CUDA error code
 */
cudaError_t launch_object_postprocess_kernel(
  const float * all_cls_scores_flat, const float * all_traj_preds_flat,
  const float * all_traj_cls_scores_flat, const float * all_bbox_preds_flat,
  float * d_output_cls_scores, float * d_output_bbox_preds, float * d_output_trajectories,
  float * d_output_traj_scores, int32_t * d_output_valid_flags,
  int32_t * d_output_max_class_indices, const ObjectPostprocessConfig & config,
  cudaStream_t stream);

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_KERNEL_HPP_
