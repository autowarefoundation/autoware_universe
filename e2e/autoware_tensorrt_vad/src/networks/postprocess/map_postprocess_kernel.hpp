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

#ifndef NETWORKS__POSTPROCESS__MAP_POSTPROCESS_KERNEL_HPP_
#define NETWORKS__POSTPROCESS__MAP_POSTPROCESS_KERNEL_HPP_

#include <cuda_runtime.h>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

/**
 * @struct MapPostprocessConfig
 * @brief Configuration parameters required for map CUDA postprocessing
 */
struct MapPostprocessConfig
{
  int32_t map_num_queries;
  int32_t map_num_classes;
  int32_t map_points_per_polylines;
  int32_t num_decoder_layers;
  std::array<float, 6> detection_range;  // [x_min, y_min, z_min, x_max, y_max, z_max]
  std::vector<std::string> map_class_names;
  std::map<std::string, float> map_confidence_thresholds;

  // Additional members for CUDA kernel (computed at runtime)
  int32_t map_class_count;
  float detection_range_array[6];  // Flat array version for CUDA
  float
    map_confidence_thresholds_array[16];  // Flat array version for CUDA (configurable max classes)

  // Helper method to prepare kernel data
  void prepare_for_kernel()
  {
    map_class_count = static_cast<int32_t>(map_class_names.size());

    // Copy detection_range to flat array
    for (int32_t i = 0; i < 6; ++i) {
      detection_range_array[i] = detection_range[i];
    }

    // Initialize confidence thresholds array
    for (int32_t i = 0; i < 16; ++i) {
      map_confidence_thresholds_array[i] = 0.0f;
    }

    // Copy confidence thresholds to flat array
    for (int32_t i = 0; i < static_cast<int32_t>(map_class_names.size()) && i < 16; ++i) {
      const std::string & class_name = map_class_names[i];
      auto it = map_confidence_thresholds.find(class_name);
      if (it != map_confidence_thresholds.end()) {
        map_confidence_thresholds_array[i] = it->second;
      }
    }
  }
};

/**
 * @brief Launch CUDA kernel to postprocess map predictions
 * @param map_cls_preds_flat Flat array of map classification scores
 * @param map_pts_preds_flat Flat array of map point predictions
 * @param d_output_cls_scores Device buffer for output classification scores [num_queries,
 * num_classes]
 * @param d_output_points Device buffer for output denormalized points [num_queries,
 * points_per_polylines, 2]
 * @param d_output_valid_flags Device buffer for valid polyline flags [num_queries]
 * @param d_output_max_class_indices Device buffer for max class indices [num_queries]
 * @param config Postprocessing configuration parameters (will be converted internally)
 * @param stream CUDA stream for asynchronous execution
 * @return cudaError_t CUDA error code
 */
cudaError_t launch_map_postprocess_kernel(
  const float * map_cls_preds_flat, const float * map_pts_preds_flat, float * d_output_cls_scores,
  float * d_output_points, int32_t * d_output_valid_flags, int32_t * d_output_max_class_indices,
  const MapPostprocessConfig & config, cudaStream_t stream);

}  // namespace autoware::tensorrt_vad

#endif  // NETWORKS__POSTPROCESS__MAP_POSTPROCESS_KERNEL_HPP_
