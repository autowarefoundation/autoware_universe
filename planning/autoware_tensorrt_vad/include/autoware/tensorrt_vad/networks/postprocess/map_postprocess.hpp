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

#ifndef AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__MAP_POSTPROCESS_HPP_
#define AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__MAP_POSTPROCESS_HPP_

#include "autoware/tensorrt_vad/data_types.hpp"
#include "autoware/tensorrt_vad/networks/postprocess/map_postprocess_kernel.hpp"
#include "autoware/tensorrt_vad/ros_vad_logger.hpp"

#include <cuda_runtime.h>

#include <memory>
#include <vector>

namespace autoware::tensorrt_vad
{

/**
 * @class MapPostprocessor
 * @brief GPU postprocessing pipeline for map predictions
 *
 * This class encapsulates CUDA-based postprocessing for map polylines,
 * replacing the CPU-based postprocess_map_preds function.
 * Resources are allocated in constructor (RAII) and released in destructor.
 */
class MapPostprocessor
{
public:
  // Template constructor to accept shared_ptr<LoggerType>
  template <typename LoggerType>
  MapPostprocessor(const MapPostprocessConfig & config, std::shared_ptr<LoggerType> logger);

  ~MapPostprocessor();

  // Prohibit copy constructor and copy assignment operator to prevent double deallocation
  MapPostprocessor(const MapPostprocessor &) = delete;
  MapPostprocessor & operator=(const MapPostprocessor &) = delete;

  /**
   * @brief CUDA-accelerated postprocessing for map predictions
   * @param map_all_cls_preds_flat Device pointer to flat map classification scores
   * @param map_all_pts_preds_flat Device pointer to flat map point predictions
   * @param stream CUDA stream to use for execution
   * @return std::vector<MapPolyline> Processed map polylines
   */
  std::vector<autoware::tensorrt_vad::MapPolyline> postprocess_map_preds(
    const float * map_all_cls_preds_flat, const float * map_all_pts_preds_flat,
    cudaStream_t stream);

private:
  /**
   * @brief Cleanup allocated CUDA memory resources.
   * Called when allocation fails or in destructor.
   */
  void cleanup_cuda_resources();

  /**
   * @brief Copy processed map results from device to host and create MapPolyline objects
   * @param d_cls_scores Device buffer with classification scores
   * @param d_points Device buffer with denormalized points
   * @param d_valid_flags Device buffer with valid polyline flags
   * @param d_max_class_indices Device buffer with max class indices for each query
   * @param stream CUDA stream for synchronization
   * @return std::vector<MapPolyline> Processed map polylines
   */
  std::vector<autoware::tensorrt_vad::MapPolyline> copy_map_results_to_host(
    const float * d_cls_scores, const float * d_points, const int32_t * d_valid_flags,
    const int32_t * d_max_class_indices, cudaStream_t stream);

  MapPostprocessConfig config_;
  std::shared_ptr<autoware::tensorrt_vad::VadLogger> logger_;  // Direct VadLogger pointer

  // --- GPU Buffers for Map Processing ---
  float * d_map_cls_scores_{nullptr};           // [num_queries, num_classes]
  float * d_map_points_{nullptr};               // [num_queries, points_per_polylines, 2]
  int32_t * d_map_valid_flags_{nullptr};        // [num_queries]
  int32_t * d_map_max_class_indices_{nullptr};  // [num_queries] - max class index for each query
};

// Template method implementations (must be in header for templates)
template <typename LoggerType>
MapPostprocessor::MapPostprocessor(
  const MapPostprocessConfig & config, std::shared_ptr<LoggerType> logger)
: config_(config), logger_(std::static_pointer_cast<autoware::tensorrt_vad::VadLogger>(logger))
{
  // Logger accepts only classes that inherit from VadLogger
  static_assert(
    std::is_base_of_v<autoware::tensorrt_vad::VadLogger, LoggerType>,
    "LoggerType must be VadLogger or derive from VadLogger.");

  // Prepare kernel data once at construction time
  config_.prepare_for_kernel();

  logger_->debug(
    "MapPostprocessor config: map_queries=" + std::to_string(config_.map_num_queries) +
    ", map_classes=" + std::to_string(config_.map_num_classes) +
    ", points_per_polyline=" + std::to_string(config_.map_points_per_polylines));

  // --- Allocate Map Processing Buffers ---
  const size_t map_cls_scores_size =
    static_cast<size_t>(config_.map_num_queries) * config_.map_num_classes * sizeof(float);
  const size_t map_points_size = static_cast<size_t>(config_.map_num_queries) *
                                 config_.map_points_per_polylines * 2 * sizeof(float);
  const size_t map_flags_size = static_cast<size_t>(config_.map_num_queries) * sizeof(int32_t);

  cudaError_t err = cudaMalloc(&d_map_cls_scores_, map_cls_scores_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate map cls_scores buffer of size " + std::to_string(map_cls_scores_size) +
      ": " + cudaGetErrorString(err));
    return;
  }

  err = cudaMalloc(&d_map_points_, map_points_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate map points buffer of size " + std::to_string(map_points_size) + ": " +
      cudaGetErrorString(err));
    cleanup_cuda_resources();
    return;
  }

  err = cudaMalloc(&d_map_valid_flags_, map_flags_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate map valid flags buffer: " + std::string(cudaGetErrorString(err)));
    cleanup_cuda_resources();
    return;
  }

  // Allocate max class indices buffer
  const size_t max_class_indices_size =
    static_cast<size_t>(config_.map_num_queries) * sizeof(int32_t);
  err = cudaMalloc(&d_map_max_class_indices_, max_class_indices_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate map max class indices buffer: " + std::string(cudaGetErrorString(err)));
    cleanup_cuda_resources();
    return;
  }

  logger_->info("MapPostprocessor initialized successfully");
}

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__MAP_POSTPROCESS_HPP_
