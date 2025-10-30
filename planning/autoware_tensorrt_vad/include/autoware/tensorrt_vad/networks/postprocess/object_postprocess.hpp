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

#ifndef AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_HPP_
#define AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_HPP_

#include "autoware/tensorrt_vad/data_types.hpp"
#include "autoware/tensorrt_vad/networks/postprocess/object_postprocess_kernel.hpp"
#include "autoware/tensorrt_vad/ros_vad_logger.hpp"

#include <cuda_runtime.h>

#include <memory>
#include <vector>

namespace autoware::tensorrt_vad
{

/**
 * @class ObjectPostprocessor
 * @brief GPU postprocessing pipeline for object predictions
 *
 * This class encapsulates CUDA-based postprocessing for object detection,
 * replacing the CPU-based postprocess_bboxes function.
 * Resources are allocated in constructor (RAII) and released in destructor.
 */
class ObjectPostprocessor
{
public:
  // Template constructor to accept shared_ptr<LoggerType>
  template <typename LoggerType>
  ObjectPostprocessor(const ObjectPostprocessConfig & config, std::shared_ptr<LoggerType> logger);

  ~ObjectPostprocessor();

  // Prohibit copy constructor and copy assignment operator to prevent double deallocation
  ObjectPostprocessor(const ObjectPostprocessor &) = delete;
  ObjectPostprocessor & operator=(const ObjectPostprocessor &) = delete;

  /**
   * @brief CUDA-accelerated postprocessing for object predictions
   * @param all_cls_scores_flat Device pointer to flat object classification scores
   * @param all_traj_preds_flat Device pointer to flat trajectory predictions
   * @param all_traj_cls_scores_flat Device pointer to flat trajectory classification scores
   * @param all_bbox_preds_flat Device pointer to flat bounding box predictions
   * @param stream CUDA stream to use for execution
   * @return std::vector<BBox> Processed object bounding boxes
   */
  std::vector<autoware::tensorrt_vad::BBox> postprocess_objects(
    const float * all_cls_scores_flat, const float * all_traj_preds_flat,
    const float * all_traj_cls_scores_flat, const float * all_bbox_preds_flat, cudaStream_t stream);

private:
  /**
   * @brief Cleanup allocated CUDA memory resources.
   * Called when allocation fails or in destructor.
   */
  void cleanup_cuda_resources();

  /**
   * @brief Copy processed object results from device to host and create BBox objects
   * @param d_cls_scores Device buffer with classification scores
   * @param d_bbox_preds Device buffer with bbox predictions
   * @param d_trajectories Device buffer with trajectory data
   * @param d_traj_scores Device buffer with trajectory scores
   * @param d_valid_flags Device buffer with valid object flags
   * @param d_max_class_indices Device buffer with max class indices for each object
   * @param stream CUDA stream for synchronization
   * @return std::vector<BBox> Processed object bounding boxes
   */
  std::vector<autoware::tensorrt_vad::BBox> copy_object_results_to_host(
    const float * d_cls_scores, const float * d_bbox_preds, const float * d_trajectories,
    const float * d_traj_scores, const int32_t * d_valid_flags, const int32_t * d_max_class_indices,
    cudaStream_t stream);

  ObjectPostprocessConfig config_;
  std::shared_ptr<autoware::tensorrt_vad::VadLogger> logger_;  // Direct VadLogger pointer

  // --- GPU Buffers for Object Processing ---
  float * d_obj_cls_scores_{nullptr};           // [num_queries, num_classes]
  float * d_obj_bbox_preds_{nullptr};           // [num_queries, bbox_pred_dim]
  float * d_obj_trajectories_{nullptr};         // [num_queries, traj_modes, timesteps, 2]
  float * d_obj_traj_scores_{nullptr};          // [num_queries, traj_modes]
  int32_t * d_obj_valid_flags_{nullptr};        // [num_queries]
  int32_t * d_obj_max_class_indices_{nullptr};  // [num_queries] - max class index for each object
};

// Template method implementations (must be in header for templates)
template <typename LoggerType>
ObjectPostprocessor::ObjectPostprocessor(
  const ObjectPostprocessConfig & config, std::shared_ptr<LoggerType> logger)
: config_(config), logger_(std::static_pointer_cast<autoware::tensorrt_vad::VadLogger>(logger))
{
  // Logger accepts only classes that inherit from VadLogger
  static_assert(
    std::is_base_of_v<autoware::tensorrt_vad::VadLogger, LoggerType>,
    "LoggerType must be VadLogger or derive from VadLogger.");

  // Prepare kernel data once at construction time
  config_.prepare_for_kernel();

  logger_->debug(
    "ObjectPostprocessor config: queries=" + std::to_string(config_.prediction_num_queries) +
    ", classes=" + std::to_string(config_.prediction_num_classes) +
    ", bbox_dim=" + std::to_string(config_.prediction_bbox_pred_dim));

  // --- Allocate Object Processing Buffers ---
  const size_t cls_scores_size = static_cast<size_t>(config_.prediction_num_queries) *
                                 config_.prediction_num_classes * sizeof(float);
  const size_t bbox_preds_size = static_cast<size_t>(config_.prediction_num_queries) *
                                 config_.prediction_bbox_pred_dim * sizeof(float);
  const size_t trajectories_size = static_cast<size_t>(config_.prediction_num_queries) *
                                   config_.prediction_trajectory_modes *
                                   config_.prediction_timesteps * 2 * sizeof(float);
  const size_t traj_scores_size = static_cast<size_t>(config_.prediction_num_queries) *
                                  config_.prediction_trajectory_modes * sizeof(float);
  const size_t valid_flags_size =
    static_cast<size_t>(config_.prediction_num_queries) * sizeof(int32_t);

  cudaError_t err = cudaMalloc(&d_obj_cls_scores_, cls_scores_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate object cls_scores buffer of size " + std::to_string(cls_scores_size) +
      ": " + cudaGetErrorString(err));
    return;
  }

  err = cudaMalloc(&d_obj_bbox_preds_, bbox_preds_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate object bbox_preds buffer of size " + std::to_string(bbox_preds_size) +
      ": " + cudaGetErrorString(err));
    cleanup_cuda_resources();
    return;
  }

  err = cudaMalloc(&d_obj_trajectories_, trajectories_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate object trajectories buffer of size " + std::to_string(trajectories_size) +
      ": " + cudaGetErrorString(err));
    cleanup_cuda_resources();
    return;
  }

  err = cudaMalloc(&d_obj_traj_scores_, traj_scores_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate object traj_scores buffer of size " + std::to_string(traj_scores_size) +
      ": " + cudaGetErrorString(err));
    cleanup_cuda_resources();
    return;
  }

  err = cudaMalloc(&d_obj_valid_flags_, valid_flags_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate object valid flags buffer: " + std::string(cudaGetErrorString(err)));
    cleanup_cuda_resources();
    return;
  }

  // Allocate max class indices buffer
  const size_t max_class_indices_size =
    static_cast<size_t>(config_.prediction_num_queries) * sizeof(int32_t);
  err = cudaMalloc(&d_obj_max_class_indices_, max_class_indices_size);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to allocate object max class indices buffer: " +
      std::string(cudaGetErrorString(err)));
    cleanup_cuda_resources();
    return;
  }

  logger_->info("ObjectPostprocessor initialized successfully");
}

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_HPP_
