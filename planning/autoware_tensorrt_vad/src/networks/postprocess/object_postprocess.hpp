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

#ifndef NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_HPP_
#define NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_HPP_

#include "data_types.hpp"
#include "networks/postprocess/object_postprocess_kernel.hpp"
#include "ros_vad_logger.hpp"

#include <cuda_runtime.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

/**
 * @struct ObjectPostprocessArgs
 * @brief Arguments bundle for object postprocessing to reduce function parameters
 */
struct ObjectPostprocessArgs
{
  const float * cls_scores{nullptr};           // Device pointer to classification scores
  const float * bbox_preds{nullptr};           // Device pointer to bbox predictions
  const float * trajectories{nullptr};         // Device pointer to trajectory data
  const float * traj_scores{nullptr};          // Device pointer to trajectory scores
  const int32_t * valid_flags{nullptr};        // Device pointer to valid object flags
  const int32_t * max_class_indices{nullptr};  // Device pointer to max class indices
  cudaStream_t stream{nullptr};                // CUDA stream for synchronization
};

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
  struct InferenceInputs
  {
    const float * cls_scores{nullptr};
    const float * traj_preds{nullptr};
    const float * traj_cls_scores{nullptr};
    const float * bbox_preds{nullptr};
  };

  // Template constructor to accept shared_ptr<LoggerType>
  template <typename LoggerType>
  ObjectPostprocessor(const ObjectPostprocessConfig & config, std::shared_ptr<LoggerType> logger);

  ~ObjectPostprocessor();

  // Prohibit copy constructor and copy assignment operator to prevent double deallocation
  ObjectPostprocessor(const ObjectPostprocessor &) = delete;
  ObjectPostprocessor & operator=(const ObjectPostprocessor &) = delete;

  /**
   * @brief CUDA-accelerated postprocessing for object predictions
   * @param inputs Device pointers bundle for object predictions
   * @param stream CUDA stream to use for execution
   * @return std::vector<BBox> Processed object bounding boxes
   */
  std::vector<autoware::tensorrt_vad::BBox> postprocess_objects(
    const InferenceInputs & inputs, cudaStream_t stream);

private:
  struct HostBuffers
  {
    HostBuffers(
      int32_t num_queries, int32_t num_classes, int32_t bbox_pred_dim, int32_t trajectory_modes,
      int32_t timesteps)
    : cls_scores(static_cast<size_t>(num_queries) * num_classes),
      bbox_preds(static_cast<size_t>(num_queries) * bbox_pred_dim),
      trajectories(static_cast<size_t>(num_queries) * trajectory_modes * timesteps * 2),
      traj_scores(static_cast<size_t>(num_queries) * trajectory_modes),
      valid_flags(static_cast<size_t>(num_queries)),
      max_class_indices(static_cast<size_t>(num_queries))
    {
    }

    std::vector<float> cls_scores;
    std::vector<float> bbox_preds;
    std::vector<float> trajectories;
    std::vector<float> traj_scores;
    std::vector<int32_t> valid_flags;
    std::vector<int32_t> max_class_indices;
  };

  /**
   * @brief Cleanup allocated CUDA memory resources.
   * Called when allocation fails or in destructor.
   */
  void cleanup_cuda_resources();

  /**
   * @brief Copy processed object results from device to host and create BBox objects
   * @param args Bundle of arguments containing device buffers and stream
   * @return std::vector<BBox> Processed object bounding boxes
   */
  std::vector<autoware::tensorrt_vad::BBox> copy_object_results_to_host(
    const ObjectPostprocessArgs & args);

  /**
   * @brief Copy device arrays to host memory
   * @param args Bundle of arguments containing device buffers and stream
   * @param buffers Host buffer container to receive device data
   * @return true if copy succeeded, false otherwise
   */
  bool copy_device_arrays_to_host(const ObjectPostprocessArgs & args, HostBuffers & buffers);

  /**
   * @brief Create a BBox object from GPU data for a single object
   * @param obj_idx Object index
   * @param buffers Host buffer container with CPU data
   * @return BBox object
   */
  autoware::tensorrt_vad::BBox create_bbox_from_gpu_data(
    int32_t obj_idx, const HostBuffers & buffers);

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

// Template implementations
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

#endif  // NETWORKS__POSTPROCESS__OBJECT_POSTPROCESS_HPP_
