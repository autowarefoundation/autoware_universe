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

#include "autoware/tensorrt_vad/networks/postprocess/object_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

// Note: Template constructor implementation is in the header file

ObjectPostprocessor::~ObjectPostprocessor()
{
  cleanup_cuda_resources();
}

void ObjectPostprocessor::cleanup_cuda_resources()
{
  if (d_obj_cls_scores_) {
    cudaFree(d_obj_cls_scores_);
    d_obj_cls_scores_ = nullptr;
  }
  if (d_obj_bbox_preds_) {
    cudaFree(d_obj_bbox_preds_);
    d_obj_bbox_preds_ = nullptr;
  }
  if (d_obj_trajectories_) {
    cudaFree(d_obj_trajectories_);
    d_obj_trajectories_ = nullptr;
  }
  if (d_obj_traj_scores_) {
    cudaFree(d_obj_traj_scores_);
    d_obj_traj_scores_ = nullptr;
  }
  if (d_obj_valid_flags_) {
    cudaFree(d_obj_valid_flags_);
    d_obj_valid_flags_ = nullptr;
  }
  if (d_obj_max_class_indices_) {
    cudaFree(d_obj_max_class_indices_);
    d_obj_max_class_indices_ = nullptr;
  }
}

std::vector<autoware::tensorrt_vad::BBox> ObjectPostprocessor::postprocess_objects(
  const float * all_cls_scores_flat, const float * all_traj_preds_flat,
  const float * all_traj_cls_scores_flat, const float * all_bbox_preds_flat, cudaStream_t stream)
{
  logger_->debug("Starting CUDA object postprocessing");

  // Launch CUDA kernel
  cudaError_t kernel_result = launch_object_postprocess_kernel(
    all_cls_scores_flat, all_traj_preds_flat, all_traj_cls_scores_flat, all_bbox_preds_flat,
    d_obj_cls_scores_, d_obj_bbox_preds_, d_obj_trajectories_, d_obj_traj_scores_,
    d_obj_valid_flags_, d_obj_max_class_indices_, config_, stream);

  if (kernel_result != cudaSuccess) {
    logger_->error(
      "Object postprocess kernel launch failed: " + std::string(cudaGetErrorString(kernel_result)));
    return {};
  }

  logger_->debug("Object postprocess kernel launched successfully");

  // Bundle arguments for copy function
  ObjectPostprocessArgs args{
    d_obj_cls_scores_,
    d_obj_bbox_preds_,
    d_obj_trajectories_,
    d_obj_traj_scores_,
    d_obj_valid_flags_,
    d_obj_max_class_indices_,
    stream};

  // Copy results from device to host and create BBox objects
  return copy_object_results_to_host(args);
}

std::vector<autoware::tensorrt_vad::BBox> ObjectPostprocessor::copy_object_results_to_host(
  const ObjectPostprocessArgs & args)
{
  logger_->debug("Copying object results from GPU to host");

  // Calculate buffer sizes
  const size_t cls_scores_size =
    static_cast<size_t>(config_.prediction_num_queries) * config_.prediction_num_classes;
  const size_t bbox_preds_size =
    static_cast<size_t>(config_.prediction_num_queries) * config_.prediction_bbox_pred_dim;
  const size_t trajectories_size = static_cast<size_t>(config_.prediction_num_queries) *
                                   config_.prediction_trajectory_modes *
                                   config_.prediction_timesteps * 2;
  const size_t traj_scores_size =
    static_cast<size_t>(config_.prediction_num_queries) * config_.prediction_trajectory_modes;
  const size_t valid_flags_size = static_cast<size_t>(config_.prediction_num_queries);

  // Allocate host memory
  std::vector<float> h_cls_scores(cls_scores_size);
  std::vector<float> h_bbox_preds(bbox_preds_size);
  std::vector<float> h_trajectories(trajectories_size);
  std::vector<float> h_traj_scores(traj_scores_size);
  std::vector<int32_t> h_valid_flags(valid_flags_size);
  std::vector<int32_t> h_max_class_indices(valid_flags_size);

  // Copy arrays from device to host
  if (!copy_device_arrays_to_host(
        args, h_cls_scores, h_bbox_preds, h_trajectories, h_traj_scores, h_valid_flags,
        h_max_class_indices)) {
    return {};
  }

  logger_->debug("Successfully copied object results from GPU to host");

  // Convert GPU results to BBox objects
  std::vector<autoware::tensorrt_vad::BBox> bboxes;
  bboxes.reserve(config_.prediction_num_queries);

  for (int32_t obj = 0; obj < config_.prediction_num_queries; ++obj) {
    // Skip invalid objects
    if (h_valid_flags.at(obj) == 0) {
      continue;
    }

    // Create BBox from GPU data
    bboxes.push_back(create_bbox_from_gpu_data(
      obj, h_cls_scores, h_bbox_preds, h_trajectories, h_traj_scores, h_max_class_indices));
  }

  logger_->debug(
    "Created " + std::to_string(bboxes.size()) + " valid BBox objects from GPU results");
  return bboxes;
}

bool ObjectPostprocessor::copy_device_arrays_to_host(
  const ObjectPostprocessArgs & args, std::vector<float> & h_cls_scores,
  std::vector<float> & h_bbox_preds, std::vector<float> & h_trajectories,
  std::vector<float> & h_traj_scores, std::vector<int32_t> & h_valid_flags,
  std::vector<int32_t> & h_max_class_indices)
{
  // Calculate buffer sizes
  const size_t cls_scores_size =
    static_cast<size_t>(config_.prediction_num_queries) * config_.prediction_num_classes;
  const size_t bbox_preds_size =
    static_cast<size_t>(config_.prediction_num_queries) * config_.prediction_bbox_pred_dim;
  const size_t trajectories_size = static_cast<size_t>(config_.prediction_num_queries) *
                                   config_.prediction_trajectory_modes *
                                   config_.prediction_timesteps * 2;
  const size_t traj_scores_size =
    static_cast<size_t>(config_.prediction_num_queries) * config_.prediction_trajectory_modes;
  const size_t valid_flags_size = static_cast<size_t>(config_.prediction_num_queries);

  // Structure to hold copy operations
  struct CopyOperation
  {
    void * dst;
    const void * src;
    size_t size_in_bytes;
    const char * name;
  };

  // Define all copy operations
  std::vector<CopyOperation> operations = {
    {h_cls_scores.data(), args.cls_scores, cls_scores_size * sizeof(float), "cls_scores"},
    {h_bbox_preds.data(), args.bbox_preds, bbox_preds_size * sizeof(float), "bbox_preds"},
    {h_trajectories.data(), args.trajectories, trajectories_size * sizeof(float), "trajectories"},
    {h_traj_scores.data(), args.traj_scores, traj_scores_size * sizeof(float), "traj_scores"},
    {h_valid_flags.data(), args.valid_flags, valid_flags_size * sizeof(int32_t), "valid_flags"},
    {h_max_class_indices.data(), args.max_class_indices, valid_flags_size * sizeof(int32_t),
     "max_class_indices"}};

  // Perform all copy operations
  for (const auto & op : operations) {
    cudaError_t result =
      cudaMemcpyAsync(op.dst, op.src, op.size_in_bytes, cudaMemcpyDeviceToHost, args.stream);
    if (result != cudaSuccess) {
      logger_->error(
        "Failed to copy " + std::string(op.name) +
        " from device: " + std::string(cudaGetErrorString(result)));
      return false;
    }
  }

  // Synchronize stream
  cudaError_t sync_result = cudaStreamSynchronize(args.stream);
  if (sync_result != cudaSuccess) {
    logger_->error(
      "CUDA stream synchronization failed: " + std::string(cudaGetErrorString(sync_result)));
    return false;
  }

  return true;
}

autoware::tensorrt_vad::BBox ObjectPostprocessor::create_bbox_from_gpu_data(
  int32_t obj_idx, const std::vector<float> & h_cls_scores, const std::vector<float> & h_bbox_preds,
  const std::vector<float> & h_trajectories, const std::vector<float> & h_traj_scores,
  const std::vector<int32_t> & h_max_class_indices)
{
  // Create BBox with dynamic trajectory modes and timesteps from config
  autoware::tensorrt_vad::BBox bbox(
    config_.prediction_trajectory_modes, config_.prediction_timesteps);

  // Copy bbox predictions
  for (int32_t i = 0; i < config_.prediction_bbox_pred_dim; ++i) {
    bbox.bbox.at(i) = h_bbox_preds.at(obj_idx * config_.prediction_bbox_pred_dim + i);
  }

  // Get max class index and confidence
  const int32_t max_class = h_max_class_indices.at(obj_idx);
  float max_score = 0.0f;
  if (max_class >= 0 && max_class < config_.prediction_num_classes) {
    max_score = h_cls_scores.at(obj_idx * config_.prediction_num_classes + max_class);
  }

  bbox.confidence = max_score;
  bbox.object_class = max_class;

  // Copy trajectory predictions
  for (int32_t mode = 0; mode < config_.prediction_trajectory_modes; ++mode) {
    bbox.trajectories[mode].confidence =
      h_traj_scores.at(obj_idx * config_.prediction_trajectory_modes + mode);

    // Copy trajectory points
    for (int32_t ts = 0; ts < config_.prediction_timesteps; ++ts) {
      const int32_t traj_idx =
        obj_idx * config_.prediction_trajectory_modes * config_.prediction_timesteps * 2 +
        mode * config_.prediction_timesteps * 2 + ts * 2;
      bbox.trajectories[mode].trajectory[ts][0] = h_trajectories.at(traj_idx);      // x
      bbox.trajectories[mode].trajectory[ts][1] = h_trajectories.at(traj_idx + 1);  // y
    }
  }

  return bbox;
}

}  // namespace autoware::tensorrt_vad
