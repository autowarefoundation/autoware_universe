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

#include "../src/networks/postprocess/object_postprocess.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

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
  const InferenceInputs & inputs, cudaStream_t stream)
{
  logger_->debug("Starting CUDA object postprocessing");

  // Launch CUDA kernel
  cudaError_t kernel_result = launch_object_postprocess_kernel(
    inputs.cls_scores, inputs.traj_preds, inputs.traj_cls_scores, inputs.bbox_preds,
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

  HostBuffers host_buffers(
    config_.prediction_num_queries, config_.prediction_num_classes,
    config_.prediction_bbox_pred_dim, config_.prediction_trajectory_modes,
    config_.prediction_timesteps);

  // Copy arrays from device to host
  if (!copy_device_arrays_to_host(args, host_buffers)) {
    return {};
  }

  logger_->debug("Successfully copied object results from GPU to host");

  // Convert GPU results to BBox objects
  std::vector<autoware::tensorrt_vad::BBox> bboxes;
  bboxes.reserve(config_.prediction_num_queries);

  for (int32_t obj = 0; obj < config_.prediction_num_queries; ++obj) {
    // Skip invalid objects
    if (host_buffers.valid_flags.at(obj) == 0) {
      continue;
    }

    // Create BBox from GPU data
    bboxes.push_back(create_bbox_from_gpu_data(obj, host_buffers));
  }

  logger_->debug(
    "Created " + std::to_string(bboxes.size()) + " valid BBox objects from GPU results");
  return bboxes;
}

bool ObjectPostprocessor::copy_device_arrays_to_host(
  const ObjectPostprocessArgs & args, HostBuffers & buffers)
{
  // Structure to hold copy operations
  struct CopyOperation
  {
    void * dst;
    const void * src;
    size_t size_in_bytes;
    const char * name;
  };

  // Define all copy operations
  const std::array<CopyOperation, 6> operations{{
    {buffers.cls_scores.data(), args.cls_scores, buffers.cls_scores.size() * sizeof(float),
     "cls_scores"},
    {buffers.bbox_preds.data(), args.bbox_preds, buffers.bbox_preds.size() * sizeof(float),
     "bbox_preds"},
    {buffers.trajectories.data(), args.trajectories, buffers.trajectories.size() * sizeof(float),
     "trajectories"},
    {buffers.traj_scores.data(), args.traj_scores, buffers.traj_scores.size() * sizeof(float),
     "traj_scores"},
    {buffers.valid_flags.data(), args.valid_flags, buffers.valid_flags.size() * sizeof(int32_t),
     "valid_flags"},
    {buffers.max_class_indices.data(), args.max_class_indices,
     buffers.max_class_indices.size() * sizeof(int32_t), "max_class_indices"},
  }};

  // Perform all copy operations
  for (const auto & op : operations) {
    if (!op.src) {
      logger_->error(std::string("Device pointer for ") + op.name + " is null");
      return false;
    }

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
  int32_t obj_idx, const HostBuffers & buffers)
{
  // Create BBox with dynamic trajectory modes and timesteps from config
  autoware::tensorrt_vad::BBox bbox(
    config_.prediction_trajectory_modes, config_.prediction_timesteps);

  // Copy bbox predictions
  for (int32_t i = 0; i < config_.prediction_bbox_pred_dim; ++i) {
    bbox.bbox.at(i) = buffers.bbox_preds.at(obj_idx * config_.prediction_bbox_pred_dim + i);
  }

  // Get max class index and confidence
  const int32_t max_class = buffers.max_class_indices.at(obj_idx);
  float max_score = 0.0f;
  if (max_class >= 0 && max_class < config_.prediction_num_classes) {
    max_score = buffers.cls_scores.at(obj_idx * config_.prediction_num_classes + max_class);
  }

  bbox.confidence = max_score;
  bbox.object_class = max_class;

  // Copy trajectory predictions
  for (int32_t mode = 0; mode < config_.prediction_trajectory_modes; ++mode) {
    bbox.trajectories[mode].confidence =
      buffers.traj_scores.at(obj_idx * config_.prediction_trajectory_modes + mode);

    // Copy trajectory points
    for (int32_t ts = 0; ts < config_.prediction_timesteps; ++ts) {
      const int32_t traj_idx =
        obj_idx * config_.prediction_trajectory_modes * config_.prediction_timesteps * 2 +
        mode * config_.prediction_timesteps * 2 + ts * 2;
      bbox.trajectories[mode].trajectory[ts][0] = buffers.trajectories.at(traj_idx);      // x
      bbox.trajectories[mode].trajectory[ts][1] = buffers.trajectories.at(traj_idx + 1);  // y
    }
  }

  return bbox;
}

}  // namespace autoware::tensorrt_vad
