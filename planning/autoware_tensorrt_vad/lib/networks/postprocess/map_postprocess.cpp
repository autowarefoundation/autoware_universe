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

#include "autoware/tensorrt_vad/networks/postprocess/map_postprocess.hpp"

#include "autoware/tensorrt_vad/networks/postprocess/map_postprocess_kernel.hpp"

#include <cmath>
#include <stdexcept>

namespace autoware::tensorrt_vad
{

MapPostprocessor::~MapPostprocessor()
{
  cleanup_cuda_resources();
}

void MapPostprocessor::cleanup_cuda_resources()
{
  if (d_map_cls_scores_) {
    cudaFree(d_map_cls_scores_);
    d_map_cls_scores_ = nullptr;
  }
  if (d_map_points_) {
    cudaFree(d_map_points_);
    d_map_points_ = nullptr;
  }
  if (d_map_valid_flags_) {
    cudaFree(d_map_valid_flags_);
    d_map_valid_flags_ = nullptr;
  }
  if (d_map_max_class_indices_) {
    cudaFree(d_map_max_class_indices_);
    d_map_max_class_indices_ = nullptr;
  }
}

std::vector<autoware::tensorrt_vad::MapPolyline> MapPostprocessor::postprocess_map_preds(
  const float * map_all_cls_preds_flat, const float * map_all_pts_preds_flat, cudaStream_t stream)
{
  logger_->debug("Starting CUDA map postprocessing");

  // Launch CUDA kernel
  cudaError_t err = launch_map_postprocess_kernel(
    map_all_cls_preds_flat, map_all_pts_preds_flat, d_map_cls_scores_, d_map_points_,
    d_map_valid_flags_, d_map_max_class_indices_, config_, stream);

  if (err != cudaSuccess) {
    logger_->error("Map postprocess kernel launch failed: " + std::string(cudaGetErrorString(err)));
    return {};
  }

  // Copy results from device to host and create MapPolyline objects
  auto result = copy_map_results_to_host(
    d_map_cls_scores_, d_map_points_, d_map_valid_flags_, d_map_max_class_indices_, stream);

  logger_->debug(
    "CUDA map postprocessing completed with " + std::to_string(result.size()) + " polylines");

  return result;
}

std::vector<autoware::tensorrt_vad::MapPolyline> MapPostprocessor::copy_map_results_to_host(
  const float * d_cls_scores, const float * d_points, const int32_t * d_valid_flags,
  const int32_t * d_max_class_indices, cudaStream_t stream)
{
  const int32_t num_queries = config_.map_num_queries;
  const int32_t num_classes = config_.map_num_classes;
  const int32_t points_per_polyline = config_.map_points_per_polylines;

  // Allocate host memory
  std::vector<float> h_cls_scores(num_queries * num_classes);
  std::vector<float> h_points(num_queries * points_per_polyline * 2);
  std::vector<int32_t> h_valid_flags(num_queries);
  std::vector<int32_t> h_max_class_indices(num_queries);

  // Copy from device to host
  cudaError_t err = cudaMemcpyAsync(
    h_cls_scores.data(), d_cls_scores, h_cls_scores.size() * sizeof(float), cudaMemcpyDeviceToHost,
    stream);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to copy cls_scores from device: " + std::string(cudaGetErrorString(err)));
    return {};
  }

  err = cudaMemcpyAsync(
    h_points.data(), d_points, h_points.size() * sizeof(float), cudaMemcpyDeviceToHost, stream);
  if (err != cudaSuccess) {
    logger_->error("Failed to copy points from device: " + std::string(cudaGetErrorString(err)));
    return {};
  }

  err = cudaMemcpyAsync(
    h_valid_flags.data(), d_valid_flags, h_valid_flags.size() * sizeof(int32_t),
    cudaMemcpyDeviceToHost, stream);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to copy valid_flags from device: " + std::string(cudaGetErrorString(err)));
    return {};
  }

  err = cudaMemcpyAsync(
    h_max_class_indices.data(), d_max_class_indices, h_max_class_indices.size() * sizeof(int32_t),
    cudaMemcpyDeviceToHost, stream);
  if (err != cudaSuccess) {
    logger_->error(
      "Failed to copy max_class_indices from device: " + std::string(cudaGetErrorString(err)));
    return {};
  }

  // Wait for all copies to complete
  err = cudaStreamSynchronize(stream);
  if (err != cudaSuccess) {
    logger_->error("Stream synchronization failed: " + std::string(cudaGetErrorString(err)));
    return {};
  }

  // Create MapPolyline objects
  std::vector<autoware::tensorrt_vad::MapPolyline> map_polylines;
  map_polylines.reserve(num_queries);

  for (int32_t query_idx = 0; query_idx < num_queries; ++query_idx) {
    if (
      query_idx >= static_cast<int32_t>(h_valid_flags.size()) || h_valid_flags.at(query_idx) == 0) {
      continue;  // Skip invalid polylines
    }

    // Get max class index directly from kernel output (no need for CPU-side max finding)
    const int32_t max_class_idx = h_max_class_indices.at(query_idx);

    // Get class name
    if (max_class_idx >= static_cast<int32_t>(config_.map_class_names.size())) {
      continue;  // Skip unknown classes
    }

    const std::string & class_name = config_.map_class_names.at(max_class_idx);

    // Extract points for this polyline
    std::vector<std::vector<float>> points(points_per_polyline, std::vector<float>(2));

    for (int32_t p = 0; p < points_per_polyline; ++p) {
      const int32_t point_base_idx = query_idx * points_per_polyline * 2 + p * 2;
      if (point_base_idx + 1 >= static_cast<int32_t>(h_points.size())) {
        break;
      }
      points[p][0] = h_points.at(point_base_idx);      // x coordinate
      points[p][1] = h_points.at(point_base_idx + 1);  // y coordinate
    }

    // Create MapPolyline object
    map_polylines.emplace_back(class_name, points);
  }

  return map_polylines;
}

}  // namespace autoware::tensorrt_vad
