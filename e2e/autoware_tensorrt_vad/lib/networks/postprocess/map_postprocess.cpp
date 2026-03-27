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

#include "../src/networks/postprocess/map_postprocess.hpp"

#include "../src/networks/postprocess/map_postprocess_kernel.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_vad
{

namespace
{
struct MapHostBuffers
{
  MapHostBuffers(int32_t num_queries, int32_t num_classes, int32_t points_per_polyline)
  : cls_scores(static_cast<size_t>(num_queries) * num_classes),
    points(static_cast<size_t>(num_queries) * points_per_polyline * 2),
    valid_flags(static_cast<size_t>(num_queries)),
    max_class_indices(static_cast<size_t>(num_queries))
  {
  }

  std::vector<float> cls_scores;
  std::vector<float> points;
  std::vector<int32_t> valid_flags;
  std::vector<int32_t> max_class_indices;
};

struct CopyContext
{
  cudaStream_t stream;
  const MapPostprocessConfig & config;
  const std::shared_ptr<VadLogger> & logger;
};

bool copy_device_to_host(
  const MapPostprocessor::MapDeviceBuffers & device_buffers, const CopyContext & context,
  MapHostBuffers & host_buffers)
{
  struct CopyOperation
  {
    void * dst;
    const void * src;
    size_t size_in_bytes;
    const char * name;
  };

  const size_t cls_scores_size = static_cast<size_t>(context.config.map_num_queries) *
                                 context.config.map_num_classes * sizeof(float);
  const size_t points_size = static_cast<size_t>(context.config.map_num_queries) *
                             context.config.map_points_per_polylines * 2 * sizeof(float);
  const size_t flags_size = static_cast<size_t>(context.config.map_num_queries) * sizeof(int32_t);

  const std::array<CopyOperation, 4> operations{{
    {host_buffers.cls_scores.data(), device_buffers.cls_scores, cls_scores_size, "cls_scores"},
    {host_buffers.points.data(), device_buffers.points, points_size, "points"},
    {host_buffers.valid_flags.data(), device_buffers.valid_flags, flags_size, "valid_flags"},
    {host_buffers.max_class_indices.data(), device_buffers.max_class_indices, flags_size,
     "max_class_indices"},
  }};

  for (const auto & op : operations) {
    if (!op.src) {
      context.logger->error(std::string("Device pointer for ") + op.name + " is null");
      return false;
    }

    cudaError_t result =
      cudaMemcpyAsync(op.dst, op.src, op.size_in_bytes, cudaMemcpyDeviceToHost, context.stream);
    if (result != cudaSuccess) {
      context.logger->error(
        "Failed to copy " + std::string(op.name) +
        " from device: " + std::string(cudaGetErrorString(result)));
      return false;
    }
  }

  const cudaError_t sync_result = cudaStreamSynchronize(context.stream);
  if (sync_result != cudaSuccess) {
    context.logger->error(
      "Stream synchronization failed: " + std::string(cudaGetErrorString(sync_result)));
    return false;
  }

  return true;
}

bool is_query_valid(
  int32_t query_idx, const MapHostBuffers & host_buffers, const MapPostprocessConfig & config)
{
  if (
    query_idx >= static_cast<int32_t>(host_buffers.valid_flags.size()) ||
    host_buffers.valid_flags.at(query_idx) == 0) {
    return false;
  }

  const int32_t max_class_idx = host_buffers.max_class_indices.at(query_idx);
  return max_class_idx >= 0 && max_class_idx < static_cast<int32_t>(config.map_class_names.size());
}

std::vector<std::vector<float>> extract_polyline_points(
  int32_t query_idx, const MapHostBuffers & host_buffers, int32_t points_per_polyline)
{
  std::vector<std::vector<float>> points(points_per_polyline, std::vector<float>(2));

  for (int32_t p = 0; p < points_per_polyline; ++p) {
    const int32_t base_idx = query_idx * points_per_polyline * 2 + p * 2;
    if (base_idx + 1 >= static_cast<int32_t>(host_buffers.points.size())) {
      break;
    }
    points[p][0] = host_buffers.points.at(base_idx);
    points[p][1] = host_buffers.points.at(base_idx + 1);
  }

  return points;
}

std::vector<MapPolyline> build_map_polylines(
  const MapHostBuffers & host_buffers, const MapPostprocessConfig & config)
{
  std::vector<MapPolyline> map_polylines;
  map_polylines.reserve(config.map_num_queries);

  for (int32_t query_idx = 0; query_idx < config.map_num_queries; ++query_idx) {
    if (!is_query_valid(query_idx, host_buffers, config)) {
      continue;
    }

    const int32_t max_class_idx = host_buffers.max_class_indices.at(query_idx);
    auto points = extract_polyline_points(query_idx, host_buffers, config.map_points_per_polylines);

    map_polylines.emplace_back(config.map_class_names.at(max_class_idx), std::move(points));
  }

  return map_polylines;
}
}  // namespace

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
  MapDeviceBuffers device_buffers{
    d_map_cls_scores_, d_map_points_, d_map_valid_flags_, d_map_max_class_indices_};
  auto result = copy_map_results_to_host(device_buffers, stream);

  logger_->debug(
    "CUDA map postprocessing completed with " + std::to_string(result.size()) + " polylines");

  return result;
}

std::vector<autoware::tensorrt_vad::MapPolyline> MapPostprocessor::copy_map_results_to_host(
  const MapDeviceBuffers & device_buffers, cudaStream_t stream)
{
  const int32_t num_queries = config_.map_num_queries;
  const int32_t num_classes = config_.map_num_classes;
  const int32_t points_per_polyline = config_.map_points_per_polylines;

  MapHostBuffers host_buffers(num_queries, num_classes, points_per_polyline);

  const CopyContext context{stream, config_, logger_};
  if (!copy_device_to_host(device_buffers, context, host_buffers)) {
    return {};
  }

  return build_map_polylines(host_buffers, config_);
}

}  // namespace autoware::tensorrt_vad
