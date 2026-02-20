// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/lidar_frnet/point_type.hpp"
#include "autoware/lidar_frnet/postprocess_kernel.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cuda_runtime.h>

#include <cstdint>

namespace autoware::lidar_frnet
{

__constant__ uint32_t const_num_classes;
__constant__ uint32_t const_num_filter_classes;
__constant__ uint32_t const_filter_class_indices[16];
__constant__ float const_filter_class_confidence_threshold;
__constant__ float const_palette[64];

PostprocessCuda::PostprocessCuda(const utils::PostprocessingParams & params, cudaStream_t stream)
: stream_(stream)
{
  auto num_classes = params.palette.size();
  auto num_filter_classes = params.filter_class_indices.size();
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_num_classes, &num_classes, sizeof(uint32_t), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_num_filter_classes, &num_filter_classes, sizeof(uint32_t), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_filter_class_indices, params.filter_class_indices.data(),
    sizeof(uint32_t) * num_filter_classes, 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_filter_class_confidence_threshold, &params.filter_class_confidence_threshold,
    sizeof(float), 0, cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyToSymbol(
    const_palette, params.palette.data(), num_classes * sizeof(float), 0, cudaMemcpyHostToDevice));
}

/// @brief Fill point from compact xyzi buffer (for interpolated points without original PointT)
template <typename PointT>
__device__ void set_point_from_xyzi(PointT & point, const float * points_xyzi, uint32_t point_idx);

template <>
__device__ void set_point_from_xyzi<InputPointTypeXYZI>(
  InputPointTypeXYZI & point, const float * points_xyzi, uint32_t point_idx)
{
  const uint32_t base = point_idx * 4;
  point.x = points_xyzi[base + 0];
  point.y = points_xyzi[base + 1];
  point.z = points_xyzi[base + 2];
  point.intensity = points_xyzi[base + 3];
}

template <>
__device__ void set_point_from_xyzi<InputPointTypeXYZIRC>(
  InputPointTypeXYZIRC & point, const float * points_xyzi, uint32_t point_idx)
{
  const uint32_t base = point_idx * 4;
  point.x = points_xyzi[base + 0];
  point.y = points_xyzi[base + 1];
  point.z = points_xyzi[base + 2];
  point.intensity = static_cast<std::uint8_t>(points_xyzi[base + 3]);
  point.return_type = 0;
  point.channel = 0;
}

template <>
__device__ void set_point_from_xyzi<InputPointTypeXYZIRADRT>(
  InputPointTypeXYZIRADRT & point, const float * points_xyzi, uint32_t point_idx)
{
  const uint32_t base = point_idx * 4;
  point.x = points_xyzi[base + 0];
  point.y = points_xyzi[base + 1];
  point.z = points_xyzi[base + 2];
  point.intensity = points_xyzi[base + 3];
  point.ring = 0;
  point.azimuth = 0.0f;
  point.distance = 0.0f;
  point.return_type = 0;
  point.time_stamp = 0.0;
}

template <>
__device__ void set_point_from_xyzi<InputPointTypeXYZIRCAEDT>(
  InputPointTypeXYZIRCAEDT & point, const float * points_xyzi, uint32_t point_idx)
{
  const uint32_t base = point_idx * 4;
  point.x = points_xyzi[base + 0];
  point.y = points_xyzi[base + 1];
  point.z = points_xyzi[base + 2];
  point.intensity = static_cast<std::uint8_t>(points_xyzi[base + 3]);
  point.return_type = 0;
  point.channel = 0;
  point.azimuth = 0.0f;
  point.elevation = 0.0f;
  point.distance = 0.0f;
  point.time_stamp = 0u;
}

template <typename PointT>
__global__ void fill_cloud_kernel(
  const float * points_xyzi, const PointT * cloud_compact,
  const uint32_t num_points_after_projection, const float * seg_logit, const uint32_t num_points,
  const bool active_comm_seg, const bool active_comm_viz, const bool active_comm_filtered,
  uint32_t * output_num_points_filtered, OutputSegmentationPointType * output_cloud_seg,
  OutputVisualizationPointType * output_cloud_viz, PointT * output_cloud_filtered)
{
  uint32_t point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= num_points) {
    return;
  }

  const float x = points_xyzi[point_idx * 4 + 0];
  const float y = points_xyzi[point_idx * 4 + 1];
  const float z = points_xyzi[point_idx * 4 + 2];
  const uint32_t pred_idx = point_idx * const_num_classes;

  float best_score = -1e9;
  uint32_t class_id = const_num_classes - 1;

  // Find the best score and class_id
  for (uint32_t i = 0; i < const_num_classes; i++) {
    float score = seg_logit[pred_idx + i];
    if (score > best_score) {
      best_score = score;
      class_id = i;
    }
  }

  if (active_comm_seg) {
    output_cloud_seg[point_idx].x = x;
    output_cloud_seg[point_idx].y = y;
    output_cloud_seg[point_idx].z = z;
    output_cloud_seg[point_idx].class_id = static_cast<uint8_t>(class_id);
  }

  if (active_comm_viz) {
    output_cloud_viz[point_idx].x = x;
    output_cloud_viz[point_idx].y = y;
    output_cloud_viz[point_idx].z = z;
    output_cloud_viz[point_idx].rgb = const_palette[class_id];
  }

  if (active_comm_filtered) {
    bool is_filtered = false;
    for (uint32_t i = 0; i < const_num_filter_classes; ++i) {
      uint32_t class_idx = const_filter_class_indices[i];
      float prob = seg_logit[pred_idx + class_idx];
      if (prob >= const_filter_class_confidence_threshold) {
        is_filtered = true;
        break;
      }
    }
    if (!is_filtered) {
      const uint32_t append_idx = atomicAdd(output_num_points_filtered, 1);
      if (cloud_compact != nullptr && point_idx < num_points_after_projection) {
        output_cloud_filtered[append_idx] = cloud_compact[point_idx];
      } else {
        set_point_from_xyzi(output_cloud_filtered[append_idx], points_xyzi, point_idx);
      }
    }
  }
}

template <typename PointT>
cudaError_t PostprocessCuda::fillCloud_launch_impl(
  const float * points_xyzi, const PointT * cloud_compact,
  const uint32_t num_points_after_projection, const float * seg_logit, const uint32_t num_points,
  const utils::ActiveComm & active_comm, uint32_t * output_num_points_filtered,
  OutputSegmentationPointType * output_cloud_seg, OutputVisualizationPointType * output_cloud_viz,
  PointT * output_cloud_filtered)
{
  dim3 block(utils::divup(num_points, utils::kernel_1d_size));
  dim3 threads(utils::kernel_1d_size);

  fill_cloud_kernel<<<block, threads, 0, stream_>>>(
    points_xyzi, cloud_compact, num_points_after_projection, seg_logit, num_points, active_comm.seg,
    active_comm.viz, active_comm.filtered, output_num_points_filtered, output_cloud_seg,
    output_cloud_viz, output_cloud_filtered);

  return cudaGetLastError();
}

// Explicit instantiations
template cudaError_t PostprocessCuda::fillCloud_launch_impl<InputPointTypeXYZI>(
  const float *, const InputPointTypeXYZI *, const uint32_t, const float *, const uint32_t,
  const utils::ActiveComm &, uint32_t *, OutputSegmentationPointType *,
  OutputVisualizationPointType *, InputPointTypeXYZI *);
template cudaError_t PostprocessCuda::fillCloud_launch_impl<InputPointTypeXYZIRC>(
  const float *, const InputPointTypeXYZIRC *, const uint32_t, const float *, const uint32_t,
  const utils::ActiveComm &, uint32_t *, OutputSegmentationPointType *,
  OutputVisualizationPointType *, InputPointTypeXYZIRC *);
template cudaError_t PostprocessCuda::fillCloud_launch_impl<InputPointTypeXYZIRADRT>(
  const float *, const InputPointTypeXYZIRADRT *, const uint32_t, const float *, const uint32_t,
  const utils::ActiveComm &, uint32_t *, OutputSegmentationPointType *,
  OutputVisualizationPointType *, InputPointTypeXYZIRADRT *);
template cudaError_t PostprocessCuda::fillCloud_launch_impl<InputPointTypeXYZIRCAEDT>(
  const float *, const InputPointTypeXYZIRCAEDT *, const uint32_t, const float *, const uint32_t,
  const utils::ActiveComm &, uint32_t *, OutputSegmentationPointType *,
  OutputVisualizationPointType *, InputPointTypeXYZIRCAEDT *);

cudaError_t PostprocessCuda::fillCloud_launch(
  const float * points_xyzi, const void * cloud_compact, const uint32_t num_points_after_projection,
  const float * seg_logit, const uint32_t num_points, InputFormat format,
  const utils::ActiveComm & active_comm, uint32_t * output_num_points_filtered,
  OutputSegmentationPointType * output_cloud_seg, OutputVisualizationPointType * output_cloud_viz,
  void * output_cloud_filtered)
{
  switch (format) {
    case InputFormat::XYZIRCAEDT:
      return fillCloud_launch_impl(
        points_xyzi, static_cast<const InputPointTypeXYZIRCAEDT *>(cloud_compact),
        num_points_after_projection, seg_logit, num_points, active_comm, output_num_points_filtered,
        output_cloud_seg, output_cloud_viz,
        static_cast<InputPointTypeXYZIRCAEDT *>(output_cloud_filtered));
    case InputFormat::XYZIRADRT:
      return fillCloud_launch_impl(
        points_xyzi, static_cast<const InputPointTypeXYZIRADRT *>(cloud_compact),
        num_points_after_projection, seg_logit, num_points, active_comm, output_num_points_filtered,
        output_cloud_seg, output_cloud_viz,
        static_cast<InputPointTypeXYZIRADRT *>(output_cloud_filtered));
    case InputFormat::XYZIRC:
      return fillCloud_launch_impl(
        points_xyzi, static_cast<const InputPointTypeXYZIRC *>(cloud_compact),
        num_points_after_projection, seg_logit, num_points, active_comm, output_num_points_filtered,
        output_cloud_seg, output_cloud_viz,
        static_cast<InputPointTypeXYZIRC *>(output_cloud_filtered));
    case InputFormat::XYZI:
      return fillCloud_launch_impl(
        points_xyzi, static_cast<const InputPointTypeXYZI *>(cloud_compact),
        num_points_after_projection, seg_logit, num_points, active_comm, output_num_points_filtered,
        output_cloud_seg, output_cloud_viz,
        static_cast<InputPointTypeXYZI *>(output_cloud_filtered));
    default:
      return cudaErrorInvalidValue;
  }
}

}  // namespace autoware::lidar_frnet
