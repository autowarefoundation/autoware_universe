// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ptv3/postprocess/postprocess_kernel.hpp"
#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/utils.hpp"

#include <autoware/point_types/types.hpp>

#include <math_constants.h>

#include <stdexcept>

namespace autoware::ptv3
{
using autoware::point_types::PointCloudClassification;

__global__ void createVisualizationPointcloudKernel(
  const float4 * input_features, const float * colors, const std::int64_t * labels,
  float4 * output_points, std::size_t num_classes, std::size_t num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const auto label = labels[idx];
  const auto color =
    label >= 0 && static_cast<std::size_t>(label) < num_classes ? colors[label] : 0.0f;

  output_points[idx] =
    make_float4(input_features[idx].x, input_features[idx].y, input_features[idx].z, color);
}

__global__ void createSegmentationPointcloudKernel(
  const float4 * input_features, const std::int64_t * labels, const float * pred_probs,
  const std::uint8_t * class_id_to_classification, const std::uint32_t * filter_class_indices,
  std::size_t num_filter_classes, std::uint32_t * output_num_points,
  point_types::PointXYZCPE * output_points, std::size_t num_classes, std::size_t num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const auto input_point = input_features[idx];
  const auto label = labels[idx];
  const bool has_valid_label = label >= 0 && static_cast<std::size_t>(label) < num_classes;

  // Points without a valid label have no meaningful class distribution, so the entropy is left as
  // NaN, which is the default value of point_types::PointXYZCPE::entropy and denotes
  // "not available". Note that the default member initializer does not apply here because the
  // output buffer is raw device memory, hence the explicit assignment below.
  float entropy = CUDART_NAN_F;
  if (has_valid_label) {
    for (std::size_t i = 0; i < num_filter_classes; ++i) {
      if (filter_class_indices[i] == static_cast<std::uint32_t>(label)) {
        return;
      }
    }

    entropy = 0.0f;
    for (std::size_t class_idx = 0; class_idx < num_classes; ++class_idx) {
      const auto probability = pred_probs[idx * num_classes + class_idx];
      if (probability > 0.0f) {
        entropy -= probability * logf(probability);
      }
    }
    if (num_classes > 1) {
      entropy /= logf(static_cast<float>(num_classes));
    }
  }

  const auto output_idx = atomicAdd(output_num_points, 1U);
  output_points[output_idx].x = input_point.x;
  output_points[output_idx].y = input_point.y;
  output_points[output_idx].z = input_point.z;
  output_points[output_idx].class_id =
    has_valid_label ? class_id_to_classification[label]
                    : static_cast<std::uint8_t>(PointCloudClassification::INVALID);
  output_points[output_idx].probability =
    has_valid_label ? pred_probs[idx * num_classes + label] : 0.0f;
  output_points[output_idx].entropy = entropy;
}

__global__ void reconstructPartialKernel(
  const std::int64_t * __restrict__ inverse_map, const std::int64_t * __restrict__ voxel_labels,
  const float * __restrict__ voxel_probs, std::int64_t * __restrict__ output_labels,
  float * __restrict__ output_probs, std::size_t num_classes, std::size_t num_cropped_points,
  std::size_t num_voxels)
{
  const auto point_idx = blockIdx.x * blockDim.y + threadIdx.y;
  const auto class_idx = blockIdx.y * blockDim.x + threadIdx.x;

  if (point_idx >= num_cropped_points || class_idx >= num_classes) return;

  const auto voxel_idx = inverse_map[point_idx];
  const bool has_valid_voxel = voxel_idx >= 0 && static_cast<std::size_t>(voxel_idx) < num_voxels;
  if (class_idx == 0) {
    output_labels[point_idx] = has_valid_voxel ? voxel_labels[voxel_idx] : 255;
  }

  output_probs[point_idx * num_classes + class_idx] =
    has_valid_voxel ? voxel_probs[static_cast<std::size_t>(voxel_idx) * num_classes + class_idx]
                    : 0.0f;
}

__global__ void reconstructFullKernel(
  const std::uint32_t * __restrict__ crop_mask, const std::uint32_t * __restrict__ crop_indices,
  const std::int64_t * __restrict__ inverse_map, const std::int64_t * __restrict__ voxel_labels,
  const float * __restrict__ voxel_probs, std::int64_t * __restrict__ output_labels,
  float * __restrict__ output_probs, std::size_t num_classes, std::size_t num_points,
  std::size_t num_voxels)
{
  const auto point_idx = blockIdx.x * blockDim.y + threadIdx.y;
  const auto class_idx = blockIdx.y * blockDim.x + threadIdx.x;

  if (point_idx >= num_points || class_idx >= num_classes) return;

  const auto mask = crop_mask[point_idx];
  if (mask == 0) {
    if (class_idx == 0) output_labels[point_idx] = 255;
    return;
  }

  const auto cropped_idx = crop_indices[point_idx] - 1;
  const auto voxel_idx = inverse_map[cropped_idx];
  if (voxel_idx < 0 || static_cast<std::size_t>(voxel_idx) >= num_voxels) {
    if (class_idx == 0) output_labels[point_idx] = 255;
    return;
  }

  if (class_idx == 0) {
    output_labels[point_idx] = voxel_labels[voxel_idx];
  }

  output_probs[point_idx * num_classes + class_idx] =
    voxel_probs[static_cast<std::size_t>(voxel_idx) * num_classes + class_idx];
}

template <typename OutputPointT>
__device__ void set_point_from_input(
  OutputPointT & output_point, const CloudPointTypeXYZI & input_point);
template <typename OutputPointT>
__device__ void set_point_from_input(
  OutputPointT & output_point, const CloudPointTypeXYZIRC & input_point);
template <typename OutputPointT>
__device__ void set_point_from_input(
  OutputPointT & output_point, const CloudPointTypeXYZIRADRT & input_point);
template <typename OutputPointT>
__device__ void set_point_from_input(
  OutputPointT & output_point, const CloudPointTypeXYZIRCAEDT & input_point);

template <>
__device__ void set_point_from_input<CloudPointTypeXYZI>(
  CloudPointTypeXYZI & output_point, const CloudPointTypeXYZI & input_point)
{
  output_point = input_point;
}

template <>
__device__ void set_point_from_input<CloudPointTypeXYZI>(
  CloudPointTypeXYZI & output_point, const CloudPointTypeXYZIRC & input_point)
{
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.intensity = static_cast<float>(input_point.intensity);
}

template <>
__device__ void set_point_from_input<CloudPointTypeXYZI>(
  CloudPointTypeXYZI & output_point, const CloudPointTypeXYZIRADRT & input_point)
{
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.intensity = input_point.intensity;
}

template <>
__device__ void set_point_from_input<CloudPointTypeXYZI>(
  CloudPointTypeXYZI & output_point, const CloudPointTypeXYZIRCAEDT & input_point)
{
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.intensity = static_cast<float>(input_point.intensity);
}

template <>
__device__ void set_point_from_input<CloudPointTypeXYZIRC>(
  CloudPointTypeXYZIRC & output_point, const CloudPointTypeXYZIRC & input_point)
{
  output_point = input_point;
}

template <>
__device__ void set_point_from_input<CloudPointTypeXYZIRC>(
  CloudPointTypeXYZIRC & output_point, const CloudPointTypeXYZIRCAEDT & input_point)
{
  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.intensity = input_point.intensity;
  output_point.return_type = input_point.return_type;
  output_point.channel = input_point.channel;
}

template <>
__device__ void set_point_from_input<CloudPointTypeXYZIRADRT>(
  CloudPointTypeXYZIRADRT & output_point, const CloudPointTypeXYZIRADRT & input_point)
{
  output_point = input_point;
}

template <>
__device__ void set_point_from_input<CloudPointTypeXYZIRCAEDT>(
  CloudPointTypeXYZIRCAEDT & output_point, const CloudPointTypeXYZIRCAEDT & input_point)
{
  output_point = input_point;
}

template <typename InputPointT, typename OutputPointT>
__global__ void createFilteredPointcloudKernel(
  const InputPointT * input_points, const float * pred_probs,
  const std::uint32_t * filter_class_indices, std::size_t num_filter_classes,
  std::size_t num_classes, std::uint32_t * output_num_points, OutputPointT * output_points,
  std::size_t num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  const float * point_probs = &pred_probs[num_classes * idx];
  std::uint32_t label = 0U;
  float max_probability = point_probs[0];
  for (std::uint32_t class_idx = 1U; class_idx < num_classes; ++class_idx) {
    if (point_probs[class_idx] > max_probability) {
      max_probability = point_probs[class_idx];
      label = class_idx;
    }
  }

  bool keep_point = true;
  for (std::size_t i = 0; i < num_filter_classes; ++i) {
    if (filter_class_indices[i] == label) {
      keep_point = false;
      break;
    }
  }

  if (!keep_point) {
    return;
  }

  const auto output_idx = atomicAdd(output_num_points, 1U);
  set_point_from_input(output_points[output_idx], input_points[idx]);
}

template <typename InputPointT, typename OutputPointT>
void createFilteredPointcloudTyped(
  cudaStream_t stream, std::uint32_t threads_per_block, const void * compact_input_points,
  const float * pred_probs, const std::uint32_t * filter_class_indices,
  std::size_t num_filter_classes, std::size_t num_classes, std::uint32_t * output_num_points,
  void * output_points, std::size_t num_points)
{
  const auto num_blocks = divup(num_points, threads_per_block);
  createFilteredPointcloudKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    static_cast<const InputPointT *>(compact_input_points), pred_probs, filter_class_indices,
    num_filter_classes, num_classes, output_num_points, static_cast<OutputPointT *>(output_points),
    num_points);
}

PostprocessCuda::PostprocessCuda(const PTv3Config & config, cudaStream_t stream)
: config_(config), stream_(stream)
{
  filtered_mask_d_ = autoware::cuda_utils::make_unique<std::uint32_t[]>(1);

  color_map_d_ = autoware::cuda_utils::make_unique<float[]>(config_.colors_rgb_.size());
  cudaMemcpyAsync(
    color_map_d_.get(), config_.colors_rgb_.data(), config_.colors_rgb_.size() * sizeof(float),
    cudaMemcpyHostToDevice, stream_);

  // The LUT is resolved from segmentation3d.class_mapping when PTv3Config is built.
  class_id_to_classification_d_ =
    autoware::cuda_utils::make_unique<std::uint8_t[]>(config_.class_id_to_classification_.size());
  cudaMemcpyAsync(
    class_id_to_classification_d_.get(), config_.class_id_to_classification_.data(),
    config_.class_id_to_classification_.size() * sizeof(std::uint8_t), cudaMemcpyHostToDevice,
    stream_);

  if (!config_.filter_class_indices_.empty()) {
    filter_class_indices_d_ =
      autoware::cuda_utils::make_unique<std::uint32_t[]>(config_.filter_class_indices_.size());
    cudaMemcpyAsync(
      filter_class_indices_d_.get(), config_.filter_class_indices_.data(),
      config_.filter_class_indices_.size() * sizeof(std::uint32_t), cudaMemcpyHostToDevice,
      stream_);
  }

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

void PostprocessCuda::createVisualizationPointcloud(
  const float * input_features, const std::int64_t * labels, float * output_points,
  std::size_t num_classes, std::size_t num_points)
{
  auto num_blocks = divup(num_points, config_.threads_per_block_);

  createVisualizationPointcloudKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<const float4 *>(input_features), color_map_d_.get(), labels,
    reinterpret_cast<float4 *>(output_points), num_classes, num_points);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

std::size_t PostprocessCuda::createSegmentationPointcloud(
  const float * input_features, const std::int64_t * pred_labels, const float * pred_probs,
  point_types::PointXYZCPE * output_points, std::size_t num_classes, std::size_t num_points)
{
  cudaMemsetAsync(filtered_mask_d_.get(), 0, sizeof(std::uint32_t), stream_);

  auto num_blocks = divup(num_points, config_.threads_per_block_);
  const auto num_filter_classes =
    config_.filter_apply_to_segmentation_ ? config_.filter_class_indices_.size() : std::size_t{0};

  createSegmentationPointcloudKernel<<<num_blocks, config_.threads_per_block_, 0, stream_>>>(
    reinterpret_cast<const float4 *>(input_features), pred_labels, pred_probs,
    class_id_to_classification_d_.get(), filter_class_indices_d_.get(), num_filter_classes,
    filtered_mask_d_.get(), output_points, num_classes, num_points);

  std::uint32_t num_segmented_points = 0;
  cudaMemcpyAsync(
    &num_segmented_points, filtered_mask_d_.get(), sizeof(std::uint32_t), cudaMemcpyDeviceToHost,
    stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return num_segmented_points;
}

void PostprocessCuda::reconstructPartial(
  const std::int64_t * inverse_map, const std::int64_t * voxel_labels, const float * voxel_probs,
  std::int64_t * output_labels, float * output_probs, std::size_t num_classes,
  std::size_t num_cropped_points, std::size_t num_voxels)
{
  auto block = dim3(32, 8);
  auto grid = dim3(divup(num_cropped_points, block.y), divup(num_classes, block.x));

  reconstructPartialKernel<<<grid, block, 0, stream_>>>(
    inverse_map, voxel_labels, voxel_probs, output_labels, output_probs, num_classes,
    num_cropped_points, num_voxels);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

void PostprocessCuda::reconstructFull(
  const std::uint32_t * crop_mask, const std::uint32_t * crop_indices,
  const std::int64_t * inverse_map, const std::int64_t * voxel_labels, const float * voxel_probs,
  std::int64_t * output_labels, float * output_probs, std::size_t num_classes,
  std::size_t num_points, std::size_t num_voxels)
{
  auto block = dim3(32, 8);
  auto grid = dim3(divup(num_points, block.y), divup(num_classes, block.x));

  reconstructFullKernel<<<grid, block, 0, stream_>>>(
    crop_mask, crop_indices, inverse_map, voxel_labels, voxel_probs, output_labels, output_probs,
    num_classes, num_points, num_voxels);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

std::size_t PostprocessCuda::createFilteredPointcloud(
  const void * compact_input_points, CloudFormat input_format, CloudFormat output_format,
  const float * pred_probs, void * output_points, std::size_t num_classes, std::size_t num_points)
{
  cudaMemsetAsync(filtered_mask_d_.get(), 0, sizeof(std::uint32_t), stream_);

  switch (input_format) {
    case CloudFormat::XYZIRCAEDT:
      switch (output_format) {
        case CloudFormat::XYZIRCAEDT:
          createFilteredPointcloudTyped<CloudPointTypeXYZIRCAEDT, CloudPointTypeXYZIRCAEDT>(
            stream_, config_.threads_per_block_, compact_input_points, pred_probs,
            filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
            filtered_mask_d_.get(), output_points, num_points);
          break;
        case CloudFormat::XYZIRC:
          createFilteredPointcloudTyped<CloudPointTypeXYZIRCAEDT, CloudPointTypeXYZIRC>(
            stream_, config_.threads_per_block_, compact_input_points, pred_probs,
            filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
            filtered_mask_d_.get(), output_points, num_points);
          break;
        case CloudFormat::XYZI:
          createFilteredPointcloudTyped<CloudPointTypeXYZIRCAEDT, CloudPointTypeXYZI>(
            stream_, config_.threads_per_block_, compact_input_points, pred_probs,
            filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
            filtered_mask_d_.get(), output_points, num_points);
          break;
        default:
          throw std::runtime_error("Unsupported filtered output format.");
      }
      break;
    case CloudFormat::XYZIRADRT:
      switch (output_format) {
        case CloudFormat::XYZIRADRT:
          createFilteredPointcloudTyped<CloudPointTypeXYZIRADRT, CloudPointTypeXYZIRADRT>(
            stream_, config_.threads_per_block_, compact_input_points, pred_probs,
            filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
            filtered_mask_d_.get(), output_points, num_points);
          break;
        case CloudFormat::XYZI:
          createFilteredPointcloudTyped<CloudPointTypeXYZIRADRT, CloudPointTypeXYZI>(
            stream_, config_.threads_per_block_, compact_input_points, pred_probs,
            filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
            filtered_mask_d_.get(), output_points, num_points);
          break;
        default:
          throw std::runtime_error("Unsupported filtered output format.");
      }
      break;
    case CloudFormat::XYZIRC:
      switch (output_format) {
        case CloudFormat::XYZIRC:
          createFilteredPointcloudTyped<CloudPointTypeXYZIRC, CloudPointTypeXYZIRC>(
            stream_, config_.threads_per_block_, compact_input_points, pred_probs,
            filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
            filtered_mask_d_.get(), output_points, num_points);
          break;
        case CloudFormat::XYZI:
          createFilteredPointcloudTyped<CloudPointTypeXYZIRC, CloudPointTypeXYZI>(
            stream_, config_.threads_per_block_, compact_input_points, pred_probs,
            filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
            filtered_mask_d_.get(), output_points, num_points);
          break;
        default:
          throw std::runtime_error("Unsupported filtered output format.");
      }
      break;
    case CloudFormat::XYZI:
      if (output_format != CloudFormat::XYZI) {
        throw std::runtime_error("Unsupported filtered output format.");
      }
      createFilteredPointcloudTyped<CloudPointTypeXYZI, CloudPointTypeXYZI>(
        stream_, config_.threads_per_block_, compact_input_points, pred_probs,
        filter_class_indices_d_.get(), config_.filter_class_indices_.size(), num_classes,
        filtered_mask_d_.get(), output_points, num_points);
      break;
    default:
      throw std::runtime_error("Unsupported input point cloud format.");
  }

  std::uint32_t num_filtered_points = 0;
  cudaMemcpyAsync(
    &num_filtered_points, filtered_mask_d_.get(), sizeof(std::uint32_t), cudaMemcpyDeviceToHost,
    stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  return num_filtered_points;
}

}  // namespace autoware::ptv3
