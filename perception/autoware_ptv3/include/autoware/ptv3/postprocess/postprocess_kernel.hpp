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

#ifndef AUTOWARE__PTV3__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__POSTPROCESS_KERNEL_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/point_types/types.hpp>

#include <cuda_runtime_api.h>

namespace autoware::ptv3
{

using autoware::cuda_utils::CudaUniquePtr;

class PostprocessCuda
{
public:
  explicit PostprocessCuda(const PTv3Config & config, cudaStream_t stream);

  /// Optional voxel mapping excludes past-only voxels. Published rows keep their input order.
  /// `input_features` holds one row of `feature_stride` floats per point, xyz first.
  std::size_t createVisualizationPointcloud(
    const float * input_features, std::int64_t feature_stride, const std::int64_t * pred_labels,
    float * output_points, std::size_t num_classes, std::size_t num_points,
    VoxelPointMapping voxel_mapping = {});

  /// Optional voxel mapping excludes past-only voxels. Published rows keep their input order.
  /// `input_features` holds one row of `feature_stride` floats per point, xyz first.
  std::size_t createSegmentationPointcloud(
    const float * input_features, std::int64_t feature_stride, const std::int64_t * pred_labels,
    const float * pred_probs, point_types::PointXYZCPE * output_points, std::size_t num_classes,
    std::size_t num_points, VoxelPointMapping voxel_mapping = {});

  void reconstructPartial(
    const std::int64_t * inverse_map, const std::int64_t * voxel_labels, const float * voxel_probs,
    std::int64_t * output_labels, float * output_probs, std::size_t num_classes,
    std::size_t num_cropped_points, std::size_t num_voxels);

  void reconstructFull(
    const std::uint32_t * crop_mask, const std::uint32_t * crop_indices,
    const std::int64_t * inverse_map, const std::int64_t * voxel_labels, const float * voxel_probs,
    std::int64_t * output_labels, float * output_probs, std::size_t num_classes,
    std::size_t num_points, std::size_t num_voxels);

  /// Optional voxel mapping selects current-frame representatives from the original input.
  /// Published rows keep their input order.
  std::size_t createFilteredPointcloud(
    const void * compact_input_points, CloudFormat input_format, CloudFormat output_format,
    const float * pred_probs, void * output_points, std::size_t num_classes, std::size_t num_points,
    VoxelPointMapping voxel_mapping = {});

private:
  // Stable compaction: mark the rows to publish, prefix-sum the mask, then write each row at its
  // scanned offset, so every output cloud keeps the input order.
  void checkOutputCapacity(std::size_t num_points) const;
  void scanKeepMask(std::size_t num_points);
  std::size_t readOutputCount(std::size_t num_points);

  PTv3Config config_;

  CudaUniquePtr<std::uint32_t[]> keep_mask_d_{nullptr};
  CudaUniquePtr<std::uint32_t[]> output_offsets_d_{nullptr};
  CudaUniquePtr<std::uint8_t[]> scan_workspace_d_{nullptr};
  std::size_t scan_workspace_size_{0};
  std::size_t output_capacity_{0};
  CudaUniquePtr<float[]> color_map_d_{nullptr};
  CudaUniquePtr<std::uint8_t[]> class_id_to_classification_d_{nullptr};
  CudaUniquePtr<std::uint32_t[]> filter_class_indices_d_{nullptr};
  cudaStream_t stream_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
