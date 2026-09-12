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

#ifndef AUTOWARE__PTV3__PREPROCESS__PREPROCESS_KERNEL_HPP_
#define AUTOWARE__PTV3__PREPROCESS__PREPROCESS_KERNEL_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>

#include <cstdint>
#include <vector>

namespace autoware::ptv3
{

struct SerializedPoolingDeviceStageView
{
  std::int64_t * indices{};
  std::int64_t * indptr{};
  std::int64_t * head_indices{};
  std::int64_t * cluster{};
  std::int32_t * grid_coord{};
  std::int64_t * serialized_code{};
  std::int64_t * serialized_order{};
  std::int64_t * serialized_inverse{};
};

class PreprocessCuda
{
public:
  PreprocessCuda(const PTv3Config & config, cudaStream_t stream);
  ~PreprocessCuda();

  /**
   * @brief Crops, voxelizes and deduplicates the input cloud.
   *
   * The emitted voxels are sorted by their order-0 serialized code, which
   * generateSerializedPoolingMetadata requires.
   *
   * @param input_data Input cloud in `input_format` layout.
   * @param input_format Point layout of `input_data`.
   * @param num_points Number of points in `input_data`.
   * @param voxel_features Output feature vector (x, y, z, intensity) per voxel.
   * @param voxel_coords Output grid coordinates, laid out [num_voxels, 3].
   * @param serialized_code Output serialized codes, laid out [num_orders, num_voxels].
   * @param compact_points Output representative input point per voxel, in `input_format` layout.
   * @param reconstruction_features Optional output feature vectors of every input point (FULL
   * reconstruction) or of the in-range ones (PARTIAL). Skipped if nullptr.
   * @param cropped_source_points Optional output of the in-range input points, in `input_format`
   * layout. Skipped if nullptr.
   * @param inverse_map Optional output mapping each in-range point to its voxel index. Skipped if
   * nullptr.
   * @param num_cropped_points Output number of in-range points.
   * @return Number of unique voxels. May exceed max_num_voxels, in which case only the first
   * max_num_voxels voxels were written to the outputs.
   */
  std::size_t generateFeatures(
    const void * input_data, CloudFormat input_format, unsigned int num_points,
    float * voxel_features, std::int32_t * voxel_coords, std::int64_t * serialized_code,
    void * compact_points, float * reconstruction_features, void * cropped_source_points,
    std::int64_t * inverse_map, std::size_t * num_cropped_points);

  /**
   * @brief Builds the per-stage pooling metadata the encoder graph consumes.
   *
   * @param grid_coord Grid coordinates of the input voxels, laid out [num_voxels, 3].
   * @param serialized_code Codes of the input voxels, laid out [num_orders, num_voxels].
   * @param num_voxels Number of input voxels; clamped to max_num_voxels internally.
   * @param stages Output device buffers to fill, one per pooling stage.
   * @param stage_counts Output voxel count per level, laid out [num_stages + 1]; entry 0 is the
   * (clamped) input count.
   * @pre The input voxels are sorted by their order-0 serialized code (`serialized_code` row 0),
   * as generateFeatures emits them. The coarser levels are derived with prefix scans that rely on
   * this ordering; an unsorted input silently produces wrong metadata. Asserted on device in
   * debug builds.
   */
  void generateSerializedPoolingMetadata(
    const std::int32_t * grid_coord, const std::int64_t * serialized_code, std::int64_t num_voxels,
    const std::vector<SerializedPoolingDeviceStageView> & stages, std::int64_t * stage_counts);

  [[nodiscard]] const std::uint32_t * cropMask() const { return crop_mask_d_.get(); }
  [[nodiscard]] const std::uint32_t * cropIndices() const { return crop_indices_d_.get(); }

private:
  PTv3Config config_;
  cudaStream_t stream_;

  autoware::cuda_utils::CudaUniquePtr<float[]> points_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> cropped_points_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> cropped_input_points_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> crop_mask_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> crop_indices_d_{nullptr};

  // Voxelization keys: order-0 serialized (Morton) codes, unique per grid cell, so sorting by them
  // both deduplicates voxels and puts them in order-0 serialization order.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> codes_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> sorted_codes_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> code_indices_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> sorted_code_indices_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> unique_mask_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> unique_indices_d_{nullptr};

  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> generate_feature_workspace_d_{nullptr};
  std::size_t generate_feature_workspace_size_{0};
  autoware::cuda_utils::CudaUniquePtrHost<std::uint32_t> num_cropped_points_;
  autoware::cuda_utils::CudaUniquePtrHost<std::uint32_t> num_unique_points_;
  cudaEvent_t num_cropped_points_copy_event_;
  cudaEvent_t num_unique_points_copy_event_;

  /// Serialization order of the input level (the deduplicated voxels generateFeatures emits),
  /// laid out [num_orders, num_voxels]. Row 0 is the identity; the remaining rows are the only
  /// sorts left in the pooling-metadata path.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> input_level_order_d_{nullptr};
  /// Keys for one of those sorts: each input voxel's code under the serialization order being
  /// sorted.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> order_sort_keys_d_{nullptr};
  /// Sorted-keys output; CUB requires the buffer, nothing reads it afterwards.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> order_sort_sorted_keys_d_{nullptr};
  /// Filled with 0..n-1 and sorted alongside the keys, which leaves it listing the voxel indices
  /// in ascending code order; written directly into the input_level_order_d_ row.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> order_sort_indices_d_{nullptr};
  /// Run-start flags: 1 where the parent voxel changes while walking a level, either in storage
  /// order (pooling) or as listed by one of its serialization orders; 0 elsewhere.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> run_flags_d_{nullptr};
  /// Inclusive scan of run_flags_d_, numbering each element's run; run id - 1 is the pooled-level
  /// slot the element scatters to.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> run_ids_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> pooling_workspace_d_{nullptr};
  std::size_t pooling_workspace_size_{0};
  int code_sort_end_bit_{64};
};
}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__PREPROCESS_KERNEL_HPP_
