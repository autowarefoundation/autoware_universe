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

  // Crops, voxelizes and deduplicates the input cloud. The emitted voxels are ordered by their
  // order-0 serialized code, which generateSerializedPoolingMetadata requires (see below).
  std::size_t generateFeatures(
    const void * input_data, CloudFormat input_format, unsigned int num_points,
    float * voxel_features, std::int32_t * voxel_coords, std::int64_t * voxel_hashes,
    void * compact_points, float * reconstruction_features, void * cropped_source_points,
    std::int64_t * inverse_map, std::size_t * num_cropped_points);

  // Builds the per-stage pooling metadata the encoder graph consumes.
  //
  // PRECONDITION: `serialized_code` row 0 must be ascending, i.e. the voxels must be stored in
  // order-0 serialization order. generateFeatures guarantees this. The whole hierarchy is then
  // derived with prefix scans instead of sorts: right-shifting a serialized code yields its
  // parent's code, which is monotone, so pooling preserves the ordering and every coarser level
  // stays sorted by construction. Feeding an arbitrarily ordered level here silently produces wrong
  // metadata.
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

  // Voxelization keys are order-0 serialized (Morton) codes, not hashes: they are an injective
  // function of the grid coordinate, so sorting by them both deduplicates voxels and leaves the
  // voxel array in order-0 serialization order (see generateFeatures).
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

  // Level-0 serialization order, laid out [num_orders, num_voxels]. Row 0 is the identity because
  // generateFeatures already sorted the voxels by their order-0 code; the remaining rows are the
  // only genuine sorts left in the pooling-metadata path.
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> level0_order_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> order_sort_keys_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> order_sort_sorted_keys_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> order_sort_indices_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> run_flags_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> run_ids_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> pooling_workspace_d_{nullptr};
  std::size_t pooling_workspace_size_{0};
  int code_sort_end_bit_{64};
};
}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__PREPROCESS_KERNEL_HPP_
