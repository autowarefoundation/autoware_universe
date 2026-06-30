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

  std::size_t generateFeatures(
    const void * input_data, CloudFormat input_format, unsigned int num_points,
    float * voxel_features, std::int32_t * voxel_coords, std::int64_t * voxel_hashes,
    void * compact_points, float * reconstruction_features, void * cropped_source_points,
    std::int64_t * inverse_map, std::size_t * num_cropped_points);

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

  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> hashes64_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> sorted_hashes64_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> hash_indexes64_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> sorted_hash_indexes64_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> unique_mask64_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint64_t[]> unique_indices64_d_{nullptr};

  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> hashes32_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> sorted_hashes32_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> hash_indexes32_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> sorted_hash_indexes32_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> unique_mask32_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint32_t[]> unique_indices32_d_{nullptr};

  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> generate_feature_workspace_d_{nullptr};
  std::size_t generate_feature_workspace_size_{0};
  autoware::cuda_utils::CudaUniquePtrHost<std::uint32_t> num_cropped_points_;
  autoware::cuda_utils::CudaUniquePtrHost<std::uint32_t> num_unique_points32_;
  autoware::cuda_utils::CudaUniquePtrHost<std::uint64_t> num_unique_points64_;
  cudaEvent_t num_cropped_points_copy_event_;
  cudaEvent_t num_unique_points32_copy_event_;
  cudaEvent_t num_unique_points64_copy_event_;

  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> pooling_keys_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> pooling_sorted_keys_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> pooling_indices_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> pooling_sorted_indices_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> pooling_run_flags_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> pooling_run_ids_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> pooling_workspace_d_{nullptr};
  std::size_t pooling_workspace_size_{0};
};
}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__PREPROCESS_KERNEL_HPP_
