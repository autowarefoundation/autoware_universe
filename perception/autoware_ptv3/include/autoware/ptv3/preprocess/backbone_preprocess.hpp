// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__PTV3__PREPROCESS__BACKBONE_PREPROCESS_HPP_
#define AUTOWARE__PTV3__PREPROCESS__BACKBONE_PREPROCESS_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>

#include <cstdint>

namespace autoware::ptv3
{

/**
 * @brief Build sparse backbone inputs from a GPU point cloud.
 */
class BackbonePreprocess
{
public:
  /**
   * @brief Allocate preprocessing scratch buffers.
   *
   * @param config Runtime configuration for crop range, voxel grid, and buffer capacity.
   * @param stream CUDA stream used for all preprocessing work.
   */
  BackbonePreprocess(const PTv3Config & config, cudaStream_t stream);

  /**
   * @brief Crop, voxelize, and serialize the cloud for the backbone.
   *
   * @param input_data Device pointer to source points in the given input_format.
   * @param input_format Layout of points in input_data.
   * @param num_points Number of source points.
   * @param voxel_features Output voxel feature array.
   * @param voxel_coords Output voxel grid coordinates.
   * @param voxel_hashes Output serialized voxel hashes.
   * @param compact_points Output source points that passed the crop.
   * @param reconstruction_features Output per-point features for partial or full reconstruction.
   * @param cropped_source_points Output cropped source points for partial reconstruction.
   * @param inverse_map Output cropped-point to voxel index map.
   * @param num_cropped_points Output number of points that passed the crop.
   * @param unclipped_num_voxels Output unique voxel count before max voxel clipping.
   * @return Number of valid voxels after max voxel clipping.
   */
  std::size_t generate_features(
    const void * input_data, CloudFormat input_format, unsigned int num_points,
    float * voxel_features, std::int32_t * voxel_coords, std::int64_t * voxel_hashes,
    void * compact_points, float * reconstruction_features, void * cropped_source_points,
    std::int64_t * inverse_map, std::size_t & num_cropped_points,
    std::size_t & unclipped_num_voxels);

  /**
   * @brief Return the crop mask from the last preprocessing run.
   *
   * @return Device pointer to the crop mask.
   */
  [[nodiscard]] const std::uint32_t * crop_mask() const { return crop_mask_d_.get(); }

  /**
   * @brief Return the prefix sum over the last crop mask.
   *
   * @return Device pointer to the crop index array.
   */
  [[nodiscard]] const std::uint32_t * crop_indices() const { return crop_indices_d_.get(); }

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

  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> sort_workspace_d_{nullptr};
  std::size_t sort_workspace_size_{0};
};
}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__BACKBONE_PREPROCESS_HPP_
