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

#ifndef AUTOWARE__PTV3__BACKBONE_PREPROCESSOR_HPP_
#define AUTOWARE__PTV3__BACKBONE_PREPROCESSOR_HPP_

#include "autoware/ptv3/execution_context.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"
#include "autoware/ptv3/utils.hpp"
#include "autoware/ptv3/visibility_control.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace autoware::ptv3
{

class PTV3_PUBLIC BackbonePreprocessor
{
public:
  explicit BackbonePreprocessor(const PTv3BackbonePreprocessConfig & config);

  [[nodiscard]] std::int32_t * gridCoord() const { return grid_coord_d_.get(); }
  [[nodiscard]] float * features() const { return feat_d_.get(); }
  [[nodiscard]] std::int64_t * serializedCode() const { return serialized_code_d_.get(); }
  [[nodiscard]] const void * compactPoints() const { return compact_points_d_.get(); }
  [[nodiscard]] const PreprocessCuda & cudaPreprocessor() const { return *pre_ptr_; }
  [[nodiscard]] CloudFormat inputFormat() const { return input_format_; }
  [[nodiscard]] std::size_t numInputPoints() const { return num_input_points_; }
  [[nodiscard]] std::size_t numCroppedPoints() const { return num_cropped_points_; }
  [[nodiscard]] const std::vector<std::int64_t> & serializedPoolingNumVoxels() const
  {
    return serialized_pooling_num_voxels_;
  }
  [[nodiscard]] std::int64_t numVoxels() const { return num_voxels_; }

  void prepareCloudFormat(const cuda_blackboard::CudaPointCloud2 & msg);
  bool run(const cuda_blackboard::CudaPointCloud2 & msg, const PTv3ExecutionContext & context);
  [[nodiscard]] std::vector<SerializedPoolingDeviceStageView> serializedPoolingStageViews();

private:
  struct SerializedPoolingDeviceStage
  {
    autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> indices{nullptr};
    autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> indptr{nullptr};
    autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> head_indices{nullptr};
    autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> cluster{nullptr};
    autoware::cuda_utils::CudaUniquePtr<std::int32_t[]> grid_coord{nullptr};
    autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> serialized_code{nullptr};
    autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> serialized_order{nullptr};
    autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> serialized_inverse{nullptr};
  };

  void allocateSerializedPoolingBuffers();
  void precomputeSerializedPoolingMetadata(const PTv3ExecutionContext & context);
  [[nodiscard]] CloudFormat detectCloudFormat(const cuda_blackboard::CudaPointCloud2 & cloud) const;

  PTv3BackbonePreprocessConfig config_;
  std::unique_ptr<PreprocessCuda> pre_ptr_{nullptr};
  std::once_flag init_cloud_;
  CloudFormat input_format_{CloudFormat::UNKNOWN};
  std::size_t num_input_points_{0};
  std::size_t num_cropped_points_{0};
  std::int64_t num_voxels_{0};

  autoware::cuda_utils::CudaUniquePtr<std::uint8_t[]> compact_points_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int32_t[]> grid_coord_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> feat_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> serialized_code_d_{nullptr};

  std::vector<SerializedPoolingDeviceStage> serialized_pooling_stages_d_;
  autoware::cuda_utils::CudaUniquePtr<std::int64_t[]> serialized_pooling_num_voxels_d_{nullptr};
  std::vector<std::int64_t> serialized_pooling_num_voxels_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__BACKBONE_PREPROCESSOR_HPP_
