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

#include "autoware/ptv3/preprocess/semseg_preprocess_kernel.hpp"
#include "autoware/ptv3/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <stdexcept>

namespace autoware::ptv3
{

template <typename scalar_t, typename mask_t>
__global__ void extractSemsegPreprocessKernel(
  const scalar_t * __restrict__ input_data, const mask_t * __restrict__ masks,
  const mask_t * __restrict__ indices, scalar_t * __restrict__ output_data, int num_points)
{
  const auto idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    output_data[indices[idx] - 1] = input_data[idx];
  }
}

template <typename mask_t>
__global__ void scatterInverseMapKernel(
  const mask_t * __restrict__ unique_indices, const mask_t * __restrict__ sorted_hash_indexes,
  std::int64_t * __restrict__ inverse_map, int num_points)
{
  const auto idx = static_cast<std::uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  inverse_map[sorted_hash_indexes[idx]] = static_cast<std::int64_t>(unique_indices[idx] - 1);
}

template <typename PointT>
void extractCroppedSourcePoints(
  const void * input_data, const std::uint32_t * crop_mask, const std::uint32_t * crop_indices,
  void * cropped_source_points, std::size_t num_points, std::uint32_t threads_per_block,
  cudaStream_t stream)
{
  const auto num_blocks = divup(num_points, threads_per_block);
  extractSemsegPreprocessKernel<<<num_blocks, threads_per_block, 0, stream>>>(
    static_cast<const PointT *>(input_data), crop_mask, crop_indices,
    static_cast<PointT *>(cropped_source_points), num_points);
}

SemsegPreprocessCuda::SemsegPreprocessCuda(const PTv3SemsegConfig & config) : config_(config)
{
}

void SemsegPreprocessCuda::reconstructSource(
  const PreprocessCuda & backbone_preprocessor, const void * input_data,
  const CloudFormat input_format, const std::size_t num_points,
  const std::size_t num_cropped_points, const SemsegSourceReconstructionView view,
  const cudaStream_t stream) const
{
  if (view.num_cropped_points != nullptr) {
    *view.num_cropped_points = num_cropped_points;
  }
  if (config_.source_reconstruction_ == SourceReconstruction::NONE) {
    return;
  }

  const auto num_blocks = divup(num_points, config_.threads_per_block_);
  if (config_.source_reconstruction_ == SourceReconstruction::FULL) {
    if (view.reconstruction_features == nullptr) {
      throw std::runtime_error("FULL source reconstruction requires reconstruction features.");
    }
    cudaMemcpyAsync(
      view.reconstruction_features, backbone_preprocessor.pointFeatures(),
      num_points * config_.num_point_feature_size_ * sizeof(float), cudaMemcpyDeviceToDevice,
      stream);
  }

  if (config_.source_reconstruction_ == SourceReconstruction::PARTIAL) {
    if (view.reconstruction_features == nullptr || view.cropped_source_points == nullptr) {
      throw std::runtime_error(
        "PARTIAL source reconstruction requires features and cropped source points.");
    }
    extractSemsegPreprocessKernel<<<num_blocks, config_.threads_per_block_, 0, stream>>>(
      reinterpret_cast<const float4 *>(backbone_preprocessor.pointFeatures()),
      backbone_preprocessor.cropMask(), backbone_preprocessor.cropIndices(),
      reinterpret_cast<float4 *>(view.reconstruction_features), num_points);

    switch (input_format) {
      case CloudFormat::XYZIRCAEDT:
        extractCroppedSourcePoints<CloudPointTypeXYZIRCAEDT>(
          input_data, backbone_preprocessor.cropMask(), backbone_preprocessor.cropIndices(),
          view.cropped_source_points, num_points, config_.threads_per_block_, stream);
        break;
      case CloudFormat::XYZIRADRT:
        extractCroppedSourcePoints<CloudPointTypeXYZIRADRT>(
          input_data, backbone_preprocessor.cropMask(), backbone_preprocessor.cropIndices(),
          view.cropped_source_points, num_points, config_.threads_per_block_, stream);
        break;
      case CloudFormat::XYZIRC:
        extractCroppedSourcePoints<CloudPointTypeXYZIRC>(
          input_data, backbone_preprocessor.cropMask(), backbone_preprocessor.cropIndices(),
          view.cropped_source_points, num_points, config_.threads_per_block_, stream);
        break;
      case CloudFormat::XYZI:
        extractCroppedSourcePoints<CloudPointTypeXYZI>(
          input_data, backbone_preprocessor.cropMask(), backbone_preprocessor.cropIndices(),
          view.cropped_source_points, num_points, config_.threads_per_block_, stream);
        break;
      default:
        throw std::runtime_error("Unsupported input point cloud format.");
    }
  }

  if (view.inverse_map == nullptr) {
    throw std::runtime_error("Source reconstruction requires an inverse map.");
  }

  const auto cropped_blocks = divup(num_cropped_points, config_.threads_per_block_);
  if (config_.use_64bit_hash_) {
    scatterInverseMapKernel<<<cropped_blocks, config_.threads_per_block_, 0, stream>>>(
      backbone_preprocessor.uniqueIndices64(), backbone_preprocessor.sortedHashIndexes64(),
      view.inverse_map, num_cropped_points);
  } else {
    scatterInverseMapKernel<<<cropped_blocks, config_.threads_per_block_, 0, stream>>>(
      backbone_preprocessor.uniqueIndices32(), backbone_preprocessor.sortedHashIndexes32(),
      view.inverse_map, num_cropped_points);
  }

  CHECK_CUDA_ERROR(cudaPeekAtLastError());
}

}  // namespace autoware::ptv3
