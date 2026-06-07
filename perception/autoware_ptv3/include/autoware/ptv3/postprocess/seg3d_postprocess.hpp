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

#ifndef AUTOWARE__PTV3__POSTPROCESS__SEG3D_POSTPROCESS_HPP_
#define AUTOWARE__PTV3__POSTPROCESS__SEG3D_POSTPROCESS_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>

namespace autoware::ptv3
{

using autoware::cuda_utils::CudaUniquePtr;

/**
 * @brief CUDA postprocessing for the 3D segmentation head.
 *
 * Builds visualization clouds, segmentation clouds, point-level reconstruction, and filtered
 * output clouds from head predictions.
 */
class Seg3dPostprocess
{
public:
  /**
   * @brief Allocate lookup tables and scratch buffers.
   *
   * @param config Runtime configuration.
   * @param stream CUDA stream used for all kernel launches.
   */
  explicit Seg3dPostprocess(const PTv3Config & config, cudaStream_t stream);

  /**
   * @brief Build an XYZ plus RGB visualization cloud from features and labels.
   *
   * @param input_features  Device pointer to backbone point features [num_points x feature_dim].
   * @param pred_labels     Device pointer to per-point predicted class labels [num_points].
   * @param output_points   Device pointer to output float array [num_points x 4].
   * @param num_classes     Number of segmentation classes.
   * @param num_points      Number of points to process.
   */
  void create_visualization_pointcloud(
    const float * input_features, const std::int64_t * pred_labels, float * output_points,
    std::size_t num_classes, std::size_t num_points);

  /**
   * @brief Build a labeled segmentation cloud with class, probability, and entropy.
   *
   * @param input_features  Device pointer to backbone point features [num_points x feature_dim].
   * @param pred_labels     Device pointer to per-point predicted class labels [num_points].
   * @param pred_probs      Device pointer to per-point per-class probabilities.
   * @param output_points   Device pointer to output point buffer.
   * @param num_classes     Number of segmentation classes.
   * @param num_points      Number of points to process.
   */
  void create_segmentation_pointcloud(
    const float * input_features, const std::int64_t * pred_labels, const float * pred_probs,
    std::uint8_t * output_points, std::size_t num_classes, std::size_t num_points);

  /**
   * @brief Reconstruct labels and probabilities for cropped points.
   *
   * @param inverse_map        Device pointer mapping cropped-point index to voxel index.
   * @param voxel_labels       Device pointer to per-voxel predicted labels.
   * @param voxel_probs        Device pointer to per-voxel predicted probabilities.
   * @param output_labels      Output per-point labels [num_cropped_points].
   * @param output_probs       Output per-point probabilities [num_cropped_points x num_classes].
   * @param num_classes        Number of segmentation classes.
   * @param num_cropped_points Number of points in the cropped region.
   * @param num_voxels         Number of voxels.
   */
  void reconstruct_partial(
    const std::int64_t * inverse_map, const std::int64_t * voxel_labels, const float * voxel_probs,
    std::int64_t * output_labels, float * output_probs, std::size_t num_classes,
    std::size_t num_cropped_points, std::size_t num_voxels);

  /**
   * @brief Reconstruct labels and probabilities for the full input cloud.
   *
   * @param crop_mask     Device pointer to binary crop mask [num_points].
   * @param crop_indices  Device pointer to prefix-sum crop index [num_points].
   * @param inverse_map   Device pointer mapping cropped-point index to voxel index.
   * @param voxel_labels  Device pointer to per-voxel predicted labels.
   * @param voxel_probs   Device pointer to per-voxel predicted probabilities.
   * @param output_labels Output per-point labels [num_points].
   * @param output_probs  Output per-point probabilities [num_points x num_classes].
   * @param num_classes   Number of segmentation classes.
   * @param num_points    Total number of input points.
   * @param num_voxels    Number of voxels.
   */
  void reconstruct_full(
    const std::uint32_t * crop_mask, const std::uint32_t * crop_indices,
    const std::int64_t * inverse_map, const std::int64_t * voxel_labels, const float * voxel_probs,
    std::int64_t * output_labels, float * output_probs, std::size_t num_classes,
    std::size_t num_points, std::size_t num_voxels);

  /**
   * @brief Build a cloud with configured classes removed above the probability threshold.
   *
   * @param compact_input_points Device pointer to compacted input points.
   * @param input_format         Layout of compact_input_points.
   * @param output_format        Desired layout of output_points.
   * @param pred_probs           Device pointer to per-point predicted probabilities.
   * @param output_points        Output filtered point cloud buffer.
   * @param num_classes          Number of segmentation classes.
   * @param num_points           Number of input points.
   * @return Number of points written to output_points.
   */
  std::size_t create_filtered_pointcloud(
    const void * compact_input_points, CloudFormat input_format, CloudFormat output_format,
    const float * pred_probs, void * output_points, std::size_t num_classes,
    std::size_t num_points);

private:
  PTv3Config config_;

  CudaUniquePtr<std::uint32_t[]> filtered_mask_d_{nullptr};
  CudaUniquePtr<float[]> color_map_d_{nullptr};
  CudaUniquePtr<std::uint32_t[]> filter_class_indices_d_{nullptr};
  cudaStream_t stream_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__POSTPROCESS__SEG3D_POSTPROCESS_HPP_
