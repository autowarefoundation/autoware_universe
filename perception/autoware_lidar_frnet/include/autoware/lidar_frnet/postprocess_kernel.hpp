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

#ifndef AUTOWARE__LIDAR_FRNET__POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__LIDAR_FRNET__POSTPROCESS_KERNEL_HPP_

#include "autoware/lidar_frnet/point_type.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <cuda_runtime_api.h>

#include <cstdint>

namespace autoware::lidar_frnet
{

/**
 * @brief CUDA postprocess: fill segmentation, visualization, and filtered clouds from network
 *        output and optional compact input copy.
 */
class PostprocessCuda
{
public:
  PostprocessCuda(const utils::PostprocessingParams & params, cudaStream_t stream);

  /**
   * @brief Fill output clouds from predictions and optional compact point copy.
   *
   * Dispatches to templated implementation by input format. Writes seg/viz per point; filtered
   * cloud contains points not in filter classes (from cloud_compact or reconstructed from xyzi).
   *
   * @param points_xyzi Compact point buffer from preprocess (num_points * 4: x, y, z, intensity)
   * @param cloud_compact Compact copy of input points for indices [0, num_points_raw); may be
   *        nullptr (then filtered points are built from points_xyzi)
   * @param num_points_raw Number of points that have valid cloud_compact entries
   * @param pred_probs Network predictions (num_points * num_classes)
   * @param num_points Total points (after interpolation)
   * @param input_format Input cloud format
   * @param output_format Filtered output cloud format
   * @param active_comm Which outputs to fill
   * @param output_num_points_filtered Output: number of points written to output_cloud_filtered
   * @param output_cloud_seg Segmentation cloud (x, y, z, class_id, probability)
   * @param output_cloud_viz Visualization cloud (x, y, z, rgb)
   * @param output_cloud_filtered Filtered point cloud (output_format)
   * @return cudaError_t
   */
  cudaError_t fillCloud_launch(
    const float * points_xyzi, const void * cloud_compact, const uint32_t num_points_raw,
    const float * pred_probs, const uint32_t num_points, CloudFormat input_format,
    CloudFormat output_format, const utils::ActiveComm & active_comm,
    uint32_t * output_num_points_filtered, OutputSegmentationPointType * output_cloud_seg,
    OutputVisualizationPointType * output_cloud_viz, void * output_cloud_filtered);

private:
  template <typename InputPointT, typename OutputPointT>
  cudaError_t fillCloud_launch_impl(
    const float * points_xyzi, const InputPointT * cloud_compact, const uint32_t num_points_raw,
    const float * pred_probs, const uint32_t num_points, const utils::ActiveComm & active_comm,
    uint32_t * output_num_points_filtered, OutputSegmentationPointType * output_cloud_seg,
    OutputVisualizationPointType * output_cloud_viz, OutputPointT * output_cloud_filtered);

  cudaStream_t stream_;
};

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__POSTPROCESS_KERNEL_HPP_
