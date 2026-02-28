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

class PostprocessCuda
{
public:
  PostprocessCuda(const utils::PostprocessingParams & params, cudaStream_t stream);

  /// @brief Fill output clouds with segmentation results (dispatches to templated implementation)
  /// @param points_xyzi Compact point buffer from preprocess (num_points * 4: x, y, z, intensity)
  /// @param cloud_compact Compact copy of input points for indices [0,
  /// num_points_after_projection);
  ///        can be nullptr
  cudaError_t fillCloud_launch(
    const float * points_xyzi, const void * cloud_compact,
    const uint32_t num_points_after_projection, const float * seg_logit, const uint32_t num_points,
    InputFormat format, const utils::ActiveComm & active_comm,
    uint32_t * output_num_points_filtered, OutputSegmentationPointType * output_cloud_seg,
    OutputVisualizationPointType * output_cloud_viz, void * output_cloud_filtered);

private:
  template <typename PointT>
  cudaError_t fillCloud_launch_impl(
    const float * points_xyzi, const PointT * cloud_compact,
    const uint32_t num_points_after_projection, const float * seg_logit, const uint32_t num_points,
    const utils::ActiveComm & active_comm, uint32_t * output_num_points_filtered,
    OutputSegmentationPointType * output_cloud_seg, OutputVisualizationPointType * output_cloud_viz,
    PointT * output_cloud_filtered);

  cudaStream_t stream_;
};

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__POSTPROCESS_KERNEL_HPP_
