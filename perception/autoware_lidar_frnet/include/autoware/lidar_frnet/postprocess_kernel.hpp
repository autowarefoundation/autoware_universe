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

  cudaError_t fillCloud_launch(
    const InputPointType * cloud, const float * seg_logit, const int32_t num_points,
    const utils::ActiveComm & active_comm, uint32_t * output_num_points_filtered,
    OutputSegmentationPointType * output_cloud_seg, OutputVisualizationPointType * output_cloud_viz,
    InputPointType * output_cloud_filtered);

private:
  cudaStream_t stream_;
};

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__POSTPROCESS_KERNEL_HPP_
