// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__PREPROCESS_KERNEL_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__PREPROCESS_KERNEL_HPP_

#include "autoware/lidar_centerpoint/centerpoint_config.hpp"
#include "autoware/lidar_centerpoint/preprocess/point_type.hpp"
#include "cuda.h"
#include "cuda_runtime_api.h"

namespace autoware::lidar_centerpoint
{
class PreprocessCuda
{
public:
  PreprocessCuda(const CenterPointConfig & config, cudaStream_t & stream);

  cudaError_t generateSweepPoints_launch(
    const InputPointType * input_points, std::size_t points_size, float time_lag,
    const float * transform_array, float * output_points);

  cudaError_t shufflePoints_launch(
    const float * points, const unsigned int * indices, float * shuffled_points,
    const std::size_t points_size, const std::size_t max_size, const std::size_t offset);

  cudaError_t generateVoxels_random_launch(
    const float * points, std::size_t points_size, unsigned int * mask, float * voxels);

  cudaError_t generateBaseFeatures_launch(
    unsigned int * mask, float * voxels, unsigned int * pillar_num, float * voxel_features,
    float * voxel_num, int * voxel_idxs);

  cudaError_t generateFeatures_launch(
    const float * voxel_features, const float * voxel_num_points, const int * coords,
    const unsigned int * num_voxels, float * features);

private:
  CenterPointConfig config_;
  cudaStream_t stream_;
};
}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__PREPROCESS_KERNEL_HPP_
