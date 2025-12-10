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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__DOWNSAMPLE_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__DOWNSAMPLE_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_voxel_grid_downsample_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Downsample Filter - Reduces pointcloud density using voxel grid downsampling
 *
 * This filter applies GPU-accelerated voxel grid downsampling to reduce the number
 * of points while preserving the overall structure of the pointcloud. Points within
 * each voxel are averaged to produce a single representative point.
 *
 * Parameters:
 *   - "voxel_size_x" (double): Voxel size in X direction (meters)
 *   - "voxel_size_y" (double): Voxel size in Y direction (meters)
 *   - "voxel_size_z" (double): Voxel size in Z direction (meters)
 *
 * Inputs:
 *   - Single pointcloud (any name)
 *
 * Outputs:
 *   - "downsampled": Downsampled pointcloud
 *
 * Note: The voxel grid downsampling algorithm groups points into 3D voxels and
 * computes the centroid of points within each voxel.
 */
class DownsampleFilter : public IFilter
{
public:
  void initialize(const std::map<std::string, std::any> & params) override;
  void process(
    const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context, const std::vector<std::string> & output_names) override;
  FilterMetadata getMetadata() const override;
  bool validateInputs(const TypedInputs & inputs) const override;

private:
  // Filter-specific parameters
  float voxel_size_x_{0.1f};
  float voxel_size_y_{0.1f};
  float voxel_size_z_{0.1f};

  // Voxel grid downsample filter instance
  std::unique_ptr<CudaVoxelGridDownsampleFilter> downsampler_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__DOWNSAMPLE_FILTER_HPP_
