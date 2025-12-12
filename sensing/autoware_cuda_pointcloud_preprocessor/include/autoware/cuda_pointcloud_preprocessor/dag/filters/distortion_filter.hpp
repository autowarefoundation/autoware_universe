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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__DISTORTION_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__DISTORTION_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Corrects motion distortion in pointcloud
 *
 * Inputs:
 *   - "pointcloud" (cuda_blackboard::CudaPointCloud2): Input pointcloud
 *
 * Outputs:
 *   - "corrected" (cuda_blackboard::CudaPointCloud2): Distortion-corrected pointcloud
 *
 * Parameters:
 *   - "use_3d" (bool): Use 3D undistortion (default: false)
 *   - "use_imu" (bool): Use IMU data for undistortion (default: true)
 */
class DistortionFilter : public IFilter
{
public:
  void initialize(const std::map<std::string, std::any> & params) override;
  void process(
    const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context, const std::vector<std::string> & output_names) override;
  FilterMetadata getMetadata() const override;
  bool validateInputs(const TypedInputs & inputs) const override;

private:
  CudaPointcloudPreprocessor::UndistortionType undistortion_type_{
    CudaPointcloudPreprocessor::UndistortionType::Undistortion2D};
  bool use_imu_{true};
  std::uint32_t first_point_rel_stamp_nsec_{0};
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__DISTORTION_FILTER_HPP_
