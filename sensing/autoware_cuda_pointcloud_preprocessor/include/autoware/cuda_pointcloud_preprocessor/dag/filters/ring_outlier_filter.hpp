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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__RING_OUTLIER_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__RING_OUTLIER_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Filters ring-based outliers
 *
 * Inputs:
 *   - "pointcloud" (cuda_blackboard::CudaPointCloud2): Input pointcloud
 *
 * Outputs:
 *   - "filtered" (cuda_blackboard::CudaPointCloud2): Pass-through (mask updated internally)
 *
 * Parameters:
 *   - "distance_ratio" (double): Distance ratio threshold (required)
 *   - "object_length_threshold" (double): Object length threshold in meters (required)
 *   - "enabled" (bool): Whether filter is enabled (optional, default: true)
 *
 * Note: This filter updates internal mask state in the shared preprocessor.
 * The actual filtering is applied in the finalize step.
 */
class RingOutlierFilter : public IFilter
{
public:
  void initialize(const std::map<std::string, std::any> & params) override;
  void process(
    const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context, const std::vector<std::string> & output_names) override;
  FilterMetadata getMetadata() const override;
  bool validateInputs(const TypedInputs & inputs) const override;

private:
  RingOutlierFilterParameters params_;
  bool enabled_{true};
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__RING_OUTLIER_FILTER_HPP_
