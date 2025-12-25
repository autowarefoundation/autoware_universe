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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__TRANSFORM_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__TRANSFORM_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Transforms pointcloud using TF2 transforms
 *
 * Inputs:
 *   - "pointcloud" (cuda_blackboard::CudaPointCloud2): Input pointcloud
 *
 * Outputs:
 *   - "transformed" (cuda_blackboard::CudaPointCloud2): Transformed pointcloud
 *
 * Parameters:
 *   - "target_frame" (string): Target frame for transformation
 */
class TransformFilter : public IFilter
{
public:
  void initialize(const std::map<std::string, std::any> & params) override;
  void process(
    const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context, const std::vector<std::string> & output_names) override;
  FilterMetadata getMetadata() const override;
  bool validateInputs(const TypedInputs & inputs) const override;

private:
  std::string target_frame_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__FILTERS__TRANSFORM_FILTER_HPP_
