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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_EXECUTOR_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_EXECUTOR_HPP_

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filter_registry.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/processing_state.hpp"

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>

#include <any>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

struct DagNodeInput
{
  std::string source;
  std::string from_node;
  std::string name;
  bool optional{false};
};

struct DagNodeConfig
{
  std::string id;
  std::string type;
  std::map<std::string, std::any> parameters;
  std::vector<DagNodeInput> inputs;
  std::vector<std::string> outputs;
};

/**
 * @brief Executes a DAG of filters
 */
class DagExecutor
{
public:
  /**
   * @brief Build DAG from configuration
   * @param node_configs Vector of node configurations
   * @param context Filter context to use for execution
   */
  void buildDag(const std::vector<DagNodeConfig> & node_configs, const FilterContext & context);

  /**
   * @brief Execute the DAG with given inputs and publishers for immediate publishing
   * @param inputs Map of input names to data pointers
   * @param publishers Map of output keys to ROS publishers
   * @return Map of output names to data pointers (only intermediate results)
   *
   * Outputs that need to be published are finalized and published immediately after
   * each node produces them, preventing modification by subsequent filters.
   */
  std::map<std::string, std::shared_ptr<void>> execute(
    const std::map<std::string, std::shared_ptr<void>> & inputs,
    const std::map<
      std::string,
      std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>> &
      publishers);

  /**
   * @brief Validate DAG structure (check for cycles, missing dependencies, etc.)
   * @return true if valid, false otherwise
   */
  bool validateDag() const;

  /**
   * @brief Get execution order (topologically sorted node indices)
   * @return Vector of node indices in execution order
   */
  std::vector<std::size_t> getExecutionOrder() const;

private:
  struct DagNode
  {
    DagNodeConfig config;
    std::unique_ptr<IFilter> filter;
  };

  std::vector<DagNode> nodes_;
  FilterContext context_;
  std::map<std::string, std::shared_ptr<void>> node_outputs_;

  struct ConsumerInfo
  {
    std::vector<std::size_t> consumer_node_indices;
    int remaining_consumers{0};
  };
  std::map<std::string, ConsumerInfo> output_consumers_;

  void analyzeConsumers();

  std::shared_ptr<autoware::cuda_pointcloud_preprocessor::dag::PointcloudProcessingState>
  makeProcessingStateCopy(
    const std::shared_ptr<autoware::cuda_pointcloud_preprocessor::dag::PointcloudProcessingState> &
      source,
    cudaStream_t stream) const;

  std::vector<std::size_t> topologicalSort() const;

  std::map<std::string, std::shared_ptr<void>> resolveInputs(
    std::size_t node_index, const std::map<std::string, std::shared_ptr<void>> & external_inputs);

  TypedInputs prepareTypedInputs(
    const std::map<std::string, std::shared_ptr<void>> & raw_inputs) const;

  bool hasCycle() const;

  bool hasCycleDFS(
    std::size_t node_index, std::vector<bool> & visited, std::vector<bool> & rec_stack) const;
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_EXECUTOR_HPP_
