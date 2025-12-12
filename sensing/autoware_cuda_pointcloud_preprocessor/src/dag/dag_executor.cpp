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

#include "autoware/cuda_pointcloud_preprocessor/dag/dag_executor.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/processing_state.hpp"

#include <algorithm>
#include <queue>
#include <stdexcept>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

void DagExecutor::buildDag(
  const std::vector<DagNodeConfig> & node_configs, const FilterContext & context)
{
  nodes_.clear();
  node_outputs_.clear();
  context_ = context;

  auto & registry = getFilterRegistry();
  for (const auto & config : node_configs) {
    DagNode node;
    node.config = config;

    auto filter = registry.createFilter(config.type);
    if (!filter) {
      throw std::runtime_error("Unknown filter type: " + config.type);
    }

    filter->initialize(config.parameters);
    node.filter = std::move(filter);

    nodes_.push_back(std::move(node));
  }

  if (!validateDag()) {
    throw std::runtime_error("Invalid DAG configuration");
  }
}

std::map<std::string, std::shared_ptr<void>> DagExecutor::execute(
  const std::map<std::string, std::shared_ptr<void>> & inputs,
  const std::map<
    std::string,
    std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>> &
    publishers)
{
  node_outputs_.clear();
  analyzeConsumers();
  const auto execution_order = topologicalSort();

  for (const auto node_index : execution_order) {
    auto & node = nodes_[node_index];
    auto raw_inputs = resolveInputs(node_index, inputs);
    auto typed_inputs = prepareTypedInputs(raw_inputs);

    if (!node.filter->validateInputs(typed_inputs)) {
      throw std::runtime_error(
        "Invalid inputs for node '" + node.config.id + "' of type '" + node.config.type + "'");
    }

    std::map<std::string, std::shared_ptr<void>> node_outputs;
    node.filter->process(typed_inputs, node_outputs, context_, node.config.outputs);

    for (const auto & output_name : node.config.outputs) {
      auto it = node_outputs.find(output_name);
      if (it != node_outputs.end()) {
        std::string output_key = node.config.id + "." + output_name;

        // Check if this output needs to be published
        auto publisher_it = publishers.find(output_key);
        bool needs_publish = (publisher_it != publishers.end());

        auto consumer_it = output_consumers_.find(output_key);
        bool has_consumers =
          (consumer_it != output_consumers_.end() &&
           !consumer_it->second.consumer_node_indices.empty());

        if (needs_publish) {
          // CRITICAL: This output will be published, so finalize and publish it NOW
          // before any subsequent filters can modify the data
          try {
            auto output_state = std::static_pointer_cast<PointcloudProcessingState>(it->second);
            if (output_state && output_state->width > 0) {
              // Finalize: apply all masks and compact the pointcloud
              auto finalized_unique =
                context_.shared_preprocessor->finalizeOutputPublic(*output_state);

              // Publish immediately to prevent modification by subsequent filters
              publisher_it->second->publish(std::move(finalized_unique));

              // If there are also downstream consumers, store the processing state
              if (has_consumers) {
                consumer_it->second.remaining_consumers =
                  static_cast<int>(consumer_it->second.consumer_node_indices.size());
                node_outputs_[output_key] = it->second;
              }
            }
          } catch (const std::exception & e) {
            if (context_.logger) {
              RCLCPP_ERROR(
                *context_.logger, "Failed to finalize and publish output '%s': %s",
                output_key.c_str(), e.what());
            }
          }
        } else {
          // No publish needed, just store for downstream consumers
          if (has_consumers) {
            consumer_it->second.remaining_consumers =
              static_cast<int>(consumer_it->second.consumer_node_indices.size());
          }
          node_outputs_[output_key] = it->second;
        }
      }
    }
  }

  return node_outputs_;
}

bool DagExecutor::validateDag() const
{
  if (hasCycle()) {
    return false;
  }

  auto & registry = getFilterRegistry();
  for (const auto & node : nodes_) {
    if (!registry.isRegistered(node.config.type)) {
      return false;
    }
  }

  for (const auto & node : nodes_) {
    for (const auto & input : node.config.inputs) {
      if (!input.from_node.empty()) {
        bool found = false;
        for (const auto & source_node : nodes_) {
          if (source_node.config.id == input.from_node) {
            if (
              std::find(
                source_node.config.outputs.begin(), source_node.config.outputs.end(),
                input.source) != source_node.config.outputs.end()) {
              found = true;
              break;
            }
          }
        }
        if (!found) {
          return false;
        }
      }
    }
  }

  return true;
}

std::vector<std::size_t> DagExecutor::getExecutionOrder() const
{
  return topologicalSort();
}

std::vector<std::size_t> DagExecutor::topologicalSort() const
{
  std::vector<std::vector<std::size_t>> adj_list(nodes_.size());
  std::vector<int> in_degree(nodes_.size(), 0);

  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    for (const auto & input : nodes_[i].config.inputs) {
      if (!input.from_node.empty()) {
        for (std::size_t j = 0; j < nodes_.size(); ++j) {
          if (nodes_[j].config.id == input.from_node) {
            adj_list[j].push_back(i);
            in_degree[i]++;
            break;
          }
        }
      }
    }
  }

  std::queue<std::size_t> queue;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (in_degree[i] == 0) {
      queue.push(i);
    }
  }

  std::vector<std::size_t> result;
  while (!queue.empty()) {
    auto u = queue.front();
    queue.pop();
    result.push_back(u);

    for (auto v : adj_list[u]) {
      in_degree[v]--;
      if (in_degree[v] == 0) {
        queue.push(v);
      }
    }
  }

  if (result.size() != nodes_.size()) {
    throw std::runtime_error("DAG has cycles or disconnected components");
  }

  return result;
}

std::map<std::string, std::shared_ptr<void>> DagExecutor::resolveInputs(
  std::size_t node_index, const std::map<std::string, std::shared_ptr<void>> & external_inputs)
{
  const auto & node = nodes_[node_index];
  std::map<std::string, std::shared_ptr<void>> resolved_inputs;

  for (const auto & input : node.config.inputs) {
    std::string input_name = input.name.empty() ? input.source : input.name;

    if (input.from_node.empty()) {
      auto it = external_inputs.find(input.source);
      if (it != external_inputs.end()) {
        resolved_inputs[input_name] = it->second;
      } else if (!input.optional) {
        throw std::runtime_error(
          "Missing required external input: " + input.source + " for node " + node.config.id);
      }
    } else {
      std::string key = input.from_node + "." + input.source;
      auto it = node_outputs_.find(key);
      if (it != node_outputs_.end()) {
        auto consumer_it = output_consumers_.find(key);
        if (consumer_it != output_consumers_.end() && consumer_it->second.remaining_consumers > 0) {
          consumer_it->second.remaining_consumers--;

          if (consumer_it->second.remaining_consumers == 0) {
            resolved_inputs[input_name] = it->second;
          } else {
            auto state = std::static_pointer_cast<PointcloudProcessingState>(it->second);
            auto copied = makeProcessingStateCopy(state, context_.stream);
            resolved_inputs[input_name] = copied;
          }
        } else {
          resolved_inputs[input_name] = it->second;
        }
      } else if (!input.optional) {
        throw std::runtime_error(
          "Missing required node output: " + input.source + " from node " + input.from_node +
          " for node " + node.config.id);
      }
    }
  }

  return resolved_inputs;
}

bool DagExecutor::hasCycle() const
{
  std::vector<bool> visited(nodes_.size(), false);
  std::vector<bool> rec_stack(nodes_.size(), false);

  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (!visited[i]) {
      if (hasCycleDFS(i, visited, rec_stack)) {
        return true;
      }
    }
  }
  return false;
}

bool DagExecutor::hasCycleDFS(
  std::size_t node_index, std::vector<bool> & visited, std::vector<bool> & rec_stack) const
{
  visited[node_index] = true;
  rec_stack[node_index] = true;

  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (i == node_index) continue;

    bool depends = false;
    for (const auto & input : nodes_[i].config.inputs) {
      if (input.from_node == nodes_[node_index].config.id) {
        depends = true;
        break;
      }
    }

    if (depends) {
      if (!visited[i]) {
        if (hasCycleDFS(i, visited, rec_stack)) {
          return true;
        }
      } else if (rec_stack[i]) {
        return true;
      }
    }
  }

  rec_stack[node_index] = false;
  return false;
}

TypedInputs DagExecutor::prepareTypedInputs(
  const std::map<std::string, std::shared_ptr<void>> & raw_inputs) const
{
  TypedInputs typed_inputs;

  for (const auto & [name, data] : raw_inputs) {
    if (name == "twist" || name == "imu") {
      typed_inputs.special_inputs.push_back(name);
    } else {
      auto state = std::static_pointer_cast<PointcloudProcessingState>(data);
      typed_inputs.processing_states[name] = state;
    }
  }

  return typed_inputs;
}

void DagExecutor::analyzeConsumers()
{
  output_consumers_.clear();

  for (std::size_t consumer_idx = 0; consumer_idx < nodes_.size(); ++consumer_idx) {
    const auto & consumer_node = nodes_[consumer_idx];

    for (const auto & input : consumer_node.config.inputs) {
      if (!input.from_node.empty()) {
        std::string output_key = input.from_node + "." + input.source;
        output_consumers_[output_key].consumer_node_indices.push_back(consumer_idx);
      }
    }
  }

  for (auto & [key, info] : output_consumers_) {
    info.remaining_consumers = static_cast<int>(info.consumer_node_indices.size());
  }
}

std::shared_ptr<autoware::cuda_pointcloud_preprocessor::dag::PointcloudProcessingState>
DagExecutor::makeProcessingStateCopy(
  const std::shared_ptr<autoware::cuda_pointcloud_preprocessor::dag::PointcloudProcessingState> &
    source,
  cudaStream_t stream) const
{
  auto copy =
    std::make_shared<autoware::cuda_pointcloud_preprocessor::dag::PointcloudProcessingState>();

  copy->header = source->header;
  copy->height = source->height;
  copy->width = source->width;
  copy->fields = source->fields;
  copy->is_bigendian = source->is_bigendian;
  copy->point_step = source->point_step;
  copy->row_step = source->row_step;
  copy->is_dense = source->is_dense;

  size_t data_size = source->row_step * source->height;
  std::uint8_t * device_copy_data;
  cudaError_t malloc_err = cudaMalloc(&device_copy_data, data_size);
  if (malloc_err != cudaSuccess) {
    throw std::runtime_error(
      "Failed to allocate GPU memory for state copy: " +
      std::string(cudaGetErrorString(malloc_err)));
  }

  copy->device_data = device_copy_data;
  copy->owns_memory = true;

  cudaError_t copy_err = cudaMemcpyAsync(
    copy->device_data, source->device_data, data_size, cudaMemcpyDeviceToDevice, stream);

  if (copy_err != cudaSuccess) {
    cudaFree(device_copy_data);
    throw std::runtime_error(
      "Failed to copy processing state GPU data: " + std::string(cudaGetErrorString(copy_err)));
  }

  return copy;
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
