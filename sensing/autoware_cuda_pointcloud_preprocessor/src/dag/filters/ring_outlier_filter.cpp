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

#include "autoware/cuda_pointcloud_preprocessor/dag/filters/ring_outlier_filter.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <stdexcept>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

void RingOutlierFilter::initialize(const std::map<std::string, std::any> & parameters)
{
  // Parse required parameters
  if (parameters.find("distance_ratio") == parameters.end()) {
    throw std::runtime_error("RingOutlierFilter: Missing required parameter 'distance_ratio'");
  }
  if (parameters.find("object_length_threshold") == parameters.end()) {
    throw std::runtime_error(
      "RingOutlierFilter: Missing required parameter 'object_length_threshold'");
  }

  try {
    params_.distance_ratio = std::any_cast<double>(parameters.at("distance_ratio"));
  } catch (const std::bad_any_cast &) {
    // Try int
    try {
      params_.distance_ratio =
        static_cast<double>(std::any_cast<int>(parameters.at("distance_ratio")));
    } catch (const std::bad_any_cast & e) {
      throw std::runtime_error(
        "RingOutlierFilter: Failed to parse 'distance_ratio' as double: " + std::string(e.what()));
    }
  }

  try {
    params_.object_length_threshold =
      std::any_cast<double>(parameters.at("object_length_threshold"));
  } catch (const std::bad_any_cast &) {
    // Try int
    try {
      params_.object_length_threshold =
        static_cast<double>(std::any_cast<int>(parameters.at("object_length_threshold")));
    } catch (const std::bad_any_cast & e) {
      throw std::runtime_error(
        "RingOutlierFilter: Failed to parse 'object_length_threshold' as double: " +
        std::string(e.what()));
    }
  }

  // Parse optional enabled parameter
  if (parameters.find("enabled") != parameters.end()) {
    try {
      enabled_ = std::any_cast<bool>(parameters.at("enabled"));
    } catch (const std::bad_any_cast & e) {
      throw std::runtime_error(
        "RingOutlierFilter: Failed to parse 'enabled' as bool: " + std::string(e.what()));
    }
  }

  // Validate parameters
  if (params_.distance_ratio <= 0.0) {
    throw std::runtime_error(
      "RingOutlierFilter: 'distance_ratio' must be positive, got: " +
      std::to_string(params_.distance_ratio));
  }
  if (params_.object_length_threshold <= 0.0) {
    throw std::runtime_error(
      "RingOutlierFilter: 'object_length_threshold' must be positive, got: " +
      std::to_string(params_.object_length_threshold));
  }
}

void RingOutlierFilter::process(
  const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context, const std::vector<std::string> & output_names)
{
  // Get the first (and only) input - name can be anything
  if (inputs.processing_states.empty()) {
    throw std::runtime_error("RingOutlierFilter: No input provided");
  }
  auto input_state = inputs.processing_states.begin()->second;

  // Apply ring outlier filter with filter-specific parameters
  // NOTE: This only updates masks, doesn't modify device_data (zero-copy!)
  context.shared_preprocessor->applyRingOutlierFilterPublic(*input_state, params_, enabled_);

  // Pass through state (actual point filtering happens in finalize step)
  outputs[output_names[0]] = std::static_pointer_cast<void>(input_state);
}

FilterMetadata RingOutlierFilter::getMetadata() const
{
  return {
    .filter_type = "RingOutlierFilter",
    .required_inputs = {"*"},
    .optional_inputs = {},
    .outputs = {"filtered"},
    .input_types = {{"*", "cuda_blackboard::CudaPointCloud2"}},
    .output_types = {{"filtered", "cuda_blackboard::CudaPointCloud2"}}};
}

bool RingOutlierFilter::validateInputs(const TypedInputs & inputs) const
{
  return !inputs.processing_states.empty();
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
