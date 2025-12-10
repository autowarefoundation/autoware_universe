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

#include "autoware/cuda_pointcloud_preprocessor/dag/filters/downsample_filter.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

void DownsampleFilter::initialize(const std::map<std::string, std::any> & params)
{
  if (params.find("voxel_size_x") != params.end()) {
    try {
      voxel_size_x_ = static_cast<float>(std::any_cast<double>(params.at("voxel_size_x")));
    } catch (const std::bad_any_cast &) {
      throw std::runtime_error("DownsampleFilter: 'voxel_size_x' must be a double");
    }
  } else {
    throw std::runtime_error("DownsampleFilter: Missing required parameter 'voxel_size_x'");
  }

  if (params.find("voxel_size_y") != params.end()) {
    try {
      voxel_size_y_ = static_cast<float>(std::any_cast<double>(params.at("voxel_size_y")));
    } catch (const std::bad_any_cast &) {
      throw std::runtime_error("DownsampleFilter: 'voxel_size_y' must be a double");
    }
  } else {
    throw std::runtime_error("DownsampleFilter: Missing required parameter 'voxel_size_y'");
  }

  if (params.find("voxel_size_z") != params.end()) {
    try {
      voxel_size_z_ = static_cast<float>(std::any_cast<double>(params.at("voxel_size_z")));
    } catch (const std::bad_any_cast &) {
      throw std::runtime_error("DownsampleFilter: 'voxel_size_z' must be a double");
    }
  } else {
    throw std::runtime_error("DownsampleFilter: Missing required parameter 'voxel_size_z'");
  }

  if (voxel_size_x_ <= 0.0f || voxel_size_y_ <= 0.0f || voxel_size_z_ <= 0.0f) {
    throw std::runtime_error("DownsampleFilter: Voxel sizes must be positive");
  }

  constexpr int64_t DEFAULT_MAX_MEM_POOL_SIZE = 1LL << 30;
  downsampler_ = std::make_unique<CudaVoxelGridDownsampleFilter>(
    voxel_size_x_, voxel_size_y_, voxel_size_z_, DEFAULT_MAX_MEM_POOL_SIZE);
}

void DownsampleFilter::process(
  const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context, const std::vector<std::string> & output_names)
{
  (void)context;  // Context not used directly, downsampler manages its own resources

  if (inputs.processing_states.empty()) {
    throw std::runtime_error("DownsampleFilter: No input processing state provided");
  }

  // Get the first (and only) input processing state
  auto input_state = inputs.processing_states.begin()->second;

  // Use the DAG-optimized interface that works directly with processing states
  // This still involves one copy internally (input state -> temp CudaPointCloud2)
  // but avoids the unsafe .release() and properly manages memory
  auto output_state = downsampler_->filterProcessingState(input_state);

  // Output with dynamic name
  outputs[output_names[0]] = output_state;
}

FilterMetadata DownsampleFilter::getMetadata() const
{
  return {
    .filter_type = "DownsampleFilter",
    .required_inputs = {"*"},  // Accept any pointcloud name
    .optional_inputs = {},
    .outputs = {"downsampled"},
    .input_types = {{"*", "cuda_blackboard::CudaPointCloud2"}},
    .output_types = {{"downsampled", "cuda_blackboard::CudaPointCloud2"}}};
}

bool DownsampleFilter::validateInputs(const TypedInputs & inputs) const
{
  // Need exactly one processing state input
  return inputs.processing_states.size() == 1;
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
