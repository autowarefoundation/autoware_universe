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

#include "autoware/cuda_pointcloud_preprocessor/dag/filters/transform_filter.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <tf2_ros/buffer.h>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

void TransformFilter::initialize(const std::map<std::string, std::any> & params)
{
  if (params.find("target_frame") != params.end()) {
    target_frame_ = std::any_cast<std::string>(params.at("target_frame"));
  }
}

void TransformFilter::process(
  const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context, const std::vector<std::string> & output_names)
{
  // Get the first (and only) input - name can be anything
  if (inputs.processing_states.empty()) {
    throw std::runtime_error("TransformFilter: No input provided");
  }
  auto input_state = inputs.processing_states.begin()->second;

  // Lookup transform from TF buffer
  geometry_msgs::msg::TransformStamped transform_msg;
  try {
    transform_msg = context.tf_buffer->lookupTransform(
      target_frame_, input_state->header.frame_id, input_state->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(*context.logger, "Transform lookup failed: %s", ex.what());
    // Pass through without transformation (zero-copy)
    outputs[output_names[0]] = std::static_pointer_cast<void>(input_state);
    return;
  }

  // Delegate to shared preprocessor (modifies state in-place)
  // NOTE: transformPointcloudPublic updates state.device_data pointer
  context.shared_preprocessor->transformPointcloudPublic(*input_state, transform_msg);

  // Update frame_id in metadata
  input_state->header.frame_id = target_frame_;

  // Output the modified state (zero-copy!)
  outputs[output_names[0]] = std::static_pointer_cast<void>(input_state);
}

FilterMetadata TransformFilter::getMetadata() const
{
  return {
    .filter_type = "TransformFilter",
    .required_inputs = {"*"},  // Accepts any single input
    .optional_inputs = {},
    .outputs = {"transformed"},
    .input_types = {{"*", "cuda_blackboard::CudaPointCloud2"}},
    .output_types = {{"transformed", "cuda_blackboard::CudaPointCloud2"}}};
}

bool TransformFilter::validateInputs(const TypedInputs & inputs) const
{
  // Just check that we have at least one input
  return !inputs.processing_states.empty();
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
