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

#include "autoware/cuda_pointcloud_preprocessor/dag/filters/distortion_filter.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <stdexcept>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

void DistortionFilter::initialize(const std::map<std::string, std::any> & parameters)
{
  // Parse use_3d parameter
  if (parameters.find("use_3d") != parameters.end()) {
    try {
      bool use_3d = std::any_cast<bool>(parameters.at("use_3d"));
      undistortion_type_ = use_3d ? CudaPointcloudPreprocessor::UndistortionType::Undistortion3D
                                  : CudaPointcloudPreprocessor::UndistortionType::Undistortion2D;
    } catch (const std::bad_any_cast & e) {
      throw std::runtime_error(
        "DistortionFilter: Failed to parse 'use_3d' as bool: " + std::string(e.what()));
    }
  }

  // Parse use_imu parameter
  if (parameters.find("use_imu") != parameters.end()) {
    try {
      use_imu_ = std::any_cast<bool>(parameters.at("use_imu"));
    } catch (const std::bad_any_cast & e) {
      throw std::runtime_error(
        "DistortionFilter: Failed to parse 'use_imu' as bool: " + std::string(e.what()));
    }
  }
}

void DistortionFilter::process(
  const TypedInputs & inputs, std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context, const std::vector<std::string> & output_names)
{
  // Get the processing state input (already typed by DAG executor)
  if (inputs.processing_states.empty()) {
    throw std::runtime_error("DistortionFilter: No processing state input found");
  }
  auto input_state = inputs.processing_states.begin()->second;

  // Check if twist/imu are marked as required (they're in special_inputs)
  // Actual data comes from context queues

  // Get twist and angular velocity queues from context
  if (!context.twist_queue || !context.angular_velocity_queue) {
    RCLCPP_WARN(
      *context.logger,
      "Twist or angular velocity queue not available, skipping distortion correction");
    outputs[output_names[0]] = std::static_pointer_cast<void>(input_state);
    return;
  }

  // Calculate first point timestamp
  // This is a simplified calculation - in production, should get from pointcloud metadata
  first_point_rel_stamp_nsec_ = 0;  // Relative to pointcloud timestamp

  // Delegate to shared preprocessor with filter-specific parameters
  // NOTE: correctDistortionPublic modifies state.device_data in-place (zero-copy!)
  context.shared_preprocessor->correctDistortionPublic(
    *input_state, *context.twist_queue, *context.angular_velocity_queue,
    first_point_rel_stamp_nsec_, undistortion_type_, use_imu_);

  outputs[output_names[0]] = std::static_pointer_cast<void>(input_state);
}

FilterMetadata DistortionFilter::getMetadata() const
{
  return {
    .filter_type = "DistortionFilter",
    .required_inputs = {"pointcloud"},
    .optional_inputs = {},
    .outputs = {"corrected"},
    .input_types = {{"pointcloud", "cuda_blackboard::CudaPointCloud2"}},
    .output_types = {{"corrected", "cuda_blackboard::CudaPointCloud2"}}};
}

bool DistortionFilter::validateInputs(const TypedInputs & inputs) const
{
  // Need at least one pointcloud input
  // twist and imu are optional special inputs (data comes from context)
  return !inputs.processing_states.empty();
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
