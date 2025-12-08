// Copyright 2025 TIER IV.
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

#include "../src/vad_interface.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <memory>

namespace autoware::tensorrt_vad
{

VadInterface::VadInterface(
  const VadInterfaceConfig & config, const std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: config_(config), prev_can_bus_(), vad_base2img_transform_(std::nullopt)
{
  // Initialize coordinate transformer
  coordinate_transformer_ = std::make_unique<vad_interface::CoordinateTransformer>(tf_buffer);

  // Initialize input converters
  input_image_converter_ =
    std::make_unique<vad_interface::InputImageConverter>(*coordinate_transformer_, config_);
  input_transform_matrix_converter_ =
    std::make_unique<vad_interface::InputTransformMatrixConverter>(
      *coordinate_transformer_, config_);
  input_can_bus_converter_ =
    std::make_unique<vad_interface::InputCanBusConverter>(*coordinate_transformer_, config_);
  input_bev_shift_converter_ =
    std::make_unique<vad_interface::InputBEVShiftConverter>(*coordinate_transformer_, config_);

  // Initialize output converters
  output_trajectory_converter_ =
    std::make_unique<vad_interface::OutputTrajectoryConverter>(*coordinate_transformer_, config_);
  output_map_converter_ =
    std::make_unique<vad_interface::OutputMapConverter>(*coordinate_transformer_, config_);
  output_objects_converter_ =
    std::make_unique<vad_interface::OutputObjectsConverter>(*coordinate_transformer_, config_);
}

VadInputData VadInterface::convert_input(const VadInputTopicData & vad_input_topic_data)
{
  VadInputData vad_input_data;

  // Process vad_base2img transformation using converter, with validation and caching
  // If not cached or if cached value is invalid (all zeros), recompute
  if (!vad_base2img_transform_.has_value()) {
    auto computed_transform =
      input_transform_matrix_converter_->process_vad_base2img(vad_input_topic_data.camera_infos);

    // Validate: check if transform contains non-zero values (valid TF lookup)
    bool is_valid = false;
    for (const auto & val : computed_transform) {
      if (std::abs(val) > 1e-6f) {
        is_valid = true;
        break;
      }
    }

    // Only cache if valid
    if (is_valid) {
      vad_base2img_transform_ = computed_transform;
    }
  }

  // Use cached value if available, otherwise use computed (potentially invalid) transform
  vad_input_data.vad_base2img =
    vad_base2img_transform_.has_value()
      ? vad_base2img_transform_.value()
      : input_transform_matrix_converter_->process_vad_base2img(vad_input_topic_data.camera_infos);

  // Process can_bus using converter
  vad_input_data.can_bus = input_can_bus_converter_->process_can_bus(
    vad_input_topic_data.kinematic_state, vad_input_topic_data.acceleration, prev_can_bus_);

  // Process shift using converter
  vad_input_data.shift =
    input_bev_shift_converter_->process_shift(vad_input_data.can_bus, prev_can_bus_);

  // Process image data using converter
  vad_input_data.camera_images = input_image_converter_->process_image(vad_input_topic_data.images);

  // Set default command
  vad_input_data.command = config_.default_command;

  // Update prev_can_bus_ for next iteration
  prev_can_bus_ = vad_input_data.can_bus;

  return vad_input_data;
}

VadOutputTopicData VadInterface::convert_output(
  const VadOutputData & vad_output_data, const rclcpp::Time & stamp,
  const double trajectory_timestep, const Eigen::Matrix4d & base2map_transform) const
{
  VadOutputTopicData vad_output_topic_data;

  // Convert candidate trajectories using converter
  vad_output_topic_data.candidate_trajectories =
    output_trajectory_converter_->process_candidate_trajectories(
      vad_output_data.predicted_trajectories, stamp, trajectory_timestep, base2map_transform);

  // Convert trajectory using converter
  vad_output_topic_data.trajectory = output_trajectory_converter_->process_trajectory(
    vad_output_data.predicted_trajectory, stamp, trajectory_timestep, base2map_transform);

  // Convert map_points using converter
  vad_output_topic_data.map_points = output_map_converter_->process_map_points(
    vad_output_data.map_polylines, stamp, base2map_transform);

  // Convert predicted objects using converter
  vad_output_topic_data.objects = output_objects_converter_->process_predicted_objects(
    vad_output_data.predicted_objects, stamp, base2map_transform);

  return vad_output_topic_data;
}

}  // namespace autoware::tensorrt_vad
