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

#ifndef AUTOWARE__TENSORRT_VAD__VAD_INTERFACE_HPP_
#define AUTOWARE__TENSORRT_VAD__VAD_INTERFACE_HPP_

#include "coordinate_transformer.hpp"
#include "data_types.hpp"
#include "input_converter/bev_shift_converter.hpp"
#include "input_converter/can_bus_converter.hpp"
#include "input_converter/image_converter.hpp"
#include "input_converter/transform_matrix_converter.hpp"
#include "output_converter/map_converter.hpp"
#include "output_converter/objects_converter.hpp"
#include "output_converter/trajectory_converter.hpp"
#include "vad_interface_config.hpp"
#include "vad_model.hpp"

#include <cmath>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_vad
{

// Forward declarations for converter classes
namespace vad_interface
{
class CoordinateTransformer;
class InputImageConverter;
class InputTransformMatrixConverter;
class InputCanBusConverter;
class InputBEVShiftConverter;
class OutputTrajectoryConverter;
class OutputMapConverter;
class OutputObjectsConverter;
}  // namespace vad_interface

// Data structures for return values of each process_* method
using CameraImagesData = std::vector<float>;
using ShiftData = std::vector<float>;
using VadBase2ImgData = std::vector<float>;
using CanBusData = std::vector<float>;

/**
 * @class VadInterface
 * @brief Interface for converting VadInputTopicData (ROS topic data) to VADInputData format, and
 * VadOutputData to VadOutputTopicData (ROS topic data).
 */

class VadInterface
{
public:
  explicit VadInterface(
    const VadInterfaceConfig & config, const std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  VadInputData convert_input(const VadInputTopicData & vad_input_topic_data);
  VadOutputTopicData convert_output(
    const VadOutputData & vad_output_data, const rclcpp::Time & stamp,
    const double trajectory_timestep, const Eigen::Matrix4d & base2map_transform) const;

private:
  // Configuration
  VadInterfaceConfig config_;

  // Coordinate transformer for all coordinate system conversions
  std::unique_ptr<vad_interface::CoordinateTransformer> coordinate_transformer_;

  // Input converters
  std::unique_ptr<vad_interface::InputImageConverter> input_image_converter_;
  std::unique_ptr<vad_interface::InputTransformMatrixConverter> input_transform_matrix_converter_;
  std::unique_ptr<vad_interface::InputCanBusConverter> input_can_bus_converter_;
  std::unique_ptr<vad_interface::InputBEVShiftConverter> input_bev_shift_converter_;

  // Output converters
  std::unique_ptr<vad_interface::OutputTrajectoryConverter> output_trajectory_converter_;
  std::unique_ptr<vad_interface::OutputMapConverter> output_map_converter_;
  std::unique_ptr<vad_interface::OutputObjectsConverter> output_objects_converter_;

  // Previous can_bus data for velocity calculation and other processes
  std::vector<float> prev_can_bus_;

  // Cached VAD base_link (coordinate used in VAD output trajectory) to camera transformation matrix
  std::optional<VadBase2ImgData> vad_base2img_transform_;
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE__TENSORRT_VAD__VAD_INTERFACE_HPP_
