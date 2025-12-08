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

#ifndef INPUT_CONVERTER__CAN_BUS_CONVERTER_HPP_
#define INPUT_CONVERTER__CAN_BUS_CONVERTER_HPP_

#include "converter.hpp"

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <vector>

namespace autoware::tensorrt_vad::vad_interface
{

using CanBusData = std::vector<float>;

/**
 * @brief InputCanBusConverter handles CAN-Bus data processing and velocity calculations
 *
 * This class converts ROS odometry and acceleration messages to VAD CAN-Bus format:
 * - Coordinate system transformation (Autoware to VAD coordinate system)
 * - Position, orientation, velocity, acceleration, and angular velocity processing
 * - Patch angle calculation from quaternion orientation
 * - Current longitudinal velocity calculation for trajectory generation
 */
class InputCanBusConverter : public Converter
{
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer for coordinate conversions
   * @param config Reference to configuration containing default values
   */
  InputCanBusConverter(
    const CoordinateTransformer & coordinate_transformer, const VadInterfaceConfig & config);

  /**
   * @brief Process odometry and acceleration data to generate CAN-Bus data
   * @param kinematic_state ROS Odometry message containing position, orientation, and velocity
   * @param acceleration ROS AccelWithCovarianceStamped message containing acceleration data
   * @param prev_can_bus Previous frame's CAN-Bus data for delta calculations
   * @return CanBusData 18-element vector containing processed CAN-Bus information
   */
  CanBusData process_can_bus(
    const nav_msgs::msg::Odometry::ConstSharedPtr & kinematic_state,
    const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & acceleration,
    const std::vector<float> & prev_can_bus) const;

private:
  // Default delta yaw value when previous CAN-Bus data is not available
  float default_delta_yaw_;
};

}  // namespace autoware::tensorrt_vad::vad_interface

#endif  // INPUT_CONVERTER__CAN_BUS_CONVERTER_HPP_
