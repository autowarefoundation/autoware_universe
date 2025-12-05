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

#include "../src/input_converter/can_bus_converter.hpp"

#include <Eigen/Dense>

#include <vector>

namespace autoware::tensorrt_vad::vad_interface
{

InputCanBusConverter::InputCanBusConverter(
  const CoordinateTransformer & coordinate_transformer, const VadInterfaceConfig & config)
: Converter(coordinate_transformer, config), default_delta_yaw_(0.0f)
{
}

CanBusData InputCanBusConverter::process_can_bus(
  const nav_msgs::msg::Odometry::ConstSharedPtr & kinematic_state,
  const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & acceleration,
  const std::vector<float> & prev_can_bus) const
{
  CanBusData can_bus(18, 0.0f);

  // translation (0:3)
  can_bus[0] = kinematic_state->pose.pose.position.x;
  can_bus[1] = kinematic_state->pose.pose.position.y;
  can_bus[2] = kinematic_state->pose.pose.position.z;

  // rotation (3:7)
  can_bus[3] = kinematic_state->pose.pose.orientation.x;
  can_bus[4] = kinematic_state->pose.pose.orientation.y;
  can_bus[5] = kinematic_state->pose.pose.orientation.z;
  can_bus[6] = kinematic_state->pose.pose.orientation.w;

  // acceleration (7:10)
  can_bus[7] = acceleration->accel.accel.linear.x;
  can_bus[8] = acceleration->accel.accel.linear.y;
  can_bus[9] = acceleration->accel.accel.linear.z;

  // angular velocity (10:13)
  can_bus[10] = kinematic_state->twist.twist.angular.x;
  can_bus[11] = kinematic_state->twist.twist.angular.y;
  can_bus[12] = kinematic_state->twist.twist.angular.z;

  // velocity (13:16)
  can_bus[13] = kinematic_state->twist.twist.linear.x;
  can_bus[14] = kinematic_state->twist.twist.linear.y;
  can_bus[15] = 0.0f;  // Set z-direction velocity to 0

  // Calculate patch_angle[rad] (16)
  // yaw = ArcTan(2 * (w * z + x * y) / (1 - 2 * (y ** 2 + z ** 2)))
  double yaw = std::atan2(
    2.0 * (can_bus[6] * can_bus[5] + can_bus[3] * can_bus[4]),
    1.0 - 2.0 * (can_bus[4] * can_bus[4] + can_bus[5] * can_bus[5]));
  if (yaw < 0) {
    yaw += 2 * M_PI;
  }
  can_bus[16] = static_cast<float>(yaw);

  // Calculate patch_angle[deg] (17)
  float delta_yaw = default_delta_yaw_;

  if (!prev_can_bus.empty()) {
    float prev_angle = prev_can_bus[16];
    delta_yaw = yaw - prev_angle;

    // Normalize to [-π, π] to handle wrap-around correctly
    while (delta_yaw > M_PI) {
      delta_yaw -= 2.0f * M_PI;
    }
    while (delta_yaw < -M_PI) {
      delta_yaw += 2.0f * M_PI;
    }
  }

  can_bus[17] = delta_yaw * 180.0f / M_PI;

  return can_bus;
}
}  // namespace autoware::tensorrt_vad::vad_interface
