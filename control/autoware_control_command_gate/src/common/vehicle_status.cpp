// Copyright 2025 The Autoware Contributors
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

#include "vehicle_status.hpp"

#include <utility>

namespace autoware::control_command_gate
{

VehicleStatus::VehicleStatus(rclcpp::Node & node) : node_(node), vehicle_stop_checker_(&node)
{
  stop_check_duration_ = node.declare_parameter<double>("stop_check_duration");

  sub_kinematics_ = node.create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    [this](Odometry::SharedPtr msg) { current_kinematics_ = *msg; });
  sub_acceleration_ = node.create_subscription<AccelWithCovarianceStamped>(
    "/localization/acceleration", 1,
    [this](AccelWithCovarianceStamped::SharedPtr msg) { current_acceleration_ = *msg; });
  sub_steering_ = node.create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", 1,
    [this](SteeringReport::SharedPtr msg) { current_steering_ = *msg; });
  sub_control_mode_ = node.create_subscription<ControlModeReport>(
    "/vehicle/status/control_mode", 1,
    [this](ControlModeReport::SharedPtr msg) { current_control_mode_ = *msg; });
}

bool VehicleStatus::is_autoware_control_enabled() const
{
  return current_control_mode_.mode == ControlModeReport::AUTONOMOUS;
}

bool VehicleStatus::is_vehicle_stopped() const
{
  return vehicle_stop_checker_.isVehicleStopped(stop_check_duration_);
}

double VehicleStatus::get_current_steering() const
{
  return current_steering_.steering_tire_angle;
}

double VehicleStatus::get_current_velocity() const
{
  return current_kinematics_.twist.twist.linear.x;
}

double VehicleStatus::get_current_acceleration() const
{
  return current_acceleration_.accel.accel.linear.x;
}

Control VehicleStatus::get_actual_status_as_command() const
{
  Control status;
  status.stamp = status.lateral.stamp = status.longitudinal.stamp = node_.now();
  status.lateral.steering_tire_angle = get_current_steering();
  status.lateral.steering_tire_rotation_rate = 0.0;
  status.longitudinal.velocity = get_current_velocity();
  status.longitudinal.acceleration = get_current_acceleration();
  return status;
}

}  // namespace autoware::control_command_gate
