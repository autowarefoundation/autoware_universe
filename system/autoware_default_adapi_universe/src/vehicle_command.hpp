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

#ifndef VEHICLE_COMMAND_HPP_
#define VEHICLE_COMMAND_HPP_

#include <autoware/adapi_specs/control.hpp>
#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs_universe/control.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

class VehicleCommandNode : public rclcpp::Node
{
public:
  explicit VehicleCommandNode(const rclcpp::NodeOptions & options);

private:
  using ExternalPedals = autoware::adapi_specs::control::PedalsCommand;
  using ExternalAcceleration = autoware::adapi_specs::control::AccelerationCommand;
  using ExternalVelocity = autoware::adapi_specs::control::VelocityCommand;
  using ExternalSteering = autoware::adapi_specs::control::SteeringCommand;
  using InternalControl = autoware::component_interface_specs::control::ControlCommand;
  using InternalActuation = autoware::component_interface_specs_universe::control::ActuationCommand;

  void on_control(const InternalControl::Message & msg);
  void on_actuation(const InternalActuation::Message & msg);

  Pub<ExternalPedals> pub_pedals_;
  Pub<ExternalAcceleration> pub_acceleration_;
  Pub<ExternalVelocity> pub_velocity_;
  Pub<ExternalSteering> pub_steering_;
  Sub<InternalControl> sub_control_;
  Sub<InternalActuation> sub_actuation_;
};

}  // namespace autoware::default_adapi

#endif  // VEHICLE_COMMAND_HPP_
