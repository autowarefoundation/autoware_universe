// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__VEHICLE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__VEHICLE_HPP_

#include <autoware/component_interface_specs/vehicle.hpp>
#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/door_status_array.hpp>
#include <autoware_adapi_v1_msgs/srv/get_door_layout.hpp>
#include <autoware_adapi_v1_msgs/srv/set_door_command.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>

namespace autoware::component_interface_specs_universe::vehicle
{

// Re-export the core specs so universe consumers keep resolving the canonical
// (single-version-authority) type; core is the sole definition and version authority.
using autoware::component_interface_specs::vehicle::ControlModeStatus;
using autoware::component_interface_specs::vehicle::GearStatus;
using autoware::component_interface_specs::vehicle::HazardLightStatus;
using autoware::component_interface_specs::vehicle::Specs;
using autoware::component_interface_specs::vehicle::SteeringStatus;
using autoware::component_interface_specs::vehicle::TurnIndicatorStatus;
using autoware::component_interface_specs::vehicle::VelocityStatus;
using autoware::component_interface_specs::vehicle::version;

// tier4/adapi-only specs kept here: they are typed on tier4_* / door messages
// and stay unversioned until vendor-specific specs get their own versioned
// registry separate from core's OSS-facing Specs tuple; they are not part of it yet.
struct EnergyStatus
{
  using Message = tier4_vehicle_msgs::msg::BatteryStatus;
  static constexpr char name[] = "/vehicle/status/battery_charge";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct DoorCommand
{
  using Service = autoware_adapi_v1_msgs::srv::SetDoorCommand;
  static constexpr char name[] = "/vehicle/doors/command";
};

struct DoorLayout
{
  using Service = autoware_adapi_v1_msgs::srv::GetDoorLayout;
  static constexpr char name[] = "/vehicle/doors/layout";
};

struct DoorStatus
{
  using Message = autoware_adapi_v1_msgs::msg::DoorStatusArray;
  static constexpr char name[] = "/vehicle/doors/status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware::component_interface_specs_universe::vehicle

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__VEHICLE_HPP_
