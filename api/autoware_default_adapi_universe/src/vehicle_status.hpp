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

#ifndef VEHICLE_STATUS_HPP_
#define VEHICLE_STATUS_HPP_

#include <autoware/adapi_specs/vehicle.hpp>
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/component_interface_specs_universe/localization.hpp>
#include <autoware/component_interface_specs_universe/map.hpp>
#include <autoware/component_interface_specs_universe/vehicle.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/gear.hpp>
#include <autoware_adapi_v1_msgs/msg/hazard_lights.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators.hpp>

#include <unordered_map>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

class VehicleStatusNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit VehicleStatusNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Pub<autoware::adapi_specs::vehicle::VehicleKinematics, NodeT> pub_kinematics_;
  Pub<autoware::adapi_specs::vehicle::VehicleStatus, NodeT> pub_status_;
  Sub<autoware::component_interface_specs_universe::localization::KinematicState, NodeT>
    sub_kinematic_state_;
  Sub<autoware::component_interface_specs_universe::localization::Acceleration, NodeT>
    sub_acceleration_;
  Sub<autoware::component_interface_specs_universe::vehicle::SteeringStatus, NodeT> sub_steering_;
  Sub<autoware::component_interface_specs_universe::vehicle::GearStatus, NodeT> sub_gear_state_;
  Sub<autoware::component_interface_specs_universe::vehicle::TurnIndicatorStatus, NodeT>
    sub_turn_indicator_;
  Sub<autoware::component_interface_specs_universe::vehicle::HazardLightStatus, NodeT>
    sub_hazard_light_;
  Sub<autoware::component_interface_specs_universe::vehicle::EnergyStatus, NodeT> sub_energy_level_;
  Sub<autoware::component_interface_specs_universe::map::MapProjectorInfo, NodeT>
    sub_map_projector_info_;
  AUTOWARE_TIMER_PTR timer_;

  autoware::component_interface_specs_universe::localization::KinematicState::Message::
    ConstSharedPtr kinematic_state_msgs_;
  autoware::component_interface_specs_universe::localization::Acceleration::Message::ConstSharedPtr
    acceleration_msgs_;
  autoware::component_interface_specs_universe::vehicle::SteeringStatus::Message::ConstSharedPtr
    steering_status_msgs_;
  autoware::component_interface_specs_universe::vehicle::GearStatus::Message::ConstSharedPtr
    gear_status_msgs_;
  autoware::component_interface_specs_universe::vehicle::TurnIndicatorStatus::Message::
    ConstSharedPtr turn_indicator_status_msgs_;
  autoware::component_interface_specs_universe::vehicle::HazardLightStatus::Message::ConstSharedPtr
    hazard_light_status_msgs_;
  autoware::component_interface_specs_universe::vehicle::EnergyStatus::Message::ConstSharedPtr
    energy_status_msgs_;
  autoware::component_interface_specs_universe::map::MapProjectorInfo::Message::ConstSharedPtr
    map_projector_info_;

  uint8_t mapping(
    std::unordered_map<uint8_t, uint8_t> hash_map, uint8_t input, uint8_t default_value);
  void publish_kinematics();
  void publish_status();
  void on_timer();
};

}  // namespace autoware::default_adapi

#endif  // VEHICLE_STATUS_HPP_
