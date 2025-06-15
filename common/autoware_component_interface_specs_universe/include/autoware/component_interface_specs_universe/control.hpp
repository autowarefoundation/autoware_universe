// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__CONTROL_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__CONTROL_HPP_

#include <rclcpp/qos.hpp>

#include <tier4_control_msgs/msg/is_paused.hpp>
#include <tier4_control_msgs/msg/is_start_requested.hpp>
#include <tier4_control_msgs/msg/is_stopped.hpp>
#include <tier4_control_msgs/srv/set_pause.hpp>
#include <tier4_control_msgs/srv/set_stop.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>

namespace autoware::component_interface_specs_universe::control
{

struct ActuationCommand
{
  using Message = tier4_vehicle_msgs::msg::ActuationCommandStamped;
  static constexpr char name[] = "/control/command/actuation_cmd";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct SetPause
{
  using Service = tier4_control_msgs::srv::SetPause;
  static constexpr char name[] = "/control/vehicle_cmd_gate/set_pause";
};

struct IsPaused
{
  using Message = tier4_control_msgs::msg::IsPaused;
  static constexpr char name[] = "/control/vehicle_cmd_gate/is_paused";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct IsStartRequested
{
  using Message = tier4_control_msgs::msg::IsStartRequested;
  static constexpr char name[] = "/control/vehicle_cmd_gate/is_start_requested";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct SetStop
{
  using Service = tier4_control_msgs::srv::SetStop;
  static constexpr char name[] = "/control/vehicle_cmd_gate/set_stop";
};

struct IsStopped
{
  using Message = tier4_control_msgs::msg::IsStopped;
  static constexpr char name[] = "/control/vehicle_cmd_gate/is_stopped";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware::component_interface_specs_universe::control

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__CONTROL_HPP_
