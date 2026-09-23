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

#include "redundancy_command_selector.hpp"

#include <algorithm>

namespace autoware::simulator::redundancy_command_selector
{

RedundancyCommandSelector::RedundancyCommandSelector(const rclcpp::NodeOptions & options)
: Node("redundancy_command_selector", options)
{
  main_ecu_id_ = declare_parameter<uint8_t>("main_ecu_id");
  sub_ecu_id_ = declare_parameter<uint8_t>("sub_ecu_id");

  const auto qos = rclcpp::QoS{10};

  pub_control_ = create_publisher<Control>("~/output/control_command", qos);
  pub_gear_ = create_publisher<GearCommand>("~/output/gear_command", qos);
  pub_hazard_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_command", qos);
  pub_turn_ = create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_command", qos);

  sub_main_control_ =
    create_relay<Control>("~/input/main/control_command", qos, pub_control_, true);
  sub_main_gear_ = create_relay<GearCommand>("~/input/main/gear_command", qos, pub_gear_, true);
  sub_main_hazard_ =
    create_relay<HazardLightsCommand>("~/input/main/hazard_lights_command", qos, pub_hazard_, true);
  sub_main_turn_ = create_relay<TurnIndicatorsCommand>(
    "~/input/main/turn_indicators_command", qos, pub_turn_, true);

  sub_sub_control_ = create_relay<Control>("~/input/sub/control_command", qos, pub_control_, false);
  sub_sub_gear_ = create_relay<GearCommand>("~/input/sub/gear_command", qos, pub_gear_, false);
  sub_sub_hazard_ =
    create_relay<HazardLightsCommand>("~/input/sub/hazard_lights_command", qos, pub_hazard_, false);
  sub_sub_turn_ = create_relay<TurnIndicatorsCommand>(
    "~/input/sub/turn_indicators_command", qos, pub_turn_, false);

  sub_active_control_unit_ = create_subscription<ActiveControlUnit>(
    "~/input/active_control_unit", rclcpp::QoS(1).transient_local(),
    std::bind(&RedundancyCommandSelector::on_active_control_unit, this, std::placeholders::_1));
}

void RedundancyCommandSelector::on_active_control_unit(
  const ActiveControlUnit::ConstSharedPtr & msg)
{
  const auto contains = [&](uint8_t id) {
    return std::find(msg->ids.begin(), msg->ids.end(), id) != msg->ids.end();
  };
  const bool main_active = contains(main_ecu_id_);
  const bool sub_active = contains(sub_ecu_id_);

  if (main_active == sub_active) {
    return;
  }

  const bool next_use_main = main_active;
  if (use_main_.exchange(next_use_main) != next_use_main) {
    RCLCPP_WARN(get_logger(), "Switch to %s ECU control commands", next_use_main ? "Main" : "Sub");
  }
}

}  // namespace autoware::simulator::redundancy_command_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::simulator::redundancy_command_selector::RedundancyCommandSelector)
