// Copyright 2025 TIER IV, Inc.
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

#include "node.hpp"

namespace autoware::hazard_lights_selector
{

HazardLightsSelector::HazardLightsSelector(const rclcpp::NodeOptions & node_options)
: Node("autoware_hazard_lights_selector", node_options)
{
  using std::placeholders::_1;

  // Parameter
  params_.update_rate = static_cast<int>(declare_parameter("update_rate", 10));

  // Subscriber
  sub_hazard_lights_command_from_planning_ =
    this->create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "input/planning/hazard_lights_command", 1,
      std::bind(&HazardLightsSelector::on_hazard_lights_command_from_planning, this, _1));
  sub_hazard_lights_command_from_system_ =
    this->create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "input/system/hazard_lights_command", 1,
      std::bind(&HazardLightsSelector::on_hazard_lights_command_from_system, this, _1));

  // Publisher
  pub_hazard_lights_command_ =
    this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "output/hazard_lights_command", 1);

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / params_.update_rate),
    std::bind(&HazardLightsSelector::on_timer, this));
}

void HazardLightsSelector::on_hazard_lights_command_from_planning(
  const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
{
  hazard_lights_command_from_planning_ = msg;
}

void HazardLightsSelector::on_hazard_lights_command_from_system(
  const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
{
  hazard_lights_command_from_system_ = msg;
}

void HazardLightsSelector::on_timer()
{
  using autoware_vehicle_msgs::msg::HazardLightsCommand;
  auto hazard_lights_command = HazardLightsCommand();
  hazard_lights_command.stamp = this->now();
  hazard_lights_command.command = HazardLightsCommand::DISABLE;

  if (hazard_lights_command_from_planning_) {
    if (hazard_lights_command_from_planning_->command == HazardLightsCommand::ENABLE) {
      hazard_lights_command.command = HazardLightsCommand::ENABLE;
    }
  }

  if (hazard_lights_command_from_system_) {
    if (hazard_lights_command_from_system_->command == HazardLightsCommand::ENABLE) {
      hazard_lights_command.command = HazardLightsCommand::ENABLE;
    }
  }

  pub_hazard_lights_command_->publish(hazard_lights_command);
}

}  // namespace autoware::hazard_lights_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::hazard_lights_selector::HazardLightsSelector)
