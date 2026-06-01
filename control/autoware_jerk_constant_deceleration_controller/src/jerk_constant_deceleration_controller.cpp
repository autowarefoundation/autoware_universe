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

#include "jerk_constant_deceleration_controller.hpp"

#include <rclcpp/qos.hpp>

namespace autoware::jerk_constant_deceleration_controller
{

JerkConstantDecelerationController::JerkConstantDecelerationController(
  const rclcpp::NodeOptions & node_options)
: Node("jerk_constant_deceleration_controller", node_options)
{
  using std::placeholders::_1;
  // Parameter

  // Subscriber
  sub_control_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    "~/input/control", rclcpp::QoS{1},
    std::bind(&JerkConstantDecelerationController::onControl, this, _1));
  sub_trigger_ = this->create_subscription<
    tier4_control_msgs::msg::JerkConstantDecelerationTrigger>(
    "~/input/jerk_constant_deceleration_trigger", rclcpp::QoS{1},
    std::bind(&JerkConstantDecelerationController::onTrigger, this, _1));

  // Publisher
  pub_control_ =
    this->create_publisher<autoware_control_msgs::msg::Control>("~/output/control", rclcpp::QoS{1});
  pub_gear_command_ = this->create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
    "~/output/gear_command", rclcpp::QoS{1});
  pub_hazard_lights_command_ =
    this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      "~/output/hazard_lights_command", rclcpp::QoS{1});
  pub_turn_indicators_command_ =
    this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "~/output/turn_indicators_command", rclcpp::QoS{1});

  // Service

  // Client

  // Timer

  // State
  trigger_.trigger = false;
  trigger_.target_acceleration = 0.0;
  trigger_.target_jerk = 0.0;
  prev_control_.longitudinal.velocity = 0.0;
  prev_control_.longitudinal.acceleration = 0.0;
  prev_control_.longitudinal.jerk = 0.0;

  // Diagnostics
}

void JerkConstantDecelerationController::onControl(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  if (!trigger_.trigger) {
    prev_control_ = *msg;
    pub_control_->publish(*msg);
    publishGearCommand();
    publishHazardLightsCommand();
    publishTurnIndicatorsCommand();
    return;
  }

  const auto dt = (rclcpp::Time(msg->stamp) - rclcpp::Time(prev_control_.stamp)).seconds();

  auto control_msg = *msg;
  control_msg.longitudinal.velocity = static_cast<float>(std::max(
    prev_control_.longitudinal.velocity + prev_control_.longitudinal.acceleration * dt, 0.0));
  control_msg.longitudinal.acceleration = std::max(
    static_cast<float>(
      prev_control_.longitudinal.acceleration + prev_control_.longitudinal.jerk * dt),
    trigger_.target_acceleration);
  if (control_msg.longitudinal.acceleration == trigger_.target_acceleration) {
    control_msg.longitudinal.jerk = 0.0;
  } else {
    control_msg.longitudinal.jerk = trigger_.target_jerk;
  }

  prev_control_ = control_msg;

  pub_control_->publish(control_msg);
  publishGearCommand();
  publishHazardLightsCommand();
  publishTurnIndicatorsCommand();
}

void JerkConstantDecelerationController::onTrigger(
  const tier4_control_msgs::msg::JerkConstantDecelerationTrigger::SharedPtr msg)
{
  trigger_ = *msg;
}

void JerkConstantDecelerationController::publishGearCommand()
{
  auto gear_cmd_msg = autoware_vehicle_msgs::msg::GearCommand();
  gear_cmd_msg.stamp = this->now();
  gear_cmd_msg.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
  pub_gear_command_->publish(gear_cmd_msg);
}

void JerkConstantDecelerationController::publishHazardLightsCommand()
{
  auto hazard_lights_cmd_msg = autoware_vehicle_msgs::msg::HazardLightsCommand();
  hazard_lights_cmd_msg.stamp = this->now();
  hazard_lights_cmd_msg.command = autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
  pub_hazard_lights_command_->publish(hazard_lights_cmd_msg);
}

void JerkConstantDecelerationController::publishTurnIndicatorsCommand()
{
  auto turn_indicators_cmd_msg = autoware_vehicle_msgs::msg::TurnIndicatorsCommand();
  turn_indicators_cmd_msg.stamp = this->now();
  turn_indicators_cmd_msg.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;
  pub_turn_indicators_command_->publish(turn_indicators_cmd_msg);
}

}  // namespace autoware::jerk_constant_deceleration_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::jerk_constant_deceleration_controller::JerkConstantDecelerationController)
