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

#ifndef AUTOWARE_JERK_CONSTANT_DECELERATION_CONTROLLER_HPP_
#define AUTOWARE_JERK_CONSTANT_DECELERATION_CONTROLLER_HPP_

// include
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_control_msgs/msg/detail/control__struct.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_control_msgs/msg/jerk_constant_deceleration_trigger.hpp>

namespace autoware::jerk_constant_deceleration_controller
{

class JerkConstantDecelerationController : public rclcpp::Node
{
public:
  explicit JerkConstantDecelerationController(const rclcpp::NodeOptions & node_options);

private:
  // Parameter

  // Subscriber
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_;
  rclcpp::Subscription<
    tier4_control_msgs::msg::JerkConstantDecelerationTrigger>::SharedPtr
    sub_trigger_;

  void onControl(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void onTrigger(const tier4_control_msgs::msg::
                   JerkConstantDecelerationTrigger::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_command_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    pub_hazard_lights_command_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    pub_turn_indicators_command_;

  void publishGearCommand();
  void publishHazardLightsCommand();
  void publishTurnIndicatorsCommand();

  // Service

  // Client

  // Timer

  // State
  tier4_control_msgs::msg::JerkConstantDecelerationTrigger trigger_;
  autoware_control_msgs::msg::Control prev_control_;

  // Diagnostics
};
}  // namespace autoware::jerk_constant_deceleration_controller

#endif  // AUTOWARE_JERK_CONSTANT_DECELERATION_CONTROLLER_HPP_
