//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "stop_mode_operator.hpp"

namespace autoware::stop_mode_operator
{

StopModeOperator::StopModeOperator(const rclcpp::NodeOptions & options)
: Node("stop_mode_operator", options)
{
  stop_hold_acceleration_ = declare_parameter<double>("stop_hold_acceleration");

  const auto control_qos = rclcpp::QoS(5);
  const auto durable_qos = rclcpp::QoS(1).transient_local();

  pub_control_ = create_publisher<Control>("~/control", control_qos);
  pub_gear_ = create_publisher<GearCommand>("~/gear", durable_qos);
  pub_turn_indicators_ = create_publisher<TurnIndicatorsCommand>("~/turn_indicators", durable_qos);
  pub_hazard_lights_ = create_publisher<HazardLightsCommand>("~/hazard_lights", durable_qos);

  sub_steering_ = create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", 1,
    [this](SteeringReport::SharedPtr msg) { current_steering_ = *msg; });

  const auto period = rclcpp::Rate(declare_parameter<double>("rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { publish_control_command(); });

  publish_trigger_command();
}

void StopModeOperator::publish_control_command()
{
  // TODO(Takagi, Isamu): stationary steering
  Control control;
  control.stamp = control.longitudinal.stamp = control.lateral.stamp = now();
  control.lateral.steering_tire_angle = current_steering_.steering_tire_angle;
  control.lateral.steering_tire_rotation_rate = 0.0;
  control.longitudinal.velocity = 0.0;
  control.longitudinal.acceleration = stop_hold_acceleration_;
  pub_control_->publish(control);
}

void StopModeOperator::publish_trigger_command()
{
  const auto stamp = now();

  GearCommand gear;
  gear.stamp = stamp;
  gear.command = GearCommand::NONE;
  pub_gear_->publish(gear);

  TurnIndicatorsCommand turn_indicators;
  turn_indicators.stamp = stamp;
  turn_indicators.command = TurnIndicatorsCommand::DISABLE;
  pub_turn_indicators_->publish(turn_indicators);

  HazardLightsCommand hazard_lights;
  hazard_lights.stamp = stamp;
  hazard_lights.command = HazardLightsCommand::DISABLE;
  pub_hazard_lights_->publish(hazard_lights);
}

}  // namespace autoware::stop_mode_operator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::stop_mode_operator::StopModeOperator)
