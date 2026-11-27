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

#ifndef STOP_MODE_OPERATOR_HPP_
#define STOP_MODE_OPERATOR_HPP_

#include "continuous_condition.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>

namespace autoware::stop_mode_operator
{

using autoware_control_msgs::msg::Control;
using autoware_planning_msgs::msg::RouteState;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::VelocityReport;

class StopModeOperator : public autoware::agnocast_wrapper::Node
{
public:
  explicit StopModeOperator(const rclcpp::NodeOptions & options);

private:
  void on_timer();
  void publish_control_command();
  void publish_gear_command();
  void publish_turn_indicators_command();
  void publish_hazard_lights_command();

  AUTOWARE_TIMER_PTR timer_;
  AUTOWARE_PUBLISHER_PTR(Control) pub_control_;
  AUTOWARE_PUBLISHER_PTR(GearCommand) pub_gear_;
  AUTOWARE_PUBLISHER_PTR(TurnIndicatorsCommand) pub_turn_indicators_;
  AUTOWARE_PUBLISHER_PTR(HazardLightsCommand) pub_hazard_lights_;
  AUTOWARE_SUBSCRIPTION_PTR(SteeringReport) sub_steering_;
  AUTOWARE_SUBSCRIPTION_PTR(VelocityReport) sub_velocity_;
  AUTOWARE_SUBSCRIPTION_PTR(RouteState) sub_route_state_;

  SteeringReport current_steering_;
  RouteState current_route_state_;
  ContinuousCondition vehicle_stop_check_;

  double stop_hold_acceleration_;
  bool enable_auto_parking_;

  static constexpr double vehicle_stop_duration_ = 1.0;
  static constexpr double vehicle_stop_timeout_ = 1.0;
};

}  // namespace autoware::stop_mode_operator

#endif  // STOP_MODE_OPERATOR_HPP_
