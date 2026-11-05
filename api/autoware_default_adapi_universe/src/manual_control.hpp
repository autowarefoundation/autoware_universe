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

#ifndef MANUAL_CONTROL_HPP_
#define MANUAL_CONTROL_HPP_

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware_utils_diagnostics/timeout_diagnostics.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/acceleration_command.hpp>
#include <autoware_adapi_v1_msgs/msg/gear_command.hpp>
#include <autoware_adapi_v1_msgs/msg/hazard_lights_command.hpp>
#include <autoware_adapi_v1_msgs/msg/manual_control_mode.hpp>
#include <autoware_adapi_v1_msgs/msg/manual_control_mode_status.hpp>
#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators_command.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_command.hpp>
#include <autoware_adapi_v1_msgs/srv/list_manual_control_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/select_manual_control_mode.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <memory>
#include <string>

namespace autoware::default_adapi
{

class ManualControlNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit ManualControlNode(const rclcpp::NodeOptions & options);

private:
  template <class T>
  using PollingSubscription = autoware::agnocast_wrapper::polling::PollingSubscriber<T>;
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  PollingSubscription<OperationModeState>::SharedPtr sub_operation_mode_;

  using ManualControlModeStatus = autoware_adapi_v1_msgs::msg::ManualControlModeStatus;
  using ManualControlMode = autoware_adapi_v1_msgs::msg::ManualControlMode;
  using ListMode = autoware_adapi_v1_msgs::srv::ListManualControlMode;
  using SelectMode = autoware_adapi_v1_msgs::srv::SelectManualControlMode;
  void update_mode_status(uint8_t mode);
  void on_list_mode(ListMode::Request::SharedPtr req, ListMode::Response::SharedPtr res);
  void on_select_mode(SelectMode::Request::SharedPtr req, SelectMode::Response::SharedPtr res);
  AUTOWARE_SERVICE_PTR(ListMode) srv_list_mode_;
  AUTOWARE_SERVICE_PTR(SelectMode) srv_select_mode_;
  AUTOWARE_PUBLISHER_PTR(ManualControlModeStatus) pub_mode_status_;

  using PedalsCommand = autoware_adapi_v1_msgs::msg::PedalsCommand;
  using AccelerationCommand = autoware_adapi_v1_msgs::msg::AccelerationCommand;
  using VelocityCommand = autoware_adapi_v1_msgs::msg::VelocityCommand;
  using SteeringCommand = autoware_adapi_v1_msgs::msg::SteeringCommand;
  using GearCommand = autoware_adapi_v1_msgs::msg::GearCommand;
  using HazardLightsCommand = autoware_adapi_v1_msgs::msg::HazardLightsCommand;
  using TurnIndicatorsCommand = autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand;
  using OperatorHeartbeat = autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
  void disable_all_commands();
  void enable_common_commands();
  void enable_pedals_commands();
  // TODO(isamu-takagi): These functions are reserved for future support.
  // void enable_acceleration_commands();
  // void enable_velocity_commands();
  AUTOWARE_SUBSCRIPTION_PTR(OperatorHeartbeat) sub_heartbeat_;
  AUTOWARE_SUBSCRIPTION_PTR(PedalsCommand) sub_pedals_;
  AUTOWARE_SUBSCRIPTION_PTR(AccelerationCommand) sub_acceleration_;
  AUTOWARE_SUBSCRIPTION_PTR(VelocityCommand) sub_velocity_;
  AUTOWARE_SUBSCRIPTION_PTR(SteeringCommand) sub_steering_;
  AUTOWARE_SUBSCRIPTION_PTR(GearCommand) sub_gear_;
  AUTOWARE_SUBSCRIPTION_PTR(TurnIndicatorsCommand) sub_turn_indicators_;
  AUTOWARE_SUBSCRIPTION_PTR(HazardLightsCommand) sub_hazard_lights_;

  using InternalGear = autoware_vehicle_msgs::msg::GearCommand;
  using InternalTurnIndicators = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using InternalHazardLights = autoware_vehicle_msgs::msg::HazardLightsCommand;
  AUTOWARE_PUBLISHER_PTR(OperatorHeartbeat) pub_heartbeat_;
  AUTOWARE_PUBLISHER_PTR(PedalsCommand) pub_pedals_;
  AUTOWARE_PUBLISHER_PTR(SteeringCommand) pub_steering_;
  AUTOWARE_PUBLISHER_PTR(InternalGear) pub_gear_;
  AUTOWARE_PUBLISHER_PTR(InternalTurnIndicators) pub_turn_indicators_;
  AUTOWARE_PUBLISHER_PTR(InternalHazardLights) pub_hazard_lights_;

  uint8_t target_operation_mode_;
  std::string ns_;
  ManualControlMode current_mode_;
  std::unique_ptr<autoware::agnocast_wrapper::diagnostic_updater::Updater> diag_updater_;
  std::unique_ptr<autoware_utils_diagnostics::TimeoutDiag> diag_heartbeat_;
};

}  // namespace autoware::default_adapi

#endif  // MANUAL_CONTROL_HPP_
