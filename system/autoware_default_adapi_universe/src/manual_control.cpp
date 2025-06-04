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

#include "manual_control.hpp"

#include "utils/command_conversion.hpp"

#include <string>

namespace autoware::default_adapi
{

ManualControlNode::ManualControlNode(const rclcpp::NodeOptions & options)
: Node("manual_control", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Handle target operation mode
  const auto mode_name = declare_parameter<std::string>("mode");
  {
    const auto convert_operation_mode = [](const std::string & mode_name) {
      if (mode_name == "remote") return OperationModeState::REMOTE;
      if (mode_name == "local") return OperationModeState::LOCAL;
      throw std::invalid_argument("The target operation mode is invalid.");
    };
    ns_ = "/api/manual/" + mode_name;
    target_operation_mode_ = convert_operation_mode(mode_name);
  }

  // Interfaces for internal.
  pub_heartbeat_ =
    create_publisher<OperatorHeartbeat>("/external/" + mode_name + "/heartbeat", rclcpp::QoS(1));
  pub_pedals_ =
    create_publisher<PedalsCommand>("/external/" + mode_name + "/pedals_cmd", rclcpp::QoS(1));
  pub_steering_ =
    create_publisher<SteeringCommand>("/external/" + mode_name + "/steering_cmd", rclcpp::QoS(1));
  pub_gear_ =
    create_publisher<InternalGear>("/external/" + mode_name + "/gear_cmd", rclcpp::QoS(1));
  pub_turn_indicators_ = create_publisher<InternalTurnIndicators>(
    "/external/" + mode_name + "/turn_indicators_cmd", rclcpp::QoS(1));
  pub_hazard_lights_ = create_publisher<InternalHazardLights>(
    "/external/" + mode_name + "/hazard_lights_cmd", rclcpp::QoS(1));

  // Interfaces for AD API.
  srv_list_mode_ = create_service<ListMode>(
    ns_ + "/control_mode/list", std::bind(&ManualControlNode::on_list_mode, this, _1, _2));
  srv_select_mode_ = create_service<SelectMode>(
    ns_ + "/control_mode/select", std::bind(&ManualControlNode::on_select_mode, this, _1, _2));
  pub_mode_status_ = create_publisher<ManualControlModeStatus>(
    ns_ + "/control_mode/status", rclcpp::QoS(1).transient_local());
  sub_operation_mode_ = PollingSubscription<OperationModeState>::create_subscription(
    this, "/api/operation_mode/state", rclcpp::QoS(1).transient_local());

  // Initialize the current manual control mode.
  update_mode_status(ManualControlMode::DISABLED);
}

void ManualControlNode::update_mode_status(uint8_t mode)
{
  current_mode_.mode = mode;

  ManualControlModeStatus msg;
  msg.stamp = now();
  msg.mode = current_mode_;
  pub_mode_status_->publish(msg);
}

void ManualControlNode::on_list_mode(
  ListMode::Request::SharedPtr, ListMode::Response::SharedPtr res)
{
  ManualControlMode mode;
  mode.mode = ManualControlMode::PEDALS;
  res->modes.push_back(mode);
  res->status.success = true;
}

void ManualControlNode::on_select_mode(
  SelectMode::Request::SharedPtr req, SelectMode::Response::SharedPtr res)
{
  const auto operation_mode = sub_operation_mode_->take_data();
  if (!operation_mode) {
    res->status.success = false;
    res->status.message = "The operation mode could not be received.";
    return;
  }
  if (operation_mode->mode == target_operation_mode_) {
    res->status.success = false;
    res->status.message = "The manual control mode cannot be changed during operation.";
    return;
  }

  switch (req->mode.mode) {
    case ManualControlMode::DISABLED:
      disable_all_commands();
      update_mode_status(ManualControlMode::DISABLED);
      res->status.success = true;
      break;
    case ManualControlMode::PEDALS:
      disable_all_commands();
      enable_common_commands();
      enable_pedals_commands();
      update_mode_status(req->mode.mode);
      res->status.success = true;
      break;
    case ManualControlMode::ACCELERATION:
    case ManualControlMode::VELOCITY:
      disable_all_commands();
      update_mode_status(ManualControlMode::DISABLED);
      res->status.success = false;
      res->status.message = "The selected control mode is not supported.";
      break;
    default:
      disable_all_commands();
      update_mode_status(ManualControlMode::DISABLED);
      res->status.success = false;
      res->status.message = "The selected control mode is invalid.";
      break;
  }
}

void ManualControlNode::disable_all_commands()
{
  sub_pedals_.reset();
  sub_acceleration_.reset();
  sub_velocity_.reset();

  sub_heartbeat_.reset();
  sub_steering_.reset();
  sub_gear_.reset();
  sub_turn_indicators_.reset();
  sub_hazard_lights_.reset();
}

void ManualControlNode::enable_pedals_commands()
{
  sub_pedals_ = create_subscription<PedalsCommand>(
    ns_ + "/command/pedals", rclcpp::QoS(1).best_effort(),
    [this](const PedalsCommand & msg) { pub_pedals_->publish(msg); });
}

void ManualControlNode::enable_acceleration_commands()
{
  // TODO(isamu-takagi): Currently not supported.
  sub_acceleration_ = create_subscription<AccelerationCommand>(
    ns_ + "/command/acceleration", rclcpp::QoS(1).best_effort(),
    [](const AccelerationCommand & msg) { (void)msg; });
}

void ManualControlNode::enable_velocity_commands()
{
  // TODO(isamu-takagi): Currently not supported.
  sub_velocity_ = create_subscription<VelocityCommand>(
    ns_ + "/command/velocity", rclcpp::QoS(1).best_effort(),
    [](const VelocityCommand & msg) { (void)msg; });
}

void ManualControlNode::enable_common_commands()
{
  using autoware::default_adapi::command_conversion::convert;

  sub_heartbeat_ = create_subscription<OperatorHeartbeat>(
    ns_ + "/operator/heartbeat", rclcpp::QoS(1).best_effort(),
    [this](const OperatorHeartbeat & msg) { pub_heartbeat_->publish(msg); });
  sub_steering_ = create_subscription<SteeringCommand>(
    ns_ + "/command/steering", rclcpp::QoS(1).best_effort(),
    [this](const SteeringCommand & msg) { pub_steering_->publish(msg); });
  sub_gear_ = create_subscription<GearCommand>(
    ns_ + "/command/gear", rclcpp::QoS(1).transient_local(),
    [this](const GearCommand & msg) { pub_gear_->publish(convert(msg)); });
  sub_turn_indicators_ = create_subscription<TurnIndicatorsCommand>(
    ns_ + "/command/turn_indicators", rclcpp::QoS(1).transient_local(),
    [this](const TurnIndicatorsCommand & msg) { pub_turn_indicators_->publish(convert(msg)); });
  sub_hazard_lights_ = create_subscription<HazardLightsCommand>(
    ns_ + "/command/hazard_lights", rclcpp::QoS(1).transient_local(),
    [this](const HazardLightsCommand & msg) { pub_hazard_lights_->publish(convert(msg)); });
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::ManualControlNode)
