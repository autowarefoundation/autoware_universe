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

#include <string>

namespace autoware::default_adapi
{

ManualControlNode::ManualControlNode(const rclcpp::NodeOptions & options)
: Node("manual_control", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Handle target operation mode
  {
    const auto convert_operation_mode = [](const std::string & mode) {
      if (mode == "remote") return OperationModeState::REMOTE;
      if (mode == "local") return OperationModeState::LOCAL;
      throw std::invalid_argument("The target operation mode is invalid.");
    };
    const auto mode = declare_parameter<std::string>("mode");
    ns_ = "/api/manual/" + mode;
    target_operation_mode_ = convert_operation_mode(mode);
  }

  // Interfaces for manual control mode.
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

  sub_steering_.reset();
}

void ManualControlNode::enable_pedals_commands()
{
  sub_pedals_ = create_subscription<PedalsCommand>(
    ns_ + "/command/pedals", rclcpp::QoS(1),
    [this](const PedalsCommand::SharedPtr msg) { (void)msg; });
}

void ManualControlNode::enable_acceleration_commands()
{
  // TODO(isamu-takagi): Implement callback.
  sub_acceleration_ = create_subscription<AccelerationCommand>(
    ns_ + "/command/acceleration", rclcpp::QoS(1),
    [this](const AccelerationCommand::SharedPtr msg) { (void)msg; });
}

void ManualControlNode::enable_velocity_commands()
{
  // TODO(isamu-takagi): Implement callback.
  sub_velocity_ = create_subscription<VelocityCommand>(
    ns_ + "/command/velocity", rclcpp::QoS(1),
    [this](const VelocityCommand::SharedPtr msg) { (void)msg; });
}

void ManualControlNode::enable_common_commands()
{
  sub_steering_ = create_subscription<SteeringCommand>(
    ns_ + "/command/steering", rclcpp::QoS(1),
    [this](const SteeringCommand::SharedPtr msg) { (void)msg; });
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::ManualControlNode)
