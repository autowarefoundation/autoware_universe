// Copyright 2023 TIER IV, Inc.
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

#include "vehicle_door.hpp"

#include "utils/topics.hpp"

namespace autoware::default_adapi
{

VehicleDoorNode::VehicleDoorNode(const rclcpp::NodeOptions & options)
: Node("vehicle_door", options), diagnostics_(this)
{
  diagnostics_.setHardwareID("none");
  diagnostics_.add("state", this, &VehicleDoorNode::diagnose_state);

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.relay_service(cli_layout_, srv_layout_, group_cli_);
  adaptor.init_cli(cli_command_, group_cli_);
  adaptor.init_srv(srv_command_, this, &VehicleDoorNode::on_command);
  adaptor.init_pub(pub_status_);
  adaptor.init_sub(sub_status_, this, &VehicleDoorNode::on_status);
  adaptor.init_sub(sub_operation_mode_, this, &VehicleDoorNode::on_operation_mode);

  check_autoware_control_ = declare_parameter<bool>("check_autoware_control");
  is_autoware_control_ = false;
  is_stop_mode_ = false;
}

void VehicleDoorNode::diagnose_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  if (!status_) {
    stat.summary(DiagnosticStatus::ERROR, "The door status is unknown.");
    return;
  }

  bool is_closed = true;
  for (const auto & door : status_->doors) {
    if (door.status != autoware_adapi_v1_msgs::msg::DoorStatus::CLOSED) {
      is_closed = false;
      break;
    }
  }
  if (is_closed) {
    stat.summary(DiagnosticStatus::OK, "");
  } else {
    stat.summary(DiagnosticStatus::ERROR, "The door is open.");
  }
}

void VehicleDoorNode::on_operation_mode(const OperationModeState::Message::ConstSharedPtr msg)
{
  is_autoware_control_ = msg->is_autoware_control_enabled;
  is_stop_mode_ = msg->mode == OperationModeState::Message::STOP;
}
void VehicleDoorNode::on_status(InternalDoorStatus::Message::ConstSharedPtr msg)
{
  utils::notify(pub_status_, status_, *msg, utils::ignore_stamp<InternalDoorStatus::Message>);
}

void VehicleDoorNode::on_command(
  const ExternalDoorCommand::Service::Request::SharedPtr req,
  const ExternalDoorCommand::Service::Response::SharedPtr res)
{
  if (!is_autoware_control_ && check_autoware_control_) {
    res->status.success = false;
    res->status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::UNKNOWN;
    res->status.message = "The door cannot be opened when autoware control is disabled.";
    return;
  }

  // For safety, do not open the door if the vehicle is not stopped.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/list/api/vehicle/doors/command/
  bool is_open = false;
  for (const auto & door : req->doors) {
    if (door.command == autoware_adapi_v1_msgs::msg::DoorCommand::OPEN) {
      is_open = true;
      break;
    }
  }
  if (!is_stop_mode_ && is_open) {
    res->status.success = false;
    res->status.code = autoware_adapi_v1_msgs::msg::ResponseStatus::UNKNOWN;
    res->status.message = "The door cannot be opened except in stop mode.";
    return;
  }
  autoware::component_interface_utils::status::copy(cli_command_->call(req), res);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::VehicleDoorNode)
