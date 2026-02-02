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

#include "autoware_engage.hpp"

#include "utils/client.hpp"
#include "utils/response.hpp"

#include <autoware/qos_utils/qos_compatibility.hpp>

#include <memory>
#include <string>

namespace autoware::evaluation_adapter
{

AutowareEngage::AutowareEngage(const rclcpp::NodeOptions & options)
: Node("autoware_engage", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto service_qos = AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE();

  autoware_control_change_ = declare_parameter("autoware_control_change", false);
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pub_engage_ = create_publisher<EngageStatus>("/api/external/get/engage", rclcpp::QoS(1));
  srv_engage_ = create_service<EngageService>(
    "/api/external/set/engage", std::bind(&AutowareEngage::on_engage, this, _1, _2));

  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    std::bind(&AutowareEngage::on_state, this, _1));
  cli_change_stop_mode_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_stop", service_qos, callback_group_);
  cli_change_autonomous_mode_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous", service_qos, callback_group_);
  cli_enable_autoware_control_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/enable_autoware_control", service_qos, callback_group_);
  cli_disable_autoware_control_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/disable_autoware_control", service_qos, callback_group_);

  state_.mode = OperationModeState::UNKNOWN;
}

void AutowareEngage::on_state(const OperationModeState & msg)
{
  state_ = msg;

  EngageStatus status;
  status.stamp = now();
  status.engage = msg.mode == OperationModeState::AUTONOMOUS;
  pub_engage_->publish(status);
}

void AutowareEngage::on_engage(
  const EngageService::Request::SharedPtr req, EngageService::Response::SharedPtr res)
{
  using tier4_external_api_msgs::msg::ResponseStatus;
  const auto create_response = [](uint16_t code, const std::string & message) {
    ResponseStatus status;
    status.code = code;
    status.message = message;
    return status;
  };

  const auto request = std::make_shared<ChangeOperationMode::Request>();
  const bool is_autonomous_mode = state_.mode == OperationModeState::AUTONOMOUS;
  const bool is_autoware_control = state_.is_autoware_control_enabled;

  if (req->engage && is_autonomous_mode && is_autoware_control) {
    res->status = create_response(ResponseStatus::IGNORED, "It is already engaged.");
    return;
  }

  if (autoware_control_change_) {
    const auto client = req->engage ? cli_enable_autoware_control_ : cli_disable_autoware_control_;
    const auto [status, response] = utils::sync_call<ChangeOperationMode>(client, request);
    if (utils::is_error(status)) {
      res->status = status;
      return;
    }
  }

  const auto client = req->engage ? cli_change_autonomous_mode_ : cli_change_stop_mode_;
  const auto [status, response] = utils::sync_call<ChangeOperationMode>(client, request);
  if (utils::is_error(status)) {
    res->status = status;
    return;
  }
  res->status.code = response->status.success ? ResponseStatus::SUCCESS : ResponseStatus::ERROR;
  res->status.message = response->status.message;
}

}  // namespace autoware::evaluation_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::evaluation_adapter::AutowareEngage)
