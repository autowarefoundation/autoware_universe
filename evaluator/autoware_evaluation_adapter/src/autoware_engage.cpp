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
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  srv_engage_ = create_service<EngageService>(
    "/api/external/set/engage", std::bind(&AutowareEngage::on_engage, this, _1, _2));
  sub_operation_mode_state_ = create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS(1).transient_local(),
    std::bind(&AutowareEngage::on_state, this, _1));
  cli_change_stop_mode_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_stop", service_qos, callback_group_);
  cli_change_autonomous_mode_ = create_client<ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous", service_qos, callback_group_);

  state_.mode = OperationModeState::UNKNOWN;
}

void AutowareEngage::on_state(const OperationModeState & msg)
{
  state_ = msg;
}

void AutowareEngage::on_engage(
  const EngageService::Request::SharedPtr req, EngageService::Response::SharedPtr res)
{
  const auto request = std::make_shared<ChangeOperationMode::Request>();
  const bool is_autonomous_mode = state_.mode == OperationModeState::AUTONOMOUS;

  if (req->engage && is_autonomous_mode) {
    res->status = utils::response_ignored("It is already engaged.");
    return;
  }

  const auto client = req->engage ? cli_change_autonomous_mode_ : cli_change_stop_mode_;
  const auto [status, response] = utils::sync_call<ChangeOperationMode>(client, request);
  if (utils::is_error(status)) {
    res->status = status;
    return;
  }

  if (response->status.success) {
    res->status = utils::response_success();
  } else {
    res->status = utils::response_error(response->status.message);
  }
}

}  // namespace autoware::evaluation_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::evaluation_adapter::AutowareEngage)
