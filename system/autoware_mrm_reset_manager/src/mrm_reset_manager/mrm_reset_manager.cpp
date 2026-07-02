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

#include "mrm_reset_manager.hpp"

#include <autoware_common_msgs/msg/response_status.hpp>
#include <tier4_external_api_msgs/msg/response_status.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace autoware::mrm_reset_manager
{

MrmResetManager::MrmResetManager(const rclcpp::NodeOptions & options)
: Node("autoware_mrm_reset_manager", options)
{
  service_timeout_ms_ = declare_parameter<int>("service_timeout_ms");
  is_redundant_ = declare_parameter<bool>("is_redundant");
  enable_autoware_ready_actions_ = declare_parameter<bool>("enable_autoware_ready_actions");

  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_set_aggregator_initializing_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_set_redundancy_switcher_interface_initializing_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_reset_redundancy_switcher_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_reset_diag_graph_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_reset_mrm_ = create_service<ResetMrm>(
    "~/input/reset_mrm", std::bind(&MrmResetManager::on_reset_mrm, this, _1, _2),
    rmw_qos_profile_services_default, service_callback_group_);

  const auto qos = rclcpp::QoS(1).transient_local();
  sub_localization_initialization_state_ = create_subscription<LocalizationState>(
    "~/input/localization_initialization_state", qos,
    [this](const LocalizationState::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      localization_initialization_state_ptr_ = msg;
      apply_ready_state();
    });
  sub_route_state_ = create_subscription<RouteState>(
    "~/input/route_state", qos, [this](const RouteState::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      route_state_ptr_ = msg;
      apply_ready_state();
    });
  sub_operation_mode_state_ = create_subscription<OperationMode>(
    "~/input/operation_mode_state", qos, [this](const OperationMode::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      operation_mode_state_ptr_ = msg;
      apply_ready_state();
    });

  cli_set_aggregator_initializing_ = create_client<SetBool>(
    "~/output/set_aggregator_initializing", rmw_qos_profile_services_default,
    cli_set_aggregator_initializing_callback_group_);
  cli_set_redundancy_switcher_interface_initializing_ = create_client<SetBool>(
    "~/output/set_redundancy_switcher_interface_initializing", rmw_qos_profile_services_default,
    cli_set_redundancy_switcher_interface_initializing_callback_group_);
  cli_reset_redundancy_switcher_ = create_client<ResetRedundancySwitcher>(
    "~/output/reset_redundancy_switcher", rmw_qos_profile_services_default,
    cli_reset_redundancy_switcher_callback_group_);
  cli_reset_diag_graph_ = create_client<ResetDiagGraph>(
    "~/output/reset_diag_graph", rmw_qos_profile_services_default,
    cli_reset_diag_graph_callback_group_);

  init_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(1), [this]() { advance_init_state(); });
}

void MrmResetManager::on_reset_mrm(
  const ResetMrm::Request::SharedPtr /*request*/, ResetMrm::Response::SharedPtr response)
{
  using ExtApi = tier4_external_api_msgs::msg::ResponseStatus;

  std::string message;
  const bool ok_diag = call_reset_diag_graph(message);
  const bool ok_switcher = call_reset_redundancy_switcher(message);

  if (ok_diag && ok_switcher) {
    response->status.code = ExtApi::SUCCESS;
    response->status.message = "";
    return;
  }
  response->status.code = ExtApi::ERROR;
  response->status.message = message.empty() ? "reset_mrm forwarding failed" : message;
}

void MrmResetManager::advance_init_state()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  switch (init_state_) {
    case InitState::WAIT_SERVICES_READY:
      if (!cli_set_aggregator_initializing_->service_is_ready()) {
        RCLCPP_INFO(get_logger(), "Waiting for set_aggregator_initializing service");
        return;
      }
      if (is_redundant_ && !cli_reset_redundancy_switcher_->service_is_ready()) {
        RCLCPP_INFO(get_logger(), "Waiting for reset_redundancy_switcher service");
        return;
      }
      if (!cli_set_redundancy_switcher_interface_initializing_->service_is_ready()) {
        RCLCPP_INFO(
          get_logger(), "Waiting for set_redundancy_switcher_interface_initializing service");
        return;
      }
      init_state_ = InitState::SET_AGGREGATOR_INIT;
      [[fallthrough]];

    case InitState::SET_AGGREGATOR_INIT:
      if (!set_aggregator_initializing(true)) {
        return;
      }
      init_state_ = InitState::RESET_SWITCHER;
      [[fallthrough]];

    case InitState::RESET_SWITCHER:
      if (!call_reset_redundancy_switcher()) {
        return;
      }
      init_state_ = InitState::SET_SWITCHER_IFACE_INIT;
      [[fallthrough]];

    case InitState::SET_SWITCHER_IFACE_INIT:
      if (!set_redundancy_switcher_interface_initializing(true)) {
        return;
      }
      init_state_ = InitState::DONE;
      init_timer_->cancel();
      reset_redundancy_switcher_timer_ =
        rclcpp::create_timer(this, get_clock(), std::chrono::seconds(5), [this]() {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (!is_initializing()) {
            return;
          }
          if (is_autoware_ready()) {
            apply_ready_state();
          } else if (!call_reset_redundancy_switcher()) {
            RCLCPP_WARN(get_logger(), "Periodic reset_redundancy_switcher failed");
          }
        });
      apply_ready_state();
      break;

    case InitState::DONE:
      break;
  }
}

void MrmResetManager::apply_ready_state()
{
  if (init_state_ != InitState::DONE) {
    return;
  }
  if (!is_autoware_ready()) {
    return;
  }
  if (!is_initializing()) {
    return;
  }

  const bool localized =
    localization_initialization_state_ptr_->state == LocalizationState::INITIALIZED;
  const bool route_set = route_state_ptr_->state == RouteState::SET;
  const bool autoware_control = operation_mode_state_ptr_->is_autoware_control_enabled;

  if (localized && route_set && autoware_control) {
    if (!enable_autoware_ready_actions_) {
      return;
    }

    if (!set_aggregator_initializing(false)) {
      return;
    }
    if (!call_reset_redundancy_switcher()) {
      return;
    }
    (void)set_redundancy_switcher_interface_initializing(false);
  }
}

bool MrmResetManager::set_aggregator_initializing(bool initializing)
{
  auto req = std::make_shared<SetBool::Request>();
  req->data = initializing;

  std::string message;
  if (!call_set_bool(
        cli_set_aggregator_initializing_, req, "set_aggregator_initializing", message)) {
    return false;
  }
  is_aggregator_initializing_ = initializing;
  return true;
}

bool MrmResetManager::set_redundancy_switcher_interface_initializing(bool initializing)
{
  auto req = std::make_shared<SetBool::Request>();
  req->data = initializing;

  std::string message;
  if (!call_set_bool(
        cli_set_redundancy_switcher_interface_initializing_, req,
        "set_redundancy_switcher_interface_initializing", message)) {
    return false;
  }
  is_redundancy_switcher_interface_initializing_ = initializing;
  return true;
}

bool MrmResetManager::call_reset_redundancy_switcher(std::string & message)
{
  if (!is_redundant_) {
    return true;
  }

  auto req = std::make_shared<ResetRedundancySwitcher::Request>();
  auto future = cli_reset_redundancy_switcher_->async_send_request(req).future.share();

  if (
    future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) != std::future_status::ready) {
    message = "reset_redundancy_switcher service timeout";
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return false;
  }

  const auto res = future.get();
  if (!res->status.success) {
    message = res->status.message.empty() ? "reset_redundancy_switcher service failed"
                                          : res->status.message;
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return false;
  }
  return true;
}

bool MrmResetManager::call_reset_redundancy_switcher()
{
  std::string message;
  return call_reset_redundancy_switcher(message);
}

bool MrmResetManager::call_reset_diag_graph(std::string & message)
{
  auto req = std::make_shared<ResetDiagGraph::Request>();
  auto future = cli_reset_diag_graph_->async_send_request(req).future.share();

  if (
    future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) != std::future_status::ready) {
    message = "reset_diag_graph service timeout";
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return false;
  }

  const auto res = future.get();
  if (!res->status.success) {
    message = res->status.message.empty() ? "reset_diag_graph service failed" : res->status.message;
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return false;
  }
  return true;
}

bool MrmResetManager::call_set_bool(
  const rclcpp::Client<SetBool>::SharedPtr & client, const SetBool::Request::SharedPtr & request,
  const char * label, std::string & message)
{
  auto future = client->async_send_request(request).future.share();
  if (
    future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) != std::future_status::ready) {
    message = std::string(label) + " service timeout";
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return false;
  }

  const auto res = future.get();
  if (!res->success) {
    message = res->message.empty() ? (std::string(label) + " service failed") : res->message;
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    return false;
  }
  return true;
}

bool MrmResetManager::is_autoware_ready() const
{
  return localization_initialization_state_ptr_ && route_state_ptr_ && operation_mode_state_ptr_;
}

bool MrmResetManager::is_initializing() const
{
  return is_aggregator_initializing_ || is_redundancy_switcher_interface_initializing_;
}

}  // namespace autoware::mrm_reset_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mrm_reset_manager::MrmResetManager)
