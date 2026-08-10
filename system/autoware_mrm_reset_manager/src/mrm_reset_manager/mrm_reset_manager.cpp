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

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace
{

/// Log and report whether an output service server is available.
template <typename ClientT>
bool is_service_ready(const ClientT & client, const char * label, const rclcpp::Logger & logger)
{
  if (client->service_is_ready()) {
    return true;
  }
  RCLCPP_INFO(logger, "Waiting for %s service", label);
  return false;
}

/// Read the outcome of a response that carries a ResponseStatus.
template <typename ResponseT>
std::pair<bool, std::string> read_result(const ResponseT & response)
{
  return std::make_pair(response.status.success, response.status.message);
}

/// std_srvs/SetBool keeps the flag and the message at the top level instead.
std::pair<bool, std::string> read_result(const std_srvs::srv::SetBool::Response & response)
{
  return std::make_pair(response.success, response.message);
}

/// Sends requests to the output services and blocks until the response or the timeout.
/// Holds the logger and the timeout so that each call only needs its own arguments.
class ServiceCaller
{
public:
  ServiceCaller(const rclcpp::Logger & logger, std::chrono::milliseconds timeout)
  : logger_(logger), timeout_(timeout)
  {
  }

  /// Returns false on timeout or on a failure response, and fills `message` with the reason.
  template <typename ServiceT>
  bool call(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request, const char * label,
    std::string & message) const
  {
    auto future = client->async_send_request(request).future.share();

    if (future.wait_for(timeout_) != std::future_status::ready) {
      return fail(std::string(label) + " service timeout", message);
    }

    const auto [success, response_message] = read_result(*future.get());
    if (!success) {
      return fail(
        response_message.empty() ? std::string(label) + " service failed" : response_message,
        message);
    }
    return true;
  }

  /// Same as above, with a default-constructed request.
  template <typename ServiceT>
  bool call(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client, const char * label,
    std::string & message) const
  {
    return call<ServiceT>(client, std::make_shared<typename ServiceT::Request>(), label, message);
  }

private:
  bool fail(const std::string & reason, std::string & message) const
  {
    message = reason;
    RCLCPP_ERROR(logger_, "%s", message.c_str());
    return false;
  }

  rclcpp::Logger logger_;
  std::chrono::milliseconds timeout_;
};

}  // namespace

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

  if (init_state_ == InitState::DONE) {
    return;
  }

  while (init_state_ != InitState::DONE) {
    if (!run_init_step(init_state_)) {
      return;  // Retried on the next tick.
    }
    init_state_ = next_init_state(init_state_);
  }

  finish_initialization();
}

bool MrmResetManager::run_init_step(InitState state)
{
  switch (state) {
    case InitState::WAIT_SERVICES_READY:
      return are_required_services_ready();

    case InitState::SET_AGGREGATOR_INIT:
      return set_initializing_flag(
        cli_set_aggregator_initializing_, "set_aggregator_initializing", true,
        is_aggregator_initializing_);

    case InitState::RESET_SWITCHER:
      return call_reset_redundancy_switcher();

    case InitState::SET_SWITCHER_INTERFACE_INIT:
      return set_initializing_flag(
        cli_set_redundancy_switcher_interface_initializing_,
        "set_redundancy_switcher_interface_initializing", true,
        is_redundancy_switcher_interface_initializing_);

    case InitState::DONE:
      break;
  }
  return true;
}

MrmResetManager::InitState MrmResetManager::next_init_state(InitState state)
{
  static constexpr std::array<InitState, 5> sequence{
    InitState::SET_AGGREGATOR_INIT, InitState::RESET_SWITCHER,
    InitState::SET_SWITCHER_INTERFACE_INIT, InitState::DONE, InitState::DONE};
  return sequence.at(static_cast<std::size_t>(state));
}

bool MrmResetManager::are_required_services_ready() const
{
  if (!is_service_ready(cli_reset_diag_graph_, "reset_diag_graph", get_logger())) {
    return false;
  }
  if (!is_service_ready(
        cli_set_aggregator_initializing_, "set_aggregator_initializing", get_logger())) {
    return false;
  }
  if (!is_redundant_) {
    return true;
  }
  if (!is_service_ready(
        cli_reset_redundancy_switcher_, "reset_redundancy_switcher", get_logger())) {
    return false;
  }
  return is_service_ready(
    cli_set_redundancy_switcher_interface_initializing_,
    "set_redundancy_switcher_interface_initializing", get_logger());
}

void MrmResetManager::finish_initialization()
{
  init_timer_->cancel();
  reset_redundancy_switcher_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(5), [this]() { on_periodic_reset_check(); });
  apply_ready_state();
}

void MrmResetManager::on_periodic_reset_check()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!is_initializing()) {
    return;
  }
  if (is_autoware_ready()) {
    apply_ready_state();
    return;
  }
  if (!call_reset_redundancy_switcher()) {
    RCLCPP_WARN(get_logger(), "Periodic reset_redundancy_switcher failed");
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
  if (!is_ready_for_operation()) {
    return;
  }
  if (!enable_autoware_ready_actions_) {
    return;
  }

  leave_initializing_phase();
}

bool MrmResetManager::is_ready_for_operation() const
{
  if (localization_initialization_state_ptr_->state != LocalizationState::INITIALIZED) {
    return false;
  }
  if (route_state_ptr_->state != RouteState::SET) {
    return false;
  }
  return operation_mode_state_ptr_->is_autoware_control_enabled;
}

void MrmResetManager::leave_initializing_phase()
{
  if (!set_initializing_flag(
        cli_set_aggregator_initializing_, "set_aggregator_initializing", false,
        is_aggregator_initializing_)) {
    return;
  }
  if (!call_reset_redundancy_switcher()) {
    return;
  }
  (void)set_initializing_flag(
    cli_set_redundancy_switcher_interface_initializing_,
    "set_redundancy_switcher_interface_initializing", false,
    is_redundancy_switcher_interface_initializing_);
}

bool MrmResetManager::set_initializing_flag(
  const rclcpp::Client<SetBool>::SharedPtr & client, const char * label, bool initializing,
  bool & flag)
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = initializing;

  const ServiceCaller caller{get_logger(), std::chrono::milliseconds(service_timeout_ms_)};

  std::string message;
  if (!caller.call<SetBool>(client, request, label, message)) {
    return false;
  }
  flag = initializing;
  return true;
}

bool MrmResetManager::call_reset_redundancy_switcher(std::string & message)
{
  if (!is_redundant_) {
    return true;
  }
  const ServiceCaller caller{get_logger(), std::chrono::milliseconds(service_timeout_ms_)};
  return caller.call<ResetRedundancySwitcher>(
    cli_reset_redundancy_switcher_, "reset_redundancy_switcher", message);
}

bool MrmResetManager::call_reset_redundancy_switcher()
{
  std::string message;
  return call_reset_redundancy_switcher(message);
}

bool MrmResetManager::call_reset_diag_graph(std::string & message)
{
  const ServiceCaller caller{get_logger(), std::chrono::milliseconds(service_timeout_ms_)};
  return caller.call<ResetDiagGraph>(cli_reset_diag_graph_, "reset_diag_graph", message);
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
