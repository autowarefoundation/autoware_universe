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

#include "command_mode_decider.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::command_mode_decider
{

std::string mode_to_text(uint32_t mode)
{
  // clang-format off
  switch (mode) {
    case ChangeOperationMode::Request::STOP:       return "stop";
    case ChangeOperationMode::Request::AUTONOMOUS: return "autonomous";
    case ChangeOperationMode::Request::LOCAL:      return "local";
    case ChangeOperationMode::Request::REMOTE:     return "remote";
    default:                                       return "";
  }
  // clang-format on
}

uint32_t text_to_mode(const std::string & text)
{
  // clang-format off
  if (text == "stop")       return OperationModeState::STOP;
  if (text == "autonomous") return OperationModeState::AUTONOMOUS;
  if (text == "local")      return OperationModeState::LOCAL;
  if (text == "remote")     return OperationModeState::REMOTE;
  // clang-format on
  return OperationModeState::UNKNOWN;
}

template <typename T>
typename T::Response::SharedPtr sync_request(
  rclcpp::Client<T> & client, typename T::Request::SharedPtr request)
{
  auto future = client.async_send_request(request);
  return future.get();
}

CommandModeDeciderBase::CommandModeDeciderBase(const rclcpp::NodeOptions & options)
: Node("command_mode_decider", options)
{
  is_modes_ready_ = false;
  target_operation_mode_ = declare_parameter<std::string>("initial_operation_mode");
  target_mrm_ = std::string();
  curr_command_mode_ = std::string();
  command_mode_request_stamp_ = std::nullopt;

  const auto command_modes = declare_parameter<std::vector<std::string>>("command_modes");
  for (const auto & mode : command_modes) {
    // NOTE: Do not set mode as this is used to check topic reception.
    CommandModeItem item;
    item.availability.available = false;
    item.status.activation = false;
    item.status.transition = false;
    item.status.mrm = CommandModeStatusItem::UNDEFINED;
    command_modes_[mode] = item;
  }

  using std::placeholders::_1;
  using std::placeholders::_2;

  pub_command_mode_request_ =
    create_publisher<CommandModeRequest>("~/command_mode/request", rclcpp::QoS(1));
  pub_operation_mode_ = create_publisher<OperationModeState>(
    "~/operation_mode/state", rclcpp::QoS(1).transient_local());
  srv_operation_mode_ = create_service<ChangeOperationMode>(
    "~/operation_mode/change_operation_mode",
    std::bind(&CommandModeDecider::on_change_operation_mode, this, _1, _2));
  srv_request_mrm_ = create_service<RequestMrm>(
    "~/mrm/request", std::bind(&CommandModeDecider::on_request_mrm, this, _1, _2));

  sub_command_mode_availability_ = create_subscription<CommandModeAvailability>(
    "~/command_mode/availability", rclcpp::QoS(1),
    std::bind(&CommandModeDeciderBase::on_availability, this, std::placeholders::_1));
  sub_command_mode_status_ = create_subscription<CommandModeStatus>(
    "~/command_mode/status", rclcpp::QoS(1).transient_local(),
    std::bind(&CommandModeDeciderBase::on_status, this, std::placeholders::_1));

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void CommandModeDeciderBase::on_availability(const CommandModeAvailability & msg)
{
  for (const auto & item : msg.items) {
    const auto iter = command_modes_.find(item.mode);
    if (iter == command_modes_.end()) continue;
    iter->second.availability = item;
  }
  update_command_mode();
}

void CommandModeDeciderBase::on_status(const CommandModeStatus & msg)
{
  for (const auto & item : msg.items) {
    const auto iter = command_modes_.find(item.mode);
    if (iter == command_modes_.end()) continue;
    iter->second.status = item;
  }
  update_command_mode();
}

void CommandModeDeciderBase::on_timer()
{
  if (!is_modes_ready_) {
    return;
  }

  const auto & mode = command_modes_.at(curr_command_mode_);

  if (mode.status.activation) {
    command_mode_request_stamp_ = std::nullopt;
    return;
  }

  if (!command_mode_request_stamp_) {
    return;
  }

  const auto duration = (now() - *command_mode_request_stamp_).seconds();
  RCLCPP_INFO_STREAM(get_logger(), "time: " << duration);
}

void CommandModeDeciderBase::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  // TODO(Takagi, Isamu): Commonize on_change_operation_mode and on_request_mrm.

  const auto mode = mode_to_text(req->mode);
  const auto iter = command_modes_.find(mode);
  if (iter == command_modes_.end()) {
    RCLCPP_WARN_STREAM(get_logger(), "invalid mode name: " << mode);
    res->status.success = false;
    res->status.message = "invalid mode name: " + mode;
    return;
  }

  const auto status = iter->second;
  if (!status.availability.available) {
    RCLCPP_WARN_STREAM(get_logger(), "mode is not available: " << mode);
    res->status.success = false;
    res->status.message = "mode is not available: " + mode;
    return;
  }

  target_operation_mode_ = mode;
  res->status.success = true;

  update_command_mode();
}

void CommandModeDeciderBase::on_request_mrm(
  RequestMrm::Request::SharedPtr req, RequestMrm::Response::SharedPtr res)
{
  // TODO(Takagi, Isamu): Commonize on_change_operation_mode and on_request_mrm.

  const auto mode = req->name;
  const auto iter = command_modes_.find(mode);
  if (iter == command_modes_.end()) {
    RCLCPP_WARN_STREAM(get_logger(), "invalid mode name: " << mode);
    res->status.success = false;
    res->status.message = "invalid mode name: " + mode;
    return;
  }

  const auto status = iter->second;
  if (!status.availability.available) {
    RCLCPP_WARN_STREAM(get_logger(), "mode is not available: " << mode);
    res->status.success = false;
    res->status.message = "mode is not available: " + mode;
    return;
  }

  target_mrm_ = mode;
  res->status.success = true;

  update_command_mode();
}

void CommandModeDeciderBase::update_command_mode()
{
  if (!is_modes_ready_) {
    for (const auto & [mode, item] : command_modes_) {
      if (item.availability.mode.empty() || item.status.mode.empty()) {
        return;
      }
    }
    is_modes_ready_ = true;
  }

  const auto stamp = now();

  // Decide target command mode.
  bool is_command_mode_changed = false;
  {
    const auto next_command_mode = decide_command_mode();
    is_command_mode_changed = curr_command_mode_ != next_command_mode;
    if (is_command_mode_changed) {
      const auto curr_text = "'" + curr_command_mode_ + "'";
      const auto next_text = "'" + next_command_mode + "'";
      RCLCPP_INFO_STREAM(
        get_logger(), "command mode changed: " << curr_text << " -> " << next_text);
    }
    curr_command_mode_ = next_command_mode;
  }

  // Request command mode to switcher nodes.
  if (is_command_mode_changed) {
    CommandModeRequest msg;
    msg.stamp = stamp;
    msg.mode = curr_command_mode_;
    pub_command_mode_request_->publish(msg);

    command_mode_request_stamp_ = stamp;
  }

  // Update operation mode status.
  const auto is_available = [this](const auto & mode) {
    const auto iter = command_modes_.find(mode);
    return iter == command_modes_.end() ? false : iter->second.availability.available;
  };
  OperationModeState state;
  state.stamp = stamp;
  state.mode = text_to_mode(target_operation_mode_);
  state.is_autoware_control_enabled = true;  // TODO(Takagi, Isamu): subscribe
  state.is_in_transition = false;            // TODO(Takagi, Isamu): check status is enabled
  state.is_stop_mode_available = is_available("stop");
  state.is_autonomous_mode_available = is_available("autonomous");
  state.is_local_mode_available = is_available("local");
  state.is_remote_mode_available = is_available("remote");
  pub_operation_mode_->publish(state);
}

CommandModeDecider::CommandModeDecider(const rclcpp::NodeOptions & options)
: CommandModeDeciderBase(options)
{
}

std::string CommandModeDecider::decide_command_mode()
{
  const auto command_modes = get_command_modes();
  const auto target_operation_mode = get_target_operation_mode();
  const auto target_mrm = get_target_mrm();

  // Use the requested MRM if available.
  {
    const auto iter = command_modes.find(target_mrm);
    if (iter != command_modes.end()) {
      const auto [mode, status] = *iter;
      if (status.availability.available) {
        return mode;
      }
    }
  }

  // Use the specified operation mode if available.
  {
    const auto iter = command_modes.find(target_operation_mode);
    if (iter != command_modes.end()) {
      const auto [mode, status] = *iter;
      if (status.availability.available) {
        return mode;
      }
    }
  }

  // TODO(Takagi, Isamu): Use the available MRM according to the state transitions at the
  // following.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/#behavior
  const auto pull_over = "pull_over";
  const auto comfortable_stop = "comfortable_stop";
  const auto emergency_stop = "emergency_stop";

  const auto is_available = [](const auto & command_modes, const auto & mode) {
    const auto iter = command_modes.find(mode);
    return iter == command_modes.end() ? false : iter->second.availability.available;
  };

  // TODO(Takagi, Isamu): check command_modes parameter
  if (is_available(command_modes, pull_over) /*&& use_pull_over_*/) {
    return pull_over;
  }
  if (is_available(command_modes, comfortable_stop) /*&& use_comfortable_stop_*/) {
    return comfortable_stop;
  }
  if (is_available(command_modes, emergency_stop)) {
    return emergency_stop;
  }

  // Use an empty string to delegate to switcher node.
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000, "no mrm available: delegate to switcher node");
  return std::string();
}

}  // namespace autoware::command_mode_decider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_decider::CommandModeDecider)
