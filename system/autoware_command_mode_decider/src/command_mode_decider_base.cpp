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

#include "command_mode_decider_base.hpp"

#include <autoware_command_mode_types/modes.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::command_mode_decider
{

void logging_mode_change(
  const rclcpp::Logger & logger, const std::string & name, uint16_t prev, uint16_t next)
{
  if (prev != next) {
    const auto prev_text = std::to_string(prev);
    const auto next_text = std::to_string(next);
    RCLCPP_INFO_STREAM(logger, name << " mode changed: " << prev_text << " -> " << next_text);
  }
}

std::string text(const std::vector<uint16_t> & modes)
{
  std::string result;
  for (size_t i = 0; i < modes.size(); ++i) {
    result += (i ? ", " : "") + std::to_string(modes.at(i));
  }
  return "[" + result + "]";
}

CommandModeDeciderBase::CommandModeDeciderBase(const rclcpp::NodeOptions & options)
: Node("command_mode_decider", options),
  diagnostics_(this),
  loader_("autoware_command_mode_decider", "autoware::command_mode_decider::DeciderPlugin")
{
  diagnostics_.setHardwareID("none");
  diagnostics_.add("ready", this, &CommandModeDeciderBase::on_diagnostics);

  const auto plugin_name = declare_parameter<std::string>("plugin_name");
  if (!loader_.isClassAvailable(plugin_name)) {
    throw std::invalid_argument("unknown plugin: " + plugin_name);
  }
  plugin_ = loader_.createSharedInstance(plugin_name);
  plugin_->construct(this);
  plugin_->initialize();

  transition_timeout_ = declare_parameter<double>("transition_timeout");
  request_timeout_ = declare_parameter<double>("request_timeout");

  // Note: declare_parameter does not support std::vector<uint16_t>
  std::vector<uint16_t> command_modes;
  for (const auto & mode : declare_parameter<std::vector<int64_t>>("command_modes")) {
    command_modes.push_back(static_cast<uint16_t>(mode));
  }
  is_modes_ready_ = false;
  command_mode_status_.init(command_modes);
  system_request_.operation_mode = declare_parameter<uint16_t>("initial_operation_mode");
  system_request_.autoware_control = declare_parameter<bool>("initial_autoware_control");

  curr_autoware_control_ = false;
  curr_manual_control_ = false;
  curr_operation_mode_ = autoware::command_mode_types::modes::unknown;
  curr_mode_ = autoware::command_mode_types::modes::unknown;
  last_mode_ = system_request_.operation_mode;

  request_stamp_ = std::nullopt;
  transition_stamp_ = std::nullopt;

  using std::placeholders::_1;
  using std::placeholders::_2;

  // Interface with switcher nodes.
  pub_command_mode_request_ =
    create_publisher<CommandModeRequest>("~/command_mode/request", rclcpp::QoS(1));
  sub_command_mode_status_ = create_subscription<CommandModeStatus>(
    "~/command_mode/status", rclcpp::QoS(50).transient_local(),
    std::bind(&CommandModeDeciderBase::on_status, this, std::placeholders::_1));
  sub_availability_ = create_subscription<CommandModeAvailability>(
    "~/command_mode/availability", rclcpp::QoS(1),
    std::bind(&CommandModeDeciderBase::on_availability, this, std::placeholders::_1));
  sub_transition_available_ = create_subscription<ModeChangeAvailable>(
    "~/command_mode/transition/available", rclcpp::QoS(1),
    std::bind(&CommandModeDeciderBase::on_transition_available, this, std::placeholders::_1));
  sub_control_mode_ = create_subscription<ControlModeReport>(
    "~/control_mode/report", rclcpp::QoS(1),
    std::bind(&CommandModeDeciderBase::on_control_mode, this, std::placeholders::_1));

  // Interface for API.
  pub_operation_mode_ = create_publisher<OperationModeState>(
    "~/operation_mode/state", rclcpp::QoS(1).transient_local());
  srv_operation_mode_ = create_service<ChangeOperationMode>(
    "~/operation_mode/change_operation_mode",
    std::bind(&CommandModeDeciderBase::on_change_operation_mode, this, _1, _2));
  srv_autoware_control_ = create_service<ChangeAutowareControl>(
    "~/operation_mode/change_autoware_control",
    std::bind(&CommandModeDeciderBase::on_change_autoware_control, this, _1, _2));

  pub_mrm_state_ = create_publisher<MrmState>("~/mrm/state", rclcpp::QoS(1));
  pub_debug_ = create_publisher<Int32MultiArrayStamped>("~/debug", rclcpp::QoS(1));

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

bool CommandModeDeciderBase::is_in_transition() const
{
  if (system_request_.autoware_control) {
    return last_mode_ != system_request_.operation_mode;
  } else {
    return last_mode_ != autoware::command_mode_types::modes::manual;
  }
}

void CommandModeDeciderBase::on_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  std::vector<uint16_t> waiting_status;
  std::vector<uint16_t> waiting_available;
  std::vector<uint16_t> waiting_drivable;

  for (const auto & [mode, item] : command_mode_status_) {
    if (!item.status.stamp) {
      waiting_status.push_back(mode);
    }
    if (!item.available.stamp) {
      waiting_available.push_back(mode);
    }
    if (!item.drivable.stamp) {
      waiting_drivable.push_back(mode);
    }
  }

  // When OK.
  if (waiting_status.empty() && waiting_available.empty() && waiting_drivable.empty()) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
    return;
  }

  // When ERROR.
  std::string message = "Waiting for mode items:";
  for (const auto & mode : waiting_status) {
    message += " S" + std::to_string(mode);
  }
  for (const auto & mode : waiting_available) {
    message += " A" + std::to_string(mode);
  }
  for (const auto & mode : waiting_drivable) {
    message += " D" + std::to_string(mode);
  }
  status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
}

void CommandModeDeciderBase::on_timer()
{
  // TODO(Takagi, Isamu): Remove this process when all mode flags are supported.
  // Note: The is_modes_ready_ depends on this process.
  // Update transition availability other than autonomous mode.
  const auto stamp = now();
  for (const auto & [mode, item] : command_mode_status_) {
    if (mode != command_mode_types::modes::autonomous) {
      command_mode_status_.set(mode, true, stamp);
    }
  }

  update();
}

void CommandModeDeciderBase::on_control_mode(const ControlModeReport & msg)
{
  const auto is_changed = prev_control_mode_ != msg.mode;
  prev_control_mode_ = msg.mode;
  curr_autoware_control_ = (msg.mode == ControlModeReport::AUTONOMOUS);
  curr_manual_control_ = (msg.mode == ControlModeReport::MANUAL);

  // Check override.
  if (is_changed && curr_manual_control_) {
    if (system_request_.autoware_control) {
      // curr_mode_ and last_mode_ will be updated in the update_current_mode function.
      system_request_.autoware_control = false;
      RCLCPP_WARN_STREAM(get_logger(), "override detected");
    }
  }

  // Update command mode status for manual mode.
  {
    StatusMessage item;
    item.mode = autoware::command_mode_types::modes::manual;
    item.mrm = StatusMessage::NORMAL;
    item.transition = false;
    item.request = curr_manual_control_;
    item.vehicle_selected = curr_manual_control_;
    item.network_selected = true;
    item.command_selected = true;
    item.command_exclusive = true;
    item.command_enabled = true;
    item.command_disabled = true;
    command_mode_status_.set(item, msg.stamp);
  }

  // Update command mode availability for manual mode.
  {
    AvailabilityMessage item;
    item.mode = autoware::command_mode_types::modes::manual;
    item.available = true;
    command_mode_status_.set(item, msg.stamp);
  }

  if (!is_changed) {
    return;
  }
  update();
}

void CommandModeDeciderBase::on_status(const CommandModeStatus & msg)
{
  bool is_changed = false;
  for (const auto & item : msg.items) {
    is_changed |= command_mode_status_.set(item, msg.stamp);
  }
  if (!is_changed) {
    return;
  }
  update();
}

void CommandModeDeciderBase::on_availability(const CommandModeAvailability & msg)
{
  bool is_changed = false;
  for (const auto & item : msg.items) {
    is_changed |= command_mode_status_.set(item, msg.stamp);
  }
  if (!is_changed) {
    return;
  }
  update();
}

void CommandModeDeciderBase::on_transition_available(const ModeChangeAvailable & msg)
{
  // TODO(Takagi, Isamu): Update all modes when all mode flags are supported.
  if (command_mode_status_.set(command_mode_types::modes::autonomous, msg.available, msg.stamp)) {
    update();
  }
}

void CommandModeDeciderBase::update()
{
  if (!is_modes_ready_) {
    is_modes_ready_ = command_mode_status_.ready();
    if (!is_modes_ready_) {
      return;
    }
  }
  command_mode_status_.check_timeout(now());

  detect_operation_mode_timeout();
  update_request_mode();
  update_current_mode();
  sync_command_mode();
  publish_operation_mode_state();
  publish_mrm_state();
  publish_decider_debug();
}

void CommandModeDeciderBase::detect_operation_mode_timeout()
{
  if (!is_in_transition()) {
    transition_stamp_ = std::nullopt;
    return;
  }
  if (!transition_stamp_) {
    transition_stamp_ = now();
  }

  const auto duration = (now() - *transition_stamp_).seconds();
  if (duration < transition_timeout_) {
    return;
  }

  // Rollback to the last mode.
  if (last_mode_ != autoware::command_mode_types::modes::manual) {
    system_request_.operation_mode = last_mode_;
    system_request_.autoware_control = true;
  } else {
    // No need to rollback the operation mode in the manual mode.
    system_request_.autoware_control = false;
  }

  RCLCPP_INFO_STREAM(get_logger(), "Mode transition is canceled due to timeout.");
}

void CommandModeDeciderBase::update_request_mode()
{
  // Decide command mode with system-dependent logic.
  const auto modes = plugin_->decide(system_request_, command_mode_status_);
  if (modes.size() == 0) {
    RCLCPP_WARN_STREAM(get_logger(), "No command mode is requested.");
  }
  if (request_modes_ != modes) {
    const auto prev = text(request_modes_);
    const auto next = text(modes);
    RCLCPP_INFO_STREAM(get_logger(), "Request mode updated: " << prev << " -> " << next);
  }
  request_modes_ = modes;
}

void CommandModeDeciderBase::update_current_mode()
{
  // Search current operation mode
  for (const auto & [mode, item] : command_mode_status_) {
    if (plugin_->to_operation_mode(mode) == OperationModeState::UNKNOWN) {
      continue;
    }
    if (item.status.data.is_network_ready()) {
      if (curr_operation_mode_ != mode) {
        curr_operation_mode_ = mode;
        RCLCPP_INFO_STREAM(get_logger(), "Operation mode changed: " << curr_operation_mode_);
      }
      break;
    }
  }

  // Search current command mode.
  std::optional<uint16_t> curr_mode;
  for (const auto & [mode, item] : command_mode_status_) {
    if (item.status.data.is_vehicle_ready()) {
      curr_mode = mode;
      break;
    }
  }
  if (curr_mode) {
    if (curr_mode_ != *curr_mode) {
      curr_mode_ = *curr_mode;
      RCLCPP_INFO_STREAM(get_logger(), "Curr mode changed: " << curr_mode_);
    }
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Curr mode is unknown.");
  }

  // Update last confirmed mode.
  {
    const auto mode = system_request_.autoware_control
                        ? system_request_.operation_mode
                        : autoware::command_mode_types::modes::manual;

    const auto status = command_mode_status_.get(mode);
    if (status.is_completed()) {
      if (last_mode_ != mode) {
        last_mode_ = mode;
        RCLCPP_INFO_STREAM(get_logger(), "Last mode changed: " << last_mode_);
      }
    }
  }
}

void CommandModeDeciderBase::sync_command_mode()
{
  // Skip the request if all modes are already requested.
  std::vector<uint16_t> requesting_modes;
  for (const auto & mode : request_modes_) {
    if (!command_mode_status_.get(mode).request()) {
      requesting_modes.push_back(mode);
    }
  }
  if (requesting_modes.empty()) {
    request_stamp_ = std::nullopt;
    return;
  }

  // Skip the request if mode is now requesting.
  if (request_stamp_) {
    const auto duration = (now() - *request_stamp_).seconds();
    if (duration < request_timeout_) {
      return;
    }
    request_stamp_ = std::nullopt;

    std::string message = "Request mode timeout:";
    for (const auto & mode : requesting_modes) {
      message += " " + std::to_string(mode);
    }
    RCLCPP_WARN_STREAM(get_logger(), message);
  }

  // Request stamp is used to check timeout and requesting.
  const auto stamp = now();
  request_stamp_ = stamp;

  // Request command mode to switcher nodes.
  CommandModeRequest msg;
  msg.stamp = stamp;
  msg.vehicle = CommandModeRequest::NONE;
  for (const auto & mode : request_modes_) {
    CommandModeRequestItem item;
    item.mode = mode;
    msg.items.push_back(item);
  }
  pub_command_mode_request_->publish(msg);
}

void CommandModeDeciderBase::publish_operation_mode_state()
{
  const auto is_transition_available = [this](const auto & mode) {
    return command_mode_status_.available(mode, curr_manual_control_);
  };
  namespace modes = autoware::command_mode_types::modes;
  OperationModeState state;
  state.stamp = now();
  state.mode = plugin_->to_operation_mode(curr_operation_mode_);
  state.is_autoware_control_enabled = curr_autoware_control_;
  state.is_in_transition = is_in_transition();
  state.is_stop_mode_available = is_transition_available(modes::stop);
  state.is_autonomous_mode_available = is_transition_available(modes::autonomous);
  state.is_local_mode_available = is_transition_available(modes::local);
  state.is_remote_mode_available = is_transition_available(modes::remote);
  pub_operation_mode_->publish(state);
}

void CommandModeDeciderBase::publish_mrm_state()
{
  const auto convert = [](const uint16_t state) {
    // clang-format off
    switch (state) {
      case StatusMessage::NORMAL:    return MrmState::NORMAL;
      case StatusMessage::OPERATING: return MrmState::MRM_OPERATING;
      case StatusMessage::SUCCEEDED: return MrmState::MRM_SUCCEEDED;
      case StatusMessage::FAILED:    return MrmState::MRM_FAILED;
      default:                       return MrmState::UNKNOWN;
    }
    // clang-format on
  };
  const auto status = command_mode_status_.get(curr_mode_);
  MrmState state;
  state.stamp = now();
  state.state = convert(status.mrm());
  state.behavior = plugin_->to_mrm_behavior(status.mode());
  pub_mrm_state_->publish(state);
}

void CommandModeDeciderBase::publish_decider_debug()
{
  Int32MultiArrayStamped msg;
  msg.stamp = now();
  msg.data.push_back(curr_mode_);
  msg.data.push_back(curr_operation_mode_);
  msg.data.push_back(last_mode_);
  msg.data.push_back(system_request_.autoware_control);
  msg.data.push_back(system_request_.operation_mode);
  for (const auto mode : request_modes_) {
    msg.data.push_back(mode);
  }
  pub_debug_->publish(msg);
}

ResponseStatus make_response(bool success, const std::string & message = "")
{
  ResponseStatus res;
  res.success = success;
  res.message = message;
  return res;
};

ResponseStatus CommandModeDeciderBase::check_mode_exists(uint16_t mode)
{
  if (!is_modes_ready_) {
    return make_response(false, "Mode management is not ready.");
  }
  if (command_mode_status_.get(mode).mode() == autoware::command_mode_types::modes::unknown) {
    return make_response(false, "Invalid mode: " + std::to_string(mode));
  }
  return make_response(true);
}

ResponseStatus CommandModeDeciderBase::check_mode_request(uint16_t mode)
{
  const auto result = check_mode_exists(mode);
  if (!result.success) {
    return result;
  }
  const auto available = command_mode_status_.available(mode, curr_manual_control_);
  if (!available) {
    return make_response(false, "Mode is not available: " + std::to_string(mode));
  }
  return make_response(true);
}

void CommandModeDeciderBase::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  const auto mode = plugin_->from_operation_mode(req->mode);
  res->status = check_mode_request(mode);
  if (!res->status.success) {
    RCLCPP_WARN_STREAM(get_logger(), res->status.message);
    return;
  }
  system_request_.operation_mode = mode;

  RCLCPP_INFO_STREAM(get_logger(), "Change operation mode: " << mode);
  update();
}

void CommandModeDeciderBase::on_change_autoware_control(
  ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res)
{
  // For autoware_control is false.
  res->status = make_response(true);

  // Assume the driver is always ready.
  if (req->autoware_control) {
    res->status = check_mode_request(system_request_.operation_mode);
    if (!res->status.success) {
      RCLCPP_WARN_STREAM(get_logger(), res->status.message);
      return;
    }
  }
  system_request_.autoware_control = req->autoware_control;

  // Request vehicle mode to switcher nodes.
  CommandModeRequest msg;
  msg.stamp = now();
  msg.vehicle = req->autoware_control ? CommandModeRequest::AUTOWARE : CommandModeRequest::MANUAL;
  pub_command_mode_request_->publish(msg);

  RCLCPP_INFO_STREAM(get_logger(), "Change autoware control: " << req->autoware_control);
  update();
}

}  // namespace autoware::command_mode_decider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_decider::CommandModeDeciderBase)
