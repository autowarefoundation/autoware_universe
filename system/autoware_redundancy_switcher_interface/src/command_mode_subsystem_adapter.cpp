//  Copyright 2025 The Autoware Contributors
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
#include "command_mode_subsystem_adapter.hpp"

#include <autoware_command_mode_types/modes.hpp>
#include <redundancy_switcher_interface/detail/overloaded.hpp>

#include <autoware_common_msgs/msg/response_status.hpp>

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace autoware::redundancy_switcher
{

void CommandModeSubSystemAdapter::initialize(
  rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) {
    throw std::invalid_argument("CommandModeSubSystemAdapter: node is null");
  }
  if (!gateway) {
    throw std::invalid_argument("CommandModeSubSystemAdapter: gateway is null");
  }

  node_ = node;
  gateway_ = gateway;

  is_main_ecu_ = node_->has_parameter("is_main_ecu")
                   ? node_->get_parameter("is_main_ecu").as_bool()
                   : node_->declare_parameter<bool>("is_main_ecu", true);
  availability_timeout_milli_ = node_->declare_parameter<double>("availability_timeout_milli");

  const auto qos = rclcpp::QoS(1);

  sub_velocity_report_ = node_->create_subscription<VelocityReport>(
    "~/input/velocity", qos,
    std::bind(
      &CommandModeSubSystemAdapter::on_velocity_report, this, std::placeholders::_1));

  sub_control_mode_ = node_->create_subscription<ControlModeReport>(
    "~/input/control_mode", qos,
    std::bind(
      &CommandModeSubSystemAdapter::on_control_mode_report, this, std::placeholders::_1));

  sub_command_mode_request_ = node_->create_subscription<CommandModeRequest>(
    "~/input/command_mode_request", qos,
    std::bind(
      &CommandModeSubSystemAdapter::on_command_mode_request, this, std::placeholders::_1));

  sub_command_mode_availability_ = node_->create_subscription<CommandModeAvailability>(
    "~/input/command_mode_availability", qos,
    std::bind(
      &CommandModeSubSystemAdapter::on_command_mode_availability, this, std::placeholders::_1));

  srv_set_initializing_ = node_->create_service<SetBool>(
    "~/set_initializing",
    std::bind(
      &CommandModeSubSystemAdapter::on_set_initializing, this, std::placeholders::_1,
      std::placeholders::_2));

  srv_reset_ = node_->create_service<ResetRedundancySwitcher>(
    "~/service/reset",
    std::bind(
      &CommandModeSubSystemAdapter::on_reset_request, this, std::placeholders::_1,
      std::placeholders::_2));

  // Publishers
  pub_active_control_unit_ = node_->create_publisher<ActiveControlUnitMsg>(
    "~/output/active_control_unit", rclcpp::QoS(1).transient_local());
}

void CommandModeSubSystemAdapter::submit_event(const InputEvent & event)
{
  gateway_->submit(event);
}

void CommandModeSubSystemAdapter::on_command_mode_request(
  const CommandModeRequest::ConstSharedPtr msg)
{
  namespace modes = autoware::command_mode_types::modes;

  if (!is_main_ecu_ || msg->items.empty()) return;

  std::optional<CommandModeRequest> prev;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    prev = std::exchange(last_command_mode_request_, *msg);
  }
  if (!prev.has_value() || prev->items == msg->items) return;

  const auto & mode = msg->items[0].mode;
  if (mode == modes::sub_ecu_standby || mode == modes::sub_ecu_in_lane_moderate_stop) {
    submit_event(
      InputEvent{SelfInterruptionEvent{
        Annotated<std::monostate>{{}, "requested mode: " + std::to_string(mode)}}});
  }
}

void CommandModeSubSystemAdapter::on_command_mode_availability(
  const CommandModeAvailability::ConstSharedPtr msg)
{
  check_sub_ecu_error(msg);
  check_availability_timeout(msg);
}

void CommandModeSubSystemAdapter::check_sub_ecu_error(
  const CommandModeAvailability::ConstSharedPtr msg)
{
  namespace modes = autoware::command_mode_types::modes;

  // Whether Autoware is in control is determined inside Processor::self_interruption().
  if (is_main_ecu_) return;

  for (const auto & item : msg->items) {
    if (item.mode == modes::sub_ecu_in_lane_moderate_stop && !item.available) {
      submit_event(
        InputEvent{SelfInterruptionEvent{
          Annotated<std::monostate>{{}, "mode unavailable: sub_ecu_in_lane_moderate_stop"}}});
      return;
    }
  }
}

void CommandModeSubSystemAdapter::check_availability_timeout(
  const CommandModeAvailability::ConstSharedPtr msg)
{
  namespace modes = autoware::command_mode_types::modes;
  const auto now = node_->now();

  const bool has_other_ecu_items =
    std::any_of(msg->items.begin(), msg->items.end(), [&](const auto & item) {
      if (is_main_ecu_) {
        return item.mode == modes::sub_ecu_in_lane_moderate_stop ||
               item.mode == modes::sub_ecu_standby;
      }
      return item.mode == modes::comfortable_stop ||
             item.mode == modes::main_ecu_in_lane_moderate_stop ||
             item.mode == modes::main_ecu_in_lane_emergency_stop;
    });

  bool prev_timeout = false;
  bool curr_timeout = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto prev_stamp = stamp_another_ecu_availability_;
    if (has_other_ecu_items) {
      stamp_another_ecu_availability_ = now;
    }

    prev_timeout = is_another_ecu_availability_timeout_;
    if (prev_stamp.has_value()) {
      const double elapsed_ms = (now - *prev_stamp).seconds() * 1000.0;
      is_another_ecu_availability_timeout_ = elapsed_ms > availability_timeout_milli_;
    }
    curr_timeout = is_another_ecu_availability_timeout_;
  }

  // Submit an event on both the rising edge (false → true) and falling edge (true → false).
  if (prev_timeout != curr_timeout) {
    const std::string annotation =
      curr_timeout ? "availability timeout exceeded" : "availability recovered";
    submit_event(
      InputEvent{SetAnotherEcuAvailabilityTimeoutEvent{Annotated<bool>{curr_timeout, annotation}}});
  }
}

void CommandModeSubSystemAdapter::on_set_initializing(
  const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response)
{
  const auto ready = request->data ? AutowareReady::False : AutowareReady::True;
  submit_event(InputEvent{SetAutowareReadyEvent{Annotated<AutowareReady>{ready, "service call"}}});

  // TODO(TetsuKawa): return an appropriate response based on the Processor state
  response->success = true;
  response->message = "Set initializing: " + std::string(request->data ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
}

void CommandModeSubSystemAdapter::on_reset_request(
  const ResetRedundancySwitcher::Request::SharedPtr request [[maybe_unused]],
  ResetRedundancySwitcher::Response::SharedPtr response)
{
  using ResponseStatus = autoware_common_msgs::msg::ResponseStatus;

  const auto commands =
    gateway_->submit_request(InputEvent{ResetEvent{Annotated<std::monostate>{{}, "service call"}}});

  for (const auto & cmd : commands) {
    if (const auto * r = std::get_if<ResetResultCommand>(&cmd)) {
      if (r->accepted || r->reason == ResetRejectedReason::NotNecessary) {
        response->status.success = true;
      } else {
        response->status.success = false;
        switch (r->reason) {
          case ResetRejectedReason::Ignored:
            response->status.code = ResponseStatus::NO_EFFECT;
            break;
          case ResetRejectedReason::NotNecessary:
            break;  // unreachable: handled above
          case ResetRejectedReason::Error:
            response->status.code = ResponseStatus::UNKNOWN;
            break;
        }
      }
      response->status.message = r->message;
      return;
    }
  }

  // No ResetResultCommand returned — this indicates a bug in the Processor implementation.
  RCLCPP_ERROR(node_->get_logger(), "Reset: no ResetResultCommand returned from processor.");
  response->status.success = false;
  response->status.code = ResponseStatus::UNKNOWN;
  response->status.message = "Internal error: no result from processor.";
}

void CommandModeSubSystemAdapter::on_velocity_report(const VelocityReport::ConstSharedPtr msg)
{
  constexpr auto th_stopped_velocity = 0.001;
  const bool is_stopped = std::abs(msg->longitudinal_velocity) < th_stopped_velocity;
  submit_event(
    InputEvent{SetVelocityStatusEvent{Annotated<VelocityStatus>{
      is_stopped ? VelocityStatus::Stopped : VelocityStatus::Moving, "velocity report"}}});
}

void CommandModeSubSystemAdapter::on_control_mode_report(
  const ControlModeReport::ConstSharedPtr msg)
{
  const auto mode =
    (msg->mode == ControlModeReport::AUTONOMOUS) ? ControlMode::Auto : ControlMode::Manual;
  submit_event(
    InputEvent{SetControlModeEvent{Annotated<ControlMode>{mode, "control mode report"}}});
}

void CommandModeSubSystemAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const UpdateActiveControlUnitCommand & e) { send_active_control_unit(e); },
      [](const auto &) { /* Not this adapter's responsibility — ignore. */ }},
    command);
}

void CommandModeSubSystemAdapter::send_active_control_unit(
  const UpdateActiveControlUnitCommand & command)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (
      last_active_control_unit_ids_.has_value() &&
      *last_active_control_unit_ids_ == command.value.unit_ids) {
      return;
    }
    last_active_control_unit_ids_ = command.value.unit_ids;
  }

  ActiveControlUnitMsg msg;
  msg.stamp = node_->now();
  msg.ids = command.value.unit_ids;
  pub_active_control_unit_->publish(msg);
}

}  // namespace autoware::redundancy_switcher
