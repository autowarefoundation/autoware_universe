//  Copyright 2026 The Autoware Contributors
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
#include "driving_mode_subsystem_adapter.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <redundancy_switcher_interface/detail/overloaded.hpp>

#include <autoware_common_msgs/msg/response_status.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::redundancy_switcher
{

void DrivingModeSubSystemAdapter::initialize(
  rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) throw std::invalid_argument("DrivingModeSubSystemAdapter: node is null");
  if (!gateway) throw std::invalid_argument("DrivingModeSubSystemAdapter: gateway is null");

  node_ = node;
  gateway_ = gateway;

  const auto qos = rclcpp::QoS(1);

  sub_velocity_report_ = node_->create_subscription<VelocityReport>(
    "~/input/velocity", qos,
    std::bind(&DrivingModeSubSystemAdapter::on_velocity_report, this, std::placeholders::_1));

  sub_control_mode_ = node_->create_subscription<ControlModeReport>(
    "~/input/control_mode", qos,
    std::bind(&DrivingModeSubSystemAdapter::on_control_mode_report, this, std::placeholders::_1));

  sub_driving_mode_request_ = node_->create_subscription<DrivingModeRequest>(
    "~/input/driving_mode_request", qos,
    std::bind(&DrivingModeSubSystemAdapter::on_driving_mode_request, this, std::placeholders::_1));

  srv_set_initializing_ = node_->create_service<SetBool>(
    "~/set_initializing", std::bind(
                            &DrivingModeSubSystemAdapter::on_set_initializing, this,
                            std::placeholders::_1, std::placeholders::_2));

  srv_reset_ = node_->create_service<ResetRedundancySwitcher>(
    "~/service/reset", std::bind(
                         &DrivingModeSubSystemAdapter::on_reset_request, this,
                         std::placeholders::_1, std::placeholders::_2));

  pub_active_control_unit_ = node_->create_publisher<ActiveControlUnitMsg>(
    "~/output/active_control_unit", rclcpp::QoS(1).transient_local());
}

void DrivingModeSubSystemAdapter::submit_event(const InputEvent & event)
{
  gateway_->submit(event);
}

void DrivingModeSubSystemAdapter::on_driving_mode_request(
  const DrivingModeRequest::ConstSharedPtr msg)
{
  submit_event(
    InputEvent{SetPriorityEvent{
      Annotated<uint16_t>{static_cast<uint16_t>(msg->priority), "driving_mode_request"}}});
}

void DrivingModeSubSystemAdapter::on_set_initializing(
  const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response)
{
  const auto ready = request->data ? AutowareReady::False : AutowareReady::True;
  submit_event(InputEvent{SetAutowareReadyEvent{Annotated<AutowareReady>{ready, "service call"}}});

  response->success = true;
  response->message = "Set initializing: " + std::string(request->data ? "true" : "false");
  RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
}

void DrivingModeSubSystemAdapter::on_reset_request(
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
            break;
          case ResetRejectedReason::Error:
            response->status.code = ResponseStatus::UNKNOWN;
            break;
        }
      }
      response->status.message = r->message;
      return;
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "Reset: no ResetResultCommand returned from processor.");
  response->status.success = false;
  response->status.code = ResponseStatus::UNKNOWN;
  response->status.message = "Internal error: no result from processor.";
}

void DrivingModeSubSystemAdapter::on_velocity_report(const VelocityReport::ConstSharedPtr msg)
{
  constexpr auto th_stopped_velocity = 0.001;
  const bool is_stopped = std::abs(msg->longitudinal_velocity) < th_stopped_velocity;
  submit_event(
    InputEvent{SetVelocityStatusEvent{Annotated<VelocityStatus>{
      is_stopped ? VelocityStatus::Stopped : VelocityStatus::Moving, "velocity report"}}});
}

void DrivingModeSubSystemAdapter::on_control_mode_report(
  const ControlModeReport::ConstSharedPtr msg)
{
  const auto mode =
    (msg->mode == ControlModeReport::AUTONOMOUS) ? ControlMode::Auto : ControlMode::Manual;
  submit_event(
    InputEvent{SetControlModeEvent{Annotated<ControlMode>{mode, "control mode report"}}});
}

void DrivingModeSubSystemAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const UpdateActiveControlUnitCommand & e) { send_active_control_unit(e); },
      [](const auto &) {}},
    command);
}

void DrivingModeSubSystemAdapter::send_active_control_unit(
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

PLUGINLIB_EXPORT_CLASS(
  autoware::redundancy_switcher::DrivingModeSubSystemAdapter,
  autoware::redundancy_switcher::IAdapterPlugin)
