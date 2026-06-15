// Copyright 2026 The Autoware Contributors
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

#include "ros_interface.hpp"

#include <memory>
#include <utility>

namespace autoware::driving_mode_manager
{

RosInterface::RosInterface(rclcpp::Node * node) : node_(node)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  cli_trajectory_source_ = node->create_client<TrajectorySourceSrv>("~/trajectory/source/change");
  cli_command_source_ = node->create_client<ChangeCommandSourceSrv>("~/command/source/change");
  cli_command_filter_ = node->create_client<ChangeCommandFilterSrv>("~/command/filter/change");
  cli_control_mode_command_ =
    node->create_client<ControlModeCommandSrv>("~/vehicle/control_mode/command");
  pub_operation_mode_ = node->create_publisher<OperationModeStateMsg>(
    "~/system/operation_mode_state", rclcpp::QoS(1).transient_local());
  pub_mrm_state_ =
    node->create_publisher<MrmStateMsg>("~/system/mrm_state", rclcpp::QoS(1).transient_local());

  sub_driving_mode_available_ = node->create_subscription<DrivingModeFlag>(
    "~/system/driving_mode/available", rclcpp::QoS(10),
    std::bind(&RosInterface::on_driving_mode_available, this, _1));
  sub_driving_mode_stable_ = node->create_subscription<DrivingModeFlag>(
    "~/system/driving_mode/stable", rclcpp::QoS(10),
    std::bind(&RosInterface::on_driving_mode_stable, this, _1));
  sub_driving_mode_continuable_ = node->create_subscription<DrivingModeFlag>(
    "~/system/driving_mode/continuable", rclcpp::QoS(10),
    std::bind(&RosInterface::on_driving_mode_continuable, this, _1));
  sub_driving_mode_mrm_state_ = node->create_subscription<DrivingModeMrmState>(
    "~/system/driving_mode/mrm_state", rclcpp::QoS(10),
    std::bind(&RosInterface::on_driving_mode_mrm_state, this, _1));
  sub_trajectory_source_ = node->create_subscription<TrajectorySourceMsg>(
    "~/trajectory/source/status", rclcpp::QoS(1).transient_local(),
    std::bind(&RosInterface::on_trajectory_source, this, _1));
  sub_command_source_ = node->create_subscription<CommandSourceMsg>(
    "~/command/source/status", rclcpp::QoS(1).transient_local(),
    std::bind(&RosInterface::on_command_source, this, _1));
  sub_command_filter_ = node->create_subscription<CommandFilterMsg>(
    "~/command/filter/status", rclcpp::QoS(1).transient_local(),
    std::bind(&RosInterface::on_command_filter, this, _1));
  sub_control_mode_report_ = node->create_subscription<ControlModeReport>(
    "~/vehicle/control_mode/report", rclcpp::QoS(1).durability_volatile(),
    std::bind(&RosInterface::on_control_mode_report, this, _1));

  srv_operation_mode_ = node->create_service<ChangeOperationMode>(
    "~/system/change_operation_mode",
    std::bind(&RosInterface::on_change_operation_mode, this, _1, _2));
  srv_autoware_control_ = node->create_service<ChangeAutowareControl>(
    "~/system/change_autoware_control",
    std::bind(&RosInterface::on_change_autoware_control, this, _1, _2));
  srv_mrm_request_ = node->create_service<ChangeMrmRequest>(
    "~/system/change_mrm_request", std::bind(&RosInterface::on_change_mrm_request, this, _1, _2));

  pub_driving_mode_request_ =
    node->create_publisher<DrivingModeRequest>("~/system/driving_mode/request", rclcpp::QoS(1));
  pub_driving_mode_info_ = node->create_publisher<DrivingModeInfo>(
    "~/system/driving_mode/info", rclcpp::QoS(1).transient_local());

  pub_debug_status_ = node->create_publisher<DebugModeFlag>("~/debug/status", rclcpp::QoS(1));
  pub_debug_request_ = node->create_publisher<DebugModeRequest>("~/debug/request", rclcpp::QoS(1));
}

rclcpp::Time RosInterface::now() const
{
  return node_->now();
}

void RosInterface::change_trajectory_source(const TrajectorySource & source)
{
  const auto request = std::make_shared<TrajectorySourceSrv::Request>();
  request->source = source.id;
  cli_trajectory_source_->async_send_request(request);
}

void RosInterface::change_command_source(const CommandSource & source)
{
  const auto request = std::make_shared<ChangeCommandSourceSrv::Request>();
  request->source = source.id;
  cli_command_source_->async_send_request(request);
}

void RosInterface::change_command_filter(const CommandFilter & filter)
{
  const auto request = std::make_shared<ChangeCommandFilterSrv::Request>();
  request->filter = filter.flag;
  cli_command_filter_->async_send_request(request);
}

void RosInterface::change_platform_mode(const PlatformMode & mode)
{
  // clang-format off
  const auto convert = [](const PlatformMode & mode) -> std::optional<uint8_t> {
    using Command = ControlModeCommandSrv::Request;
    switch (mode) {
      case PlatformMode::kAutoware:         return Command::AUTONOMOUS;
      case PlatformMode::kAutowareSteering: return Command::AUTONOMOUS_STEER_ONLY;
      case PlatformMode::kAutowareVelocity: return Command::AUTONOMOUS_VELOCITY_ONLY;
      case PlatformMode::kManual:           return Command::MANUAL;
      default:                              return std::nullopt;
    }
  };
  // clang-format on

  const auto command = convert(mode);
  if (!command) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "unknown platform mode");
    return;
  }
  const auto request = std::make_shared<ControlModeCommandSrv::Request>();
  request->mode = command.value();
  cli_control_mode_command_->async_send_request(request);
}

void RosInterface::publish_operation_mode(const OperationModeState & state) const
{
  const auto convert = [](const OperationMode & mode) {
    // clang-format off
    switch (mode) {
      case OperationMode::kStop:       return OperationModeStateMsg::STOP;
      case OperationMode::kAutonomous: return OperationModeStateMsg::AUTONOMOUS;
      case OperationMode::kLocal:      return OperationModeStateMsg::LOCAL;
      case OperationMode::kRemote:     return OperationModeStateMsg::REMOTE;
      default:                         return OperationModeStateMsg::UNKNOWN;
    }
    // clang-format on
  };

  OperationModeStateMsg msg;
  msg.stamp = now();
  msg.mode = convert(state.mode);
  msg.is_autoware_control_enabled = state.is_autoware_control_enabled;
  msg.is_in_transition = state.is_in_transition;
  msg.is_stop_mode_available = state.is_stop_mode_available;
  msg.is_autonomous_mode_available = state.is_autonomous_mode_available;
  msg.is_local_mode_available = state.is_local_mode_available;
  msg.is_remote_mode_available = state.is_remote_mode_available;
  pub_operation_mode_->publish(msg);
}

void RosInterface::publish_mrm_state(const MrmState & state) const
{
  // clang-format off
  const auto convert = [](const MrmState::State & state) {
    switch (state) {
      case MrmState::State::kNormal:    return MrmStateMsg::NORMAL;
      case MrmState::State::kOperating: return MrmStateMsg::MRM_OPERATING;
      case MrmState::State::kSucceeded: return MrmStateMsg::MRM_SUCCEEDED;
      case MrmState::State::kFailed:    return MrmStateMsg::MRM_FAILED;
      case MrmState::State::kUnknown:   return MrmStateMsg::UNKNOWN;
      default:                          return MrmStateMsg::UNKNOWN;
    }
  };
  // clang-format on

  MrmStateMsg msg;
  msg.stamp = now();
  msg.state = convert(state.state);
  msg.behavior = state.behavior.id;
  pub_mrm_state_->publish(msg);
}

void RosInterface::publish_debug(const DebugStatus & status) const
{
  DebugModeFlag msg;
  msg.stamp = now();
  for (const auto & [mode, flag] : status.flags) {
    msg.mode.push_back(mode.id);
    msg.available.push_back(flag.available);
    msg.stable.push_back(flag.stable);
    msg.continuable.push_back(flag.continuable);
  }
  pub_debug_status_->publish(msg);
}

void RosInterface::publish_driving_mode_info(const ModeInfo & info) const
{
  DrivingModeInfo msg;
  msg.stamp = now();
  for (const auto & [mode, name] : info.names) {
    tier4_system_msgs::msg::DrivingModeInfoItem item;
    item.mode = mode.id;
    item.name = name;
    msg.items.push_back(item);
  }
  pub_driving_mode_info_->publish(msg);
}

void RosInterface::publish_debug(const RequestModes & request) const
{
  DebugModeRequest msg;
  msg.stamp = now();
  msg.operation_mode = request.operation_mode.id;
  msg.platform_mode = static_cast<std::underlying_type_t<PlatformMode>>(request.platform_mode);
  msg.mrm_strategy = static_cast<std::underlying_type_t<MrmStrategy>>(request.mrm_strategy);
  msg.mrm_behavior = request.mrm_behavior.id;
  msg.autoware_mode = request.autoware_mode.id;
  pub_debug_request_->publish(msg);
}

void RosInterface::on_driving_mode_available(const DrivingModeFlag & msg)
{
  for (const auto & item : msg.items) {
    logic_->on_available_flag(AutowareMode{item.mode}, item.flag);
  }
}

void RosInterface::on_driving_mode_stable(const DrivingModeFlag & msg)
{
  for (const auto & item : msg.items) {
    logic_->on_stable_flag(AutowareMode{item.mode}, item.flag);
  }
}

void RosInterface::on_driving_mode_continuable(const DrivingModeFlag & msg)
{
  for (const auto & item : msg.items) {
    logic_->on_continuable_flag(AutowareMode{item.mode}, item.flag);
  }
}

void RosInterface::on_driving_mode_mrm_state(const DrivingModeMrmState & msg)
{
  using Item = tier4_system_msgs::msg::DrivingModeMrmStateItem;

  // clang-format off
  const auto convert = [](const uint16_t & state) {
    switch (state) {
      case Item::NORMAL:    return MrmState::State::kNormal;
      case Item::OPERATING: return MrmState::State::kOperating;
      case Item::SUCCEEDED: return MrmState::State::kSucceeded;
      case Item::FAILED:    return MrmState::State::kFailed;
      default:              return MrmState::State::kUnknown;
    }
  };
  // clang-format on

  for (const auto & item : msg.items) {
    logic_->on_mrm_state(AutowareMode{item.mode}, convert(item.state));
  }
}

void RosInterface::publish_driving_mode_request(const AutowareMode & mode) const
{
  DrivingModeRequest msg;
  msg.stamp = now();
  msg.mode = mode.id;
  pub_driving_mode_request_->publish(msg);
}

void RosInterface::on_trajectory_source(const TrajectorySourceMsg & msg)
{
  logic_->on_trajectory_source(TrajectorySource{msg.source});
}

void RosInterface::on_command_source(const CommandSourceMsg & msg)
{
  logic_->on_command_source(CommandSource{msg.source});
}

void RosInterface::on_command_filter(const CommandFilterMsg & msg)
{
  logic_->on_command_filter(CommandFilter{msg.filter});
}

void RosInterface::on_control_mode_report(const ControlModeReport & msg)
{
  // clang-format off
  const auto convert = [](const ControlModeReport & msg) {
    switch (msg.mode) {
      case ControlModeReport::AUTONOMOUS:               return PlatformMode::kAutoware;
      case ControlModeReport::AUTONOMOUS_STEER_ONLY:    return PlatformMode::kAutowareSteering;
      case ControlModeReport::AUTONOMOUS_VELOCITY_ONLY: return PlatformMode::kAutowareVelocity;
      case ControlModeReport::MANUAL:                   return PlatformMode::kManual;
      default:                                          return PlatformMode::kUnknown;
    }
  };
  // clang-format on

  logic_->on_vehicle_control_mode(convert(msg));
}

void RosInterface::on_change_operation_mode(
  ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res)
{
  // clang-format off
  const auto convert = [](const ChangeOperationMode::Request & req) {
    switch (req.mode) {
      case ChangeOperationMode::Request::STOP:       return OperationMode::kStop;
      case ChangeOperationMode::Request::AUTONOMOUS: return OperationMode::kAutonomous;
      case ChangeOperationMode::Request::LOCAL:      return OperationMode::kLocal;
      case ChangeOperationMode::Request::REMOTE:     return OperationMode::kRemote;
      default:                                       return OperationMode::kUnknown;
    }
  };
  // clang-format on

  const auto status = logic_->change_operation_mode(convert(*req));
  res->status.success = status.success;
  res->status.message = status.message;
}

void RosInterface::on_change_autoware_control(
  ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res)
{
  const auto convert = [](const ChangeAutowareControl::Request & req) {
    return req.autoware_control ? AutowareControl::kEnable : AutowareControl::kDisable;
  };

  const auto status = logic_->change_autoware_control(convert(*req));
  res->status.success = status.success;
  res->status.message = status.message;
}

void RosInterface::on_change_mrm_request(
  ChangeMrmRequest::Request::SharedPtr req, ChangeMrmRequest::Response::SharedPtr res)
{
  const auto convert = [](const ChangeMrmRequest::Request & req) {
    // clang-format off
    switch (req.strategy) {
      case ChangeMrmRequest::Request::CANCEL:   return MrmStrategy::kNone;
      case ChangeMrmRequest::Request::DELEGATE: return MrmStrategy::kDelegate;
      case ChangeMrmRequest::Request::BEHAVIOR: return MrmStrategy::kBehavior;
      default:                                  return MrmStrategy::kUnknown;
    }
    // clang-format on
  };

  const auto status = logic_->change_mrm_request({convert(*req), {req->behavior}});
  res->status.success = status.success;
  res->status.message = status.message;
}

}  // namespace autoware::driving_mode_manager
