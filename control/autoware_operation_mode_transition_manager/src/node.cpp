// Copyright 2022 Autoware Foundation
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

#include "node.hpp"

#include <autoware/component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>

namespace autoware::operation_mode_transition_manager
{

OperationModeTransitionManager::OperationModeTransitionManager(const rclcpp::NodeOptions & options)
: Node("autoware_operation_mode_transition_manager", options), compatibility_(this)
{
  cli_control_mode_ = create_client<ControlModeCommand>("control_mode_request");
  pub_debug_info_ = create_publisher<ModeChangeBase::DebugInfo>("~/debug_info", 1);

  // component interface
  {
    const auto node = autoware::component_interface_utils::NodeAdaptor(this);
    node.init_srv(
      srv_autoware_control_, this, &OperationModeTransitionManager::onChangeAutowareControl);
    node.init_srv(
      srv_operation_mode_, this, &OperationModeTransitionManager::onChangeOperationMode);
    node.init_pub(pub_operation_mode_);
  }

  // timer
  {
    const auto period_ns = rclcpp::Rate(declare_parameter<double>("frequency_hz")).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&OperationModeTransitionManager::onTimer, this));
  }

  // initialize state
  current_mode_ = OperationMode::STOP;
  transition_ = nullptr;
  control_mode_report_.mode = ControlModeReport::NO_COMMAND;
  transition_timeout_ = declare_parameter<double>("transition_timeout");
  {
    // check `transition_timeout` value
    const auto stable_duration = declare_parameter<double>("stable_check.duration");
    const double TIMEOUT_MARGIN = 0.5;
    if (transition_timeout_ < stable_duration + TIMEOUT_MARGIN) {
      transition_timeout_ = stable_duration + TIMEOUT_MARGIN;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "`transition_timeout` must be somewhat larger than `stable_check.duration`");
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 3000, "transition_timeout is set to " << transition_timeout_);
    }
  }
  input_timeout_ = declare_parameter<double>("input_timeout");

  // modes
  modes_[OperationMode::STOP] = std::make_unique<StopMode>();
  modes_[OperationMode::AUTONOMOUS] = std::make_unique<AutonomousMode>(this);
  modes_[OperationMode::LOCAL] = std::make_unique<LocalMode>();
  modes_[OperationMode::REMOTE] = std::make_unique<RemoteMode>();
}

void OperationModeTransitionManager::onChangeAutowareControl(
  const ChangeAutowareControlAPI::Service::Request::SharedPtr request,
  const ChangeAutowareControlAPI::Service::Response::SharedPtr response)
{
  if (request->autoware_control) {
    // Treat as a transition to the current operation mode.
    changeOperationMode(std::nullopt);
  } else {
    // Allow mode transition to complete without canceling.
    compatibility_transition_ = std::nullopt;
    transition_.reset();
    changeControlMode(ControlModeCommand::Request::MANUAL);
  }
  response->status.success = true;
}

void OperationModeTransitionManager::onChangeOperationMode(
  const ChangeOperationModeAPI::Service::Request::SharedPtr request,
  const ChangeOperationModeAPI::Service::Response::SharedPtr response)
{
  const auto mode = toEnum(*request);
  if (!mode) {
    throw autoware::component_interface_utils::ParameterError("The operation mode is invalid.");
  }
  changeOperationMode(mode.value());
  response->status.success = true;
}

void OperationModeTransitionManager::changeControlMode(ControlModeCommandType mode)
{
  const auto callback = [this](rclcpp::Client<ControlModeCommand>::SharedFuture future) {
    if (!future.get()->success) {
      RCLCPP_WARN(get_logger(), "Autonomous mode change was rejected.");
      if (transition_) {
        transition_->is_engage_failed = true;
      }
    }
  };

  const auto request = std::make_shared<ControlModeCommand::Request>();
  request->stamp = now();
  request->mode = mode;
  cli_control_mode_->async_send_request(request, callback);
}

void OperationModeTransitionManager::changeOperationMode(std::optional<OperationMode> request_mode)
{
  // NOTE: If request_mode is nullopt, indicates to enable autoware control

  const bool current_control = control_mode_report_.mode == ControlModeReport::AUTONOMOUS;
  const bool request_control = request_mode ? false : true;

  if (current_mode_ == request_mode) {
    throw autoware::component_interface_utils::NoEffectWarning(
      "The mode is the same as the current.");
  }

  if (current_control && request_control) {
    throw autoware::component_interface_utils::NoEffectWarning(
      "Autoware already controls the vehicle.");
  }

  // TODO(Takagi, Isamu): Consider mode change request during transition.
  if (transition_ && request_mode != OperationMode::STOP) {
    throw autoware::component_interface_utils::ServiceException(
      ServiceResponse::ERROR_IN_TRANSITION, "The mode transition is in progress.");
  }

  // Enter transition mode if the vehicle is being or will be controlled by Autoware.
  if (current_control || request_control) {
    if (!available_mode_change_[request_mode.value_or(current_mode_)]) {
      throw autoware::component_interface_utils::ServiceException(
        ServiceResponse::ERROR_NOT_AVAILABLE, "The mode change condition is not satisfied.");
    }
    if (request_control) {
      transition_ = std::make_unique<Transition>(now(), request_control, std::nullopt);
    } else {
      transition_ = std::make_unique<Transition>(now(), request_control, current_mode_);
    }
  }
  compatibility_transition_ = now();
  current_mode_ = request_mode.value_or(current_mode_);
}

void OperationModeTransitionManager::cancelTransition()
{
  const auto & previous = transition_->previous;
  if (previous) {
    compatibility_transition_ = now();
    current_mode_ = previous.value();
  } else {
    compatibility_transition_ = std::nullopt;
    changeControlMode(ControlModeCommand::Request::MANUAL);
  }
  transition_.reset();
}

void OperationModeTransitionManager::processTransition(const InputData & input_data)
{
  const bool current_control = control_mode_report_.mode == ControlModeReport::AUTONOMOUS;

  // Check timeout.
  if (transition_timeout_ < (now() - transition_->time).seconds()) {
    return cancelTransition();
  }

  // Check engage failure.
  if (transition_->is_engage_failed) {
    return cancelTransition();
  }

  // Check override.
  if (current_control) {
    transition_->is_engage_completed = true;
  } else {
    if (transition_->is_engage_completed) {
      return cancelTransition();
    }
  }

  // Check reflection of mode change to the compatible interface.
  if (current_mode_ != compatibility_.get_mode()) {
    return;
  }

  // Check completion when engaged, otherwise engage after the gate reflects transition.
  if (current_control) {
    if (modes_.at(current_mode_)->isModeChangeCompleted(input_data)) {
      return transition_.reset();
    }
  } else {
    if (transition_->is_engage_requested && input_data.gate_operation_mode.is_in_transition) {
      transition_->is_engage_requested = false;
      return changeControlMode(ControlModeCommand::Request::AUTONOMOUS);
    }
  }
}

void OperationModeTransitionManager::onTimer()
{
  const auto input_data = subscribeData();

  for (const auto & [type, mode] : modes_) {
    mode->update(current_mode_ == type && transition_);
  }

  for (const auto & [type, mode] : modes_) {
    available_mode_change_[type] = mode->isModeChangeAvailable(input_data);
  }

  // Check sync timeout to the compatible interface.
  if (compatibility_transition_) {
    if (compatibility_timeout_ < (now() - compatibility_transition_.value()).seconds()) {
      compatibility_transition_ = std::nullopt;
    }
  }

  // Reflects the mode when changed by the compatible interface.
  if (compatibility_transition_) {
    compatibility_.set_mode(current_mode_);
  } else {
    current_mode_ = compatibility_.get_mode().value_or(current_mode_);
  }

  // Reset sync timeout when it is completed.
  if (compatibility_transition_) {
    if (current_mode_ == compatibility_.get_mode()) {
      compatibility_transition_ = std::nullopt;
    }
  }

  if (transition_) {
    processTransition(input_data);
  }

  publishData();
}

InputData OperationModeTransitionManager::subscribeData()
{
  InputData input_data;

  const auto kinematics_ptr = sub_kinematics_.take_data();
  if (kinematics_ptr) {
    if (input_timeout_ < (now() - kinematics_ptr->header.stamp).seconds()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Subscribed kinematics is timed out.");
    } else {
      input_data.kinematics = *kinematics_ptr;
    }
  }

  const auto trajectory_ptr = sub_trajectory_.take_data();
  if (trajectory_ptr) {
    if (input_timeout_ < (now() - trajectory_ptr->header.stamp).seconds()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Subscribed trajectory is timed out.");
    } else {
      input_data.trajectory = *trajectory_ptr;
    }
  }

  const auto trajectory_follower_control_cmd_ptr = sub_trajectory_follower_control_cmd_.take_data();
  if (trajectory_follower_control_cmd_ptr) {
    if (input_timeout_ < (now() - trajectory_follower_control_cmd_ptr->stamp).seconds()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Subscribed trajectory_follower_control_cmd is timed out.");
    } else {
      input_data.trajectory_follower_control_cmd = *trajectory_follower_control_cmd_ptr;
    }
  }

  const auto control_cmd_ptr = sub_control_cmd_.take_data();
  if (control_cmd_ptr) {
    if (input_timeout_ < (now() - control_cmd_ptr->stamp).seconds()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Subscribed control_cmd is timed out.");
    } else {
      input_data.control_cmd = *control_cmd_ptr;
    }
  }

  // NOTE: Do not check the timeout of gate_operation_mode since the timestamp of this node's output
  // is used in the vehicle_cmd_gate node, which is updated only when the state changes.
  const auto gate_operation_mode_ptr = sub_gate_operation_mode_.take_data();
  if (gate_operation_mode_ptr) {
    input_data.gate_operation_mode = *gate_operation_mode_ptr;
  } else {
    // NOTE: initial state when the data is not subscribed yet.
    input_data.gate_operation_mode = OperationModeState{};
    input_data.gate_operation_mode.mode = OperationModeState::UNKNOWN;
    input_data.gate_operation_mode.is_in_transition = false;
  }

  const auto control_mode_report_ptr = sub_control_mode_report_.take_data();
  if (control_mode_report_ptr) {
    // NOTE: This will be used outside the onTimer function. Therefore, it
    // has to be a member variable
    control_mode_report_ = *control_mode_report_ptr;
  }

  return input_data;
}

void OperationModeTransitionManager::publishData()
{
  const bool current_control = control_mode_report_.mode == ControlModeReport::AUTONOMOUS;
  const auto time = now();

  OperationModeStateAPI::Message state;
  state.mode = toMsg(current_mode_);
  state.is_autoware_control_enabled = current_control;
  state.is_in_transition = transition_ ? true : false;
  state.is_stop_mode_available = available_mode_change_[OperationMode::STOP];
  state.is_autonomous_mode_available = available_mode_change_[OperationMode::AUTONOMOUS];
  state.is_local_mode_available = available_mode_change_[OperationMode::LOCAL];
  state.is_remote_mode_available = available_mode_change_[OperationMode::REMOTE];

  if (prev_state_ != state) {
    prev_state_ = state;
    state.stamp = time;
    pub_operation_mode_->publish(state);
  }

  const auto status_str = [&]() {
    if (!current_control) return "DISENGAGE (autoware mode = " + toString(current_mode_) + ")";
    if (transition_)
      return toString(current_mode_) + " (in transition from " + toString(transition_->previous) +
             ")";
    return toString(current_mode_);
  }();

  ModeChangeBase::DebugInfo debug_info = modes_.at(OperationMode::AUTONOMOUS)->getDebugInfo();
  debug_info.stamp = time;
  debug_info.status = status_str;
  debug_info.in_autoware_control = current_control;
  debug_info.in_transition = transition_ ? true : false;
  pub_debug_info_->publish(debug_info);
}

}  // namespace autoware::operation_mode_transition_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::operation_mode_transition_manager::OperationModeTransitionManager)
