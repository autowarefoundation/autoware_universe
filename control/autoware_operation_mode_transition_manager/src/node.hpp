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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "compatibility.hpp"
#include "state.hpp"

#include <autoware/component_interface_specs_universe/system.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <unordered_map>

namespace autoware::operation_mode_transition_manager
{

class OperationModeTransitionManager : public rclcpp::Node
{
public:
  explicit OperationModeTransitionManager(const rclcpp::NodeOptions & options);

private:
  using ChangeAutowareControlAPI =
    autoware::component_interface_specs_universe::system::ChangeAutowareControl;
  using ChangeOperationModeAPI =
    autoware::component_interface_specs_universe::system::ChangeOperationMode;
  using OperationModeStateAPI =
    autoware::component_interface_specs_universe::system::OperationModeState;
  autoware::component_interface_utils::Service<ChangeAutowareControlAPI>::SharedPtr
    srv_autoware_control_;
  autoware::component_interface_utils::Service<ChangeOperationModeAPI>::SharedPtr
    srv_operation_mode_;
  autoware::component_interface_utils::Publisher<OperationModeStateAPI>::SharedPtr
    pub_operation_mode_;
  void onChangeAutowareControl(
    const ChangeAutowareControlAPI::Service::Request::SharedPtr request,
    const ChangeAutowareControlAPI::Service::Response::SharedPtr response);
  void onChangeOperationMode(
    const ChangeOperationModeAPI::Service::Request::SharedPtr request,
    const ChangeOperationModeAPI::Service::Response::SharedPtr response);

  using ControlModeCommandType = ControlModeCommand::Request::_mode_type;
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_kinematics_{this, "kinematics"};
  autoware_utils::InterProcessPollingSubscriber<Trajectory> sub_trajectory_{this, "trajectory"};
  autoware_utils::InterProcessPollingSubscriber<Control> sub_trajectory_follower_control_cmd_{
    this, "trajectory_follower_control_cmd"};
  autoware_utils::InterProcessPollingSubscriber<Control> sub_control_cmd_{this, "control_cmd"};
  autoware_utils::InterProcessPollingSubscriber<OperationModeState> sub_gate_operation_mode_{
    this, "gate_operation_mode"};
  autoware_utils::InterProcessPollingSubscriber<ControlModeReport> sub_control_mode_report_{
    this, "control_mode_report"};

  rclcpp::Client<ControlModeCommand>::SharedPtr cli_control_mode_;
  rclcpp::Publisher<ModeChangeBase::DebugInfo>::SharedPtr pub_debug_info_;
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();
  InputData subscribeData();
  void publishData();
  void changeControlMode(ControlModeCommandType mode);
  void changeOperationMode(std::optional<OperationMode> request_mode);
  void cancelTransition();
  void processTransition(const InputData & input_data);

  double transition_timeout_;
  double input_timeout_;
  OperationMode current_mode_;
  std::unique_ptr<Transition> transition_;
  std::unordered_map<OperationMode, std::unique_ptr<ModeChangeBase>> modes_;
  std::unordered_map<OperationMode, bool> available_mode_change_;
  ControlModeReport control_mode_report_;

  std::optional<OperationModeStateAPI::Message> prev_state_;

  static constexpr double compatibility_timeout_ = 1.0;
  Compatibility compatibility_;
  std::optional<rclcpp::Time> compatibility_transition_;
};

}  // namespace autoware::operation_mode_transition_manager

#endif  // NODE_HPP_
