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

#ifndef COMMAND_MODE_DECIDER_BASE_HPP_
#define COMMAND_MODE_DECIDER_BASE_HPP_

#include "autoware_command_mode_decider/plugin.hpp"
#include "autoware_command_mode_decider/status.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_debug_msgs/msg/int32_multi_array_stamped.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tier4_system_msgs/msg/command_mode_status.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <memory>
#include <vector>

namespace autoware::command_mode_decider
{

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_common_msgs::msg::ResponseStatus;
using autoware_internal_debug_msgs::msg::Int32MultiArrayStamped;
using autoware_system_msgs::srv::ChangeAutowareControl;
using autoware_system_msgs::srv::ChangeOperationMode;
using autoware_vehicle_msgs::msg::ControlModeReport;
using tier4_system_msgs::msg::CommandModeAvailability;
using tier4_system_msgs::msg::CommandModeRequest;
using tier4_system_msgs::msg::CommandModeRequestItem;
using tier4_system_msgs::msg::CommandModeStatus;
using tier4_system_msgs::msg::ModeChangeAvailable;

class CommandModeDeciderBase : public rclcpp::Node
{
public:
  explicit CommandModeDeciderBase(const rclcpp::NodeOptions & options);

private:
  bool is_in_transition() const;
  void update();
  void detect_override();
  void detect_operation_mode_timeout();
  void update_request_mode();
  void update_current_mode();
  void sync_command_mode();
  void publish_operation_mode_state();
  void publish_mrm_state();
  void publish_decider_debug();

  void on_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void on_timer();
  void on_control_mode(const ControlModeReport & msg);
  void on_status(const CommandModeStatus & msg);
  void on_availability(const CommandModeAvailability & msg);
  void on_transition_available(const ModeChangeAvailable & msg);
  void on_change_operation_mode(
    ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res);
  void on_change_autoware_control(
    ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res);

  ResponseStatus check_mode_exists(uint16_t mode);
  ResponseStatus check_mode_request(uint16_t mode);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<CommandModeRequest>::SharedPtr pub_command_mode_request_;
  rclcpp::Subscription<CommandModeStatus>::SharedPtr sub_command_mode_status_;
  rclcpp::Subscription<CommandModeAvailability>::SharedPtr sub_availability_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_transition_available_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_;

  rclcpp::Service<ChangeAutowareControl>::SharedPtr srv_autoware_control_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_operation_mode_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<MrmState>::SharedPtr pub_mrm_state_;
  rclcpp::Publisher<Int32MultiArrayStamped>::SharedPtr pub_debug_;

  diagnostic_updater::Updater diagnostics_;
  pluginlib::ClassLoader<DeciderPlugin> loader_;
  std::shared_ptr<DeciderPlugin> plugin_;

  // parameters
  double transition_timeout_;
  double request_timeout_;

  // status
  bool is_modes_ready_;
  CommandModeStatusTable command_mode_status_;
  RequestModeStatus system_request_;
  std::vector<uint16_t> request_modes_;

  bool curr_autoware_control_;
  bool curr_manual_control_;
  std::optional<uint8_t> prev_control_mode_;

  uint16_t curr_operation_mode_;
  uint16_t curr_mode_;
  uint16_t last_mode_;

  std::optional<rclcpp::Time> request_stamp_;
  std::optional<rclcpp::Time> transition_stamp_;
};

}  // namespace autoware::command_mode_decider

#endif  // COMMAND_MODE_DECIDER_BASE_HPP_
