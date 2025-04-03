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

#ifndef COMMAND_MODE_DECIDER_HPP_
#define COMMAND_MODE_DECIDER_HPP_

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tier4_system_msgs/msg/command_mode_status.hpp>
#include <tier4_system_msgs/srv/change_operation_mode.hpp>
#include <tier4_system_msgs/srv/request_mrm.hpp>

#include <string>
#include <unordered_map>

namespace autoware::command_mode_decider
{

using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_system_msgs::msg::CommandModeAvailability;
using tier4_system_msgs::msg::CommandModeAvailabilityItem;
using tier4_system_msgs::msg::CommandModeRequest;
using tier4_system_msgs::msg::CommandModeStatus;
using tier4_system_msgs::msg::CommandModeStatusItem;
using tier4_system_msgs::srv::ChangeOperationMode;
using tier4_system_msgs::srv::RequestMrm;

struct CommandModeItem
{
  CommandModeAvailabilityItem availability;
  CommandModeStatusItem status;
};

class CommandModeDeciderBase : public rclcpp::Node
{
public:
  explicit CommandModeDeciderBase(const rclcpp::NodeOptions & options);

protected:
  virtual std::string decide_command_mode() = 0;
  const auto & get_command_modes() const { return command_modes_; }
  const auto & get_target_operation_mode() const { return target_operation_mode_; }
  const auto & get_target_mrm() const { return target_mrm_; }

private:
  void update_command_mode();
  void on_timer();
  void on_availability(const CommandModeAvailability & msg);
  void on_status(const CommandModeStatus & msg);
  void on_change_operation_mode(
    ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res);
  void on_request_mrm(RequestMrm::Request::SharedPtr req, RequestMrm::Response::SharedPtr res);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_operation_mode_;
  rclcpp::Service<RequestMrm>::SharedPtr srv_request_mrm_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<CommandModeRequest>::SharedPtr pub_command_mode_request_;
  rclcpp::Subscription<CommandModeAvailability>::SharedPtr sub_command_mode_availability_;
  rclcpp::Subscription<CommandModeStatus>::SharedPtr sub_command_mode_status_;

  bool is_modes_ready_;
  std::string target_operation_mode_;
  std::string target_mrm_;
  std::string curr_command_mode_;
  std::unordered_map<std::string, CommandModeItem> command_modes_;
  std::optional<rclcpp::Time> command_mode_request_stamp_;
};

class CommandModeDecider : public CommandModeDeciderBase
{
public:
  explicit CommandModeDecider(const rclcpp::NodeOptions & options);

protected:
  std::string decide_command_mode() override;

private:
  bool use_pull_over_;
  bool use_comfortable_stop_;
};

}  // namespace autoware::command_mode_decider

#endif  // COMMAND_MODE_DECIDER_HPP_
