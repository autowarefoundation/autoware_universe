// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE_ENGAGE_HPP_
#define AUTOWARE_ENGAGE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <tier4_external_api_msgs/msg/engage_status.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

namespace autoware::evaluation_adapter
{

using EngageService = tier4_external_api_msgs::srv::Engage;
using EngageStatus = tier4_external_api_msgs::msg::EngageStatus;
using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;

class AutowareEngage : public rclcpp::Node
{
public:
  explicit AutowareEngage(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<EngageStatus>::SharedPtr pub_engage_;
  rclcpp::Service<EngageService>::SharedPtr srv_engage_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_stop_mode_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr cli_change_autonomous_mode_;

  void on_state(const OperationModeState & msg);
  void on_engage(
    const EngageService::Request::SharedPtr req, EngageService::Response::SharedPtr res);

  OperationModeState state_;
};

}  // namespace autoware::evaluation_adapter

#endif  // AUTOWARE_ENGAGE_HPP_
