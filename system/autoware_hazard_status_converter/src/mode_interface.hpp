// Copyright 2025 The Autoware Contributors
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

#ifndef MODE_INTERFACE_HPP_
#define MODE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>

#include <optional>

namespace autoware::hazard_status_converter
{

struct RootModeStatus
{
  enum class Mode {
    UNKNOWN,
    AUTO,
    REMOTE,
    LOCAL,
  };
  Mode mode;
  bool ignore;
};

class ModeInterface
{
public:
  explicit ModeInterface(rclcpp::Node & node);
  RootModeStatus get_root() const;

private:
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using AutowareState = autoware_system_msgs::msg::AutowareState;
  rclcpp::Node & node_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;
  rclcpp::Subscription<AutowareState>::SharedPtr sub_autoware_state_;

  std::optional<OperationModeState> operation_mode_state_;
  std::optional<AutowareState> autoware_state_;
};

}  // namespace autoware::hazard_status_converter

#endif  // MODE_INTERFACE_HPP_
