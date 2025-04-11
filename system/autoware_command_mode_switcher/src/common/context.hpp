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

#ifndef COMMON__CONTEXT_HPP_
#define COMMON__CONTEXT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/srv/select_command_source.hpp>

#include <string>
#include <vector>

namespace autoware::command_mode_switcher
{

class SwitcherContext
{
public:
  using SelectCommandSource = tier4_system_msgs::srv::SelectCommandSource;

  explicit SwitcherContext(rclcpp::Node & node);
  bool select_source(const std::string & source);

private:
  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<SelectCommandSource>::SharedPtr cli_select_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__CONTEXT_HPP_
