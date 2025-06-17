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

#ifndef COMMAND__FILTER_HPP_
#define COMMAND__FILTER_HPP_

#include "common/control_command_filter.hpp"
#include "common/vehicle_status.hpp"
#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::control_command_gate
{

class CommandFilter : public CommandBridge
{
public:
  explicit CommandFilter(std::unique_ptr<CommandOutput> && output, rclcpp::Node & node);
  void set_nominal_filter_params(const VehicleCmdFilterParam & p);
  void set_transition_filter_params(const VehicleCmdFilterParam & p);
  void set_transition_flag(bool flag);
  bool get_transition_flag() const { return transition_flag_; }

  void on_control(const Control & msg) override;

private:
  double get_delta_time();
  Control filter_command(const Control & msg);

  rclcpp::Node & node_;
  VehicleCmdFilter nominal_filter_;
  VehicleCmdFilter transition_filter_;
  VehicleStatus vehicle_status_;
  bool enable_command_limit_filter_;
  bool transition_flag_;
  std::optional<rclcpp::Time> prev_time_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__FILTER_HPP_
