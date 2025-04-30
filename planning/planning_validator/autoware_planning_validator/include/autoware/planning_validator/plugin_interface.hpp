// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__PLUGIN_INTERFACE_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__PLUGIN_INTERFACE_HPP_

#include "autoware/planning_validator/planning_validator_data.hpp"
#include "autoware_planning_validator/msg/planning_validator_status.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
using autoware_planning_validator::msg::PlanningValidatorStatus;

class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void init(rclcpp::Node & node, const std::string & name) = 0;
  virtual void validate(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status, bool & is_critical) = 0;
  virtual std::string get_module_name() const = 0;
  rclcpp::Logger logger_ = rclcpp::get_logger("");
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__PLUGIN_INTERFACE_HPP_
