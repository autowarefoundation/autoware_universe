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

#ifndef AUTOWARE__PLANNING_LATENCY_VALIDATOR__LATENCY_VALIDATOR_HPP_
#define AUTOWARE__PLANNING_LATENCY_VALIDATOR__LATENCY_VALIDATOR_HPP_

#include <autoware/planning_validator/plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::planning_validator
{

class LatencyValidator : public PluginInterface
{
public:
  void init(rclcpp::Node & node, const std::string & name) override;
  void validate(
    const std::shared_ptr<const PlanningValidatorData> & data,
    const std::shared_ptr<PlanningValidatorStatus> & status, bool & is_critical) override;
  std::string get_module_name() const override { return module_name_; };

private:
  std::string module_name_;
  rclcpp::Clock::SharedPtr clock_{};

  bool enable_latency_check_;
  bool is_critical_check_;
  double latency_threshold_;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_LATENCY_VALIDATOR__LATENCY_VALIDATOR_HPP_
