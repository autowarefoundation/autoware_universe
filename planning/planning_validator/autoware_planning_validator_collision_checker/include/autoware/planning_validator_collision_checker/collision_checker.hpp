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

#ifndef AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__COLLISION_CHECKER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__COLLISION_CHECKER_HPP_

#include <autoware/planning_validator/plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{

class CollisionChecker : public PluginInterface
{
public:
  void init(
    rclcpp::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) override;
  void validate(bool & is_critical) override;
  void setup_diag() override;
  std::string get_module_name() const override { return module_name_; };

private:
  bool enable_latency_check_;
  bool is_critical_check_;
  double latency_threshold_;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__COLLISION_CHECKER_HPP_
