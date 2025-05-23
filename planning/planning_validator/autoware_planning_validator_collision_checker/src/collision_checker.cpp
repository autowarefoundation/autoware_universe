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

#include "autoware/planning_validator_collision_checker/collision_checker.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using autoware_utils::get_or_declare_parameter;

void CollisionChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;

  clock_ = node.get_clock();
  logger_ = node.get_logger();
  context_ = context;

  setup_diag();
}

void CollisionChecker::validate(bool & is_critical)
{
  (void)is_critical;
}

void CollisionChecker::setup_diag()
{
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::CollisionChecker, autoware::planning_validator::PluginInterface)
