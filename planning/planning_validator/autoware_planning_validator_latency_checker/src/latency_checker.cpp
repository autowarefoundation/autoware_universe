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

#include "autoware/planning_validator_latency_checker/latency_checker.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using autoware_utils::get_or_declare_parameter;

void LatencyChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;

  clock_ = node.get_clock();
  logger_ = node.get_logger();
  context_ = context;

  enable_latency_check_ = get_or_declare_parameter<bool>(node, "latency_checker.enable");
  is_critical_check_ = get_or_declare_parameter<bool>(node, "latency_checker.is_critical");
  latency_threshold_ = get_or_declare_parameter<double>(node, "latency_checker.threshold");

  setup_diag();
}

void LatencyChecker::validate(bool & is_critical)
{
  const auto & data = context_->data;
  auto & status = context_->validation_status;
  if (!enable_latency_check_ || !data->current_trajectory) {
    return;
  }

  const auto & current_trajectory = *data->current_trajectory;
  const auto latency = (clock_->now() - current_trajectory.header.stamp).seconds();

  status->latency = latency;
  status->is_valid_latency = latency <= latency_threshold_;
  is_critical = !status->is_valid_latency && is_critical_check_;
}

void LatencyChecker::setup_diag()
{
  context_->add_diag(
    "trajectory_validation_latency", context_->validation_status->is_valid_latency,
    "latency is larger than expected value", is_critical_check_);
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::LatencyChecker, autoware::planning_validator::PluginInterface)
