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
  latency_threshold_ = get_or_declare_parameter<double>(node, "latency_checker.threshold");

  setup_diag();
}

void LatencyChecker::validate()
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
}

void LatencyChecker::setup_diag()
{
  if (!context_->diag_updater) return;

  const auto & status = context_->validation_status->is_valid_latency;
  context_->diag_updater->add("trajectory_validation_latency", [&](auto & stat) {
    const std::string msg = "latency is larger than expected value";
    set_diag_status(stat, status, msg);
  });
}

void LatencyChecker::set_diag_status(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
    return;
  }

  const auto invalid_count = context_->validation_status->invalid_count;
  const auto count_threshold = context_->params.diag_error_count_threshold;
  if (invalid_count < count_threshold) {
    const auto warn_msg =
      msg + " (invalid count is less than error threshold: " + std::to_string(invalid_count) +
      " < " + std::to_string(count_threshold) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
    return;
  }

  stat.summary(DiagnosticStatus::ERROR, msg);
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::LatencyChecker, autoware::planning_validator::PluginInterface)
