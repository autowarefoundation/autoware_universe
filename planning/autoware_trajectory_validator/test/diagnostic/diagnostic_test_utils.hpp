// Copyright 2026 TIER IV, Inc.
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

#ifndef DIAGNOSTIC__DIAGNOSTIC_TEST_UTILS_HPP_
#define DIAGNOSTIC__DIAGNOSTIC_TEST_UTILS_HPP_

#include "autoware/trajectory_validator/detail/diagnostic.hpp"

#include <autoware_trajectory_validator/autoware_trajectory_validator_diagnostic_param.hpp>
#include <autoware_trajectory_validator/msg/metric_report.hpp>
#include <autoware_trajectory_validator/msg/risk_level.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using autoware::trajectory_validator::Action;
using autoware::trajectory_validator::build_diagnostic_interface_map;
using autoware::trajectory_validator::FilterConfiguredActionsMap;
using autoware::trajectory_validator::TrajectoryValidatorDiagnostic;
using autoware_trajectory_validator::msg::MetricReport;
using autoware_trajectory_validator::msg::RiskLevel;
using autoware_trajectory_validator::msg::ValidationReport;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

inline MetricReport make_metric(const std::string & validator_name, uint8_t risk_level)
{
  MetricReport m;
  m.validator_name = validator_name;
  m.risk.level = risk_level;
  m.metric_name = "test_metric";
  m.metric_value = 0.0;
  return m;
}

inline ValidationReport make_report(std::vector<MetricReport> metrics)
{
  ValidationReport r;
  r.metrics = std::move(metrics);
  return r;
}

inline bool spin_until(
  rclcpp::Node::SharedPtr node, std::function<bool()> pred,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  rclcpp::Rate rate(200);
  while (std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    if (pred()) return true;
    rate.sleep();
  }
  return false;
}

inline const DiagnosticStatus * find_status(
  const DiagnosticArray & arr, const std::string & name_suffix)
{
  for (const auto & diag_status : arr.status) {
    const auto pos = diag_status.name.find(": ");
    if (pos != std::string::npos && diag_status.name.substr(pos + 2) == name_suffix) {
      return &diag_status;
    }
  }
  return nullptr;
}

inline TrajectoryValidatorDiagnostic make_diag(
  rclcpp::Node & node, FilterConfiguredActionsMap filter_map,
  std::string no_candidates_diag_status_name,
  const std::unordered_set<std::string> & active_filter_names = {})
{
  trajectory_validator_diagnostic::Params params;
  params.no_candidates_diag_status_name = no_candidates_diag_status_name;
  auto diag_by_name =
    build_diagnostic_interface_map(node, filter_map, no_candidates_diag_status_name);
  return {std::move(filter_map), params, active_filter_names, std::move(diag_by_name)};
}

struct DiagHarness
{
  rclcpp::Node::SharedPtr node;
  std::vector<DiagnosticArray> received;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub;

  explicit DiagHarness(const std::string & node_name = "test_node")
  {
    node = rclcpp::Node::make_shared(node_name);
    sub = node->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(20),
      [this](const DiagnosticArray::SharedPtr msg) { received.push_back(*msg); });
  }

  void run(TrajectoryValidatorDiagnostic & diag, const std::vector<ValidationReport> & reports)
  {
    spin_until(node, [this]() { return sub->get_publisher_count() > 0; });
    received.clear();
    const auto stamp = node->get_clock()->now();
    diag.update_and_publish(reports, stamp);
    spin_until(node, [this]() { return !received.empty(); });
    rclcpp::spin_some(node);
    rclcpp::spin_some(node);
  }

  [[nodiscard]] std::vector<DiagnosticStatus> all_statuses() const
  {
    std::vector<DiagnosticStatus> result;
    for (const auto & arr : received) {
      for (const auto & s : arr.status) {
        result.push_back(s);
      }
    }
    return result;
  }

  [[nodiscard]] std::optional<DiagnosticStatus> find(const std::string & name_suffix) const
  {
    std::optional<DiagnosticStatus> found;
    for (const auto & arr : received) {
      if (const auto * matched = find_status(arr, name_suffix)) {
        found = *matched;
      }
    }
    return found;
  }
};

#endif  // DIAGNOSTIC__DIAGNOSTIC_TEST_UTILS_HPP_
