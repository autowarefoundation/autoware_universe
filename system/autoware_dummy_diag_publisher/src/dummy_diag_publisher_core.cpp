// Copyright 2020 Tier IV, Inc.
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

#include "autoware/dummy_diag_publisher/dummy_diag_publisher_core.hpp"

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

#include <sstream>

namespace autoware::dummy_diag_publisher
{
std::optional<DummyDiagPublisher::Status> DummyDiagPublisher::convertStrToStatus(
  const std::string & status_str)
{
  static std::unordered_map<std::string, Status> const table = {
    {"OK", Status::OK}, {"Warn", Status::WARN}, {"Error", Status::ERROR}, {"Stale", Status::STALE}};

  auto it = table.find(status_str);
  Status status;
  if (it != table.end()) {
    status = it->second;
    return status;
  }
  return {};
}
std::string DummyDiagPublisher::convertStatusToStr(const Status & status)
{
  if (status == Status::OK) {
    return std::string("OK");
  } else if (status == Status::WARN) {
    return std::string("Warn");
  } else if (status == Status::ERROR) {
    return std::string("Error");
  } else {
    return std::string("Stale");
  }
}

diagnostic_msgs::msg::DiagnosticStatus::_level_type DummyDiagPublisher::convertStatusToLevel(
  const Status & status)
{
  switch (status) {
    case Status::OK:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case Status::WARN:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case Status::ERROR:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    default:
      return diagnostic_msgs::msg::DiagnosticStatus::STALE;
  }
}

void DummyDiagPublisher::loadRequiredDiags()
{
  const auto param_key = std::string("required_diags");
  const uint64_t depth = 3;
  const auto param_names = this->list_parameters({param_key}, depth).names;

  if (param_names.empty()) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  std::set<std::string> diag_names;

  const std::string required_diags_prefix = param_key + std::string(".");
  static const std::string is_active_suffix = ".is_active";
  static const std::string status_suffix = ".status";
  const auto ends_with = [](const std::string & s, const std::string & suf) {
    return s.size() > suf.size() &&
           s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
  };

  for (const auto & param_name : param_names) {
    if (param_name.rfind(required_diags_prefix, 0) != 0) {
      continue;
    }

    std::string param_diag = param_name.substr(required_diags_prefix.size());
    if (ends_with(param_diag, is_active_suffix)) {
      param_diag.resize(param_diag.size() - is_active_suffix.size());
    } else if (ends_with(param_diag, status_suffix)) {
      param_diag.resize(param_diag.size() - status_suffix.size());
    }

    const auto diag_name_with_prefix = required_diags_prefix + param_diag;

    if (diag_names.count(diag_name_with_prefix) != 0) {
      continue;
    }

    diag_names.insert(diag_name_with_prefix);

    const auto is_active_key = diag_name_with_prefix + std::string(".is_active");
    std::string is_active_str;
    this->get_parameter_or(is_active_key, is_active_str, std::string("true"));
    const auto status_key = diag_name_with_prefix + std::string(".status");
    std::string status_str;
    this->get_parameter_or(status_key, status_str, std::string("OK"));

    bool is_active{};
    std::istringstream(is_active_str) >> std::boolalpha >> is_active;
    const auto status = convertStrToStatus(status_str);
    if (!status) {
      throw std::runtime_error(fmt::format("invalid status found: {}", status_str));
    }
    required_diags_.push_back({param_diag, is_active, *status});
  }
}

void DummyDiagPublisher::onTimer()
{
  diagnostic_msgs::msg::DiagnosticArray array;

  for (const auto & e : required_diags_) {
    if (e.is_active) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.hardware_id = diag_config_.hardware_id;
      status.name = e.name;
      status.message = convertStatusToStr(e.status);
      status.level = convertStatusToLevel(e.status);
      array.status.push_back(status);
    }
  }
  array.header.stamp = this->now();
  pub_->publish(array);
}

rclcpp::NodeOptions override_options(rclcpp::NodeOptions options)
{
  return options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
    true);
}

rcl_interfaces::msg::SetParametersResult DummyDiagPublisher::onSetParams(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    bool param_found = false;
    const auto & param_name = parameter.get_name();

    for (auto & diag : required_diags_) {
      if (param_name == diag.name + std::string(".status")) {
        param_found = true;
        auto new_status = convertStrToStatus(parameter.as_string());
        if (new_status) {
          diag.status = *new_status;
          RCLCPP_INFO(
            this->get_logger(), "Updated %s status to: %s", diag.name.c_str(),
            parameter.as_string().c_str());
        } else {
          result.successful = false;
          result.reason = "Invalid status value for: " + parameter.as_string();
          RCLCPP_WARN(
            this->get_logger(), "Invalid status value for %s: %s", diag.name.c_str(),
            parameter.as_string().c_str());
        }
      } else if (param_name == diag.name + std::string(".is_active")) {
        param_found = true;
        try {
          diag.is_active = parameter.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "Updated %s is_active to: %s", diag.name.c_str(),
            diag.is_active ? "true" : "false");
        } catch (const rclcpp::ParameterTypeException & e) {
          result.successful = false;
          result.reason = "Invalid is_active value for: " + parameter.as_string();
          RCLCPP_WARN(
            this->get_logger(), "Invalid is_active value for %s: %s", diag.name.c_str(),
            parameter.as_string().c_str());
        }
      }
    }

    if (!param_found) {
      result.successful = false;
      result.reason = "Parameter not registered: " + parameter.get_name();
      RCLCPP_WARN(
        this->get_logger(), "Attempted to set unregistered parameter: %s",
        parameter.get_name().c_str());
    }
  }

  return result;
}

DummyDiagPublisher::DummyDiagPublisher(const rclcpp::NodeOptions & options)
: Node("dummy_diag_publisher", override_options(options))

{
  // Parameter
  update_rate_ = this->get_parameter_or("update_rate", 10.0);

  // Diagnostic Updater
  loadRequiredDiags();

  const std::string hardware_id = "dummy_diag";
  diag_config_ = DiagConfig{hardware_id, "OK", "Warn", "Error", "Stale"};

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&DummyDiagPublisher::onTimer, this));

  // Publisher
  pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));

  // Parameter Callback Handle
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DummyDiagPublisher::onSetParams, this, std::placeholders::_1));
}
}  // namespace autoware::dummy_diag_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::dummy_diag_publisher::DummyDiagPublisher)
