// Copyright 2023 TIER IV, Inc.
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

#include "autoware/scenario_simulator_v2_adapter/converter_node.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <regex>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::scenario_simulator_v2_adapter
{
std::string removeInvalidTopicString(const std::string & input_string)
{
  std::regex pattern{R"([a-zA-Z0-9/_]+)"};

  std::string result;
  for (std::sregex_iterator itr(std::begin(input_string), std::end(input_string), pattern), end;
        itr != end; ++itr) {
    result += itr->str();
  }

  return std::regex_replace(result, std::regex(R"(/+)"), "/");
}

MetricConverter::MetricConverter(const rclcpp::NodeOptions & node_options)
: Node("scenario_simulator_v2_adapter", node_options)
{
  using std::placeholders::_1;

  // metric subscription
  std::vector<std::string> metric_topic_list;
  declare_parameter<std::vector<std::string>>("metric_topic_list", std::vector<std::string>());
  get_parameter<std::vector<std::string>>("metric_topic_list", metric_topic_list);
  for (const std::string & metric_topic : metric_topic_list) {
    // std::function required with multiple arguments https://answers.ros.org/question/289207
    const std::function<void(const MetricArray::ConstSharedPtr)> fn =
      std::bind(&MetricConverter::onMetrics, this, _1, metric_topic);
    metrics_sub_.push_back(create_subscription<MetricArray>(metric_topic, 1, fn));
  }

  // diagnostics subscription
  loadDiagnosticConfig();
  const std::function<void(const DiagnosticArray::ConstSharedPtr)> fn =
    std::bind(&MetricConverter::onDiagnostics, this, _1, std::cref(diagnostic_aggregation_map_));
  diagnostics_sub_ = create_subscription<DiagnosticArray>("/diagnostics", 1, fn);
}

void MetricConverter::onMetrics(
  const MetricArray::ConstSharedPtr metrics_msg, const std::string & base_topic_name)
{
  for (const auto & metric : metrics_msg->metric_array) {
    std::string metric_name = base_topic_name + (metric.name.empty() ? "" : "/" + metric.name);
    const auto valid_topic_name = removeInvalidTopicString(metric_name);
    getPublisher(valid_topic_name)->publish(createUserDefinedValue(metric));
  }
}

void MetricConverter::onDiagnostics(
  const DiagnosticArray::ConstSharedPtr diagnostics_msg,
  const std::unordered_map<std::string, std::unordered_set<std::string>> & diagnostic_aggregation_map)
{
  for (const auto & status : diagnostics_msg->status) {
    std::string diag_name = "/diagnostics/" + status.name;
    size_t pos = diag_name.find(": ");
    if (pos != std::string::npos) {
      diag_name.replace(pos, 2, "/");  // Replace ": " with "/"
    }
    const auto valid_topic_name = removeInvalidTopicString(diag_name);
    getPublisher(valid_topic_name)->publish(createUserDefinedValue(status));

    // Check if the diagnostic is in any of the aggregation lists
    for (const auto & [output_topic_name, diagnostic_aggregation_set] : diagnostic_aggregation_map) {
      if (diagnostic_aggregation_set.find(valid_topic_name) != diagnostic_aggregation_set.end()) {
        getPublisher(output_topic_name)->publish(createUserDefinedValue(status));
        if (status.level == DiagnosticStatus::ERROR) {
          RCLCPP_WARN(get_logger(), "Diagnostic ERROR %s is in the aggregation list for %s", valid_topic_name.c_str(), output_topic_name.c_str());
        }
      }
    }
  }
}

UserDefinedValue MetricConverter::createUserDefinedValue(const Metric & metric) const
{
  UserDefinedValue param_msg;
  param_msg.type.data = UserDefinedValueType::DOUBLE;
  param_msg.value = metric.value;
  return param_msg;
}

UserDefinedValue MetricConverter::createUserDefinedValue(const DiagnosticStatus & status) const
{
  UserDefinedValue param_msg;
  param_msg.type.data = UserDefinedValueType::UNSIGNED_INT;
  param_msg.value = std::to_string(status.level);
  return param_msg;
}

rclcpp::Publisher<UserDefinedValue>::SharedPtr MetricConverter::getPublisher(
  const std::string & topic_name)
{
  if (params_pub_.count(topic_name) == 0) {
    params_pub_[topic_name] = create_publisher<UserDefinedValue>(topic_name, 1);
  }
  return params_pub_.at(topic_name);
}

void MetricConverter::loadDiagnosticConfig()
{
  declare_parameter<std::string>("diagnostic_config_file", "");
  std::string yaml_path = get_parameter("diagnostic_config_file").as_string();
  
  if (yaml_path.empty()) {
    return;
  }

  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    YAML::Node params = config["/**"]["ros__parameters"];
    
    if (!params["diagnostic_groups"]) {
      return;
    }

    // First pass: Load all groups first (needed for lookup during expansion)
    std::unordered_map<std::string, std::vector<std::string>> temp_map;
    for (const auto & group : params["diagnostic_groups"]) {
      if (!group["output_topic_name"] || !group["aggregation_list"]) {
        continue;
      }

      std::string output_topic = group["output_topic_name"].as<std::string>();
      std::vector<std::string> aggregation_list;
      
      for (const auto & item : group["aggregation_list"]) {
        aggregation_list.push_back(item.as<std::string>());
      }
      
      temp_map[output_topic] = aggregation_list;
    }
    
    // Second pass: Recursively expand nested groups to individual diagnostic topics, then convert to set for O(1) lookup
    for (auto & [output_topic, aggregation_list] : temp_map) {
      std::set<std::string> visited;
      std::vector<std::string> expanded = expandAggregationList(output_topic, aggregation_list, temp_map, visited);
      diagnostic_aggregation_map_[output_topic] = std::unordered_set<std::string>(expanded.begin(), expanded.end());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load diagnostic config file '%s': %s", yaml_path.c_str(), e.what());
  }
}

std::vector<std::string> MetricConverter::expandAggregationList(
  const std::string & group_output_topic,
  const std::vector<std::string> & aggregation_list,
  const std::unordered_map<std::string, std::vector<std::string>> & temp_map,
  std::set<std::string> & visited)
{
  // Detect circular dependencies to prevent infinite recursion
  if (visited.find(group_output_topic) != visited.end()) {
    RCLCPP_WARN(
      get_logger(), 
      "Circular dependency detected for group: %s", 
      group_output_topic.c_str()
    );
    return aggregation_list;
  }
  
  visited.insert(group_output_topic);
  
  std::vector<std::string> expanded_list;
  
  for (const auto & item : aggregation_list) {
    auto it = temp_map.find(item);
    if (it != temp_map.end()) {
      // Item is a group, recursively expand it
      const auto & nested_list = it->second;
      auto expanded_nested = expandAggregationList(item, nested_list, temp_map, visited);
      expanded_list.insert(expanded_list.end(), expanded_nested.begin(), expanded_nested.end());
    } else {
      expanded_list.push_back(item);
    }
  }

  visited.erase(group_output_topic);  // Allow same group in different branches
  return expanded_list;
}
}  // namespace autoware::scenario_simulator_v2_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::scenario_simulator_v2_adapter::MetricConverter)
