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

#include <regex>
#include <string>
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
  diagnostics_sub_ = create_subscription<DiagnosticArray>(
    "/diagnostics", 10, std::bind(&MetricConverter::onDiagnostics, this, std::placeholders::_1));
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

void MetricConverter::onDiagnostics(const DiagnosticArray::ConstSharedPtr diagnostics_msg)
{
  for (const auto & status : diagnostics_msg->status) {
    std::string diag_name = "/diagnostics/" + status.name;
    size_t pos = diag_name.find(": ");
    if (pos != std::string::npos) {
      diag_name.replace(pos, 2, "/");  // Replace ": " with "/"
    }
    const auto valid_topic_name = removeInvalidTopicString(diag_name);
    getPublisher(valid_topic_name)->publish(createUserDefinedValue(status));
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
}  // namespace autoware::scenario_simulator_v2_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::scenario_simulator_v2_adapter::MetricConverter)
