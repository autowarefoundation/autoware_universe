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

#ifndef AUTOWARE__SCENARIO_SIMULATOR_V2_ADAPTER__CONVERTER_NODE_HPP_
#define AUTOWARE__SCENARIO_SIMULATOR_V2_ADAPTER__CONVERTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "tier4_simulation_msgs/msg/user_defined_value.hpp"
#include "tier4_simulation_msgs/msg/user_defined_value_type.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::scenario_simulator_v2_adapter
{
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_metric_msgs::msg::Metric;
using tier4_metric_msgs::msg::MetricArray;
using tier4_simulation_msgs::msg::UserDefinedValue;
using tier4_simulation_msgs::msg::UserDefinedValueType;

struct DiagnosticAggregationMap
{
  std::string output_topic_name;
  std::unordered_set<std::string> aggregation_list;
};

/**
 * @brief Node for converting Autoware's messages to UserDefinedValue
 */
class MetricConverter : public rclcpp::Node
{
public:
  explicit MetricConverter(const rclcpp::NodeOptions & node_options);

  /**
   * @brief callback for MetricArray msgs that publishes equivalent UserDefinedValue msgs
   * @param [in] metrics_msg received metrics message
   */
  void onMetrics(const MetricArray::ConstSharedPtr metrics_msg, const std::string & topic);

  /**
   * @brief callback for DiagnosticArray msgs that can be used to handle diagnostics if needed
   * @param [in] diagnostics_msg received diagnostics message
   * @param [in] diagnostic_aggregation_maps vector of DiagnosticAggregationMap
   */
  void onDiagnostics(
    const DiagnosticArray::ConstSharedPtr diagnostics_msg,
    const std::unordered_map<std::string, DiagnosticAggregationMap> & diagnostic_aggregation_maps);

  UserDefinedValue createUserDefinedValue(const Metric & metric) const;
  UserDefinedValue createUserDefinedValue(const DiagnosticStatus & status) const;

  rclcpp::Publisher<UserDefinedValue>::SharedPtr getPublisher(const std::string & topic);

private:
  std::unordered_map<std::string, DiagnosticAggregationMap> loadDiagnosticConfig();

  // Helper functions for loadDiagnosticConfig
  std::unordered_map<std::string, DiagnosticAggregationMap> collectDiagnosticGroupsFromParams();
  void validateAndFilterGroups(std::unordered_map<std::string, DiagnosticAggregationMap> & groups);
  std::unordered_map<std::string, std::string> createTopicToGroupMap(
    const std::unordered_map<std::string, DiagnosticAggregationMap> & groups);

  std::unordered_set<std::string> expandAggregationList(
    const std::string & group_name, const std::unordered_set<std::string> & aggregation_list,
    const std::unordered_map<std::string, DiagnosticAggregationMap> & diagnostic_aggregation_maps,
    const std::unordered_map<std::string, std::string> & output_topic_to_group_name,
    std::set<std::string> & visited);

  // ROS
  std::vector<rclcpp::Subscription<MetricArray>::SharedPtr> metrics_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;

  std::unordered_map<std::string, rclcpp::Publisher<UserDefinedValue>::SharedPtr> params_pub_;
};
}  // namespace autoware::scenario_simulator_v2_adapter

#endif  // AUTOWARE__SCENARIO_SIMULATOR_V2_ADAPTER__CONVERTER_NODE_HPP_
