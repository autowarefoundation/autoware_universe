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

#include "autoware/predicted_path_postprocessor/node.hpp"

#include "autoware/predicted_path_postprocessor/processor/composable.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <chrono>
#include <memory>
#include <ratio>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor
{
using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using Float64Stamped = autoware_internal_debug_msgs::msg::Float64Stamped;

PredictedPathPostprocessorNode::PredictedPathPostprocessorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("predicted_path_postprocessor", options)
{
  auto processors = declare_parameter<std::vector<std::string>>("processors");
  context_ = std::make_unique<processor::Context>();
  processor_ = std::make_unique<processor::ComposableProcessor>(this, std::move(processors));

  stopwatch_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stopwatch_->tic("cyclic_time");
  stopwatch_->tic("processing_time");
  debug_publisher_ = std::make_unique<autoware_utils_debug::DebugPublisher>(this, get_name());

  // NOTE: add additional subscriptions if needed, such as traffic light, etc.
  object_subscription_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    [this, debug = declare_parameter<bool>("debug")](const PredictedObjects::ConstSharedPtr & msg) {
      this->stopwatch_->toc("processing_time", true);

      this->callback(msg, debug);  // main callback

      const auto cyclic_time = this->stopwatch_->toc("cyclic_time", true);
      const auto processing_time = this->stopwatch_->toc("processing_time", true);
      this->debug_publisher_->publish<Float64Stamped>("debug/cyclic_time_ms", cyclic_time);
      this->debug_publisher_->publish<Float64Stamped>("debug/processing_time_ms", processing_time);
    });

  lanelet_subscription_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet_map", rclcpp::QoS{1}.transient_local(),
    [this](const LaneletMapBin::ConstSharedPtr & msg) { this->on_map(msg); });

  object_publisher_ = create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});
}

void PredictedPathPostprocessorNode::callback(
  const PredictedObjects::ConstSharedPtr & msg, bool debug)
{
  auto objects = std::make_shared<PredictedObjects>(*msg);

  // update the context with the objects data
  context_->update(objects);

  if (debug) {
    const auto publish_reports = [this](const auto & reports) {
      for (const auto & [processor_name, report] : reports) {
        const auto topic_namespace = "debug/" + processor_name;

        debug_publisher_->publish<Float64Stamped>(
          topic_namespace + "/processing_time_ms", report.processing_time_ms);
        debug_publisher_->publish<PredictedObjects>(topic_namespace + "/objects", report.objects);
      }
    };

    const auto result = processor_->process_with_reports(objects, *context_);
    if (result) {
      const auto & [output, reports] = result.ok();
      object_publisher_->publish(output);
      publish_reports(reports);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to process objects: " << result.err());
    }
  } else {
    const auto result = processor_->process(objects, *context_);
    if (result) {
      object_publisher_->publish(result.ok());
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to process objects: " << result.err());
    }
  }
}

void PredictedPathPostprocessorNode::on_map(const LaneletMapBin::ConstSharedPtr & msg)
{
  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map, &traffic_rules, &routing_graph);

  // update the context with the extracted lanelet data
  context_->update(lanelet_map, traffic_rules, routing_graph);
}
}  // namespace autoware::predicted_path_postprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::predicted_path_postprocessor::PredictedPathPostprocessorNode);
