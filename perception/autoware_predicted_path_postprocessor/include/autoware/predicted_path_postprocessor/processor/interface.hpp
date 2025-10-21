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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__INTERFACE_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__INTERFACE_HPP_

#include "autoware/predicted_path_postprocessor/processor/result.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
/**
 * @brief A context which processors can access.
 */
class Context
{
public:
  Context() {}

  /**
   * @brief Return the read-only reference to the current set of predicted objects.
   *
   * @return The current set of predicted objects.
   */
  autoware_perception_msgs::msg::PredictedObjects::SharedPtr as_objects() const { return objects_; }

  /**
   * @brief Update the context with a new set of predicted objects.
   *
   * @param new_objects The new set of predicted objects.
   */
  void update(autoware_perception_msgs::msg::PredictedObjects::SharedPtr new_objects)
  {
    objects_ = std::move(new_objects);
  }

  /**
   * @brief Update the context with lanelet map data.
   *
   * @param lanelet_map The lanelet map
   * @param traffic_rules The traffic rules
   * @param routing_graph The routing graph
   */
  void update(
    lanelet::LaneletMapPtr lanelet_map, lanelet::traffic_rules::TrafficRulesPtr traffic_rules,
    lanelet::routing::RoutingGraphPtr routing_graph)
  {
    lanelet_record_.lanelet_map = std::move(lanelet_map);
    lanelet_record_.traffic_rules = std::move(traffic_rules);
    lanelet_record_.routing_graph = std::move(routing_graph);
  }

  /**
   * @brief Return the shared pointer to the lanelet map.
   *
   * @return The lanelet map (may be nullptr if not set)
   */
  lanelet::LaneletMapPtr as_lanelet_map() const { return lanelet_record_.lanelet_map; }

  /**
   * @brief Return the shared pointer to the traffic rules.
   *
   * @return The traffic rules (may be nullptr if not set)
   */
  lanelet::traffic_rules::TrafficRulesPtr as_traffic_rules() const
  {
    return lanelet_record_.traffic_rules;
  }

  /**
   * @brief Return the shared pointer to the routing graph.
   *
   * @return The routing graph (may be nullptr if not set)
   */
  lanelet::routing::RoutingGraphPtr as_routing_graph() const
  {
    return lanelet_record_.routing_graph;
  }

private:
  struct LaneletRecord
  {
    lanelet::LaneletMapPtr lanelet_map;
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
    lanelet::routing::RoutingGraphPtr routing_graph;
  };

  // NOTE: Add more context information below if needed, such as traffic light etc.
  autoware_perception_msgs::msg::PredictedObjects::SharedPtr objects_;
  LaneletRecord lanelet_record_;
};

/**
 * @brief Interface for processors that process a single predicted object.
 */
class ProcessorInterface
{
public:
  using UniquePtr = std::unique_ptr<ProcessorInterface>;
  using DeclareParametersFunc = std::function<void(rclcpp::Node *, const std::string &)>;

  using target_type = autoware_perception_msgs::msg::PredictedObject;
  using error_type = std::string;  // TODO(ktro2828): Define concrete error type
  using result_type = EmptyResult<error_type>;

  explicit ProcessorInterface(const std::string & processor_name) : processor_name_(processor_name)
  {
  }

  virtual ~ProcessorInterface() = default;

  /**
   * @brief Return the read-only reference to the processor name.
   *
   * @return const std::string& The name of the processor.
   */
  const std::string & name() const noexcept { return processor_name_; }

  /**
   * @brief Run a processing for a single predicted object.
   *
   * @param target The predicted object to process.
   * @param context The context in which the object is processed.
   * @return The result of the processing.
   */
  result_type run(target_type & target, const Context & context)
  {
    if (auto result = check_context(context); !result) {
      return result;
    }
    return process(target, context);
  };

protected:
  /**
   * @brief Check the validity of the context.
   *
   * @param context The context to check.
   * @return The result of the check.
   */
  virtual result_type check_context(const Context &) const noexcept
  {
    return make_ok<error_type>();
  };

  /**
   * @brief Perform processor specific processing.
   *
   * @param target The predicted object to process.
   * @param context The context in which the object is processed.
   * @return The result of the processing.
   */
  virtual result_type process(target_type & target, const Context & context) = 0;

private:
  const std::string processor_name_;  //!< Name of the processor.
};
}  // namespace autoware::predicted_path_postprocessor::processor

#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__INTERFACE_HPP_
