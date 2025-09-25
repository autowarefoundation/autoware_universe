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
  struct LaneletRecord
  {
    lanelet::LaneletMapPtr lanelet_map;
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
    lanelet::routing::RoutingGraphPtr routing_graph;

    /**
     * @brief Check if the all lanelet relevant data is available.
     *
     * @return True if the context is available, false otherwise.
     */
    bool is_available() const { return lanelet_map && traffic_rules && routing_graph; }
  };

  Context() : lanelet_record_(std::make_unique<LaneletRecord>()) {}

  /**
   * @brief Get the reference to the current set of predicted objects.
   *
   * @return The current set of predicted objects.
   */
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects() const
  {
    return objects_;
  }

  /**
   * @brief Update the context with a new set of predicted objects.
   *
   * @param new_objects The new set of predicted objects.
   */
  void update(autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr new_objects)
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
    lanelet_record_->lanelet_map = std::move(lanelet_map);
    lanelet_record_->traffic_rules = std::move(traffic_rules);
    lanelet_record_->routing_graph = std::move(routing_graph);
  }

  /**
   * @brief Get the lanelet map.
   *
   * @return The lanelet map (may be nullptr if not set)
   */
  const lanelet::LaneletMapPtr & lanelet_map() const { return lanelet_record_->lanelet_map; }

  /**
   * @brief Get the traffic rules.
   *
   * @return The traffic rules (may be nullptr if not set)
   */
  const lanelet::traffic_rules::TrafficRulesPtr & traffic_rules() const
  {
    return lanelet_record_->traffic_rules;
  }

  /**
   * @brief Get the routing graph.
   *
   * @return The routing graph (may be nullptr if not set)
   */
  const lanelet::routing::RoutingGraphPtr & routing_graph() const
  {
    return lanelet_record_->routing_graph;
  }

private:
  // NOTE: In the future, we may want to add more context information here, such as traffic light,
  // etc.
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_{nullptr};
  std::unique_ptr<LaneletRecord> lanelet_record_;
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

  const std::string & name() const { return processor_name_; }

  /**
   * @brief Load configuration parameters for the processor.
   *
   * @param node_ptr The node to load parameters from.
   * @param processor_name The name of the processor.
   * @param declare_parameters The function to declare parameters.
   */
  void load_config(
    rclcpp::Node * node_ptr, const std::string & processor_name,
    const DeclareParametersFunc & declare_parameters)
  {
    declare_parameters(node_ptr, processor_name);

    auto parameters_client = rclcpp::SyncParametersClient(node_ptr);

    const auto package_share_dir =
      ament_index_cpp::get_package_share_directory("autoware_predicted_path_postprocessor");
    auto yaml_filepath = package_share_dir + "/config/" + processor_name + ".param.yaml";

    const auto results = parameters_client.load_parameters(std::move(yaml_filepath));

    for (const auto & result : results) {
      if (!result.successful) {
        RCLCPP_ERROR_STREAM(
          node_ptr->get_logger(),
          "Failed to load parameters for " << processor_name << ": " << result.reason);
      }
    }
  }

  /**
   * @brief Process a single predicted object.
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
