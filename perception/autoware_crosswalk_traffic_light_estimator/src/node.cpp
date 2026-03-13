// Copyright 2022-2025 TIER IV, Inc.
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

#include "node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>

#include <memory>

namespace autoware::crosswalk_traffic_light_estimator
{

CrosswalkTrafficLightEstimatorNode::CrosswalkTrafficLightEstimatorNode(
  const rclcpp::NodeOptions & options)
: Node("crosswalk_traffic_light_estimator", options)
{
  using std::placeholders::_1;

  CrosswalkTrafficLightEstimatorConfig config;
  config.use_last_detect_color = declare_parameter<bool>("use_last_detect_color");
  config.use_pedestrian_signal_detect = declare_parameter<bool>("use_pedestrian_signal_detect");
  config.last_detect_color_hold_time = declare_parameter<double>("last_detect_color_hold_time");
  config.flashing_detection.last_colors_hold_time =
    declare_parameter<double>("last_colors_hold_time");

  estimator_ = CrosswalkTrafficLightEstimator(config);

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CrosswalkTrafficLightEstimatorNode::on_map, this, _1));
  sub_traffic_light_array_ = create_subscription<TrafficSignalArray>(
    "~/input/classified/traffic_signals", rclcpp::QoS{1},
    std::bind(&CrosswalkTrafficLightEstimatorNode::on_traffic_light_array, this, _1));

  pub_traffic_light_array_ =
    this->create_publisher<TrafficSignalArray>("~/output/traffic_signals", rclcpp::QoS{1});
  pub_processing_time_ = std::make_shared<DebugPublisher>(this, "~/debug");
}

void CrosswalkTrafficLightEstimatorNode::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Start loading lanelet");
  auto lanelet_map_ptr = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  auto routing_graph_and_traffic_rules =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr);

  auto routing_graph_ptr =
    autoware::experimental::lanelet2_utils::remove_const(routing_graph_and_traffic_rules.first);

  estimator_.update_map(lanelet_map_ptr, routing_graph_ptr);

  RCLCPP_DEBUG(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Map is loaded");
}

void CrosswalkTrafficLightEstimatorNode::on_traffic_light_array(
  const TrafficSignalArray::ConstSharedPtr msg)
{
  if (!estimator_.is_map_loaded()) {
    RCLCPP_WARN(get_logger(), "cannot process traffic light array because the map is not received");
    return;
  }

  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");

  const auto unregistered_ids = estimator_.find_unregistered_traffic_light_group_ids(*msg);
  for (const auto & id : unregistered_ids) {
    RCLCPP_WARN(get_logger(), "Traffic light group ID %ld is not registered in the map", id);
  }

  const auto output = estimator_.estimate(*msg, get_clock()->now());

  pub_traffic_light_array_->publish(output);
  pub_processing_time_->publish<Float64Stamped>("processing_time_ms", stop_watch.toc("Total"));
}

}  // namespace autoware::crosswalk_traffic_light_estimator

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode)
