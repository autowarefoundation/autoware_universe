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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "flashing_detection.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <unordered_map>
#include <vector>
namespace autoware::crosswalk_traffic_light_estimator
{

using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_utils::DebugPublisher;
using autoware_utils::StopWatch;

class CrosswalkTrafficLightEstimatorNode : public rclcpp::Node
{
public:
  explicit CrosswalkTrafficLightEstimatorNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr sub_traffic_light_array_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_traffic_light_array_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelets> traffic_light_id_to_crosswalks_;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelets> crosswalk_to_vehicle_lanelets_;

  void on_map(const LaneletMapBin::ConstSharedPtr msg);
  void on_traffic_light_array(const TrafficSignalArray::ConstSharedPtr msg);

  void update_last_detected_signal(const TrafficLightIdMap & traffic_light_id_map);
  /// @brief update the overrides of crosswalk signals from the lanelet map for the given traffic
  /// light id
  void update_crosswalk_overrides_from_map(
    std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides,
    const lanelet::Id traffic_light_group_id, const TrafficLightIdMap & traffic_light_id_map);

  void set_crosswalk_traffic_signal(
    const lanelet::ConstLanelet & crosswalk, const uint8_t color, const TrafficSignalArray & msg,
    TrafficSignalArray & output,
    const std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides);

  lanelet::ConstLanelets get_non_red_lanelets(
    const lanelet::ConstLanelets & lanelets, const TrafficLightIdMap & traffic_light_id_map) const;

  uint8_t estimate_crosswalk_traffic_signal(
    const lanelet::ConstLanelet & crosswalk, const lanelet::ConstLanelets & non_red_lanelets) const;

  boost::optional<uint8_t> get_highest_confidence_traffic_signal(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    const TrafficLightIdMap & traffic_light_id_map) const;

  boost::optional<uint8_t> get_highest_confidence_traffic_signal(
    const lanelet::Id & id, const TrafficLightIdMap & traffic_light_id_map) const;

  void remove_duplicate_ids(TrafficSignalArray & signal_array) const;

  bool is_invalid_detection_status(const TrafficSignal & signal) const;

  // Node param
  bool use_last_detect_color_;
  bool use_pedestrian_signal_detect_;
  double last_detect_color_hold_time_;

  // Signal history
  TrafficLightIdMap last_detect_color_;

  // Flashing detection
  FlashingDetector flashing_detector_;

  // Stop watch
  StopWatch<std::chrono::milliseconds> stop_watch_;

  // Debug
  std::shared_ptr<DebugPublisher> pub_processing_time_;
};

}  // namespace autoware::crosswalk_traffic_light_estimator

#endif  // NODE_HPP_
