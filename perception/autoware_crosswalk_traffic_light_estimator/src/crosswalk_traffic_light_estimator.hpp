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

#ifndef CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR_HPP_
#define CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR_HPP_

#include "flashing_detection.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{

struct CrosswalkTrafficLightEstimatorConfig
{
  bool use_last_detect_color;
  bool use_pedestrian_signal_detect;
  double last_detect_color_hold_time;
  FlashingDetectionConfig flashing_detection;
};

class CrosswalkTrafficLightEstimator
{
public:
  explicit CrosswalkTrafficLightEstimator(const CrosswalkTrafficLightEstimatorConfig & config);
  CrosswalkTrafficLightEstimator() = default;

  void update_map(lanelet::LaneletMapPtr lanelet_map_ptr);

  bool is_map_loaded() const;

  TrafficSignalArray estimate(const TrafficSignalArray & msg, const rclcpp::Time & current_time);

  std::vector<lanelet::Id> find_unregistered_traffic_light_group_ids(
    const TrafficSignalArray & msg) const;

private:
  void update_last_detected_signal(
    const TrafficLightIdMap & traffic_light_id_map, const rclcpp::Time & current_time);
  /// @brief update the overrides of crosswalk signals from the lanelet map for the given traffic
  /// light id
  void update_crosswalk_overrides_from_map(
    std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides,
    lanelet::Id traffic_light_group_id, const TrafficLightIdMap & traffic_light_id_map);

  void set_crosswalk_traffic_signal(
    const lanelet::ConstLanelet & crosswalk, const uint8_t color, const TrafficSignalArray & msg,
    TrafficSignalArray & output,
    const std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides,
    const rclcpp::Time & current_time);

  lanelet::ConstLanelets get_non_red_lanelets(
    const lanelet::ConstLanelets & lanelets, const TrafficLightIdMap & traffic_light_id_map) const;

  uint8_t estimate_crosswalk_traffic_signal(
    const lanelet::ConstLanelet & crosswalk, const lanelet::ConstLanelets & non_red_lanelets) const;

  CrosswalkTrafficLightEstimatorConfig config_{};

  // Map data
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelets> traffic_light_id_to_crosswalks_;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelets> crosswalk_to_vehicle_lanelets_;

  // Signal state
  TrafficLightIdMap last_detect_color_;
  FlashingDetector flashing_detector_;
};

}  // namespace autoware::crosswalk_traffic_light_estimator

#endif  // CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR_HPP_
