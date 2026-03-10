// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE_CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR__FLASHING_DETECTION_HPP_
#define AUTOWARE_CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR__FLASHING_DETECTION_HPP_

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{

using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficSignalElement = autoware_perception_msgs::msg::TrafficLightElement;
using TrafficSignalAndTime = std::pair<TrafficSignal, rclcpp::Time>;
using TrafficLightIdMap = std::unordered_map<lanelet::Id, TrafficSignalAndTime>;
using TrafficLightIdArray = std::unordered_map<lanelet::Id, std::vector<TrafficSignalAndTime>>;

struct FlashingDetectionConfig
{
  double last_colors_hold_time;
};

class FlashingDetector
{
public:
  explicit FlashingDetector(const FlashingDetectionConfig & config);
  FlashingDetector() = default;

  uint8_t estimate_stable_color(const TrafficSignal & signal, const rclcpp::Time & current_time);

  void clear_state(lanelet::Id id);

private:
  void update_signal_history(const TrafficSignal & signal, const rclcpp::Time & current_time);
  void remove_expired_entries(lanelet::Id id, const rclcpp::Time & current_time);
  void update_flashing_state(const TrafficSignal & signal);
  uint8_t update_and_get_color_state(const TrafficSignal & signal);

  FlashingDetectionConfig config_{};
  std::unordered_map<lanelet::Id, bool> is_flashing_;
  std::unordered_map<lanelet::Id, uint8_t> current_color_state_;
  TrafficLightIdArray signal_history_;
};

}  // namespace autoware::crosswalk_traffic_light_estimator

#endif  // AUTOWARE_CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR__FLASHING_DETECTION_HPP_
