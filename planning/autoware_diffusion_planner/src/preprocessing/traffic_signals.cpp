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

#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"

#include <iostream>
#include <map>

namespace autoware::diffusion_planner::preprocess
{
void process_traffic_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg,
  std::map<lanelet::Id, TrafficSignalStamped> & traffic_signal_id_map,
  const rclcpp::Time & current_time, const double time_threshold_seconds,
  const bool keep_last_observation)
{
  // clear previous observation
  if (!msg) {
    return;
  }

  rclcpp::Time msg_time = msg->stamp;
  const auto time_diff = (current_time - msg_time).seconds();
  if (time_diff > time_threshold_seconds) {
    std::cerr << "WARNING(" << __func__
              << ") TrafficLightGroupArray message is too old. Message discarded.\n";
    // Discard outdated message
    return;
  }
  const auto traffic_light_id_map_last_observed_old = traffic_signal_id_map;
  traffic_signal_id_map.clear();

  for (const auto & signal : msg->traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg->stamp;
    traffic_signal.signal = signal;
    traffic_signal_id_map[signal.traffic_light_group_id] = traffic_signal;
    if (!keep_last_observation) {
      continue;
    }

    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      });
    const auto old_data =
      traffic_light_id_map_last_observed_old.find(signal.traffic_light_group_id);

    if (is_unknown_observation && old_data != traffic_light_id_map_last_observed_old.end()) {
      traffic_signal_id_map[signal.traffic_light_group_id] = old_data->second;
      traffic_signal_id_map[signal.traffic_light_group_id].stamp = msg->stamp;
    }
  }
}

}  // namespace autoware::diffusion_planner::preprocess
