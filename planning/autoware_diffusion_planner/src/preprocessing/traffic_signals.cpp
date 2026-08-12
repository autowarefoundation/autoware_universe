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
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
void process_traffic_signals(
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr> & msgs,
  std::map<lanelet::Id, TrafficSignalStamped> & traffic_signal_id_map,
  const rclcpp::Time & current_time, const double time_threshold_seconds)
{
  // Update traffic signals with the latest information
  for (const auto & msg : msgs) {
    const rclcpp::Time msg_time = msg->stamp;
    for (const auto & signal : msg->traffic_light_groups) {
      auto & curr = traffic_signal_id_map[signal.traffic_light_group_id];
      if (msg_time > rclcpp::Time(curr.stamp)) {
        curr.signal = signal;
        curr.stamp = msg_time;
      }
    }
  }

  // Remove outdated traffic signals
  auto itr = traffic_signal_id_map.begin();
  while (itr != traffic_signal_id_map.end()) {
    rclcpp::Time signal_time = itr->second.stamp;
    const double age = (current_time - signal_time).seconds();
    if (age > time_threshold_seconds) {
      itr = traffic_signal_id_map.erase(itr);
    } else {
      ++itr;
    }
  }
}

}  // namespace autoware::diffusion_planner::preprocess
