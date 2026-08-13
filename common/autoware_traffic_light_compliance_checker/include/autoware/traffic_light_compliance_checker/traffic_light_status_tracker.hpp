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

#ifndef AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__TRAFFIC_LIGHT_STATUS_TRACKER_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__TRAFFIC_LIGHT_STATUS_TRACKER_HPP_

#include "autoware/traffic_light_compliance_checker/structs.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <cstdint>
#include <unordered_map>

namespace autoware::traffic_light_compliance_checker
{

/// @brief tracks traffic light signal stability over time
class TrafficLightStatusTracker
{
public:
  explicit TrafficLightStatusTracker(const StatusTrackerParameters & parameters);

  void update_parameters(const StatusTrackerParameters & parameters);

  /**
   * @brief filter unstable red/amber signals using per-signal state history
   * @param signals raw traffic light signals
   * @param current_time current time stamp
   * @param is_ego_stopped true if ego velocity is below the stopped threshold
   * @return signals with unstable states cleared
   */
  [[nodiscard]] autoware_perception_msgs::msg::TrafficLightGroupArray filter_signals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray & signals,
    const rclcpp::Time & current_time, bool is_ego_stopped);

private:
  void cleanup_signal_history(const rclcpp::Time & current_time);

  struct SignalStateHistory
  {
    autoware_perception_msgs::msg::TrafficLightGroup msg;
    rclcpp::Time first_seen_time;
    rclcpp::Time last_seen_time;
  };

  StatusTrackerParameters params_;
  std::unordered_map<int64_t, SignalStateHistory> signal_history_;
};

}  // namespace autoware::traffic_light_compliance_checker

#endif  // AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__TRAFFIC_LIGHT_STATUS_TRACKER_HPP_
