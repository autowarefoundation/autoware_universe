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

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

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
   * @return signals with stable states (or raw signals if ego is stopped)
   */
  [[nodiscard]] autoware_perception_msgs::msg::TrafficLightGroupArray filter_signals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray & signals,
    const rclcpp::Time & current_time, bool is_ego_stopped);

  /**
   * @brief return how the current amber phase was reached for a traffic light group
   * @param traffic_light_group_id traffic light regulatory element / group id
   * @return amber transition state; kNotAmber if the id is unknown or not amber
   */
  [[nodiscard]] AmberState get_amber_transition_state(int64_t traffic_light_group_id) const;

  [[nodiscard]] double get_duration(const int64_t traffic_light_id) const
  {
    const auto it = signal_history_.find(traffic_light_id);
    if (it == signal_history_.end()) return 0.0;
    return (it->second.last_seen_time - it->second.first_seen_time).seconds();
  }

private:
  struct SignalStateHistory
  {
    autoware_perception_msgs::msg::TrafficLightGroup current_state;
    std::optional<autoware_perception_msgs::msg::TrafficLightGroup> stable_state;
    rclcpp::Time first_seen_time;
    rclcpp::Time last_seen_time;
    AmberState amber_transition_state{AmberState::kNotAmber};
  };

  void cleanup_signal_history(const rclcpp::Time & current_time);

  static void update_amber_transition_state(
    SignalStateHistory & history,
    const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & previous_elements,
    const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & current_elements);

  double required_stability_duration(
    const autoware_perception_msgs::msg::TrafficLightGroup & signal) const;

  StatusTrackerParameters params_;
  std::unordered_map<int64_t, SignalStateHistory> signal_history_;
};

}  // namespace autoware::traffic_light_compliance_checker

#endif  // AUTOWARE__TRAFFIC_LIGHT_COMPLIANCE_CHECKER__TRAFFIC_LIGHT_STATUS_TRACKER_HPP_
