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

#include "autoware/traffic_light_compliance_checker/traffic_light_status_tracker.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>

#include <vector>

namespace
{
bool is_equal(
  const autoware_perception_msgs::msg::TrafficLightElement & a,
  const autoware_perception_msgs::msg::TrafficLightElement & b)
{
  return a.color == b.color && a.shape == b.shape && a.status == b.status;
}

bool is_equal(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & a,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & b)
{
  if (a.size() != b.size()) {
    return false;
  }

  for (size_t i = 0; i < a.size(); ++i) {
    if (!is_equal(a[i], b[i])) {
      return false;
    }
  }

  return true;
}
}  // namespace

namespace autoware::traffic_light_compliance_checker
{

TrafficLightStatusTracker::TrafficLightStatusTracker(const StatusTrackerParameters & parameters)
: params_(parameters)
{
}

void TrafficLightStatusTracker::update_parameters(const StatusTrackerParameters & parameters)
{
  params_ = parameters;
}

autoware_perception_msgs::msg::TrafficLightGroupArray TrafficLightStatusTracker::filter_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & signals,
  const rclcpp::Time & current_time, const bool is_ego_stopped)
{
  autoware_perception_msgs::msg::TrafficLightGroupArray filtered_signals;
  filtered_signals.stamp = signals.stamp;

  for (const auto & signal : signals.traffic_light_groups) {
    const auto id = signal.traffic_light_group_id;
    if (signal_history_.find(id) == signal_history_.end()) {
      signal_history_[id] = {signal, current_time, current_time};
    } else {
      if (!is_equal(signal_history_[id].msg.elements, signal.elements)) {
        signal_history_[id].first_seen_time = current_time;
        signal_history_[id].msg = signal;
      }
      signal_history_[id].last_seen_time = current_time;
    }

    auto filtered_signal = signal;
    if (is_ego_stopped) {
      filtered_signals.traffic_light_groups.push_back(filtered_signal);
      continue;
    }
    const auto state_duration = (current_time - signal_history_[id].first_seen_time).seconds();
    const bool is_red = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::RED);
    const bool is_amber = autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      signal.elements, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::msg::TrafficLightElement::AMBER);

    if (is_red && state_duration < params_.stable_duration_threshold_red) {
      filtered_signal.elements.clear();
    } else if (is_amber && state_duration < params_.stable_duration_threshold_amber) {
      filtered_signal.elements.clear();
    }
    filtered_signals.traffic_light_groups.push_back(filtered_signal);
  }

  cleanup_signal_history(current_time);

  return filtered_signals;
}

void TrafficLightStatusTracker::cleanup_signal_history(const rclcpp::Time & current_time)
{
  for (auto it = signal_history_.begin(); it != signal_history_.end();) {
    const auto stable_duration =
      autoware::traffic_light_utils::hasTrafficLightColor(
        it->second.msg.elements, autoware_perception_msgs::msg::TrafficLightElement::RED)
        ? params_.stable_duration_threshold_red
        : params_.stable_duration_threshold_amber;
    if ((current_time - it->second.last_seen_time).seconds() > stable_duration) {
      it = signal_history_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace autoware::traffic_light_compliance_checker
