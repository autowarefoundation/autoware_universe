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
#include "autoware/traffic_light_compliance_checker/utils.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>

#include <vector>

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

YellowState TrafficLightStatusTracker::get_yellow_transition_state(
  const int64_t traffic_light_group_id) const
{
  const auto it = signal_history_.find(traffic_light_group_id);
  if (it == signal_history_.end()) {
    return YellowState::kNotYellow;
  }
  return it->second.yellow_transition_state;
}

void TrafficLightStatusTracker::update_yellow_transition_state(
  SignalStateHistory & history,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & previous_elements,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & current_elements)
{
  const bool is_amber_now = has_amber_circle(current_elements);
  if (!is_amber_now) {
    history.yellow_transition_state = YellowState::kNotYellow;
    return;
  }

  // Only classify the transition on the first frame of amber.
  if (history.yellow_transition_state != YellowState::kNotYellow) {
    return;
  }

  if (previous_elements.empty()) {
    // Origin unknown; keep kNotYellow so arrow-aware pass does not apply.
    return;
  }

  if (has_green_circle(previous_elements)) {
    history.yellow_transition_state = YellowState::kFromGreen;
  } else {
    history.yellow_transition_state = YellowState::kFromNonGreen;
  }
}

autoware_perception_msgs::msg::TrafficLightGroupArray TrafficLightStatusTracker::filter_signals(
  const autoware_perception_msgs::msg::TrafficLightGroupArray & signals,
  const rclcpp::Time & current_time, const bool is_ego_stopped)
{
  autoware_perception_msgs::msg::TrafficLightGroupArray filtered_signals;
  filtered_signals.stamp = signals.stamp;

  for (const auto & signal : signals.traffic_light_groups) {
    const auto id = signal.traffic_light_group_id;
    std::vector<autoware_perception_msgs::msg::TrafficLightElement> previous_elements;
    if (signal_history_.find(id) == signal_history_.end()) {
      signal_history_[id] = {signal, current_time, current_time, YellowState::kNotYellow};
    } else {
      previous_elements = signal_history_[id].msg.elements;
      if (!is_equal(previous_elements, signal.elements)) {
        signal_history_[id].first_seen_time = current_time;
        signal_history_[id].msg = signal;
      }
      signal_history_[id].last_seen_time = current_time;
    }
    // Classify from raw elements before stability filtering clears amber.
    update_yellow_transition_state(signal_history_[id], previous_elements, signal.elements);

    auto filtered_signal = signal;
    if (is_ego_stopped) {
      filtered_signals.traffic_light_groups.push_back(filtered_signal);
      continue;
    }
    const auto state_duration = (current_time - signal_history_[id].first_seen_time).seconds();
    const bool is_red = has_red_circle(signal.elements);
    const bool is_amber = has_amber_circle(signal.elements);
    const bool is_unknown = has_unknown(signal.elements);

    if (is_red && state_duration < params_.stable_duration_threshold_red) {
      filtered_signal.elements.clear();
    } else if (is_amber && state_duration < params_.stable_duration_threshold_amber) {
      filtered_signal.elements.clear();
    } else if (is_unknown && state_duration < params_.stable_duration_threshold_unknown) {
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
    const bool is_red = has_red_circle(it->second.msg.elements);
    const bool is_amber = has_amber_circle(it->second.msg.elements);
    const double stable_duration = is_red     ? params_.stable_duration_threshold_red
                                   : is_amber ? params_.stable_duration_threshold_amber
                                              : params_.stable_duration_threshold_unknown;
    if ((current_time - it->second.last_seen_time).seconds() > stable_duration) {
      it = signal_history_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace autoware::traffic_light_compliance_checker
