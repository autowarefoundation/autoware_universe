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
    auto history_it = signal_history_.find(id);
    if (history_it == signal_history_.end()) {
      SignalStateHistory signal_state{
        signal, std::nullopt, current_time, current_time, YellowState::kNotYellow};
      history_it = signal_history_.insert({id, signal_state}).first;
    } else {
      auto & history = history_it->second;
      if (!is_equal(history.current_state.elements, signal.elements)) {
        history.first_seen_time = current_time;
        history.current_state = signal;
      }
      history.last_seen_time = current_time;
    }

    const auto state_duration = (current_time - history_it->second.first_seen_time).seconds();

    if (state_duration >= required_stability_duration(signal)) {
      auto stable_changed = !history_it->second.stable_state ||
                            !is_equal(history_it->second.stable_state->elements, signal.elements);
      if (stable_changed && history_it->second.stable_state) {
        update_yellow_transition_state(
          history_it->second, history_it->second.stable_state->elements, signal.elements);
      }
      history_it->second.stable_state = signal;
    }

    if (is_ego_stopped) {
      filtered_signals.traffic_light_groups.push_back(signal);
    } else if (history_it->second.stable_state) {
      filtered_signals.traffic_light_groups.push_back(*history_it->second.stable_state);
    }
  }

  cleanup_signal_history(current_time);

  return filtered_signals;
}

void TrafficLightStatusTracker::cleanup_signal_history(const rclcpp::Time & current_time)
{
  for (auto it = signal_history_.begin(); it != signal_history_.end();) {
    const double stable_duration = required_stability_duration(it->second.current_state);
    if ((current_time - it->second.last_seen_time).seconds() > stable_duration) {
      it = signal_history_.erase(it);
    } else {
      ++it;
    }
  }
}

double TrafficLightStatusTracker::required_stability_duration(
  const autoware_perception_msgs::msg::TrafficLightGroup & signal) const
{
  if (has_red_circle(signal.elements)) {
    return params_.stable_duration_threshold_red;
  }
  if (has_amber_circle(signal.elements)) {
    return params_.stable_duration_threshold_amber;
  }
  if (has_unknown(signal.elements)) {
    return params_.stable_duration_threshold_unknown;
  }
  return 0.0;
}
}  // namespace autoware::traffic_light_compliance_checker
