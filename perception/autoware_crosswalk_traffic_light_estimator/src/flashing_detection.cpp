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

#include "autoware_crosswalk_traffic_light_estimator/flashing_detection.hpp"

#include <algorithm>
#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{

bool is_skippable_signal(const TrafficSignal & signal)
{
  const auto & elements = signal.elements;
  if (elements.empty()) return true;
  // Occluded signal: UNKNOWN with confidence=1
  return elements.front().color == TrafficSignalElement::UNKNOWN &&
         elements.front().confidence == 1;
}

FlashingDetector::FlashingDetector(const FlashingDetectionConfig & config) : config_(config)
{
}

uint8_t FlashingDetector::estimate_stable_color(
  const TrafficSignal & signal, const rclcpp::Time & current_time)
{
  update_signal_history(signal, current_time);
  update_flashing_state(signal);
  return update_and_get_color_state(signal);
}

void FlashingDetector::update_signal_history(
  const TrafficSignal & signal, const rclcpp::Time & current_time)
{
  const auto id = signal.traffic_light_group_id;

  if (!is_skippable_signal(signal)) {
    signal_history_[id].push_back({signal, current_time});
  }

  auto history_iter = signal_history_.find(id);
  if (history_iter == signal_history_.end()) return;

  auto & history = history_iter->second;
  history.erase(
    std::remove_if(
      history.begin(), history.end(),
      [&](const TrafficSignalAndTime & entry) {
        return (current_time - entry.second).seconds() > config_.last_colors_hold_time;
      }),
    history.end());

  if (history.empty()) {
    signal_history_.erase(history_iter);
  }
}

void FlashingDetector::clear_state(lanelet::Id id)
{
  is_flashing_.erase(id);
  current_color_state_.erase(id);
  signal_history_.erase(id);
}

void FlashingDetector::update_flashing_state(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;

  // no record of detected color in history
  if (is_flashing_.count(id) == 0) {
    is_flashing_.emplace(id, false);
    return;
  }

  // flashing green: UNKNOWN color with non-zero confidence (not occlusion)
  if (
    !signal.elements.empty() && signal.elements.front().color == TrafficSignalElement::UNKNOWN &&
    signal.elements.front().confidence != 0 &&
    current_color_state_.at(id) != TrafficSignalElement::UNKNOWN) {
    is_flashing_.at(id) = true;
    return;
  }

  // check history: if all entries match current signal color, flashing has stopped
  if (signal_history_.count(id) > 0) {
    std::vector<TrafficSignalAndTime> history = signal_history_.at(id);
    for (const auto & history_entry : history) {
      if (history_entry.first.elements.front().color != signal.elements.front().color) {
        return;
      }
    }
    is_flashing_.at(id) = false;
  }
}

uint8_t FlashingDetector::update_and_get_color_state(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;
  const auto color = signal.elements[0].color;

  // First observation: initialize with the detected color
  if (current_color_state_.count(id) == 0) {
    current_color_state_.emplace(id, color);
  } else if (is_flashing_.at(id) == false) {
    // Not flashing: simply follow the detected color
    current_color_state_.at(id) = color;
  } else if (is_flashing_.at(id) == true) {
    // During flashing: only update the color on definitive transitions.
    // While flashing, UNKNOWN detections appear intermittently between real colors,
    // so we hold the last known color and only transition on clear color changes.
    if (
      current_color_state_.at(id) == TrafficSignalElement::GREEN &&
      color == TrafficSignalElement::RED) {
      // Flashing green ended, now red
      current_color_state_.at(id) = TrafficSignalElement::RED;
    } else if (
      current_color_state_.at(id) == TrafficSignalElement::RED &&
      color == TrafficSignalElement::GREEN) {
      // Red ended, now green
      current_color_state_.at(id) = TrafficSignalElement::GREEN;
    } else if (current_color_state_.at(id) == TrafficSignalElement::UNKNOWN) {
      // Previous state is unknown: resolve to a concrete color once detected
      if (color == TrafficSignalElement::GREEN || color == TrafficSignalElement::UNKNOWN)
        current_color_state_.at(id) = TrafficSignalElement::GREEN;
      if (color == TrafficSignalElement::RED)
        current_color_state_.at(id) = TrafficSignalElement::RED;
    }
  }

  return current_color_state_.at(id);
}

}  // namespace autoware::crosswalk_traffic_light_estimator
