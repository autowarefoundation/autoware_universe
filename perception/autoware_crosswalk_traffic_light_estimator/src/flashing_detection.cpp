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

#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{

FlashingDetector::FlashingDetector(const FlashingDetectionConfig & config) : config_(config)
{
}

void FlashingDetector::update_signal_history(
  const TrafficLightIdMap & traffic_light_id_map, const rclcpp::Time & current_time)
{
  // --- Append new detections to the history ---
  for (const auto & input_traffic_signal : traffic_light_id_map) {
    const auto & elements = input_traffic_signal.second.first.elements;

    // Skip signals with no detection data
    if (elements.empty()) {
      continue;
    }

    // Skip occluded signals (UNKNOWN with confidence=1 means the light is intentionally
    // classified as unknown due to occlusion, not a detection failure)
    if (
      elements.front().color == TrafficSignalElement::UNKNOWN && elements.front().confidence == 1) {
      continue;
    }

    const auto & id = input_traffic_signal.second.first.traffic_light_group_id;

    // Create a new history entry if this traffic light ID is seen for the first time
    if (signal_history_.count(id) == 0) {
      std::vector<TrafficSignalAndTime> signal{input_traffic_signal.second};
      signal_history_.emplace(id, signal);
      continue;
    }

    // Append to existing history
    signal_history_.at(id).push_back(input_traffic_signal.second);
  }

  // --- Remove stale entries that exceed the hold time ---
  std::vector<int32_t> erase_id_list;
  for (auto & last_traffic_signal : signal_history_) {
    const auto & id = last_traffic_signal.first;
    for (auto history_entry = last_traffic_signal.second.begin();
         history_entry != last_traffic_signal.second.end();) {
      const auto time_from_last_detected = (current_time - history_entry->second).seconds();
      if (time_from_last_detected > config_.last_colors_hold_time) {
        history_entry = last_traffic_signal.second.erase(history_entry);
      } else {
        ++history_entry;
      }
    }
    // Mark traffic light IDs whose history is now empty for removal
    if (last_traffic_signal.second.empty()) {
      erase_id_list.emplace_back(id);
    }
  }
  for (const auto id : erase_id_list) signal_history_.erase(id);
}

uint8_t FlashingDetector::estimate_stable_color(const TrafficSignal & signal)
{
  update_flashing_state(signal);
  return update_and_get_color_state(signal);
}

void FlashingDetector::clear_state(lanelet::Id id)
{
  is_flashing_.erase(id);
  current_color_state_.erase(id);
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
