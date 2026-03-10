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

#include "flashing_detection.hpp"

#include <algorithm>
#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{

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

bool is_skippable_signal(const TrafficSignal & signal)
{
  const auto & elements = signal.elements;
  if (elements.empty()) return true;
  // Occluded signal: UNKNOWN with confidence=0
  return elements.front().color == TrafficSignalElement::UNKNOWN &&
         elements.front().confidence == 0;
}

void FlashingDetector::update_signal_history(
  const TrafficSignal & signal, const rclcpp::Time & current_time)
{
  const auto id = signal.traffic_light_group_id;

  if (!is_skippable_signal(signal)) {
    signal_history_[id].push_back({signal, current_time});
  }

  remove_expired_entries(id, current_time);
}

void FlashingDetector::remove_expired_entries(lanelet::Id id, const rclcpp::Time & current_time)
{
  if (signal_history_.count(id) == 0) return;

  auto & history = signal_history_.at(id);
  const auto is_expired = [&](const TrafficSignalAndTime & entry) {
    return (current_time - entry.second).seconds() > config_.last_colors_hold_time;
  };
  history.erase(std::remove_if(history.begin(), history.end(), is_expired), history.end());

  if (history.empty()) {
    signal_history_.erase(id);
  }
}

void FlashingDetector::clear_state(lanelet::Id id)
{
  is_flashing_.erase(id);
  current_color_state_.erase(id);
  signal_history_.erase(id);
}

bool all_history_matches_color(
  const TrafficLightIdArray & signal_history, lanelet::Id id, uint8_t color)
{
  const auto history_iter = signal_history.find(id);
  if (history_iter == signal_history.end()) return false;
  const auto & history = history_iter->second;
  return std::all_of(history.begin(), history.end(), [color](const auto & entry) {
    return entry.first.elements.front().color == color;
  });
}

bool is_flashing_green_signal(const TrafficSignal & signal)
{
  return !signal.elements.empty() &&
         signal.elements.front().color == TrafficSignalElement::UNKNOWN &&
         signal.elements.front().confidence != 0;
}

void FlashingDetector::update_flashing_state(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;

  // no record of detected color in history
  const auto [_, inserted] = is_flashing_.try_emplace(id, false);
  if (inserted) {
    return;
  }

  // reset flashing if history contains only UNKNOWN entries (no evidence of prior color)
  if (all_history_matches_color(signal_history_, id, TrafficSignalElement::UNKNOWN)) {
    is_flashing_.at(id) = false;
    return;
  }

  // flashing green: UNKNOWN color with non-zero confidence (not occlusion)
  if (
    is_flashing_green_signal(signal) &&
    current_color_state_.at(id) != TrafficSignalElement::UNKNOWN) {
    is_flashing_.at(id) = true;
    return;
  }

  // check history: if all entries match current signal color, flashing has stopped
  if (all_history_matches_color(signal_history_, id, signal.elements.front().color)) {
    is_flashing_.at(id) = false;
  }
}

uint8_t resolve_flashing_color_transition(uint8_t current_state, uint8_t detected_color)
{
  if (current_state == TrafficSignalElement::GREEN && detected_color == TrafficSignalElement::RED) {
    return TrafficSignalElement::RED;
  }
  if (current_state == TrafficSignalElement::RED && detected_color == TrafficSignalElement::GREEN) {
    return TrafficSignalElement::GREEN;
  }
  if (current_state == TrafficSignalElement::UNKNOWN) {
    if (detected_color == TrafficSignalElement::RED) {
      return TrafficSignalElement::RED;
    }
    return TrafficSignalElement::GREEN;
  }
  return current_state;
}

uint8_t FlashingDetector::update_and_get_color_state(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;
  const auto color = signal.elements[0].color;

  // First observation: initialize with the detected color
  if (current_color_state_.count(id) == 0) {
    current_color_state_.emplace(id, color);
  } else if (!is_flashing_.at(id)) {
    // Not flashing: simply follow the detected color
    current_color_state_.at(id) = color;
  } else {
    current_color_state_.at(id) =
      resolve_flashing_color_transition(current_color_state_.at(id), color);
  }

  return current_color_state_.at(id);
}

}  // namespace autoware::crosswalk_traffic_light_estimator
