// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/priority_predictor/signal_stop_hysteresis.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <cstddef>
#include <unordered_map>

namespace autoware::map_based_prediction::priority_predictor
{
using autoware_perception_msgs::msg::TrafficLightElement;

std::unordered_map<lanelet::Id, TrafficLightGroup> stabilizeTrafficSignalMap(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & raw_traffic_signal_map,
  std::unordered_map<lanelet::Id, SignalStabilizeState> & signal_stabilize_states_histories,
  const rclcpp::Time & observe_time, const double stop_time_hysteresis,
  const double go_time_hysteresis, const double retention_timeout)
{
  std::unordered_map<lanelet::Id, TrafficLightGroup> stabilized_traffic_signal_map;
  debounceObservedSignals(
    raw_traffic_signal_map, signal_stabilize_states_histories, stabilized_traffic_signal_map,
    observe_time, stop_time_hysteresis, go_time_hysteresis);
  retainUnobservedSignalsWithinTimeout(
    signal_stabilize_states_histories, stabilized_traffic_signal_map, observe_time,
    retention_timeout);
  return stabilized_traffic_signal_map;
}

void debounceObservedSignals(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & raw_traffic_signal_map,
  std::unordered_map<lanelet::Id, SignalStabilizeState> & signal_stabilize_states_histories,
  std::unordered_map<lanelet::Id, TrafficLightGroup> & stabilized_traffic_signal_map,
  const rclcpp::Time & observe_time, const double stop_time_hysteresis,
  const double go_time_hysteresis)
{
  for (const auto & [lanelet_id, raw_traffic_signal_group] : raw_traffic_signal_map) {
    if (!hasKnownColor(raw_traffic_signal_group)) {
      continue;
    }
    auto & signal_stabilize_state_history = signal_stabilize_states_histories[lanelet_id];

    const auto stabilized_traffic_signal_group = debounceSignalGroup(
      signal_stabilize_state_history, raw_traffic_signal_group, observe_time, stop_time_hysteresis,
      go_time_hysteresis);

    signal_stabilize_state_history.last_observed_time = observe_time;
    stabilized_traffic_signal_map[lanelet_id] = stabilized_traffic_signal_group;
  }
}

bool hasKnownColor(const TrafficLightGroup & traffic_signal_group)
{
  for (const auto & element : traffic_signal_group.elements) {
    if (element.color != TrafficLightElement::UNKNOWN) {
      return true;
    }
  }
  return false;
}

TrafficLightGroup debounceSignalGroup(
  SignalStabilizeState & signal_stabilize_state_history,
  const TrafficLightGroup & raw_traffic_signal_group, const rclcpp::Time & observe_time,
  const double stop_time_hysteresis, const double go_time_hysteresis)
{
  const bool is_first_observation = !signal_stabilize_state_history.stop_start_time.has_value() &&
                                    !signal_stabilize_state_history.go_start_time.has_value();
  const bool is_traffic_signal_stop = showsRedOrAmberCircle(raw_traffic_signal_group);
  debounceStopDecision(
    signal_stabilize_state_history, is_traffic_signal_stop, observe_time, stop_time_hysteresis,
    go_time_hysteresis);

  if (is_first_observation) {
    signal_stabilize_state_history.is_confirmed_stop = is_traffic_signal_stop;
    signal_stabilize_state_history.is_confirmed_go = !is_traffic_signal_stop;
  }
  if (shouldAdoptRawGroup(
        is_traffic_signal_stop, signal_stabilize_state_history.is_confirmed_stop,
        signal_stabilize_state_history.is_confirmed_go)) {
    signal_stabilize_state_history.last_stable_signal = raw_traffic_signal_group;
  }
  return signal_stabilize_state_history.last_stable_signal;
}

bool showsRedOrAmberCircle(const TrafficLightGroup & traffic_signal_group)
{
  for (const auto & element : traffic_signal_group.elements) {
    if (
      element.shape == TrafficLightElement::CIRCLE &&
      (element.color == TrafficLightElement::RED || element.color == TrafficLightElement::AMBER)) {
      return true;
    }
  }
  return false;
}

bool debounceStopDecision(
  SignalStabilizeState & signal_stabilize_state_history, const bool is_traffic_signal_stop,
  const rclcpp::Time & observe_time, const double stop_time_hysteresis,
  const double go_time_hysteresis)
{
  armObservedColorTimer(signal_stabilize_state_history, is_traffic_signal_stop, observe_time);
  commitDecisionWhenHeldLongEnough(
    signal_stabilize_state_history, is_traffic_signal_stop, observe_time, stop_time_hysteresis,
    go_time_hysteresis);
  return signal_stabilize_state_history.is_confirmed_stop;
}

void armObservedColorTimer(
  SignalStabilizeState & signal_stabilize_state_history, const bool is_traffic_signal_stop,
  const rclcpp::Time & observe_time)
{
  if (is_traffic_signal_stop) {
    signal_stabilize_state_history.go_start_time.reset();
    if (!signal_stabilize_state_history.stop_start_time) {
      signal_stabilize_state_history.stop_start_time = observe_time;
    }
  } else {
    signal_stabilize_state_history.stop_start_time.reset();
    if (!signal_stabilize_state_history.go_start_time) {
      signal_stabilize_state_history.go_start_time = observe_time;
    }
  }
}

void commitDecisionWhenHeldLongEnough(
  SignalStabilizeState & signal_stabilize_state_history, const bool is_traffic_signal_stop,
  const rclcpp::Time & observe_time, const double stop_time_hysteresis,
  const double go_time_hysteresis)
{
  if (is_traffic_signal_stop) {
    if (
      (observe_time - *signal_stabilize_state_history.stop_start_time).seconds() >=
      stop_time_hysteresis) {
      signal_stabilize_state_history.is_confirmed_stop = true;
      signal_stabilize_state_history.is_confirmed_go = false;
    }
  } else {
    if (
      (observe_time - *signal_stabilize_state_history.go_start_time).seconds() >=
      go_time_hysteresis) {
      signal_stabilize_state_history.is_confirmed_go = true;
      signal_stabilize_state_history.is_confirmed_stop = false;
    }
  }
}

bool shouldAdoptRawGroup(
  const bool is_traffic_signal_stop, const bool is_confirmed_stop, const bool is_confirmed_go)
{
  return (is_traffic_signal_stop && is_confirmed_stop) ||
         (!is_traffic_signal_stop && is_confirmed_go);
}

void retainUnobservedSignalsWithinTimeout(
  std::unordered_map<lanelet::Id, SignalStabilizeState> & signal_stabilize_states_histories,
  std::unordered_map<lanelet::Id, TrafficLightGroup> & stabilized_traffic_signal_map,
  const rclcpp::Time & observe_time, const double retention_timeout)
{
  for (auto state_it = signal_stabilize_states_histories.begin();
       state_it != signal_stabilize_states_histories.end();) {
    const auto & lanelet_id = state_it->first;
    const bool observed_this_frame = stabilized_traffic_signal_map.count(lanelet_id) != 0;
    if (observed_this_frame) {
      ++state_it;
      continue;
    }
    const auto & last_observed_time = state_it->second.last_observed_time;
    const bool retention_expired =
      retention_timeout <= 0.0 || !last_observed_time ||
      (observe_time - *last_observed_time).seconds() > retention_timeout;
    if (retention_expired) {
      state_it = signal_stabilize_states_histories.erase(state_it);
    } else {
      stabilized_traffic_signal_map[lanelet_id] = state_it->second.last_stable_signal;
      ++state_it;
    }
  }
}

}  // namespace autoware::map_based_prediction::priority_predictor
