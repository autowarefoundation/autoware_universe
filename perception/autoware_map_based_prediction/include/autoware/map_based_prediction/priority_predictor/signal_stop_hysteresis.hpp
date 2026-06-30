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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PRIORITY_PREDICTOR__SIGNAL_STOP_HYSTERESIS_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PRIORITY_PREDICTOR__SIGNAL_STOP_HYSTERESIS_HPP_

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>

#include <lanelet2_core/Forward.h>

#include <cstddef>
#include <optional>
#include <unordered_map>

namespace autoware::map_based_prediction::priority_predictor
{
using autoware_perception_msgs::msg::TrafficLightGroup;

struct SignalStabilizeState
{
  TrafficLightGroup last_stable_signal;
  bool is_confirmed_stop{false};
  bool is_confirmed_go{false};
  std::optional<rclcpp::Time> stop_start_time{};
  std::optional<rclcpp::Time> go_start_time{};
  std::optional<rclcpp::Time> last_observed_time{};
};

std::unordered_map<lanelet::Id, TrafficLightGroup> stabilizeTrafficSignalMap(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & raw_traffic_signal_map,
  std::unordered_map<lanelet::Id, SignalStabilizeState> & signal_stabilize_states_histories,
  const rclcpp::Time & observe_time, double stop_time_hysteresis, double go_time_hysteresis,
  double retention_timeout);

void debounceObservedSignals(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & raw_traffic_signal_map,
  std::unordered_map<lanelet::Id, SignalStabilizeState> & signal_stabilize_states_histories,
  std::unordered_map<lanelet::Id, TrafficLightGroup> & stabilized_traffic_signal_map,
  const rclcpp::Time & observe_time, double stop_time_hysteresis, double go_time_hysteresis);

bool hasKnownColor(const TrafficLightGroup & traffic_signal_group);

TrafficLightGroup debounceSignalGroup(
  SignalStabilizeState & signal_stabilize_state_history,
  const TrafficLightGroup & raw_traffic_signal_group, const rclcpp::Time & observe_time,
  double stop_time_hysteresis, double go_time_hysteresis);

bool showsRedOrAmberCircle(const TrafficLightGroup & traffic_signal_group);

bool debounceStopDecision(
  SignalStabilizeState & signal_stabilize_state_history, bool is_traffic_signal_stop,
  const rclcpp::Time & observe_time, double stop_time_hysteresis, double go_time_hysteresis);

void armObservedColorTimer(
  SignalStabilizeState & signal_stabilize_state_history, bool is_traffic_signal_stop,
  const rclcpp::Time & observe_time);

void commitDecisionWhenHeldLongEnough(
  SignalStabilizeState & signal_stabilize_state_history, bool is_traffic_signal_stop,
  const rclcpp::Time & observe_time, double stop_time_hysteresis, double go_time_hysteresis);

bool shouldAdoptRawGroup(bool is_traffic_signal_stop, bool is_confirmed_stop, bool is_confirmed_go);

void retainUnobservedSignalsWithinTimeout(
  std::unordered_map<lanelet::Id, SignalStabilizeState> & signal_stabilize_states_histories,
  std::unordered_map<lanelet::Id, TrafficLightGroup> & stabilized_traffic_signal_map,
  const rclcpp::Time & observe_time, double retention_timeout);

}  // namespace autoware::map_based_prediction::priority_predictor

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PRIORITY_PREDICTOR__SIGNAL_STOP_HYSTERESIS_HPP_
