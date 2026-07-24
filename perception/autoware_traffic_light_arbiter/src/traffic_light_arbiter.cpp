// Copyright 2026 The Autoware Contributors
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

#include <autoware/traffic_light_arbiter/traffic_light_arbiter.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

namespace
{

using autoware_perception_msgs::msg::PredictedTrafficLightState;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

// One element paired with whether it came from the prioritized source.
// Accumulated per regulatory-element id while routing signals, then reduced to
// one element per shape by get_highest_confidence_elements().
using ElementAndPriority = std::pair<TrafficLightElement, bool>;

// Snapshot of the stored external signals plus the freshest stamp among them.
// The perception-staleness check and latest_input_time both derive from
// max_stamp/has_any, so they read from one source of truth.
struct ExternalSnapshot
{
  TrafficLightGroupArray signals;
  rclcpp::Time max_stamp{0, 0, RCL_ROS_TIME};
  bool has_any = false;
};

// Builds the external-signal array and tracks the freshest stamp in one pass
// over the per-id external cache.
ExternalSnapshot collect_external_snapshot(
  const std::unordered_map<lanelet::Id, std::pair<rclcpp::Time, TrafficLightGroup>> &
    external_traffic_lights)
{
  ExternalSnapshot snapshot;
  for (const auto & [id, info] : external_traffic_lights) {
    snapshot.signals.traffic_light_groups.emplace_back(info.second);
    if (!snapshot.has_any || info.first > snapshot.max_stamp) {
      snapshot.max_stamp = info.first;
      snapshot.has_any = true;
    }
  }
  return snapshot;
}

// Applies the perception staleness gate. Returns the stored perception snapshot
// to use this cycle, or an empty stand-in carrying the same stamp when
// perception lags the freshest external by more than perception_time_tolerance.
// Non-destructive: the stored perception is never modified, so
// ingest_perception() stays its sole writer.
TrafficLightGroupArray select_effective_perception(
  const TrafficLightGroupArray & perception, const ExternalSnapshot & external,
  double perception_time_tolerance)
{
  const auto perception_stamp = rclcpp::Time(perception.stamp);
  const bool perception_is_stale =
    external.has_any &&
    (external.max_stamp - perception_stamp).seconds() > perception_time_tolerance;
  if (!perception_is_stale) {
    return perception;
  }

  TrafficLightGroupArray empty_perception;
  empty_perception.stamp = perception.stamp;
  return empty_perception;
}

// Appends each group's predictions into predictions_map keyed by
// regulatory-element id. Predictions accumulate across sources in call order.
void append_predictions(
  std::unordered_map<lanelet::Id, std::vector<PredictedTrafficLightState>> & predictions_map,
  const std::vector<TrafficLightGroup> & groups)
{
  for (const auto & group : groups) {
    auto & predictions = predictions_map[group.traffic_light_group_id];
    predictions.insert(predictions.end(), group.predictions.begin(), group.predictions.end());
  }
}

// Routes one signal's elements into `signals_map` under its regulatory-element
// id, tagging each with `priority`. Ids absent from `map_regulatory_elements`
// are recorded in `off_map_signal_ids` and dropped.
void route_signal(
  const TrafficLightGroup & signal, bool priority,
  const std::unordered_set<lanelet::Id> & map_regulatory_elements,
  std::unordered_map<lanelet::Id, std::vector<ElementAndPriority>> & signals_map,
  std::vector<lanelet::Id> & off_map_signal_ids)
{
  const auto id = signal.traffic_light_group_id;
  if (!map_regulatory_elements.count(id)) {
    off_map_signal_ids.push_back(id);
    return;
  }

  auto & elements_and_priority = signals_map[id];
  for (const auto & element : signal.elements) {
    elements_and_priority.emplace_back(element, priority);
  }
}

// Routes every signal in `signals` into `signals_map` via route_signal().
void route_signals(
  const std::vector<TrafficLightGroup> & signals, bool priority,
  const std::unordered_set<lanelet::Id> & map_regulatory_elements,
  std::unordered_map<lanelet::Id, std::vector<ElementAndPriority>> & signals_map,
  std::vector<lanelet::Id> & off_map_signal_ids)
{
  for (const auto & signal : signals) {
    route_signal(signal, priority, map_regulatory_elements, signals_map, off_map_signal_ids);
  }
}

// Reduces the accumulated (element, priority) pairs for one regulatory element
// to a single element per shape, keeping the highest (priority, confidence).
std::vector<TrafficLightElement> get_highest_confidence_elements(
  const std::vector<ElementAndPriority> & elements_and_priority_vector)
{
  using Key = TrafficLightElement::_shape_type;
  std::map<Key, ElementAndPriority> highest_score_element_and_priority_map;
  std::vector<TrafficLightElement> highest_score_elements_vector;

  for (const auto & elements_and_priority : elements_and_priority_vector) {
    const auto & element = elements_and_priority.first;
    const auto & element_priority = elements_and_priority.second;
    const auto key = element.shape;
    auto [iter, success] =
      highest_score_element_and_priority_map.try_emplace(key, elements_and_priority);
    const auto & iter_element = iter->second.first;
    const auto & iter_priority = iter->second.second;

    if (
      !success &&
      (element_priority > iter_priority ||
       (element_priority == iter_priority && element.confidence > iter_element.confidence))) {
      iter->second = elements_and_priority;
    }
  }

  for (const auto & [k, v] : highest_score_element_and_priority_map) {
    highest_score_elements_vector.emplace_back(v.first);
  }

  return highest_score_elements_vector;
}

std::unordered_set<lanelet::Id> extract_traffic_light_ids(const lanelet::LaneletMapConstPtr & map)
{
  std::unordered_set<lanelet::Id> traffic_light_ids;
  for (const auto & element : map->regulatoryElementLayer) {
    const auto traffic_light = std::dynamic_pointer_cast<const lanelet::TrafficLight>(element);
    if (traffic_light) {
      traffic_light_ids.emplace(traffic_light->id());
    }
  }
  return traffic_light_ids;
}

std::unordered_set<lanelet::Id> extract_pedestrian_traffic_light_ids(
  const lanelet::LaneletMapConstPtr & map)
{
  namespace query = lanelet::utils::query;

  const auto all_lanelets = query::laneletLayer(map);
  const auto crosswalks = query::crosswalkLanelets(all_lanelets);

  std::unordered_set<lanelet::Id> pedestrian_traffic_light_ids;
  for (const auto & crosswalk : crosswalks) {
    const auto traffic_lights = crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();
    for (const auto & traffic_light : traffic_lights) {
      pedestrian_traffic_light_ids.emplace(traffic_light->id());
    }
  }
  return pedestrian_traffic_light_ids;
}

}  // namespace

TrafficLightArbiter::TrafficLightArbiter(
  SourcePriority source_priority, bool enable_signal_matching, double external_delay_tolerance,
  double external_time_tolerance, double perception_time_tolerance)
: source_priority_(source_priority),
  external_delay_tolerance_(external_delay_tolerance),
  external_time_tolerance_(external_time_tolerance),
  perception_time_tolerance_(perception_time_tolerance)
{
  if (enable_signal_matching) {
    signal_match_validator_ = std::make_unique<SignalMatchValidator>(source_priority_);
  }
}

void TrafficLightArbiter::set_map(const lanelet::LaneletMapConstPtr & map)
{
  map_regulatory_elements_set_ =
    std::make_unique<std::unordered_set<lanelet::Id>>(extract_traffic_light_ids(map));
  if (is_signal_matching_enabled()) {
    signal_match_validator_->set_pedestrian_traffic_light_ids(
      extract_pedestrian_traffic_light_ids(map));
  }
}

bool TrafficLightArbiter::is_external_outdated(
  const rclcpp::Time & current_time, const rclcpp::Time & msg_stamp) const
{
  return std::abs((current_time - msg_stamp).seconds()) > external_delay_tolerance_;
}

void TrafficLightArbiter::sweep_expired_external_signals(
  const rclcpp::Time & reference_time, double tolerance)
{
  auto it = external_traffic_lights_.begin();
  while (it != external_traffic_lights_.end()) {
    const auto & msg_stamp = it->second.first;
    if (std::abs((reference_time - msg_stamp).seconds()) > tolerance) {
      it = external_traffic_lights_.erase(it);
    } else {
      ++it;
    }
  }
}

void TrafficLightArbiter::ingest_perception(const TrafficSignalArray & msg)
{
  perception_traffic_light_ = msg;
  sweep_expired_external_signals(rclcpp::Time(msg.stamp), external_time_tolerance_);
}

// Admission control then cache maintenance:
//   1. Reject (return false) when |current_time - msg.stamp| exceeds
//      external_delay_tolerance_ — the arrival is too far off to trust.
//   2. Otherwise refresh each group's cache entry with msg.stamp and sweep the
//      cache against current_time (external_delay_tolerance_). Perception state
//      is untouched here; its staleness is handled non-destructively inside
//      arbitrate().
bool TrafficLightArbiter::ingest_external(
  const TrafficSignalArray & msg, const rclcpp::Time & current_time)
{
  const auto msg_time = rclcpp::Time(msg.stamp);
  if (is_external_outdated(current_time, msg_time)) {
    return false;
  }

  // Update external traffic lights map with new information
  for (const auto & signal : msg.traffic_light_groups) {
    external_traffic_lights_[signal.traffic_light_group_id] = std::make_pair(msg_time, signal);
  }

  sweep_expired_external_signals(current_time, external_delay_tolerance_);
  return true;
}

TrafficLightArbiter::ArbitrationResult TrafficLightArbiter::arbitrate() const
{
  ArbitrationResult result;

  std::unordered_map<lanelet::Id, std::vector<ElementAndPriority>> regulatory_element_signals_map;

  const auto external = collect_external_snapshot(external_traffic_lights_);

  const auto effective_perception =
    select_effective_perception(perception_traffic_light_, external, perception_time_tolerance_);

  std::unordered_map<lanelet::Id, std::vector<PredictedTrafficLightState>> predictions_map;
  // add in order from perception msg
  append_predictions(predictions_map, effective_perception.traffic_light_groups);
  append_predictions(predictions_map, external.signals.traffic_light_groups);

  if (map_regulatory_elements_set_ == nullptr) {
    return result;
  }

  TrafficSignalArray output_signals_msg;
  // stamp deliberately left default — the Node owns stamp inheritance.

  if (map_regulatory_elements_set_->empty()) {
    result.output = std::move(output_signals_msg);
    return result;
  }

  const auto & map_regulatory_elements = *map_regulatory_elements_set_;
  if (is_signal_matching_enabled()) {
    const auto validated_signals =
      signal_match_validator_->validate_signals(effective_perception, external.signals);
    route_signals(
      validated_signals.traffic_light_groups, false, map_regulatory_elements,
      regulatory_element_signals_map, result.off_map_signal_ids);
  } else {
    route_signals(
      effective_perception.traffic_light_groups, source_priority_ == SourcePriority::PERCEPTION,
      map_regulatory_elements, regulatory_element_signals_map, result.off_map_signal_ids);
    route_signals(
      external.signals.traffic_light_groups, source_priority_ == SourcePriority::EXTERNAL,
      map_regulatory_elements, regulatory_element_signals_map, result.off_map_signal_ids);
  }

  output_signals_msg.traffic_light_groups.reserve(regulatory_element_signals_map.size());

  for (const auto & [regulatory_element_id, elements] : regulatory_element_signals_map) {
    TrafficSignal signal_msg;
    signal_msg.traffic_light_group_id = regulatory_element_id;
    signal_msg.elements = get_highest_confidence_elements(elements);
    signal_msg.predictions = predictions_map[regulatory_element_id];
    output_signals_msg.traffic_light_groups.emplace_back(signal_msg);
  }

  // Latest input stamp across stored sources, for the Node's behind-input check.
  // perception_stamp stays valid even when perception was gated out (stand-in keeps it).
  const auto perception_stamp = rclcpp::Time(effective_perception.stamp);
  result.latest_input_time = (external.has_any && external.max_stamp > perception_stamp)
                               ? external.max_stamp
                               : perception_stamp;

  result.output = std::move(output_signals_msg);
  return result;
}

}  // namespace autoware::traffic_light
