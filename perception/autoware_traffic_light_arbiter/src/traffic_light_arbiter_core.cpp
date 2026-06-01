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

#include <autoware/traffic_light_arbiter/traffic_light_arbiter_core.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

TrafficLightArbiterCore::TrafficLightArbiterCore(
  SourcePriority source_priority, bool enable_signal_matching, double external_delay_tolerance,
  double external_time_tolerance, double perception_time_tolerance)
: source_priority_(source_priority),
  enable_signal_matching_(enable_signal_matching),
  external_delay_tolerance_(external_delay_tolerance),
  external_time_tolerance_(external_time_tolerance),
  perception_time_tolerance_(perception_time_tolerance)
{
  if (enable_signal_matching_) {
    signal_match_validator_ = std::make_unique<SignalMatchValidator>();
    signal_match_validator_->set_source_priority(source_priority_);
  }
}

void TrafficLightArbiterCore::set_traffic_light_ids(std::unordered_set<lanelet::Id> ids)
{
  map_regulatory_elements_set_ = std::make_unique<std::unordered_set<lanelet::Id>>(std::move(ids));
}

void TrafficLightArbiterCore::set_pedestrian_traffic_light_ids(std::unordered_set<lanelet::Id> ids)
{
  if (enable_signal_matching_) {
    signal_match_validator_->set_pedestrian_traffic_light_ids(std::move(ids));
  }
}

bool TrafficLightArbiterCore::is_external_outdated(
  const rclcpp::Time & current_time, const rclcpp::Time & msg_stamp) const
{
  return std::abs((current_time - msg_stamp).seconds()) > external_delay_tolerance_;
}

std::vector<TrafficLightArbiterCore::DroppedExternalSignal>
TrafficLightArbiterCore::sweep_expired_external_signals(
  const rclcpp::Time & reference_time, double tolerance)
{
  std::vector<DroppedExternalSignal> dropped;
  auto it = external_traffic_lights_.begin();
  while (it != external_traffic_lights_.end()) {
    const auto & msg_stamp = it->second.first;
    const auto age = (reference_time - msg_stamp).seconds();
    if (std::abs(age) > tolerance) {
      dropped.push_back({it->first, age});
      it = external_traffic_lights_.erase(it);
    } else {
      ++it;
    }
  }
  return dropped;
}

std::vector<TrafficLightArbiterCore::DroppedExternalSignal>
TrafficLightArbiterCore::ingest_perception(const TrafficSignalArray & msg)
{
  latest_perception_msg_ = msg;
  return sweep_expired_external_signals(rclcpp::Time(msg.stamp), external_time_tolerance_);
}

TrafficLightArbiterCore::ExternalIngestResult TrafficLightArbiterCore::ingest_external(
  const TrafficSignalArray & msg, const rclcpp::Time & current_time)
{
  const auto msg_time = rclcpp::Time(msg.stamp);
  if (is_external_outdated(current_time, msg_time)) {
    return {false, {}};
  }

  // Update external traffic lights map with new information
  for (const auto & signal : msg.traffic_light_groups) {
    external_traffic_lights_[signal.traffic_light_group_id] = std::make_pair(msg_time, signal);
  }

  // Clear perception data if too old relative to this external signal
  if (
    std::abs((msg_time - rclcpp::Time(latest_perception_msg_.stamp)).seconds()) >
    perception_time_tolerance_) {
    latest_perception_msg_.traffic_light_groups.clear();
  }

  return {true, sweep_expired_external_signals(current_time, external_delay_tolerance_)};
}

TrafficLightArbiterCore::ArbitrationResult TrafficLightArbiterCore::arbitrate()
{
  ArbitrationResult result;

  using ElementAndPriority = std::pair<Element, bool>;
  std::unordered_map<lanelet::Id, std::vector<ElementAndPriority>> regulatory_element_signals_map;

  // Create external signals array from stored valid signals
  TrafficSignalArray valid_external_signals;
  for (const auto & [id, info] : external_traffic_lights_) {
    valid_external_signals.traffic_light_groups.emplace_back(info.second);
  }

  auto append_predictions = [](auto & map, const auto & groups) {
    for (const auto & group : groups) {
      auto & predictions = map[group.traffic_light_group_id];
      predictions.insert(predictions.end(), group.predictions.begin(), group.predictions.end());
    }
  };
  std::unordered_map<lanelet::Id, std::vector<PredictedTrafficLightState>> predictions_map;
  // add in order from perception msg
  append_predictions(predictions_map, latest_perception_msg_.traffic_light_groups);
  append_predictions(predictions_map, valid_external_signals.traffic_light_groups);

  if (map_regulatory_elements_set_ == nullptr) {
    return result;
  }

  TrafficSignalArray output_signals_msg;
  // stamp deliberately left default — the Node owns stamp inheritance.

  if (map_regulatory_elements_set_->empty()) {
    result.output = std::move(output_signals_msg);
    return result;
  }

  auto add_signal_function = [&](const auto & signal, bool priority) {
    const auto id = signal.traffic_light_group_id;
    if (!map_regulatory_elements_set_->count(id)) {
      result.off_map_signal_ids.push_back(id);
      return;
    }

    auto & elements_and_priority = regulatory_element_signals_map[id];
    for (const auto & element : signal.elements) {
      elements_and_priority.emplace_back(element, priority);
    }
  };

  if (enable_signal_matching_) {
    const auto validated_signals =
      signal_match_validator_->validate_signals(latest_perception_msg_, valid_external_signals);
    for (const auto & signal : validated_signals.traffic_light_groups) {
      add_signal_function(signal, false);
    }
  } else {
    for (const auto & signal : latest_perception_msg_.traffic_light_groups) {
      add_signal_function(signal, source_priority_ == SourcePriority::PERCEPTION);
    }

    for (const auto & signal : valid_external_signals.traffic_light_groups) {
      add_signal_function(signal, source_priority_ == SourcePriority::EXTERNAL);
    }
  }

  const auto get_highest_confidence_elements =
    [](const std::vector<ElementAndPriority> & elements_and_priority_vector) {
      using Key = Element::_shape_type;
      std::map<Key, ElementAndPriority> highest_score_element_and_priority_map;
      std::vector<Element> highest_score_elements_vector;

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
    };

  output_signals_msg.traffic_light_groups.reserve(regulatory_element_signals_map.size());

  for (const auto & [regulatory_element_id, elements] : regulatory_element_signals_map) {
    TrafficSignal signal_msg;
    signal_msg.traffic_light_group_id = regulatory_element_id;
    signal_msg.elements = get_highest_confidence_elements(elements);
    signal_msg.predictions = predictions_map[regulatory_element_id];
    output_signals_msg.traffic_light_groups.emplace_back(signal_msg);
  }

  // Compute latest input time across stored sources. The Node compares this
  // against its trigger stamp to decide whether the published output is
  // behind some input that has arrived but hasn't yet driven a publish cycle.
  rclcpp::Time latest_time = rclcpp::Time(latest_perception_msg_.stamp);
  for (const auto & [id, info] : external_traffic_lights_) {
    const auto & external_time = info.first;
    if (external_time > latest_time) {
      latest_time = external_time;
    }
  }
  result.latest_input_time = latest_time;

  result.output = std::move(output_signals_msg);
  return result;
}

}  // namespace autoware::traffic_light
