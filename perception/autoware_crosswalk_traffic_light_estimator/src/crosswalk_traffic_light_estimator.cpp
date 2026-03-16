// Copyright 2022-2025 UCI SORA Lab, TIER IV, Inc.
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
#include "crosswalk_traffic_light_estimator.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <boost/optional.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{
namespace
{

bool has_merge_lane(
  const lanelet::ConstLanelet & lanelet_1, const lanelet::ConstLanelet & lanelet_2,
  const lanelet::routing::RoutingGraphPtr & routing_graph_ptr)
{
  const auto next_lanelets_1 = routing_graph_ptr->following(lanelet_1);
  const auto next_lanelets_2 = routing_graph_ptr->following(lanelet_2);

  for (const auto & next_lanelet_1 : next_lanelets_1) {
    for (const auto & next_lanelet_2 : next_lanelets_2) {
      if (next_lanelet_1.id() == next_lanelet_2.id()) {
        return true;
      }
    }
  }

  return false;
}

bool has_merge_lane(
  const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphPtr & routing_graph_ptr)
{
  for (size_t i = 0; i < lanelets.size(); ++i) {
    for (size_t j = i + 1; j < lanelets.size(); ++j) {
      const auto lanelet_1 = lanelets.at(i);
      const auto lanelet_2 = lanelets.at(j);

      if (lanelet_1.id() == lanelet_2.id()) {
        continue;
      }

      const std::string turn_direction_1 = lanelet_1.attributeOr("turn_direction", "none");
      const std::string turn_direction_2 = lanelet_2.attributeOr("turn_direction", "none");
      if (turn_direction_1 == turn_direction_2) {
        continue;
      }

      if (!has_merge_lane(lanelet_1, lanelet_2, routing_graph_ptr)) {
        continue;
      }

      return true;
    }
  }

  return false;
}

/// @brief convert a string to the corresponding traffic signal color
std::optional<uint8_t> str_to_color(std::string_view str)
{
  if (str == "red") {
    return TrafficSignalElement::RED;
  }
  if (str == "green") {
    return TrafficSignalElement::GREEN;
  }
  if (str == "amber") {
    return TrafficSignalElement::AMBER;
  }
  if (str == "white") {
    return TrafficSignalElement::WHITE;
  }
  return std::nullopt;
}

/// @brief parse the input string and extract a rule to estimate a traffic signal
/// @details the string is expected to have format "signal_color_relation:color1:color2"
std::optional<std::pair<uint8_t, uint8_t>> parse_signal_estimation_rules(std::string_view input)
{
  constexpr auto delimiter = ':';
  constexpr std::string_view prefix = "signal_color_relation:";
  if (input.size() < prefix.size() || input.substr(0, prefix.length()) != prefix) {
    return std::nullopt;
  }
  input.remove_prefix(prefix.length());

  // extract the color mapping
  const auto delimiter_pos = input.find(delimiter);
  if (delimiter_pos == std::string_view::npos) {
    return std::nullopt;
  }

  std::string_view from_str = input.substr(0, delimiter_pos);
  std::string_view to_str = input.substr(delimiter_pos + 1);

  if (const auto from_color = str_to_color(from_str)) {
    if (const auto to_color = str_to_color(to_str)) {
      return std::make_pair(*from_color, *to_color);
    }
  }
  return std::nullopt;
}

/// @brief extract ids from the input string
/// @details the string is expected to have format "id1,id2,...", without any space
lanelet::Ids parse_ids(std::string_view input)
{
  lanelet::Ids ids;
  if (input.empty()) {
    return ids;
  }

  constexpr auto delimiter = ',';
  size_t start = 0;
  size_t end = 0;
  lanelet::Id id{};
  while ((end = input.find(delimiter, start)) != std::string_view::npos) {
    const auto [_, err] = std::from_chars(input.data() + start, input.data() + end, id);
    if (err == std::errc()) {
      ids.push_back(id);
    }
    start = end + 1;
  }
  const auto [_, err] = std::from_chars(input.data() + start, input.data() + input.size(), id);
  if (err == std::errc()) {
    ids.push_back(id);
  }
  return ids;
}
bool is_invalid_detection_status(const TrafficSignal & signal)
{
  if (signal.elements.empty()) {
    return true;
  }
  if (
    signal.elements.front().color == TrafficSignalElement::UNKNOWN &&
    signal.elements.front().confidence == 0.0) {
    return true;
  }
  return false;
}

boost::optional<uint8_t> get_highest_confidence_traffic_signal(
  const lanelet::Id & id, const TrafficLightIdMap & traffic_light_id_map)
{
  boost::optional<uint8_t> ret{boost::none};

  double highest_confidence = 0.0;
  if (traffic_light_id_map.count(id) == 0) {
    return ret;
  }

  for (const auto & element : traffic_light_id_map.at(id).first.elements) {
    if (element.confidence < highest_confidence) {
      continue;
    }

    highest_confidence = element.confidence;
    ret = element.color;
  }

  return ret;
}

void remove_duplicate_ids(TrafficSignalArray & signal_array)
{
  auto & signals = signal_array.traffic_light_groups;
  std::stable_sort(signals.begin(), signals.end(), [](const auto & s1, const auto & s2) {
    return s1.traffic_light_group_id < s2.traffic_light_group_id;
  });

  signals.erase(
    std::unique(
      signals.begin(), signals.end(),
      [](const auto & s1, const auto s2) {
        return s1.traffic_light_group_id == s2.traffic_light_group_id;
      }),
    signals.end());
}

}  // namespace

CrosswalkTrafficLightEstimator::CrosswalkTrafficLightEstimator(
  const CrosswalkTrafficLightEstimatorConfig & config)
: config_(config), flashing_detector_(FlashingDetector(config.flashing_detection))
{
}

void CrosswalkTrafficLightEstimator::update_map(
  lanelet::LaneletMapPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  routing_graph_ptr_ = routing_graph_ptr;

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  const lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  // Build precomputed mappings: traffic light ID -> crosswalks, crosswalk -> vehicle lanelets
  traffic_light_id_to_crosswalks_.clear();
  crosswalk_to_vehicle_lanelets_.clear();

  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  const auto crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);

  const int VEHICLE_GRAPH_INDEX = 0;
  for (const auto & crosswalk : crosswalk_lanelets) {
    const auto traffic_light_reg_elems =
      crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();
    if (traffic_light_reg_elems.empty()) {
      continue;
    }
    const auto vehicle_lanelets = overall_graphs.conflictingInGraph(crosswalk, VEHICLE_GRAPH_INDEX);
    if (vehicle_lanelets.empty()) {
      continue;
    }

    crosswalk_to_vehicle_lanelets_[crosswalk.id()] = vehicle_lanelets;

    std::unordered_set<lanelet::Id> added_known_color_vehicle_traffic_light_ids;
    for (const auto & vehicle_lanelet : vehicle_lanelets) {
      for (const auto & traffic_light :
           vehicle_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>()) {
        if (added_known_color_vehicle_traffic_light_ids.insert(traffic_light->id()).second) {
          traffic_light_id_to_crosswalks_[traffic_light->id()].push_back(crosswalk);
        }
      }
    }
  }
}

bool CrosswalkTrafficLightEstimator::is_map_loaded() const
{
  return lanelet_map_ptr_ != nullptr;
}

std::vector<lanelet::Id> CrosswalkTrafficLightEstimator::find_unregistered_traffic_light_group_ids(
  const TrafficSignalArray & msg) const
{
  std::vector<lanelet::Id> unregistered_ids;
  for (const auto & traffic_signal : msg.traffic_light_groups) {
    const auto id = traffic_signal.traffic_light_group_id;
    if (
      lanelet_map_ptr_->regulatoryElementLayer.find(id) ==
      lanelet_map_ptr_->regulatoryElementLayer.end()) {
      unregistered_ids.push_back(id);
    }
  }
  return unregistered_ids;
}

void CrosswalkTrafficLightEstimator::update_crosswalk_overrides_from_map(
  std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides,
  lanelet::Id traffic_light_group_id, const TrafficLightIdMap & traffic_light_id_map)
{
  const auto traffic_light_it =
    lanelet_map_ptr_->regulatoryElementLayer.find(traffic_light_group_id);
  if (traffic_light_it == lanelet_map_ptr_->regulatoryElementLayer.end()) {
    return;
  }
  const auto & traffic_light = *traffic_light_it;
  const auto current_vehicle_traffic_light_color =
    get_highest_confidence_traffic_signal(traffic_light->id(), traffic_light_id_map);
  for (const auto & attribute : traffic_light->attributes()) {
    const auto & color_mapping = parse_signal_estimation_rules(attribute.first);
    if (!color_mapping) {
      continue;
    }
    const auto & [from_color, to_color] = *color_mapping;
    if (from_color != current_vehicle_traffic_light_color) {
      continue;
    }
    for (const auto id : parse_ids(attribute.second.value())) {
      crosswalk_traffic_signal_overrides[id] = to_color;
    }
  }
}

TrafficSignalArray CrosswalkTrafficLightEstimator::estimate(
  const TrafficSignalArray & msg, const rclcpp::Time & current_time)
{
  TrafficSignalArray output = msg;

  TrafficLightIdMap traffic_light_id_map;

  std::unordered_map<lanelet::Id, uint8_t> crosswalk_traffic_signal_overrides;
  for (const auto & traffic_signal : msg.traffic_light_groups) {
    traffic_light_id_map[traffic_signal.traffic_light_group_id] =
      std::pair<TrafficSignal, rclcpp::Time>(traffic_signal, current_time);
  }
  // we need the full traffic_light_id_map before calculating overrides from map
  for (const auto & traffic_signal : msg.traffic_light_groups) {
    update_crosswalk_overrides_from_map(
      crosswalk_traffic_signal_overrides, traffic_signal.traffic_light_group_id,
      traffic_light_id_map);
  }

  // Collect vehicle traffic light IDs with known colors (from received and last detected signals)
  std::unordered_set<lanelet::Id> known_color_vehicle_traffic_light_ids;
  for (const auto & traffic_signal : msg.traffic_light_groups) {
    known_color_vehicle_traffic_light_ids.insert(traffic_signal.traffic_light_group_id);
  }
  if (config_.use_last_detect_color) {
    for (const auto & [traffic_light_id, _] : last_detect_color_) {
      known_color_vehicle_traffic_light_ids.insert(traffic_light_id);
    }
  }

  // Collect crosswalks related to the vehicle traffic lights with known colors
  std::unordered_set<lanelet::Id> crosswalk_ids_to_process;
  for (const auto & traffic_light_id : known_color_vehicle_traffic_light_ids) {
    const auto crosswalks_iter = traffic_light_id_to_crosswalks_.find(traffic_light_id);
    if (crosswalks_iter == traffic_light_id_to_crosswalks_.end()) {
      continue;
    }
    for (const auto & crosswalk : crosswalks_iter->second) {
      crosswalk_ids_to_process.insert(crosswalk.id());
    }
  }

  for (const auto & crosswalk_id : crosswalk_ids_to_process) {
    const auto & crosswalk = lanelet_map_ptr_->laneletLayer.get(crosswalk_id);
    const auto & conflict_lls = crosswalk_to_vehicle_lanelets_.at(crosswalk_id);
    const auto non_red_lanelets = get_non_red_lanelets(conflict_lls, traffic_light_id_map);

    const auto crosswalk_tl_color = estimate_crosswalk_traffic_signal(crosswalk, non_red_lanelets);
    set_crosswalk_traffic_signal(
      crosswalk, crosswalk_tl_color, msg, output, crosswalk_traffic_signal_overrides, current_time);
  }

  remove_duplicate_ids(output);

  update_last_detected_signal(traffic_light_id_map, current_time);

  return output;
}

void CrosswalkTrafficLightEstimator::update_last_detected_signal(
  const TrafficLightIdMap & traffic_light_id_map, const rclcpp::Time & current_time)
{
  for (const auto & input_traffic_signal : traffic_light_id_map) {
    const auto & elements = input_traffic_signal.second.first.elements;

    if (elements.empty()) {
      continue;
    }

    if (elements.front().color == TrafficSignalElement::UNKNOWN) {
      continue;
    }

    const auto & id = input_traffic_signal.second.first.traffic_light_group_id;

    last_detect_color_.insert_or_assign(id, input_traffic_signal.second);
  }

  std::vector<int32_t> erase_id_list;
  for (const auto & last_traffic_signal : last_detect_color_) {
    const auto & id = last_traffic_signal.second.first.traffic_light_group_id;

    if (traffic_light_id_map.count(id) == 0) {
      // hold signal recognition results for [last_detect_color_hold_time] seconds.
      const auto time_from_last_detected =
        (current_time - last_traffic_signal.second.second).seconds();
      if (time_from_last_detected > config_.last_detect_color_hold_time) {
        erase_id_list.emplace_back(id);
      }
    }
  }
  for (const auto id : erase_id_list) {
    last_detect_color_.erase(id);
    flashing_detector_.clear_state(id);
  }
}

void CrosswalkTrafficLightEstimator::set_crosswalk_traffic_signal(
  const lanelet::ConstLanelet & crosswalk, const uint8_t color, const TrafficSignalArray & msg,
  TrafficSignalArray & output,
  const std::unordered_map<lanelet::Id, uint8_t> & crosswalk_traffic_signal_overrides,
  const rclcpp::Time & current_time)
{
  const auto tl_reg_elems = crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();

  std::unordered_map<lanelet::Id, size_t> valid_id2idx_map;  // detected traffic light
  for (size_t i = 0; i < msg.traffic_light_groups.size(); ++i) {
    const auto & signal = msg.traffic_light_groups[i];
    valid_id2idx_map[signal.traffic_light_group_id] = i;
  }

  std::unordered_map<lanelet::Id, size_t> output_id2idx_map;  // to check duplicate
  for (size_t i = 0; i < output.traffic_light_groups.size(); ++i) {
    const auto & signal = output.traffic_light_groups[i];
    output_id2idx_map[signal.traffic_light_group_id] = i;
  }

  TrafficSignalElement base_traffic_signal_element;
  base_traffic_signal_element.color = color;
  base_traffic_signal_element.shape = TrafficSignalElement::CIRCLE;
  base_traffic_signal_element.confidence = 1.0;

  for (const auto & tl_reg_elem : tl_reg_elems) {
    const lanelet::Id id = tl_reg_elem->id();

    // helper lambda to get or create output signal
    auto get_or_create_output_signal = [&](lanelet::Id id) -> TrafficSignal & {
      if (output_id2idx_map.count(id)) {
        return output.traffic_light_groups[output_id2idx_map[id]];
      } else {
        // element need to be added in later
        TrafficSignal new_signal;
        new_signal.traffic_light_group_id = id;
        output.traffic_light_groups.push_back(new_signal);
        output_id2idx_map[id] = output.traffic_light_groups.size() - 1;
        return output.traffic_light_groups.back();
      }
    };

    TrafficSignal & out_signal = get_or_create_output_signal(id);

    auto replace_out_signal_elements = [&](const TrafficSignalElement & element) {
      out_signal.elements.clear();
      out_signal.elements.push_back(element);
    };

    // 1. Map-based override (highest priority)
    if (auto it = crosswalk_traffic_signal_overrides.find(id);
        it != crosswalk_traffic_signal_overrides.end()) {
      replace_out_signal_elements(base_traffic_signal_element);
      out_signal.elements[0].color = it->second;  // override color
      continue;
    }
    // 2. Use detected pedestrian signal if valid
    if (auto it = valid_id2idx_map.find(id); it != valid_id2idx_map.end()) {
      const auto & detected = msg.traffic_light_groups[it->second];

      if (!config_.use_pedestrian_signal_detect || is_invalid_detection_status(detected)) {
        // Replace detection with estimated base color
        replace_out_signal_elements(base_traffic_signal_element);
        continue;
      }

      // Update flashing state and apply the most recent color
      if (out_signal.elements
            .empty()) {  // unnecessary check because msg has detection but for safety
        out_signal.elements.push_back(base_traffic_signal_element);
      }
      out_signal.elements[0].color =
        flashing_detector_.estimate_stable_color(detected, current_time);
      continue;
    }

    // 3. No detection available → use estimated vehicle-based color
    replace_out_signal_elements(base_traffic_signal_element);
  }
}

lanelet::ConstLanelets CrosswalkTrafficLightEstimator::get_non_red_lanelets(
  const lanelet::ConstLanelets & lanelets, const TrafficLightIdMap & traffic_light_id_map) const
{
  lanelet::ConstLanelets non_red_lanelets{};

  for (const auto & lanelet : lanelets) {
    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();

    if (tl_reg_elems.empty()) {
      continue;
    }

    const auto tl_reg_elem = tl_reg_elems.front();
    const auto current_detected_signal =
      get_highest_confidence_traffic_signal(tl_reg_elem->id(), traffic_light_id_map);

    if (!current_detected_signal && !config_.use_last_detect_color) {
      continue;
    }

    const auto current_is_not_red =
      current_detected_signal ? current_detected_signal.get() == TrafficSignalElement::GREEN ||
                                  current_detected_signal.get() == TrafficSignalElement::AMBER
                              : true;

    const auto current_is_unknown_or_none =
      current_detected_signal ? current_detected_signal.get() == TrafficSignalElement::UNKNOWN
                              : true;

    const auto last_detected_signal =
      get_highest_confidence_traffic_signal(tl_reg_elem->id(), last_detect_color_);

    if (!last_detected_signal) {
      continue;
    }

    const auto was_not_red = current_is_unknown_or_none &&
                             (last_detected_signal.get() == TrafficSignalElement::GREEN ||
                              last_detected_signal.get() == TrafficSignalElement::AMBER) &&
                             config_.use_last_detect_color;

    if (!current_is_not_red && !was_not_red) {
      continue;
    }

    non_red_lanelets.push_back(lanelet);
  }

  return non_red_lanelets;
}

uint8_t CrosswalkTrafficLightEstimator::estimate_crosswalk_traffic_signal(
  const lanelet::ConstLanelet & crosswalk, const lanelet::ConstLanelets & non_red_lanelets) const
{
  bool has_left_non_red_lane = false;
  bool has_right_non_red_lane = false;
  bool has_straight_non_red_lane = false;
  bool has_related_non_red_tl = false;

  const std::string related_tl_id = crosswalk.attributeOr("related_traffic_light", "none");

  for (const auto & lanelet : non_red_lanelets) {
    const std::string turn_direction = lanelet.attributeOr("turn_direction", "none");

    if (turn_direction == "left") {
      has_left_non_red_lane = true;
    } else if (turn_direction == "right") {
      has_right_non_red_lane = true;
    } else {
      has_straight_non_red_lane = true;
    }

    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
    if (tl_reg_elems.front()->id() == std::atoi(related_tl_id.c_str())) {
      has_related_non_red_tl = true;
    }
  }

  if (has_straight_non_red_lane || has_related_non_red_tl) {
    return TrafficSignalElement::RED;
  }

  const auto merge_lane_exists = has_merge_lane(non_red_lanelets, routing_graph_ptr_);
  return !merge_lane_exists && has_left_non_red_lane && has_right_non_red_lane
           ? TrafficSignalElement::RED
           : TrafficSignalElement::UNKNOWN;
}

}  // namespace autoware::crosswalk_traffic_light_estimator
