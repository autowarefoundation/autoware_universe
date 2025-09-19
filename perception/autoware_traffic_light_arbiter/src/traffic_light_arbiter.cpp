// Copyright 2023 The Autoware Contributors
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
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/time.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lanelet
{

using TrafficLightConstPtr = std::shared_ptr<const TrafficLight>;

std::vector<TrafficLightConstPtr> filter_traffic_signals(const LaneletMapConstPtr map)
{
  std::vector<TrafficLightConstPtr> signals;
  for (const auto & element : map->regulatoryElementLayer) {
    const auto signal = std::dynamic_pointer_cast<const TrafficLight>(element);
    if (signal) {
      signals.push_back(signal);
    }
  }

  return signals;
}

std::vector<TrafficLightConstPtr> filter_pedestrian_signals(const LaneletMapConstPtr map)
{
  namespace query = lanelet::utils::query;

  const auto all_lanelets = query::laneletLayer(map);
  const auto crosswalks = query::crosswalkLanelets(all_lanelets);
  std::vector<TrafficLightConstPtr> signals;

  for (const auto & crosswalk : crosswalks) {
    const auto traffic_light_reg_elems =
      crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();
    std::copy(
      traffic_light_reg_elems.begin(), traffic_light_reg_elems.end(), std::back_inserter(signals));
  }

  return signals;
}

}  // namespace lanelet

namespace autoware::traffic_light
{
TrafficLightArbiter::TrafficLightArbiter(const rclcpp::NodeOptions & options)
: Node("traffic_light_arbiter", options)
{
  external_delay_tolerance_ = this->declare_parameter<double>("external_delay_tolerance");
  external_time_tolerance_ = this->declare_parameter<double>("external_time_tolerance");
  perception_time_tolerance_ = this->declare_parameter<double>("perception_time_tolerance");
  external_priority_ = this->declare_parameter<bool>("external_priority");
  enable_signal_matching_ = this->declare_parameter<bool>("enable_signal_matching");

  if (enable_signal_matching_) {
    signal_match_validator_ = std::make_unique<SignalMatchValidator>();
    signal_match_validator_->setExternalPriority(external_priority_);
  }

  map_sub_ = create_subscription<LaneletMapBin>(
    "~/sub/vector_map", rclcpp::QoS(1).transient_local(),
    std::bind(&TrafficLightArbiter::onMap, this, std::placeholders::_1));

  perception_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/perception_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::onPerceptionMsg, this, std::placeholders::_1));

  external_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/external_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::onExternalMsg, this, std::placeholders::_1));

  pub_ = create_publisher<TrafficSignalArray>("~/pub/traffic_signals", rclcpp::QoS(1));
}

void TrafficLightArbiter::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  const auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);

  const auto signals = lanelet::filter_traffic_signals(map);
  map_regulatory_elements_set_ = std::make_unique<std::unordered_set<lanelet::Id>>();

  for (const auto & signal : signals) {
    map_regulatory_elements_set_->emplace(signal->id());
  }

  if (enable_signal_matching_) {
    // Filter only pedestrian signals to distinguish them in signal matching
    const auto pedestrian_signals = lanelet::filter_pedestrian_signals(map);
    signal_match_validator_->setPedestrianSignals(pedestrian_signals);
  }
}

void TrafficLightArbiter::onPerceptionMsg(const TrafficSignalArray::ConstSharedPtr msg)
{
  latest_perception_msg_ = *msg;

  // Clean up external signals that are too old relative to perception message
  const auto msg_time = rclcpp::Time(msg->stamp);
  cleanupExpiredExternalSignals(msg_time, external_time_tolerance_);

  arbitrateAndPublish(msg->stamp);
}

void TrafficLightArbiter::onExternalMsg(const TrafficSignalArray::ConstSharedPtr msg)
{
  const auto current_time = this->now();
  const auto msg_time = rclcpp::Time(msg->stamp);

  if (std::abs((current_time - msg_time).seconds()) > external_delay_tolerance_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received outdated V2X traffic signal messages");
    return;
  }

  // Update external traffic lights map with new information
  for (const auto & signal : msg->traffic_light_groups) {
    external_traffic_lights_[signal.traffic_light_group_id] = *msg;
  }

  // Clean up expired signals
  cleanupExpiredExternalSignals(current_time, external_delay_tolerance_);

  // Clear perception data if too old relative to any external signal
  if (
    std::abs((msg_time - rclcpp::Time(latest_perception_msg_.stamp)).seconds()) >
    perception_time_tolerance_) {
    latest_perception_msg_.traffic_light_groups.clear();
  }

  arbitrateAndPublish(msg->stamp);
}

void TrafficLightArbiter::cleanupExpiredExternalSignals(
  const rclcpp::Time & current_time, double tolerance)
{
  auto it = external_traffic_lights_.begin();
  while (it != external_traffic_lights_.end()) {
    const auto age = (current_time - rclcpp::Time(it->second.stamp)).seconds();
    if (std::abs(age) > tolerance) {
      RCLCPP_DEBUG(
        get_logger(), "Removing expired external traffic light signal (ID: %lu, age: %.2f s)",
        it->first, age);
      it = external_traffic_lights_.erase(it);
    } else {
      ++it;
    }
  }
}

void TrafficLightArbiter::arbitrateAndPublish(const builtin_interfaces::msg::Time & stamp)
{
  using ElementAndPriority = std::pair<Element, bool>;
  std::unordered_map<lanelet::Id, std::vector<ElementAndPriority>> regulatory_element_signals_map;

  // Create external signals array from stored valid signals
  TrafficSignalArray valid_external_signals;
  for (const auto & [id, info] : external_traffic_lights_) {
    valid_external_signals.traffic_light_groups.insert(
      valid_external_signals.traffic_light_groups.end(), info.traffic_light_groups.begin(),
      info.traffic_light_groups.end());
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
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received traffic signal messages before a map");
    return;
  }

  TrafficSignalArray output_signals_msg;
  output_signals_msg.stamp = stamp;

  if (map_regulatory_elements_set_->empty()) {
    pub_->publish(output_signals_msg);
    return;
  }

  auto add_signal_function = [&](const auto & signal, bool priority) {
    const auto id = signal.traffic_light_group_id;
    if (!map_regulatory_elements_set_->count(id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received a traffic signal not present in the current map (%lu)", id);
      return;
    }

    auto & elements_and_priority = regulatory_element_signals_map[id];
    for (const auto & element : signal.elements) {
      elements_and_priority.emplace_back(element, priority);
    }
  };

  if (enable_signal_matching_) {
    const auto validated_signals =
      signal_match_validator_->validateSignals(latest_perception_msg_, valid_external_signals);
    for (const auto & signal : validated_signals.traffic_light_groups) {
      add_signal_function(signal, false);
    }
  } else {
    for (const auto & signal : latest_perception_msg_.traffic_light_groups) {
      add_signal_function(signal, false);
    }

    for (const auto & signal : valid_external_signals.traffic_light_groups) {
      add_signal_function(signal, external_priority_);
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
          (iter_element.confidence < element.confidence || iter_priority < element_priority)) {
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

  pub_->publish(output_signals_msg);

  // Calculate latest time from available sources
  rclcpp::Time latest_time = rclcpp::Time(latest_perception_msg_.stamp);
  for (const auto & [id, info] : external_traffic_lights_) {
    const auto external_time = rclcpp::Time(info.stamp);
    if (external_time > latest_time) {
      latest_time = external_time;
    }
  }

  if (rclcpp::Time(output_signals_msg.stamp) < latest_time) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Published traffic signal messages are not latest");
  }
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightArbiter)
