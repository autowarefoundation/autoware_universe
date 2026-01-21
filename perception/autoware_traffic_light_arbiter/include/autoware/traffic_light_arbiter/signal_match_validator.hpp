// Copyright 2024 The Autoware Contributors
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

#ifndef AUTOWARE__TRAFFIC_LIGHT_ARBITER__SIGNAL_MATCH_VALIDATOR_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_ARBITER__SIGNAL_MATCH_VALIDATOR_HPP_

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

namespace autoware::traffic_light
{

enum class SourcePriority {
  CONFIDENCE,  // Use confidence-based selection
  EXTERNAL,    // Prioritize external signals
  PERCEPTION   // Prioritize perception signals
};

/**
 * @class SignalMatchValidator
 * @brief Validates and compares traffic signal data from different sources.
 *
 * This class is designed to compare and validate traffic signal information
 * received from internal (e.g. camera perception) and external (e.g. V2I communication).
 * It aims to ensure that the traffic signal information from both sources is consistent.
 */
class SignalMatchValidator
{
public:
  using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
  using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficLightElement;
  using TrafficLightConstPtr = lanelet::TrafficLightConstPtr;

  /**
   * @brief Default constructor for SignalMatchValidator.
   */
  SignalMatchValidator() = default;

  /**
   * @brief Validates and compares traffic signal data from perception and external sources.
   *
   * This method compares traffic signal data obtained from perception results
   * with data received from external source. It aims to find and return signals
   * that are present and consistent in both datasets. If signals are not consistent,
   * treat them as the unknown signal.
   *
   * @param perception_signals Traffic signal data from perception.
   * @param external_signals Traffic signal data from external source.
   * @return A validated TrafficSignalArray.
   */
  TrafficSignalArray validateSignals(
    const TrafficSignalArray & perception_signals, const TrafficSignalArray & external_signals);

  /**
   * @brief Sets the pedestrian signals to be considered during validation.
   *
   * This method allows the specification of pedestrian signals, which are then
   * used to adjust the validation logic, acknowledging their unique characteristics
   * in traffic signal datasets.
   *
   * @param pedestrian_signals A vector of pedestrian signal pointers.
   */
  void setPedestrianSignals(const std::vector<TrafficLightConstPtr> & pedestrian_signals);

  /**
   * @brief Sets the source priority for signal selection.
   *
   * This method sets the priority mode for selecting between perception and external signals.
   * Options are CONFIDENCE (use confidence-based selection), EXTERNAL (prioritize external),
   * or PERCEPTION (prioritize perception signals).
   *
   * @param source_priority The priority mode for signal selection.
   */
  void setSourcePriority(const SourcePriority source_priority);

private:
  SourcePriority source_priority_;
  std::unordered_set<lanelet::Id> map_pedestrian_signal_regulatory_elements_set_;

  /**
   * @brief Checks if a given signal ID corresponds to a pedestrian signal.
   *
   * This method determines whether a signal, identified by its ID, is classified
   * as a pedestrian signal.
   *
   * @param signal_id The ID of the signal to check.
   * @return True if the signal is a pedestrian signal, false otherwise.
   */
  bool isPedestrianSignal(const lanelet::Id & signal_id);
};

}  // namespace autoware::traffic_light

#endif  // AUTOWARE__TRAFFIC_LIGHT_ARBITER__SIGNAL_MATCH_VALIDATOR_HPP_
