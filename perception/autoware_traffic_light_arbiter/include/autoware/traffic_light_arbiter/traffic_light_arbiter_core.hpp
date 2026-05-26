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

#ifndef AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_CORE_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_CORE_HPP_

#include <autoware/traffic_light_arbiter/signal_match_validator.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

class TrafficLightArbiterCore
{
public:
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  using PredictedTrafficLightState = autoware_perception_msgs::msg::PredictedTrafficLightState;
  using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
  using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
  using TrafficLightConstPtr = lanelet::TrafficLightConstPtr;

  TrafficLightArbiterCore(
    SourcePriority source_priority, bool enable_signal_matching, double external_delay_tolerance,
    double external_time_tolerance, double perception_time_tolerance);

  void setTrafficLightIds(std::unordered_set<lanelet::Id> ids);

  void setPedestrianSignals(const std::vector<TrafficLightConstPtr> & pedestrian_signals);

  bool isExternalOutdated(const rclcpp::Time & current_time, const rclcpp::Time & msg_stamp) const;

  struct DroppedExternalSignal
  {
    lanelet::Id id;
    double age;
  };
  std::vector<DroppedExternalSignal> cleanupExpiredExternalSignals(
    const rclcpp::Time & reference_time, double tolerance);

  void ingestPerception(const TrafficSignalArray & msg);

  void ingestExternal(const TrafficSignalArray & msg);

  struct ArbitrationResult
  {
    std::optional<TrafficSignalArray> output;
    std::vector<lanelet::Id> off_map_signal_ids;
    bool output_not_latest = false;
  };
  ArbitrationResult arbitrate(const builtin_interfaces::msg::Time & stamp);

private:
  SourcePriority source_priority_;
  bool enable_signal_matching_;
  double external_delay_tolerance_;
  double external_time_tolerance_;
  double perception_time_tolerance_;

  std::unique_ptr<std::unordered_set<lanelet::Id>> map_regulatory_elements_set_;
  std::unique_ptr<SignalMatchValidator> signal_match_validator_;

  TrafficSignalArray latest_perception_msg_;
  std::unordered_map<lanelet::Id, std::pair<rclcpp::Time, TrafficSignal>> external_traffic_lights_;
};

}  // namespace autoware::traffic_light

#endif  // AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_CORE_HPP_
