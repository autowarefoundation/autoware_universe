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
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <optional>
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

  TrafficLightArbiterCore(
    SourcePriority source_priority, bool enable_signal_matching, double external_delay_tolerance,
    double external_time_tolerance, double perception_time_tolerance);

  // Extracts and stores the regulatory-element IDs the Core needs from the map
  // (vehicle traffic lights, plus pedestrian ones when signal matching is on).
  void set_map(const lanelet::LaneletMapConstPtr & map);

  // Stores the latest perception msg, then evicts external cache entries that
  // are stale relative to its stamp.
  void ingest_perception(const TrafficSignalArray & msg);

  // Rejects the msg (returns false) when its stamp is too far from current_time;
  // otherwise refreshes the external cache, sweeps stale entries, returns true.
  bool ingest_external(const TrafficSignalArray & msg, const rclcpp::Time & current_time);

  // Result of one arbitration cycle. `output` holds the arbitrated signals by
  // value; std::nullopt means no map has arrived yet, so the Node skips the
  // publish. The Core leaves output unstamped — the Node owns stamp inheritance
  // and uses latest_input_time for staleness logging.
  struct ArbitrationResult
  {
    std::optional<TrafficSignalArray> output;  // stamp left default; Node fills it in.
    std::vector<lanelet::Id> off_map_signal_ids;
    rclcpp::Time latest_input_time{0, 0, RCL_ROS_TIME};
  };
  ArbitrationResult arbitrate() const;

private:
  // True when |current_time - msg_stamp| exceeds external_delay_tolerance_.
  // Used by ingest_external for admission control.
  bool is_external_outdated(
    const rclcpp::Time & current_time, const rclcpp::Time & msg_stamp) const;

  // Sweeps external cache: removes every stored entry whose stamp deviates
  // from `reference_time` beyond `tolerance`.
  void sweep_expired_external_signals(const rclcpp::Time & reference_time, double tolerance);

  // Signal matching is on iff the validator exists: it is created in the
  // constructor exactly when matching is enabled and never replaced afterward,
  // so the pointer is the single source of truth for the mode.
  bool is_signal_matching_enabled() const { return signal_match_validator_ != nullptr; }

  SourcePriority source_priority_;
  double external_delay_tolerance_;
  double external_time_tolerance_;
  double perception_time_tolerance_;

  std::unique_ptr<std::unordered_set<lanelet::Id>> map_regulatory_elements_set_;
  std::unique_ptr<SignalMatchValidator> signal_match_validator_;

  TrafficSignalArray perception_traffic_light_;
  std::unordered_map<lanelet::Id, std::pair<rclcpp::Time, TrafficSignal>> external_traffic_lights_;
};

}  // namespace autoware::traffic_light

#endif  // AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_CORE_HPP_
