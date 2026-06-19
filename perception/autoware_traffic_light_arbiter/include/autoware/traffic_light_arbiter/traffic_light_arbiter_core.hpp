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

  // Load map-derived state from a parsed LaneletMap. Core extracts the
  // regulatory-element IDs it needs (vehicle traffic lights and, when
  // signal matching is enabled, pedestrian traffic lights) internally so
  // callers don't have to know which subsets matter.
  void set_map(const lanelet::LaneletMapConstPtr & map);

  struct ExpiredExternalSignal
  {
    lanelet::Id id;
    double age;
  };

  // Update perception buffer, then sweep external cache against msg.stamp
  // using external_time_tolerance_. Returns expired entries for caller logging.
  std::vector<ExpiredExternalSignal> ingest_perception(const TrafficSignalArray & msg);

  // Outcome of an external-msg ingest. `accepted == false` means the msg's
  // stamp differed from current_time by more than external_delay_tolerance_
  // and was rejected without touching internal state. When accepted,
  // `expired` carries any cache entries that the bundled sweep evicted.
  struct ExternalIngestResult
  {
    bool accepted;
    std::vector<ExpiredExternalSignal> expired;
  };

  // Admission-control + update + sweep for an external msg:
  //   1. Reject (return {false, {}}) when the msg arrival is too far from
  //      current_time, using external_delay_tolerance_.
  //   2. Otherwise update external cache entries with msg.stamp, sweep
  //      external cache against current_time using external_delay_tolerance_,
  //      and return expired entries.
  // Perception staleness is evaluated non-destructively inside arbitrate()
  // using perception_time_tolerance_; ingest_external no longer touches
  // latest_perception_msg_.
  ExternalIngestResult ingest_external(
    const TrafficSignalArray & msg, const rclcpp::Time & current_time);

  // Result of one arbitration cycle. arbitrate() writes the arbitrated signals
  // into the caller-provided `output`. The arbiter intentionally does not stamp
  // the output: stamp inheritance is an I/O concern owned by the Node (e.g.
  // "publish carries the trigger msg's stamp"). The Node assigns
  // `output.stamp` before publishing and compares its trigger stamp against
  // `latest_input_time` for staleness logging.
  //
  // latest_input_time defaults to epoch on RCL_ROS_TIME so the Node can compare
  // it against rclcpp::Time(msg->stamp) (also RCL_ROS_TIME) regardless of which
  // arbitrate() branch was taken.
  struct ArbitrationResult
  {
    bool has_output = false;  // false when no map yet (skip publish); true otherwise.
    std::vector<lanelet::Id> off_map_signal_ids;
    rclcpp::Time latest_input_time{0, 0, RCL_ROS_TIME};
  };
  ArbitrationResult arbitrate(TrafficSignalArray & output);

private:
  // True when |current_time - msg_stamp| exceeds external_delay_tolerance_.
  // Used by ingest_external for admission control.
  bool is_external_outdated(
    const rclcpp::Time & current_time, const rclcpp::Time & msg_stamp) const;

  // Sweeps external cache: removes every stored entry whose stamp deviates
  // from `reference_time` beyond `tolerance`, returning the removed entries.
  std::vector<ExpiredExternalSignal> sweep_expired_external_signals(
    const rclcpp::Time & reference_time, double tolerance);

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
