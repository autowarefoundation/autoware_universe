// Copyright 2024 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__TRAFFIC_SIGNAL_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__TRAFFIC_SIGNAL_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{

class TrafficSignalModule
{
public:
  struct Params
  {
    double threshold_velocity_assumed_as_stopping{0.25};
    std::vector<double> distance_set_for_no_intention_to_walk;
    std::vector<double> timeout_set_for_no_intention_to_walk;
  };

  explicit TrafficSignalModule(rclcpp::Node & node) : node_(node) {}

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
  {
    time_keeper_ = std::move(time_keeper);
  }

  void setParams(const Params & params)
  {
    threshold_velocity_assumed_as_stopping_ = params.threshold_velocity_assumed_as_stopping;
    distance_set_for_no_intention_to_walk_ = params.distance_set_for_no_intention_to_walk;
    timeout_set_for_no_intention_to_walk_ = params.timeout_set_for_no_intention_to_walk;
  }

  void update(const TrafficLightGroupArray & traffic_signal_groups);
  void removeDisappearedObjects(const TrackedObjects & objects);

  [[nodiscard]] std::optional<lanelet::Id> getSignalId(
    const lanelet::ConstLanelet & way_lanelet) const;

  bool calcIntentionToCross(
    const TrackedObject & object, const lanelet::ConstLanelet & crosswalk,
    const lanelet::Id & signal_id);

private:
  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  std::unordered_map<lanelet::Id, TrafficLightGroup> signal_id_map_;
  std::map<std::pair<std::string, lanelet::Id>, rclcpp::Time> stopped_times_against_green_;

  double threshold_velocity_assumed_as_stopping_{0.0};
  std::vector<double> distance_set_for_no_intention_to_walk_;
  std::vector<double> timeout_set_for_no_intention_to_walk_;

  [[nodiscard]] std::optional<TrafficLightElement> getSignalElement(const lanelet::Id & id) const;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__TRAFFIC_SIGNAL_HPP_
