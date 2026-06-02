// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PREDICTOR_VEHICLE_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PREDICTOR_VEHICLE_HPP_

#include "autoware/map_based_prediction/predictor_vehicle/maneuver_prediction.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/object_processing.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/path_processing.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>
#include <optional>

namespace autoware::map_based_prediction
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::TrackedObject;

class PredictorVehicle
{
public:
  struct Params
  {
    ObjectTracker::Params object_tracker;
    ManeuverPredictor::Params maneuver_predictor;
    PathProcessor::Params path_processor;
  };

  explicit PredictorVehicle(rclcpp::Node & node);
  ~PredictorVehicle() = default;

  void setParams(const Params & params);
  const Params & getParams() const { return params_; }

  void setLaneletMap(
    std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
    std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr,
    std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr);

  void removeOldHistory(double current_time, double buffer_time);

  std::optional<PredictedObject> predict(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    double objects_detected_time, visualization_msgs::msg::MarkerArray * debug_markers);

private:
  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  Params params_;

  // Sub-modules
  ObjectTracker object_tracker_;
  ManeuverPredictor maneuver_predictor_;
  PathProcessor path_processor_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PREDICTOR_VEHICLE_HPP_
