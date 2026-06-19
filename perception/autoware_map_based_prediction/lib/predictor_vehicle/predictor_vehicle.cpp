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

#include "autoware/map_based_prediction/predictor_vehicle/predictor_vehicle.hpp"

#include <memory>

namespace autoware::map_based_prediction
{

PredictorVehicle::PredictorVehicle(rclcpp::Node & node)
: node_(node), object_tracker_(node), maneuver_predictor_(node), path_processor_(node)
{
  path_processor_.setManeuverPredictor(maneuver_predictor_);
  path_processor_.setObjectTracker(object_tracker_);
}

void PredictorVehicle::setParams(const Params & params)
{
  params_ = params;
  object_tracker_.setParams(params.object_tracker);
  maneuver_predictor_.setParams(params.maneuver_predictor);
  path_processor_.setParams(params.path_processor);
}

void PredictorVehicle::setLaneletMap(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr,
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr)
{
  object_tracker_.setLaneletMap(lanelet_map_ptr);
  object_tracker_.setRoutingGraph(routing_graph_ptr);
  maneuver_predictor_.setRoutingGraph(routing_graph_ptr);
  path_processor_.setLaneletMap(lanelet_map_ptr, routing_graph_ptr, traffic_rules_ptr);
}

void PredictorVehicle::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = time_keeper_ptr;
  object_tracker_.setTimeKeeper(time_keeper_ptr);
  maneuver_predictor_.setTimeKeeper(time_keeper_ptr);
  path_processor_.setTimeKeeper(time_keeper_ptr);
}

void PredictorVehicle::removeOldHistory(double current_time, double buffer_time)
{
  object_tracker_.removeOldHistory(current_time, buffer_time);
}

std::optional<PredictedObject> PredictorVehicle::predict(
  const std_msgs::msg::Header & header, const TrackedObject & object, double objects_detected_time,
  visualization_msgs::msg::MarkerArray * debug_markers)
{
  return path_processor_.predict(header, object, objects_detected_time, debug_markers);
}

}  // namespace autoware::map_based_prediction
