// Copyright 2026 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PRIORITY_PREDICTOR__TRAFFIC_SIGNAL_STOP_PREDICTOR_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PRIORITY_PREDICTOR__TRAFFIC_SIGNAL_STOP_PREDICTOR_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/priority_predictor/debug_priority_pred.hpp"
#include "autoware/map_based_prediction/priority_predictor/signal_stop_hysteresis.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Forward.h>

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace autoware::map_based_prediction::priority_predictor
{
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

bool hasTrafficLight(const lanelet::ConstLanelet & way_lanelet);

std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & way_lanelet);

std::optional<lanelet::ConstLineString3d> getStopLineOrEntryEdge(
  const lanelet::ConstLanelet & way_lanelet);

std::optional<lanelet::Id> getTrafficSignalId(const lanelet::ConstLanelet & way_lanelet);

std::optional<TrafficLightGroup> getSignalForLanelet(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & signal_id_map,
  const lanelet::ConstLanelet & way_lanelet);

std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line);

bool hasStopLineAhead(
  const geometry_msgs::msg::Point & position, const PosePath & ref_path,
  const lanelet::ConstLineString3d & stop_line);

bool findTrafficLightLaneletOnPath(
  const lanelet::routing::LaneletPath & lanelet_path, lanelet::ConstLanelet & signal_lanelet);

lanelet::routing::LaneletPath buildLaneletPathFromPredictedPath(
  const PredictedPath & predicted_path,
  const autoware::experimental::lanelet2_utils::LaneletRTree & road_lanelet_rtree,
  double sample_interval_m = 3.0);

bool findTrafficLightLaneletOnPredictedPath(
  const PredictedPath & predicted_path,
  const autoware::experimental::lanelet2_utils::LaneletRTree & road_lanelet_rtree,
  lanelet::ConstLanelet & signal_lanelet);

bool evaluateSignalStopRequirement(
  const lanelet::ConstLanelet & lanelet, const std::optional<TrafficLightGroup> & signal);

bool shouldAddStopHypothesis(bool signal_requires_stop, bool has_stop_line_ahead);

double weakenConfidenceInLaneChange(const Maneuver & maneuver, const double stop_weight);

struct ObjectPrediction
{
  const TrackedObject & object;
  std::vector<PredictedPath> predicted_paths;
};

struct PriorityPredictionParams
{
  double stop_time_hysteresis{0.2};
  double go_time_hysteresis{0.1};
  double signal_retention_timeout{15.0};
};

class TrafficSignalStopPredictor
{
public:
  void setParameters(
    const PriorityPredictionParams & params, const double signal_observation_timeout)
  {
    params_ = params;
    signal_observation_timeout_ = signal_observation_timeout;
  }

  void setLaneletMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  void setTrafficSignal(const TrafficLightGroupArray & traffic_signals, const rclcpp::Time & now);

  void clearFrameDebug();

  std::vector<PredictedPath> addStopHypotheses(
    const ObjectPrediction & prediction, const rclcpp::Time & now);

  const StopHypothesisDebug & getDebugInfo() const { return debug_; }

private:
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::optional<autoware::experimental::lanelet2_utils::LaneletRTree> road_lanelet_rtree_;
  std::unordered_map<lanelet::Id, TrafficLightGroup> traffic_signal_id_map_;
  std::unordered_map<lanelet::Id, TrafficLightGroup> stabilized_traffic_signal_id_map_;
  std::unordered_map<lanelet::Id, SignalStabilizeState> signal_stabilize_state_;
  std::optional<rclcpp::Time> latest_traffic_signal_time_;
  PriorityPredictionParams params_;
  double signal_observation_timeout_{0.0};
  StopHypothesisDebug debug_;
};

}  // namespace autoware::map_based_prediction::priority_predictor

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PRIORITY_PREDICTOR__TRAFFIC_SIGNAL_STOP_PREDICTOR_HPP_
