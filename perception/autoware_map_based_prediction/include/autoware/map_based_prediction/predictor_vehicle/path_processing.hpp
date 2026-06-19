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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PATH_PROCESSING_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PATH_PROCESSING_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"
#include "autoware/map_based_prediction/path_generator/path_generator.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/maneuver_prediction.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/object_processing.hpp"

#include <autoware_utils/system/lru_cache.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/LaneletPath.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

// Hash specialization for LaneletPath used by the LRU cache
namespace std
{
template <>
struct hash<lanelet::routing::LaneletPath>
{
  // 0x9e3779b9 is a magic number. See
  // https://stackoverflow.com/questions/4948780/magic-number-in-boosthash-combine
  size_t operator()(const lanelet::routing::LaneletPath & path) const
  {
    size_t seed = 0;
    for (const auto & lanelet : path) {
      seed ^= hash<int64_t>{}(lanelet.id()) + 0x9e3779b9 + (seed << 6U) + (seed >> 2U);
    }
    return seed;
  }
};
}  // namespace std

namespace autoware::map_based_prediction
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class PathProcessor
{
public:
  struct Params
  {
    double lateral_control_time_horizon{5.0};
    double prediction_time_horizon{15.0};
    double prediction_time_horizon_rate_for_validate_lane_length{0.8};
    double prediction_sampling_time_interval{0.5};
    double min_velocity_for_map_based_prediction{1.0};
    double reference_path_resolution{0.5};
    bool check_lateral_acceleration_constraints{true};
    double max_lateral_accel{0.5};
    double min_acceleration_before_curve{-2.5};
    bool use_vehicle_acceleration{false};
    double speed_limit_multiplier{1.5};
    double acceleration_exponential_half_life{2.5};
    bool consider_only_routable_neighbours{false};
  };

  explicit PathProcessor(rclcpp::Node & node);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr);
  void setLaneletMap(
    std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
    std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr,
    std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr);
  void setManeuverPredictor(ManeuverPredictor & maneuver_predictor);
  void setObjectTracker(ObjectTracker & object_tracker);
  void setParams(const Params & params);
  void clearLRUCache();

  std::optional<PredictedObject> predict(
    const std_msgs::msg::Header & header, const TrackedObject & object,
    double objects_detected_time, visualization_msgs::msg::MarkerArray * debug_markers);

private:
  std::optional<size_t> searchProperStartingRefPathIndex(
    const TrackedObject & object, const PosePath & pose_path) const;
  std::vector<LaneletPathWithPathInfo> getPredictedReferencePath(
    const TrackedObject & object, const LaneletsData & current_lanelets_data,
    double object_detected_time, double time_horizon);
  std::vector<PredictedRefPath> convertPredictedReferencePath(
    const TrackedObject & object,
    const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths) const;
  std::pair<PosePath, double> convertLaneletPathToPosePath(
    const lanelet::routing::LaneletPath & path) const;
  std::vector<double> calcTrajectoryCurvatureFrom3Points(
    const TrajectoryPoints & trajectory, size_t idx_dist);
  TrajectoryPoints toTrajectoryPoints(const PredictedPath & path, double velocity);
  bool isLateralAccelerationConstraintSatisfied(
    const TrajectoryPoints & trajectory, double delta_time);

  rclcpp::Node & node_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  std::shared_ptr<PathGenerator> path_generator_;
  ManeuverPredictor * maneuver_predictor_{nullptr};
  ObjectTracker * object_tracker_{nullptr};
  mutable autoware_utils::LRUCache<lanelet::routing::LaneletPath, std::pair<PosePath, double>>
    lru_cache_of_convert_path_type_{1000};
  Params params_;
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VEHICLE__PATH_PROCESSING_HPP_
