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
#include "autoware/map_based_prediction/predictor_vehicle/lane_change_path_suppression.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/maneuver_prediction.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/object_processing.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/system/lru_cache.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/LaneletPath.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
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
    bool lane_change_path_suppression_enable{true};
    bool lane_change_path_suppression_keep_when_lane_ends{true};
    double lane_change_path_suppression_lane_end_rescue_time_horizon{5.0};
  };

  explicit PathProcessor(autoware::agnocast_wrapper::Node & node);

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
  friend class PathProcessorSuppressionTest;

  /// Per-object quantities shared by the reference path search of every current lanelet.
  struct ReferencePathSearchContext
  {
    double obj_vel{0.0};
    double obj_acc{0.0};
    double time_horizon{0.0};
    double lambda{0.0};
    double validate_time_horizon{0.0};
    double final_speed_after_acceleration{0.0};
  };

  /// Reference path candidates generated from one current lanelet.
  struct LaneletReferencePaths
  {
    std::vector<LaneletPathWithPathInfo> paths;
    bool left_exists{false};
    bool right_exists{false};
    bool center_exists{false};
    bool lane_ends_within_horizon{false};
    double lane_end_distance{-1.0};
  };

  /// Predicted paths accumulated while converting reference paths of one object.
  struct OnLanePathCandidates
  {
    std::vector<PredictedPath> predicted_paths;
    double min_avg_curvature{std::numeric_limits<double>::max()};
    std::optional<PredictedPath> path_with_smallest_avg_curvature;
    /// Origin lanelets whose lane-follow reference (two or more points) produced an accepted path.
    std::set<lanelet::Id> surviving_center_origins;
  };

  std::optional<PredictedObject> predictOnLane(
    const TrackedObject & object, const LaneletsData & current_lanelets,
    const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths,
    const std::vector<LaneletPathWithPathInfo> & suppressed_paths,
    visualization_msgs::msg::MarkerArray * debug_markers, size_t * processing_passes = nullptr);
  std::optional<size_t> searchProperStartingRefPathIndex(
    const TrackedObject & object, const PosePath & pose_path) const;
  std::vector<LaneletPathWithPathInfo> getPredictedReferencePath(
    const TrackedObject & object, const LaneletsData & current_lanelets_data,
    double object_detected_time, double time_horizon,
    std::vector<LaneletPathWithPathInfo> * suppressed_paths);
  ReferencePathSearchContext makeReferencePathSearchContext(
    const TrackedObject & object, double time_horizon) const;
  double calcRoutingCostLimit(
    const ReferencePathSearchContext & context, const lanelet::ConstLanelet & lanelet,
    double target_speed_limit) const;
  lanelet::routing::LaneletPaths getPathsForLanelet(
    const lanelet::ConstLanelet & lanelet,
    const lanelet::routing::PossiblePathsParams & possible_params, const TrackedObject & object,
    double validate_time_horizon);
  std::optional<lanelet::ConstLanelet> getLeftOrRightLanelet(
    const lanelet::ConstLanelet & lanelet, bool get_left) const;
  LaneletReferencePaths collectLaneletReferencePaths(
    const TrackedObject & object, const LaneletData & current_lanelet_data,
    const ReferencePathSearchContext & context, bool rescue_lane_end);
  void updateFuturePossibleLanelets(
    const std::string & object_id, const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths);
  void logLaneChangeSuppressionDecisions(
    const std::string & object_id, const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths,
    const std::vector<detail::LaneChangePathSuppressionInfo> & path_origins);
  void generateOnLanePaths(
    const TrackedObject & yaw_fixed_object, double abs_obj_speed,
    const std::vector<PredictedRefPath> & reference_paths, OnLanePathCandidates & candidates);
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

  autoware::agnocast_wrapper::Node & node_;
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
