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

#include "autoware/map_based_prediction/predictor_vehicle/path_processing.hpp"

#include "autoware/map_based_prediction/predictor_vehicle/debug.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/lane_change_path_suppression.hpp"
#include "autoware/map_based_prediction/utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_object_kinematics.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

namespace detail
{
bool laneEndsWithinHorizon(
  const lanelet::routing::LaneletPath & path, const geometry_msgs::msg::Point & position,
  const double travel_distance, const lanelet::routing::RoutingGraph & graph)
{
  if (path.empty() || !graph.following(path.back()).empty()) return false;

  // Measure the remaining centerline arc length from the object's projection, not
  // the full current lanelet length or the straight-line distance to the endpoint.
  const auto centerline = path.front().centerline2d();
  const double object_arc_length =
    lanelet::geometry::toArcCoordinates(centerline, lanelet::BasicPoint2d(position.x, position.y))
      .length;
  double remaining_distance = -object_arc_length;
  for (const auto & lanelet : path) {
    remaining_distance += lanelet::geometry::length2d(lanelet);
  }
  return std::max(0.0, remaining_distance) <= travel_distance;
}

bool laneEndsWithinHorizon(
  const lanelet::ConstLanelet & current_lanelet, const geometry_msgs::msg::Point & position,
  const double travel_distance, const lanelet::routing::RoutingGraph & graph,
  size_t * expanded_lanelets, double * lane_end_distance)
{
  if (lane_end_distance) *lane_end_distance = -1.0;
  const double object_arc_length =
    lanelet::geometry::toArcCoordinates(
      current_lanelet.centerline2d(), lanelet::BasicPoint2d(position.x, position.y))
      .length;
  if (expanded_lanelets) *expanded_lanelets = 0;
  std::unordered_map<lanelet::Id, double> lengths;
  const auto length = [&](const lanelet::ConstLanelet & lane) {
    const auto found = lengths.find(lane.id());
    if (found != lengths.end()) return found->second;
    const double value = lanelet::geometry::length2d(lane);
    lengths.emplace(lane.id(), value);
    return value;
  };
  // Distances are measured at lanelet exits. Cache each length once per query.
  std::unordered_map<lanelet::Id, double> shortest;
  std::queue<std::pair<lanelet::ConstLanelet, double>> pending;
  const double initial_distance = length(current_lanelet) - object_arc_length;
  shortest.emplace(current_lanelet.id(), initial_distance);
  pending.emplace(current_lanelet, initial_distance);
  // A non-branching chain of depth D takes O(D) expected time and O(D) memory.
  // Equal/longer revisits are discarded, so nonnegative-length cycles terminate.
  while (!pending.empty()) {
    const auto [lane, distance] = pending.front();
    pending.pop();
    if (distance > shortest.at(lane.id()) || std::max(0.0, distance) > travel_distance) {
      continue;
    }
    if (expanded_lanelets) ++*expanded_lanelets;
    const auto following = graph.following(lane);
    if (following.empty()) {
      if (lane_end_distance) *lane_end_distance = std::max(0.0, distance);
      return true;
    }
    for (const auto & next : following) {
      const double next_distance = distance + length(next);
      if (std::max(0.0, next_distance) > travel_distance) continue;
      const auto found = shortest.find(next.id());
      if (found != shortest.end() && next_distance >= found->second) continue;
      shortest[next.id()] = next_distance;
      pending.emplace(next, next_distance);
    }
  }
  return false;
}

std::vector<LaneletPathWithPathInfo> suppressUndetectedLaneChangePaths(
  const std::vector<LaneletPathWithPathInfo> & paths,
  const std::vector<LaneChangePathSuppressionInfo> & origins, const bool enable,
  const bool keep_when_lane_ends, std::vector<LaneletPathWithPathInfo> * suppressed_paths)
{
  if (!enable) return paths;

  std::vector<LaneletPathWithPathInfo> filtered_paths;
  filtered_paths.reserve(paths.size());
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto & origin = origins.at(i);
    const bool suppress = paths[i].second.maneuver != Maneuver::LANE_FOLLOW &&
                          origin.center_exists &&
                          origin.detected_maneuver == Maneuver::LANE_FOLLOW &&
                          !(keep_when_lane_ends && origin.lane_ends_within_horizon);
    if (!suppress) {
      filtered_paths.push_back(paths[i]);
    } else if (suppressed_paths) {
      suppressed_paths->push_back(paths[i]);
    }
  }
  // A center candidate is never suppressed; retain the original set defensively.
  if (filtered_paths.empty()) {
    if (suppressed_paths) suppressed_paths->clear();
    return paths;
  }
  return filtered_paths;
}

std::vector<LaneletPathWithPathInfo> selectPathsToRestore(
  const std::vector<LaneletPathWithPathInfo> & suppressed_paths,
  const std::set<lanelet::Id> & surviving_center_origins)
{
  std::vector<LaneletPathWithPathInfo> paths_to_restore;
  for (const auto & path : suppressed_paths) {
    if (surviving_center_origins.count(path.second.origin_lanelet_id) == 0) {
      paths_to_restore.push_back(path);
    }
  }
  return paths_to_restore;
}
}  // namespace detail

namespace
{
lanelet::ConstLanelets getRightLineSharingLanelets(
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelets output_lanelets;
  lanelet::Lanelets right_lane_candidates =
    lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.rightBound());
  for (auto & candidate : right_lane_candidates) {
    if (candidate == current_lanelet) continue;
    if (candidate.leftBound() == current_lanelet.rightBound()) {
      output_lanelets.push_back(candidate);
    }
  }
  return output_lanelets;
}

lanelet::ConstLanelets getLeftLineSharingLanelets(
  const lanelet::ConstLanelet & current_lanelet, const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelets output_lanelets;
  lanelet::Lanelets left_lane_candidates =
    lanelet_map_ptr->laneletLayer.findUsages(current_lanelet.leftBound());
  for (auto & candidate : left_lane_candidates) {
    if (candidate == current_lanelet) continue;
    if (candidate.rightBound() == current_lanelet.leftBound()) {
      output_lanelets.push_back(candidate);
    }
  }
  return output_lanelets;
}

bool isIsolatedLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::routing::RoutingGraphPtr & graph)
{
  const auto & following_lanelets = graph->following(lanelet);
  const auto & left_lanelets = graph->lefts(lanelet);
  const auto & right_lanelets = graph->rights(lanelet);
  return left_lanelets.empty() && right_lanelets.empty() && following_lanelets.empty();
}

lanelet::routing::LaneletPaths getPossiblePathsForIsolatedLanelet(
  const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets possible_lanelets;
  possible_lanelets.push_back(lanelet);
  lanelet::routing::LaneletPaths possible_paths;
  lanelet::routing::LaneletPath possible_path(possible_lanelets);
  possible_paths.push_back(possible_path);
  return possible_paths;
}

bool validateIsolatedLaneletLength(
  const lanelet::ConstLanelet & lanelet, const TrackedObject & object, const double prediction_time)
{
  const auto & center_line = lanelet.centerline2d();
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const lanelet::BasicPoint2d obj_point(obj_pos.x, obj_pos.y);
  const auto & end_point = center_line.back();
  const double approx_distance = lanelet::geometry::distance2d(obj_point, end_point);
  const double abs_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  const double min_length = abs_speed * prediction_time;
  return approx_distance > min_length;
}

void replaceObjectYawWithLaneletsYaw(
  const LaneletsData & current_lanelets, TrackedObject & transformed_object)
{
  if (current_lanelets.empty()) return;
  auto & pose_with_cov = transformed_object.kinematics.pose_with_covariance;
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto & current_lanelet : current_lanelets) {
    const auto lanelet_angle = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      current_lanelet.lanelet,
      autoware::experimental::lanelet2_utils::from_ros(pose_with_cov.pose).basicPoint());
    sum_x += std::cos(lanelet_angle);
    sum_y += std::sin(lanelet_angle);
  }
  const double mean_yaw_angle = std::atan2(sum_y, sum_x);
  double roll, pitch, yaw;
  tf2::Quaternion original_quaternion;
  tf2::fromMsg(pose_with_cov.pose.orientation, original_quaternion);
  tf2::Matrix3x3(original_quaternion).getRPY(roll, pitch, yaw);
  tf2::Quaternion filtered_quaternion;
  filtered_quaternion.setRPY(roll, pitch, mean_yaw_angle);
  pose_with_cov.pose.orientation = tf2::toMsg(filtered_quaternion);
}
}  // namespace

PathProcessor::PathProcessor(autoware::agnocast_wrapper::Node & node) : node_(node)
{
}

void PathProcessor::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = time_keeper_ptr;
  if (path_generator_) path_generator_->setTimeKeeper(time_keeper_);
}

void PathProcessor::setLaneletMap(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr,
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  routing_graph_ptr_ = routing_graph_ptr;
  traffic_rules_ptr_ = traffic_rules_ptr;
  lru_cache_of_convert_path_type_.clear();
}

void PathProcessor::setManeuverPredictor(ManeuverPredictor & maneuver_predictor)
{
  maneuver_predictor_ = &maneuver_predictor;
}

void PathProcessor::setObjectTracker(ObjectTracker & object_tracker)
{
  object_tracker_ = &object_tracker;
}

void PathProcessor::setParams(const Params & params)
{
  const bool recreate_generator =
    !path_generator_ ||
    std::abs(params_.prediction_sampling_time_interval - params.prediction_sampling_time_interval) >
      1e-9;

  params_ = params;

  if (recreate_generator) {
    path_generator_ = std::make_shared<PathGenerator>(params_.prediction_sampling_time_interval);
    if (time_keeper_) path_generator_->setTimeKeeper(time_keeper_);
  }

  path_generator_->setUseVehicleAcceleration(params_.use_vehicle_acceleration);
  path_generator_->setAccelerationHalfLife(params_.acceleration_exponential_half_life);
}

void PathProcessor::clearLRUCache()
{
  lru_cache_of_convert_path_type_.clear();
}

std::optional<PredictedObject> PathProcessor::predict(
  const std_msgs::msg::Header & header, const TrackedObject & transformed_object,
  const double objects_detected_time, visualization_msgs::msg::MarkerArray * debug_markers)
{
  auto object = transformed_object;

  object_tracker_->updateObjectData(object);

  const auto current_lanelets = object_tracker_->getCurrentLanelets(object);

  object_tracker_->updateRoadUsersHistory(header, object, current_lanelets);

  if (current_lanelets.empty()) {
    PredictedPath predicted_path =
      path_generator_->generatePathForOffLaneVehicle(object, params_.prediction_time_horizon);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) return std::nullopt;

    auto predicted_object_vehicle = utils::convertToPredictedObject(object);
    predicted_object_vehicle.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_object_vehicle;
  }

  const double abs_obj_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  if (std::fabs(abs_obj_speed) < params_.min_velocity_for_map_based_prediction) {
    PredictedPath predicted_path =
      path_generator_->generatePathForLowSpeedVehicle(object, params_.prediction_time_horizon);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) return std::nullopt;

    auto predicted_slow_object = utils::convertToPredictedObject(object);
    predicted_slow_object.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_slow_object;
  }

  std::vector<LaneletPathWithPathInfo> suppressed_paths;
  const auto lanelet_ref_paths = getPredictedReferencePath(
    object, current_lanelets, objects_detected_time, params_.prediction_time_horizon,
    &suppressed_paths);
  return predictOnLane(
    object, current_lanelets, lanelet_ref_paths, suppressed_paths, debug_markers);
}

std::optional<PredictedObject> PathProcessor::predictOnLane(
  const TrackedObject & object, const LaneletsData & current_lanelets,
  const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths,
  const std::vector<LaneletPathWithPathInfo> & suppressed_paths,
  visualization_msgs::msg::MarkerArray * debug_markers, size_t * processing_passes)
{
  const double abs_obj_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  auto ref_paths = convertPredictedReferencePath(object, lanelet_ref_paths);

  TrackedObject yaw_fixed_object = object;
  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE) {
    replaceObjectYawWithLaneletsYaw(current_lanelets, yaw_fixed_object);
  }

  OnLanePathCandidates candidates;
  if (processing_passes) ++*processing_passes;
  generateOnLanePaths(yaw_fixed_object, abs_obj_speed, ref_paths, candidates);
  // Held LC candidates are restored only for origin lanelets whose lane-follow prediction
  // did not survive; origins with an accepted center keep their suppression.
  const auto paths_to_restore =
    detail::selectPathsToRestore(suppressed_paths, candidates.surviving_center_origins);
  if (!paths_to_restore.empty()) {
    const auto restored_paths = convertPredictedReferencePath(object, paths_to_restore);
    if (processing_passes) ++*processing_passes;
    generateOnLanePaths(yaw_fixed_object, abs_obj_speed, restored_paths, candidates);
    ref_paths.insert(ref_paths.end(), restored_paths.begin(), restored_paths.end());
  }

  if (debug_markers && !ref_paths.empty()) {
    const auto max_prob_path = std::max_element(
      ref_paths.begin(), ref_paths.end(),
      [](const PredictedRefPath & a, const PredictedRefPath & b) {
        return a.probability < b.probability;
      });
    debug_markers->markers.push_back(
      DebugModule::getDebugMarker(object, max_prob_path->maneuver, debug_markers->markers.size()));
  }

  auto & predicted_paths = candidates.predicted_paths;
  if (predicted_paths.empty()) {
    if (candidates.path_with_smallest_avg_curvature) {
      predicted_paths.push_back(*candidates.path_with_smallest_avg_curvature);
    } else {
      PredictedPath straight_path = path_generator_->generatePathForOffLaneVehicle(
        yaw_fixed_object, params_.prediction_time_horizon);
      straight_path.confidence = 1.0;
      if (straight_path.path.empty()) return std::nullopt;
      predicted_paths.push_back(straight_path);
    }
  }

  float sum_confidence = 0.0;
  for (const auto & predicted_path : predicted_paths) {
    sum_confidence += predicted_path.confidence;
  }
  const float min_sum_confidence_value = 1e-3;
  sum_confidence = std::max(sum_confidence, min_sum_confidence_value);

  auto predicted_object = utils::convertToPredictedObject(object);

  for (auto & predicted_path : predicted_paths) {
    predicted_path.confidence = predicted_path.confidence / sum_confidence;
    if (predicted_object.kinematics.predicted_paths.size() >= 100) break;
    predicted_object.kinematics.predicted_paths.push_back(predicted_path);
  }
  return predicted_object;
}

void PathProcessor::generateOnLanePaths(
  const TrackedObject & yaw_fixed_object, const double abs_obj_speed,
  const std::vector<PredictedRefPath> & reference_paths, OnLanePathCandidates & candidates)
{
  // A one-point lane-follow reference yields the straight fallback of the path generator; it is
  // still emitted as before but does not count as a surviving map-following center.
  const auto record_center_survival = [&candidates](const PredictedRefPath & ref_path) {
    if (ref_path.maneuver == Maneuver::LANE_FOLLOW && ref_path.path.size() >= 2) {
      candidates.surviving_center_origins.insert(ref_path.origin_lanelet_id);
    }
  };
  for (const auto & ref_path : reference_paths) {
    PredictedPath predicted_path = path_generator_->generatePathForOnLaneVehicle(
      yaw_fixed_object, ref_path.path, params_.prediction_time_horizon,
      params_.lateral_control_time_horizon, ref_path.width, ref_path.speed_limit);
    if (predicted_path.path.empty()) continue;

    if (!params_.check_lateral_acceleration_constraints) {
      predicted_path.confidence = ref_path.probability;
      candidates.predicted_paths.push_back(predicted_path);
      record_center_survival(ref_path);
      continue;
    }

    const auto trajectory_with_const_velocity = toTrajectoryPoints(predicted_path, abs_obj_speed);

    if (
      isLateralAccelerationConstraintSatisfied(
        trajectory_with_const_velocity, params_.prediction_sampling_time_interval)) {
      predicted_path.confidence = ref_path.probability;
      candidates.predicted_paths.push_back(predicted_path);
      record_center_survival(ref_path);
      continue;
    }

    constexpr double curvature_calculation_distance = 2.0;
    constexpr double points_interval = 1.0;
    const size_t idx_dist = static_cast<size_t>(
      std::max(static_cast<int>((curvature_calculation_distance) / points_interval), 1));
    const auto curvature_v =
      calcTrajectoryCurvatureFrom3Points(trajectory_with_const_velocity, idx_dist);
    if (curvature_v.empty()) continue;

    const auto curvature_avg =
      std::accumulate(curvature_v.begin(), curvature_v.end(), 0.0) / curvature_v.size();
    if (curvature_avg < candidates.min_avg_curvature) {
      candidates.min_avg_curvature = curvature_avg;
      candidates.path_with_smallest_avg_curvature = predicted_path;
      candidates.path_with_smallest_avg_curvature->confidence = ref_path.probability;
    }
  }
}

std::optional<size_t> PathProcessor::searchProperStartingRefPathIndex(
  const TrackedObject & object, const PosePath & pose_path) const
{
  std::unique_ptr<ScopedTimeTrack> st1_ptr;
  if (time_keeper_) st1_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  bool is_position_found = false;
  std::optional<size_t> opt_index{std::nullopt};
  auto & index = opt_index.emplace(0);

  const auto obj_point = object.kinematics.pose_with_covariance.pose.position;
  {
    std::unique_ptr<ScopedTimeTrack> st2_ptr;
    if (time_keeper_)
      st2_ptr = std::make_unique<ScopedTimeTrack>("find_close_segment_index", *time_keeper_);
    double min_dist_sq = std::numeric_limits<double>::max();
    constexpr double acceptable_dist_sq = 1.0;
    for (size_t i = 0; i < pose_path.size(); i++) {
      const double dx = pose_path.at(i).position.x - obj_point.x;
      const double dy = pose_path.at(i).position.y - obj_point.y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        index = i;
      }
      if (dist_sq < acceptable_dist_sq) break;
    }
  }

  size_t idx = 0;
  {
    std::unique_ptr<ScopedTimeTrack> st3_ptr;
    if (time_keeper_)
      st3_ptr = std::make_unique<ScopedTimeTrack>("find_target_seg_index", *time_keeper_);

    constexpr double search_distance = 22.0;
    constexpr double yaw_diff_limit = M_PI / 3.0;

    const double obj_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const size_t search_segment_count =
      static_cast<size_t>(std::floor(search_distance / params_.reference_path_resolution));
    const size_t search_segment_num =
      std::min(search_segment_count, static_cast<size_t>(pose_path.size() - index));

    double best_score = 1e9;
    for (size_t i = 0; i < search_segment_num; ++i) {
      const auto & path_pose = pose_path.at(index + i);
      const double path_yaw = tf2::getYaw(path_pose.orientation);
      const double relative_path_yaw = autoware_utils::normalize_radian(path_yaw - obj_yaw);
      if (std::abs(relative_path_yaw) > yaw_diff_limit) continue;

      const double dx = path_pose.position.x - obj_point.x;
      const double dy = path_pose.position.y - obj_point.y;
      const double dx_cp = std::cos(obj_yaw) * dx + std::sin(obj_yaw) * dy;
      const double dy_cp = -std::sin(obj_yaw) * dx + std::cos(obj_yaw) * dy;
      const double neutral_yaw = std::atan2(dy_cp, dx_cp) * 2.0;
      const double delta_yaw = autoware_utils::normalize_radian(path_yaw - obj_yaw - neutral_yaw);
      if (std::abs(delta_yaw) > yaw_diff_limit) continue;

      constexpr double weight_ratio = 0.01;
      double score = delta_yaw * delta_yaw + weight_ratio * neutral_yaw * neutral_yaw;
      constexpr double acceptable_score = 1e-3;

      if (score < best_score) {
        best_score = score;
        idx = i;
        is_position_found = true;
        if (score < acceptable_score) break;
      }
    }
  }

  index += idx;
  index = std::clamp(index, 0ul, pose_path.size() - 1);

  return is_position_found ? opt_index : std::nullopt;
}

PathProcessor::ReferencePathSearchContext PathProcessor::makeReferencePathSearchContext(
  const TrackedObject & object, const double time_horizon) const
{
  ReferencePathSearchContext context;
  context.obj_vel = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  context.obj_acc = params_.use_vehicle_acceleration
                      ? std::hypot(
                          object.kinematics.acceleration_with_covariance.accel.linear.x,
                          object.kinematics.acceleration_with_covariance.accel.linear.y)
                      : 0.0;
  context.time_horizon = time_horizon;
  context.lambda = std::log(2) / params_.acceleration_exponential_half_life;
  context.validate_time_horizon =
    time_horizon * params_.prediction_time_horizon_rate_for_validate_lane_length;
  context.final_speed_after_acceleration =
    context.obj_vel +
    context.obj_acc * (1.0 / context.lambda) * (1.0 - std::exp(-context.lambda * time_horizon));
  return context;
}

double PathProcessor::calcRoutingCostLimit(
  const ReferencePathSearchContext & context, const lanelet::ConstLanelet & lanelet,
  const double target_speed_limit) const
{
  const double obj_vel = context.obj_vel;
  const double obj_acc = context.obj_acc;
  const double t_h = context.time_horizon;
  const double lambda = context.lambda;

  const auto get_search_distance_with_decaying_acc = [&]() -> double {
    const double acceleration_distance =
      obj_acc * (1.0 / lambda) * t_h +
      obj_acc * (1.0 / (lambda * lambda)) * std::expm1(-lambda * t_h);
    return acceleration_distance + obj_vel * t_h;
  };

  const auto get_search_distance_with_partial_acc = [&](const double final_speed) -> double {
    constexpr double epsilon = 1E-5;
    if (std::abs(obj_acc) < epsilon) return obj_vel * t_h;
    const double t_f = (-1.0 / lambda) * std::log(1 - ((final_speed - obj_vel) * lambda) / obj_acc);
    return obj_acc * (1.0 / lambda) * t_f +
           obj_acc * (1.0 / std::pow(lambda, 2)) * std::expm1(-lambda * t_f) + obj_vel * t_f +
           final_speed * (t_h - t_f);
  };

  const bool final_speed_surpasses_limit =
    context.final_speed_after_acceleration > target_speed_limit;
  const bool object_has_surpassed_limit_already = obj_vel > target_speed_limit;
  const double search_dist = (final_speed_surpasses_limit && !object_has_surpassed_limit_already)
                               ? get_search_distance_with_partial_acc(target_speed_limit)
                               : get_search_distance_with_decaying_acc();
  return search_dist + lanelet::geometry::length3d(lanelet);
}

lanelet::routing::LaneletPaths PathProcessor::getPathsForLanelet(
  const lanelet::ConstLanelet & lanelet,
  const lanelet::routing::PossiblePathsParams & possible_params, const TrackedObject & object,
  const double validate_time_horizon)
{
  if (!isIsolatedLanelet(lanelet, routing_graph_ptr_)) {
    return routing_graph_ptr_->possiblePaths(lanelet, possible_params);
  }
  if (!validateIsolatedLaneletLength(lanelet, object, validate_time_horizon)) {
    return lanelet::routing::LaneletPaths{};
  }
  return getPossiblePathsForIsolatedLanelet(lanelet);
}

std::optional<lanelet::ConstLanelet> PathProcessor::getLeftOrRightLanelet(
  const lanelet::ConstLanelet & lanelet, const bool get_left) const
{
  const auto opt =
    get_left ? routing_graph_ptr_->left(lanelet) : routing_graph_ptr_->right(lanelet);
  if (!!opt) return *opt;
  if (!params_.consider_only_routable_neighbours) {
    const auto adjacent = get_left ? routing_graph_ptr_->adjacentLeft(lanelet)
                                   : routing_graph_ptr_->adjacentRight(lanelet);
    if (!!adjacent) return *adjacent;
    const auto unconnected_lanelets = get_left
                                        ? getLeftLineSharingLanelets(lanelet, lanelet_map_ptr_)
                                        : getRightLineSharingLanelets(lanelet, lanelet_map_ptr_);
    if (!unconnected_lanelets.empty()) return unconnected_lanelets.front();
  }
  return std::nullopt;
}

PathProcessor::LaneletReferencePaths PathProcessor::collectLaneletReferencePaths(
  const TrackedObject & object, const LaneletData & current_lanelet_data,
  const ReferencePathSearchContext & context, const bool rescue_lane_end)
{
  LaneletReferencePaths result;

  const lanelet::traffic_rules::SpeedLimitInformation limit =
    traffic_rules_ptr_->speedLimit(current_lanelet_data.lanelet);
  const double target_speed_limit =
    static_cast<double>(limit.speedLimit.value()) * params_.speed_limit_multiplier;
  lanelet::routing::PossiblePathsParams possible_params{0, {}, 0, false, true};
  possible_params.routingCostLimit =
    calcRoutingCostLimit(context, current_lanelet_data.lanelet, target_speed_limit);

  const auto append_paths =
    [&](const lanelet::routing::LaneletPaths & paths, const Maneuver maneuver) {
      PredictedRefPath ref_path_info;
      ref_path_info.speed_limit = target_speed_limit;
      ref_path_info.maneuver = maneuver;
      ref_path_info.origin_lanelet_id = current_lanelet_data.lanelet.id();
      for (const auto & path : paths) result.paths.emplace_back(path, ref_path_info);
    };
  const auto get_side_paths = [&](const bool get_left) {
    const auto side_lanelet = getLeftOrRightLanelet(current_lanelet_data.lanelet, get_left);
    if (!side_lanelet) return lanelet::routing::LaneletPaths{};
    return getPathsForLanelet(
      *side_lanelet, possible_params, object, context.validate_time_horizon);
  };

  const auto left_paths = get_side_paths(true);
  result.left_exists = !left_paths.empty();
  append_paths(left_paths, Maneuver::LEFT_LANE_CHANGE);

  const auto right_paths = get_side_paths(false);
  result.right_exists = !right_paths.empty();
  append_paths(right_paths, Maneuver::RIGHT_LANE_CHANGE);

  const auto center_paths = getPathsForLanelet(
    current_lanelet_data.lanelet, possible_params, object, context.validate_time_horizon);
  result.center_exists = !center_paths.empty();
  if (rescue_lane_end) {
    result.lane_ends_within_horizon = detail::laneEndsWithinHorizon(
      current_lanelet_data.lanelet, object.kinematics.pose_with_covariance.pose.position,
      std::max(context.obj_vel, 1.0) *
        params_.lane_change_path_suppression_lane_end_rescue_time_horizon,
      *routing_graph_ptr_, nullptr, &result.lane_end_distance);
  }
  append_paths(center_paths, Maneuver::LANE_FOLLOW);
  return result;
}

void PathProcessor::updateFuturePossibleLanelets(
  const std::string & object_id, const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths)
{
  auto & history = object_tracker_->getHistory();
  if (history.count(object_id) == 0) return;
  std::vector<lanelet::ConstLanelet> & possible_lanelets =
    history.at(object_id).back().future_possible_lanelets;
  for (const auto & ref_path : lanelet_ref_paths) {
    for (const auto & lanelet : ref_path.first) {
      if (
        std::find(possible_lanelets.begin(), possible_lanelets.end(), lanelet) ==
        possible_lanelets.end()) {
        possible_lanelets.push_back(lanelet);
      }
    }
  }
}

void PathProcessor::logLaneChangeSuppressionDecisions(
  const std::string & object_id, const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths,
  const std::vector<detail::LaneChangePathSuppressionInfo> & path_origins)
{
  for (size_t i = 0; i < lanelet_ref_paths.size(); ++i) {
    if (lanelet_ref_paths[i].second.maneuver == Maneuver::LANE_FOLLOW) continue;
    const auto & origin = path_origins.at(i);
    const char * decision =
      !origin.center_exists                               ? "kept_no_center"
      : origin.detected_maneuver != Maneuver::LANE_FOLLOW ? "kept_detected_lc"
      : params_.lane_change_path_suppression_keep_when_lane_ends && origin.lane_ends_within_horizon
        ? "kept_lane_end"
        : "suppressed";
    RCLCPP_DEBUG(
      node_.get_logger(),
      "LC suppression object=%s current_lanelet=%s decision=%s lane_end_distance=%.3f",
      object_id.c_str(), std::to_string(origin.current_lanelet_id).c_str(), decision,
      origin.lane_end_distance);
  }
}

std::vector<LaneletPathWithPathInfo> PathProcessor::getPredictedReferencePath(
  const TrackedObject & object, const LaneletsData & current_lanelets_data,
  const double object_detected_time, const double time_horizon,
  std::vector<LaneletPathWithPathInfo> * suppressed_paths)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto context = makeReferencePathSearchContext(object, time_horizon);
  const std::string object_id = autoware_utils::to_hex_string(object.object_id);
  const geometry_msgs::msg::Pose & object_pose = object.kinematics.pose_with_covariance.pose;

  const auto label = autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  using autoware_perception_msgs::msg::ObjectClassification;
  const bool is_vehicle =
    label == ObjectClassification::CAR || label == ObjectClassification::TRUCK ||
    label == ObjectClassification::BUS || label == ObjectClassification::TRAILER ||
    label == ObjectClassification::MOTORCYCLE;
  const bool suppression_enabled = params_.lane_change_path_suppression_enable && is_vehicle;
  const bool rescue_lane_end =
    suppression_enabled && params_.lane_change_path_suppression_keep_when_lane_ends;

  std::vector<LaneletPathWithPathInfo> lanelet_ref_paths;
  std::vector<detail::LaneChangePathSuppressionInfo> path_origins;
  for (const auto & current_lanelet_data : current_lanelets_data) {
    auto candidates =
      collectLaneletReferencePaths(object, current_lanelet_data, context, rescue_lane_end);
    if (candidates.paths.empty()) continue;

    const Maneuver predicted_maneuver = maneuver_predictor_->predictObjectManeuver(
      object_id, object_pose, current_lanelet_data, object_detected_time,
      object_tracker_->getHistory());

    const float & path_prob = current_lanelet_data.probability;
    const auto maneuver_prob = maneuver_predictor_->calculateManeuverProbability(
      predicted_maneuver, candidates.left_exists, candidates.right_exists,
      candidates.center_exists);
    for (auto & ref_path : candidates.paths) {
      auto & ref_path_info = ref_path.second;
      ref_path_info.probability = maneuver_prob.at(ref_path_info.maneuver) * path_prob;
    }

    path_origins.insert(
      path_origins.end(), candidates.paths.size(),
      detail::LaneChangePathSuppressionInfo{
        predicted_maneuver, candidates.lane_ends_within_horizon, candidates.center_exists,
        current_lanelet_data.lanelet.id(), candidates.lane_end_distance});
    lanelet_ref_paths.insert(
      lanelet_ref_paths.end(), candidates.paths.begin(), candidates.paths.end());
  }

  updateFuturePossibleLanelets(object_id, lanelet_ref_paths);
  if (suppression_enabled) {
    logLaneChangeSuppressionDecisions(object_id, lanelet_ref_paths, path_origins);
  }

  // Preserve all reachable lanelets in history before suppressing output branches.
  return detail::suppressUndetectedLaneChangePaths(
    lanelet_ref_paths, path_origins, suppression_enabled,
    params_.lane_change_path_suppression_keep_when_lane_ends, suppressed_paths);
}

std::vector<PredictedRefPath> PathProcessor::convertPredictedReferencePath(
  const TrackedObject & object,
  const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::vector<PredictedRefPath> converted_ref_paths;

  for (const auto & ref_path : lanelet_ref_paths) {
    const auto & lanelet_path = ref_path.first;
    const auto & ref_path_info = ref_path.second;
    const auto converted_path = convertLaneletPathToPosePath(lanelet_path);
    PredictedRefPath predicted_path;
    predicted_path.probability = ref_path_info.probability;
    predicted_path.path = converted_path.first;
    predicted_path.width = converted_path.second;
    predicted_path.maneuver = ref_path_info.maneuver;
    predicted_path.speed_limit = ref_path_info.speed_limit;
    predicted_path.origin_lanelet_id = ref_path_info.origin_lanelet_id;
    converted_ref_paths.push_back(predicted_path);
  }

  for (auto it = converted_ref_paths.begin(); it != converted_ref_paths.end();) {
    auto & pose_path = it->path;
    if (pose_path.empty()) {
      it = converted_ref_paths.erase(it);
      continue;
    }
    const std::optional<size_t> opt_starting_idx =
      searchProperStartingRefPathIndex(object, pose_path);
    if (opt_starting_idx.has_value()) {
      pose_path.erase(pose_path.begin(), pose_path.begin() + opt_starting_idx.value());
      ++it;
    } else {
      it = converted_ref_paths.erase(it);
    }
  }

  return converted_ref_paths;
}

std::pair<PosePath, double> PathProcessor::convertLaneletPathToPosePath(
  const lanelet::routing::LaneletPath & path) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (lru_cache_of_convert_path_type_.contains(path)) {
    return *lru_cache_of_convert_path_type_.get(path);
  }

  std::pair<PosePath, double> converted_path_and_width;
  {
    PosePath converted_path;
    double width = 10.0;

    if (!path.empty()) {
      lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(path.front());
      if (!prev_lanelets.empty()) {
        lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
        bool init_flag = true;
        geometry_msgs::msg::Pose prev_p;
        for (const auto & lanelet_p : prev_lanelet.centerline()) {
          geometry_msgs::msg::Pose current_p;
          current_p.position = experimental::lanelet2_utils::to_ros(lanelet_p);
          if (init_flag) {
            init_flag = false;
            prev_p = current_p;
            continue;
          }
          const double lane_yaw = std::atan2(
            current_p.position.y - prev_p.position.y, current_p.position.x - prev_p.position.x);
          const double sin_yaw_half = std::sin(lane_yaw / 2.0);
          const double cos_yaw_half = std::cos(lane_yaw / 2.0);
          current_p.orientation.x = 0.0;
          current_p.orientation.y = 0.0;
          current_p.orientation.z = sin_yaw_half;
          current_p.orientation.w = cos_yaw_half;
          converted_path.push_back(current_p);
          prev_p = current_p;
        }
      }
    }

    for (const auto & lanelet : path) {
      bool init_flag = true;
      geometry_msgs::msg::Pose prev_p;
      for (const auto & lanelet_p : lanelet.centerline()) {
        geometry_msgs::msg::Pose current_p;
        current_p.position = experimental::lanelet2_utils::to_ros(lanelet_p);
        if (init_flag) {
          init_flag = false;
          prev_p = current_p;
          continue;
        }
        if (!converted_path.empty()) {
          const auto last_p = converted_path.back();
          const double tmp_dist = autoware_utils::calc_distance2d(last_p, current_p);
          if (tmp_dist < 1e-6) {
            prev_p = current_p;
            continue;
          }
        }
        const double lane_yaw = std::atan2(
          current_p.position.y - prev_p.position.y, current_p.position.x - prev_p.position.x);
        const double sin_yaw_half = std::sin(lane_yaw / 2.0);
        const double cos_yaw_half = std::cos(lane_yaw / 2.0);
        current_p.orientation.x = 0.0;
        current_p.orientation.y = 0.0;
        current_p.orientation.z = sin_yaw_half;
        current_p.orientation.w = cos_yaw_half;
        converted_path.push_back(current_p);
        prev_p = current_p;
      }
      const auto left_bound = lanelet.leftBound2d();
      const auto right_bound = lanelet.rightBound2d();
      const double lanelet_width_front = std::hypot(
        left_bound.front().x() - right_bound.front().x(),
        left_bound.front().y() - right_bound.front().y());
      width = std::min(width, lanelet_width_front);
    }

    // the options use_akima_spline_for_xy and use_lerp_for_z are set to true
    // but the implementation of use_akima_spline_for_xy in resamplePoseVector and
    // resamplePointVector is opposite to the options so the options are set to true to use linear
    // interpolation for xy
    const bool use_akima_spline_for_xy = true;
    const bool use_lerp_for_z = true;
    const auto resampled_converted_path = autoware::motion_utils::resamplePoseVector(
      converted_path, params_.reference_path_resolution, use_akima_spline_for_xy, use_lerp_for_z);
    converted_path_and_width = std::make_pair(resampled_converted_path, width);
  }

  lru_cache_of_convert_path_type_.put(path, converted_path_and_width);
  return converted_path_and_width;
}

std::vector<double> PathProcessor::calcTrajectoryCurvatureFrom3Points(
  const TrajectoryPoints & trajectory, size_t idx_dist)
{
  using autoware_utils::calc_curvature;
  using autoware_utils::get_point;

  if (trajectory.size() < 3) {
    return std::vector<double>(trajectory.size(), 0.0);
  }

  const auto max_idx_dist = static_cast<size_t>(std::floor((trajectory.size() - 1) / 2.0));
  idx_dist = std::max(1ul, std::min(idx_dist, max_idx_dist));

  if (idx_dist < 1) throw std::logic_error("idx_dist less than 1 is not expected");

  std::vector<double> k_arr(trajectory.size(), 0.0);

  for (size_t i = 1; i + 1 < trajectory.size(); i++) {
    double curvature = 0.0;
    const auto p0 = get_point(trajectory.at(i - std::min(idx_dist, i)));
    const auto p1 = get_point(trajectory.at(i));
    const auto p2 = get_point(trajectory.at(i + std::min(idx_dist, trajectory.size() - 1 - i)));
    try {
      curvature = calc_curvature(p0, p1, p2);
    } catch (std::exception const & e) {
      RCLCPP_WARN(node_.get_logger(), "%s", e.what());
      curvature = (i > 1) ? k_arr.at(i - 1) : 0.0;
    }
    k_arr.at(i) = curvature;
  }
  k_arr.at(0) = k_arr.at(1);
  k_arr.back() = k_arr.at(trajectory.size() - 2);

  return k_arr;
}

TrajectoryPoints PathProcessor::toTrajectoryPoints(
  const PredictedPath & path, const double velocity)
{
  TrajectoryPoints out_trajectory;
  std::for_each(path.path.begin(), path.path.end(), [&out_trajectory, velocity](const auto & pose) {
    TrajectoryPoint p;
    p.pose = pose;
    p.longitudinal_velocity_mps = velocity;
    out_trajectory.push_back(p);
  });
  return out_trajectory;
}

bool PathProcessor::isLateralAccelerationConstraintSatisfied(
  const TrajectoryPoints & trajectory, const double delta_time)
{
  constexpr double epsilon = 1E-6;
  if (delta_time < epsilon) throw std::invalid_argument("delta_time must be a positive value");

  if (trajectory.size() < 3) return true;
  const double max_lateral_accel_abs = std::fabs(params_.max_lateral_accel);

  double arc_length = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto current_pose = trajectory.at(i).pose;
    const auto next_pose = trajectory.at(i - 1).pose;
    const double delta_s = std::hypot(
      next_pose.position.x - current_pose.position.x,
      next_pose.position.y - current_pose.position.y);
    arc_length += delta_s;

    tf2::Quaternion q_current, q_next;
    tf2::convert(current_pose.orientation, q_current);
    tf2::convert(next_pose.orientation, q_next);
    double delta_theta = q_current.angleShortestPath(q_next);
    if (delta_theta > M_PI) {
      delta_theta -= 2.0 * M_PI;
    } else if (delta_theta < -M_PI) {
      delta_theta += 2.0 * M_PI;
    }

    const double yaw_rate = std::max(std::abs(delta_theta / delta_time), 1.0E-5);
    const double current_speed = std::abs(trajectory.at(i).longitudinal_velocity_mps);
    const double lateral_acceleration = std::abs(current_speed * yaw_rate);
    if (lateral_acceleration < max_lateral_accel_abs) continue;

    const double v_curvature_max = std::sqrt(max_lateral_accel_abs / yaw_rate);
    const double t = (v_curvature_max - current_speed) / params_.min_acceleration_before_curve;
    const double distance_to_slow_down =
      current_speed * t + 0.5 * params_.min_acceleration_before_curve * std::pow(t, 2);
    if (distance_to_slow_down > arc_length) return false;
  }
  return true;
}

}  // namespace autoware::map_based_prediction
