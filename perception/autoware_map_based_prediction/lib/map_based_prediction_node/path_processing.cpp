// Copyright 2021 Tier IV, Inc.
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

#include "autoware/map_based_prediction/map_based_prediction_node/path_processing.hpp"

#include "autoware/map_based_prediction/utils.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <autoware_perception_msgs/msg/tracked_object_kinematics.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

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

PredictedObject MapBasedPredictionNode::getPredictionForNonVehicleObject(
  const std_msgs::msg::Header & header, const TrackedObject & object)
{
  return predictor_vru_->predict(header, object);
}

std::optional<PredictedObject> MapBasedPredictionNode::getPredictionForVehicleObject(
  const std_msgs::msg::Header & header, const TrackedObject & transformed_object,
  const double objects_detected_time, visualization_msgs::msg::MarkerArray & debug_markers)
{
  auto object = transformed_object;

  // Update object yaw and velocity
  updateObjectData(object);

  // Get Closest Lanelet
  const auto current_lanelets = utils::getCurrentLanelets(
    object, lanelet_map_ptr_, road_users_history_, dist_threshold_for_searching_lanelet_,
    delta_yaw_threshold_for_searching_lanelet_, sigma_lateral_offset_, sigma_yaw_angle_deg_);

  // Update Objects History
  updateRoadUsersHistory(header, object, current_lanelets);

  // For off lane obstacles
  if (current_lanelets.empty()) {
    PredictedPath predicted_path =
      path_generator_->generatePathForOffLaneVehicle(object, prediction_time_horizon_.vehicle);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) {
      return std::nullopt;
    }

    auto predicted_object_vehicle = utils::convertToPredictedObject(object);
    predicted_object_vehicle.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_object_vehicle;
  }

  // For too-slow vehicle
  const double abs_obj_speed = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  if (std::fabs(abs_obj_speed) < min_velocity_for_map_based_prediction_) {
    PredictedPath predicted_path =
      path_generator_->generatePathForLowSpeedVehicle(object, prediction_time_horizon_.vehicle);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) {
      return std::nullopt;
    }

    auto predicted_slow_object = utils::convertToPredictedObject(object);
    predicted_slow_object.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_slow_object;
  }

  // Get Predicted Reference Path for Each Maneuver and current lanelets
  // return: <probability, paths>
  const auto lanelet_ref_paths = getPredictedReferencePath(
    object, current_lanelets, objects_detected_time, prediction_time_horizon_.vehicle);
  const auto ref_paths = convertPredictedReferencePath(object, lanelet_ref_paths);

  // If predicted reference path is empty, assume this object is out of the lane
  if (ref_paths.empty()) {
    PredictedPath predicted_path =
      path_generator_->generatePathForOffLaneVehicle(object, prediction_time_horizon_.vehicle);
    predicted_path.confidence = 1.0;
    if (predicted_path.path.empty()) {
      return std::nullopt;
    }

    auto predicted_object_out_of_lane = utils::convertToPredictedObject(object);
    predicted_object_out_of_lane.kinematics.predicted_paths.push_back(predicted_path);
    return predicted_object_out_of_lane;
  }

  // Get Debug Marker for On Lane Vehicles
  if (pub_debug_markers_) {
    const auto max_prob_path = std::max_element(
      ref_paths.begin(), ref_paths.end(),
      [](const PredictedRefPath & a, const PredictedRefPath & b) {
        return a.probability < b.probability;
      });
    const auto debug_marker =
      getDebugMarker(object, max_prob_path->maneuver, debug_markers.markers.size());
    debug_markers.markers.push_back(debug_marker);
  }

  // Fix object angle if its orientation unreliable (e.g. far object by radar sensor)
  // This prevent bending predicted path
  TrackedObject yaw_fixed_object = object;
  if (
    object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE) {
    replaceObjectYawWithLaneletsYaw(current_lanelets, yaw_fixed_object);
  }
  // Generate Predicted Path
  std::vector<PredictedPath> predicted_paths;
  double min_avg_curvature = std::numeric_limits<double>::max();
  PredictedPath path_with_smallest_avg_curvature;

  for (const auto & ref_path : ref_paths) {
    PredictedPath predicted_path = path_generator_->generatePathForOnLaneVehicle(
      yaw_fixed_object, ref_path.path, prediction_time_horizon_.vehicle,
      lateral_control_time_horizon_, ref_path.width, ref_path.speed_limit);
    if (predicted_path.path.empty()) continue;

    if (!check_lateral_acceleration_constraints_) {
      predicted_path.confidence = ref_path.probability;
      predicted_paths.push_back(predicted_path);
      continue;
    }

    // Check lat. acceleration constraints
    const auto trajectory_with_const_velocity = toTrajectoryPoints(predicted_path, abs_obj_speed);

    if (isLateralAccelerationConstraintSatisfied(
          trajectory_with_const_velocity, prediction_sampling_time_interval_)) {
      predicted_path.confidence = ref_path.probability;
      predicted_paths.push_back(predicted_path);
      continue;
    }

    // Calculate curvature assuming the trajectory points interval is constant
    // In case all paths are deleted, a copy of the straightest path is kept

    constexpr double curvature_calculation_distance = 2.0;
    constexpr double points_interval = 1.0;
    const size_t idx_dist = static_cast<size_t>(
      std::max(static_cast<int>((curvature_calculation_distance) / points_interval), 1));
    const auto curvature_v =
      calcTrajectoryCurvatureFrom3Points(trajectory_with_const_velocity, idx_dist);
    if (curvature_v.empty()) {
      continue;
    }
    const auto curvature_avg =
      std::accumulate(curvature_v.begin(), curvature_v.end(), 0.0) / curvature_v.size();
    if (curvature_avg < min_avg_curvature) {
      min_avg_curvature = curvature_avg;
      path_with_smallest_avg_curvature = predicted_path;
      path_with_smallest_avg_curvature.confidence = ref_path.probability;
    }
  }

  if (predicted_paths.empty()) predicted_paths.push_back(path_with_smallest_avg_curvature);
  // Normalize Path Confidence and output the predicted object

  float sum_confidence = 0.0;
  for (const auto & predicted_path : predicted_paths) {
    sum_confidence += predicted_path.confidence;
  }
  const float min_sum_confidence_value = 1e-3;
  sum_confidence = std::max(sum_confidence, min_sum_confidence_value);

  auto predicted_object = utils::convertToPredictedObject(transformed_object);

  for (auto & predicted_path : predicted_paths) {
    predicted_path.confidence = predicted_path.confidence / sum_confidence;
    if (predicted_object.kinematics.predicted_paths.size() >= 100) break;
    predicted_object.kinematics.predicted_paths.push_back(predicted_path);
  }
  return predicted_object;
}

std::optional<size_t> MapBasedPredictionNode::searchProperStartingRefPathIndex(
  const TrackedObject & object, const PosePath & pose_path) const
{
  std::unique_ptr<ScopedTimeTrack> st1_ptr;
  if (time_keeper_) st1_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  bool is_position_found = false;
  std::optional<size_t> opt_index{std::nullopt};
  auto & index = opt_index.emplace(0);

  // starting segment index is a segment close enough to the object
  const auto obj_point = object.kinematics.pose_with_covariance.pose.position;
  {
    std::unique_ptr<ScopedTimeTrack> st2_ptr;
    if (time_keeper_)
      st2_ptr = std::make_unique<ScopedTimeTrack>("find_close_segment_index", *time_keeper_);
    double min_dist_sq = std::numeric_limits<double>::max();
    constexpr double acceptable_dist_sq = 1.0;  // [m2]
    for (size_t i = 0; i < pose_path.size(); i++) {
      const double dx = pose_path.at(i).position.x - obj_point.x;
      const double dy = pose_path.at(i).position.y - obj_point.y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        index = i;
      }
      if (dist_sq < acceptable_dist_sq) {
        break;
      }
    }
  }

  // calculate score that object can reach the target path smoothly, and search the
  // starting segment index that have the best score
  size_t idx = 0;
  {  // find target segmentation index
    std::unique_ptr<ScopedTimeTrack> st3_ptr;
    if (time_keeper_)
      st3_ptr = std::make_unique<ScopedTimeTrack>("find_target_seg_index", *time_keeper_);

    constexpr double search_distance = 22.0;       // [m]
    constexpr double yaw_diff_limit = M_PI / 3.0;  // 60 degrees

    const double obj_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const size_t search_segment_count =
      static_cast<size_t>(std::floor(search_distance / reference_path_resolution_));
    const size_t search_segment_num =
      std::min(search_segment_count, static_cast<size_t>(pose_path.size() - index));

    // search for the best score, which is the smallest
    double best_score = 1e9;  // initial value is large enough
    for (size_t i = 0; i < search_segment_num; ++i) {
      const auto & path_pose = pose_path.at(index + i);
      // yaw difference
      const double path_yaw = tf2::getYaw(path_pose.orientation);
      const double relative_path_yaw = autoware_utils::normalize_radian(path_yaw - obj_yaw);
      if (std::abs(relative_path_yaw) > yaw_diff_limit) {
        continue;
      }

      const double dx = path_pose.position.x - obj_point.x;
      const double dy = path_pose.position.y - obj_point.y;
      const double dx_cp = std::cos(obj_yaw) * dx + std::sin(obj_yaw) * dy;
      const double dy_cp = -std::sin(obj_yaw) * dx + std::cos(obj_yaw) * dy;
      const double neutral_yaw = std::atan2(dy_cp, dx_cp) * 2.0;
      const double delta_yaw = autoware_utils::normalize_radian(path_yaw - obj_yaw - neutral_yaw);
      if (std::abs(delta_yaw) > yaw_diff_limit) {
        continue;
      }

      // objective function score
      constexpr double weight_ratio = 0.01;
      double score = delta_yaw * delta_yaw + weight_ratio * neutral_yaw * neutral_yaw;
      constexpr double acceptable_score = 1e-3;

      if (score < best_score) {
        best_score = score;
        idx = i;
        is_position_found = true;
        if (score < acceptable_score) {
          // if the score is small enough, we can break the loop
          break;
        }
      }
    }
  }

  // update starting segment index
  index += idx;
  index = std::clamp(index, 0ul, pose_path.size() - 1);

  return is_position_found ? opt_index : std::nullopt;
}

std::vector<LaneletPathWithPathInfo> MapBasedPredictionNode::getPredictedReferencePath(
  const TrackedObject & object, const LaneletsData & current_lanelets_data,
  const double object_detected_time, const double time_horizon)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Step 1. Set conditions for the prediction
  const double obj_vel = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);

  // Using a decaying acceleration model. Consult the README for more information about the model.
  const double obj_acc = (use_vehicle_acceleration_)
                           ? std::hypot(
                               object.kinematics.acceleration_with_covariance.accel.linear.x,
                               object.kinematics.acceleration_with_covariance.accel.linear.y)
                           : 0.0;
  const double t_h = time_horizon;
  const double lambda = std::log(2) / acceleration_exponential_half_life_;
  const double validate_time_horizon = t_h * prediction_time_horizon_rate_for_validate_lane_length_;
  const double final_speed_after_acceleration =
    obj_vel + obj_acc * (1.0 / lambda) * (1.0 - std::exp(-lambda * t_h));

  auto get_search_distance_with_decaying_acc = [&]() -> double {
    const double acceleration_distance =
      obj_acc * (1.0 / lambda) * t_h +
      obj_acc * (1.0 / (lambda * lambda)) * std::expm1(-lambda * t_h);
    double search_dist = acceleration_distance + obj_vel * t_h;
    return search_dist;
  };

  auto get_search_distance_with_partial_acc = [&](const double final_speed) -> double {
    constexpr double epsilon = 1E-5;
    if (std::abs(obj_acc) < epsilon) {
      // Assume constant speed
      return obj_vel * t_h;
    }
    // Time to reach final speed
    const double t_f = (-1.0 / lambda) * std::log(1 - ((final_speed - obj_vel) * lambda) / obj_acc);
    // It is assumed the vehicle accelerates until final_speed is reached and
    // then continues at constant speed for the rest of the time horizon
    const double search_dist =
      // Distance covered while accelerating
      obj_acc * (1.0 / lambda) * t_f +
      obj_acc * (1.0 / std::pow(lambda, 2)) * std::expm1(-lambda * t_f) + obj_vel * t_f +
      // Distance covered at constant speed
      final_speed * (t_h - t_f);
    return search_dist;
  };

  std::string object_id = autoware_utils::to_hex_string(object.object_id);
  geometry_msgs::msg::Pose object_pose = object.kinematics.pose_with_covariance.pose;

  // Step 2. Get possible paths for each lanelet
  std::vector<LaneletPathWithPathInfo> lanelet_ref_paths;
  for (const auto & current_lanelet_data : current_lanelets_data) {
    std::vector<LaneletPathWithPathInfo> ref_paths_per_lanelet;

    // Set condition on each lanelet
    lanelet::routing::PossiblePathsParams possible_params{0, {}, 0, false, true};
    double target_speed_limit = 0.0;
    {
      const lanelet::traffic_rules::SpeedLimitInformation limit =
        traffic_rules_ptr_->speedLimit(current_lanelet_data.lanelet);
      const double legal_speed_limit = static_cast<double>(limit.speedLimit.value());
      target_speed_limit = legal_speed_limit * speed_limit_multiplier_;
      const bool final_speed_surpasses_limit = final_speed_after_acceleration > target_speed_limit;
      const bool object_has_surpassed_limit_already = obj_vel > target_speed_limit;

      double search_dist = (final_speed_surpasses_limit && !object_has_surpassed_limit_already)
                             ? get_search_distance_with_partial_acc(target_speed_limit)
                             : get_search_distance_with_decaying_acc();
      search_dist += lanelet::geometry::length3d(current_lanelet_data.lanelet);
      possible_params.routingCostLimit = search_dist;
    }

    // lambda function to get possible paths for isolated lanelet
    // isolated is often caused by lanelet with no connection e.g. shoulder-lane
    auto getPathsForNormalOrIsolatedLanelet = [&](const lanelet::ConstLanelet & lanelet) {
      // if lanelet is not isolated, return normal possible paths
      if (!isIsolatedLanelet(lanelet, routing_graph_ptr_)) {
        return routing_graph_ptr_->possiblePaths(lanelet, possible_params);
      }
      // if lanelet is isolated, check if it has enough length
      if (!validateIsolatedLaneletLength(lanelet, object, validate_time_horizon)) {
        return lanelet::routing::LaneletPaths{};
      } else {
        // if lanelet has enough length, return possible paths
        return getPossiblePathsForIsolatedLanelet(lanelet);
      }
    };

    // lambda function to extract left/right lanelets
    auto getLeftOrRightLanelets = [&](
                                    const lanelet::ConstLanelet & lanelet,
                                    const bool get_left) -> std::optional<lanelet::ConstLanelet> {
      const auto opt =
        get_left ? routing_graph_ptr_->left(lanelet) : routing_graph_ptr_->right(lanelet);
      if (!!opt) {
        return *opt;
      }
      if (!consider_only_routable_neighbours_) {
        const auto adjacent = get_left ? routing_graph_ptr_->adjacentLeft(lanelet)
                                       : routing_graph_ptr_->adjacentRight(lanelet);
        if (!!adjacent) {
          return *adjacent;
        }
        // search for unconnected lanelet
        const auto unconnected_lanelets =
          get_left ? getLeftLineSharingLanelets(lanelet, lanelet_map_ptr_)
                   : getRightLineSharingLanelets(lanelet, lanelet_map_ptr_);
        // just return first candidate of unconnected lanelet for now
        if (!unconnected_lanelets.empty()) {
          return unconnected_lanelets.front();
        }
      }

      // if no candidate lanelet found, return empty
      return std::nullopt;
    };

    bool left_paths_exists = false;
    bool right_paths_exists = false;
    bool center_paths_exists = false;

    // a-1. Get the left lanelet
    {
      PredictedRefPath ref_path_info;
      lanelet::routing::LaneletPaths left_paths;
      const auto left_lanelet = getLeftOrRightLanelets(current_lanelet_data.lanelet, true);
      if (!!left_lanelet) {
        left_paths = getPathsForNormalOrIsolatedLanelet(left_lanelet.value());
        left_paths_exists = !left_paths.empty();
      }
      ref_path_info.speed_limit = target_speed_limit;
      ref_path_info.maneuver = Maneuver::LEFT_LANE_CHANGE;
      for (auto & path : left_paths) {
        ref_paths_per_lanelet.emplace_back(path, ref_path_info);
      }
    }

    // a-2. Get the right lanelet
    {
      PredictedRefPath ref_path_info;
      lanelet::routing::LaneletPaths right_paths;
      const auto right_lanelet = getLeftOrRightLanelets(current_lanelet_data.lanelet, false);
      if (!!right_lanelet) {
        right_paths = getPathsForNormalOrIsolatedLanelet(right_lanelet.value());
        right_paths_exists = !right_paths.empty();
      }
      ref_path_info.speed_limit = target_speed_limit;
      ref_path_info.maneuver = Maneuver::RIGHT_LANE_CHANGE;
      for (auto & path : right_paths) {
        ref_paths_per_lanelet.emplace_back(path, ref_path_info);
      }
    }

    // a-3. Get the center lanelet
    {
      PredictedRefPath ref_path_info;
      lanelet::routing::LaneletPaths center_paths =
        getPathsForNormalOrIsolatedLanelet(current_lanelet_data.lanelet);
      center_paths_exists = !center_paths.empty();
      ref_path_info.speed_limit = target_speed_limit;
      ref_path_info.maneuver = Maneuver::LANE_FOLLOW;
      for (auto & path : center_paths) {
        ref_paths_per_lanelet.emplace_back(path, ref_path_info);
      }
    }

    // Skip calculations if all paths are empty
    if (ref_paths_per_lanelet.empty()) {
      continue;
    }

    // b. Predict Object Maneuver
    const Maneuver predicted_maneuver =
      predictObjectManeuver(object_id, object_pose, current_lanelet_data, object_detected_time);

    // c. Allocate probability for each predicted maneuver
    const float & path_prob = current_lanelet_data.probability;
    const auto maneuver_prob = calculateManeuverProbability(
      predicted_maneuver, left_paths_exists, right_paths_exists, center_paths_exists);
    for (auto & ref_path : ref_paths_per_lanelet) {
      auto & ref_path_info = ref_path.second;
      ref_path_info.probability = maneuver_prob.at(ref_path_info.maneuver) * path_prob;
    }

    // move the calculated ref paths to the lanelet_ref_paths
    lanelet_ref_paths.insert(
      lanelet_ref_paths.end(), ref_paths_per_lanelet.begin(), ref_paths_per_lanelet.end());
  }

  // update future possible lanelets
  if (road_users_history_.count(object_id) != 0) {
    std::vector<lanelet::ConstLanelet> & possible_lanelets =
      road_users_history_.at(object_id).back().future_possible_lanelets;
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

  return lanelet_ref_paths;
}

std::vector<PredictedRefPath> MapBasedPredictionNode::convertPredictedReferencePath(
  const TrackedObject & object,
  const std::vector<LaneletPathWithPathInfo> & lanelet_ref_paths) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::vector<PredictedRefPath> converted_ref_paths;

  // Step 1. Convert lanelet path to pose path
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
    converted_ref_paths.push_back(predicted_path);
  }

  // Step 2. Search starting point for each reference path
  for (auto it = converted_ref_paths.begin(); it != converted_ref_paths.end();) {
    auto & pose_path = it->path;
    if (pose_path.empty()) {
      continue;
    }

    const std::optional<size_t> opt_starting_idx =
      searchProperStartingRefPathIndex(object, pose_path);

    if (opt_starting_idx.has_value()) {
      // Trim the reference path
      pose_path.erase(pose_path.begin(), pose_path.begin() + opt_starting_idx.value());
      ++it;
    } else {
      // Proper starting point is not found, remove the reference path
      it = converted_ref_paths.erase(it);
    }
  }

  return converted_ref_paths;
}

std::pair<PosePath, double> MapBasedPredictionNode::convertLaneletPathToPosePath(
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
    double width = 10.0;  // Initialize with a large value

    // Insert Positions. Note that we start inserting points from previous lanelet
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

          // only considers yaw of the lanelet
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

        // Prevent from inserting same points
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

      // Update minimum width
      const auto left_bound = lanelet.leftBound2d();
      const auto right_bound = lanelet.rightBound2d();
      const double lanelet_width_front = std::hypot(
        left_bound.front().x() - right_bound.front().x(),
        left_bound.front().y() - right_bound.front().y());
      width = std::min(width, lanelet_width_front);
    }

    // Resample Path
    const bool use_akima_spline_for_xy = true;
    const bool use_lerp_for_z = true;
    // the options use_akima_spline_for_xy and use_lerp_for_z are set to true
    // but the implementation of use_akima_spline_for_xy in resamplePoseVector and
    // resamplePointVector is opposite to the options so the options are set to true to use linear
    // interpolation for xy
    const auto resampled_converted_path = autoware::motion_utils::resamplePoseVector(
      converted_path, reference_path_resolution_, use_akima_spline_for_xy, use_lerp_for_z);
    converted_path_and_width = std::make_pair(resampled_converted_path, width);
  }

  lru_cache_of_convert_path_type_.put(path, converted_path_and_width);
  return converted_path_and_width;
}

}  // namespace autoware::map_based_prediction
