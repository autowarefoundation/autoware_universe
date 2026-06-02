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

#include "autoware/map_based_prediction/predictor_vehicle/maneuver_prediction.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <tf2/utils.hpp>

#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

ManeuverPredictor::ManeuverPredictor(rclcpp::Node & node) : node_(node)
{
}

void ManeuverPredictor::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = time_keeper_ptr;
}

void ManeuverPredictor::setRoutingGraph(
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr)
{
  routing_graph_ptr_ = routing_graph_ptr;
}

void ManeuverPredictor::setParams(const Params & params)
{
  params_ = params;
}

Maneuver ManeuverPredictor::predictObjectManeuver(
  const std::string & object_id, const geometry_msgs::msg::Pose & object_pose,
  const LaneletData & current_lanelet_data, const double object_detected_time,
  std::unordered_map<std::string, std::deque<RoadUser>> & history)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto current_maneuver = [&]() {
    if (params_.lane_change_detection_method == "time_to_change_lane") {
      return predictObjectManeuverByTimeToLaneChange(
        object_id, current_lanelet_data, object_detected_time, history);
    } else if (params_.lane_change_detection_method == "lat_diff_distance") {
      return predictObjectManeuverByLatDiffDistance(
        object_id, object_pose, current_lanelet_data, object_detected_time, history);
    }
    throw std::logic_error("Lane change detection method is invalid.");
  }();

  if (history.count(object_id) == 0) {
    return current_maneuver;
  }
  auto & object_info = history.at(object_id);

  if (!object_info.empty()) {
    object_info.back().one_shot_maneuver = current_maneuver;
  }

  if (object_info.size() < 2) {
    object_info.back().output_maneuver = current_maneuver;
    return current_maneuver;
  }
  // NOTE: The index of previous maneuver is not object_info.size() - 1
  const auto prev_output_maneuver =
    object_info.at(static_cast<int>(object_info.size()) - 2).output_maneuver;

  for (int i = 0;
       i < std::min(params_.num_continuous_state_transition, static_cast<int>(object_info.size()));
       ++i) {
    const auto & tmp_maneuver =
      object_info.at(static_cast<int>(object_info.size()) - 1 - i).one_shot_maneuver;
    if (tmp_maneuver != current_maneuver) {
      object_info.back().output_maneuver = prev_output_maneuver;
      return prev_output_maneuver;
    }
  }

  object_info.back().output_maneuver = current_maneuver;
  return current_maneuver;
}

Maneuver ManeuverPredictor::predictObjectManeuverByTimeToLaneChange(
  const std::string & object_id, const LaneletData & current_lanelet_data,
  const double /*object_detected_time*/,
  const std::unordered_map<std::string, std::deque<RoadUser>> & history)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (history.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const std::deque<RoadUser> & object_info = history.at(object_id);

  const int latest_id = static_cast<int>(object_info.size()) - 1;
  if (latest_id < 1) {
    return Maneuver::LANE_FOLLOW;
  }

  const auto & latest_info = object_info.at(static_cast<size_t>(latest_id));

  bool not_found_corresponding_lanelet = true;
  double left_dist, right_dist;
  double v_left_filtered, v_right_filtered;
  if (latest_info.lateral_kinematics_set.count(current_lanelet_data.lanelet) != 0) {
    const auto & lateral_kinematics =
      latest_info.lateral_kinematics_set.at(current_lanelet_data.lanelet);
    left_dist = lateral_kinematics.dist_from_left_boundary;
    right_dist = lateral_kinematics.dist_from_right_boundary;
    v_left_filtered = lateral_kinematics.filtered_left_lateral_velocity;
    v_right_filtered = lateral_kinematics.filtered_right_lateral_velocity;
    not_found_corresponding_lanelet = false;
  }

  if (not_found_corresponding_lanelet) {
    return Maneuver::LANE_FOLLOW;
  }

  const double latest_lane_width = left_dist + right_dist;
  if (latest_lane_width < 1e-3) {
    RCLCPP_ERROR(node_.get_logger(), "[Map Based Prediction]: Lane Width is too small");
    return Maneuver::LANE_FOLLOW;
  }

  const double epsilon = 1e-9;
  const double margin_to_reach_left_bound = left_dist / (std::fabs(v_left_filtered) + epsilon);
  const double margin_to_reach_right_bound = right_dist / (std::fabs(v_right_filtered) + epsilon);

  if (
    left_dist < right_dist && left_dist < params_.dist_threshold_to_bound && v_left_filtered < 0 &&
    margin_to_reach_left_bound < params_.time_threshold_to_bound) {
    return Maneuver::LEFT_LANE_CHANGE;
  } else if (
    right_dist < left_dist && right_dist < params_.dist_threshold_to_bound &&
    v_right_filtered < 0 && margin_to_reach_right_bound < params_.time_threshold_to_bound) {
    return Maneuver::RIGHT_LANE_CHANGE;
  }

  return Maneuver::LANE_FOLLOW;
}

Maneuver ManeuverPredictor::predictObjectManeuverByLatDiffDistance(
  const std::string & object_id, const geometry_msgs::msg::Pose & object_pose,
  const LaneletData & current_lanelet_data, const double /*object_detected_time*/,
  const std::unordered_map<std::string, std::deque<RoadUser>> & history)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (history.count(object_id) == 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const std::deque<RoadUser> & object_info = history.at(object_id);
  const double current_time = (node_.get_clock()->now()).seconds();

  int prev_id = static_cast<int>(object_info.size()) - 1;
  while (prev_id >= 0) {
    const double prev_time_delay = object_info.at(prev_id).time_delay;
    const double prev_time =
      rclcpp::Time(object_info.at(prev_id).header.stamp).seconds() + prev_time_delay;
    if (current_time - prev_time > params_.history_time_length) {
      break;
    }
    --prev_id;
  }

  if (prev_id < 0) {
    return Maneuver::LANE_FOLLOW;
  }

  const auto & prev_info = object_info.at(static_cast<size_t>(prev_id));
  const auto prev_pose = prev_info.pose;
  const lanelet::ConstLanelets prev_lanelets =
    object_info.at(static_cast<size_t>(prev_id)).current_lanelets;
  if (prev_lanelets.empty()) {
    return Maneuver::LANE_FOLLOW;
  }
  lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
  double closest_prev_yaw = std::numeric_limits<double>::max();
  for (const auto & lanelet : prev_lanelets) {
    const double lane_yaw = autoware::experimental::lanelet2_utils::get_lanelet_angle(
      lanelet, autoware::experimental::lanelet2_utils::from_ros(prev_pose).basicPoint());
    const double delta_yaw = tf2::getYaw(prev_pose.orientation) - lane_yaw;
    const double normalized_delta_yaw = autoware_utils::normalize_radian(delta_yaw);
    if (normalized_delta_yaw < closest_prev_yaw) {
      closest_prev_yaw = normalized_delta_yaw;
      prev_lanelet = lanelet;
    }
  }

  const auto current_lanelet = current_lanelet_data.lanelet;
  const auto current_pose = object_pose;
  const double dist = autoware_utils::calc_distance2d(prev_pose, current_pose);
  lanelet::routing::LaneletPaths possible_paths =
    routing_graph_ptr_->possiblePaths(prev_lanelet, dist + 2.0, 0, false);
  bool has_lane_changed = true;
  if (prev_lanelet == current_lanelet) {
    has_lane_changed = false;
  } else {
    for (const auto & path : possible_paths) {
      for (const auto & lanelet : path) {
        if (lanelet == current_lanelet) {
          has_lane_changed = false;
          break;
        }
      }
    }
  }

  if (has_lane_changed) {
    return Maneuver::LANE_FOLLOW;
  }

  const lanelet::ConstLineString2d prev_left_bound = prev_lanelet.leftBound2d();
  const lanelet::ConstLineString2d prev_right_bound = prev_lanelet.rightBound2d();
  const lanelet::ConstLineString2d current_left_bound = current_lanelet.leftBound2d();
  const lanelet::ConstLineString2d current_right_bound = current_lanelet.rightBound2d();
  const double prev_left_dist = calcLeftLateralOffset(prev_left_bound, prev_pose);
  const double prev_right_dist = calcRightLateralOffset(prev_right_bound, prev_pose);
  const double current_left_dist = calcLeftLateralOffset(current_left_bound, current_pose);
  const double current_right_dist = calcRightLateralOffset(current_right_bound, current_pose);
  const double prev_lane_width = std::fabs(prev_left_dist) + std::fabs(prev_right_dist);
  const double current_lane_width = std::fabs(current_left_dist) + std::fabs(current_right_dist);
  if (prev_lane_width < 1e-3 || current_lane_width < 1e-3) {
    RCLCPP_ERROR(node_.get_logger(), "[Map Based Prediction]: Lane Width is too small");
    return Maneuver::LANE_FOLLOW;
  }

  const double current_left_dist_ratio = current_left_dist / current_lane_width;
  const double current_right_dist_ratio = current_right_dist / current_lane_width;
  const double diff_left_current_prev = current_left_dist - prev_left_dist;
  const double diff_right_current_prev = current_right_dist - prev_right_dist;

  if (
    current_left_dist_ratio > params_.dist_ratio_threshold_to_left_bound &&
    diff_left_current_prev > params_.diff_dist_threshold_to_left_bound) {
    return Maneuver::LEFT_LANE_CHANGE;
  } else if (
    current_right_dist_ratio < params_.dist_ratio_threshold_to_right_bound &&
    diff_right_current_prev < params_.diff_dist_threshold_to_right_bound) {
    return Maneuver::RIGHT_LANE_CHANGE;
  }

  return Maneuver::LANE_FOLLOW;
}

double ManeuverPredictor::calcRightLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = autoware_utils::create_point(x, y, 0.0);
  }
  return std::fabs(autoware::motion_utils::calcLateralOffset(boundary_path, search_pose.position));
}

double ManeuverPredictor::calcLeftLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  return -calcRightLateralOffset(boundary_line, search_pose);
}

ManeuverProbability ManeuverPredictor::calculateManeuverProbability(
  const Maneuver & predicted_maneuver, const bool left_paths_exists, const bool right_paths_exists,
  const bool center_paths_exists) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  float left_lane_change_probability = 0.0;
  float right_lane_change_probability = 0.0;
  float lane_follow_probability = 0.0;
  if (left_paths_exists && predicted_maneuver == Maneuver::LEFT_LANE_CHANGE) {
    constexpr float LF_PROB_WHEN_LC = 0.9;
    constexpr float LC_PROB_WHEN_LC = 1.0;
    left_lane_change_probability = LC_PROB_WHEN_LC;
    right_lane_change_probability = 0.0;
    lane_follow_probability = LF_PROB_WHEN_LC;
  } else if (right_paths_exists && predicted_maneuver == Maneuver::RIGHT_LANE_CHANGE) {
    constexpr float LF_PROB_WHEN_LC = 0.9;
    constexpr float RC_PROB_WHEN_LC = 1.0;
    left_lane_change_probability = 0.0;
    right_lane_change_probability = RC_PROB_WHEN_LC;
    lane_follow_probability = LF_PROB_WHEN_LC;
  } else if (center_paths_exists) {
    constexpr float LF_PROB = 1.0;
    constexpr float LC_PROB = 0.3;
    constexpr float RC_PROB = 0.3;
    if (predicted_maneuver == Maneuver::LEFT_LANE_CHANGE) {
      left_lane_change_probability = 0.0;
      right_lane_change_probability = (right_paths_exists) ? RC_PROB : 0.0;
    } else if (predicted_maneuver == Maneuver::RIGHT_LANE_CHANGE) {
      left_lane_change_probability = (left_paths_exists) ? LC_PROB : 0.0;
      right_lane_change_probability = 0.0;
    } else {
      left_lane_change_probability = LC_PROB;
      right_lane_change_probability = RC_PROB;
    }
    lane_follow_probability = LF_PROB;
  } else {
    constexpr float LC_PROB = 1.0;
    constexpr float RC_PROB = 1.0;
    lane_follow_probability = 0.0;
    left_lane_change_probability = left_paths_exists ? LC_PROB : 0.0;
    right_lane_change_probability = right_paths_exists ? RC_PROB : 0.0;
  }

  const float MIN_PROBABILITY = 1e-3;
  const float max_prob = std::max(
    MIN_PROBABILITY, std::max(
                       lane_follow_probability,
                       std::max(left_lane_change_probability, right_lane_change_probability)));

  ManeuverProbability maneuver_prob;
  maneuver_prob[Maneuver::LEFT_LANE_CHANGE] = left_lane_change_probability / max_prob;
  maneuver_prob[Maneuver::RIGHT_LANE_CHANGE] = right_lane_change_probability / max_prob;
  maneuver_prob[Maneuver::LANE_FOLLOW] = lane_follow_probability / max_prob;

  return maneuver_prob;
}

}  // namespace autoware::map_based_prediction
