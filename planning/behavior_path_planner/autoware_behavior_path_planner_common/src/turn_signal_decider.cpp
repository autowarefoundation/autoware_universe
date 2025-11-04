// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"

#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/lanelet2_utils/hatched_road_markings.hpp>
#include <autoware/motion_utils/constants.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::motion_utils::calcSignedArcLength;

// Helper: parse optional exit turn signal attribute and check if it contains the exit lanelet id.
// Returns true if (a) exit lanelet id is valid, (b) attribute exists (not "none"),
// and (c) the parsed list contains the exit lanelet id.
static bool has_exit_turn_signal_tag(
  const lanelet::ConstLanelet & entry_lanelet, const lanelet::ConstLanelet & exit_lanelet,
  const std::string & attribute_key)
{
  if (!exit_lanelet.id()) return false;
  const std::string tag = entry_lanelet.attributeOr(attribute_key, std::string("none"));
  if (tag == "none") return false;

  std::vector<std::string> tokens;
  boost::split(tokens, tag, boost::is_any_of(", []"), boost::token_compress_on);
  const std::string exit_id_str = std::to_string(exit_lanelet.id());
  return std::find(tokens.begin(), tokens.end(), exit_id_str) != tokens.end();
}

double calc_distance(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const Pose & input_point, const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  const size_t nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, input_point, nearest_dist_threshold, nearest_yaw_threshold);
  return autoware::motion_utils::calcSignedArcLength(
    path.points, current_pose.position, current_seg_idx, input_point.position, nearest_seg_idx);
}

std::string turn_indicator_command_str(const uint8_t command)
{
  if (command == TurnIndicatorsCommand::ENABLE_LEFT) return "left";
  if (command == TurnIndicatorsCommand::ENABLE_RIGHT) return "right";
  return "unknown";
}

/***
 * @brief:
 * Gets the turn signal info after comparing the turn signal info output from the behavior path
 * module and comparing it to turn signal info obtained from intersections.
 */
TurnIndicatorsCommand TurnSignalDecider::getTurnSignal(
  const std::shared_ptr<RouteHandler> & route_handler, const PathWithLaneId & path,
  const TurnSignalInfo & turn_signal_info, const Pose & current_pose, const double current_vel,
  const BehaviorPathPlannerParameters & parameters, TurnSignalDebugData & debug_data)
{
  debug_data.behavior_turn_signal_info = turn_signal_info;

  // Guard
  if (path.points.empty()) {
    return turn_signal_info.turn_signal;
  }

  const double forward_length = parameters.forward_path_length;
  const double nearest_dist_threshold = parameters.ego_nearest_dist_threshold;
  const double nearest_yaw_threshold = parameters.ego_nearest_yaw_threshold;
  double backward_length = 50.0;

  // In roundabouts, backward length is needed to adjust the backward length to ensure proper path
  // planning.
  lanelet::ConstLanelet current_lanelet;
  if (route_handler->getClosestLaneletWithConstrainsWithinRoute(
        current_pose, &current_lanelet, nearest_dist_threshold, nearest_yaw_threshold)) {
    backward_length = calculateRoundaboutBackwardLength(
      current_lanelet, *route_handler, backward_length, roundabout_backward_depth_);
  }

  // Get current lanelets
  const lanelet::ConstLanelets current_lanes =
    utils::calcLaneAroundPose(route_handler, current_pose, forward_length, backward_length);

  if (current_lanes.empty()) {
    return turn_signal_info.turn_signal;
  }

  const PathWithLaneId extended_path = utils::getCenterLinePath(
    *route_handler, current_lanes, current_pose, backward_length, forward_length, parameters);

  if (extended_path.points.empty()) {
    return turn_signal_info.turn_signal;
  }

  // Closest ego segment
  const size_t ego_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      extended_path.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);

  // Get closest intersection turn signal if exists
  const auto intersection_turn_signal_info = getIntersectionTurnSignalInfo(
    extended_path, current_pose, current_vel, ego_seg_idx, *route_handler, nearest_dist_threshold,
    nearest_yaw_threshold, parameters.turn_signal_minimum_search_distance,
    turn_signal_info.turn_signal.command);

  // Get roundabout turn signal if exists
  const auto roundabout_turn_signal_info = getRoundaboutTurnSignalInfo(
    extended_path, current_pose, current_vel, ego_seg_idx, *route_handler, nearest_dist_threshold,
    nearest_yaw_threshold);

  // Debug data
  if (roundabout_turn_signal_info) {
    debug_data.roundabout_turn_signal_info = *roundabout_turn_signal_info;
  }
  if (intersection_turn_signal_info) {
    debug_data.intersection_turn_signal_info = *intersection_turn_signal_info;
  }

  TurnSignalInfo empty_signal_info;
  const TurnSignalInfo & inter_ref =
    intersection_turn_signal_info ? *intersection_turn_signal_info : empty_signal_info;
  const TurnSignalInfo & round_ref =
    roundabout_turn_signal_info ? *roundabout_turn_signal_info : empty_signal_info;
  return resolve_turn_signal(
    extended_path, current_pose, ego_seg_idx, inter_ref, round_ref, turn_signal_info,
    nearest_dist_threshold, nearest_yaw_threshold);
}

std::pair<bool, bool> TurnSignalDecider::getIntersectionTurnSignalFlag()
{
  return std::make_pair(intersection_turn_signal_, approaching_intersection_turn_signal_);
}

std::pair<Pose, double> TurnSignalDecider::getIntersectionPoseAndDistance()
{
  return std::make_pair(intersection_pose_point_, intersection_distance_);
}

std::optional<TurnSignalInfo> TurnSignalDecider::getIntersectionTurnSignalInfo(
  const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
  const size_t current_seg_idx, const RouteHandler & route_handler,
  const double nearest_dist_threshold, const double nearest_yaw_threshold,
  const double th_search_dist_to_turn_direction_lane, const uint8_t turn_indicator_command)
{
  // base search distance
  const double base_search_distance =
    turn_signal_search_time_ * current_vel + intersection_search_distance_;

  // unique lane ids
  const std::vector<lanelet::Id> unique_lane_ids = utils::get_lanelet_id_from_path(path);

  const lanelet::ConstLanelets lanelet_sequence =
    utils::get_lanelet_sequence_from_path(path, route_handler);

  const std::string expected_turn_command = turn_indicator_command_str(turn_indicator_command);

  const auto dist_to_next_turn_direction_lane = utils::calc_distance_to_next_turn_direction_lane(
    current_pose, route_handler, lanelet_sequence, expected_turn_command);

  const bool is_near_next_intersection =
    dist_to_next_turn_direction_lane.value_or(std::numeric_limits<double>::max()) <
    th_search_dist_to_turn_direction_lane;

  const lanelet::ConstLanelets nearest_turn_direction_lanes =
    utils::nearest_turn_direction_lane_within_route(current_pose, route_handler, lanelet_sequence);

  const bool is_in_nearest_turn_direction_lane = std::any_of(
    nearest_turn_direction_lanes.begin(), nearest_turn_direction_lanes.end(),
    [&](const auto & llt) {
      return std::string(llt.attributeOr("turn_direction", "else")) == expected_turn_command;
    });

  const auto requires_intersection_turn_signal = [&](const auto & turn_direction) -> bool {
    if (utils::is_turn_direction_lane(turn_direction)) {
      return true;
    }
    constexpr double stop_velocity_threshold = 0.1;

    if (!is_near_next_intersection && !is_in_nearest_turn_direction_lane) {
      return false;
    }

    // if ego is near or in turn direction lane, if it stops, then we need turn signal for
    // intersection.
    return current_vel < stop_velocity_threshold;
  };

  // combine consecutive lanes of the same turn direction
  // stores lanes that have already been combine
  std::unordered_set<lanelet::Id> processed_lanes;
  // since combined_lane does not inherit id and attribute,
  // and ConstantLanelet does not rewrite the value,
  // we keep front_lane together as a representative.
  std::vector<std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet>> combined_and_front_vec{};
  for (const auto & lane_id : unique_lane_ids) {
    // Skip if already processed
    if (processed_lanes.find(lane_id) != processed_lanes.end()) continue;
    auto current_lane = route_handler.getLaneletsFromId(lane_id);
    // Skip if lanelet is part of a roundabout regulatory element
    const auto regulatory_elements =
      current_lane.regulatoryElementsAs<lanelet::autoware::Roundabout>();
    if (!regulatory_elements.empty()) continue;

    lanelet::ConstLanelets combined_lane_elems{};
    // Get the lane and its attribute
    const std::string lane_attribute =
      current_lane.attributeOr("turn_direction", std::string("none"));
    if (!requires_intersection_turn_signal(lane_attribute)) continue;

    do {
      processed_lanes.insert(current_lane.id());
      combined_lane_elems.push_back(current_lane);
      lanelet::ConstLanelet next_lane{};
      current_lane = next_lane;
    } while (route_handler.getNextLaneletWithinRoute(current_lane, &current_lane) &&
             current_lane.attributeOr("turn_direction", std::string("none")) == lane_attribute);

    if (!combined_lane_elems.empty()) {
      // store combined lane and its front lane
      const auto & combined_and_first = std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet>(
        lanelet::utils::combineLaneletsShape(combined_lane_elems), combined_lane_elems.front());
      combined_and_front_vec.push_back(combined_and_first);
    }
  }

  std::queue<TurnSignalInfo> signal_queue;
  for (const auto & combined_and_front : combined_and_front_vec) {
    // use combined_lane's centerline
    const auto & combined_lane = combined_and_front.first;
    if (combined_lane.centerline3d().size() < 2) {
      continue;
    }

    // use front lane's id, attribute, and search distance as a representative
    const auto & front_lane = combined_and_front.second;
    const auto lane_id = front_lane.id();
    const double search_distance =
      front_lane.attributeOr("turn_signal_distance", base_search_distance);
    const std::string lane_attribute =
      front_lane.attributeOr("turn_direction", std::string("none"));

    const auto lane_front_pose = calculateLaneFrontPose(combined_lane.centerline3d());
    const auto lane_back_pose = calculateLaneBackPose(combined_lane.centerline3d());

    // Distance from ego vehicle front pose to front point of the lane
    const double dist_to_front_point = calc_distance(
                                         path, current_pose, current_seg_idx, lane_front_pose,
                                         nearest_dist_threshold, nearest_yaw_threshold) -
                                       base_link2front_;

    // Distance from ego vehicle base link to the terminal point of the lane
    const double dist_to_back_point = calc_distance(
      path, current_pose, current_seg_idx, lane_back_pose, nearest_dist_threshold,
      nearest_yaw_threshold);
    if (dist_to_back_point < 0.0) {
      // Vehicle is already passed this lane
      intersection_desired_start_point_association_.erase(lane_id);
      continue;
    }

    if (search_distance <= dist_to_front_point) {
      continue;
    }
    if (requires_intersection_turn_signal(lane_attribute)) {
      // update map if necessary
      if (
        intersection_desired_start_point_association_.find(lane_id) ==
        intersection_desired_start_point_association_.end()) {
        intersection_desired_start_point_association_.emplace(lane_id, current_pose);
      }

      TurnSignalInfo turn_signal_info{};
      turn_signal_info.desired_start_point =
        intersection_desired_start_point_association_.at(lane_id);
      turn_signal_info.required_start_point = lane_front_pose;
      turn_signal_info.required_end_point =
        get_required_end_point(combined_lane.centerline3d(), intersection_angle_threshold_deg_);
      turn_signal_info.desired_end_point = lane_back_pose;
      turn_signal_info.turn_signal.command = g_signal_map.at(lane_attribute);
      signal_queue.push(turn_signal_info);
    }
  }

  return resolveSignalQueue(
    signal_queue, path, current_pose, current_seg_idx, nearest_dist_threshold,
    nearest_yaw_threshold);
}

std::optional<TurnSignalInfo> TurnSignalDecider::getRoundaboutTurnSignalInfo(
  const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
  const size_t current_seg_idx, const RouteHandler & route_handler,
  const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  std::set<lanelet::Id> processed_lanes;
  lanelet::ConstLanelets roundabout_entry_lanelets;
  lanelet::ConstLanelets roundabout_exit_lanelets;
  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (processed_lanes.count(lane_id)) continue;
      const auto lanelet = route_handler.getLaneletsFromId(lane_id);
      const auto roundabouts = lanelet.regulatoryElementsAs<lanelet::autoware::Roundabout>();
      for (const auto & roundabout : roundabouts) {
        if (roundabout->isEntryLanelet(lanelet.id())) {
          roundabout_entry_lanelets.push_back(lanelet);
        }
        if (roundabout->isExitLanelet(lanelet.id())) {
          roundabout_exit_lanelets.push_back(lanelet);
        }
      }
      processed_lanes.insert(lane_id);
    }
  }
  if (roundabout_entry_lanelets.empty() && roundabout_exit_lanelets.empty()) {
    return {};
  }

  // base search distance
  const double base_search_distance =
    turn_signal_search_time_ * current_vel + roundabout_search_distance_;
  std::queue<TurnSignalInfo> signal_queue;

  // Process entry lanes
  for (const auto & entry_lanelet : roundabout_entry_lanelets) {
    const auto & centerline = entry_lanelet.centerline3d();
    if (centerline.size() < 2) continue;

    const auto front_pose = calculateLaneFrontPose(centerline);
    const auto dist_to_front_pose = calc_distance(
                                      path, current_pose, current_seg_idx, front_pose,
                                      nearest_dist_threshold, nearest_yaw_threshold) -
                                    base_link2front_;
    const double search_distance =
      entry_lanelet.attributeOr("turn_signal_distance", base_search_distance);
    if (
      search_distance <= dist_to_front_pose &&
      !roundabout_desired_start_point_association_.count(entry_lanelet.id()))
      continue;  // Skip if the front point is too far

    const auto back_pose = calculateLaneBackPose(centerline);
    const auto [iter, inserted] =
      roundabout_desired_start_point_association_.try_emplace(entry_lanelet.id(), current_pose);
    TurnSignalInfo turn_signal_info;
    turn_signal_info.desired_start_point = iter->second;
    turn_signal_info.required_start_point = front_pose;
    turn_signal_info.required_end_point =
      get_required_end_point(centerline, roundabout_angle_threshold_deg_);
    turn_signal_info.desired_end_point = back_pose;
    turn_signal_info.turn_signal.command = roundabout_on_entry_;

    // If defined in the lanelet map, change the turn signal based on the exit lanelet
    lanelet::ConstLanelet roundabout_exit_lanelet;
    lanelet::ConstLanelet next_lanelet = entry_lanelet;
    const auto & roundabouts = entry_lanelet.regulatoryElementsAs<lanelet::autoware::Roundabout>();
    // From the entry lanelet, iterate over subsequent lanelets along the route,
    // updating until just before leaving the roundabout to identify the exit lanelet
    while (route_handler.getNextLaneletWithinRoute(next_lanelet, &next_lanelet) &&
           !roundabouts.empty()) {
      if (!roundabouts.front()->isRoundaboutLanelet(next_lanelet.id())) break;
      roundabout_exit_lanelet = next_lanelet;
    }
    // if entry lanelets has optional tags
    const bool has_exit_turn_signal_left =
      has_exit_turn_signal_tag(entry_lanelet, roundabout_exit_lanelet, "exit_turn_signal_left");
    const bool has_exit_turn_signal_right =
      has_exit_turn_signal_tag(entry_lanelet, roundabout_exit_lanelet, "exit_turn_signal_right");

    // Change the entry turn signal based on the exit lanelet
    if (has_exit_turn_signal_left) {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
    } else if (has_exit_turn_signal_right) {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    } else if (roundabout_on_entry_ == TurnIndicatorsCommand::DISABLE) {
      roundabout_desired_start_point_association_.erase(entry_lanelet.id());
      continue;
    }

    // If the roundabout entry indicator persistence is enabled, keep the indicator on until the
    // vehicle exits the roundabout.
    if (roundabout_entry_indicator_persistence_ && roundabout_exit_lanelet.id()) {
      const auto exit_lane_back_pose =
        calculateLaneBackPose(roundabout_exit_lanelet.centerline3d());
      turn_signal_info.desired_end_point = exit_lane_back_pose;
    }
    const auto dist_to_desired_end_point = calc_distance(
      path, current_pose, current_seg_idx, turn_signal_info.desired_end_point,
      nearest_dist_threshold, nearest_yaw_threshold);
    if (dist_to_desired_end_point < 0.0) {  // Vehicle is already passed this lane
      roundabout_desired_start_point_association_.erase(entry_lanelet.id());
      continue;
    }
    signal_queue.push(turn_signal_info);
  }

  // Process exit lanes
  for (const auto & exit_lanelet : roundabout_exit_lanelets) {
    // use combined_lane's centerline
    const auto & centerline = exit_lanelet.centerline3d();
    if (centerline.size() < 2) continue;

    // calculate back pose and distance to back pose
    const auto back_pose = calculateLaneBackPose(centerline);
    const auto dist_to_back_pose = calc_distance(
      path, current_pose, current_seg_idx, back_pose, nearest_dist_threshold,
      nearest_yaw_threshold);
    if (dist_to_back_pose < 0.0) {  // Vehicle is already passed this lane
      roundabout_desired_start_point_association_.erase(exit_lanelet.id());
      continue;
    }

    // Find enable_exit_turn_signal lanelet
    bool found_enable_exit_turn_signal = false;
    lanelet::ConstLanelet enable_exit_turn_signal_lanelet =
      findEnableExitTurnSignalLanelet(exit_lanelet, route_handler, found_enable_exit_turn_signal);
    // If no enable_exit_turn_signal lanelet is found, use the front pose of exit lanelet as the
    // desired start point.
    const auto required_start_point = calculateLaneFrontPose(centerline);
    Pose desired_start_point;
    if (found_enable_exit_turn_signal) {
      desired_start_point = calculateLaneFrontPose(enable_exit_turn_signal_lanelet.centerline3d());
    } else {
      desired_start_point = required_start_point;
    }
    const auto dist_to_desired_start_point =
      calc_distance(
        path, current_pose, current_seg_idx, desired_start_point, nearest_dist_threshold,
        nearest_yaw_threshold) -
      base_link2front_;
    if (dist_to_desired_start_point >= 0.0) continue;  // Skip if the front point is too far
    // update map if necessary
    const auto [iter, inserted] = roundabout_desired_start_point_association_.try_emplace(
      exit_lanelet.id(), desired_start_point);

    TurnSignalInfo turn_signal_info;
    turn_signal_info.desired_start_point = iter->second;
    turn_signal_info.required_start_point = required_start_point;
    turn_signal_info.required_end_point =
      get_required_end_point(centerline, roundabout_angle_threshold_deg_);
    turn_signal_info.desired_end_point = back_pose;
    turn_signal_info.turn_signal.command = roundabout_on_exit_;
    signal_queue.push(turn_signal_info);
  }

  return resolveSignalQueue(
    signal_queue, path, current_pose, current_seg_idx, nearest_dist_threshold,
    nearest_yaw_threshold);
}

lanelet::ConstLanelet TurnSignalDecider::findEnableExitTurnSignalLanelet(
  const lanelet::ConstLanelet & start_lanelet, const RouteHandler & route_handler,
  bool & found_enable_exit_turn_signal)
{
  lanelet::ConstLanelet current_lanelet = start_lanelet;
  found_enable_exit_turn_signal = false;
  while (true) {
    lanelet::ConstLanelets prev_lanelets;
    if (!route_handler.getPreviousLaneletsWithinRoute(current_lanelet, &prev_lanelets)) break;

    const auto it =
      std::find_if(prev_lanelets.cbegin(), prev_lanelets.cend(), [](const auto & lanelet) {
        return lanelet.attributeOr("enable_exit_turn_signal", std::string("none")) == "true";
      });

    if (it != prev_lanelets.cend()) {
      current_lanelet = *it;
      found_enable_exit_turn_signal = true;
    } else {
      break;
    }
  }
  return current_lanelet;
}

Pose TurnSignalDecider::calculateLaneFrontPose(const lanelet::ConstLineString3d & centerline)
{
  Pose front_pose;
  front_pose.position = lanelet::utils::conversion::toGeomMsgPt(centerline.front());
  front_pose.orientation =
    calc_orientation(front_pose.position, lanelet::utils::conversion::toGeomMsgPt(centerline[1]));
  return front_pose;
}

Pose TurnSignalDecider::calculateLaneBackPose(const lanelet::ConstLineString3d & centerline)
{
  Pose back_pose;
  back_pose.position = lanelet::utils::conversion::toGeomMsgPt(centerline.back());
  const auto & prev_point =
    lanelet::utils::conversion::toGeomMsgPt(centerline[centerline.size() - 2]);
  back_pose.orientation = calc_orientation(prev_point, back_pose.position);
  return back_pose;
}

std::optional<TurnSignalInfo> TurnSignalDecider::resolveSignalQueue(
  std::queue<TurnSignalInfo> & signal_queue, const PathWithLaneId & path, const Pose & current_pose,
  const size_t current_seg_idx, const double nearest_dist_threshold,
  const double nearest_yaw_threshold)
{
  while (!signal_queue.empty()) {
    if (signal_queue.size() == 1) {
      return signal_queue.front();
    }

    const auto & turn_signal_info = signal_queue.front();
    const auto & required_end_point = turn_signal_info.required_end_point;
    const size_t nearest_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.points, required_end_point, nearest_dist_threshold, nearest_yaw_threshold);
    const double dist_to_end_point = autoware::motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_seg_idx, required_end_point.position,
      nearest_seg_idx);

    if (dist_to_end_point >= 0.0) {
      return turn_signal_info;
    }

    signal_queue.pop();
  }

  return {};
}

TurnIndicatorsCommand TurnSignalDecider::resolve_turn_signal(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const TurnSignalInfo & intersection_signal_info, const TurnSignalInfo & roundabout_signal_info,
  const TurnSignalInfo & behavior_signal_info, const double nearest_dist_threshold,
  const double nearest_yaw_threshold)
{
  const auto get_distance = [&](const Pose & input_point) {
    return calc_distance(
      path, current_pose, current_seg_idx, input_point, nearest_dist_threshold,
      nearest_yaw_threshold);
  };

  auto create_candidate = [&](const TurnSignalInfo & signal_info, const std::string & type) {
    return SignalCandidate{
      signal_info,
      get_distance(signal_info.desired_start_point) - base_link2front_,
      get_distance(signal_info.desired_end_point),
      get_distance(signal_info.required_start_point) - base_link2front_,
      get_distance(signal_info.required_end_point),
      type};
  };

  std::vector<SignalCandidate> candidates;
  candidates.reserve(3);

  candidates.push_back(create_candidate(intersection_signal_info, "intersection"));
  candidates.push_back(create_candidate(roundabout_signal_info, "roundabout"));
  candidates.push_back(create_candidate(behavior_signal_info, "behavior"));

  //  Remove invalid candidates
  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(),
      [](const SignalCandidate & candidate) { return !candidate.isValid(); }),
    candidates.end());

  if (candidates.empty()) {
    TurnIndicatorsCommand empty_signal_command;
    empty_signal_command.command = TurnIndicatorsCommand::DISABLE;
    initialize_intersection_info();
    return empty_signal_command;
  }

  // If there is only one candidate, return it directly
  if (candidates.size() == 1) {
    const auto & candidate = candidates[0];
    if (candidate.signal_type == "intersection") {
      set_intersection_info(
        path, current_pose, current_seg_idx, candidate.signal_info, nearest_dist_threshold,
        nearest_yaw_threshold);
    } else {
      initialize_intersection_info();
    }
    return candidate.signal_info.turn_signal;
  }

  //  Sort candidates by desired start distance
  std::sort(
    candidates.begin(), candidates.end(), [](const SignalCandidate & a, const SignalCandidate & b) {
      if (a.desired_start_distance != b.desired_start_distance) {
        return a.desired_start_distance < b.desired_start_distance;
      }
      // If same desired_start_distance, prefer intersection > roundabout > behavior
      if (a.signal_type == "intersection" && b.signal_type != "intersection") {
        return true;
      }
      if (a.signal_type == "roundabout" && b.signal_type == "behavior") {
        return true;
      }
      return false;
    });

  // Helper function to compare two signal candidates
  auto compare_signals =
    [&](const SignalCandidate & a, const SignalCandidate & b) -> const SignalCandidate & {
    const bool use_a = use_prior_turn_signal(
      a.required_start_distance, a.required_end_distance, b.required_start_distance,
      b.required_end_distance);
    return use_a ? a : b;
  };

  // Compare all candidates
  SignalCandidate candidate = candidates[0];
  for (size_t i = 1; i < candidates.size(); ++i) {
    candidate = compare_signals(candidate, candidates[i]);
  }

  if (candidate.signal_type == "intersection") {
    set_intersection_info(
      path, current_pose, current_seg_idx, candidate.signal_info, nearest_dist_threshold,
      nearest_yaw_threshold);
  } else {
    initialize_intersection_info();
  }

  return candidate.signal_info.turn_signal;
}

TurnSignalInfo TurnSignalDecider::overwrite_turn_signal(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const TurnSignalInfo & original_signal, const TurnSignalInfo & new_signal,
  const double nearest_dist_threshold, const double nearest_yaw_threshold) const
{
  if (original_signal.turn_signal.command == TurnIndicatorsCommand::NO_COMMAND) {
    return new_signal;
  }

  if (original_signal.turn_signal.command == TurnIndicatorsCommand::DISABLE) {
    return new_signal;
  }

  const auto get_distance = [&](const Pose & input_point) {
    const size_t nearest_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.points, input_point, nearest_dist_threshold, nearest_yaw_threshold);
    return autoware::motion_utils::calcSignedArcLength(
             path.points, current_pose.position, current_seg_idx, input_point.position,
             nearest_seg_idx) -
           base_link2front_;
  };

  const auto & original_desired_end_point = original_signal.desired_end_point;
  const auto & new_desired_start_point = new_signal.desired_start_point;

  const double dist_to_original_desired_end = get_distance(original_desired_end_point);
  const double dist_to_new_desired_start = get_distance(new_desired_start_point);
  if (dist_to_new_desired_start > dist_to_original_desired_end) {
    return original_signal;
  }

  return new_signal;
}

TurnSignalInfo TurnSignalDecider::use_prior_turn_signal(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const TurnSignalInfo & original_signal, const TurnSignalInfo & new_signal,
  const double nearest_dist_threshold, const double nearest_yaw_threshold)
{
  const auto get_distance = [&](const Pose & input_point) {
    const size_t nearest_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.points, input_point, nearest_dist_threshold, nearest_yaw_threshold);
    return autoware::motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_seg_idx, input_point.position, nearest_seg_idx);
  };

  const auto & original_desired_start_point = original_signal.desired_start_point;
  const auto & original_desired_end_point = original_signal.desired_end_point;
  const auto & original_required_start_point = original_signal.required_start_point;
  const auto & original_required_end_point = original_signal.required_end_point;
  const auto & new_desired_start_point = new_signal.desired_start_point;
  const auto & new_desired_end_point = new_signal.desired_end_point;
  const auto & new_required_start_point = new_signal.required_start_point;
  const auto & new_required_end_point = new_signal.required_end_point;

  const double dist_to_original_desired_start =
    get_distance(original_desired_start_point) - base_link2front_;
  const double dist_to_new_desired_start = get_distance(new_desired_start_point) - base_link2front_;

  // If we still do not reach the desired front point we ignore it
  if (dist_to_original_desired_start > 0.0 && dist_to_new_desired_start > 0.0) {
    TurnSignalInfo empty_signal_info;
    return empty_signal_info;
  } else if (dist_to_original_desired_start > 0.0) {
    return new_signal;
  } else if (dist_to_new_desired_start > 0.0) {
    return original_signal;
  }

  const double dist_to_original_desired_end = get_distance(original_desired_end_point);
  const double dist_to_new_desired_end = get_distance(new_desired_end_point);

  // If we already passed the desired end point, return the other signal
  if (dist_to_original_desired_end < 0.0 && dist_to_new_desired_end < 0.0) {
    TurnSignalInfo empty_signal_info;
    return empty_signal_info;
  } else if (dist_to_original_desired_end < 0.0) {
    return new_signal;
  } else if (dist_to_new_desired_end < 0.0) {
    return original_signal;
  }

  const double dist_to_original_required_start =
    get_distance(original_required_start_point) - base_link2front_;
  const double dist_to_original_required_end = get_distance(original_required_end_point);
  const double dist_to_new_required_start =
    get_distance(new_required_start_point) - base_link2front_;
  const double dist_to_new_required_end = get_distance(new_required_end_point);

  if (dist_to_original_desired_start <= dist_to_new_desired_start) {
    const auto enable_prior = use_prior_turn_signal(
      dist_to_original_required_start, dist_to_original_required_end, dist_to_new_required_start,
      dist_to_new_required_end);

    if (enable_prior) {
      return original_signal;
    }
    return new_signal;
  }

  const auto enable_prior = use_prior_turn_signal(
    dist_to_new_required_start, dist_to_new_required_end, dist_to_original_required_start,
    dist_to_original_required_end);
  if (enable_prior) {
    return new_signal;
  }
  return original_signal;
}

bool TurnSignalDecider::use_prior_turn_signal(
  const double dist_to_prior_required_start, const double dist_to_prior_required_end,
  const double dist_to_subsequent_required_start, const double dist_to_subsequent_required_end)
{
  const bool before_prior_required = dist_to_prior_required_start > 0.0;
  const bool before_subsequent_required = dist_to_subsequent_required_start > 0.0;
  const bool inside_prior_required =
    dist_to_prior_required_start < 0.0 && 0.0 <= dist_to_prior_required_end;

  if (dist_to_prior_required_start < dist_to_subsequent_required_start) {
    // subsequent signal required section is completely overlapped the prior signal required
    // section
    if (dist_to_subsequent_required_end < dist_to_prior_required_end) {
      return true;
    }

    // Vehicle is inside or in front of the prior required section
    if (before_prior_required || inside_prior_required) {
      return true;
    }

    // passed prior required section but in front of the subsequent required section
    if (before_subsequent_required) {
      return true;
    }

    // within or passed subsequent required section and completely passed prior required section
    return false;
  }

  // Subsequent required section starts faster than prior required starts section

  // If the prior section is inside of the subsequent required section
  if (dist_to_prior_required_end < dist_to_subsequent_required_end) {
    return before_prior_required || inside_prior_required;
  }

  // inside or passed the intersection required
  if (before_prior_required) {
    return false;
  }

  return true;
}

geometry_msgs::msg::Pose TurnSignalDecider::get_required_end_point(
  const lanelet::ConstLineString3d & centerline, const double angle_threshold_deg)
{
  std::vector<geometry_msgs::msg::Pose> converted_centerline(centerline.size());
  for (size_t i = 0; i < centerline.size(); ++i) {
    converted_centerline.at(i).position = lanelet::utils::conversion::toGeomMsgPt(centerline[i]);
  }
  autoware::motion_utils::insertOrientation(converted_centerline, true);

  const double length = autoware::motion_utils::calcArcLength(converted_centerline);

  // Create resampling intervals
  const double resampling_interval = 1.0;
  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < length; s += resampling_interval) {
    resampling_arclength.push_back(s);
  }

  // Insert terminal point
  if (length - resampling_arclength.back() < autoware::motion_utils::overlap_threshold) {
    resampling_arclength.back() = length;
  } else {
    resampling_arclength.push_back(length);
  }

  const auto resampled_centerline =
    autoware::motion_utils::resamplePoseVector(converted_centerline, resampling_arclength);

  const double terminal_yaw = tf2::getYaw(resampled_centerline.back().orientation);
  for (auto itr = resampled_centerline.rbegin(); itr != resampled_centerline.rend(); ++itr) {
    const double yaw = tf2::getYaw(itr->orientation);
    const double yaw_diff = autoware_utils::normalize_radian(yaw - terminal_yaw);
    if (std::fabs(yaw_diff) >= autoware_utils::deg2rad(angle_threshold_deg)) {
      return *itr;
    }
  }

  return resampled_centerline.back();
}

void TurnSignalDecider::set_intersection_info(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const TurnSignalInfo & intersection_turn_signal_info, const double nearest_dist_threshold,
  const double nearest_yaw_threshold)
{
  const auto get_distance = [&](const Pose & input_point) {
    const size_t nearest_seg_idx =
      autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        path.points, input_point, nearest_dist_threshold, nearest_yaw_threshold);
    return autoware::motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_seg_idx, input_point.position, nearest_seg_idx);
  };

  const auto & inter_desired_start_point = intersection_turn_signal_info.desired_start_point;
  const auto & inter_desired_end_point = intersection_turn_signal_info.desired_end_point;
  const auto & inter_required_start_point = intersection_turn_signal_info.required_start_point;

  const double dist_to_intersection_desired_start =
    get_distance(inter_desired_start_point) - base_link2front_;
  const double dist_to_intersection_desired_end = get_distance(inter_desired_end_point);
  const double dist_to_intersection_required_start =
    get_distance(inter_required_start_point) - base_link2front_;

  if (dist_to_intersection_desired_start < 0.0 && dist_to_intersection_desired_end > 0.0) {
    if (dist_to_intersection_required_start > 0.0) {
      intersection_turn_signal_ = false;
      approaching_intersection_turn_signal_ = true;
    } else {
      intersection_turn_signal_ = true;
      approaching_intersection_turn_signal_ = false;
    }
    intersection_distance_ = dist_to_intersection_required_start;
    intersection_pose_point_ = inter_required_start_point;
  } else {
    initialize_intersection_info();
  }
}

void TurnSignalDecider::initialize_intersection_info()
{
  intersection_turn_signal_ = false;
  approaching_intersection_turn_signal_ = false;
  intersection_pose_point_ = Pose();
  intersection_distance_ = std::numeric_limits<double>::max();
}

geometry_msgs::msg::Quaternion TurnSignalDecider::calc_orientation(
  const Point & src_point, const Point & dst_point)
{
  const double pitch = autoware_utils::calc_elevation_angle(src_point, dst_point);
  const double yaw = autoware_utils::calc_azimuth_angle(src_point, dst_point);
  return autoware_utils::create_quaternion_from_rpy(0.0, pitch, yaw);
}

std::pair<TurnSignalInfo, bool> TurnSignalDecider::getBehaviorTurnSignalInfo(
  const ShiftedPath & path, const ShiftLine & shift_line,
  const lanelet::ConstLanelets & current_lanelets,
  const std::shared_ptr<RouteHandler> route_handler,
  const BehaviorPathPlannerParameters & parameters, const Odometry::ConstSharedPtr self_odometry,
  const vehicle_info_utils::VehicleInfo & vehicle_info, const double current_shift_length,
  const bool is_driving_forward, const bool egos_lane_is_shifted,
  const bool override_ego_stopped_check, const bool is_pull_out, const bool is_lane_change,
  const bool is_pull_over) const
{
  using autoware_utils::get_pose;

  const auto & p = parameters;
  const auto & rh = route_handler;
  const auto & ego_pose = self_odometry->pose.pose;
  const auto & ego_speed = self_odometry->twist.twist.linear.x;

  if (!is_driving_forward) {
    TurnSignalInfo turn_signal_info{};
    turn_signal_info.hazard_signal.command = HazardLightsCommand::ENABLE;
    const auto back_start_pose = rh->getOriginalStartPose();
    const Pose & start_pose = self_odometry->pose.pose;

    turn_signal_info.desired_start_point = back_start_pose;
    turn_signal_info.required_start_point = back_start_pose;
    // pull_out start_pose is same to backward driving end_pose
    turn_signal_info.required_end_point = start_pose;
    turn_signal_info.desired_end_point = start_pose;
    return std::make_pair(turn_signal_info, false);
  }

  if (shift_line.start_idx + 1 > path.shift_length.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return std::make_pair(TurnSignalInfo{}, true);
  }

  if (shift_line.start_idx + 1 > path.path.points.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return std::make_pair(TurnSignalInfo{}, true);
  }

  if (shift_line.end_idx + 1 > path.shift_length.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return std::make_pair(TurnSignalInfo{}, true);
  }

  if (shift_line.end_idx + 1 > path.path.points.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return std::make_pair(TurnSignalInfo{}, true);
  }

  const auto [start_shift_length, end_shift_length] =
    std::invoke([&path, &shift_line, &egos_lane_is_shifted]() -> std::pair<double, double> {
      const auto temp_start_shift_length = path.shift_length.at(shift_line.start_idx);
      const auto temp_end_shift_length = path.shift_length.at(shift_line.end_idx);
      // Shift is done using the target lane and not current ego's lane
      if (!egos_lane_is_shifted) {
        return std::make_pair(temp_end_shift_length, -temp_start_shift_length);
      }
      return std::make_pair(temp_start_shift_length, temp_end_shift_length);
    });

  const auto relative_shift_length = end_shift_length - start_shift_length;

  const auto p_path_start = get_pose(path.path.points.front());
  const auto p_path_end = get_pose(path.path.points.back());

  // If shift length is shorter than the threshold, it does not need to turn on blinkers
  if (std::fabs(relative_shift_length) < p.turn_signal_shift_length_threshold) {
    return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), true);
  }

  // If the vehicle does not shift anymore, we turn off the blinker
  if (
    std::abs(end_shift_length - current_shift_length) <
    p.turn_signal_remaining_shift_length_threshold) {
    return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), true);
  }

  const auto signal_prepare_distance =
    std::max(ego_speed * p.turn_signal_search_time, p.turn_signal_minimum_search_distance);
  const auto ego_front_to_shift_start =
    calcSignedArcLength(path.path.points, ego_pose.position, shift_line.start_idx) -
    p.vehicle_info.max_longitudinal_offset_m;

  if (signal_prepare_distance < ego_front_to_shift_start) {
    return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), false);
  }

  const auto blinker_start_pose = path.path.points.at(shift_line.start_idx).point.pose;
  const auto blinker_end_pose = path.path.points.at(shift_line.end_idx).point.pose;
  const auto get_start_pose = [&](const auto & ego_to_shift_start) {
    return ego_to_shift_start > 0.0 ? ego_pose : blinker_start_pose;
  };

  if (!p.turn_signal_on_swerving) {
    return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), false);
  }

  lanelet::ConstLanelet lanelet;
  const auto query_pose = (egos_lane_is_shifted) ? shift_line.end : shift_line.start;
  if (!rh->getClosestLaneletWithinRoute(query_pose, &lanelet)) {
    return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), true);
  }

  const auto left_same_direction_lane = rh->getLeftLanelet(lanelet, true, true);
  const auto left_opposite_lanes = rh->getLeftOppositeLanelets(lanelet);
  const auto right_same_direction_lane = rh->getRightLanelet(lanelet, true, true);
  const auto right_opposite_lanes = rh->getRightOppositeLanelets(lanelet);
  const bool has_left_lane = left_same_direction_lane.has_value() || !left_opposite_lanes.empty();
  const bool has_right_lane =
    right_same_direction_lane.has_value() || !right_opposite_lanes.empty();

  const auto adjacent_hatched_road_markings =
    autoware::experimental::lanelet2_utils::get_adjacent_hatched_road_markings(
      {lanelet}, rh->getLaneletMapPtr());

  const bool left_hatched_road_marking_only =
    !adjacent_hatched_road_markings.left.empty() && !has_left_lane;
  const bool right_hatched_road_marking_only =
    !adjacent_hatched_road_markings.right.empty() && !has_right_lane;

  if (
    (!is_pull_out && !is_lane_change && !is_pull_over) &&
    !isAdjacentToHatchedRoadMarking(
      start_shift_length, end_shift_length, left_hatched_road_marking_only,
      right_hatched_road_marking_only, p.turn_signal_shift_length_threshold)) {
    return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), true);
  }

  // Check if the ego will cross lane bounds.
  // Note that pull out requires blinkers, even if the ego does not cross lane bounds
  if (
    (!is_pull_out && !is_pull_over) &&
    !straddleRoadBound(path, shift_line, current_lanelets, p.vehicle_info)) {
    return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), true);
  }

  // If the ego has stopped and its close to completing its shift, turn off the blinkers
  constexpr double STOPPED_THRESHOLD = 0.1;  // [m/s]
  if (ego_speed < STOPPED_THRESHOLD && !override_ego_stopped_check) {
    if (isNearEndOfShift(
          start_shift_length, end_shift_length, ego_pose.position, current_lanelets,
          p.turn_signal_shift_length_threshold)) {
      return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), true);
    }
  }

  const auto turn_signal_direction = relative_shift_length > 0.0 ? "left" : "right";

  if (is_pull_out && !is_lane_change) {
    const auto dist_to_bound_opt = utils::calc_point_dist_to_closest_lane_boundary(
      *route_handler, ego_pose, vehicle_info, turn_signal_direction);
    const bool is_close_to_bound =
      dist_to_bound_opt.has_value() &&
      std::abs(dist_to_bound_opt.value()) < p.turn_signal_remaining_distance_to_bound_threshold;

    if (is_close_to_bound) {
      return std::make_pair(TurnSignalInfo(p_path_start, p_path_end), true);
    }
  }

  TurnSignalInfo turn_signal_info{};
  turn_signal_info.desired_start_point = get_start_pose(ego_front_to_shift_start);
  turn_signal_info.desired_end_point = blinker_end_pose;
  turn_signal_info.required_start_point = blinker_start_pose;
  turn_signal_info.required_end_point = blinker_end_pose;
  turn_signal_info.turn_signal.command = g_signal_map.at(turn_signal_direction);

  return std::make_pair(turn_signal_info, false);
}

double TurnSignalDecider::calculateRoundaboutBackwardLength(
  const lanelet::ConstLanelet & current_lanelet, const RouteHandler & route_handler,
  double default_backward_length, int max_backward_depth)
{
  const auto roundabouts = current_lanelet.regulatoryElementsAs<lanelet::autoware::Roundabout>();
  double max_backward_length = default_backward_length;

  // Process each roundabout that contains the current lanelet
  for (const auto & roundabout : roundabouts) {
    if (
      roundabout->isInternalLanelet(current_lanelet.id()) ||
      roundabout->isExitLanelet(current_lanelet.id())) {
      // Calculate the maximum distance from current lanelet to any entry lanelet
      const double distance_to_entry = calculateMaxDistanceToDesiredStartPoint(
        current_lanelet, roundabout, route_handler, max_backward_depth);
      max_backward_length = std::max(max_backward_length, distance_to_entry);
    }
  }

  return max_backward_length;
}

double TurnSignalDecider::calculateMaxDistanceToDesiredStartPoint(
  const lanelet::ConstLanelet & start_lanelet,
  const std::shared_ptr<const lanelet::autoware::Roundabout> & roundabout,
  const RouteHandler & route_handler, int max_backward_depth)
{
  // Structure to hold lanelet and accumulated distance information
  struct SearchNode
  {
    lanelet::ConstLanelet lanelet;
    double accumulated_distance = 0.0;
    int depth = 0;
  };

  std::vector<SearchNode> search_stack;
  search_stack.push_back({start_lanelet, 0.0, 0});
  std::unordered_set<lanelet::Id> visited_lanelets;
  visited_lanelets.insert(start_lanelet.id());

  double max_total_distance = 0.0;
  std::set<lanelet::Id> desired_start_lanelet_ids;

  while (!search_stack.empty()) {
    const SearchNode current_node = search_stack.back();
    search_stack.pop_back();

    const double accumulated_distance_with_current_lanelet =
      current_node.accumulated_distance +
      lanelet::geometry::length(lanelet::utils::to2D(current_node.lanelet.centerline3d()));

    // Optional depth guard (skip expanding children when reaching the configured depth)
    if (max_backward_depth >= 0 && current_node.depth >= max_backward_depth) {
      continue;
    }

    // Get all previous lanelets within the route
    lanelet::ConstLanelets previous_lanelets;
    if (!route_handler.getPreviousLaneletsWithinRoute(current_node.lanelet, &previous_lanelets)) {
      continue;  // No previous lanelets found, this path ends here
    }

    // Process each previous lanelet
    for (const auto & prev_lanelet : previous_lanelets) {
      if (visited_lanelets.count(prev_lanelet.id())) {
        continue;  // Already processed this lanelet
      }
      if (roundabout->isEntryLanelet(prev_lanelet.id())) {
        // Found an entry lanelet - get the desired start point if available
        const auto it = roundabout_desired_start_point_association_.find(prev_lanelet.id());
        if (it != roundabout_desired_start_point_association_.end()) {
          // Try to find the lanelet that contains the desired start point
          lanelet::ConstLanelet desired_start_lanelet;
          if (route_handler.getClosestLaneletWithinRoute(it->second, &desired_start_lanelet)) {
            desired_start_lanelet_ids.insert(desired_start_lanelet.id());
          }
        }
      } else if (desired_start_lanelet_ids.count(prev_lanelet.id())) {
        // Found a previously identified desired start lanelet - update max distance
        const double desired_start_lanelet_length =
          lanelet::geometry::length(lanelet::utils::to2D(prev_lanelet.centerline3d()));
        max_total_distance = std::max(
          max_total_distance,
          accumulated_distance_with_current_lanelet + desired_start_lanelet_length);
      }
      // Found another roundabout lanelet - continue search if not yet visited
      visited_lanelets.insert(prev_lanelet.id());
      search_stack.push_back(
        {prev_lanelet, accumulated_distance_with_current_lanelet, current_node.depth + 1});
    }
  }

  return max_total_distance;
}

}  // namespace autoware::behavior_path_planner
