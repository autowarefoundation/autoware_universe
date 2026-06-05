// Copyright 2025 Tier IV, Inc.
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

#include "autoware/behavior_velocity_intersection_module/experimental/util.hpp"
#include "scene_intersection.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>  // for planning_utils::
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>  // for lanelet::autoware::RoadMarking

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace bg = boost::geometry;

Result<IntersectionModule::BasicData, InternalError> IntersectionModule::prepareIntersectionData(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  const auto lanelet_map_ptr = planner_data.route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data.route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto & baselink2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  const auto footprint = planner_data.vehicle_info_.createFootprint(0.0, 0.0);
  const auto & current_pose = planner_data.current_odometry->pose;

  // ==========================================================================================
  // update traffic light information
  // updateTrafficSignalObservation() must be called at first because other traffic signal
  // fuctions use last_valid_observation_
  // ==========================================================================================
  // save previous information before calling updateTrafficSignalObservation()
  previous_prioritized_level_ = getTrafficPrioritizedLevel(planner_data);
  updateTrafficSignalObservation(planner_data);
  const auto traffic_prioritized_level = getTrafficPrioritizedLevel(planner_data);
  const bool is_prioritized =
    traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED;

  const auto lane_id_interval_opt = util::findLaneIdsInterval(path, associative_ids_);
  if (!lane_id_interval_opt) {
    return make_err<IntersectionModule::BasicData, InternalError>(
      "Path has no interval on intersection lane " + std::to_string(lane_id_));
  }
  const auto & lane_id_interval = lane_id_interval_opt.value();

  internal_debug_data_.distance =
    lane_id_interval.end -
    autoware::experimental::trajectory::find_nearest_index(path, current_pose.position);

  if (!intersection_lanelets_) {
    intersection_lanelets_ =
      generateObjectiveLanelets(lanelet_map_ptr, routing_graph_ptr, assigned_lanelet);
  }
  auto & intersection_lanelets = intersection_lanelets_.value();
  debug_data_.attention_area = intersection_lanelets.attention_area();
  debug_data_.first_attention_area = intersection_lanelets.first_attention_area();
  debug_data_.occlusion_attention_area = intersection_lanelets.occlusion_attention_area();
  debug_data_.adjacent_area = intersection_lanelets.adjacent_area();

  // ==========================================================================================
  // at the very first time of registration of this module, the path may not be conflicting with
  // the attention area, so update() is called to update the internal data as well as traffic
  // light info
  // ==========================================================================================
  intersection_lanelets.update(is_prioritized, path, lane_id_interval, footprint, baselink2front);

  const auto & conflicting_lanelets = intersection_lanelets.conflicting();
  const auto & first_conflicting_area_opt = intersection_lanelets.first_conflicting_area();
  const auto & first_conflicting_lane_opt = intersection_lanelets.first_conflicting_lane();
  if (conflicting_lanelets.empty() || !first_conflicting_area_opt || !first_conflicting_lane_opt) {
    // this is abnormal
    return make_err<IntersectionModule::BasicData, InternalError>("conflicting area is empty");
  }
  const auto & first_conflicting_lane = first_conflicting_lane_opt.value();
  const auto & first_conflicting_area = first_conflicting_area_opt.value();

  // ==========================================================================================
  // even if the attention area is null, stuck vehicle stop line needs to be generated from
  // conflicting lanes, so dummy_first_attention_lane is used
  // ==========================================================================================
  const auto & dummy_first_attention_lane = intersection_lanelets.first_attention_lane()
                                              ? intersection_lanelets.first_attention_lane().value()
                                              : first_conflicting_lane;

  const auto intersection_stoplines_opt = generateIntersectionStopLines(
    assigned_lanelet, first_conflicting_area, dummy_first_attention_lane, previous_stop_pose_,
    lane_id_interval, path, left_bound, right_bound, planner_data);
  if (!intersection_stoplines_opt) {
    return make_err<IntersectionModule::BasicData, InternalError>(
      "failed to generate intersection_stoplines");
  }
  const auto & intersection_stoplines = intersection_stoplines_opt.value();
  const auto & closest_s = intersection_stoplines.closest_s;

  const auto & first_attention_area_opt = intersection_lanelets.first_attention_area();
  const auto & conflicting_area = intersection_lanelets.conflicting_area();
  const auto lanelets_on_path =
    util::getLaneletsOnPathFromCurrent(path, lanelet_map_ptr, current_pose);
  const auto path_lanelets_opt = generatePathLanelets(
    lanelets_on_path, lane_id_interval, path, first_conflicting_area, conflicting_area,
    first_attention_area_opt, intersection_lanelets.attention_area(), closest_s, planner_data);
  if (!path_lanelets_opt.has_value()) {
    return make_err<IntersectionModule::BasicData, InternalError>(
      "failed to generate PathLanelets");
  }
  const auto & path_lanelets = path_lanelets_opt.value();

  if (!occlusion_attention_divisions_) {
    occlusion_attention_divisions_ = generateDetectionLaneDivisions(
      intersection_lanelets.occlusion_attention(), intersection_lanelets.attention_non_preceding(),
      routing_graph_ptr, planner_data.occupancy_grid->info.resolution);
  }

  if (has_traffic_light_) {
    const bool is_green_solid_on = isGreenSolidOn();
    if (is_green_solid_on && !initial_green_light_observed_time_) {
      const auto assigned_lane_begin_point =
        autoware::experimental::lanelet2_utils::to_ros(assigned_lanelet.centerline().front());
      const auto assigned_lane_begin_s =
        autoware::experimental::trajectory::find_nearest_index(path, assigned_lane_begin_point);
      const auto approached_assigned_lane =
        (assigned_lane_begin_s - closest_s) <
        planner_param_.collision_detection.yield_on_green_traffic_light
          .distance_to_assigned_lanelet_start;
      if (approached_assigned_lane) {
        initial_green_light_observed_time_ = clock_->now();
      }
    }
  }

  return make_ok<IntersectionModule::BasicData, InternalError>(
    intersection_stoplines, lane_id_interval, path_lanelets);
}

std::optional<double> IntersectionModule::getStopLineIndexFromMap(
  const Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const Interval & lane_id_interval,
  const lanelet::ConstLanelet & assigned_lanelet, const PlannerData & planner_data) const
{
  const auto road_markings =
    assigned_lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stopline;
  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (type == lanelet::AttributeValueString::StopLine) {
      stopline.push_back(road_marking->roadMarking());
      break;  // only one stopline exists.
    }
  }
  if (stopline.empty()) {
    return std::nullopt;
  }

  const auto p_start = stopline.front().front();
  const auto p_end = stopline.front().back();
  const auto extended_stopline = planning_utils::extendSegmentToBounds(
    {p_start.basicPoint2d(), p_end.basicPoint2d()}, left_bound, right_bound);

  const auto intersections = autoware::experimental::trajectory::crossed(
    path, extended_stopline, lane_id_interval.start, lane_id_interval.end);
  if (!intersections.empty()) {
    return intersections.front();
  }

  geometry_msgs::msg::Pose stop_point_from_map;
  stop_point_from_map.position.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.position.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.position.z = 0.5 * (p_start.z() + p_end.z());

  const auto stop_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, stop_point_from_map, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);
  if (stop_s) {
    return stop_s.value();
  }

  return autoware::experimental::trajectory::find_nearest_index(path, stop_point_from_map.position);
}

std::optional<IntersectionStopLines> IntersectionModule::generateIntersectionStopLines(
  const lanelet::ConstLanelet & assigned_lanelet,
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const lanelet::ConstLanelet & first_attention_lane,
  const IntersectionStopLines::PreviousStopPose & previous_stop_pose,
  const autoware::experimental::trajectory::Interval & lane_id_interval, Trajectory & path,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data) const
{
  const auto & use_stuck_stopline = planner_param_.stuck_vehicle.use_stuck_stopline;
  const auto & stopline_margin = planner_param_.common.default_stopline_margin;
  const auto & peeking_offset = planner_param_.occlusion.peeking_offset;

  const auto & first_attention_area = first_attention_lane.polygon3d();
  const auto & baselink2front = planner_data.vehicle_info_.max_longitudinal_offset_m;

  // (1) find the index of the first point whose vehicle footprint on it intersects with
  // attention_area for the first time
  const auto local_footprint = planner_data.vehicle_info_.createFootprint(0.0, 0.0);
  const auto first_footprint_inside_1st_attention_s_opt =
    util::getFirstIndexInsidePolygonByFootprint(
      path, lane_id_interval, first_attention_area, local_footprint, baselink2front);
  if (!first_footprint_inside_1st_attention_s_opt) {
    return std::nullopt;
  }

  IntersectionStopLines intersection_stoplines;
  intersection_stoplines.first_attention_stopline =
    first_footprint_inside_1st_attention_s_opt.value();

  // (2) pass judge line position on interpolated path
  const auto & braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    planner_data.current_velocity->twist.linear.x,
    planner_data.current_acceleration->accel.accel.linear.x, planner_param_.common.max_accel,
    planner_param_.common.max_jerk, 0.0);
  intersection_stoplines.pass_judge_line = std::clamp(
    intersection_stoplines.first_attention_stopline - braking_dist -
      planner_param_.common.pass_judge_line_margin,
    0.0, path.length());

  // (3) ego front stop line position on interpolated path
  const auto & current_pose = planner_data.current_odometry->pose;
  const auto closest_s_opt = autoware::experimental::trajectory::find_first_nearest_index(
    path, current_pose, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);
  if (closest_s_opt) {
    intersection_stoplines.closest_s = closest_s_opt.value();
  } else {
    intersection_stoplines.closest_s =
      autoware::experimental::trajectory::find_nearest_index(path, current_pose.position);
  }

  // (4) default stop line position on interpolated path
  intersection_stoplines.default_stopline = [&]() -> std::optional<double> {
    const auto map_stop_s = getStopLineIndexFromMap(
      path, left_bound, right_bound, lane_id_interval, assigned_lanelet, planner_data);
    if (!map_stop_s) {
      return std::nullopt;
    }
    auto default_stopline_s = map_stop_s.value() - baselink2front;
    if (default_stopline_s < 0) {
      default_stopline_s = intersection_stoplines.first_attention_stopline - stopline_margin;
    }
    if (default_stopline_s < 0) {
      return std::nullopt;
    }
    return default_stopline_s;
  }();

  // (5) collision stop line
  intersection_stoplines.collision_stopline = [&]() -> std::optional<double> {
    if (previous_stop_pose.collision_stopline_pose) {
      const auto previous_stop_pose_s =
        autoware::experimental::trajectory::find_first_nearest_index(
          path, previous_stop_pose.collision_stopline_pose.value(),
          planner_data.ego_nearest_dist_threshold, planner_data.ego_nearest_yaw_threshold);
      if (previous_stop_pose_s) {
        return previous_stop_pose_s.value();
      }
      return autoware::experimental::trajectory::find_nearest_index(
        path, previous_stop_pose.collision_stopline_pose.value().position);
    }
    if (!intersection_stoplines.default_stopline) {
      return std::nullopt;
    }

    const auto dist_to_default_stopline =
      intersection_stoplines.default_stopline.value() - intersection_stoplines.closest_s;

    if (dist_to_default_stopline > braking_dist) {
      // default stop line is feasible
      return intersection_stoplines.default_stopline.value();
    }
    // default stop line is infeasible
    if (intersection_stoplines.closest_s < intersection_stoplines.pass_judge_line) {
      // but ego is still before pass judge line
      return std::clamp(intersection_stoplines.closest_s + braking_dist, 0.0, path.length());
    }
    if (previous_stop_pose.occlusion_peeking_stopline_pose) {
      // NOTE(soblin): see IntersectionModule::isOverPassJudgeLinesStatus. during peeking, ego can
      // pass original pass judge line
      const auto previous_stop_pose_s =
        autoware::experimental::trajectory::find_first_nearest_index(
          path, previous_stop_pose.occlusion_peeking_stopline_pose.value(),
          planner_data.ego_nearest_dist_threshold, planner_data.ego_nearest_yaw_threshold);
      if (previous_stop_pose_s) {
        return previous_stop_pose_s.value();
      }
      return autoware::experimental::trajectory::find_nearest_index(
        path, previous_stop_pose.occlusion_peeking_stopline_pose.value().position);
    }
    // ego is over pass judge line
    return intersection_stoplines.first_attention_stopline;
  }();

  // (6) occlusion peeking stop line position on interpolated path
  // static position
  const auto static_occlusion_peeking_line_s = [&]() -> std::optional<double> {
    // NOTE: if footprints[0] is already inside the attention area, invalid
    const auto & base_pose0 =
      path.compute(intersection_stoplines.default_stopline.value()).point.pose;
    const auto path_footprint0 =
      autoware_utils::transform_vector(local_footprint, autoware_utils::pose2transform(base_pose0));
    if (bg::intersects(
          path_footprint0, lanelet::utils::to2D(first_attention_area).basicPolygon())) {
      return std::nullopt;
    }
    return std::clamp(
      intersection_stoplines.first_attention_stopline + peeking_offset, 0.0, path.length());
  }();

  // dynamic position
  intersection_stoplines.occlusion_peeking_stopline = [&]() -> std::optional<double> {
    if (previous_stop_pose.occlusion_peeking_stopline_pose) {
      const auto previous_stop_pose_s =
        autoware::experimental::trajectory::find_first_nearest_index(
          path, previous_stop_pose.occlusion_peeking_stopline_pose.value(),
          planner_data.ego_nearest_dist_threshold, planner_data.ego_nearest_yaw_threshold);
      if (previous_stop_pose_s) {
        return previous_stop_pose_s.value();
      }
      return autoware::experimental::trajectory::find_nearest_index(
        path, previous_stop_pose.occlusion_peeking_stopline_pose.value().position);
    }
    if (!static_occlusion_peeking_line_s) {
      return std::nullopt;
    }

    const auto dist_to_static_stopline =
      static_occlusion_peeking_line_s.value() - intersection_stoplines.closest_s;

    if (dist_to_static_stopline > braking_dist) {
      // static stop line position is feasible
      return static_occlusion_peeking_line_s.value();
    }
    // static stop line is infeasible
    if (intersection_stoplines.closest_s < intersection_stoplines.pass_judge_line) {
      // but ego is still before pass judge line
      return std::clamp(intersection_stoplines.closest_s + braking_dist, 0.0, path.length());
    }
    // ego is over pass judge line
    return intersection_stoplines.first_attention_stopline;
  }();

  // (7) stuck vehicle stopline position on interpolated path
  intersection_stoplines.stuck_stopline = [&]() -> std::optional<double> {
    auto stuck_stopline_s = 0.0;
    if (use_stuck_stopline) {
      // ==========================================================================================
      // NOTE: when ego vehicle is approaching attention area and already passed
      // first_conflicting_area, this could be null.
      // ==========================================================================================
      const auto stuck_stopline_s_opt = util::getFirstIndexInsidePolygonByFootprint(
        path, lane_id_interval, first_conflicting_area, local_footprint, baselink2front);
      if (!stuck_stopline_s_opt) {
        return std::nullopt;
      }
      stuck_stopline_s = stuck_stopline_s_opt.value() - stopline_margin;
    } else {
      stuck_stopline_s = lane_id_interval.start - (stopline_margin + baselink2front);
    }
    if (stuck_stopline_s < 0.0) {
      return std::nullopt;
    }
    return std::clamp(stuck_stopline_s, 0.0, path.length());
  }();

  // (8) the position where ego footprint most approaches the opposite boundary of
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");
  const auto compute_lane_end_azimuth = [](const lanelet::ConstLanelet & lane) {
    const auto & centerline = lane.centerline();
    const auto & p1 = centerline[(centerline.size() - 2)];
    const auto & p2 = centerline.back();
    return autoware_utils_geometry::calc_azimuth_angle(
      autoware::experimental::lanelet2_utils::to_ros(p1),
      autoware::experimental::lanelet2_utils::to_ros(p2));
  };
  const double merging_angle_diff = autoware_utils_math::normalize_radian(
    compute_lane_end_azimuth(assigned_lanelet) - compute_lane_end_azimuth(first_attention_lane));
  const bool is_merging = std::fabs(merging_angle_diff) <
                          planner_param_.conservative_merging.merging_judge_angle_threshold;
  const std::optional<double> maximum_footprint_overshoot_line_s_opt =
    is_merging
      ? util::findMaximumFootprintOvershootPosition(
          path, local_footprint, first_attention_lane,
          planner_param_.conservative_merging.minimum_lateral_distance_threshold, turn_direction)
      : std::nullopt;
  intersection_stoplines.maximum_footprint_overshoot_line =
    maximum_footprint_overshoot_line_s_opt.value_or(0.0);

  return intersection_stoplines;
}

static std::vector<std::deque<lanelet::ConstLanelet>> getPrecedingLaneletsUptoIntersectionRecursive(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet,
  const double length, const lanelet::ConstLanelets & exclude_lanelets)
{
  std::vector<std::deque<lanelet::ConstLanelet>> preceding_lanelet_sequences;

  const auto prev_lanelets = graph->previous(lanelet);
  const double lanelet_length = lanelet::geometry::length3d(lanelet);

  // end condition of the recursive function
  if (prev_lanelets.empty() || lanelet_length >= length) {
    preceding_lanelet_sequences.push_back({lanelet});
    return preceding_lanelet_sequences;
  }

  for (const auto & prev_lanelet : prev_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, prev_lanelet)) {
      // if prev_lanelet is included in exclude_lanelets,
      // remove prev_lanelet from preceding_lanelet_sequences
      continue;
    }
    if (const std::string turn_direction = prev_lanelet.attributeOr("turn_direction", "else");
        turn_direction == "left" || turn_direction == "right") {
      continue;
    }

    // get lanelet sequence after prev_lanelet
    auto tmp_lanelet_sequences = getPrecedingLaneletsUptoIntersectionRecursive(
      graph, prev_lanelet, length - lanelet_length, exclude_lanelets);
    for (auto & tmp_lanelet_sequence : tmp_lanelet_sequences) {
      tmp_lanelet_sequence.push_back(lanelet);
      preceding_lanelet_sequences.push_back(tmp_lanelet_sequence);
    }
  }

  if (preceding_lanelet_sequences.empty()) {
    preceding_lanelet_sequences.push_back({lanelet});
  }
  return preceding_lanelet_sequences;
}

static std::vector<lanelet::ConstLanelets> getPrecedingLaneletsUptoIntersection(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet,
  const double length, const lanelet::ConstLanelets & exclude_lanelets)
{
  std::vector<lanelet::ConstLanelets> lanelet_sequences_vec;
  const auto prev_lanelets = graph->previous(lanelet);
  for (const auto & prev_lanelet : prev_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, prev_lanelet)) {
      // if prev_lanelet is included in exclude_lanelets,
      // remove prev_lanelet from preceding_lanelet_sequences
      continue;
    }
    if (const std::string turn_direction = prev_lanelet.attributeOr("turn_direction", "else");
        turn_direction == "left" || turn_direction == "right") {
      continue;
    }
    // convert deque into vector
    const auto lanelet_sequences_deq =
      getPrecedingLaneletsUptoIntersectionRecursive(graph, prev_lanelet, length, exclude_lanelets);
    for (const auto & lanelet_sequence : lanelet_sequences_deq) {
      lanelet_sequences_vec.emplace_back(lanelet_sequence.begin(), lanelet_sequence.end());
    }
  }
  return lanelet_sequences_vec;
}

IntersectionLanelets IntersectionModule::generateObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const lanelet::ConstLanelet assigned_lanelet) const
{
  const double detection_area_length = planner_param_.common.attention_area_length;
  const double occlusion_detection_area_length =
    planner_param_.occlusion.occlusion_attention_area_length;
  const bool consider_wrong_direction_vehicle =
    planner_param_.collision_detection.consider_wrong_direction_vehicle;

  // retrieve a stopline associated with a traffic light
  bool has_traffic_light = false;
  if (const auto tl_reg_elems = assigned_lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
      tl_reg_elems.size() != 0) {
    const auto tl_reg_elem = tl_reg_elems.front();
    const auto stopline_opt = tl_reg_elem->stopLine();
    if (!!stopline_opt) has_traffic_light = true;
  }

  // for low priority lane
  // if ego_lane has right of way (i.e. is high priority),
  // ignore yieldLanelets (i.e. low priority lanes)
  lanelet::ConstLanelets yield_lanelets{};
  const auto right_of_ways = assigned_lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    if (lanelet::utils::contains(right_of_way->rightOfWayLanelets(), assigned_lanelet)) {
      for (const auto & yield_lanelet : right_of_way->yieldLanelets()) {
        yield_lanelets.push_back(yield_lanelet);
        for (const auto & previous_lanelet : routing_graph_ptr->previous(yield_lanelet)) {
          yield_lanelets.push_back(previous_lanelet);
        }
      }
    }
  }

  // get all following lanes of previous lane
  lanelet::ConstLanelets ego_lanelets{};
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    ego_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(ego_lanelets, following_lanelet)) {
        continue;
      }
      ego_lanelets.push_back(following_lanelet);
    }
  }

  // get conflicting lanes on assigned lanelet
  const auto & conflicting_lanelets =
    autoware::experimental::lanelet2_utils::get_conflicting_lanelets(
      assigned_lanelet, routing_graph_ptr);
  std::vector<lanelet::ConstLanelet> adjacent_followings;

  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    for (const auto & following_lanelet : routing_graph_ptr->following(conflicting_lanelet)) {
      adjacent_followings.push_back(following_lanelet);
    }
    for (const auto & following_lanelet : routing_graph_ptr->previous(conflicting_lanelet)) {
      adjacent_followings.push_back(following_lanelet);
    }
  }

  // final objective lanelets
  lanelet::ConstLanelets detection_lanelets;
  lanelet::ConstLanelets conflicting_ex_ego_lanelets;
  // conflicting lanes is necessary to get stopline for stuck vehicle
  for (auto && conflicting_lanelet : conflicting_lanelets) {
    if (!lanelet::utils::contains(ego_lanelets, conflicting_lanelet))
      conflicting_ex_ego_lanelets.push_back(conflicting_lanelet);
  }

  // exclude yield lanelets and ego lanelets from detection_lanelets
  if (turn_direction_ == std::string("straight") && has_traffic_light) {
    // if assigned lanelet is "straight" with traffic light, detection area is not necessary
  } else {
    if (consider_wrong_direction_vehicle) {
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        if (lanelet::utils::contains(yield_lanelets, conflicting_lanelet)) {
          continue;
        }
        detection_lanelets.push_back(conflicting_lanelet);
      }
      for (const auto & adjacent_following : adjacent_followings) {
        detection_lanelets.push_back(adjacent_following);
      }
    } else {
      // otherwise we need to know the priority from RightOfWay
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        if (
          lanelet::utils::contains(yield_lanelets, conflicting_lanelet) ||
          lanelet::utils::contains(ego_lanelets, conflicting_lanelet)) {
          continue;
        }
        detection_lanelets.push_back(conflicting_lanelet);
      }
    }
  }

  // get possible lanelet path that reaches conflicting_lane longer than given length
  lanelet::ConstLanelets detection_and_preceding_lanelets;
  {
    const double length = detection_area_length;
    std::set<lanelet::Id> detection_ids;
    for (const auto & ll : detection_lanelets) {
      // Preceding lanes does not include detection_lane so add them at the end
      const auto & inserted = detection_ids.insert(ll.id());
      if (inserted.second) detection_and_preceding_lanelets.push_back(ll);
      // get preceding lanelets without ego_lanelets
      // to prevent the detection area from including the ego lanes and its' preceding lanes.
      const auto lanelet_sequences =
        autoware::experimental::lanelet2_utils::get_preceding_lanelet_sequences(
          ll, routing_graph_ptr, length, ego_lanelets);
      for (const auto & ls : lanelet_sequences) {
        for (const auto & l : ls) {
          const auto & inner_inserted = detection_ids.insert(l.id());
          if (inner_inserted.second) detection_and_preceding_lanelets.push_back(l);
        }
      }
    }
  }

  lanelet::ConstLanelets occlusion_detection_and_preceding_lanelets;
  {
    const double length = occlusion_detection_area_length;
    std::set<lanelet::Id> detection_ids;
    for (const auto & ll : detection_lanelets) {
      // Preceding lanes does not include detection_lane so add them at the end
      const auto & inserted = detection_ids.insert(ll.id());
      if (inserted.second) occlusion_detection_and_preceding_lanelets.push_back(ll);
      // get preceding lanelets without ego_lanelets
      // to prevent the detection area from including the ego lanes and its' preceding lanes.
      const auto lanelet_sequences =
        getPrecedingLaneletsUptoIntersection(routing_graph_ptr, ll, length, ego_lanelets);
      for (const auto & ls : lanelet_sequences) {
        for (const auto & l : ls) {
          const auto & inner_inserted = detection_ids.insert(l.id());
          if (inner_inserted.second) occlusion_detection_and_preceding_lanelets.push_back(l);
        }
      }
    }
  }

  auto [attention_lanelets, original_attention_lanelet_sequences] =
    util::mergeLaneletsByTopologicalSort(
      detection_and_preceding_lanelets, detection_lanelets, routing_graph_ptr);

  IntersectionLanelets result;
  result.attention_ = std::move(attention_lanelets);
  for (const auto & original_attention_lanelet_seq : original_attention_lanelet_sequences) {
    // NOTE: in mergeLaneletsByTopologicalSort(), sub_ids are empty checked, so it is ensured that
    // back() exists.
    std::optional<lanelet::ConstLineString3d> stopline{std::nullopt};
    for (auto it = original_attention_lanelet_seq.rbegin();
         it != original_attention_lanelet_seq.rend(); ++it) {
      const auto traffic_lights = it->regulatoryElementsAs<lanelet::TrafficLight>();
      for (const auto & traffic_light : traffic_lights) {
        const auto stopline_opt = traffic_light->stopLine();
        if (!stopline_opt) continue;
        stopline = stopline_opt.get();
        break;
      }
      if (stopline) break;
    }
    result.attention_stoplines_.push_back(stopline);
  }
  result.attention_non_preceding_ = std::move(detection_lanelets);
  for (unsigned i = 0; i < result.attention_non_preceding_.size(); ++i) {
    std::optional<lanelet::ConstLineString3d> stopline = std::nullopt;
    const auto & ll = result.attention_non_preceding_.at(i);
    const auto traffic_lights = ll.regulatoryElementsAs<lanelet::TrafficLight>();
    for (const auto & traffic_light : traffic_lights) {
      const auto stopline_opt = traffic_light->stopLine();
      if (!stopline_opt) continue;
      stopline = stopline_opt.get();
    }
    result.attention_non_preceding_stoplines_.push_back(stopline);
  }
  result.conflicting_ = std::move(conflicting_ex_ego_lanelets);
  result.adjacent_ = planning_utils::getConstLaneletsFromIds(lanelet_map_ptr, associative_ids_);
  // NOTE: occlusion_attention is not inverted here
  // TODO(Mamoru Sobue): apply mergeLaneletsByTopologicalSort for occlusion lanelets as well and
  // then trim part of them based on curvature threshold
  result.occlusion_attention_ = std::move(occlusion_detection_and_preceding_lanelets);

  // NOTE: to properly update(), each element in conflicting_/conflicting_area_,
  // attention_non_preceding_/attention_non_preceding_area_ need to be matched
  result.attention_area_ = util::getPolygon3dFromLanelets(result.attention_);
  result.attention_non_preceding_area_ =
    util::getPolygon3dFromLanelets(result.attention_non_preceding_);
  result.conflicting_area_ = util::getPolygon3dFromLanelets(result.conflicting_);
  result.adjacent_area_ = util::getPolygon3dFromLanelets(result.adjacent_);
  result.occlusion_attention_area_ = util::getPolygon3dFromLanelets(result.occlusion_attention_);
  return result;
}

std::optional<PathLanelets> IntersectionModule::generatePathLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const Interval & lane_id_interval,
  const Trajectory & path, const lanelet::CompoundPolygon3d & first_conflicting_area,
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const double closest_s,
  const PlannerData & planner_data) const
{
  const double width = planner_data.vehicle_info_.vehicle_width_m;
  static constexpr double path_lanelet_interval = 1.5;

  PathLanelets path_lanelets;
  // prev
  path_lanelets.prev = util::getPrevLanelets(lanelets_on_path, associative_ids_);
  path_lanelets.all = path_lanelets.prev;

  // entry2ego if exist
  if (closest_s > lane_id_interval.start) {
    const auto path_lanelet_entry2ego_opt = util::generatePathLanelet(
      path, lane_id_interval.start, closest_s, width, path_lanelet_interval);
    if (path_lanelet_entry2ego_opt.has_value()) {
      path_lanelets.all.push_back(path_lanelet_entry2ego_opt.value());
    }
  }

  // ego_or_entry2exit
  const auto ego_or_entry_s = std::max(closest_s, lane_id_interval.start);
  const auto path_lanelet_ego_or_entry2exit_opt = util::generatePathLanelet(
    path, ego_or_entry_s, lane_id_interval.end, width, path_lanelet_interval);
  if (path_lanelet_ego_or_entry2exit_opt.has_value()) {
    path_lanelets.ego_or_entry2exit = path_lanelet_ego_or_entry2exit_opt.value();
    path_lanelets.all.push_back(path_lanelets.ego_or_entry2exit);
  }

  // next
  if (lane_id_interval.end < path.length()) {
    const auto next_id = path.compute(lane_id_interval.end).lane_ids.front();
    const auto next_lane_interval_opt = util::findLaneIdsInterval(path, {next_id});
    if (next_lane_interval_opt) {
      const auto & [next_start_s, next_end_s] = next_lane_interval_opt.value();
      const auto path_lanelet_next_opt =
        util::generatePathLanelet(path, next_start_s, next_end_s, width, path_lanelet_interval);
      path_lanelets.next = path_lanelet_next_opt;
      if (path_lanelet_next_opt.has_value()) {
        path_lanelets.all.push_back(path_lanelets.next.value());
      }
    }
  }

  const auto first_inside_conflicting_s_opt =
    first_attention_area.has_value()
      ? util::getFirstIndexInsidePolygon(path, lane_id_interval, first_attention_area.value())
      : util::getFirstIndexInsidePolygon(path, lane_id_interval, first_conflicting_area);
  const auto last_inside_conflicting_s_opt =
    first_attention_area.has_value()
      ? util::getFirstIndexInsidePolygons(path, lane_id_interval, attention_areas, false)
      : util::getFirstIndexInsidePolygons(path, lane_id_interval, conflicting_areas, false);

  if (!first_inside_conflicting_s_opt || !last_inside_conflicting_s_opt) {
    return path_lanelets;
  }
  const auto first_inside_conflicting_s = first_inside_conflicting_s_opt.value();
  const auto last_inside_conflicting_s = last_inside_conflicting_s_opt.value().s;

  // conflicting_interval
  const auto path_lanelet_conflicting_interval_opt = util::generatePathLanelet(
    path, first_inside_conflicting_s, last_inside_conflicting_s, width, path_lanelet_interval);
  if (path_lanelet_conflicting_interval_opt.has_value()) {
    path_lanelets.conflicting_interval_and_remaining.push_back(
      std::move(path_lanelet_conflicting_interval_opt.value()));
  }

  if (last_inside_conflicting_s >= lane_id_interval.end) {
    return path_lanelets;
  }

  // remaining
  const auto path_lanelet_remaining_opt = util::generatePathLanelet(
    path, last_inside_conflicting_s, lane_id_interval.end, width, path_lanelet_interval);
  if (path_lanelet_remaining_opt.has_value()) {
    path_lanelets.conflicting_interval_and_remaining.push_back(
      std::move(path_lanelet_remaining_opt.value()));
  }
  return path_lanelets;
}

std::vector<lanelet::ConstLineString3d> IntersectionModule::generateDetectionLaneDivisions(
  const lanelet::ConstLanelets & occlusion_detection_lanelets,
  const lanelet::ConstLanelets & conflicting_detection_lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution) const
{
  const double curvature_threshold =
    planner_param_.occlusion.attention_lane_crop_curvature_threshold;
  const double curvature_calculation_ds =
    planner_param_.occlusion.attention_lane_curvature_calculation_ds;

  using autoware::experimental::lanelet2_utils::get_centerline_with_offset;

  // (0) remove curved
  lanelet::ConstLanelets detection_lanelets;
  for (const auto & detection_lanelet : occlusion_detection_lanelets) {
    // TODO(Mamoru Sobue): instead of ignoring, only trim straight part of lanelet
    const auto fine_centerline = autoware::experimental::lanelet2_utils::get_fine_centerline(
      detection_lanelet, curvature_calculation_ds);
    const double highest_curvature = util::getHighestCurvature(fine_centerline);
    if (highest_curvature > curvature_threshold) {
      continue;
    }
    detection_lanelets.push_back(detection_lanelet);
  }

  std::vector<lanelet::ConstLineString3d> detection_divisions;
  if (detection_lanelets.empty()) {
    // NOTE(soblin): due to the above filtering detection_lanelets may be empty or do not contain
    // conflicting_detection_lanelets
    // OK to return empty detection_divisions
    return detection_divisions;
  }

  // (1) tsort detection_lanelets
  const auto [merged_detection_lanelets, originals] = util::mergeLaneletsByTopologicalSort(
    detection_lanelets, conflicting_detection_lanelets, routing_graph_ptr);

  // (2) merge each branch to one lanelet
  // NOTE: somehow bg::area() for merged lanelet does not work, so calculate it here
  std::vector<std::pair<lanelet::ConstLanelet, double>> merged_lanelet_with_area;
  for (unsigned i = 0; i < merged_detection_lanelets.size(); ++i) {
    const auto & merged_detection_lanelet = merged_detection_lanelets.at(i);
    const auto & original = originals.at(i);
    double area = 0;
    for (const auto & partition : original) {
      area += bg::area(partition.polygon2d().basicPolygon());
    }
    merged_lanelet_with_area.emplace_back(merged_detection_lanelet, area);
  }

  // (3) discretize each merged lanelet
  for (const auto & [merged_lanelet, area] : merged_lanelet_with_area) {
    const double length = bg::length(merged_lanelet.centerline());
    const double width = area / length;
    for (int i = 0; i < static_cast<int>(width / resolution); ++i) {
      const double offset = resolution * i - width / 2;
      detection_divisions.push_back(
        get_centerline_with_offset(merged_lanelet, offset, resolution).invert());
    }
    detection_divisions.push_back(
      get_centerline_with_offset(merged_lanelet, width / 2, resolution).invert());
  }
  return detection_divisions;
}

}  // namespace autoware::behavior_velocity_planner::experimental
