// Copyright 2024 Tier IV, Inc.
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

#include "scene_roundabout.hpp"

#include <autoware/behavior_velocity_intersection_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for to_bg2d
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>  // for planning_utils::
#include <autoware/interpolation/spline_interpolation_points_2d.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>  // for lanelet::autoware::RoadMarking
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Point.h>

#include <algorithm>
#include <list>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware_utils_geometry
{

template <>
inline geometry_msgs::msg::Point get_point(const lanelet::ConstPoint3d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  return point;
}

}  // namespace autoware_utils_geometry

namespace
{
namespace bg = boost::geometry;

lanelet::ConstLanelets getPrevLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const std::set<lanelet::Id> & associative_ids)
{
  lanelet::ConstLanelets previous_lanelets;
  for (const auto & ll : lanelets_on_path) {
    if (associative_ids.find(ll.id()) != associative_ids.end()) {
      return previous_lanelets;
    }
    previous_lanelets.push_back(ll);
  }
  return previous_lanelets;
}

// end inclusive
lanelet::ConstLanelet generatePathLanelet(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const size_t start_idx,
  const size_t end_idx, const double width, const double interval)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  size_t prev_idx = start_idx;
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const auto & p = path.points.at(i).point.pose;
    const auto & p_prev = path.points.at(prev_idx).point.pose;
    if (i != start_idx && autoware_utils::calc_distance2d(p_prev, p) < interval) {
      continue;
    }
    prev_idx = i;
    const double yaw = tf2::getYaw(p.orientation);
    const double x = p.position.x;
    const double y = p.position.y;
    // NOTE: maybe this is opposite
    const double left_x = x + width / 2 * std::sin(yaw);
    const double left_y = y - width / 2 * std::cos(yaw);
    const double right_x = x - width / 2 * std::sin(yaw);
    const double right_y = y + width / 2 * std::cos(yaw);
    lefts.emplace_back(lanelet::InvalId, left_x, left_y, p.position.z);
    rights.emplace_back(lanelet::InvalId, right_x, right_y, p.position.z);
  }
  lanelet::LineString3d left = lanelet::LineString3d(lanelet::InvalId, lefts);
  lanelet::LineString3d right = lanelet::LineString3d(lanelet::InvalId, rights);

  return lanelet::Lanelet(lanelet::InvalId, left, right);
}

std::optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>> getFirstPointInsidePolygons(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval,
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const bool search_forward = true)
{
  if (search_forward) {
    for (size_t i = lane_interval.first; i <= lane_interval.second; ++i) {
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        if (bg::within(autoware::behavior_velocity_planner::to_bg2d(p), polygon_2d)) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
    }
  } else {
    for (size_t i = lane_interval.second; i >= lane_interval.first; --i) {
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        if (bg::within(autoware::behavior_velocity_planner::to_bg2d(p), polygon_2d)) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
      if (i == 0) {
        break;
      }
    }
  }
  return std::nullopt;
}

}  // namespace

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

Result<RoundaboutModule::BasicData, InternalError> RoundaboutModule::prepareRoundaboutData(
  PathWithLaneId * path)
{
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  const auto footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  const auto & current_pose = planner_data_->current_odometry->pose;

  // spline interpolation
  const auto interpolated_path_info_opt = util::generateInterpolatedPath(
    lane_id_, associative_ids_, *path, planner_param_.common.path_interpolation_ds, logger_);
  if (!interpolated_path_info_opt) {
    return make_err<RoundaboutModule::BasicData, InternalError>("splineInterpolate failed");
  }

  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  if (!interpolated_path_info.lane_id_interval) {
    return make_err<RoundaboutModule::BasicData, InternalError>(
      "Path has no interval on roundabout lane " + std::to_string(lane_id_));
  }

  const auto & path_ip = interpolated_path_info.path;
  const auto & path_ip_roundabout_end = interpolated_path_info.lane_id_interval.value().second;
  internal_debug_data_.distance = autoware::motion_utils::calcSignedArcLength(
    path->points, current_pose.position,
    path_ip.points.at(path_ip_roundabout_end).point.pose.position);

  if (!roundabout_lanelets_) {
    roundabout_lanelets_ =
      generateObjectiveLanelets(lanelet_map_ptr, routing_graph_ptr, assigned_lanelet);
  }
  auto & roundabout_lanelets = roundabout_lanelets_.value();
  debug_data_.attention_area = roundabout_lanelets.attention_area();
  debug_data_.first_attention_area = roundabout_lanelets.first_attention_area();
  debug_data_.adjacent_area = roundabout_lanelets.adjacent_area();

  // ==========================================================================================
  // at the very first time of registration of this module, the path may not be conflicting with
  // the attention area, so update() is called to update the internal data
  // ==========================================================================================
  roundabout_lanelets.update(interpolated_path_info, footprint, baselink2front, routing_graph_ptr);

  const auto & conflicting_lanelets = roundabout_lanelets.conflicting();
  const auto & first_conflicting_area_opt = roundabout_lanelets.first_conflicting_area();
  if (conflicting_lanelets.empty()) {
    // this is abnormal
    return make_err<RoundaboutModule::BasicData, InternalError>("conflicting area is empty");
  }
  if (!roundabout_lanelets.first_attention_lane()) {
    // this is abnormal
    return make_err<RoundaboutModule::BasicData, InternalError>("first attention area is null");
  }
  const auto & first_conflicting_area = first_conflicting_area_opt.value();

  const auto roundabout_stoplines_opt = generateRoundaboutStopLines(
    roundabout_lanelets.first_attention_lane().value(), interpolated_path_info, path);
  if (!roundabout_stoplines_opt) {
    return make_err<RoundaboutModule::BasicData, InternalError>(
      "failed to generate roundabout_stoplines");
  }
  const auto & roundabout_stoplines = roundabout_stoplines_opt.value();

  const auto & first_attention_area_opt = roundabout_lanelets.first_attention_area();
  const auto & conflicting_area = roundabout_lanelets.conflicting_area();
  const auto lanelets_on_path =
    planning_utils::getLaneletsOnPath(*path, lanelet_map_ptr, current_pose);
  // see the doc for struct PathLanelets
  const auto closest_idx_ip = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    path_ip.points, current_pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);
  const auto path_lanelets_opt = generatePathLanelets(
    lanelets_on_path, interpolated_path_info, first_conflicting_area, conflicting_area,
    first_attention_area_opt, roundabout_lanelets.attention_area(), closest_idx_ip);
  if (!path_lanelets_opt.has_value()) {
    return make_err<RoundaboutModule::BasicData, InternalError>("failed to generate PathLanelets");
  }
  const auto & path_lanelets = path_lanelets_opt.value();

  return make_ok<RoundaboutModule::BasicData, InternalError>(
    interpolated_path_info, roundabout_stoplines, path_lanelets);
}

std::optional<RoundaboutStopLines> RoundaboutModule::generateRoundaboutStopLines(
  const lanelet::ConstLanelet & first_attention_lane,
  const InterpolatedPathInfo & interpolated_path_info,
  autoware_internal_planning_msgs::msg::PathWithLaneId * original_path) const
{
  const double stopline_margin = planner_param_.common.default_stopline_margin;
  const double max_accel = planner_param_.common.max_accel;
  const double max_jerk = planner_param_.common.max_jerk;
  const double delay_response_time = planner_param_.common.delay_response_time;

  const auto first_attention_area = first_attention_lane.polygon3d();
  const auto & path_ip = interpolated_path_info.path;
  const double ds = interpolated_path_info.ds;
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const int stopline_margin_idx_dist = std::ceil(stopline_margin / ds);

  // find the index of the first point whose vehicle footprint on it intersects with attention_area
  const auto local_footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  const std::optional<size_t> first_footprint_inside_1st_attention_ip_opt =
    util::getFirstPointInsidePolygonByFootprint(
      first_attention_area, interpolated_path_info, local_footprint, baselink2front);
  if (!first_footprint_inside_1st_attention_ip_opt) {
    return std::nullopt;
  }
  const auto first_footprint_inside_1st_attention_ip =
    first_footprint_inside_1st_attention_ip_opt.value();

  // (1) default stop line position on interpolated path
  bool default_stopline_valid = true;
  int stop_idx_ip_int = first_footprint_inside_1st_attention_ip - stopline_margin_idx_dist;
  if (stop_idx_ip_int < 0) {
    default_stopline_valid = false;
  }
  const auto default_stopline_ip = stop_idx_ip_int >= 0 ? static_cast<size_t>(stop_idx_ip_int) : 0;

  // (2) ego front stop line position on interpolated path
  const geometry_msgs::msg::Pose & current_pose = planner_data_->current_odometry->pose;
  const auto closest_idx_ip = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    path_ip.points, current_pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);

  const double velocity = planner_data_->current_velocity->twist.linear.x;
  const double acceleration = planner_data_->current_acceleration->accel.accel.linear.x;
  const double braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    velocity, acceleration, max_accel, max_jerk, delay_response_time);

  // collision_stopline
  const size_t collision_stopline_ip = closest_idx_ip + std::ceil(braking_dist / ds);

  // (4) first attention stopline position on interpolated path
  const auto first_attention_stopline_ip = first_footprint_inside_1st_attention_ip;

  // (5) 1st pass judge line position on interpolated path
  int first_pass_judge_ip_int =
    static_cast<int>(first_footprint_inside_1st_attention_ip) - std::ceil(braking_dist / ds);
  const auto first_pass_judge_line_ip = static_cast<size_t>(
    std::clamp<int>(first_pass_judge_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));

  struct RoundaboutStopLinesTemp
  {
    size_t closest_idx{0};
    size_t default_stopline{0};
    size_t collision_stopline{0};
    size_t first_attention_stopline{0};
    size_t first_pass_judge_line{0};
  };

  RoundaboutStopLinesTemp roundabout_stoplines_temp;
  std::list<std::pair<const size_t *, size_t *>> stoplines = {
    {&closest_idx_ip, &roundabout_stoplines_temp.closest_idx},
    {&default_stopline_ip, &roundabout_stoplines_temp.default_stopline},
    {&collision_stopline_ip, &roundabout_stoplines_temp.collision_stopline},
    {&first_attention_stopline_ip, &roundabout_stoplines_temp.first_attention_stopline},
    {&first_pass_judge_line_ip, &roundabout_stoplines_temp.first_pass_judge_line},
  };
  stoplines.sort(
    [](const auto & it1, const auto & it2) { return *(std::get<0>(it1)) < *(std::get<0>(it2)); });
  for (const auto & [stop_idx_ip, stop_idx] : stoplines) {
    const auto & insert_point = path_ip.points.at(*stop_idx_ip).point.pose;
    const auto insert_idx = util::insertPointIndex(
      insert_point, original_path, planner_data_->ego_nearest_dist_threshold,
      planner_data_->ego_nearest_yaw_threshold);
    if (!insert_idx) {
      return std::nullopt;
    }
    *stop_idx = insert_idx.value();
  }

  RoundaboutStopLines roundabout_stoplines;
  roundabout_stoplines.closest_idx = roundabout_stoplines_temp.closest_idx;
  roundabout_stoplines.collision_stopline = roundabout_stoplines_temp.collision_stopline;
  if (default_stopline_valid) {
    roundabout_stoplines.default_stopline = roundabout_stoplines_temp.default_stopline;
  }
  roundabout_stoplines.first_attention_stopline =
    roundabout_stoplines_temp.first_attention_stopline;
  roundabout_stoplines.first_pass_judge_line = roundabout_stoplines_temp.first_pass_judge_line;
  return roundabout_stoplines;
}

RoundaboutLanelets RoundaboutModule::generateObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const lanelet::ConstLanelet assigned_lanelet) const
{
  const double detection_area_length = planner_param_.common.attention_area_length;

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
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
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
  lanelet::ConstLanelets conflicting_ex_ego_yield_lanelets;
  lanelet::ConstLanelets conflicting_ex_ego_lanelets;
  // conflicting lanes is necessary to get stopline for stuck vehicle
  for (auto && conflicting_lanelet : conflicting_lanelets) {
    if (!lanelet::utils::contains(ego_lanelets, conflicting_lanelet))
      conflicting_ex_ego_lanelets.push_back(conflicting_lanelet);
  }

  // exclude yield lanelets and ego lanelets from conflicting_lanelet
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    if (
      lanelet::utils::contains(yield_lanelets, conflicting_lanelet) ||
      lanelet::utils::contains(ego_lanelets, conflicting_lanelet)) {
      continue;
    }
    conflicting_ex_ego_yield_lanelets.push_back(conflicting_lanelet);
  }

  // get possible lanelet path that reaches conflicting_lane longer than given length
  lanelet::ConstLanelets detection_and_preceding_lanelets;
  {
    std::set<lanelet::Id> detection_ids;
    for (const auto & ll : conflicting_ex_ego_yield_lanelets) {
      // Preceding lanes does not include detection_lane so add them at the end
      const auto & inserted = detection_ids.insert(ll.id());
      if (inserted.second) detection_and_preceding_lanelets.push_back(ll);
      // get preceding lanelets without ego_lanelets
      // to prevent the detection area from including the ego lanes and its' preceding lanes.
      const auto lanelet_sequences = lanelet::utils::query::getPrecedingLaneletSequences(
        routing_graph_ptr, ll, detection_area_length, ego_lanelets);
      for (const auto & ls : lanelet_sequences) {
        for (const auto & l : ls) {
          const auto & inner_inserted = detection_ids.insert(l.id());
          if (lanelet::utils::contains(associative_ids_, l.id())) {
            break;
          }
          if (inner_inserted.second) detection_and_preceding_lanelets.push_back(l);
        }
      }
    }
  }

  auto [attention_lanelets, original_attention_lanelet_sequences] =
    util::mergeLaneletsByTopologicalSort(
      detection_and_preceding_lanelets, conflicting_ex_ego_yield_lanelets, routing_graph_ptr);

  RoundaboutLanelets result;
  result.attention_ = std::move(attention_lanelets);
  result.attention_non_preceding_ = std::move(conflicting_ex_ego_yield_lanelets);
  result.conflicting_ = std::move(conflicting_ex_ego_lanelets);
  result.adjacent_ = planning_utils::getConstLaneletsFromIds(lanelet_map_ptr, associative_ids_);

  // NOTE: to properly update(), each element in conflicting_/conflicting_area_,
  // attention_non_preceding_/attention_non_preceding_area_ need to be matched
  result.attention_area_ = util::getPolygon3dFromLanelets(result.attention_);
  result.attention_non_preceding_area_ =
    util::getPolygon3dFromLanelets(result.attention_non_preceding_);
  result.conflicting_area_ = util::getPolygon3dFromLanelets(result.conflicting_);
  result.adjacent_area_ = util::getPolygon3dFromLanelets(result.adjacent_);
  return result;
}

std::optional<PathLanelets> RoundaboutModule::generatePathLanelets(
  const lanelet::ConstLanelets & lanelets_on_path,
  const InterpolatedPathInfo & interpolated_path_info,
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx) const
{
  const double width = planner_data_->vehicle_info_.vehicle_width_m;
  static constexpr double path_lanelet_interval = 1.5;

  const auto & assigned_lane_interval_opt = interpolated_path_info.lane_id_interval;
  if (!assigned_lane_interval_opt) {
    return std::nullopt;
  }
  const auto assigned_lane_interval = assigned_lane_interval_opt.value();
  const auto & path = interpolated_path_info.path;

  PathLanelets path_lanelets;
  // prev
  path_lanelets.prev = ::getPrevLanelets(lanelets_on_path, associative_ids_);
  path_lanelets.all = path_lanelets.prev;

  // entry2ego if exist
  const auto [assigned_lane_start, assigned_lane_end] = assigned_lane_interval;
  if (closest_idx > assigned_lane_start) {
    path_lanelets.all.push_back(
      ::generatePathLanelet(path, assigned_lane_start, closest_idx, width, path_lanelet_interval));
  }

  // ego_or_entry2exit
  const auto ego_or_entry_start = std::max(closest_idx, assigned_lane_start);
  path_lanelets.ego_or_entry2exit =
    generatePathLanelet(path, ego_or_entry_start, assigned_lane_end, width, path_lanelet_interval);
  path_lanelets.all.push_back(path_lanelets.ego_or_entry2exit);

  // next
  if (assigned_lane_end < path.points.size() - 1) {
    const int next_id = path.points.at(assigned_lane_end).lane_ids.at(0);
    const auto next_lane_interval_opt = util::findLaneIdsInterval(path, {next_id});
    if (next_lane_interval_opt) {
      const auto [next_start, next_end] = next_lane_interval_opt.value();
      path_lanelets.next =
        generatePathLanelet(path, next_start, next_end, width, path_lanelet_interval);
      path_lanelets.all.push_back(path_lanelets.next.value());
    }
  }

  const auto first_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? util::getFirstPointInsidePolygon(path, assigned_lane_interval, first_attention_area.value())
      : util::getFirstPointInsidePolygon(path, assigned_lane_interval, first_conflicting_area);
  const auto last_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? ::getFirstPointInsidePolygons(path, assigned_lane_interval, attention_areas, false)
      : ::getFirstPointInsidePolygons(path, assigned_lane_interval, conflicting_areas, false);
  if (first_inside_conflicting_idx_opt && last_inside_conflicting_idx_opt) {
    const auto first_inside_conflicting_idx = first_inside_conflicting_idx_opt.value();
    const auto last_inside_conflicting_idx = last_inside_conflicting_idx_opt.value().first;
    lanelet::ConstLanelet conflicting_interval = generatePathLanelet(
      path, first_inside_conflicting_idx, last_inside_conflicting_idx, width,
      path_lanelet_interval);
    path_lanelets.conflicting_interval_and_remaining.push_back(std::move(conflicting_interval));
    if (last_inside_conflicting_idx < assigned_lane_end) {
      lanelet::ConstLanelet remaining_interval = generatePathLanelet(
        path, last_inside_conflicting_idx, assigned_lane_end, width, path_lanelet_interval);
      path_lanelets.conflicting_interval_and_remaining.push_back(std::move(remaining_interval));
    }
  }
  return path_lanelets;
}

}  // namespace autoware::behavior_velocity_planner
