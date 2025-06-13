// Copyright 2020 Tier IV, Inc.
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

#include "autoware/boundary_departure_checker/boundary_departure_checker.hpp"

#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_utils::Point2d;
using geometry_msgs::msg::Point;

double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity;
}

bool isInAnyLane(const lanelet::ConstLanelets & candidate_lanelets, const Point2d & point)
{
  return std::any_of(
    candidate_lanelets.begin(), candidate_lanelets.end(), [&](const lanelet::ConstLanelet & ll) {
      return boost::geometry::within(point, ll.polygon2d().basicPolygon());
    });
}

}  // namespace

namespace autoware::boundary_departure_checker
{
BoundaryDepartureChecker::BoundaryDepartureChecker(
  lanelet::LaneletMapPtr lanelet_map_ptr, const VehicleInfo & vehicle_info, const Param & param,
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: lanelet_map_ptr_(lanelet_map_ptr),
  param_ptr_(std::make_unique<Param>(param)),
  vehicle_info_ptr_(std::make_shared<VehicleInfo>(vehicle_info)),
  time_keeper_(std::move(time_keeper))
{
  auto try_uncrossable_boundaries_rtree = build_uncrossable_boundaries_tree(lanelet_map_ptr);

  if (!try_uncrossable_boundaries_rtree) {
    throw std::runtime_error(try_uncrossable_boundaries_rtree.error());
  }

  uncrossable_boundaries_rtree_ptr_ =
    std::make_unique<UncrossableBoundRTree>(*try_uncrossable_boundaries_rtree);
}

Output BoundaryDepartureChecker::update(const Input & input)
{
  Output output{};

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  {
    constexpr double min_velocity = 0.01;
    const auto & raw_abs_velocity = std::abs(input.current_odom->twist.twist.linear.x);
    const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;

    const auto braking_distance = std::max(
      param_.min_braking_distance,
      calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time));

    output.resampled_trajectory = utils::cutTrajectory(
      utils::resampleTrajectory(*input.predicted_trajectory, param_.resample_interval),
      braking_distance);
    output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);
  }
  output.vehicle_footprints = utils::createVehicleFootprints(
    input.current_odom->pose, output.resampled_trajectory, *vehicle_info_ptr_,
    param_.footprint_margin_scale);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = utils::createVehiclePassingAreas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  const auto candidate_road_lanelets =
    utils::getCandidateLanelets(input.route_lanelets, output.vehicle_footprints);
  const auto candidate_shoulder_lanelets =
    utils::getCandidateLanelets(input.shoulder_lanelets, output.vehicle_footprints);
  output.candidate_lanelets = candidate_road_lanelets;
  output.candidate_lanelets.insert(
    output.candidate_lanelets.end(), candidate_shoulder_lanelets.begin(),
    candidate_shoulder_lanelets.end());

  output.processing_time_map["getCandidateLanelets"] = stop_watch.toc(true);

  output.will_leave_lane = willLeaveLane(output.candidate_lanelets, output.vehicle_footprints);
  output.processing_time_map["willLeaveLane"] = stop_watch.toc(true);

  output.is_out_of_lane = isOutOfLane(output.candidate_lanelets, output.vehicle_footprints.front());
  output.processing_time_map["isOutOfLane"] = stop_watch.toc(true);

  const double max_search_length_for_boundaries =
    utils::calcMaxSearchLengthForBoundaries(*input.predicted_trajectory, *vehicle_info_ptr_);
  const auto uncrossable_boundaries = extractUncrossableBoundaries(
    *input.lanelet_map, input.predicted_trajectory->points.front().pose.position,
    max_search_length_for_boundaries, input.boundary_types_to_detect);
  output.will_cross_boundary = willCrossBoundary(output.vehicle_footprints, uncrossable_boundaries);
  output.processing_time_map["willCrossBoundary"] = stop_watch.toc(true);

  return output;
}

tl::expected<AbnormalitiesData, std::string> BoundaryDepartureChecker::get_abnormalities_data(
  const TrajectoryPoints & predicted_traj,
  const trajectory::Trajectory<TrajectoryPoint> & aw_raw_traj,
  const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov,
  const SteeringReport & current_steering)
{
  if (predicted_traj.empty()) {
    return tl::make_unexpected("Ego predicted trajectory is empty");
  }

  const auto underlying_bases = aw_raw_traj.get_underlying_bases();
  if (underlying_bases.size() < 4) {
    return tl::make_unexpected("trajectory too short");
  }

  const auto uncertainty_fp_margin =
    utils::calc_margin_from_covariance(curr_pose_with_cov, param_ptr_->footprint_extra_margin);

  AbnormalitiesData abnormalities_data;
  for (const auto abnormality_type : param_ptr_->abnormality_types_to_compensate) {
    auto & fps = abnormalities_data.footprints[abnormality_type];
    fps = utils::create_ego_footprints(
      abnormality_type, uncertainty_fp_margin, predicted_traj, current_steering, *vehicle_info_ptr_,
      *param_ptr_);

    abnormalities_data.footprints_sides[abnormality_type] = utils::get_sides_from_footprints(fps);
  }

  const auto & normal_footprints = abnormalities_data.footprints_sides[AbnormalityType::NORMAL];

  const auto boundary_segments_opt = get_boundary_segments_from_side(normal_footprints);

  if (!boundary_segments_opt) {
    return tl::make_unexpected(boundary_segments_opt.error());
  }
  abnormalities_data.boundary_segments = *boundary_segments_opt;

  for (const auto abnormality_type : param_ptr_->abnormality_types_to_compensate) {
    auto & proj_to_bound = abnormalities_data.projections_to_bound[abnormality_type];
    proj_to_bound = utils::get_closest_boundary_segments_from_side(
      abnormalities_data.boundary_segments, abnormalities_data.footprints_sides[abnormality_type]);
  }
  return abnormalities_data;
}

tl::expected<BoundarySideWithIdx, std::string>
BoundaryDepartureChecker::get_boundary_segments_from_side(
  const EgoSides & ego_sides_from_footprints)
{
  if (!lanelet_map_ptr_) {
    return tl::make_unexpected(std::string(__func__) + ": invalid lanelet_map_ptr");
  }

  if (!uncrossable_boundaries_rtree_ptr_) {
    return tl::make_unexpected(std::string(__func__) + ": invalid uncrossable boundaries rtree");
  }

  if (!param_ptr_) {
    return tl::make_unexpected(std::string(__func__) + ": invalid param_ptr");
  }

  const auto max_lat_query_num = param_ptr_->th_max_lateral_query_num;

  const auto & rtree = *uncrossable_boundaries_rtree_ptr_;
  const auto & linestring_layer = lanelet_map_ptr_->lineStringLayer;

  const auto closest_segment =
    [&](const auto & ego_seg, const auto & compare_seg, auto & output_side) {
      std::vector<SegmentWithIdx> nearest_raw;

      const lanelet::BasicPoint2d ego_start{ego_seg.first.x(), ego_seg.first.y()};
      rtree.query(bgi::nearest(ego_start, max_lat_query_num), std::back_inserter(nearest_raw));

      for (const auto & nearest : nearest_raw) {
        const auto ll_id = std::get<1>(nearest);
        const auto basic_ls = linestring_layer.get(ll_id).basicLineString();

        const auto idx_curr = std::get<2>(nearest);
        const auto idx_next = std::get<3>(nearest);
        const auto seg = utils::to_segment2d(basic_ls.at(idx_curr), basic_ls.at(idx_next));

        const auto dist_from_curr_side = bg::comparable_distance(ego_seg, seg);
        const auto dist_from_compare_side = bg::comparable_distance(compare_seg, seg);

        if (dist_from_compare_side < dist_from_curr_side) {
          continue;
        }

        const auto found = [&](const SegmentWithIdx & output_seg) {
          const auto & [out_seg, out_ll_id, out_ls_idx_curr, out_ls_idx_next] = output_seg;
          return out_ll_id == ll_id && out_ls_idx_curr == idx_curr;
        };

        if (!std::any_of(output_side.begin(), output_side.end(), found)) {
          output_side.push_back(std::make_tuple(seg, ll_id, idx_curr, idx_next));
        }
      }
    };

  BoundarySideWithIdx boundary_sides_with_idx;
  for (const auto & fp : ego_sides_from_footprints) {
    closest_segment(fp.left, fp.right, boundary_sides_with_idx.left);   // Left side
    closest_segment(fp.right, fp.left, boundary_sides_with_idx.right);  // Right side
  }

  return boundary_sides_with_idx;
}

tl::expected<std::vector<ClosestProjectionToBound>, std::string>
BoundaryDepartureChecker::get_closest_projections_to_boundaries_side(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const Abnormalities<ProjectionsToBound> & projections_to_bound, const SideKey side_key)
{
  const auto & abnormality_to_check = param_ptr_->abnormality_types_to_compensate;

  if (abnormality_to_check.empty()) {
    return tl::make_unexpected(std::string(__func__) + ": Nothing to check.");
  }

  const auto is_empty = std::any_of(
    abnormality_to_check.begin(), abnormality_to_check.end(),
    [&projections_to_bound, &side_key](const auto abnormality_type) {
      return projections_to_bound[abnormality_type][side_key].empty();
    });

  if (is_empty) {
    return tl::make_unexpected(std::string(__func__) + ": projections to bound is empty.");
  }

  const auto & fr_proj_to_bound = projections_to_bound[abnormality_to_check.front()][side_key];

  const auto check_size = [&](const auto abnormality_type) {
    return fr_proj_to_bound.size() != projections_to_bound[abnormality_type][side_key].size();
  };

  const auto has_size_diff =
    std::any_of(std::next(abnormality_to_check.begin()), abnormality_to_check.end(), check_size);

  if (has_size_diff) {
    return tl::make_unexpected(
      std::string(__func__) + ": Some abnormality type has incorrect size.");
  }

  std::vector<ClosestProjectionToBound> min_to_bound;

  const auto is_on_bound = [this](const double lat_dist, const SideKey side_key) {
    return lat_dist < param_ptr_->th_trigger.th_dist_to_boundary_m[side_key].min;
  };

  const auto is_close_to_bound = [&](const double lat_dist, const SideKey side_key) {
    return !is_on_bound(lat_dist, side_key) &&
           lat_dist <= param_ptr_->th_trigger.th_dist_to_boundary_m[side_key].max;
  };

  const auto fp_size = projections_to_bound[abnormality_to_check.front()][side_key].size();
  min_to_bound.reserve(fp_size);
  for (size_t idx = 0; idx < fp_size; ++idx) {
    std::unique_ptr<ClosestProjectionToBound> min_pt;
    for (const auto abnormality_type : abnormality_to_check) {
      const auto pt = projections_to_bound[abnormality_type][side_key][idx];
      if (pt.ego_sides_idx != idx) {
        continue;
      }

      if (abnormality_type == AbnormalityType::NORMAL && is_on_bound(pt.lat_dist, side_key)) {
        min_pt = std::make_unique<ClosestProjectionToBound>(pt);
        min_pt->departure_type = DepartureType::CRITICAL_DEPARTURE;
        min_pt->lon_dist_on_ref_traj = utils::calc_dist_on_traj(aw_ref_traj, min_pt->pt_on_ego);
        break;
      }

      if (!min_pt && is_close_to_bound(pt.lat_dist, side_key)) {
        min_pt = std::make_unique<ClosestProjectionToBound>(pt);
        continue;
      }
      if (min_pt && pt.lat_dist < min_pt->lat_dist) {
        min_pt = std::make_unique<ClosestProjectionToBound>(pt);
        if (abnormality_type == abnormality_to_check.back()) {
          min_pt->departure_type = DepartureType::NEAR_BOUNDARY;
          min_pt->lon_dist_on_ref_traj = utils::calc_dist_on_traj(aw_ref_traj, min_pt->pt_on_ego);
          break;
        }
      }
    }
    if (!min_pt) {
      continue;
    }

    min_to_bound.push_back(*min_pt);
    if (min_to_bound.back().departure_type == DepartureType::CRITICAL_DEPARTURE) {
      for (auto itr = std::next(min_to_bound.rbegin()); itr != min_to_bound.rend(); ++itr) {
        const auto & th_trigger = param_ptr_->th_trigger;
        const auto v_init = th_trigger.max_slow_down_vel_mps;
        constexpr auto v_end = 0.0;
        const auto acc = th_trigger.th_acc_mps2.min;
        const auto jerk = th_trigger.th_jerk_mps3.min;
        const auto braking_delay = th_trigger.brake_delay_s;
        const auto braking_dist =
          utils::compute_braking_distance(v_init, v_end, acc, jerk, braking_delay);
        if (braking_dist + itr->lon_dist_on_ref_traj > min_to_bound.back().lon_dist_on_ref_traj) {
          itr->departure_type = DepartureType::APPROACHING_DEPARTURE;
        }
      }

      break;
    }
  }
  return min_to_bound;
}

tl::expected<ClosestProjectionsToBound, std::string>
BoundaryDepartureChecker::get_closest_projections_to_boundaries(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
  const Abnormalities<ProjectionsToBound> & projections_to_bound)
{
  ClosestProjectionsToBound min_to_bound;

  for (const auto side_key : g_side_keys) {
    const auto min_to_bound_opt =
      get_closest_projections_to_boundaries_side(aw_ref_traj, projections_to_bound, side_key);

    if (!min_to_bound_opt) {
      return tl::make_unexpected(min_to_bound_opt.error());
    }
    min_to_bound[side_key] = *min_to_bound_opt;
  }

  return min_to_bound;
}

Side<DeparturePoints> BoundaryDepartureChecker::get_departure_points(
  const ClosestProjectionsToBound & projections_to_bound)
{
  const auto th_dist_hysteresis_m = param_ptr_->th_dist_hysteresis_m;

  Side<DeparturePoints> departure_points;
  for (const auto side_key : g_side_keys) {
    departure_points[side_key] =
      utils::get_departure_points(projections_to_bound[side_key], th_dist_hysteresis_m);
  }
  return departure_points;
}

tl::expected<UncrossableBoundRTree, std::string>
BoundaryDepartureChecker::build_uncrossable_boundaries_tree(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!lanelet_map_ptr) {
    return tl::unexpected(std::string("lanelet_map_ptr is null"));
  }

  if (!param_ptr_) {
    return tl::unexpected(std::string("param_ptr is null"));
  }

  return utils::build_uncrossable_boundaries_rtree(
    *lanelet_map_ptr, param_ptr_->boundary_types_to_detect);
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  lanelet::ConstLanelets candidate_lanelets =
    utils::getCandidateLanelets(lanelets, vehicle_footprints);
  return willLeaveLane(candidate_lanelets, vehicle_footprints);
}

bool BoundaryDepartureChecker::willLeaveLane(
  const lanelet::ConstLanelets & candidate_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & vehicle_footprint : vehicle_footprints) {
    if (isOutOfLane(candidate_lanelets, vehicle_footprint)) {
      return true;
    }
  }

  return false;
}

std::vector<std::pair<double, lanelet::Lanelet>> BoundaryDepartureChecker::getLaneletsFromPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Get Footprint Hull basic polygon
  std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  LinearRing2d footprint_hull = utils::createHullFromFootprints(vehicle_footprints);

  lanelet::BasicPolygon2d footprint_hull_basic_polygon = utils::toBasicPolygon2D(footprint_hull);

  // Find all lanelets that intersect the footprint hull
  return lanelet::geometry::findWithin2d(
    lanelet_map_ptr->laneletLayer, footprint_hull_basic_polygon, 0.0);
}

std::optional<autoware_utils::Polygon2d> BoundaryDepartureChecker::getFusedLaneletPolygonForPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto lanelets_distance_pair = getLaneletsFromPath(lanelet_map_ptr, path);
  if (lanelets_distance_pair.empty()) return std::nullopt;

  // Fuse lanelets into a single BasicPolygon2d
  autoware_utils::MultiPolygon2d lanelet_unions;
  autoware_utils::MultiPolygon2d result;

  for (size_t i = 0; i < lanelets_distance_pair.size(); ++i) {
    const auto & route_lanelet = lanelets_distance_pair.at(i).second;
    const auto & p = route_lanelet.polygon2d().basicPolygon();
    autoware_utils::Polygon2d poly = toPolygon2D(p);
    boost::geometry::union_(lanelet_unions, poly, result);
    lanelet_unions = result;
    result.clear();
  }

  if (lanelet_unions.empty()) return std::nullopt;
  return lanelet_unions.front();
}

bool BoundaryDepartureChecker::updateFusedLaneletPolygonForPath(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto lanelets_distance_pair = getLaneletsFromPath(lanelet_map_ptr, path);
  if (lanelets_distance_pair.empty()) return false;

  autoware_utils::MultiPolygon2d lanelet_unions;
  autoware_utils::MultiPolygon2d result;

  if (fused_lanelets_polygon) lanelet_unions.push_back(fused_lanelets_polygon.value());

  for (size_t i = 0; i < lanelets_distance_pair.size(); ++i) {
    const auto & route_lanelet = lanelets_distance_pair.at(i).second;
    bool id_exist = std::any_of(
      fused_lanelets_id.begin(), fused_lanelets_id.end(),
      [&](const auto & id) { return id == route_lanelet.id(); });

    if (id_exist) continue;

    const auto & p = route_lanelet.polygon2d().basicPolygon();
    autoware_utils::Polygon2d poly = toPolygon2D(p);
    boost::geometry::union_(lanelet_unions, poly, result);
    lanelet_unions = result;
    result.clear();
    fused_lanelets_id.push_back(route_lanelet.id());
  }

  if (lanelet_unions.empty()) {
    fused_lanelets_polygon = std::nullopt;
    return false;
  }

  fused_lanelets_polygon = lanelet_unions.front();
  return true;
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // check if the footprint is not fully contained within the fused lanelets polygon
  const std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);
  const auto fused_lanelets_polygon = getFusedLaneletPolygonForPath(lanelet_map_ptr, path);
  if (!fused_lanelets_polygon) return true;
  return !std::all_of(
    vehicle_footprints.begin(), vehicle_footprints.end(),
    [&fused_lanelets_polygon](const auto & footprint) {
      return boost::geometry::within(footprint, fused_lanelets_polygon.value());
    });
}

bool BoundaryDepartureChecker::checkPathWillLeaveLane(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const std::vector<LinearRing2d> vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);

  auto is_all_footprints_within = [&](const auto & polygon) {
    return std::all_of(
      vehicle_footprints.begin(), vehicle_footprints.end(),
      [&polygon](const auto & footprint) { return boost::geometry::within(footprint, polygon); });
  };

  // If lanelets polygon exists and all footprints are within it, the path doesn't leave the lane
  if (fused_lanelets_polygon && is_all_footprints_within(fused_lanelets_polygon.value())) {
    return false;
  }

  // Update the lanelet polygon for the current path
  if (!updateFusedLaneletPolygonForPath(
        lanelet_map_ptr, path, fused_lanelets_id, fused_lanelets_polygon)) {
    // If update fails, assume the path leaves the lane
    return true;
  }

  // Check if any footprint is outside the updated lanelets polygon
  return !is_all_footprints_within(fused_lanelets_polygon.value());
}

PathWithLaneId BoundaryDepartureChecker::cropPointsOutsideOfLanes(
  const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path, const size_t end_index,
  std::vector<lanelet::Id> & fused_lanelets_id,
  std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  PathWithLaneId temp_path;

  // Update the lanelet polygon for the current path
  if (
    path.points.empty() || !updateFusedLaneletPolygonForPath(
                             lanelet_map_ptr, path, fused_lanelets_id, fused_lanelets_polygon)) {
    return temp_path;
  }

  const auto vehicle_footprints =
    utils::createVehicleFootprints(path, *vehicle_info_ptr_, param_.footprint_extra_margin);

  {
    autoware_utils::ScopedTimeTrack st2(
      "check if footprint is within fused_lanelets_polygon", *time_keeper_);

    size_t idx = 0;
    std::for_each(
      vehicle_footprints.begin(), vehicle_footprints.end(), [&](const auto & footprint) {
        if (idx > end_index || boost::geometry::within(footprint, fused_lanelets_polygon.value())) {
          temp_path.points.push_back(path.points.at(idx));
        }
        ++idx;
      });
  }
  PathWithLaneId cropped_path = path;
  cropped_path.points = temp_path.points;
  return cropped_path;
}

bool BoundaryDepartureChecker::isOutOfLane(
  const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (!isInAnyLane(candidate_lanelets, point)) {
      return true;
    }
  }

  return false;
}

SegmentRtree BoundaryDepartureChecker::extractUncrossableBoundaries(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & ego_point,
  const double max_search_length, const std::vector<std::string> & boundary_types_to_detect)
{
  const auto has_types =
    [](const lanelet::ConstLineString3d & ls, const std::vector<std::string> & types) {
      constexpr auto no_type = "";
      const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
      return (type != no_type && std::find(types.begin(), types.end(), type) != types.end());
    };

  SegmentRtree uncrossable_segments_in_range;
  LineString2d line;
  const auto ego_p = Point2d{ego_point.x, ego_point.y};
  for (const auto & ls : lanelet_map.lineStringLayer) {
    if (has_types(ls, boundary_types_to_detect)) {
      line.clear();
      for (const auto & p : ls) line.push_back(Point2d{p.x(), p.y()});
      for (auto segment_idx = 0LU; segment_idx + 1 < line.size(); ++segment_idx) {
        const Segment2d segment = {line[segment_idx], line[segment_idx + 1]};
        if (boost::geometry::distance(segment, ego_p) < max_search_length) {
          uncrossable_segments_in_range.insert(segment);
        }
      }
    }
  }
  return uncrossable_segments_in_range;
}

bool BoundaryDepartureChecker::willCrossBoundary(
  const std::vector<LinearRing2d> & vehicle_footprints,
  const SegmentRtree & uncrossable_segments) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  for (const auto & footprint : vehicle_footprints) {
    std::vector<Segment2d> intersection_result;
    uncrossable_segments.query(
      boost::geometry::index::intersects(footprint), std::back_inserter(intersection_result));
    if (!intersection_result.empty()) {
      return true;
    }
  }
  return false;
}

autoware_utils::Polygon2d BoundaryDepartureChecker::toPolygon2D(
  const lanelet::BasicPolygon2d & poly) const
{
  autoware_utils::Polygon2d polygon;
  auto & outer = polygon.outer();

  for (const auto & p : poly) {
    autoware_utils::Point2d p2d(p.x(), p.y());
    outer.push_back(p2d);
  }
  boost::geometry::correct(polygon);
  return polygon;
}
}  // namespace autoware::boundary_departure_checker
