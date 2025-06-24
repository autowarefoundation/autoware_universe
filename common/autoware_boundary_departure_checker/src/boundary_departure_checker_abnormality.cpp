// Copyright 2025 TIER IV, Inc.
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
#include "autoware/boundary_departure_checker/conversion.hpp"
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

#include <fmt/format.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

tl::expected<AbnormalitiesData, std::string> BoundaryDepartureChecker::get_abnormalities_data(
  const TrajectoryPoints & predicted_traj,
  const trajectory::Trajectory<TrajectoryPoint> & aw_raw_traj,
  const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov,
  const SteeringReport & current_steering)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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

  const auto closest_segment = [&](
                                 const auto & ego_seg, const auto & compare_seg,
                                 auto & output_side) {
    std::vector<SegmentWithIdx> nearest_raw;

    const lanelet::BasicPoint2d ego_start{ego_seg.first.x(), ego_seg.first.y()};
    rtree.query(bgi::nearest(ego_start, max_lat_query_num), std::back_inserter(nearest_raw));

    for (const auto & nearest : nearest_raw) {
      const auto & id = std::get<1>(nearest);
      const auto basic_ls = linestring_layer.get(id.linestring_id).basicLineString();

      const auto seg =
        utils::to_segment_2d(basic_ls.at(id.segment_start_idx), basic_ls.at(id.segment_end_idx));

      const auto dist_from_curr_side = bg::comparable_distance(ego_seg, seg);
      const auto dist_from_compare_side = bg::comparable_distance(compare_seg, seg);

      if (dist_from_compare_side < dist_from_curr_side) {
        continue;
      }

      const auto found = [&](const SegmentWithIdx & output_seg) { return output_seg.second == id; };

      if (!std::any_of(output_side.begin(), output_side.end(), found)) {
        output_side.emplace_back(seg, id);
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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
    return lat_dist <= param_ptr_->th_trigger.th_dist_to_boundary_m[side_key].max;
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
        min_pt->abnormality_type = abnormality_type;
        break;
      }

      if (!is_close_to_bound(pt.lat_dist, side_key)) {
        continue;
      }

      if (!min_pt || pt.lat_dist < min_pt->lat_dist) {
        min_pt = std::make_unique<ClosestProjectionToBound>(pt);
        min_pt->departure_type = DepartureType::NEAR_BOUNDARY;
        min_pt->abnormality_type = abnormality_type;
      }
    }
    if (!min_pt) {
      continue;
    }

    if (
      !min_to_bound.empty() && min_pt->departure_type != DepartureType::CRITICAL_DEPARTURE &&
      std::abs(min_to_bound.back().lon_dist_on_ref_traj - min_pt->lon_dist_on_ref_traj) < 0.5) {
      continue;
    }
    min_pt->lon_dist_on_ref_traj =
      trajectory::closest(aw_ref_traj, utils::to_geom_pt(min_pt->pt_on_ego));
    min_to_bound.push_back(*min_pt);
    if (min_to_bound.back().departure_type == DepartureType::CRITICAL_DEPARTURE) {
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
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  ClosestProjectionsToBound min_to_bound;

  for (const auto side_key : g_side_keys) {
    const auto min_to_bound_opt =
      get_closest_projections_to_boundaries_side(aw_ref_traj, projections_to_bound, side_key);

    if (!min_to_bound_opt) {
      return tl::make_unexpected(min_to_bound_opt.error());
    }
    min_to_bound[side_key] = *min_to_bound_opt;

    if (min_to_bound[side_key].size() <= 1) {
      continue;
    }

    for (auto itr = std::next(min_to_bound[side_key].rbegin());
         itr != min_to_bound[side_key].rend(); ++itr) {
      const auto & th_trigger = param_ptr_->th_trigger;
      const auto v_init = th_trigger.th_vel_mps.min;
      constexpr auto v_end = 0.0;
      const auto acc = th_trigger.th_acc_mps2.min;
      const auto jerk = th_trigger.th_jerk_mps3.max;
      const auto braking_delay = th_trigger.brake_delay_s;
      const auto braking_dist =
        utils::compute_braking_distance(v_init, v_end, acc, jerk, braking_delay);
      if (
        min_to_bound[side_key].back().lon_dist_on_ref_traj - itr->lon_dist_on_ref_traj <
        braking_dist) {
        itr->departure_type = DepartureType::APPROACHING_DEPARTURE;
      }
    }
  }

  return min_to_bound;
}

Side<DeparturePoints> BoundaryDepartureChecker::get_departure_points(
  const ClosestProjectionsToBound & projections_to_bound, const double lon_offset_m)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto th_dist_hysteresis_m = param_ptr_->th_dist_hysteresis_m;

  Side<DeparturePoints> departure_points;
  for (const auto side_key : g_side_keys) {
    departure_points[side_key] = utils::get_departure_points(
      projections_to_bound[side_key], th_dist_hysteresis_m, lon_offset_m);
  }
  return departure_points;
}

tl::expected<UncrossableBoundRTree, std::string>
BoundaryDepartureChecker::build_uncrossable_boundaries_tree(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!lanelet_map_ptr) {
    return tl::make_unexpected("lanelet_map_ptr is null");
  }

  if (!param_ptr_) {
    return tl::make_unexpected("param_ptr is null");
  }

  return utils::build_uncrossable_boundaries_rtree(
    *lanelet_map_ptr, param_ptr_->boundary_types_to_detect);
}

SegmentRtree BoundaryDepartureChecker::extractUncrossableBoundaries(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & ego_point,
  const double max_search_length, const std::vector<std::string> & boundary_types_to_detect) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
}  // namespace autoware::boundary_departure_checker
