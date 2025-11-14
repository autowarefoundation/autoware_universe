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
#include <tf2/utils.hpp>
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>

#include <fmt/format.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
namespace bg = boost::geometry;

/**
 * @brief Retrieves a 3D line segment from the Lanelet2 map.
 *
 * @param lanelet_map_ptr A pointer to the Lanelet2 map from which to retrieve the data.
 * @param seg_id An identifier struct containing the ID of the parent LineString and the start/end
 * indices of the specific segment within it.
 * @return The corresponding Segment3d defined by the start and end points.
 */
autoware_utils_geometry::Segment3d get_segment_3d_from_id(
  const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const autoware::boundary_departure_checker::IdxForRTreeSegment & seg_id)
{
  const auto & linestring_layer = lanelet_map_ptr->lineStringLayer;
  const auto basic_ls = linestring_layer.get(seg_id.linestring_id).basicLineString();

  auto p_start = autoware_utils_geometry::Point3d{
    basic_ls.at(seg_id.segment_start_idx).x(), basic_ls.at(seg_id.segment_start_idx).y(),
    basic_ls.at(seg_id.segment_start_idx).z()};

  auto p_end = autoware_utils_geometry::Point3d{
    basic_ls.at(seg_id.segment_end_idx).x(), basic_ls.at(seg_id.segment_end_idx).y(),
    basic_ls.at(seg_id.segment_end_idx).z()};

  return {p_start, p_end};
}

/**
 * @brief Checks if a given boundary segment is closer to the reference ego side than the opposite
 * side.
 *
 * @param boundary_segment The boundary segment to check.
 * @param ego_side_ref_segment The reference side of the ego vehicle (e.g., the left side).
 * @param ego_side_opposite_ref_segment The opposite side of the ego vehicle (e.g., the right side).
 * @return True if the boundary is closer to or equidistant to the reference side; false otherwise.
 */
bool is_closest_to_boundary_segment(
  const autoware_utils_geometry::Segment2d & boundary_segment,
  const autoware_utils_geometry::Segment2d & ego_side_ref_segment,
  const autoware_utils_geometry::Segment2d & ego_side_opposite_ref_segment)
{
  const auto dist_from_curr_side = bg::comparable_distance(ego_side_ref_segment, boundary_segment);
  const auto dist_from_compare_side =
    bg::comparable_distance(ego_side_opposite_ref_segment, boundary_segment);

  return dist_from_curr_side <= dist_from_compare_side;
}

/**
 * @brief Checks if a 3D boundary segment is vertically within the height range of the ego vehicle.
 *
 * This helps filter out irrelevant boundaries like overpasses (too high) or underpass (too low).
 *
 * @param boundary_segment The 3D boundary segment to check.
 * @param ego_z_position The reference vertical (Z-axis) position of the ego vehicle (e.g., at its
 * base).
 * @param ego_height The total height of the ego vehicle.
 * @return True if the segment's closest vertical point is within the vehicle's height; false
 * otherwise.
 */
bool is_segment_within_ego_height(
  const autoware_utils_geometry::Segment3d & boundary_segment, const double ego_z_position,
  const double ego_height)
{
  auto height_diff = std::min(
    std::abs(boundary_segment.first.z() - ego_z_position),
    std::abs(boundary_segment.second.z() - ego_z_position));
  return height_diff < ego_height;
}
}  // namespace

namespace autoware::boundary_departure_checker
{
BoundaryDepartureChecker::BoundaryDepartureChecker(
  lanelet::LaneletMapPtr lanelet_map_ptr, const VehicleInfo & vehicle_info, const Param & param,
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper)
: param_(param),
  lanelet_map_ptr_(lanelet_map_ptr),
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

  const auto trimmed_pred_traj =
    utils::trim_pred_path(predicted_traj, param_.th_cutoff_time_predicted_path_s);

  const auto uncertainty_fp_margin =
    utils::calc_margin_from_covariance(curr_pose_with_cov, param_.footprint_extra_margin);

  AbnormalitiesData abnormalities_data;
  for (const auto abnormality_type : param_.abnormality_types_to_compensate) {
    auto & fps = abnormalities_data.footprints[abnormality_type];
    fps = utils::create_ego_footprints(
      abnormality_type, uncertainty_fp_margin, trimmed_pred_traj, current_steering,
      *vehicle_info_ptr_, param_);

    abnormalities_data.footprints_sides[abnormality_type] = utils::get_sides_from_footprints(fps);
  }

  const auto & normal_footprints = abnormalities_data.footprints_sides[AbnormalityType::NORMAL];

  abnormalities_data.boundary_segments =
    get_boundary_segments(normal_footprints, trimmed_pred_traj);

  if (
    abnormalities_data.boundary_segments.left.empty() &&
    abnormalities_data.boundary_segments.right.empty()) {
    return tl::make_unexpected("Unable to find any closest segments");
  }

  for (const auto abnormality_type : param_.abnormality_types_to_compensate) {
    auto & proj_to_bound = abnormalities_data.projections_to_bound[abnormality_type];
    proj_to_bound = utils::get_closest_boundary_segments_from_side(
      trimmed_pred_traj, abnormalities_data.boundary_segments,
      abnormalities_data.footprints_sides[abnormality_type]);
  }
  return abnormalities_data;
}

std::vector<SegmentWithIdx> BoundaryDepartureChecker::find_closest_boundary_segments(
  const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
  const double ego_z_position,
  const std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id)
{
  if (!lanelet_map_ptr_ || !uncrossable_boundaries_rtree_ptr_) {
    return {};
  }

  const auto & rtree = *uncrossable_boundaries_rtree_ptr_;
  const auto max_lat_query_num = param_.th_max_lateral_query_num;
  const lanelet::BasicPoint2d ego_start{ego_ref_segment.first.x(), ego_ref_segment.first.y()};

  std::vector<SegmentWithIdx> nearest_raw;
  rtree.query(bgi::nearest(ego_start, max_lat_query_num), std::back_inserter(nearest_raw));

  std::vector<SegmentWithIdx> new_segments;
  for (const auto & nearest : nearest_raw) {
    const auto & id = nearest.second;
    if (unique_id.find(id) != unique_id.end()) {
      continue;  // Skip if this segment has already been added
    }

    auto boundary_segment_3d = get_segment_3d_from_id(lanelet_map_ptr_, id);

    if (!is_segment_within_ego_height(
          boundary_segment_3d, ego_z_position, vehicle_info_ptr_->vehicle_height_m)) {
      continue;
    }

    auto boundary_segment = utils::to_segment_2d(boundary_segment_3d);

    if (is_closest_to_boundary_segment(
          boundary_segment, ego_ref_segment, ego_opposite_ref_segment)) {
      new_segments.emplace_back(boundary_segment, id);
    }
  }
  return new_segments;
}

void BoundaryDepartureChecker::update_closest_boundary_segments(
  const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
  const double ego_z_position,
  std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_ids,
  std::vector<SegmentWithIdx> & output_segments)
{
  auto closest_segments = find_closest_boundary_segments(
    ego_ref_segment, ego_opposite_ref_segment, ego_z_position, unique_ids);

  for (auto & boundary_segment : closest_segments) {
    unique_ids.insert(boundary_segment.second);
    output_segments.emplace_back(std::move(boundary_segment));
  }
}

BoundarySideWithIdx BoundaryDepartureChecker::get_boundary_segments(
  const EgoSides & ego_sides_from_footprints, const TrajectoryPoints & trimmed_pred_trajectory)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  BoundarySideWithIdx boundary_sides_with_idx;

  std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> unique_ids;

  for (const auto & [fp, traj_pt] :
       ranges::views::zip(ego_sides_from_footprints, trimmed_pred_trajectory)) {
    const auto ego_z_position = traj_pt.pose.position.z;
    update_closest_boundary_segments(
      fp.left, fp.right, ego_z_position, unique_ids, boundary_sides_with_idx.left);
    update_closest_boundary_segments(
      fp.right, fp.left, ego_z_position, unique_ids, boundary_sides_with_idx.right);
  }

  return boundary_sides_with_idx;
}

tl::expected<std::vector<ClosestProjectionToBound>, std::string>
BoundaryDepartureChecker::get_closest_projections_to_boundaries_side(
  const Abnormalities<ProjectionsToBound> & projections_to_bound, const double min_braking_dist,
  const double max_braking_dist, const SideKey side_key)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & abnormality_to_check = param_.abnormality_types_to_compensate;

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
    return lat_dist < param_.th_trigger.th_dist_to_boundary_m[side_key].min;
  };

  const auto is_close_to_bound = [&](const double lat_dist, const SideKey side_key) {
    return lat_dist <= param_.th_trigger.th_dist_to_boundary_m[side_key].max;
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

      const auto create_min_pt =
        [](const auto pt, const auto dpt_type, const auto abnormality_type) {
          std::unique_ptr<ClosestProjectionToBound> min_pt =
            std::make_unique<ClosestProjectionToBound>(pt);
          min_pt->departure_type = dpt_type;
          min_pt->abnormality_type = abnormality_type;
          min_pt->time_from_start = pt.time_from_start;
          return min_pt;
        };

      if (abnormality_type == AbnormalityType::NORMAL && is_on_bound(pt.lat_dist, side_key)) {
        min_pt = create_min_pt(pt, DepartureType::CRITICAL_DEPARTURE, abnormality_type);
        break;
      }

      if (!is_close_to_bound(pt.lat_dist, side_key)) {
        continue;
      }

      if (!min_pt || pt.lat_dist < min_pt->lat_dist) {
        min_pt = create_min_pt(pt, DepartureType::NEAR_BOUNDARY, abnormality_type);
      }
    }
    if (!min_pt) {
      continue;
    }

    if (
      !min_to_bound.empty() && min_pt->departure_type != DepartureType::CRITICAL_DEPARTURE &&
      std::abs(min_to_bound.back().lon_dist_on_pred_traj - min_pt->lon_dist_on_pred_traj) < 0.5) {
      continue;
    }

    const auto is_exceeding_cutoff =
      [&min_pt](const auto type, const auto braking_dist, const auto cutoff_time) {
        return min_pt->departure_type == type && min_pt->lon_dist_on_pred_traj > braking_dist &&
               min_pt->time_from_start > cutoff_time;
      };

    if (is_exceeding_cutoff(
          DepartureType::NEAR_BOUNDARY, max_braking_dist, param_.th_cutoff_time_near_boundary_s)) {
      continue;
    }

    if (is_exceeding_cutoff(
          DepartureType::CRITICAL_DEPARTURE, min_braking_dist, param_.th_cutoff_time_departure_s)) {
      min_pt->departure_type = DepartureType::APPROACHING_DEPARTURE;
    }

    min_to_bound.push_back(*min_pt);
    if (min_to_bound.back().departure_type == DepartureType::CRITICAL_DEPARTURE) {
      break;
    }
  }

  return min_to_bound;
}

tl::expected<ClosestProjectionsToBound, std::string>
BoundaryDepartureChecker::get_closest_projections_to_boundaries(
  const Abnormalities<ProjectionsToBound> & projections_to_bound, const double curr_vel,
  const double curr_acc)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & th_trigger = param_.th_trigger;
  const auto min_braking_dist = utils::calc_judge_line_dist_with_jerk_limit(
    curr_vel, curr_acc, th_trigger.th_acc_mps2.max, th_trigger.th_jerk_mps3.max,
    th_trigger.brake_delay_s);
  const auto max_braking_dist = utils::calc_judge_line_dist_with_jerk_limit(
    curr_vel, curr_acc, th_trigger.th_acc_mps2.min, th_trigger.th_jerk_mps3.min,
    th_trigger.brake_delay_s);

  ClosestProjectionsToBound min_to_bound;

  for (const auto side_key : g_side_keys) {
    const auto min_to_bound_opt = get_closest_projections_to_boundaries_side(
      projections_to_bound, min_braking_dist, max_braking_dist, side_key);

    if (!min_to_bound_opt) {
      return tl::make_unexpected(min_to_bound_opt.error());
    }
    min_to_bound[side_key] = *min_to_bound_opt;

    if (min_to_bound[side_key].size() <= 1) {
      continue;
    }

    if (min_to_bound[side_key].back().departure_type != DepartureType::CRITICAL_DEPARTURE) {
      continue;
    }

    for (auto itr = std::next(min_to_bound[side_key].rbegin());
         itr != min_to_bound[side_key].rend(); ++itr) {
      if (
        min_to_bound[side_key].back().lon_dist_on_pred_traj - itr->lon_dist_on_pred_traj <
        max_braking_dist) {
        itr->departure_type = DepartureType::APPROACHING_DEPARTURE;
      }
    }
  }

  return min_to_bound;
}

Side<DeparturePoints> BoundaryDepartureChecker::get_departure_points(
  const ClosestProjectionsToBound & projections_to_bound,
  const std::vector<double> & pred_traj_idx_to_ref_traj_lon_dist)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto th_point_merge_distance_m = param_.th_point_merge_distance_m;

  Side<DeparturePoints> departure_points;
  for (const auto side_key : g_side_keys) {
    departure_points[side_key] = utils::get_departure_points(
      projections_to_bound[side_key], pred_traj_idx_to_ref_traj_lon_dist,
      th_point_merge_distance_m);
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

  return utils::build_uncrossable_boundaries_rtree(
    *lanelet_map_ptr, param_.boundary_types_to_detect);
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
