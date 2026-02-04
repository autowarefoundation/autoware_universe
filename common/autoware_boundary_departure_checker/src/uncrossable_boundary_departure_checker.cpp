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

#include "autoware/boundary_departure_checker/uncrossable_boundary_departure_checker.hpp"

#include "autoware/boundary_departure_checker/conversion.hpp"
#include "autoware/boundary_departure_checker/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <range/v3/algorithm.hpp>
#include <range/v3/view.hpp>
#include <tf2/utils.hpp>
#include <tl_expected/expected.hpp>

#include <boost/geometry.hpp>

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

bool has_critical_departure(
  const std::vector<autoware::boundary_departure_checker::ProjectionToBound> & closest_projections)
{
  const auto is_critical_departure_type = [](const auto & pt) {
    return pt.is_critical_departure();
  };
  return std::any_of(
    closest_projections.rbegin(), closest_projections.rend(), is_critical_departure_type);
}
}  // namespace

namespace autoware::boundary_departure_checker
{
UncrossableBoundaryDepartureChecker::UncrossableBoundaryDepartureChecker(
  const rclcpp::Clock::SharedPtr clock_ptr, lanelet::LaneletMapPtr lanelet_map_ptr,
  const VehicleInfo & vehicle_info, Param param,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: param_(std::move(param)),
  lanelet_map_ptr_(std::move(lanelet_map_ptr)),
  vehicle_info_ptr_(std::make_shared<VehicleInfo>(vehicle_info)),
  last_no_critical_dpt_time_(clock_ptr->now().seconds()),
  last_found_critical_dpt_time_(clock_ptr->now().seconds()),
  clock_ptr_(clock_ptr),
  time_keeper_(std::move(time_keeper))
{
  auto try_uncrossable_boundaries_rtree = build_uncrossable_boundaries_tree(lanelet_map_ptr_);

  if (!try_uncrossable_boundaries_rtree) {
    throw std::runtime_error(try_uncrossable_boundaries_rtree.error());
  }

  uncrossable_boundaries_rtree_ptr_ =
    std::make_unique<UncrossableBoundRTree>(*try_uncrossable_boundaries_rtree);
}

void UncrossableBoundaryDepartureChecker::update_critical_departure_points(
  const std::vector<TrajectoryPoint> & raw_ref_traj, const double offset_from_ego,
  const Side<DeparturePoints> & new_departure_points,
  const Side<ProjectionsToBound> & closest_projections_to_bound)
{
  if (!is_critical_departure_persist(closest_projections_to_bound)) {
    critical_departure_points_.clear();
  }

  for (auto & crit_dpt_pt_mut : critical_departure_points_) {
    crit_dpt_pt_mut.ego_dist_on_ref_traj = autoware::motion_utils::calcSignedArcLength(
      raw_ref_traj, 0UL, crit_dpt_pt_mut.pose_on_current_ref_traj.position);

    if (crit_dpt_pt_mut.ego_dist_on_ref_traj < offset_from_ego) {
      crit_dpt_pt_mut.can_be_removed = true;
      continue;
    }

    const auto updated_pose =
      motion_utils::calcInterpolatedPose(raw_ref_traj, crit_dpt_pt_mut.ego_dist_on_ref_traj);
    if (
      const auto is_shifted_opt = utils::is_point_shifted(
        crit_dpt_pt_mut.pose_on_current_ref_traj, updated_pose, param_.th_pt_shift_dist_m,
        param_.th_pt_shift_angle_rad)) {
      crit_dpt_pt_mut.can_be_removed = true;
    }
  }

  auto remove_itr = std::remove_if(
    critical_departure_points_.begin(), critical_departure_points_.end(),
    [](const DeparturePoint & pt) { return pt.can_be_removed; });

  critical_departure_points_.erase(remove_itr, critical_departure_points_.end());

  if (!is_continuous_critical_departure(closest_projections_to_bound)) {
    return;
  }

  auto new_critical_departure_point = find_new_critical_departure_points(
    new_departure_points, critical_departure_points_, raw_ref_traj,
    param_.th_point_merge_distance_m);

  if (new_critical_departure_point.empty()) {
    return;
  }

  std::move(
    new_critical_departure_point.begin(), new_critical_departure_point.end(),
    std::back_inserter(critical_departure_points_));

  std::sort(critical_departure_points_.begin(), critical_departure_points_.end());
}

bool is_critical_departure(const Side<ProjectionsToBound> & closest_projections_to_bound)
{
  const auto check_side_for_critical_departure = [&](const auto side_key) {
    const auto & closest_projections = closest_projections_to_bound[side_key];
    return has_critical_departure(closest_projections);
  };

  return std::any_of(g_side_keys.begin(), g_side_keys.end(), check_side_for_critical_departure);
}

bool UncrossableBoundaryDepartureChecker::is_continuous_critical_departure(
  const Side<ProjectionsToBound> & closest_projections_to_bound)
{
  const auto is_critical_departure_detected = is_critical_departure(closest_projections_to_bound);

  if (!is_critical_departure_detected) {
    last_no_critical_dpt_time_ = clock_ptr_->now().seconds();
    return false;
  }

  const auto t_diff = clock_ptr_->now().seconds() - last_no_critical_dpt_time_;
  return t_diff >= param_.critical_departure_on_time_buffer_s;
}

bool UncrossableBoundaryDepartureChecker::is_critical_departure_persist(
  const Side<ProjectionsToBound> & closest_projections_to_bound)
{
  const auto is_critical_departure_detected =
    is_critical_departure(closest_projections_to_bound) && !critical_departure_points_.empty();

  if (is_critical_departure_detected) {
    last_found_critical_dpt_time_ = clock_ptr_->now().seconds();
    return true;
  }

  const auto t_diff = clock_ptr_->now().seconds() - last_found_critical_dpt_time_;
  return t_diff >= param_.critical_departure_off_time_buffer_s;
}

DeparturePoints UncrossableBoundaryDepartureChecker::find_new_critical_departure_points(
  const Side<DeparturePoints> & new_departure_points,
  const DeparturePoints & critical_departure_points,
  const std::vector<TrajectoryPoint> & raw_ref_traj, const double th_point_merge_distance_m)
{
  DeparturePoints new_critical_departure_points;
  for (const auto side_key : g_side_keys) {
    for (const auto & dpt_pt : new_departure_points[side_key]) {
      if (dpt_pt.departure_type != DepartureType::CRITICAL_DEPARTURE) {
        continue;
      }

      if (dpt_pt.can_be_removed) {
        continue;
      }

      const auto is_near_curr_pts = std::any_of(
        critical_departure_points.begin(), critical_departure_points.end(),
        [&](const DeparturePoint & crit_pt) {
          return std::abs(dpt_pt.ego_dist_on_ref_traj - crit_pt.ego_dist_on_ref_traj) <
                 th_point_merge_distance_m;
        });

      if (is_near_curr_pts) {
        continue;
      }

      DeparturePoint crit_pt = dpt_pt;
      crit_pt.pose_on_current_ref_traj =
        motion_utils::calcInterpolatedPose(raw_ref_traj, crit_pt.ego_dist_on_ref_traj);
      new_critical_departure_points.push_back(crit_pt);
    }
  }
  return new_critical_departure_points;
}

tl::expected<AbnormalitiesData, std::string>
UncrossableBoundaryDepartureChecker::get_abnormalities_data(
  const TrajectoryPoints & trajectory_points, const TrajectoryPoints & predicted_traj,
  const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov,
  const SteeringReport & current_steering, const double curr_vel, const double curr_acc)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (predicted_traj.empty()) {
    return tl::make_unexpected("Ego predicted trajectory is empty");
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

  auto closest_projections_to_bound_opt = get_closest_projections_to_boundaries(
    abnormalities_data.projections_to_bound, curr_vel, curr_acc);

  if (!closest_projections_to_bound_opt) {
    return tl::make_unexpected(closest_projections_to_bound_opt.error());
  }

  abnormalities_data.closest_projections_to_bound = std::move(*closest_projections_to_bound_opt);

  std::vector<double> pred_traj_idx_to_ref_traj_lon_dist;
  pred_traj_idx_to_ref_traj_lon_dist.reserve(predicted_traj.size());
  for (const auto & p : predicted_traj) {
    pred_traj_idx_to_ref_traj_lon_dist.push_back(
      motion_utils::calcSignedArcLength(trajectory_points, 0UL, p.pose.position));
  }

  abnormalities_data.departure_points = get_departure_points(
    abnormalities_data.closest_projections_to_bound, pred_traj_idx_to_ref_traj_lon_dist);

  const auto ego_dist_on_traj_m =
    motion_utils::calcSignedArcLength(trajectory_points, 0UL, curr_pose_with_cov.pose.position);

  update_critical_departure_points(
    trajectory_points, ego_dist_on_traj_m, abnormalities_data.departure_points,
    abnormalities_data.closest_projections_to_bound);

  abnormalities_data.critical_departure_points = critical_departure_points_;

  return abnormalities_data;
}

std::vector<SegmentWithIdx> UncrossableBoundaryDepartureChecker::find_closest_boundary_segments(
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

void UncrossableBoundaryDepartureChecker::update_closest_boundary_segments(
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

BoundarySideWithIdx UncrossableBoundaryDepartureChecker::get_boundary_segments(
  const EgoSides & ego_sides_from_footprints, const TrajectoryPoints & trimmed_pred_trajectory)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

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

tl::expected<ProjectionsToBound, std::string>
UncrossableBoundaryDepartureChecker::get_closest_projections_to_boundaries_side(
  const Abnormalities<Side<ProjectionsToBound>> & projections_to_bound,
  const double min_braking_dist, const double max_braking_dist, const SideKey side_key)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

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

  ProjectionsToBound min_to_bound;

  const auto is_on_bound = [this](const double lat_dist, const SideKey side_key) {
    return lat_dist < param_.th_trigger.th_dist_to_boundary_m[side_key].min;
  };

  const auto is_close_to_bound = [&](const double lat_dist, const SideKey side_key) {
    return lat_dist <= param_.th_trigger.th_dist_to_boundary_m[side_key].max;
  };

  const auto fp_size = projections_to_bound[abnormality_to_check.front()][side_key].size();
  min_to_bound.reserve(fp_size);
  for (size_t idx = 0; idx < fp_size; ++idx) {
    std::unique_ptr<ProjectionToBound> min_pt;
    for (const auto abnormality_type : abnormality_to_check) {
      const auto pt = projections_to_bound[abnormality_type][side_key][idx];
      if (pt.ego_sides_idx != idx) {
        continue;
      }

      const auto create_min_pt =
        [](const auto pt, const auto dpt_type, const auto abnormality_type) {
          std::unique_ptr<ProjectionToBound> min_pt = std::make_unique<ProjectionToBound>(pt);
          min_pt->departure_type_opt = dpt_type;
          min_pt->abnormality_type_opt = abnormality_type;
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
    if (!min_pt || !min_pt->departure_type_opt || !min_pt->abnormality_type_opt) {
      continue;
    }

    if (
      !min_to_bound.empty() && !min_pt->is_critical_departure() &&
      std::abs(min_to_bound.back().lon_dist_on_pred_traj - min_pt->lon_dist_on_pred_traj) < 0.5) {
      continue;
    }

    const auto is_exceeding_cutoff =
      [&min_pt](const auto type, const auto braking_dist, const auto cutoff_time) {
        return min_pt->departure_type_opt.value() == type &&
               min_pt->lon_dist_on_pred_traj > braking_dist &&
               min_pt->time_from_start > cutoff_time;
      };

    if (is_exceeding_cutoff(
          DepartureType::NEAR_BOUNDARY, max_braking_dist, param_.th_cutoff_time_near_boundary_s)) {
      continue;
    }

    if (is_exceeding_cutoff(
          DepartureType::CRITICAL_DEPARTURE, min_braking_dist, param_.th_cutoff_time_departure_s)) {
      min_pt->departure_type_opt = DepartureType::APPROACHING_DEPARTURE;
    }

    min_to_bound.push_back(*min_pt);
    if (min_to_bound.back().is_critical_departure()) {
      break;
    }
  }

  return min_to_bound;
}

tl::expected<Side<ProjectionsToBound>, std::string>
UncrossableBoundaryDepartureChecker::get_closest_projections_to_boundaries(
  const Abnormalities<Side<ProjectionsToBound>> & projections_to_bound, const double curr_vel,
  const double curr_acc)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & th_trigger = param_.th_trigger;
  const auto min_braking_dist = utils::calc_judge_line_dist_with_jerk_limit(
    curr_vel, curr_acc, th_trigger.th_acc_mps2.max, th_trigger.th_jerk_mps3.max,
    th_trigger.brake_delay_s);
  const auto max_braking_dist = utils::calc_judge_line_dist_with_jerk_limit(
    curr_vel, curr_acc, th_trigger.th_acc_mps2.min, th_trigger.th_jerk_mps3.min,
    th_trigger.brake_delay_s);

  Side<ProjectionsToBound> min_to_bound;

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

    if (!min_to_bound[side_key].back().is_critical_departure()) {
      continue;
    }

    for (auto itr = std::next(min_to_bound[side_key].rbegin());
         itr != min_to_bound[side_key].rend(); ++itr) {
      if (
        min_to_bound[side_key].back().lon_dist_on_pred_traj - itr->lon_dist_on_pred_traj <
        max_braking_dist) {
        itr->departure_type_opt = DepartureType::APPROACHING_DEPARTURE;
      }
    }
  }

  return min_to_bound;
}

Side<DeparturePoints> UncrossableBoundaryDepartureChecker::get_departure_points(
  const Side<ProjectionsToBound> & projections_to_bound,
  const std::vector<double> & pred_traj_idx_to_ref_traj_lon_dist)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

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
UncrossableBoundaryDepartureChecker::build_uncrossable_boundaries_tree(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!lanelet_map_ptr) {
    return tl::make_unexpected("lanelet_map_ptr is null");
  }

  return utils::build_uncrossable_boundaries_rtree(
    *lanelet_map_ptr, param_.boundary_types_to_detect);
}
}  // namespace autoware::boundary_departure_checker
