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

#include "autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp"

#include "autoware/boundary_departure_checker/conversion.hpp"
#include "autoware/boundary_departure_checker/footprints_generator.hpp"
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

namespace autoware::boundary_departure_checker
{
void UncrossableBoundaryChecker::set_lanelet_map(const lanelet::LaneletMapPtr lanelet_map_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
}

tl::expected<void, std::string> UncrossableBoundaryChecker::initialize()
{
  if (!lanelet_map_ptr_ || lanelet_map_ptr_->lineStringLayer.empty()) {
    return tl::make_unexpected("Invalid lanelet map pointer or empty linestring layer");
  }

  uncrossable_boundaries_rtree_ptr_ = std::make_unique<UncrossableBoundsRTree>(
    utils::build_uncrossable_boundaries_rtree(*lanelet_map_ptr_, param_.boundary_types_to_detect));

  return {};
}

void UncrossableBoundaryChecker::set_param(const UncrossableBoundaryDepartureParam & param)
{
  param_ = param;
}

tl::expected<DepartureData, std::string> UncrossableBoundaryChecker::check_departure(
  const TrajectoryPoints & predicted_traj, const vehicle_info_utils::VehicleInfo & vehicle_info,
  const EgoDynamicState & ego_state)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!lanelet_map_ptr_ || !uncrossable_boundaries_rtree_ptr_) {
    return tl::make_unexpected("Checker not properly initialized with lanelet map and R-tree");
  }

  if (predicted_traj.empty()) {
    return {};
  }

  DepartureData departure_data;
  departure_data.footprints =
    footprints::generate(predicted_traj, vehicle_info, ego_state.pose_with_cov);

  if (predicted_traj.size() != departure_data.footprints.size()) {
    return tl::make_unexpected(
      "Size of generated footprints does not match size of predicted trajectory");
  }

  departure_data.footprints_sides =
    footprints::get_sides_from_footprints(departure_data.footprints);

  if (departure_data.footprints.size() != departure_data.footprints_sides.size()) {
    return tl::make_unexpected(
      "Size of generated footprints does not match size of predicted trajectory");
  }

  departure_data.boundary_segments = get_boundary_segments(
    departure_data.footprints_sides, predicted_traj, vehicle_info.vehicle_height_m);

  if (departure_data.boundary_segments.all_empty()) {
    // couldn't find any nearby boundary segments, so we can skip the rest of the processing and
    // return early
    return departure_data;
  }

  departure_data.projections_to_bound = utils::get_closest_boundary_segments_from_side(
    predicted_traj, departure_data.boundary_segments, departure_data.footprints_sides);

  departure_data.evaluated_projections = utils::evaluate_projections_severity(
    departure_data.projections_to_bound, param_, ego_state, vehicle_info);

  departure_data.status =
    determine_departure_type(departure_data.evaluated_projections, ego_state.current_time_s);

  departure_data.critical_departure_history = critical_departure_;

  return departure_data;
}

DepartureType UncrossableBoundaryChecker::determine_departure_type(
  const Side<std::optional<CriticalPointPair>> & evaluated_projections, const double current_time_s)
{
  const bool current_is_critical = utils::is_critical(evaluated_projections);

  if (current_is_critical) {
    // Geometrically critical. Check if the ON buffer has expired.
    if (current_time_s - last_no_critical_dpt_time_ >= param_.on_time_buffer_s) {
      // We officially entered the CRITICAL state. Record this exact time!
      last_found_critical_dpt_time_ = current_time_s;

      // Save the critical points for visualization/downstream use
      critical_departure_.for_each_side([](auto & side) { side.clear(); });
      evaluated_projections.for_each([&](auto key_constant, auto & side_value) {
        if (side_value.has_value() && side_value->safety_buffer_start.is_critical()) {
          critical_departure_[key_constant.value].push_back(side_value->physical_departure_point);
        }
      });
      return DepartureType::CRITICAL;
    }
    // Geometrically critical, but still waiting for the ON buffer to expire
    return DepartureType::NONE;
  }
  // Geometrically safe. Continually update the safe timestamp.
  last_no_critical_dpt_time_ = current_time_s;

  // If we were previously in a CRITICAL state, check the OFF buffer
  if (!critical_departure_.all_empty()) {
    if (current_time_s - last_found_critical_dpt_time_ < param_.off_time_buffer_s) {
      return DepartureType::CRITICAL;  // Hold the CRITICAL state!
    }
    // The OFF buffer has officially expired. Clear the saved state.
    critical_departure_.for_each_side([](auto & side) { side.clear(); });
  }

  return DepartureType::NONE;
}

std::vector<SegmentWithIdx> UncrossableBoundaryChecker::find_closest_boundary_segments(
  const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
  const double ego_z_position, const double ego_vehicle_height,
  const std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id) const
{
  if (!lanelet_map_ptr_ || !uncrossable_boundaries_rtree_ptr_) {
    return {};
  }

  const auto & rtree = *uncrossable_boundaries_rtree_ptr_;
  const lanelet::BasicPoint2d ego_start{ego_ref_segment.first.x(), ego_ref_segment.first.y()};

  std::vector<SegmentWithIdx> nearest_raw;
  rtree.query(
    bgi::nearest(ego_start, param_.max_lateral_rtree_queries), std::back_inserter(nearest_raw));

  std::vector<SegmentWithIdx> new_segments;
  for (const auto & nearest : nearest_raw) {
    const auto & id = nearest.second;
    if (unique_id.find(id) != unique_id.end()) {
      continue;  // Skip if this segment has already been added
    }

    auto boundary_segment_3d = utils::get_segment_3d_from_id(lanelet_map_ptr_, id);

    if (!utils::is_segment_within_ego_height(
          boundary_segment_3d, ego_z_position, ego_vehicle_height)) {
      continue;
    }

    auto boundary_segment = utils::to_segment_2d(boundary_segment_3d);

    if (utils::is_closest_to_boundary_segment(
          boundary_segment, ego_ref_segment, ego_opposite_ref_segment)) {
      new_segments.emplace_back(boundary_segment, id);
    }
  }
  return new_segments;
}

BoundarySegmentsBySide UncrossableBoundaryChecker::get_boundary_segments(
  const FootprintSideSegmentsArray & footprints_sides,
  const TrajectoryPoints & trimmed_pred_trajectory, const double ego_vehicle_height) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  BoundarySegmentsBySide boundary_sides_with_idx;
  std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> unique_ids;

  for (const auto & [fp, traj_pt] : ranges::views::zip(footprints_sides, trimmed_pred_trajectory)) {
    const auto ego_z = traj_pt.pose.position.z;

    auto left_segs =
      find_closest_boundary_segments(fp.left, fp.right, ego_z, ego_vehicle_height, unique_ids);
    for (auto & seg : left_segs) {
      unique_ids.insert(seg.second);
      boundary_sides_with_idx.left.emplace_back(std::move(seg));
    }

    auto right_segs =
      find_closest_boundary_segments(fp.right, fp.left, ego_z, ego_vehicle_height, unique_ids);
    for (auto & seg : right_segs) {
      unique_ids.insert(seg.second);
      boundary_sides_with_idx.right.emplace_back(std::move(seg));
    }
  }
  return boundary_sides_with_idx;
}
}  // namespace autoware::boundary_departure_checker
