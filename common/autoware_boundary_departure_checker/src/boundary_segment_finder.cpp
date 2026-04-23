// Copyright 2026 TIER IV, Inc.
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

#include "autoware/boundary_departure_checker/boundary_segment_finder.hpp"

#include "autoware/boundary_departure_checker/conversion.hpp"
#include "autoware/boundary_departure_checker/geometry_projection.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <range/v3/view.hpp>

#include <algorithm>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker::boundary_segment_finder
{
Side<ProjectionsToBound> get_closest_boundary_segments_from_side(
  const TrajectoryPoints & ego_pred_traj, const BoundarySegmentsBySide & boundaries,
  const FootprintSideSegmentsArray & footprints_sides)
{
  Side<ProjectionsToBound> side;
  side.reserve_all(footprints_sides.size());
  Side<bool> has_passed_boundary{false, false};

  auto s = 0.0;
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    if (i > 0) {
      s += autoware_utils_geometry::calc_distance2d(ego_pred_traj[i - 1], ego_pred_traj[i]);
    }

    const auto & fp = footprints_sides[i];

    const auto & ego_lb = fp.left.second;
    const auto & ego_rb = fp.right.second;

    const auto front_seg = Segment2d(fp.left.first, fp.right.first);
    const auto rear_seg = Segment2d(ego_lb, ego_rb);

    side.for_each([&](auto key_constant, auto & side_value) {
      constexpr SideKey side_key = key_constant.value;
      auto closest_bound = geometry_projection::find_closest_segment(
        fp[side_key], front_seg, rear_seg, i, boundaries[side_key]);

      if (
        closest_bound.lat_dist > 0.0 &&
        closest_bound.lat_dist < std::numeric_limits<double>::max() &&
        has_passed_boundary[side_key]) {
        const auto & ego_front = fp[side_key].first;
        const auto & ego_rear = fp[side_key].second;

        // forward vector of ego side
        const double v_fwd_x = ego_front.x() - ego_rear.x();
        const double v_fwd_y = ego_front.y() - ego_rear.y();

        // lateral vector from ego to boundary
        const double v_lat_x = closest_bound.pt_on_bound.x() - closest_bound.pt_on_ego.x();
        const double v_lat_y = closest_bound.pt_on_bound.y() - closest_bound.pt_on_ego.y();

        // cross product for crossing direction
        const double cross_prod = v_fwd_x * v_lat_y - v_fwd_y * v_lat_x;

        const bool is_crossing_left_boundary = side_key == SideKey::LEFT && cross_prod < 0.0;
        const bool is_crossing_right_boundary = side_key == SideKey::RIGHT && cross_prod > 0.0;
        if (is_crossing_left_boundary || is_crossing_right_boundary) {
          closest_bound.lat_dist = -closest_bound.lat_dist;
        }
      }

      closest_bound.time_from_start =
        ego_pred_traj[i].time_from_start.sec + ego_pred_traj[i].time_from_start.nanosec * 1e-9;
      closest_bound.dist_along_trajectory_m = s;
      side_value.push_back(closest_bound);
      if (closest_bound.lat_dist < 0.01 && !has_passed_boundary[side_key]) {
        has_passed_boundary[side_key] = true;
      }
    });
  }

  return side;
}

bool is_closest_to_boundary_segment(
  const autoware_utils_geometry::Segment2d & boundary_segment,
  const autoware_utils_geometry::Segment2d & ego_side_ref_segment,
  const autoware_utils_geometry::Segment2d & ego_side_opposite_ref_segment)
{
  const auto dist_from_curr_side =
    boost::geometry::comparable_distance(ego_side_ref_segment, boundary_segment);
  const auto dist_from_compare_side =
    boost::geometry::comparable_distance(ego_side_opposite_ref_segment, boundary_segment);

  return dist_from_curr_side <= dist_from_compare_side;
}

bool is_segment_within_ego_height(
  const autoware_utils_geometry::Segment3d & boundary_segment, const double ego_z_position,
  const double ego_height)
{
  auto height_diff = std::min(
    std::abs(boundary_segment.first.z() - ego_z_position),
    std::abs(boundary_segment.second.z() - ego_z_position));
  return height_diff < ego_height;
}

std::vector<SegmentWithIdx> find_closest_boundary_segments(
  const UncrossableBoundariesRTree & rtree, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const UncrossableBoundaryDepartureParam & param, const Segment2d & ego_ref_segment,
  const Segment2d & ego_opposite_ref_segment, const double ego_z_position,
  const double ego_vehicle_height,
  std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id)
{
  if (!lanelet_map_ptr || rtree.empty()) {
    return {};
  }

  std::vector<SegmentWithIdx> nearest_raw = rtree.query(
    Point2d{ego_ref_segment.first.x(), ego_ref_segment.first.y()}, param.max_lateral_rtree_queries);

  std::vector<SegmentWithIdx> new_segments;
  for (const auto & nearest : nearest_raw) {
    const auto & id = nearest.second;
    if (unique_id.find(id) != unique_id.end()) {
      continue;
    }

    auto boundary_segment_3d = rtree.get_segment_3d_from_id(lanelet_map_ptr, id);

    if (!is_segment_within_ego_height(boundary_segment_3d, ego_z_position, ego_vehicle_height)) {
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

BoundarySegmentsBySide get_boundary_segments(
  const UncrossableBoundariesRTree & rtree, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const UncrossableBoundaryDepartureParam & param,
  const FootprintSideSegmentsArray & footprints_sides,
  const TrajectoryPoints & trimmed_pred_trajectory, const double ego_vehicle_height)
{
  BoundarySegmentsBySide boundary_sides_with_idx;
  std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> unique_ids;

  for (const auto & [fp, traj_pt] : ranges::views::zip(footprints_sides, trimmed_pred_trajectory)) {
    const auto ego_z = traj_pt.pose.position.z;

    auto left_segs = find_closest_boundary_segments(
      rtree, lanelet_map_ptr, param, fp.left, fp.right, ego_z, ego_vehicle_height, unique_ids);
    for (auto & seg : left_segs) {
      unique_ids.insert(seg.second);
      boundary_sides_with_idx.left.emplace_back(std::move(seg));
    }

    auto right_segs = find_closest_boundary_segments(
      rtree, lanelet_map_ptr, param, fp.right, fp.left, ego_z, ego_vehicle_height, unique_ids);
    for (auto & seg : right_segs) {
      unique_ids.insert(seg.second);
      boundary_sides_with_idx.right.emplace_back(std::move(seg));
    }
  }
  return boundary_sides_with_idx;
}

}  // namespace autoware::boundary_departure_checker::boundary_segment_finder
