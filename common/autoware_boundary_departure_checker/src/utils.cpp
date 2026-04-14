// Copyright 2024 TIER IV, Inc.
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

#include "autoware/boundary_departure_checker/utils.hpp"

#include "autoware/boundary_departure_checker/conversion.hpp"
#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <range/v3/view.hpp>
#include <tl_expected/expected.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::boundary_departure_checker::IdxForRTreeSegment;
using autoware::boundary_departure_checker::ProjectionToBound;
using autoware::boundary_departure_checker::Segment2d;
using autoware::boundary_departure_checker::SegmentWithIdx;
using autoware::boundary_departure_checker::utils::to_segment_2d;
namespace bg = boost::geometry;

std::vector<SegmentWithIdx> create_local_segments(const lanelet::ConstLineString3d & linestring)
{
  std::vector<SegmentWithIdx> local_segments;
  local_segments.reserve(linestring.size());
  const auto basic_ls = linestring.basicLineString();
  for (size_t i = 0; i + 1 < basic_ls.size(); ++i) {
    const auto segment = to_segment_2d(basic_ls.at(i), basic_ls.at(i + 1));
    local_segments.emplace_back(
      bg::return_envelope<Segment2d>(segment), IdxForRTreeSegment(linestring.id(), i, i + 1));
  }
  return local_segments;
}
}  // namespace

namespace autoware::boundary_departure_checker::utils
{
ProjectionsToBound filter_and_assign_departure_types(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param,
  const double min_braking_dist)
{
  ProjectionsToBound out;
  out.reserve(side_value.size());

  DepartureCheckThresholds thresholds;
  thresholds.min_braking_distance = min_braking_dist;
  thresholds.cutoff_time = param.time_to_departure_cutoff_s;
  thresholds.th_lat_critical = param.lateral_margin_m;

  for (size_t idx = 0; idx < side_value.size(); ++idx) {
    const auto & original_candidate = side_value[idx];
    if (original_candidate.pose_index != idx) continue;

    ProjectionEvaluationMetrics metrics;
    metrics.lon_dist_to_departure =
      original_candidate.dist_along_trajectory_m - original_candidate.ego_front_to_proj_offset_m;
    metrics.time_from_start = original_candidate.time_from_start;
    metrics.lat_dist = original_candidate.lat_dist;

    out.push_back(original_candidate);
    out.back().departure_type = assign_departure_type(metrics, thresholds);

    if (out.back().is_critical()) break;
  }

  return out;
}

std::optional<CriticalPointPair> apply_backward_buffer_and_filter(
  const ProjectionsToBound & side_value, const UncrossableBoundaryDepartureParam & param)
{
  if (side_value.empty() || side_value.back().is_none_departure()) return std::nullopt;

  const auto & departure_point = side_value.back();

  CriticalPointPair result;
  result.physical_departure_point = departure_point;
  result.safety_buffer_start = departure_point;  // Default to the crash point itself

  // Only apply buffering if the intersection is actually critical
  if (!departure_point.is_critical()) {
    return result;  // No need to search backwards if it's not critical
  }

  const double departure_s =
    departure_point.dist_along_trajectory_m - departure_point.ego_front_to_proj_offset_m;

  // Search backwards for the earliest point within the longitudinal buffer
  for (auto it = std::next(side_value.rbegin()); it != side_value.rend(); ++it) {
    const double dist_to_crash = departure_s - it->dist_along_trajectory_m;

    if (dist_to_crash <= param.longitudinal_margin_m) {
      result.safety_buffer_start = *it;
      result.safety_buffer_start.departure_type = DepartureType::CRITICAL;
    } else {
      // dist_to_crash strictly increases, so we can stop searching.
      break;
    }
  }

  return result;
}

Side<std::optional<CriticalPointPair>> evaluate_projections_severity(
  const Side<ProjectionsToBound> & projections_to_bound,
  const UncrossableBoundaryDepartureParam & param, const EgoDynamicState & ego_state,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const auto min_braking_dist =
    utils::calc_minimum_braking_distance(ego_state, param, vehicle_info);

  return projections_to_bound.transform_each_side([&](const auto & side_value) {
    const auto min_to_bounds =
      filter_and_assign_departure_types(side_value, param, min_braking_dist);
    return apply_backward_buffer_and_filter(min_to_bounds, param);
  });
}

DepartureType assign_departure_type(
  const ProjectionEvaluationMetrics & metrics, const DepartureCheckThresholds & thresholds)
{
  if (metrics.lat_dist > thresholds.th_lat_critical) {
    return DepartureType::NONE;
  }

  if (
    metrics.lon_dist_to_departure > thresholds.min_braking_distance &&
    metrics.time_from_start > thresholds.cutoff_time) {
    return DepartureType::APPROACHING;
  }
  // Set CRITICAL if:
  // - Short Dist & Short Time: boundary crossing is less than braking distance and we will hit it
  // in less than cutoff time.
  // - Long Dist but Short Time: At 100 km/h, the boundary crossing is 30 meters away, but ego
  // will hit the crossing in less than cutoff time.
  // - Long time, but dist less than braking: Creeping forward in a parking lot at 2 km/h, and it
  // takes it will 4 seconds to reach it, however, the boundary less than minimum braking
  // distance.
  return DepartureType::CRITICAL;
}
bool is_uncrossable_type(
  std::vector<std::string> boundary_types_to_detect, const lanelet::ConstLineString3d & ls)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  return (
    type != no_type &&
    std::find(boundary_types_to_detect.begin(), boundary_types_to_detect.end(), type) !=
      boundary_types_to_detect.end());
};

UncrossableBoundsRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect)
{
  std::vector<SegmentWithIdx> segments;
  for (const auto & linestring : lanelet_map.lineStringLayer) {
    if (!is_uncrossable_type(boundary_types_to_detect, linestring)) {
      continue;
    }

    auto local_segments = create_local_segments(linestring);
    std::move(local_segments.begin(), local_segments.end(), std::back_inserter(segments));
  }

  return {segments.begin(), segments.end()};
}

tl::expected<std::pair<Point2d, double>, std::string> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 < 0.0) return tl::make_unexpected("Point before segment start");

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c1 > c2) return tl::make_unexpected("Point after segment end");

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};

  return std::make_pair(projection_point, boost::geometry::distance(p, projection_point));
}

tl::expected<ProjectionToBound, std::string> calc_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t pose_index)
{
  const auto & [ego_f, ego_b] = ego_seg;
  const auto & [lane_pt1, lane_pt2] = lane_seg;

  if (
    const auto is_intersecting = autoware_utils_geometry::intersect(
      to_geom_pt(ego_f), to_geom_pt(ego_b), to_geom_pt(lane_pt1), to_geom_pt(lane_pt2))) {
    Point2d point(is_intersecting->x, is_intersecting->y);
    return ProjectionToBound{
      point, point, lane_seg, 0.0, boost::geometry::distance(point, ego_f), pose_index};
  }

  ProjectionsToBound projections;
  projections.reserve(4);
  if (const auto projection_opt = point_to_segment_projection(ego_f, lane_seg)) {
    const auto & [proj, dist] = *projection_opt;
    constexpr auto ego_front_to_proj_offset_m = 0.0;
    projections.emplace_back(ego_f, proj, lane_seg, dist, ego_front_to_proj_offset_m, pose_index);
  }

  if (const auto projection_opt = point_to_segment_projection(ego_b, lane_seg)) {
    const auto & [proj, dist] = *projection_opt;
    const auto ego_front_to_proj_offset_m = boost::geometry::distance(ego_b, ego_f);
    projections.emplace_back(ego_b, proj, lane_seg, dist, ego_front_to_proj_offset_m, pose_index);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt1, ego_seg)) {
    const auto & [proj, dist] = *projection_opt;
    const auto ego_front_to_proj_offset_m = boost::geometry::distance(proj, ego_f);
    projections.emplace_back(
      proj, lane_pt1, lane_seg, dist, ego_front_to_proj_offset_m, pose_index);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt2, ego_seg)) {
    const auto & [proj, dist] = *projection_opt;
    const auto ego_front_to_proj_offset_m = boost::geometry::distance(proj, ego_f);
    projections.emplace_back(
      proj, lane_pt2, lane_seg, dist, ego_front_to_proj_offset_m, pose_index);
  }

  if (projections.empty())
    return tl::make_unexpected("Couldn't generate projection at " + std::to_string(pose_index));
  if (projections.size() == 1) return projections.front();

  auto min_elem = std::min_element(
    projections.begin(), projections.end(),
    [](const ProjectionToBound & proj1, const ProjectionToBound & proj2) {
      return std::abs(proj1.lat_dist) < std::abs(proj2.lat_dist);
    });

  return *min_elem;
}

ProjectionToBound find_closest_segment(
  const Segment2d & ego_side_seg, const Segment2d & ego_rear_seg, const size_t curr_fp_idx,
  const std::vector<SegmentWithIdx> & boundary_segments)
{
  std::optional<ProjectionToBound> closest_proj;
  for (const auto & [seg, id] : boundary_segments) {
    const auto & [ego_lr, ego_rr] = ego_rear_seg;
    const auto & [seg_f, seg_r] = seg;
    // we can assume that before front touches boundary, either left or right side will touch
    // boundary first
    if (const auto proj_opt = calc_nearest_projection(ego_side_seg, seg, curr_fp_idx)) {
      if (!closest_proj || proj_opt->lat_dist < closest_proj->lat_dist) {
        closest_proj = *proj_opt;
      }
    }
    if (closest_proj) {
      continue;
    }

    if (
      const auto is_intersecting_rear = autoware_utils_geometry::intersect(
        to_geom_pt(ego_lr), to_geom_pt(ego_rr), to_geom_pt(seg_f), to_geom_pt(seg_r))) {
      Point2d point(is_intersecting_rear->x, is_intersecting_rear->y);
      closest_proj =
        ProjectionToBound{point,
                          point,
                          seg,
                          0.0,
                          boost::geometry::distance(ego_side_seg.first, ego_side_seg.second),
                          curr_fp_idx};
      break;
    }
  }

  if (closest_proj) {
    return *closest_proj;
  }

  return ProjectionToBound(curr_fp_idx);
}

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

    const auto rear_seg = Segment2d(ego_lb, ego_rb);

    side.for_each([&](auto key_constant, auto & side_value) {
      constexpr SideKey side_key = key_constant.value;
      auto closest_bound = find_closest_segment(fp[side_key], rear_seg, i, boundaries[side_key]);

      if (
        closest_bound.lat_dist > 0.0 &&
        closest_bound.lat_dist < std::numeric_limits<double>::max() &&
        has_passed_boundary[side_key]) {
        const auto & ego_front = fp[side_key].first;
        const auto & ego_rear = fp[side_key].second;

        // Forward vector of the ego side segment
        const double v_fwd_x = ego_front.x() - ego_rear.x();
        const double v_fwd_y = ego_front.y() - ego_rear.y();

        // Lateral vector pointing from ego to the boundary
        const double v_lat_x = closest_bound.pt_on_bound.x() - closest_bound.pt_on_ego.x();
        const double v_lat_y = closest_bound.pt_on_bound.y() - closest_bound.pt_on_ego.y();

        // 2D Cross Product (Z-component)
        const double cross_prod = v_fwd_x * v_lat_y - v_fwd_y * v_lat_x;

        const bool is_crossing_left_boundary = side_key == SideKey::LEFT && cross_prod < 0.0;
        const bool is_crossing_right_boundary = side_key == SideKey::RIGHT && cross_prod > 0.0;
        if (is_crossing_left_boundary || is_crossing_right_boundary) {
          closest_bound.lat_dist = -closest_bound.lat_dist;  // crossed left boundary
        }
      }

      closest_bound.time_from_start = rclcpp::Duration(ego_pred_traj[i].time_from_start).seconds();
      closest_bound.dist_along_trajectory_m = s - closest_bound.ego_front_to_proj_offset_m;
      side_value.push_back(closest_bound);
      if (closest_bound.lat_dist < 0.01 && !has_passed_boundary[side_key]) {
        has_passed_boundary[side_key] = true;
      }
    });
  }

  return side;
}

std::optional<double> calc_signed_lateral_distance_to_boundary(
  const lanelet::ConstLineString3d & boundary, const Pose & reference_pose)
{
  if (boundary.size() < 2) {
    return std::nullopt;
  }

  const double yaw = tf2::getYaw(reference_pose.orientation);
  const Eigen::Vector2d y_axis_direction(-std::sin(yaw), std::cos(yaw));
  const Eigen::Vector2d reference_point(reference_pose.position.x, reference_pose.position.y);

  double min_distance = std::numeric_limits<double>::max();
  std::optional<double> signed_lateral_distance;

  for (size_t i = 0; i + 1 < boundary.size(); ++i) {
    const auto & p1 = boundary[i];
    const auto & p2 = boundary[i + 1];

    const Eigen::Vector2d segment_start(p1.x(), p1.y());
    const Eigen::Vector2d segment_end(p2.x(), p2.y());
    const Eigen::Vector2d segment_direction = segment_end - segment_start;

    // Calculate intersection between Y-axis line and boundary segment
    const double det = y_axis_direction.x() * (-segment_direction.y()) -
                       y_axis_direction.y() * (-segment_direction.x());

    if (std::abs(det) < 1e-10) {
      // this segment and the Y-axis are parallel
      continue;
    }

    const Eigen::Vector2d rhs = segment_start - reference_point;
    const double t =
      ((-segment_direction.y()) * rhs.x() - (-segment_direction.x()) * rhs.y()) / det;
    const double s = (y_axis_direction.x() * rhs.y() - y_axis_direction.y() * rhs.x()) / det;

    // Check if intersection is within segment bounds
    if (s >= 0.0 && s <= 1.0) {
      const double distance = std::abs(t);

      if (distance < min_distance) {
        min_distance = distance;
        signed_lateral_distance = t;
      }
    }
  }

  return signed_lateral_distance;
}

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

bool is_segment_within_ego_height(
  const autoware_utils_geometry::Segment3d & boundary_segment, const double ego_z_position,
  const double ego_height)
{
  auto height_diff = std::min(
    std::abs(boundary_segment.first.z() - ego_z_position),
    std::abs(boundary_segment.second.z() - ego_z_position));
  return height_diff < ego_height;
}

bool is_critical(const Side<std::optional<CriticalPointPair>> & evaluated_projections)
{
  return evaluated_projections.any_of_side([](const auto & critical_pair_opt) {
    return critical_pair_opt.has_value() &&
           critical_pair_opt->physical_departure_point.is_critical();
  });
}

double calc_minimum_braking_distance(
  const EgoDynamicState & ego_state, const UncrossableBoundaryDepartureParam & param,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  // 1. Calculate the kinematic distance needed to stop the base_link coordinate
  const auto kinematic_stop_dist = motion_utils::calculate_stop_distance(
    ego_state.velocity, ego_state.acceleration, param.max_deceleration_mps2, param.max_jerk_mps3,
    param.brake_delay_s);

  // 2. Total distance = (Front Overhang) + (Kinematic Braking Distance)
  // Even at zero velocity, the "braking zone" is the front of the car.
  return vehicle_info.front_overhang_m +
         (kinematic_stop_dist ? std::max(0.0, *kinematic_stop_dist) : 0.0);
}
}  // namespace autoware::boundary_departure_checker::utils
