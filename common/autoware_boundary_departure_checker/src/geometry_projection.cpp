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

#include "autoware/boundary_departure_checker/geometry_projection.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker::geometry_projection
{
Point to_geom_pt(const Point2d & point, const double z = 0.0)
{
  return autoware_utils_geometry::to_msg(point.to_3d(z));
}

std::optional<std::pair<Point2d, double>> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 < 0.0) return std::nullopt;

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c1 > c2) return std::nullopt;

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};

  return std::make_pair(projection_point, boost::geometry::distance(p, projection_point));
}

std::optional<ProjectionToBound> calc_nearest_projection(
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

  if (projections.empty()) return std::nullopt;
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

    const double det = y_axis_direction.x() * (-segment_direction.y()) -
                       y_axis_direction.y() * (-segment_direction.x());

    if (std::abs(det) < 1e-10) {
      continue;
    }

    const Eigen::Vector2d rhs = segment_start - reference_point;
    const double t =
      ((-segment_direction.y()) * rhs.x() - (-segment_direction.x()) * rhs.y()) / det;
    const double s = (y_axis_direction.x() * rhs.y() - y_axis_direction.y() * rhs.x()) / det;

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

}  // namespace autoware::boundary_departure_checker::geometry_projection
