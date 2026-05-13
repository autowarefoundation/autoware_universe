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

#include "autoware/deprecated/boundary_departure_checker/utils.hpp"

#include "autoware/deprecated/boundary_departure_checker/conversion.hpp"
#include "autoware/deprecated/boundary_departure_checker/data_structs.hpp"
#include "autoware/deprecated/boundary_departure_checker/parameters.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
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
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using autoware::boundary_departure_checker::DeparturePoint;
using autoware::boundary_departure_checker::DeparturePoints;
using autoware::boundary_departure_checker::DepartureType;
using autoware::boundary_departure_checker::IdxForRTreeSegment;
using autoware::boundary_departure_checker::ProjectionToBound;
using autoware::boundary_departure_checker::Segment2d;
using autoware::boundary_departure_checker::SegmentWithIdx;
using autoware::boundary_departure_checker::VehicleInfo;
using autoware::boundary_departure_checker::utils::to_segment_2d;
namespace bg = boost::geometry;

DeparturePoint create_departure_point(
  const ProjectionToBound & projection_to_bound,
  const std::vector<double> & pred_traj_idx_to_ref_traj_lon_dist,
  const double th_point_merge_distance_m)
{
  DeparturePoint point;
  point.lat_dist_to_bound = projection_to_bound.lat_dist;
  point.can_be_removed =
    !projection_to_bound.departure_type_opt || point.ego_dist_on_ref_traj <= 0.0;
  if (point.can_be_removed) {
    return point;
  }
  point.departure_type = projection_to_bound.departure_type_opt.value();
  point.point = projection_to_bound.pt_on_bound;
  point.th_point_merge_distance_m = th_point_merge_distance_m;
  point.idx_from_ego_traj = projection_to_bound.ego_sides_idx;
  point.ego_dist_on_ref_traj =
    pred_traj_idx_to_ref_traj_lon_dist[projection_to_bound.ego_sides_idx];
  return point;
}

void erase_after_first_match(DeparturePoints & departure_points)
{
  const auto find_cri_dpt = [](const DeparturePoint & point) {
    return point.departure_type == DepartureType::CRITICAL_DEPARTURE;
  };

  auto crit_dpt_finder =
    std::find_if(departure_points.begin(), departure_points.end(), find_cri_dpt);

  if (
    crit_dpt_finder != departure_points.end() &&
    std::next(crit_dpt_finder) != departure_points.end()) {
    departure_points.erase(std::next(crit_dpt_finder), departure_points.end());
  }
}

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

bool is_valid_footprints(
  const autoware::boundary_departure_checker::FootprintMap<
    autoware::boundary_departure_checker::Side<
      autoware::boundary_departure_checker::ProjectionsToBound>> & projections_to_bound,
  const std::vector<autoware::boundary_departure_checker::FootprintType> & footprint_type_order,
  const autoware::boundary_departure_checker::SideKey side_key)
{
  if (footprint_type_order.empty()) {
    return false;
  }

  const auto is_empty = std::any_of(
    footprint_type_order.begin(), footprint_type_order.end(),
    [&projections_to_bound, &side_key](const auto footprint_type) {
      return projections_to_bound[footprint_type][side_key].empty();
    });

  if (is_empty) {
    return false;
  }

  const auto & fr_proj_to_bound = projections_to_bound[footprint_type_order.front()][side_key];
  const auto check_size = [&](const auto footprint_type) {
    return fr_proj_to_bound.size() != projections_to_bound[footprint_type][side_key].size();
  };

  return !std::any_of(
    std::next(footprint_type_order.begin()), footprint_type_order.end(), check_size);
}
}  // namespace

namespace autoware::boundary_departure_checker::utils
{
TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length)
{
  if (trajectory.empty()) {
    return {};
  }

  TrajectoryPoints cut;

  double total_length = 0.0;
  auto last_point = autoware_utils_geometry::from_msg(trajectory.front().pose.position);
  auto end_it = std::next(trajectory.cbegin());
  for (; end_it != trajectory.cend(); ++end_it) {
    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
      break;
    }

    const auto & new_pose = end_it->pose;
    const auto new_point = autoware_utils_geometry::from_msg(new_pose.position);
    const auto points_distance = boost::geometry::distance(last_point.to_2d(), new_point.to_2d());

    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated =
        last_point + remain_distance * (new_point - last_point).normalized();

      TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = new_pose.orientation;

      cut.push_back(p);
      break;
    }

    total_length += points_distance;
    last_point = new_point;
  }
  cut.insert(cut.begin(), trajectory.begin(), end_it);

  return cut;
}

TrajectoryPoints resampleTrajectory(const Trajectory & trajectory, const double interval)
{
  if (trajectory.points.size() < 2) {
    return trajectory.points;
  }

  TrajectoryPoints resampled;

  resampled.push_back(trajectory.points.front());
  auto prev_point = autoware_utils_geometry::from_msg(trajectory.points.front().pose.position);
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & traj_point = trajectory.points.at(i);

    const auto next_point = autoware_utils_geometry::from_msg(traj_point.pose.position);

    if (boost::geometry::distance(prev_point.to_2d(), next_point.to_2d()) >= interval) {
      resampled.push_back(traj_point);
      prev_point = next_point;
    }
  }
  resampled.push_back(trajectory.points.back());

  return resampled;
}

std::vector<LinearRing2d> createVehicleFootprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_margin_scale)
{
  // Calculate longitudinal and lateral margin based on covariance
  const auto margin = utils::calc_margin_from_covariance(covariance, footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory) {
    vehicle_footprints.push_back(
      autoware_utils_geometry::transform_vector(
        local_vehicle_footprint, autoware_utils_geometry::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> createVehicleFootprints(
  const PathWithLaneId & path, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_extra_margin)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(footprint_extra_margin);

  // Create vehicle footprint on each Path point
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : path.points) {
    vehicle_footprints.push_back(
      autoware_utils_geometry::transform_vector(
        local_vehicle_footprint, autoware_utils_geometry::pose2transform(p.point.pose)));
  }

  return vehicle_footprints;
}

lanelet::ConstLanelets getCandidateLanelets(
  const lanelet::ConstLanelets & route_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  lanelet::ConstLanelets candidate_lanelets;

  // Find lanes within the convex hull of footprints
  const auto footprint_hull = createHullFromFootprints(vehicle_footprints);

  for (const auto & route_lanelet : route_lanelets) {
    const auto poly = route_lanelet.polygon2d().basicPolygon();
    if (!boost::geometry::disjoint(poly, footprint_hull)) {
      candidate_lanelets.push_back(route_lanelet);
    }
  }

  return candidate_lanelets;
}

LinearRing2d createHullFromFootprints(const std::vector<LinearRing2d> & footprints)
{
  MultiPoint2d combined;
  for (const auto & footprint : footprints) {
    for (const auto & p : footprint) {
      combined.push_back(p);
    }
  }

  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);

  return hull;
}

std::vector<LinearRing2d> createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  if (vehicle_footprints.empty()) {
    return {};
  }

  if (vehicle_footprints.size() == 1) {
    return {vehicle_footprints.front()};
  }

  std::vector<LinearRing2d> areas;
  areas.reserve(vehicle_footprints.size() - 1);

  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(createHullFromFootprints({footprint1, footprint2}));
  }

  return areas;
}

double calcMaxSearchLengthForBoundaries(
  const Trajectory & trajectory, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const double max_ego_lon_length = std::max(
    std::abs(vehicle_info.max_longitudinal_offset_m),
    std::abs(vehicle_info.min_longitudinal_offset_m));
  const double max_ego_lat_length = std::max(
    std::abs(vehicle_info.max_lateral_offset_m), std::abs(vehicle_info.min_lateral_offset_m));
  const double max_ego_search_length = std::hypot(max_ego_lon_length, max_ego_lat_length);
  return autoware::motion_utils::calcArcLength(trajectory.points) + max_ego_search_length;
}

FootprintMargin calc_margin_from_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto cov_in_map = covariance.covariance;
  Eigen::Matrix2d cov_xy_map;
  cov_xy_map << cov_in_map[0 * 6 + 0], cov_in_map[0 * 6 + 1], cov_in_map[1 * 6 + 0],
    cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d r_map2vehicle;
  r_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d cov_xy_vehicle = r_map2vehicle * cov_xy_map * r_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in cov_xy_vehicle(0,0), cov_xy_vehicle(1,1) respectively.
  return FootprintMargin{cov_xy_vehicle(0, 0) * scale, cov_xy_vehicle(1, 1) * scale};
}

std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const FootprintMargin & uncertainty_fp_margin, const LongitudinalConfig & longitudinal_config)
{
  std::vector<LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.size());
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const TrajectoryPoint & p) -> LinearRing2d {
      using autoware_utils_geometry::transform_vector;
      using autoware_utils_geometry::pose2transform;

      auto margin = uncertainty_fp_margin;
      const auto & lon_tracking = longitudinal_config.lon_tracking;
      margin.lon_m +=
        (p.longitudinal_velocity_mps * lon_tracking.scale) + lon_tracking.extra_margin_m;
      const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);
      return transform_vector(local_vehicle_footprint, pose2transform(p.pose));
    });

  return vehicle_footprints;
}

std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const FootprintMargin & margin)
{
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);

  std::vector<LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.size());
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const auto & p) -> LinearRing2d {
      using autoware_utils_geometry::transform_vector;
      using autoware_utils_geometry::pose2transform;
      return transform_vector(local_vehicle_footprint, pose2transform(p.pose));
    });

  return vehicle_footprints;
}

Side<Segment2d> get_footprint_sides(
  const LinearRing2d & footprint, const bool use_center_right, const bool use_center_left)
{
  const auto center_right = use_center_right ? 2 : 3;
  const auto center_left = use_center_left ? 5 : 4;

  const auto & right_front = footprint[1];
  const auto & right_back = footprint[center_right];

  const auto & left_front = footprint[6];
  const auto & left_back = footprint[center_left];

  Side<Segment2d> side;
  side.right = {Point2d(right_front.x(), right_front.y()), Point2d(right_back.x(), right_back.y())};
  side.left = {Point2d(left_front.x(), left_front.y()), Point2d(left_back.x(), left_back.y())};

  return side;
}

EgoSide get_ego_side_from_footprint(
  const Footprint & footprint, const bool use_center_right, const bool use_center_left)
{
  auto fp_side = get_footprint_sides(footprint, use_center_right, use_center_left);
  EgoSide ego_side;
  ego_side.left = std::move(fp_side.left);
  ego_side.right = std::move(fp_side.right);
  return ego_side;
}

EgoSides get_sides_from_footprints(
  const Footprints & footprints, const bool use_center_right, const bool use_center_left)
{
  EgoSides footprints_sides;
  footprints_sides.reserve(footprints.size());
  for (const auto & footprint : footprints) {
    auto ego_side = get_ego_side_from_footprint(footprint, use_center_right, use_center_left);
    footprints_sides.push_back(ego_side);
  }

  return footprints_sides;
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

UncrossableBoundRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect)
{
  std::vector<SegmentWithIdx> segments;
  segments.reserve(lanelet_map.lineStringLayer.size());
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

tl::expected<ProjectionToBound, std::string> segment_to_segment_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t ego_sides_idx)
{
  const auto & [ego_f, ego_b] = ego_seg;
  const auto & [lane_pt1, lane_pt2] = lane_seg;

  if (
    const auto is_intersecting = autoware_utils_geometry::intersect(
      to_geom_pt(ego_f), to_geom_pt(ego_b), to_geom_pt(lane_pt1), to_geom_pt(lane_pt2))) {
    Point2d point(is_intersecting->x, is_intersecting->y);
    return ProjectionToBound{
      point, point, lane_seg, 0.0, boost::geometry::distance(point, ego_f), ego_sides_idx};
  }

  ProjectionsToBound projections;
  projections.reserve(4);
  if (const auto projection_opt = point_to_segment_projection(ego_f, lane_seg)) {
    const auto & [proj, dist] = *projection_opt;
    constexpr auto lon_offset = 0.0;
    projections.emplace_back(ego_f, proj, lane_seg, dist, lon_offset, ego_sides_idx);
  }

  if (const auto projection_opt = point_to_segment_projection(ego_b, lane_seg)) {
    const auto & [proj, dist] = *projection_opt;
    const auto lon_offset = boost::geometry::distance(ego_b, ego_f);
    projections.emplace_back(ego_b, proj, lane_seg, dist, lon_offset, ego_sides_idx);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt1, ego_seg)) {
    const auto & [proj, dist] = *projection_opt;
    const auto lon_offset = boost::geometry::distance(proj, ego_f);
    projections.emplace_back(proj, lane_pt1, lane_seg, dist, lon_offset, ego_sides_idx);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt2, ego_seg)) {
    const auto & [proj, dist] = *projection_opt;
    const auto lon_offset = boost::geometry::distance(proj, ego_f);
    projections.emplace_back(proj, lane_pt2, lane_seg, dist, lon_offset, ego_sides_idx);
  }

  if (projections.empty())
    return tl::make_unexpected("Couldn't generate projection at " + std::to_string(ego_sides_idx));
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
    if (
      const auto proj_opt = segment_to_segment_nearest_projection(ego_side_seg, seg, curr_fp_idx)) {
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
  const TrajectoryPoints & ego_pred_traj, const BoundarySideWithIdx & boundaries,
  const EgoSides & ego_sides_from_footprints)
{
  Side<ProjectionsToBound> side;
  std::unordered_map<SideKey, bool> has_passed_border;
  for (const auto & side_key : g_side_keys) {
    side[side_key].reserve(ego_sides_from_footprints.size());
    has_passed_border[side_key] = false;
  }

  auto s = 0.0;
  for (size_t i = 0; i < ego_pred_traj.size(); ++i) {
    if (i > 0) {
      s += autoware_utils_geometry::calc_distance2d(ego_pred_traj[i - 1], ego_pred_traj[i]);
    }

    const auto & fp = ego_sides_from_footprints[i];

    const auto & ego_lb = fp.left.second;
    const auto & ego_rb = fp.right.second;

    const auto rear_seg = Segment2d(ego_lb, ego_rb);

    for (const auto & side_key : g_side_keys) {
      auto closest_bound = find_closest_segment(fp[side_key], rear_seg, i, boundaries[side_key]);

      // Assign negative sign if the boundary has been crossed
      if (
        closest_bound.lat_dist > 0.0 &&
        closest_bound.lat_dist < std::numeric_limits<double>::max() &&
        has_passed_border[side_key]) {
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

        // If cross_prod < 0, boundary is to the RIGHT. If > 0, boundary is to the LEFT.
        const bool crossed_left = (side_key == SideKey::LEFT && cross_prod < 0.0);
        const bool crossed_right = (side_key == SideKey::RIGHT && cross_prod > 0.0);
        if (crossed_left || crossed_right) {
          closest_bound.lat_dist = -closest_bound.lat_dist;
        }
      }

      closest_bound.time_from_start = rclcpp::Duration(ego_pred_traj[i].time_from_start).seconds();
      closest_bound.lon_dist_on_pred_traj = s - closest_bound.lon_offset;
      side[side_key].push_back(closest_bound);
      if (closest_bound.lat_dist < 0.01 && !has_passed_border[side_key]) {
        has_passed_border[side_key] = true;
      }
    }
  }

  return side;
}

DeparturePoints cluster_by_distance(const DeparturePoints & departure_points)
{
  DeparturePoints filtered_points;
  filtered_points.reserve(departure_points.size());
  auto ref_point_it = departure_points.begin();
  filtered_points.push_back(*ref_point_it);
  for (auto it = std::next(departure_points.begin()); it < departure_points.end(); ++it) {
    if (
      it->departure_type == DepartureType::CRITICAL_DEPARTURE ||
      std::abs(ref_point_it->ego_dist_on_ref_traj - it->ego_dist_on_ref_traj) >
        ref_point_it->th_point_merge_distance_m) {
      ref_point_it = it;
      filtered_points.push_back(*ref_point_it);
    }

    if (ref_point_it->departure_type == DepartureType::CRITICAL_DEPARTURE) {
      continue;
    }

    if (
      ref_point_it->departure_type == DepartureType::APPROACHING_DEPARTURE ||
      it->departure_type == DepartureType::APPROACHING_DEPARTURE) {
      filtered_points.back().departure_type = DepartureType::APPROACHING_DEPARTURE;
    }
  }
  return filtered_points;
}

DeparturePoints get_departure_points(
  const ProjectionsToBound & projections_to_bound,
  const std::vector<double> & pred_traj_idx_to_ref_traj_lon_dist,
  const double th_point_merge_distance_m)
{
  DeparturePoints departure_points;
  departure_points.reserve(projections_to_bound.size());
  for (const auto & projection_to_bound : projections_to_bound) {
    const auto point = create_departure_point(
      projection_to_bound, pred_traj_idx_to_ref_traj_lon_dist, th_point_merge_distance_m);

    if (point.can_be_removed) {
      continue;
    }
    departure_points.push_back(point);
  }

  std::sort(departure_points.begin(), departure_points.end());
  erase_after_first_match(departure_points);

  if (departure_points.empty()) {
    return departure_points;
  }

  return cluster_by_distance(departure_points);
}

tl::expected<std::vector<lanelet::LineString3d>, std::string> get_uncrossable_linestrings_near_pose(
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const Pose & ego_pose,
  const double search_distance, const std::vector<std::string> & uncrossable_boundary_types)
{
  if (!lanelet_map_ptr) {
    return tl::make_unexpected("lanelet_map_ptr is null");
  }

  if (search_distance < 0.0) {
    return tl::make_unexpected("Search distance must be non-negative.");
  }

  const auto p = lanelet::BasicPoint2d(ego_pose.position.x, ego_pose.position.y);

  if (!std::isfinite(p.x()) || !std::isfinite(p.y())) {
    return tl::make_unexpected("ego_pose contains non-finite values.");
  }

  const auto offset = lanelet::BasicPoint2d(search_distance, search_distance);
  auto bbox = lanelet::BoundingBox2d(p - offset, p + offset);

  auto nearby_linestrings = lanelet_map_ptr->lineStringLayer.search(bbox);

  const auto remove_itr = std::remove_if(
    nearby_linestrings.begin(), nearby_linestrings.end(),
    [&](const auto & ls) { return !is_uncrossable_type(uncrossable_boundary_types, ls); });

  nearby_linestrings.erase(remove_itr, nearby_linestrings.end());

  if (nearby_linestrings.empty()) {
    return tl::make_unexpected(
      "No nearby uncrossable boundaries within " + std::to_string(search_distance) + " meter.");
  }

  return nearby_linestrings;
}

TrajectoryPoints trim_pred_path(const TrajectoryPoints & ego_pred_traj, const double cutoff_time_s)
{
  TrajectoryPoints trimmed_traj;
  trimmed_traj.reserve(ego_pred_traj.size());
  for (const auto & p : ego_pred_traj) {
    trimmed_traj.push_back(p);
    if (rclcpp::Duration(p.time_from_start).seconds() > cutoff_time_s) {
      break;
    }
  }
  return trimmed_traj;
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

std::optional<std::pair<double, double>> is_point_shifted(
  const autoware::boundary_departure_checker::Pose & prev_iter_pt,
  const autoware::boundary_departure_checker::Pose & curr_iter_pt, const double th_shift_m,
  const double th_yaw_diff_rad)
{
  const auto curr_pt_yaw_rad = tf2::getYaw(curr_iter_pt.orientation);
  const auto prev_pt_yaw_rad = tf2::getYaw(prev_iter_pt.orientation);
  const auto yaw_diff_rad = std::abs(curr_pt_yaw_rad - prev_pt_yaw_rad);

  const auto dist_m =
    autoware_utils_geometry::calc_distance2d(curr_iter_pt.position, prev_iter_pt.position);
  if (dist_m > th_shift_m || yaw_diff_rad > th_yaw_diff_rad) {
    return std::make_pair(dist_m, yaw_diff_rad);
  }
  return std::nullopt;
}

std::optional<ProjectionsToBound> get_closest_projections_for_side(
  const FootprintMap<Side<ProjectionsToBound>> & projections_to_bound, const Param & param,
  const double min_braking_dist, const double max_braking_dist, const SideKey side_key)
{
  const auto & footprint_type_order = param.footprint_types_to_check;

  if (!is_valid_footprints(projections_to_bound, footprint_type_order, side_key)) {
    return std::nullopt;
  }

  const auto & fr_proj_to_bound = projections_to_bound[footprint_type_order.front()][side_key];

  ProjectionsToBound min_to_bound;
  const auto fp_size = fr_proj_to_bound.size();
  min_to_bound.reserve(fp_size);

  for (size_t idx = 0; idx < fp_size; ++idx) {
    std::vector<ProjectionToBound> candidate_projections;
    candidate_projections.reserve(footprint_type_order.size());

    for (const auto footprint_type : footprint_type_order) {
      auto candidate = projections_to_bound[footprint_type][side_key][idx];
      if (candidate.ego_sides_idx != idx) continue;

      candidate.footprint_type_opt = footprint_type;
      candidate_projections.push_back(candidate);
    }

    std::optional<double> previous_longitudinal_distance =
      min_to_bound.empty() ? std::nullopt
                           : std::make_optional(min_to_bound.back().lon_dist_on_pred_traj);

    auto closest_projection = get_closest_projection_by_departure_severity(
      candidate_projections, param, min_braking_dist, max_braking_dist, side_key,
      previous_longitudinal_distance);

    if (closest_projection) {
      min_to_bound.push_back(*closest_projection);
      if (closest_projection->is_critical_departure()) {
        break;
      }
    }
  }

  if (!min_to_bound.empty() && min_to_bound.back().is_critical_departure()) {
    const double arc_length_to_departure =
      min_to_bound.back().lon_dist_on_pred_traj - min_to_bound.back().lon_offset;
    constexpr double longitudinal_buffer_m = 1.0;

    auto earliest_critical_it = min_to_bound.end() - 1;

    for (auto itr = min_to_bound.rbegin(); itr != min_to_bound.rend(); ++itr) {
      if (itr->footprint_type_opt == FootprintType::NORMAL) {
        const double dist_to_departure = arc_length_to_departure - itr->lon_dist_on_pred_traj;

        if (dist_to_departure <= longitudinal_buffer_m) {
          itr->departure_type_opt = DepartureType::CRITICAL_DEPARTURE;
          earliest_critical_it = itr.base() - 1;
        } else if (dist_to_departure <= max_braking_dist + longitudinal_buffer_m) {
          itr->departure_type_opt = DepartureType::APPROACHING_DEPARTURE;
        }
      }
    }

    // Erase the physical departure points, keeping ONLY the newly buffered critical point
    if (earliest_critical_it != min_to_bound.end() - 1) {
      min_to_bound.erase(earliest_critical_it + 1, min_to_bound.end());
    }
  }

  // NEW STEP 3: Sweep up! Remove any points that are still NONE
  // (These are the safe points we temporarily kept to ensure the backward loop had physical
  // indices)
  min_to_bound.erase(
    std::remove_if(
      min_to_bound.begin(), min_to_bound.end(),
      [](const ProjectionToBound & p) { return p.departure_type_opt == DepartureType::NONE; }),
    min_to_bound.end());

  return min_to_bound;
}

std::optional<ProjectionToBound> get_closest_projection_by_departure_severity(
  const std::vector<ProjectionToBound> & candidate_projections, const Param & param,
  const double min_braking_dist, const double max_braking_dist, const SideKey side_key,
  const std::optional<double> previous_longitudinal_distance)
{
  std::optional<ProjectionToBound> best_projection;
  std::optional<ProjectionToBound> fallback_normal;  // Store the skeleton!

  const double th_dist_critical = param.th_trigger.th_dist_to_boundary_m[side_key].min;
  const double th_dist_near = param.th_trigger.th_dist_to_boundary_m[side_key].max;

  for (auto candidate : candidate_projections) {
    if (!candidate.footprint_type_opt) continue;

    // Save the NORMAL footprint to act as our physical skeleton
    if (candidate.footprint_type_opt.value() == FootprintType::NORMAL) {
      fallback_normal = candidate;
    }

    candidate.departure_type_opt = assign_departure_type(
      candidate.lon_dist_on_pred_traj, candidate.lon_offset, min_braking_dist, max_braking_dist,
      param.th_cutoff_time_departure_s, candidate.time_from_start, candidate.lat_dist, th_dist_near,
      th_dist_critical, candidate.footprint_type_opt.value());

    if (candidate.departure_type_opt.value() == DepartureType::NONE) {
      continue;
    }

    if (!best_projection) {
      best_projection = candidate;
    } else {
      const auto cand_severity = candidate.departure_type_opt.value();
      const auto best_severity = best_projection->departure_type_opt.value();

      if (cand_severity > best_severity) {
        best_projection = candidate;
      } else if (cand_severity == best_severity) {
        if (candidate.lat_dist < best_projection->lat_dist) {
          best_projection = candidate;
        }
      }
    }

    if (best_projection->is_critical_departure()) {
      break;
    }
  }

  // If no threats were found, return the safe NORMAL skeleton as NONE!
  if (!best_projection) {
    if (fallback_normal) {
      fallback_normal->departure_type_opt = DepartureType::NONE;
      best_projection = fallback_normal;
    } else {
      return std::nullopt;
    }
  }

  // Downsampling logic (Bypassed if CRITICAL)
  if (
    previous_longitudinal_distance && !best_projection->is_critical_departure() &&
    std::abs(*previous_longitudinal_distance - best_projection->lon_dist_on_pred_traj) <=
      param.th_point_merge_distance_m) {
    return std::nullopt;
  }

  return best_projection;
}

DepartureType assign_departure_type(
  const double longitudinal_distance_to_departure_point,
  const double longitudinal_offset_to_departure_point, const double minimum_braking_distance,
  const double maximum_braking_distance, const double cutoff_time, const double time_from_start,
  const double lat_dist, const double th_lat_near, const double th_lat_critical,
  const FootprintType footprint_type)
{
  if (lat_dist > th_lat_near) {
    return DepartureType::NONE;
  }

  const double dist_to_departure_point_with_offset =
    longitudinal_distance_to_departure_point -
    longitudinal_offset_to_departure_point;  // shorter distance

  if (footprint_type == FootprintType::NORMAL && lat_dist <= th_lat_critical) {
    if (
      dist_to_departure_point_with_offset > minimum_braking_distance &&
      time_from_start > cutoff_time) {
      return DepartureType::APPROACHING_DEPARTURE;
    }
    // Set CRITICAL if:
    // - Short Dist & Short Time: boundary crossing is less than braking distance and we will hit it
    // in less than cutoff time.
    // - Long Dist but Short Time: At 100 km/h, the boundary crossing is 30 meters away, but ego
    // will hit the crossing in less than cutoff time.
    // - Long time, but dist less than braking: Creeping forward in a parking lot at 2 km/h, and it
    // takes it will 4 seconds to reach it, however, the boundary less than minimum braking
    // distance.
    return DepartureType::CRITICAL_DEPARTURE;
  }

  if (dist_to_departure_point_with_offset < maximum_braking_distance) {
    return DepartureType::NEAR_BOUNDARY;
  }

  return DepartureType::NONE;
}
}  // namespace autoware::boundary_departure_checker::utils
