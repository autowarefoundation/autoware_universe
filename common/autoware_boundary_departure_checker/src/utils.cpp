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
#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <range/v3/view.hpp>
#include <tl_expected/expected.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <cstddef>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using autoware::boundary_departure_checker::ClosestProjectionsToBound;
using autoware::boundary_departure_checker::ClosestProjectionToBound;
using autoware::boundary_departure_checker::DeparturePoint;
using autoware::boundary_departure_checker::DeparturePoints;
using autoware::boundary_departure_checker::DepartureType;
using autoware::boundary_departure_checker::IdxForRTreeSegment;
using autoware::boundary_departure_checker::Segment2d;
using autoware::boundary_departure_checker::SegmentWithIdx;
using autoware::boundary_departure_checker::VehicleInfo;
using autoware::boundary_departure_checker::utils::to_segment_2d;
namespace bg = boost::geometry;

DeparturePoint create_departure_point(
  const ClosestProjectionToBound & projection_to_bound, const double th_dist_hysteresis_m,
  const double lon_offset_m)
{
  DeparturePoint point;
  point.uuid = autoware_utils::to_hex_string(autoware_utils::generate_uuid());
  point.lat_dist_to_bound = projection_to_bound.lat_dist;
  point.departure_type = projection_to_bound.departure_type;
  point.point = projection_to_bound.pt_on_bound;
  point.th_dist_hysteresis = th_dist_hysteresis_m;
  point.dist_on_traj = projection_to_bound.lon_dist_on_ref_traj - lon_offset_m;
  point.idx_from_ego_traj = projection_to_bound.ego_sides_idx;
  point.can_be_removed = (point.departure_type == DepartureType::NONE) || point.dist_on_traj <= 0.0;
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
}  // namespace

namespace autoware::boundary_departure_checker::utils
{
double calc_dist_on_traj(
  const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const Point2d & point)
{
  return trajectory::closest(aw_ref_traj, to_geom_pt(point));
}

TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length)
{
  if (trajectory.empty()) {
    return {};
  }

  TrajectoryPoints cut;

  double total_length = 0.0;
  auto last_point = autoware_utils::from_msg(trajectory.front().pose.position);
  auto end_it = std::next(trajectory.cbegin());
  for (; end_it != trajectory.cend(); ++end_it) {
    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0) {
      break;
    }

    const auto & new_pose = end_it->pose;
    const auto new_point = autoware_utils::from_msg(new_pose.position);
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
  auto prev_point = autoware_utils::from_msg(trajectory.points.front().pose.position);
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & traj_point = trajectory.points.at(i);

    const auto next_point = autoware_utils::from_msg(traj_point.pose.position);

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
    vehicle_footprints.push_back(autoware_utils::transform_vector(
      local_vehicle_footprint, autoware_utils::pose2transform(p.pose)));
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
    vehicle_footprints.push_back(autoware_utils::transform_vector(
      local_vehicle_footprint, autoware_utils::pose2transform(p.point.pose)));
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
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;

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
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      return transform_vector(local_vehicle_footprint, pose2transform(p.pose));
    });

  return vehicle_footprints;
}

std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const SteeringReport & current_steering)
{
  constexpr auto steering_rate_gain = 1.0;
  constexpr auto steering_rate_rad_per_s = 0.25;

  std::vector<LinearRing2d> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.size());
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const TrajectoryPoint & p) -> LinearRing2d {
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      const double raw_angle_rad =
        current_steering.steering_tire_angle +
        (steering_rate_rad_per_s * rclcpp::Duration(p.time_from_start).seconds());

      constexpr auto min_angle = autoware_utils::deg2rad(-89);
      constexpr auto max_angle = autoware_utils::deg2rad(89);
      const double clamped_angle_rad = std::clamp(raw_angle_rad, min_angle, max_angle);

      const auto local_vehicle_footprint = vehicle_info.createFootprint(
        std::max(std::tan(clamped_angle_rad) * steering_rate_gain, 0.0), 0.0, 0.0, 0.0, 0.0, true);
      return transform_vector(local_vehicle_footprint, pose2transform(p.pose));
    });

  return vehicle_footprints;
}

std::vector<LinearRing2d> create_ego_footprints(
  const AbnormalityType abnormality_type, const FootprintMargin & uncertainty_fp_margin,
  const TrajectoryPoints & ego_pred_traj, const SteeringReport & current_steering,
  const VehicleInfo & vehicle_info, const Param & param)
{
  if (abnormality_type == AbnormalityType::LONGITUDINAL) {
    const auto longitudinal_config_opt =
      param.get_abnormality_config<LongitudinalConfig>(abnormality_type);
    return create_vehicle_footprints(
      ego_pred_traj, vehicle_info, uncertainty_fp_margin, longitudinal_config_opt->get());
  }

  if (abnormality_type == AbnormalityType::STEERING) {
    return utils::create_vehicle_footprints(ego_pred_traj, vehicle_info, current_steering);
  }

  FootprintMargin margin = uncertainty_fp_margin;
  if (abnormality_type == AbnormalityType::LOCALIZATION) {
    const auto loc_config_opt = param.get_abnormality_config<LocalizationConfig>(abnormality_type);
    const auto & footprint_envelop = loc_config_opt->get().footprint_envelop;
    margin = margin + footprint_envelop;
  }
  return utils::create_vehicle_footprints(ego_pred_traj, vehicle_info, margin);
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

tl::expected<std::tuple<Point2d, Point2d, double>, std::string> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment, const bool swap_points)
{
  const auto & p1 = segment.first;
  const auto & p2 = segment.second;

  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto result = [&swap_points](
                        const Point2d & orig,
                        const Point2d & proj) -> std::tuple<Point2d, Point2d, double> {
    return swap_points ? std::make_tuple(proj, orig, boost::geometry::distance(proj, orig))
                       : std::make_tuple(orig, proj, boost::geometry::distance(orig, proj));
  };

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 < 0.0) return tl::make_unexpected("Point before segment start");
  if (c1 == 0.0) return result(p, p1);

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c1 > c2) return tl::make_unexpected("Point after segment end");

  if (c1 == c2) return result(p, p2);

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};

  return result(p, projection_point);
}

tl::expected<ProjectionToBound, std::string> segment_to_segment_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t ego_sides_idx)
{
  const auto & [ego_f, ego_b] = ego_seg;
  const auto & [lane_pt1, lane_pt2] = lane_seg;

  if (
    const auto is_intersecting = autoware_utils::intersect(
      to_geom_pt(ego_f), to_geom_pt(ego_b), to_geom_pt(lane_pt1), to_geom_pt(lane_pt2))) {
    Point2d point(is_intersecting->x, is_intersecting->y);
    return ProjectionToBound{point, point, lane_seg, 0.0, ego_sides_idx};
  }

  std::vector<ProjectionToBound> projections;
  projections.reserve(4);
  constexpr bool swap_result = true;
  if (const auto projection_opt = point_to_segment_projection(ego_f, lane_seg, swap_result)) {
    const auto & [pt_ego, pt_lane, dist] = *projection_opt;
    projections.emplace_back(pt_ego, pt_lane, lane_seg, dist, ego_sides_idx);
  }

  if (const auto projection_opt = point_to_segment_projection(ego_b, lane_seg, swap_result)) {
    const auto & [pt_ego, pt_lane, dist] = *projection_opt;
    projections.emplace_back(pt_ego, pt_lane, lane_seg, dist, ego_sides_idx);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt1, ego_seg, !swap_result)) {
    const auto & [pt_ego, pt_lane, dist] = *projection_opt;
    projections.emplace_back(pt_ego, pt_lane, lane_seg, dist, ego_sides_idx);
  }

  if (const auto projection_opt = point_to_segment_projection(lane_pt2, ego_seg, !swap_result)) {
    const auto & [pt_ego, pt_lane, dist] = *projection_opt;
    projections.emplace_back(pt_ego, pt_lane, lane_seg, dist, ego_sides_idx);
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
      const auto is_intersecting_rear = autoware_utils::intersect(
        to_geom_pt(ego_lr), to_geom_pt(ego_rr), to_geom_pt(seg_f), to_geom_pt(seg_r))) {
      Point2d point(is_intersecting_rear->x, is_intersecting_rear->y);
      closest_proj = ProjectionToBound{point, point, seg, 0.0, curr_fp_idx};
      break;
    }
  }

  if (closest_proj) {
    return *closest_proj;
  }

  return ProjectionToBound(curr_fp_idx);
}

ProjectionsToBound get_closest_boundary_segments_from_side(
  const BoundarySideWithIdx & boundaries, const EgoSides & ego_sides_from_footprints)
{
  ProjectionsToBound side;
  for (const auto & side_key : g_side_keys) {
    side[side_key].reserve(ego_sides_from_footprints.size());
  }

  for (size_t i = 0; i < ego_sides_from_footprints.size(); ++i) {
    const auto & fp = ego_sides_from_footprints[i];

    const auto & ego_lb = fp.left.second;
    const auto & ego_rb = fp.right.second;

    const auto rear_seg = Segment2d(ego_lb, ego_rb);

    for (const auto & side_key : g_side_keys) {
      const auto closest_bound =
        find_closest_segment(fp[side_key], rear_seg, i, boundaries[side_key]);
      side[side_key].push_back(closest_bound);
    }
  }

  return side;
}

double compute_braking_distance(
  double v_init, double v_end, double acc, double jerk, double t_braking_delay)
{
  // Phase 1: jerk phase
  const double t1 = acc / jerk;
  const double d1 = std::max(v_init * t1 - (acc / 6.0) * t1 * t1, 0.0);

  // Midpoint velocity after jerk phase
  const double v_mid = std::max(v_init - (acc / 2.0) * t1, 0.0);

  // Phase 2: constant deceleration
  const double dv2 = std::max(v_mid - v_end, 0.0);
  const double t2 = dv2 / acc;
  const double d2 = ((v_mid + v_end) / 2.0) * t2;

  return d1 + d2 + v_init * t_braking_delay;
}

DeparturePoints get_departure_points(
  const std::vector<ClosestProjectionToBound> & projections_to_bound,
  const double th_dist_hysteresis_m, const double lon_offset_m)
{
  DeparturePoints departure_points;
  departure_points.reserve(projections_to_bound.size());
  for (const auto & projection_to_bound : projections_to_bound) {
    const auto point =
      create_departure_point(projection_to_bound, th_dist_hysteresis_m, lon_offset_m);

    if (point.can_be_removed) {
      continue;
    }

    departure_points.push_back(point);
  }

  std::sort(departure_points.begin(), departure_points.end());
  erase_after_first_match(departure_points);
  return departure_points;
}
}  // namespace autoware::boundary_departure_checker::utils
