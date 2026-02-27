// Copyright 2025 Tier IV, Inc. All rights reserved.
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

#include "utils.hpp"

#include "../risk_predictive_braking.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <deque>
#include <limits>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace utils
{
namespace
{
bool isBlockedByPartition(const LineString2d & direction, const BasicPolygons2d & partitions)
{
  return std::any_of(partitions.begin(), partitions.end(), [&](const lanelet::BasicPolygon2d & p) {
    return bg::intersects(direction, p);
  });
}

lanelet::BasicLineString2d to_line_string_2d(const Trajectory & trajectory)
{
  lanelet::BasicLineString2d line_string;
  for (const auto & point : trajectory.restore()) {
    const auto & p = point.point.pose.position;
    line_string.push_back(autoware::experimental::lanelet2_utils::from_ros(p).basicPoint2d());
  }
  return line_string;
}

double calcSignedLateralDistanceWithOffset(
  const double lateral, const double right_overhang, const double left_overhang,
  const double wheel_tread)
{
  const auto offset_left = left_overhang + wheel_tread / 2;
  const auto offset_right = right_overhang + wheel_tread / 2;
  if (lateral > 0) {
    return std::max(lateral - offset_left, 0.0);
  }
  return std::min(lateral + offset_right, 0.0);
}

lanelet::ArcCoordinates getOcclusionPoint(
  const PredictedObject & obj, const lanelet::BasicLineString2d & line_string)
{
  const auto poly = autoware_utils::to_polygon2d(obj);
  std::deque<lanelet::ArcCoordinates> arcs;
  for (const auto & p : poly.outer()) {
    arcs.emplace_back(lanelet::geometry::toArcCoordinates(line_string, {p.x(), p.y()}));
  }

  /** remove
   *  x--------*
   *  |        |
   *  x--------* <- return
   * Ego===============> path
   **/
  // sort by arc length
  std::sort(
    arcs.begin(), arcs.end(),
    [](const lanelet::ArcCoordinates & arc1, const lanelet::ArcCoordinates & arc2) {
      return arc1.length < arc2.length;
    });

  // remove closest 2 polygon point
  arcs.pop_front();
  arcs.pop_front();

  // sort by arc distance
  std::sort(
    arcs.begin(), arcs.end(),
    [](const lanelet::ArcCoordinates & arc1, const lanelet::ArcCoordinates & arc2) {
      return std::abs(arc1.distance) < std::abs(arc2.distance);
    });

  // return closest to path point which is choosen by farthest 2 points.
  return arcs.front();
}

/**
 * @brief calculate obstacle collision intersection from arc coordinate info.
 *                                      ^
 *                                      |
 * Ego ---------collision----------intersection-------> path
 *                                      |
 *             ------------------       |
 *            |     Vehicle      |   obstacle
 *             ------------------
 */
occlusion_spot_utils::PossibleCollisionInfo calculateCollisionPathPointFromOcclusionSpot(
  const lanelet::ArcCoordinates & arc_coord_occlusion,
  const lanelet::ArcCoordinates & arc_coord_occlusion_with_offset,
  const lanelet::BasicLineString2d & path_line_string,
  const occlusion_spot_utils::PlannerParam & param)
{
  constexpr auto calcPosition =
    [](const lanelet::BasicLineString2d & line_string, const lanelet::ArcCoordinates & arc) {
      const auto bp = lanelet::geometry::fromArcCoordinates(line_string, arc);
      Point position;
      position.x = bp[0];
      position.y = bp[1];
      position.z = 0.0;
      return position;
    };

  occlusion_spot_utils::PossibleCollisionInfo pc;
  // ttv: time to vehicle for pedestrian
  // ttc: time to collision for ego vehicle
  const auto ttv = std::abs(arc_coord_occlusion_with_offset.distance / param.pedestrian_vel);
  const auto sm = calculateSafeMotion(param.v, ttv);

  // avoid inserting path point behind original path
  constexpr auto eps = 0.1;
  const auto distance_to_stop =
    std::max(eps, arc_coord_occlusion_with_offset.length - sm.stop_dist);

  pc.arc_lane_dist_at_collision = {distance_to_stop, arc_coord_occlusion_with_offset.distance};
  pc.obstacle_info.safe_motion = sm;
  pc.obstacle_info.ttv = ttv;

  pc.obstacle_info.position = calcPosition(path_line_string, arc_coord_occlusion);
  pc.obstacle_info.max_velocity = param.pedestrian_vel;
  pc.collision_pose.position =
    calcPosition(path_line_string, {arc_coord_occlusion_with_offset.length, 0.0});
  pc.collision_with_margin.pose.position = calcPosition(path_line_string, {distance_to_stop, 0.0});
  pc.intersection_pose.position = calcPosition(path_line_string, {arc_coord_occlusion.length, 0.0});

  return pc;
}

std::optional<occlusion_spot_utils::PossibleCollisionInfo>
generateOneNotableCollisionFromOcclusionSpot(
  const grid_map::GridMap & grid, const std::vector<grid_map::Position> & occlusion_spot_positions,
  const double offset_from_start_to_ego, const Point2d base_point, const Trajectory & path,
  const occlusion_spot_utils::PlannerParam & param,
  const occlusion_spot_utils::DebugData & debug_data)
{
  const auto path_line_string = to_line_string_2d(path);

  const auto & baselink_to_front = param.baselink_to_front;
  const auto & right_overhang = param.right_overhang;
  const auto & left_overhang = param.left_overhang;
  const auto & wheel_tread = param.wheel_tread;
  auto distance_lower_bound = std::numeric_limits<double>::max();

  occlusion_spot_utils::PossibleCollisionInfo candidate;
  auto has_collision = false;
  const auto & partition_lanelets = debug_data.close_partition;

  for (const auto & occlusion_spot_position : occlusion_spot_positions) {
    // arc intersection
    const lanelet::BasicPoint2d obstacle_point{
      occlusion_spot_position[0], occlusion_spot_position[1]};
    const auto dist =
      std::hypot(base_point.x() - obstacle_point[0], base_point.y() - obstacle_point[1]);

    // skip if absolute distance is larger
    if (distance_lower_bound < dist) {
      continue;
    }

    auto arc_coord_occlusion_point =
      lanelet::geometry::toArcCoordinates(path_line_string, obstacle_point);
    const auto length_to_col = arc_coord_occlusion_point.length - baselink_to_front;

    // skip if occlusion is behind ego bumper
    if (length_to_col < offset_from_start_to_ego) {
      continue;
    }

    const lanelet::ArcCoordinates arc_coord_collision_point{
      length_to_col,
      calcSignedLateralDistanceWithOffset(
        arc_coord_occlusion_point.distance, right_overhang, left_overhang, wheel_tread)};

    const auto pc = calculateCollisionPathPointFromOcclusionSpot(
      arc_coord_occlusion_point, arc_coord_collision_point, path_line_string, param);
    const auto & ip = pc.intersection_pose.position;

    if (param.use_partition_lanelet) {
      const auto & op = obstacle_point;
      const LineString2d obstacle_vec{{op[0], op[1]}, {ip.x, ip.y}};
      if (isBlockedByPartition(obstacle_vec, partition_lanelets)) {
        continue;
      }
    }

    if (!grid_utils::isCollisionFree(
          grid, occlusion_spot_position, grid_map::Position(ip.x, ip.y), param.pedestrian_radius)) {
      continue;
    }

    distance_lower_bound = dist;
    candidate = pc;
    has_collision = true;
  }

  if (!has_collision) {
    return std::nullopt;
  }
  return candidate;
}
}  // namespace

void applyVelocityToPath(Trajectory & path, const double velocity)
{
  constexpr auto min_velocity = 0.1;
  path.longitudinal_velocity_mps() = std::max(min_velocity, velocity);
}

bool buildDetectionAreaPolygons(
  Polygons2d & polygons, const Trajectory & path, const double s_ego,
  const occlusion_spot_utils::PlannerParam & param)
{
  polygons.clear();

  DetectionRange detection_range;
  detection_range.interval = param.detection_area.slice_length;
  detection_range.min_longitudinal_distance =
    std::max(0.0, param.baselink_to_front - param.detection_area.min_longitudinal_offset);
  detection_range.max_longitudinal_distance =
    std::min(param.detection_area_max_length, param.detection_area_length) +
    detection_range.min_longitudinal_distance;
  detection_range.max_lateral_distance = param.detection_area.max_lateral_distance;
  detection_range.wheel_tread = param.wheel_tread;
  detection_range.right_overhang = param.right_overhang;
  detection_range.left_overhang = param.left_overhang;

  return planning_utils::createDetectionAreaPolygons(
    polygons, path, s_ego, detection_range, param.pedestrian_vel);
}

bool generatePossibleCollisionsFromGridMap(
  std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions,
  const grid_map::GridMap & grid, const Trajectory & path, const double offset_from_start_to_ego,
  const occlusion_spot_utils::PlannerParam & param, occlusion_spot_utils::DebugData & debug_data)
{
  auto distance_lower_bound = std::numeric_limits<double>::max();
  const auto & da_polygons = debug_data.detection_area_polygons;

  for (const auto & detection_area_slice : da_polygons) {
    std::vector<grid_map::Position> occlusion_spot_positions;
    grid_utils::findOcclusionSpots(
      occlusion_spot_positions, grid, detection_area_slice,
      param.detection_area.min_occlusion_spot_size);
    if (occlusion_spot_positions.empty()) {
      continue;
    }

    if (param.is_show_occlusion) {
      for (const auto & op : occlusion_spot_positions) {
        debug_data.occlusion_points.push_back(
          autoware_utils::create_point(op[0], op[1], path.compute(0).point.pose.position.z));
      }
    }

    // for each partition find nearest occlusion spot from polygon's origin
    const auto & base_point = detection_area_slice.outer().at(0);
    const auto pc = generateOneNotableCollisionFromOcclusionSpot(
      grid, occlusion_spot_positions, offset_from_start_to_ego, base_point, path, param,
      debug_data);
    if (!pc) {
      continue;
    }

    const auto lateral_distance = std::abs(pc->arc_lane_dist_at_collision.distance);
    if (lateral_distance > distance_lower_bound) {
      continue;
    }

    distance_lower_bound = lateral_distance;
    possible_collisions.emplace_back(*pc);
  }

  return !possible_collisions.empty();
}

bool generatePossibleCollisionsFromObjects(
  std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions,
  const Trajectory & path, const occlusion_spot_utils::PlannerParam & param,
  const double offset_from_start_to_ego, const std::vector<PredictedObject> & dyn_objects)
{
  const auto path_line_string = to_line_string_2d(path);

  for (const auto & dyn : dyn_objects) {
    const auto arc_coord_occlusion = getOcclusionPoint(dyn, path_line_string);
    const lanelet::ArcCoordinates arc_coord_occlusion_with_offset{
      arc_coord_occlusion.length - param.baselink_to_front,
      calcSignedLateralDistanceWithOffset(
        arc_coord_occlusion.distance, param.right_overhang, param.left_overhang,
        param.wheel_tread)};

    // ignore if collision is not avoidable by velocity control.
    if (
      arc_coord_occlusion_with_offset.length < offset_from_start_to_ego ||
      arc_coord_occlusion_with_offset.length > param.detection_area_length ||
      arc_coord_occlusion_with_offset.length > path.length() ||
      std::abs(arc_coord_occlusion_with_offset.distance) <= 1e-3 ||
      std::abs(arc_coord_occlusion_with_offset.distance) > param.lateral_distance_thr) {
      continue;
    }

    possible_collisions.push_back(calculateCollisionPathPointFromOcclusionSpot(
      arc_coord_occlusion, arc_coord_occlusion_with_offset, path_line_string, param));
  }

  return !possible_collisions.empty();
}

void calcSlowDownPointsForPossibleCollision(
  const Trajectory & path, const double offset,
  std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions)
{
  if (possible_collisions.empty()) {
    return;
  }

  std::sort(
    possible_collisions.begin(), possible_collisions.end(),
    [](
      const occlusion_spot_utils::PossibleCollisionInfo & pc1,
      const occlusion_spot_utils::PossibleCollisionInfo & pc2) {
      return pc1.arc_lane_dist_at_collision.length < pc2.arc_lane_dist_at_collision.length;
    });

  for (auto & col : possible_collisions) {
    const auto s_col = col.arc_lane_dist_at_collision.length + offset;
    if (s_col < 0 || s_col > path.length()) {
      continue;
    }

    const auto path_point = path.compute(s_col);
    col.collision_with_margin.longitudinal_velocity_mps =
      path_point.point.longitudinal_velocity_mps;
    col.collision_with_margin.pose.position.z = path_point.point.pose.position.z;
    col.collision_pose.position.z = path_point.point.pose.position.z;
    col.intersection_pose.position.z = path_point.point.pose.position.z;
    col.obstacle_info.position.z = path_point.point.pose.position.z;
  }
}

void applySafeVelocityConsideringPossibleCollision(
  Trajectory & path, std::vector<occlusion_spot_utils::PossibleCollisionInfo> & possible_collisions,
  std::vector<geometry_msgs::msg::Pose> & debug_poses,
  const occlusion_spot_utils::PlannerParam & param)
{
  const auto & v0 = param.v.v_ego;
  const auto & a0 = param.v.a_ego;
  const auto & j_min = param.v.max_slow_down_jerk;
  const auto & a_min = param.v.max_slow_down_accel;
  const auto & v_min = param.v.min_allowed_velocity;

  for (auto & possible_collision : possible_collisions) {
    const auto & l_obs = possible_collision.arc_lane_dist_at_collision.length;
    const auto & v_org = possible_collision.collision_with_margin.longitudinal_velocity_mps;

    // safe velocity : consider ego emergency braking deceleration
    const auto & v_safe = possible_collision.obstacle_info.safe_motion.safe_velocity;
    // safe slow down: consider ego smooth brake
    const auto v_safe_slow_down =
      planning_utils::calcDecelerationVelocityFromDistanceToTarget(j_min, a_min, a0, v0, l_obs);

    // TODO(tanaka): consider edge case if ego passed safe margin
    const auto & v_slow_down = (l_obs < 0 && v0 <= v_safe) ? v_safe : v_safe_slow_down;

    // skip non effective velocity insertion
    if (v_org < v_safe || v_org < v_slow_down) {
      continue;
    }

    constexpr auto max_vel_noise = 0.05;
    // ensure safe velocity doesn't exceed maximum allowed pbs deceleration
    auto safe_velocity = std::max(v_safe_slow_down + max_vel_noise, v_slow_down);

    // set safe velocity is not to stop
    safe_velocity = std::max(safe_velocity, v_min);
    possible_collision.obstacle_info.safe_motion.safe_velocity = safe_velocity;

    const auto s_decel = autoware::experimental::trajectory::find_nearest_index(
      path, possible_collision.collision_with_margin.pose.position);
    path.longitudinal_velocity_mps().range(s_decel, path.length()).clamp(safe_velocity);
    debug_poses.push_back(path.compute(s_decel).point.pose);
  }
}
}  // namespace utils
}  // namespace autoware::behavior_velocity_planner::experimental
