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

#include "autoware/trajectory_processor/trajectory_modifier_utils/obstacle_stop_utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/sat_2d.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::trajectory_processor::utils::obstacle_stop
{

void trim_trajectory_and_remove_duplicates(TrajectoryPoints & trajectory_points)
{
  if (trajectory_points.empty()) return;
  const auto zero_velocity_index =
    autoware::motion_utils::searchZeroVelocityIndex(trajectory_points);
  const bool found_zero_vel =
    zero_velocity_index && zero_velocity_index.value() < trajectory_points.size() - 1;
  if (found_zero_vel) {
    trajectory_points.erase(
      trajectory_points.begin() + zero_velocity_index.value() + 1, trajectory_points.end());
  }
  trajectory_points = autoware::motion_utils::removeOverlapPoints(trajectory_points);
  if (found_zero_vel)
    trajectory_points.back().longitudinal_velocity_mps =
      0.0;  // set zero velocity at the end of the trajectory
}

double get_detection_length(
  const double forward_traj_length, const double current_vel, const double current_accel,
  const double decel, const double jerk, const double stop_margin)
{
  // add a buffer length to account for the reaction time of the vehicle
  constexpr double buffer_length = 1.0;
  constexpr double time_delay = 0.3;
  const auto margin = stop_margin + buffer_length;
  auto nominal_stopping_distance = autoware::motion_utils::calculate_stop_distance(
    current_vel, current_accel, decel, jerk, time_delay);
  if (nominal_stopping_distance) {
    return nominal_stopping_distance.value() + margin;
  }
  return forward_traj_length + margin;
}

std::optional<std::pair<double, double>> get_curvature_at_end(
  const TrajectoryPoints & trajectory_points, const double lookback_distance)
{
  // need at least 3 points to calculate curvature
  if (trajectory_points.size() < 3) return std::nullopt;

  const auto & last_p = trajectory_points.back();
  const auto lookback_pose = motion_utils::calcLongitudinalOffsetPose(
    trajectory_points, trajectory_points.size() - 1, -1.0 * lookback_distance);
  if (!lookback_pose) return std::nullopt;

  const auto mid_pose = motion_utils::calcLongitudinalOffsetPose(
    trajectory_points, trajectory_points.size() - 1, -1.0 * lookback_distance / 2.0);
  if (!mid_pose) return std::nullopt;

  double curvature = 0.0;
  try {
    curvature = autoware_utils_geometry::calc_curvature(
      lookback_pose.value().position, mid_pose.value().position, last_p.pose.position);
  } catch (const std::exception &) {
    return std::nullopt;
  }

  const auto yaw =
    autoware_utils_geometry::calc_azimuth_angle(mid_pose.value().position, last_p.pose.position);
  return std::make_pair(curvature, yaw);
}

TrajectoryPoints extend_trajectory(const TrajectoryPoints & trajectory_points, const double length)
{
  if (length < 1e-3 || trajectory_points.empty()) return trajectory_points;

  TrajectoryPoints extended_trajectory = trajectory_points;

  const auto & last_p = trajectory_points.back();
  constexpr double lookback_distance = 2.0;  // [m]
  const auto curvature_at_end = get_curvature_at_end(trajectory_points, lookback_distance);
  const auto [curvature, yaw] = curvature_at_end
                                  ? curvature_at_end.value()
                                  : std::pair{0.0, tf2::getYaw(last_p.pose.orientation)};

  const auto end_vel = last_p.longitudinal_velocity_mps;
  constexpr double low_speed_threshold = 0.5;
  constexpr double low_curvature_threshold = 1e-3;
  if (std::abs(curvature) < low_curvature_threshold || end_vel < low_speed_threshold) {
    auto p = trajectory_points.back();
    p.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    p.pose = autoware_utils::calc_offset_pose(p.pose, length, 0, 0);
    extended_trajectory.push_back(p);
    return extended_trajectory;
  }

  const auto turn_radius = 1.0 / curvature;
  constexpr double step = 0.5;
  TrajectoryPoints extension_points;
  for (auto d = length; d > 0; d -= step) {
    auto p = last_p;
    const auto beta = d / turn_radius;
    p.pose.position.x += turn_radius * (std::sin(yaw + beta) - std::sin(yaw));
    p.pose.position.y += turn_radius * (std::cos(yaw) - std::cos(yaw + beta));
    p.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw + beta);
    extension_points.push_back(p);
  }
  std::reverse(extension_points.begin(), extension_points.end());

  extended_trajectory.insert(
    extended_trajectory.end(), extension_points.begin(), extension_points.end());
  return extended_trajectory;
}

TrajectoryShape build_trajectory_footprint_index(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double ego_vel,
  const double ego_accel, const double decel, const double jerk, const double stop_margin)
{
  TrajectoryShape shape;
  shape.trajectory_length = 0.0;
  shape.forward_traj_length = 0.0;
  shape.ego_half_width = vehicle_info.vehicle_width_m / 2.0;
  shape.ego_front_offset = vehicle_info.max_longitudinal_offset_m;
  shape.ego_back_offset = -vehicle_info.min_longitudinal_offset_m;

  if (trajectory_points.empty()) return shape;

  const auto offset_pose =
    autoware_utils::calc_offset_pose(ego_pose, vehicle_info.max_longitudinal_offset_m, 0, 0);
  auto start_idx = motion_utils::findNearestSegmentIndex(trajectory_points, offset_pose.position);

  const auto traj_length = motion_utils::calcArcLength(trajectory_points);
  const auto ego_arc_length =
    motion_utils::calcSignedArcLength(trajectory_points, 0, ego_pose.position);
  const auto forward_traj_length = traj_length - ego_arc_length;
  shape.trajectory_length = traj_length;
  shape.forward_traj_length = forward_traj_length;

  const auto detection_length =
    get_detection_length(forward_traj_length, ego_vel, ego_accel, decel, jerk, stop_margin);

  const auto detection_traj = std::invoke([&]() -> TrajectoryPoints {
    if (detection_length < forward_traj_length) {
      return motion_utils::cropForwardPoints(
        trajectory_points, ego_pose.position, start_idx, detection_length);
    }
    return extend_trajectory(trajectory_points, stop_margin);
  });

  if (detection_traj.empty()) return shape;

  constexpr double min_ds = 0.1;
  constexpr double max_ds = 1.0;

  auto add_sample =
    [&](const geometry_msgs::msg::Pose & pose, const size_t traj_index, const double arc_length) {
      shape.footprints.push_back(EgoFootprint{pose, traj_index, arc_length});
    };

  const auto to_original_index = [&](const size_t detection_idx) {
    return std::min(detection_idx, trajectory_points.size() - 1);
  };

  auto prev_pose = detection_traj.front().pose;
  double prev_s = 0.0;
  size_t prev_idx = 0;
  add_sample(prev_pose, prev_idx, prev_s);

  for (size_t i = 1; i < detection_traj.size(); ++i) {
    const auto & curr_pose = detection_traj[i].pose;
    const auto dist = autoware_utils::calc_distance2d(prev_pose, curr_pose);
    const bool is_last = (i + 1 == detection_traj.size());
    if (!is_last && dist < min_ds) continue;

    const auto traj_index = to_original_index(i);
    if (dist > max_ds) {
      const auto n_steps = static_cast<size_t>(std::ceil(dist / max_ds));
      for (size_t k = 1; k < n_steps; ++k) {
        const auto ratio = static_cast<double>(k) / static_cast<double>(n_steps);
        const auto interp_pose =
          autoware_utils_geometry::calc_interpolated_pose(prev_pose, curr_pose, ratio, false);
        add_sample(interp_pose, prev_idx, prev_s + ratio * dist);
      }
    }

    const auto curr_s = prev_s + dist;
    add_sample(curr_pose, traj_index, curr_s);
    prev_pose = curr_pose;
    prev_s = curr_s;
    prev_idx = traj_index;
  }

  std::vector<FootprintNode> nodes;
  nodes.reserve(shape.footprints.size());
  for (size_t i = 0; i < shape.footprints.size(); ++i) {
    const auto poly = autoware_utils::to_footprint(
      shape.footprints[i].pose, shape.ego_front_offset, shape.ego_back_offset,
      2.0 * shape.ego_half_width);
    const auto box = boost::geometry::return_envelope<autoware_utils_geometry::Box2d>(poly);
    nodes.emplace_back(box, i);
    if (i == 0) {
      shape.bounding_box = box;
    } else {
      boost::geometry::expand(shape.bounding_box, box);
    }
  }
  shape.rtree = FootprintRtree(nodes.begin(), nodes.end());
  return shape;
}

namespace
{
autoware_utils_geometry::Box2d inflate_box(
  const autoware_utils_geometry::Box2d & box, const double margin)
{
  return {
    {box.min_corner().x() - margin, box.min_corner().y() - margin},
    {box.max_corner().x() + margin, box.max_corner().y() + margin}};
}

std::vector<FootprintNode> query_rtree_candidates(
  const TrajectoryShape & shape, const autoware_utils_geometry::Box2d & query_box)
{
  std::vector<FootprintNode> candidates;
  if (shape.rtree.empty()) return candidates;
  shape.rtree.query(boost::geometry::index::intersects(query_box), std::back_inserter(candidates));
  return candidates;
}

std::vector<size_t> sort_hits_by_arc_length(
  const TrajectoryShape & shape, std::vector<size_t> && hits)
{
  std::sort(hits.begin(), hits.end(), [&](const size_t a, const size_t b) {
    return shape.footprints[a].arc_length < shape.footprints[b].arc_length;
  });
  return hits;
}

bool is_point_within_footprint(
  const TrajectoryShape & shape, const EgoFootprint & footprint, const Point2d & point,
  const double lat_margin)
{
  const auto rel = autoware_utils::inverse_transform_point(point.to_3d(), footprint.pose);
  return rel.x() >= -shape.ego_back_offset && rel.x() <= shape.ego_front_offset &&
         std::abs(rel.y()) <= shape.ego_half_width + lat_margin;
}

Polygon2d make_local_ego_polygon(const TrajectoryShape & shape, const double lat_margin)
{
  const double half_width = shape.ego_half_width + lat_margin;
  Polygon2d ego;
  ego.outer() = {
    {shape.ego_front_offset, half_width},  {shape.ego_front_offset, -half_width},
    {-shape.ego_back_offset, -half_width}, {-shape.ego_back_offset, half_width},
    {shape.ego_front_offset, half_width},
  };
  return ego;
}

Polygon2d transform_polygon_to_pose_frame(
  const Polygon2d & polygon, const geometry_msgs::msg::Pose & pose)
{
  Polygon2d local;
  local.outer().reserve(polygon.outer().size());
  for (const auto & p : polygon.outer()) {
    const auto rel = autoware_utils::inverse_transform_point(p.to_3d(), pose);
    local.outer().emplace_back(rel.x(), rel.y());
  }
  return local;
}
}  // namespace

std::vector<size_t> query_overlapping_footprints(
  const TrajectoryShape & shape, const Polygon2d & polygon, const double lat_margin)
{
  if (polygon.outer().empty()) return {};

  const auto query_box = inflate_box(
    boost::geometry::return_envelope<autoware_utils_geometry::Box2d>(polygon), lat_margin);
  const auto candidates = query_rtree_candidates(shape, query_box);
  const auto ego_local = make_local_ego_polygon(shape, lat_margin);

  std::vector<size_t> hits;
  hits.reserve(candidates.size());
  for (const auto & node : candidates) {
    const auto object_local =
      transform_polygon_to_pose_frame(polygon, shape.footprints[node.second].pose);
    if (autoware_utils_geometry::sat::intersects(ego_local, object_local)) {
      hits.push_back(node.second);
    }
  }
  return sort_hits_by_arc_length(shape, std::move(hits));
}

std::vector<size_t> query_overlapping_footprints(
  const TrajectoryShape & shape, const Point2d & point, const double lat_margin)
{
  const autoware_utils_geometry::Box2d query_box{
    {point.x() - lat_margin, point.y() - lat_margin},
    {point.x() + lat_margin, point.y() + lat_margin}};
  const auto candidates = query_rtree_candidates(shape, query_box);

  std::vector<size_t> hits;
  hits.reserve(candidates.size());
  for (const auto & node : candidates) {
    if (is_point_within_footprint(shape, shape.footprints[node.second], point, lat_margin)) {
      hits.push_back(node.second);
    }
  }
  return sort_hits_by_arc_length(shape, std::move(hits));
}

std::optional<CollisionPoint> get_nearest_pcd_collision(
  const TrajectoryShape & trajectory_shape, const PointCloud::Ptr & pointcloud,
  const LateralMarginMap & lateral_margin_map,
  std::vector<geometry_msgs::msg::Point> & target_pcd_points)
{
  if (pointcloud->empty() || trajectory_shape.footprints.empty()) return std::nullopt;

  auto min_arc_length = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_collision_point;
  bool found_collision = false;

  for (const auto & point : *pointcloud) {
    const auto classification = static_cast<PointCloudClassification>(point.class_id);
    const auto lat_margin = get_lateral_margin(lateral_margin_map, to_object_type(classification));
    const Point2d query_point{point.x, point.y};
    const auto hits = query_overlapping_footprints(trajectory_shape, query_point, lat_margin);
    if (hits.empty()) continue;

    const auto & footprint = trajectory_shape.footprints[hits.front()];
    const auto rel = autoware_utils::inverse_transform_point(query_point.to_3d(), footprint.pose);
    const auto arc_length = footprint.arc_length + rel.x();

    geometry_msgs::msg::Point p =
      geometry_msgs::msg::Point().set__x(point.x).set__y(point.y).set__z(point.z);
    target_pcd_points.emplace_back(p);

    if (arc_length < min_arc_length) {
      min_arc_length = arc_length;
      nearest_collision_point = p;
      found_collision = true;
    }
  }

  if (!found_collision) return std::nullopt;
  return CollisionPoint(nearest_collision_point, min_arc_length, 0.0);
}

geometry_msgs::msg::Pose extrapolate_object_pose_from_kinematics(
  const PredictedObject & object, const double t)
{
  const auto & initial_pose = object.kinematics.initial_pose_with_covariance.pose;
  if (t <= 0.0) return initial_pose;

  const auto obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto obj_accel = object.kinematics.initial_acceleration_with_covariance.accel.linear.x;
  const auto dist = obj_vel * t + 0.5 * obj_accel * t * t;
  return autoware_utils::calc_offset_pose(initial_pose, dist, 0, 0);
}

geometry_msgs::msg::Pose get_predicted_obj_pose_at_time(
  const PredictedObject & object, const double t)
{
  if (object.kinematics.predicted_paths.empty()) {
    return extrapolate_object_pose_from_kinematics(object, t);
  }

  const auto predicted_path = [&]() {
    std::optional<autoware_perception_msgs::msg::PredictedPath> p;
    for (const auto & path : object.kinematics.predicted_paths) {
      if (path.path.empty()) continue;
      if (!p || path.confidence > p->confidence) p = path;
    }
    return p;
  }();

  if (!predicted_path) {
    return extrapolate_object_pose_from_kinematics(object, t);
  }

  const double dt = std::max(rclcpp::Duration(predicted_path->time_step).seconds(), 1e-3);
  const double path_horizon = dt * static_cast<double>(predicted_path->path.size() - 1);
  if (t >= path_horizon) {
    return predicted_path->path.back();
  }

  auto interpolated_pose =
    autoware::object_recognition_utils::calcInterpolatedPose(*predicted_path, t);
  if (!interpolated_pose) {
    return extrapolate_object_pose_from_kinematics(object, t);
  }

  return interpolated_pose.value();
}

ObjectState get_object_state_at_time(
  const TrajectoryPoints & trajectory_points, const PredictedObject & object, const double t)
{
  const auto predicted_obj_pose = get_predicted_obj_pose_at_time(object, t);

  const auto nearest_seg =
    motion_utils::findNearestSegmentIndex(trajectory_points, predicted_obj_pose.position);
  const auto p1 = trajectory_points.at(nearest_seg).pose.position;
  const auto p2 = trajectory_points.at(nearest_seg + 1).pose.position;
  auto lon_vel = [&]() {
    const auto traj_dir = Eigen::Vector2d(p2.x - p1.x, p2.y - p1.y).normalized();
    const Eigen::Rotation2Dd obj_rot(tf2::getYaw(predicted_obj_pose.orientation));
    const auto obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto obj_vel_vector = obj_rot * Eigen::Vector2d(obj_vel.x, obj_vel.y);
    return obj_vel_vector.dot(traj_dir);
  }();

  const auto obj_polygon = autoware_utils::to_polygon2d(predicted_obj_pose, object.shape);
  auto min_arc_length = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_point;
  for (const auto & point : obj_polygon.outer()) {
    const geometry_msgs::msg::Point p =
      geometry_msgs::msg::Point().set__x(point.x()).set__y(point.y());
    const auto l = motion_utils::calcSignedArcLength(trajectory_points, 0, p);
    if (l < min_arc_length) {
      min_arc_length = l;
      nearest_point = p;
    }
  }

  return ObjectState{std::max(0.0, min_arc_length), lon_vel, nearest_point};
}

double get_safe_distance(
  const double ego_vel, const double ego_decel, const double object_stopping_distance,
  const double reaction_time, const double safety_margin)
{
  constexpr double eps = 1e-3;
  const auto ego_decel_mag = std::max(std::abs(ego_decel), eps);
  const auto reaction_distance = ego_vel * reaction_time;
  const auto ego_stopping_distance = ego_vel * ego_vel / (2 * ego_decel_mag);
  const auto safe_distance =
    reaction_distance + ego_stopping_distance - object_stopping_distance + safety_margin;
  return std::max(safe_distance, safety_margin);
}

std::optional<CollisionPoint> get_nearest_object_collision(
  TargetObjects & target_objects, const TrajectoryPoints & trajectory_points,
  const double stopped_vel_th)
{
  if (target_objects.empty() || trajectory_points.size() < 2) return std::nullopt;

  auto min_arc_length = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_collision_point;
  bool found_collision = false;
  double obstacle_lon_vel = 0.0;
  for (auto & object : target_objects) {
    const auto obj_state = get_object_state_at_time(trajectory_points, object.object, 0.0);
    object.is_safe = false;
    object.safe_distance = obj_state.arc_length;
    object.distance_from_ego = obj_state.arc_length;
    found_collision = true;
    if (obj_state.arc_length < min_arc_length) {
      min_arc_length = obj_state.arc_length;
      nearest_collision_point = obj_state.nearest_point;
      obstacle_lon_vel = obj_state.lon_vel;
    }
  }

  if (!found_collision) return std::nullopt;
  return CollisionPoint(
    nearest_collision_point, min_arc_length, obstacle_lon_vel,
    std::abs(obstacle_lon_vel) > stopped_vel_th);
}

std::optional<CollisionPoint> get_nearest_object_collision(
  TargetObjects & target_objects, const TrajectoryPoints & trajectory_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const ObjectDecelMap & object_decel_map, const double ego_decel, const double reaction_time,
  const double safety_margin, const double stopped_vel_th, const double lookahead_horizon,
  const bool use_rss_check)
{
  if (target_objects.empty() || trajectory_points.size() < 2) return std::nullopt;

  // If RSS check is disabled, get the nearest object collision by pure geometric overlap.
  if (!use_rss_check) {
    return get_nearest_object_collision(target_objects, trajectory_points, stopped_vel_th);
  }

  const auto ego_front_offset = vehicle_info.max_longitudinal_offset_m;
  constexpr double eps = 1e-3;

  auto get_object_stopping_distance = [&](const auto & object, const double obj_lon_vel) {
    if (obj_lon_vel < stopped_vel_th) return 0.0;
    const auto label =
      object.classification.empty()
        ? ObjectClassification::UNKNOWN
        : autoware::object_recognition_utils::getHighestProbLabel(object.classification);
    const auto obj_type = classification_to_object_type.at(label);
    if (!object_decel_map.count(obj_type)) return 0.0;
    const auto obj_decel = object_decel_map.at(obj_type);
    const auto object_decel_mag = std::max(std::abs(obj_decel), eps);
    const auto object_stopping_distance = obj_lon_vel * obj_lon_vel / (2 * object_decel_mag);
    return object_stopping_distance;
  };

  auto is_safe = [&](
                   const double obj_arc_length, const double obj_stopping_distance,
                   const double ego_arc_length,
                   const double ego_vel) -> std::tuple<bool, double, double> {
    const auto ego_front_arc_length = ego_arc_length + ego_front_offset;
    const auto relative_arc_length = std::max(0.0, obj_arc_length - ego_front_arc_length);
    if (obj_stopping_distance <= eps)
      return std::make_tuple(false, relative_arc_length, relative_arc_length);
    const auto safe_dist =
      get_safe_distance(ego_vel, ego_decel, obj_stopping_distance, reaction_time, safety_margin);
    return std::make_tuple(relative_arc_length - safe_dist > 1e-3, safe_dist, relative_arc_length);
  };

  auto min_collision_arc_length = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_collision_point;
  bool found_collision = false;
  double obstacle_lon_vel = 0.0;

  for (auto & object : target_objects) {
    auto last_p = trajectory_points.front().pose.position;
    auto curr_arc_length = 0.0;
    bool is_first = true;
    for (const auto & traj_p : trajectory_points) {
      const auto t = rclcpp::Duration(traj_p.time_from_start).seconds();
      if (t > lookahead_horizon) break;
      curr_arc_length += autoware_utils::calc_distance2d(last_p, traj_p.pose.position);
      last_p = traj_p.pose.position;
      const auto target_ego_vel = traj_p.longitudinal_velocity_mps;
      const auto obj_state = get_object_state_at_time(trajectory_points, object.object, t);
      const auto obj_stopping_distance =
        get_object_stopping_distance(object.object, obj_state.lon_vel);
      const auto [safe, safe_distance, distance_from_ego] =
        is_safe(obj_state.arc_length, obj_stopping_distance, curr_arc_length, target_ego_vel);
      object.is_safe = safe;
      if (is_first) {
        is_first = false;
        object.safe_distance = safe_distance;
        object.distance_from_ego = distance_from_ego;
      }
      if (object.is_safe) continue;
      found_collision = true;
      auto collision_arc_length = obj_state.arc_length + obj_stopping_distance;
      if (collision_arc_length < min_collision_arc_length) {
        min_collision_arc_length = collision_arc_length;
        const auto collision_point = motion_utils::calcLongitudinalOffsetPose(
          trajectory_points, obj_state.nearest_point, obj_stopping_distance);
        nearest_collision_point =
          collision_point.has_value() ? collision_point.value().position : obj_state.nearest_point;
        obstacle_lon_vel = obj_state.lon_vel;
      }
      break;
    }
  }

  if (!found_collision) return std::nullopt;
  return CollisionPoint(
    nearest_collision_point, min_collision_arc_length, obstacle_lon_vel,
    std::abs(obstacle_lon_vel) > stopped_vel_th);
}

void PointCloudFilter::filter_pointcloud(
  PointCloud::Ptr & pointcloud, const double min_x, const double max_x, const double min_y,
  const double max_y, const double min_z, const double max_z)
{
  if (pointcloud->empty()) return;

  auto is_within_range = [&](const auto & point) {
    return point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y &&
           point.z >= min_z && point.z <= max_z;
  };

  auto is_target_type = [&](const auto & point) {
    const auto classification = static_cast<PointCloudClassification>(point.class_id);
    if (pcd_class_to_object_type.count(classification) == 0) return false;
    return pcd_types_.count(pcd_class_to_object_type.at(classification)) != 0;
  };

  // Axis-aligned crop via x/y/z only. pcl::CropBox cannot be used with PointXYZCPE because
  // PCL transform helpers require a .data member that this point type does not provide.
  pointcloud->erase(
    std::remove_if(
      pointcloud->begin(), pointcloud->end(),
      [&](const auto & point) { return !is_within_range(point) || !is_target_type(point); }),
    pointcloud->end());
}

void PointCloudFilter::filter_pointcloud_by_object(
  PointCloud::Ptr & pointcloud, const PredictedObjects & objects)
{
  if (pointcloud->empty() || objects.objects.empty()) return;

  const auto margin = 0.1;
  auto get_object_polygon = [&](const auto & object) {
    auto polygon = autoware_utils::to_polygon2d(object);
    auto expanded_polygon = autoware_utils::expand_polygon(polygon, margin);
    return expanded_polygon;
  };

  for (const auto & obj : objects.objects) {
    const auto obj_pose = obj.kinematics.initial_pose_with_covariance.pose;
    std::optional<autoware_utils_geometry::Polygon2d> obj_polygon;
    const auto x_th = obj.shape.dimensions.x / 2.0 + margin;
    const auto y_th = obj.shape.dimensions.y / 2.0 + margin;
    pointcloud->erase(
      std::remove_if(
        pointcloud->begin(), pointcloud->end(),
        [&](const auto & point) {
          const autoware_utils_geometry::Point2d p(point.x, point.y);
          const auto rel_p = autoware_utils::inverse_transform_point(p.to_3d(), obj_pose);
          if (std::abs(rel_p.x()) > x_th || std::abs(rel_p.y()) > y_th) {
            return false;
          }
          if (!obj_polygon) obj_polygon = get_object_polygon(obj);
          return !boost::geometry::disjoint(p, *obj_polygon);
        }),
      pointcloud->end());
  }
}

void ObjectFilter::filter_by_target_area(
  TargetObjects & target_objects, const TrajectoryPoints & trajectory_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const TrajectoryShape & trajectory_shape, const LateralMarginMap & lateral_margin_map)
{
  const auto ego_front_offset = vehicle_info.max_longitudinal_offset_m;
  constexpr double time_buffer = 0.5;
  auto time_to_obj_current_pos =
    [&](const auto & object_pose, const size_t nearest_seg_idx) -> std::optional<double> {
    const auto t_to_nearest_seg =
      rclcpp::Duration(trajectory_points.at(nearest_seg_idx).time_from_start).seconds();
    const auto lon_offset_dist =
      motion_utils::calcSignedArcLength(trajectory_points, nearest_seg_idx, object_pose.position);
    const auto nearest_seg_vel = trajectory_points.at(nearest_seg_idx).longitudinal_velocity_mps;
    if (nearest_seg_vel < 1e-3) {
      if (lon_offset_dist > ego_front_offset) return std::nullopt;
      return std::max(0.0, t_to_nearest_seg - time_buffer);
    }
    const auto ego_front_time_offset = ego_front_offset / nearest_seg_vel;
    const auto t_to_obj =
      t_to_nearest_seg + (lon_offset_dist / nearest_seg_vel) - ego_front_time_offset - time_buffer;
    return std::max(0.0, t_to_obj);
  };

  auto get_object_polygon = [&](const auto & pose, const auto & shape) {
    if (shape.type != Shape::POLYGON) {
      auto s = shape;
      s.dimensions.x += safety_buffer_;
      s.dimensions.y += safety_buffer_;
      return autoware_utils::to_polygon2d(pose, s);
    }
    const auto polygon = autoware_utils::to_polygon2d(pose, shape);
    return autoware_utils::expand_polygon(polygon, safety_buffer_);
  };

  auto overlaps_ego_footprints = [&](const Polygon2d & polygon, const double lat_margin) {
    return !query_overlapping_footprints(trajectory_shape, polygon, lat_margin).empty();
  };

  auto is_exiting = [&](const auto & object, const double lat_margin) -> bool {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto obj_rot = Eigen::Rotation2Dd(tf2::getYaw(object_pose.orientation));
    const auto obj_vel = object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto obj_vel_vector = (obj_rot * Eigen::Vector2d(obj_vel.x, obj_vel.y));
    if (obj_vel_vector.norm() < stopped_velocity_th_) return false;  // object is stopped
    const auto nearest_seg =
      motion_utils::findNearestSegmentIndex(trajectory_points, object_pose.position);
    const auto p1 = trajectory_points.at(nearest_seg).pose.position;
    const auto p2 = trajectory_points.at(nearest_seg + 1).pose.position;
    const auto traj_dir = Eigen::Vector2d(p2.x - p1.x, p2.y - p1.y).normalized();
    const auto traj_lat_dir = Eigen::Vector2d(traj_dir.y(), -traj_dir.x());
    const auto obj_lon_vel = std::abs(obj_vel_vector.dot(traj_dir));
    const auto obj_lat_vel = std::abs(obj_vel_vector.dot(traj_lat_dir));
    if (obj_lat_vel < max_lateral_velocity_th_ && obj_lat_vel < obj_lon_vel) return false;
    const auto t_to_obj_current_pos = time_to_obj_current_pos(object_pose, nearest_seg);
    if (!t_to_obj_current_pos) return true;
    const auto obj_pred_pose = get_predicted_obj_pose_at_time(object, t_to_obj_current_pos.value());
    const auto obj_pred_polygon = get_object_polygon(obj_pred_pose, object.shape);
    return !overlaps_ego_footprints(obj_pred_polygon, lat_margin);
  };

  target_objects.erase(
    std::remove_if(
      target_objects.begin(), target_objects.end(),
      [&](auto & obj) {
        const auto & object = obj.object;
        const auto lat_margin = get_lateral_margin(lateral_margin_map, to_object_type(object));
        const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
        const auto object_polygon = get_object_polygon(object_pose, object.shape);
        if (!overlaps_ego_footprints(object_polygon, lat_margin)) return true;
        if (is_exiting(object, lat_margin)) return true;
        obj.polygon = object_polygon;
        return false;
      }),
    target_objects.end());
}

void ObstacleTracker::update_objects(
  const PredictedObjects & objects, TargetObjects & persistent_objects, const rclcpp::Time & now)
{
  for (auto it = persistent_objects_map_.begin(); it != persistent_objects_map_.end();) {
    const auto idle_time = (now - it->second.last_seen_time).seconds();
    const bool is_erase =
      !it->second.is_active ? idle_time > grace_period_ : idle_time > off_time_buffer_;
    if (is_erase)
      it = persistent_objects_map_.erase(it);
    else
      it++;
  }

  auto estimate_pose = [&](const PersistentObject & object) -> geometry_msgs::msg::Pose {
    const auto & pose = object.object.kinematics.initial_pose_with_covariance.pose;
    const auto & twist = object.object.kinematics.initial_twist_with_covariance.twist.linear;
    const auto dt = (now - object.last_seen_time).seconds();
    const auto dx = twist.x * dt;
    const auto dy = twist.y * dt;
    return autoware_utils::calc_offset_pose(pose, dx, dy, 0.0);
  };

  auto get_closest_object_uuid =
    [&](const PredictedObject & object) -> std::optional<boost::uuids::uuid> {
    std::optional<boost::uuids::uuid> closest_uuid = std::nullopt;
    if (persistent_objects_map_.empty()) return std::nullopt;
    double min_distance = object_distance_th_ + std::numeric_limits<double>::epsilon();
    for (const auto & [uuid, existing_object] : persistent_objects_map_) {
      const auto existing_obj_label = existing_object.object.classification.empty()
                                        ? ObjectClassification::UNKNOWN
                                        : autoware::object_recognition_utils::getHighestProbLabel(
                                            existing_object.object.classification);
      const auto obj_label =
        object.classification.empty()
          ? ObjectClassification::UNKNOWN
          : autoware::object_recognition_utils::getHighestProbLabel(object.classification);
      if (existing_obj_label != obj_label) continue;
      const auto distance = autoware_utils::calc_distance2d(
        object.kinematics.initial_pose_with_covariance.pose.position,
        estimate_pose(existing_object).position);
      if (distance > min_distance) continue;
      // ignore orientation difference for symmetric objects
      const bool is_symmetric =
        std::abs(object.shape.dimensions.x - object.shape.dimensions.y) < 0.1;
      const auto yaw_diff =
        is_symmetric ? 0.0
                     : std::abs(
                         autoware_utils_geometry::calc_yaw_deviation(
                           object.kinematics.initial_pose_with_covariance.pose,
                           existing_object.object.kinematics.initial_pose_with_covariance.pose));
      if (yaw_diff > object_yaw_th_) continue;
      min_distance = distance;
      closest_uuid = uuid;
    }
    return closest_uuid;
  };

  for (const auto & object : objects.objects) {
    const auto closest_uuid = get_closest_object_uuid(object);
    if (!closest_uuid) {
      persistent_objects_map_.emplace(id_generator_(), PersistentObject(object, now));
      continue;
    }
    auto & closest_object = persistent_objects_map_.at(closest_uuid.value());
    closest_object.last_seen_time = now;
    closest_object.object = object;
    const auto duration = (now - closest_object.first_seen_time).seconds();
    closest_object.is_active = duration >= on_time_buffer_;
  }

  for (const auto & [uuid, entry] : persistent_objects_map_) {
    if (entry.is_active) {
      persistent_objects.emplace_back(entry.object);
    }
  }
}

void ObstacleTracker::update_points(
  const PointCloud::Ptr & points, PointCloud::Ptr & persistent_points, const rclcpp::Time & now)
{
  for (auto it = persistent_point_map_.begin(); it != persistent_point_map_.end();) {
    const auto idle_time = (now - it->second.last_seen_time).seconds();
    const bool is_erase =
      !it->second.is_active ? idle_time > grace_period_ : idle_time > off_time_buffer_;
    if (is_erase)
      it = persistent_point_map_.erase(it);
    else
      it++;
  }

  auto get_closest_point_uuid =
    [&](const PointXYZCPE & point) -> std::optional<boost::uuids::uuid> {
    std::optional<boost::uuids::uuid> closest_uuid = std::nullopt;
    if (persistent_point_map_.empty()) return std::nullopt;
    double min_distance = pcd_distance_th_ + std::numeric_limits<double>::epsilon();
    for (const auto & [uuid, existing_point] : persistent_point_map_) {
      const auto distance = autoware_utils::calc_distance2d(point, existing_point.point);
      if (distance > min_distance) continue;
      min_distance = distance;
      closest_uuid = uuid;
    }
    return closest_uuid;
  };

  for (const auto & point : points->points) {
    const auto closest_uuid = get_closest_point_uuid(point);
    if (!closest_uuid) {
      persistent_point_map_.emplace(id_generator_(), PersistentPoint(point, now));
      continue;
    }
    auto & closest_point = persistent_point_map_.at(closest_uuid.value());
    closest_point.last_seen_time = now;
    closest_point.point = point;
    const auto duration = (now - closest_point.first_seen_time).seconds();
    closest_point.is_active = duration >= on_time_buffer_;
  }

  for (const auto & [uuid, entry] : persistent_point_map_) {
    if (entry.is_active) {
      persistent_points->points.push_back(entry.point);
    }
  }
}

}  // namespace autoware::trajectory_processor::utils::obstacle_stop
