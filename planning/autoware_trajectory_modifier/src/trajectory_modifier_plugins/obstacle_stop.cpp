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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/obstacle_stop.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/trajectory/interpolator/akima_spline.hpp>
#include <autoware/trajectory/interpolator/interpolator.hpp>
#include <autoware/trajectory/pose.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/view.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using utils::obstacle_stop::get_nearest_object_collision;
using utils::obstacle_stop::get_nearest_pcd_collision;
using utils::obstacle_stop::get_trajectory_shape;
using utils::obstacle_stop::PointCloud;
using utils::obstacle_stop::PointCloud2;

using autoware::experimental::trajectory::interpolator::AkimaSpline;
using InterpolationTrajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;

void ObstacleStop::on_initialize(const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_obstacle_stop");

  pub_clustered_pointcloud_ =
    node_ptr->create_publisher<PointCloud2>("~/obstacle_stop/debug/cluster_points", 1);
  debug_viz_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/obstacle_stop/debug/marker", 1);

  params_ = params.obstacle_stop;
  enabled_ = params.use_obstacle_stop;
  trajectory_time_step_ = params.trajectory_time_step;

  {
    const auto & p = params_.pointcloud;
    pointcloud_filter_ = std::make_unique<utils::obstacle_stop::PointCloudFilter>(
      p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z,
      p.voxel_grid_filter.min_size, p.clustering.tolerance, p.clustering.min_size,
      p.clustering.max_size);
  }

  {
    const auto & p = params_.objects;
    object_filter_ =
      std::make_unique<utils::obstacle_stop::ObjectFilter>(p.object_types, p.max_velocity_th);
  }

  {
    const auto & p = params_.obstacle_tracking;
    obstacle_tracker_ = std::make_unique<utils::obstacle_stop::ObstacleTracker>(
      p.on_time_buffer, p.off_time_buffer, p.object_distance_th, p.object_yaw_th, p.pcd_distance_th,
      p.grace_period);
  }

  {
    const auto & p = params_.rss_params;
    object_decel_map_ = {
      {utils::obstacle_stop::ObjectType::CAR, p.object_decel.car},
      {utils::obstacle_stop::ObjectType::TRUCK, p.object_decel.truck},
      {utils::obstacle_stop::ObjectType::BUS, p.object_decel.bus},
      {utils::obstacle_stop::ObjectType::TRAILER, p.object_decel.trailer},
      {utils::obstacle_stop::ObjectType::MOTORCYCLE, p.object_decel.motorcycle},
      {utils::obstacle_stop::ObjectType::BICYCLE, p.object_decel.bicycle},
      {utils::obstacle_stop::ObjectType::PEDESTRIAN, p.object_decel.pedestrian}};
  }
}

void ObstacleStop::update_params(const TrajectoryModifierParams & params)
{
  params_ = params.obstacle_stop;
  enabled_ = params.use_obstacle_stop;
  trajectory_time_step_ = params.trajectory_time_step;

  {
    const auto & p = params_.pointcloud;
    pointcloud_filter_->set_params(
      p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z,
      p.voxel_grid_filter.min_size, p.clustering.tolerance, p.clustering.min_size,
      p.clustering.max_size);
  }

  {
    const auto & p = params_.objects;
    object_filter_->set_params(p.object_types, p.max_velocity_th);
  }

  {
    const auto & p = params_.obstacle_tracking;
    obstacle_tracker_->set_params(
      p.on_time_buffer, p.off_time_buffer, p.object_distance_th, p.object_yaw_th, p.pcd_distance_th,
      p.grace_period);
  }

  {
    const auto & p = params_.rss_params;
    object_decel_map_ = {
      {utils::obstacle_stop::ObjectType::CAR, p.object_decel.car},
      {utils::obstacle_stop::ObjectType::TRUCK, p.object_decel.truck},
      {utils::obstacle_stop::ObjectType::BUS, p.object_decel.bus},
      {utils::obstacle_stop::ObjectType::TRAILER, p.object_decel.trailer},
      {utils::obstacle_stop::ObjectType::MOTORCYCLE, p.object_decel.motorcycle},
      {utils::obstacle_stop::ObjectType::BICYCLE, p.object_decel.bicycle},
      {utils::obstacle_stop::ObjectType::PEDESTRIAN, p.object_decel.pedestrian}};
  }
}

bool ObstacleStop::is_trajectory_modification_required(const TrajectoryPoints & traj_points)
{
  debug_data_ = DebugData();
  safety_factors_ = SafetyFactorArray{};

  if (traj_points.empty()) {
    nearest_collision_point_ = std::nullopt;
    return false;
  }

  {
    autoware_utils_debug::ScopedTimeTrack st(
      "ObstacleStop::get_trajectory_shape", *get_time_keeper());

    debug_data_.trajectory_shape = get_trajectory_shape(
      traj_points, data_->current_odometry->pose.pose, data_->vehicle_info,
      data_->current_odometry->twist.twist.linear.x,
      data_->current_acceleration->accel.accel.linear.x, params_.nominal_stopping_decel,
      params_.stopping_jerk, params_.stop_margin, params_.lateral_margin);
  }

  check_obstacles(traj_points);

  debug_data_.active_collision_point =
    nearest_collision_point_ ? nearest_collision_point_->point : geometry_msgs::msg::Point();

  return nearest_collision_point_ != std::nullopt;
}

bool ObstacleStop::modify_trajectory(TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::modify_trajectory", *get_time_keeper());

  if (!enabled_) return false;

  auto trajectory = traj_points;
  utils::obstacle_stop::trim_trajectory_and_remove_duplicates(trajectory);
  if (trajectory.empty()) return false;

  if (!is_trajectory_modification_required(trajectory)) return false;

  if (!nearest_collision_point_) return false;

  traj_points = std::move(trajectory);

  return set_stop_point(traj_points);
}

bool ObstacleStop::set_stop_point(TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::set_stop_point", *get_time_keeper());
  const auto target_stop_point_arc_length = std::invoke([&]() -> double {
    const auto stop_margin = params_.stop_margin + data_->vehicle_info.max_longitudinal_offset_m;
    auto min_stopping_distance = motion_utils::calculate_stop_distance(
      data_->current_odometry->twist.twist.linear.x,
      data_->current_acceleration->accel.accel.linear.x, params_.maximum_stopping_decel,
      params_.stopping_jerk, 0.0);
    if (!min_stopping_distance) min_stopping_distance = 0.0;
    return std::clamp(
      nearest_collision_point_->arc_length - stop_margin, min_stopping_distance.value(),
      debug_data_.trajectory_shape.trajectory_length);
  });

  auto skip = [&](const std::string & msg) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] %s, skip inserting stop point", msg.c_str());
    return false;
  };

  constexpr double zero_velocity_threshold = 0.01;
  auto checked_distance = 0.0;
  for (size_t i = 1; i < traj_points.size(); ++i) {
    const auto & curr = traj_points.at(i);
    const auto & prev = traj_points.at(i - 1);
    checked_distance +=
      autoware_utils_geometry::calc_distance2d(curr.pose.position, prev.pose.position);
    if (curr.longitudinal_velocity_mps > zero_velocity_threshold) continue;
    if (checked_distance < target_stop_point_arc_length) {
      return skip("Preceding stop point exists");
    }
  }

  if (
    target_stop_point_arc_length < params_.arrived_distance_threshold ||
    !apply_stopping(traj_points, target_stop_point_arc_length)) {
    traj_points = std::invoke([&]() {
      TrajectoryPoints stop_points;
      auto p = traj_points.front();
      p.longitudinal_velocity_mps = 0.0;
      p.acceleration_mps2 = 0.0;
      p.time_from_start = rclcpp::Duration::from_seconds(0.0);
      stop_points.push_back(p);
      p.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_step_);
      stop_points.push_back(p);
      return stop_points;
    });
  }

  const auto & stop_pose = traj_points.back().pose;
  const auto & ego_pose = data_->current_odometry->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, stop_pose, PlanningFactor::STOP, safety_factors_);

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM ObstacleStop] Inserted stop point at arc length %f m", target_stop_point_arc_length);
  return true;
}

size_t update_velocities(TrajectoryPoints & trajectory, const double jerk, const double decel)
{
  if (trajectory.size() < 2) return 0;

  auto get_vel_accel = [&](
                         const auto & point, const auto & ref_point) -> std::pair<double, double> {
    const auto v_ref = ref_point.longitudinal_velocity_mps;
    const auto a_ref = std::abs(ref_point.acceleration_mps2);

    const auto ds =
      autoware_utils_geometry::calc_distance2d(point.pose.position, ref_point.pose.position);
    const auto da = jerk * (ds / std::max<double>(v_ref, 0.1));
    const auto a_curr = std::min(a_ref + da, decel);
    const auto a_avg = (a_ref + a_curr) / 2.0;
    const auto v_curr = std::sqrt(std::max(0.0, v_ref * v_ref + 2.0 * a_avg * ds));
    return {v_curr, -1.0 * a_curr};
  };

  const auto stop_index = trajectory.size() - 1;
  auto vel_update_start_index = stop_index;
  for (int i = static_cast<int>(stop_index) - 1; i > 0; --i) {
    auto & curr = trajectory.at(i);
    auto & next = trajectory.at(i + 1);
    const auto [v_curr, a_curr] = get_vel_accel(curr, next);

    if (v_curr >= curr.longitudinal_velocity_mps) break;
    curr.longitudinal_velocity_mps = static_cast<float>(v_curr);
    curr.acceleration_mps2 = static_cast<float>(a_curr);
    vel_update_start_index = i;
  }
  return vel_update_start_index;
}

size_t insert_stop_point(
  TrajectoryPoints & trajectory, const double target_stop_point_arc_length,
  const double traj_length)
{
  const auto index = motion_utils::insertStopPoint(target_stop_point_arc_length, trajectory);
  if (index) return index.value();

  // TODO(Quda): this is a temporary fix, need to check why insertStopPoint fails when target
  // distance is equal to trajectory length
  if (target_stop_point_arc_length < traj_length) {
    auto dist = 0.0;
    auto it = std::adjacent_find(
      trajectory.begin(), trajectory.end(), [&](const auto & p, const auto & next) {
        dist += autoware_utils_geometry::calc_distance2d(p.pose.position, next.pose.position);
        return dist >= target_stop_point_arc_length - 1e-3;
      });
    if (it != trajectory.end()) {
      it->longitudinal_velocity_mps = 0.0;
      it->acceleration_mps2 = 0.0;
      return std::distance(trajectory.begin(), it);
    }
  }

  trajectory.back().longitudinal_velocity_mps = 0.0;
  trajectory.back().acceleration_mps2 = 0.0;
  return trajectory.size() - 1;
}

bool ObstacleStop::apply_stopping(
  TrajectoryPoints & traj_points, const double target_stop_point_arc_length) const
{
  if (target_stop_point_arc_length < 1e-3) return false;

  auto trajectory = traj_points;

  const auto stop_index = insert_stop_point(
    trajectory, target_stop_point_arc_length, debug_data_.trajectory_shape.trajectory_length);

  trajectory.erase(trajectory.begin() + stop_index + 1, trajectory.end());
  trajectory.back().longitudinal_velocity_mps = 0.0;
  trajectory.back().acceleration_mps2 = 0.0;

  constexpr double min_v_ref = 1.0;
  const auto v_ref = std::max<double>(min_v_ref, trajectory.front().longitudinal_velocity_mps);
  const auto required_decel = std::abs(v_ref * v_ref / (2.0 * target_stop_point_arc_length));
  const auto jerk = std::abs(params_.stopping_jerk);
  const auto decel = std::clamp(
    required_decel, std::abs(params_.nominal_stopping_decel),
    std::abs(params_.maximum_stopping_decel));

  const auto vel_update_start_index = update_velocities(trajectory, jerk, decel);
  const auto interpolate_start_index = vel_update_start_index > 0 ? vel_update_start_index - 1 : 0;

  auto trajectory_interpolation_util =
    InterpolationTrajectory::Builder{}
      .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
      .set_longitudinal_velocity_interpolator<AkimaSpline>()
      .build(trajectory);
  if (!trajectory_interpolation_util) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Failed to build interpolation trajectory");
    return false;
  }

  const auto dt = trajectory_time_step_;

  if (dt < 1e-3) {
    RCLCPP_ERROR_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Invalid trajectory time step: %f, unable to interpolate trajectory", dt);
    traj_points = std::move(trajectory);
    return true;
  }

  const auto s_max = trajectory_interpolation_util->length();
  const auto interpolate_start_arc_length =
    trajectory_interpolation_util->get_underlying_bases().at(interpolate_start_index);
  trajectory_interpolation_util->align_orientation_with_trajectory_direction();
  traj_points.erase(traj_points.begin() + interpolate_start_index + 1, traj_points.end());
  double s_curr{interpolate_start_arc_length};
  while (s_curr <= s_max) {
    const auto v_curr = traj_points.back().longitudinal_velocity_mps;
    const auto a_curr = traj_points.back().acceleration_mps2;
    const auto t_curr = rclcpp::Duration(traj_points.back().time_from_start);
    if (v_curr < 1e-3) break;

    double ds = v_curr * dt + 0.5 * a_curr * dt * dt;
    if (ds < 1e-3) break;

    s_curr += ds;
    const auto s_clamped = std::min(s_curr, s_max);

    auto p = trajectory_interpolation_util->compute(s_clamped);
    p.acceleration_mps2 = (p.longitudinal_velocity_mps - v_curr) / static_cast<float>(dt);
    p.time_from_start = t_curr + rclcpp::Duration::from_seconds(dt);
    traj_points.push_back(p);
  }

  return true;
}

void ObstacleStop::check_obstacles(const TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::check_obstacles", *get_time_keeper());
  const auto collision_point_objects = check_predicted_objects(traj_points);
  const auto collision_point_pcd = check_pointcloud(traj_points);

  auto get_safety_factor = [&](
                             const geometry_msgs::msg::Point & point,
                             const SafetyFactor::_type_type type) -> SafetyFactor {
    SafetyFactor safety_factor;
    safety_factor.type = type;
    safety_factor.points.emplace_back(point);
    safety_factor.is_safe = false;
    return safety_factor;
  };

  if (collision_point_objects) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Detected collision with object at arc length %f m",
      collision_point_objects->arc_length);
    if (debug_data_.colliding_object) {
      auto safety_factor = get_safety_factor(
        debug_data_.colliding_object->kinematics.initial_pose_with_covariance.pose.position,
        SafetyFactor::OBJECT);
      safety_factor.object_id = debug_data_.colliding_object->object_id;
      safety_factors_.factors.push_back(safety_factor);
    }
  }

  if (collision_point_pcd) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Detected collision with pointcloud at arc length %f m",
      collision_point_pcd->arc_length);
    auto safety_factor = get_safety_factor(collision_point_pcd->point, SafetyFactor::POINTCLOUD);
    safety_factors_.factors.push_back(safety_factor);
  }

  nearest_collision_point_ = std::invoke([&]() -> std::optional<CollisionPoint> {
    const auto is_collision_point_pcd = params_.enable_stop_for_pointcloud && collision_point_pcd;
    const auto is_collision_point_objects =
      params_.enable_stop_for_objects && collision_point_objects;
    if (!is_collision_point_pcd && !is_collision_point_objects) return std::nullopt;
    if (!is_collision_point_pcd) return collision_point_objects.value();
    if (!is_collision_point_objects) return collision_point_pcd.value();
    return collision_point_pcd->arc_length < collision_point_objects->arc_length
             ? collision_point_pcd.value()
             : collision_point_objects.value();
  });
}

std::optional<CollisionPoint> ObstacleStop::check_predicted_objects(
  const TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "ObstacleStop::check_predicted_objects", *get_time_keeper());
  if (
    !params_.use_objects || !data_->predicted_objects || data_->predicted_objects->objects.empty())
    return std::nullopt;
  auto predicted_objects = *data_->predicted_objects;

  object_filter_->filter_objects(predicted_objects);

  PredictedObjects active_objects;
  obstacle_tracker_->update_objects(predicted_objects, active_objects, get_clock()->now());

  autoware_perception_msgs::msg::PredictedObject colliding_object;
  auto collision_point = std::invoke([&]() -> std::optional<CollisionPoint> {
    if (!params_.rss_params.enable) {
      return get_nearest_object_collision(
        traj_points, debug_data_.trajectory_shape, active_objects, debug_data_.target_polygons,
        colliding_object);
    }
    return get_nearest_object_collision(
      traj_points, debug_data_.trajectory_shape, data_->vehicle_info, active_objects,
      object_decel_map_, data_->current_odometry->twist.twist.linear.x,
      params_.nominal_stopping_decel, params_.rss_params.reaction_time,
      params_.rss_params.safety_margin, params_.rss_params.min_vel_th, debug_data_.target_polygons,
      colliding_object);
  });

  if (collision_point) debug_data_.colliding_object = colliding_object;

  return collision_point;
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(const TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::check_pointcloud", *get_time_keeper());
  if (
    !params_.use_pointcloud || !data_->obstacle_pointcloud ||
    data_->obstacle_pointcloud->data.empty()) {
    return std::nullopt;
  }

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*data_->obstacle_pointcloud, *filtered_pointcloud);
  {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::filter_pointcloud", *get_time_keeper());
    const auto & bounding_box = debug_data_.trajectory_shape.bounding_box;
    const auto rel_min_corner = autoware_utils_geometry::inverse_transform_point(
      bounding_box.min_corner().to_3d(), data_->current_odometry->pose.pose);
    const auto rel_max_corner = autoware_utils_geometry::inverse_transform_point(
      bounding_box.max_corner().to_3d(), data_->current_odometry->pose.pose);
    constexpr double buffer = 1.0;
    const auto [min_x, max_x] = std::minmax(rel_min_corner.x(), rel_max_corner.x());
    const auto [min_y, max_y] = std::minmax(rel_min_corner.y(), rel_max_corner.y());
    const auto min_z = params_.pointcloud.min_height;
    const auto max_z = data_->vehicle_info.vehicle_height_m + params_.pointcloud.height_buffer;
    pointcloud_filter_->filter_pointcloud(
      filtered_pointcloud, min_x - buffer, max_x + buffer, min_y - buffer, max_y + buffer, min_z,
      max_z);
  }

  PointCloud::Ptr clustered_points(new PointCloud);
  {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::cluster_pointcloud", *get_time_keeper());
    pointcloud_filter_->cluster_pointcloud(
      filtered_pointcloud, clustered_points, params_.pointcloud.clustering.min_height);
  }

  if (!clustered_points->empty()) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = data_->tf_buffer.lookupTransform(
        "map", data_->obstacle_pointcloud->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_node_ptr()->get_logger(), "no transform found for pointcloud: %s", e.what());
      return std::nullopt;
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*clustered_points, *clustered_points, isometry);
  }

  {
    const auto cluster_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*clustered_points, *cluster_pointcloud);
    cluster_pointcloud->header.stamp = data_->obstacle_pointcloud->header.stamp;
    cluster_pointcloud->header.frame_id = "map";
    debug_data_.cluster_points = cluster_pointcloud;
  }

  if (data_->predicted_objects && !data_->predicted_objects->objects.empty()) {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::filter_pointcloud_by_object", *get_time_keeper());
    pointcloud_filter_->filter_pointcloud_by_object(clustered_points, *data_->predicted_objects);
  }

  PointCloud::Ptr active_points(new PointCloud);
  obstacle_tracker_->update_points(clustered_points, active_points, get_clock()->now());

  std::optional<CollisionPoint> collision_point;
  {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::get_nearest_pcd_collision", *get_time_keeper());
    collision_point = get_nearest_pcd_collision(
      traj_points, debug_data_.trajectory_shape, active_points, debug_data_.target_pcd_points);
  }

  return collision_point;
}

void ObstacleStop::publish_debug_data(const std::string & ns) const
{
  if (debug_data_.cluster_points) pub_clustered_pointcloud_->publish(*debug_data_.cluster_points);

  MarkerArray marker_array;
  const auto ego_z = data_->current_odometry->pose.pose.position.z;
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);
  const auto yellow = autoware_utils::create_marker_color(1.0, 1.0, 0.0, 1.0);
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);

  auto add_point_marker = [&](
                            const geometry_msgs::msg::Point & point, const std::string & ns,
                            const int id, const std_msgs::msg::ColorRGBA & color,
                            const double scale = 0.1) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::SPHERE,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.pose.position = point;
    marker_array.markers.push_back(marker);
  };

  auto add_polygon_marker = [&](
                              const autoware_utils_geometry::Polygon2d & polygon,
                              const std::string & ns, const int id,
                              const std_msgs::msg::ColorRGBA & color) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.1, 0.1, 0.1), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (const auto & p : polygon.outer()) {
      marker.points.push_back(autoware_utils_geometry::create_point(p.x(), p.y(), ego_z));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    marker_array.markers.push_back(marker);
  };

  int id = 0;
  for (const auto & traj_polygon : debug_data_.trajectory_shape.polygon) {
    add_polygon_marker(traj_polygon, ns + "/traj_polygon", id, yellow);
    id++;
  }

  {
    const auto & bounding_box = debug_data_.trajectory_shape.bounding_box;
    Polygon2d polygon;
    polygon.outer().emplace_back(bounding_box.min_corner());
    polygon.outer().emplace_back(bounding_box.min_corner().x(), bounding_box.max_corner().y());
    polygon.outer().emplace_back(bounding_box.max_corner());
    polygon.outer().emplace_back(bounding_box.max_corner().x(), bounding_box.min_corner().y());
    add_polygon_marker(polygon, ns + "/traj_bounding_box", id, white);
    id++;
  }

  for (const auto & target_polygon : debug_data_.target_polygons) {
    add_polygon_marker(target_polygon, ns + "/target_objects", id, magenta);
    id++;
  }

  for (const auto & target_pcd_point : debug_data_.target_pcd_points) {
    add_point_marker(target_pcd_point, ns + "/target_pcd", id, magenta, 0.25);
    id++;
  }

  debug_viz_pub_->publish(marker_array);
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
