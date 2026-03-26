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
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using utils::obstacle_stop::filter_objects_by_type;
using utils::obstacle_stop::filter_objects_by_velocity;
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

  pointcloud_filter_ = std::make_unique<utils::obstacle_stop::PointCloudFilter>(
    params_.pointcloud.voxel_grid_filter.x, params_.pointcloud.voxel_grid_filter.y,
    params_.pointcloud.voxel_grid_filter.z, params_.pointcloud.voxel_grid_filter.min_size,
    params_.pointcloud.clustering.tolerance, params_.pointcloud.clustering.min_size,
    params_.pointcloud.clustering.max_size);

  obstacle_tracker_ = std::make_unique<utils::obstacle_stop::ObstacleTracker>(
    params_.on_time_buffer, params_.off_time_buffer);
}

bool ObstacleStop::is_trajectory_modification_required(const TrajectoryPoints & traj_points)
{
  debug_data_ = DebugData();
  safety_factors_ = SafetyFactorArray{};
  {
    autoware_utils_debug::ScopedTimeTrack st(
      "ObstacleStop::get_trajectory_shape", *get_time_keeper());
    debug_data_.trajectory_shape = get_trajectory_shape(
      traj_points, data_->current_odometry->pose.pose, data_->vehicle_info, params_.lateral_margin);
  }
  const auto collision_point_objects = check_predicted_objects(traj_points);
  const auto collision_point_pcd = check_pointcloud(traj_points);

  if (collision_point_objects) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Detected collision with object at arc length %f m",
      collision_point_objects->arc_length);
    if (debug_data_.colliding_object) {
      SafetyFactor safety_factor;
      safety_factor.object_id = debug_data_.colliding_object->object_id;
      safety_factor.type = SafetyFactor::OBJECT;
      safety_factor.points.emplace_back(
        debug_data_.colliding_object->kinematics.initial_pose_with_covariance.pose.position);
      safety_factor.is_safe = false;
      safety_factors_.factors.push_back(safety_factor);
    }
  }

  if (collision_point_pcd) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Detected collision with pointcloud at arc length %f m",
      collision_point_pcd->arc_length);
    SafetyFactor safety_factor;
    safety_factor.type = SafetyFactor::POINTCLOUD;
    safety_factor.points.emplace_back(collision_point_pcd->point);
    safety_factor.is_safe = false;
    safety_factors_.factors.emplace_back(safety_factor);
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

  debug_data_.active_collision_point =
    nearest_collision_point_ ? nearest_collision_point_->point : geometry_msgs::msg::Point();

  return nearest_collision_point_ != std::nullopt;
}

bool ObstacleStop::modify_trajectory(TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::modify_trajectory", *get_time_keeper());

  if (!enabled_ || !is_trajectory_modification_required(traj_points)) return false;

  if (!nearest_collision_point_) return false;

  return set_stop_point(traj_points);
}

bool ObstacleStop::set_stop_point(TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::set_stop_point", *get_time_keeper());
  const auto stop_margin = params_.stop_margin + data_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length =
    std::max(nearest_collision_point_->arc_length - stop_margin, 0.0);

  auto skip = [&](const std::string & msg) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] %s, skip inserting stop point", msg.c_str());
    return false;
  };

  constexpr double stop_velocity_threshold = 0.01;

  if (params_.duplicate_check_threshold > std::numeric_limits<double>::epsilon()) {
    auto checked_distance = 0.0;
    for (size_t i = 1; i < traj_points.size(); ++i) {
      const auto & curr = traj_points.at(i);
      const auto & prev = traj_points.at(i - 1);
      checked_distance +=
        autoware_utils_geometry::calc_distance2d(curr.pose.position, prev.pose.position);
      if (curr.longitudinal_velocity_mps > stop_velocity_threshold) continue;
      if (checked_distance < target_stop_point_arc_length) {
        return skip("Preceding stop point exists");
      }
      if (
        abs(target_stop_point_arc_length - checked_distance) < params_.duplicate_check_threshold) {
        return skip("Duplicate stop point detected near target stop point");
      }
      if (checked_distance > target_stop_point_arc_length + params_.duplicate_check_threshold)
        break;
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
  const auto stop_index = trajectory.size() - 1;
  auto vel_update_start_index = stop_index;
  for (int i = static_cast<int>(stop_index) - 1; i >= 0; --i) {
    auto & curr = trajectory.at(i);
    const auto & next = trajectory.at(i + 1);
    const auto v_next = next.longitudinal_velocity_mps;
    const auto a_next = std::abs(next.acceleration_mps2);

    const auto ds =
      autoware_utils_geometry::calc_distance2d(curr.pose.position, next.pose.position);
    const auto da = jerk * (ds / std::max<double>(v_next, 0.1));
    const auto a_curr = std::min(a_next + da, decel);
    const auto a_avg = (a_next + a_curr) / 2.0;
    const auto v_curr = std::sqrt(v_next * v_next + 2.0 * a_avg * ds);

    // apply moving average to velocities around connection to smoothen acceleration
    if (v_curr >= curr.longitudinal_velocity_mps) {
      auto j = std::min(i + 3, static_cast<int>(stop_index) - 1);
      auto stop = std::max(i - 3, 1);
      for (; j > stop; --j) {
        auto v_sum = trajectory.at(j - 1).longitudinal_velocity_mps +
                     trajectory.at(j).longitudinal_velocity_mps +
                     trajectory.at(j + 1).longitudinal_velocity_mps;
        trajectory.at(j).longitudinal_velocity_mps = v_sum / 3.0f;
        vel_update_start_index = j;
      }
      break;
    }

    curr.longitudinal_velocity_mps = static_cast<float>(v_curr);
    curr.acceleration_mps2 = static_cast<float>(a_curr) * -1.0f;
    vel_update_start_index = i;
  }
  return vel_update_start_index;
}

bool ObstacleStop::apply_stopping(
  TrajectoryPoints & traj_points, const double target_stop_point_arc_length) const
{
  if (target_stop_point_arc_length < 1e-3) return false;

  auto trajectory = traj_points;

  const auto stop_index = motion_utils::insertStopPoint(target_stop_point_arc_length, trajectory);
  if (!stop_index) return false;

  trajectory.erase(trajectory.begin() + stop_index.value() + 1, trajectory.end());
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
  auto interpolate_start_index = vel_update_start_index > 0 ? vel_update_start_index - 1 : 0;

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

std::optional<CollisionPoint> ObstacleStop::check_predicted_objects(
  const TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "ObstacleStop::check_predicted_objects", *get_time_keeper());
  if (
    !params_.use_objects || !data_->predicted_objects || data_->predicted_objects->objects.empty())
    return std::nullopt;
  auto predicted_objects = *data_->predicted_objects;

  filter_objects_by_type(predicted_objects, params_.objects.object_types);
  filter_objects_by_velocity(predicted_objects, params_.objects.max_velocity_th);

  PredictedObjects active_objects;
  obstacle_tracker_->update_objects(predicted_objects, active_objects);

  autoware_perception_msgs::msg::PredictedObject colliding_object;
  auto collision_point = get_nearest_object_collision(
    traj_points, debug_data_.trajectory_shape.polygon, active_objects, debug_data_.target_polygons,
    colliding_object);
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
    autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::filter_pointcloud", *get_time_keeper());
    const auto & bounding_box = debug_data_.trajectory_shape.bounding_box;
    const auto rel_min_point = autoware_utils::inverse_transform_point(
      bounding_box.min_corner().to_3d(), data_->current_odometry->pose.pose);
    const auto rel_max_point = autoware_utils::inverse_transform_point(
      bounding_box.max_corner().to_3d(), data_->current_odometry->pose.pose);
    const auto min_z = params_.pointcloud.min_height;
    const auto max_z = data_->vehicle_info.vehicle_height_m + params_.pointcloud.height_buffer;
    pointcloud_filter_->filter_pointcloud(
      filtered_pointcloud, rel_min_point.x(), rel_max_point.x(), rel_min_point.y(),
      rel_max_point.y(), min_z, max_z);
  }

  PointCloud::Ptr clustered_points(new PointCloud);
  {
    autoware_utils_debug::ScopedTimeTrack st(
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
    autoware_utils_debug::ScopedTimeTrack st(
      "ObstacleStop::filter_pointcloud_by_object", *get_time_keeper());
    pointcloud_filter_->filter_pointcloud_by_object(clustered_points, *data_->predicted_objects);
  }

  PointCloud::Ptr active_points(new PointCloud);
  obstacle_tracker_->update_points(clustered_points, active_points);

  std::optional<CollisionPoint> collision_point;
  {
    autoware_utils_debug::ScopedTimeTrack st(
      "ObstacleStop::get_nearest_pcd_collision", *get_time_keeper());
    collision_point = get_nearest_pcd_collision(
      traj_points, debug_data_.trajectory_shape.polygon, active_points,
      debug_data_.target_pcd_points);
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
    add_polygon_marker(traj_polygon, ns + "/traj_polygon", id, white);
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
    add_polygon_marker(target_polygon, ns + "/target_objects", id, yellow);
    id++;
  }

  for (const auto & target_pcd_point : debug_data_.target_pcd_points) {
    add_point_marker(target_pcd_point, ns + "/target_pcd", id, yellow, 0.25);
    id++;
  }

  debug_viz_pub_->publish(marker_array);
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
