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
using utils::obstacle_stop::cluster_pointcloud;
using utils::obstacle_stop::filter_objects_by_type;
using utils::obstacle_stop::filter_objects_by_velocity;
using utils::obstacle_stop::filter_pointcloud_by_range;
using utils::obstacle_stop::filter_pointcloud_by_voxel_grid;
using utils::obstacle_stop::get_nearest_object_collision;
using utils::obstacle_stop::get_nearest_pcd_collision;
using utils::obstacle_stop::get_trajectory_polygon;
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
      node_ptr, "obstacle_stop");

  pub_cluster_pointcloud_ =
    node_ptr->create_publisher<PointCloud2>("~/obstacle_stop/debug/cluster_points", 1);
  pub_voxel_pointcloud_ =
    node_ptr->create_publisher<PointCloud2>("~/obstacle_stop/debug/voxel_points", 1);
  pub_target_pcd_pointcloud_ =
    node_ptr->create_publisher<PointCloud2>("~/obstacle_stop/debug/target_pcd_points", 1);
  debug_viz_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/obstacle_stop/debug/marker", 1);

  params_ = params.obstacle_stop;
  enabled_ = params.use_obstacle_stop;
  trajectory_time_step_ = params.trajectory_time_step;
}

bool ObstacleStop::is_trajectory_modification_required(const TrajectoryPoints & traj_points)
{
  debug_data_ = DebugData();
  debug_data_.trajectory_polygon = get_trajectory_polygon(
    traj_points, data_->current_odometry->pose.pose, data_->vehicle_info, params_.lateral_margin_m);
  const auto collision_point_pcd = check_pointcloud(traj_points, debug_data_.trajectory_polygon);
  const auto collision_point_objects =
    check_predicted_objects(traj_points, debug_data_.trajectory_polygon);
  if (!collision_point_pcd && !collision_point_objects) return false;

  if (!collision_point_pcd) {
    nearest_collision_point_ = collision_point_objects;
  } else if (!collision_point_objects) {
    nearest_collision_point_ = collision_point_pcd;
  } else {
    nearest_collision_point_ = collision_point_pcd->arc_length < collision_point_objects->arc_length
                                 ? collision_point_pcd
                                 : collision_point_objects;
  }

  debug_data_.active_collision_point =
    nearest_collision_point_ ? nearest_collision_point_->point : geometry_msgs::msg::Point();

  return nearest_collision_point_ != std::nullopt;
}

void ObstacleStop::modify_trajectory(TrajectoryPoints & traj_points)
{
  if (!enabled_ || !is_trajectory_modification_required(traj_points)) return;

  if (!nearest_collision_point_) return;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[TM ObstacleStop] Detected collision point at arc length %f m",
    nearest_collision_point_->arc_length);

  // TODO(Quda): enable this when stopping behavior is improved
  // set_stop_point(traj_points);
}

void ObstacleStop::set_stop_point(TrajectoryPoints & traj_points)
{
  const auto stop_margin = params_.stop_margin_m + data_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length =
    std::max(nearest_collision_point_->arc_length - stop_margin, 0.0);

  auto skip = [&](const std::string & msg) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] %s, skip inserting stop point", msg.c_str());
    return;
  };

  constexpr double stop_velocity_threshold = 0.01;

  auto max_lon_velocity = 0.0f;
  auto max_lon_accel = 0.0f;
  if (params_.duplicate_check_threshold_m > std::numeric_limits<double>::epsilon()) {
    auto checked_distance = 0.0;
    for (size_t i = 1; i < traj_points.size(); ++i) {
      const auto & curr = traj_points.at(i);
      const auto & prev = traj_points.at(i - 1);
      checked_distance += autoware_utils::calc_distance2d(curr.pose.position, prev.pose.position);
      if (max_lon_velocity < curr.longitudinal_velocity_mps) {
        max_lon_velocity = curr.longitudinal_velocity_mps;
        max_lon_accel = curr.acceleration_mps2;
      }
      if (curr.longitudinal_velocity_mps > stop_velocity_threshold) continue;
      if (checked_distance < target_stop_point_arc_length) {
        return skip("Preceding stop point exists");
      }
      if (
        abs(target_stop_point_arc_length - checked_distance) <
        params_.duplicate_check_threshold_m) {
        return skip("Duplicate stop point detected near target stop point");
      }
      if (checked_distance > target_stop_point_arc_length + params_.duplicate_check_threshold_m)
        break;
    }
  }

  // check if ego vehicle is already at the stop point
  const auto & ego_pose = data_->current_odometry->pose.pose;
  const auto ego_lon_vel = data_->current_odometry->twist.twist.linear.x;
  const auto ego_nearest_index = motion_utils::findNearestSegmentIndex(traj_points, ego_pose);
  const auto ego_arc_length =
    motion_utils::calcSignedArcLength(traj_points, 0.0, ego_pose.position);
  if (
    ego_nearest_index && ego_lon_vel < ego_low_speed_threshold &&
    target_stop_point_arc_length - ego_arc_length < ego_stop_point_distance_threshold) {
    traj_points.clear();
    TrajectoryPoint stop_point;
    stop_point.pose = ego_pose;
    stop_point.longitudinal_velocity_mps = 0.0;
    stop_point.acceleration_mps2 = 0.0;
    stop_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    traj_points.push_back(stop_point);
    stop_point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_step_);
    traj_points.push_back(stop_point);
  } else if (!apply_stopping(
               traj_points, target_stop_point_arc_length, max_lon_velocity, max_lon_accel)) {
    return skip("Failed to apply smooth stopping");
  }

  const auto & stop_pose = traj_points.back().pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM ObstacleStop] Inserted stop point at arc length %f m", target_stop_point_arc_length);
}

bool ObstacleStop::apply_stopping(
  TrajectoryPoints & traj_points, const double target_stop_point_arc_length,
  const float max_lon_velocity, const float max_lon_accel) const
{
  const auto [stopping_distance, stopping_accel, stopping_jerk] = std::invoke([&]() {
    auto distance = motion_utils::calculate_stop_distance(
      max_lon_velocity, max_lon_accel, params_.nom_stopping_decel, params_.nom_stopping_jerk);
    if (distance && *distance < target_stop_point_arc_length)
      return std::make_tuple(*distance, params_.nom_stopping_decel, params_.nom_stopping_jerk);
    distance = motion_utils::calculate_stop_distance(
      max_lon_velocity, max_lon_accel, params_.max_stopping_decel, params_.max_stopping_jerk);
    if (distance && *distance < target_stop_point_arc_length)
      return std::make_tuple(*distance, params_.max_stopping_decel, params_.max_stopping_jerk);
    return std::make_tuple(
      target_stop_point_arc_length, params_.max_stopping_decel, params_.max_stopping_jerk);
  });

  if (stopping_distance < 1e-3) return false;

  auto slow_down_start_arc_length = std::max(target_stop_point_arc_length - stopping_distance, 0.0);
  auto slow_down_start_point =
    motion_utils::calcLongitudinalOffsetPoint(traj_points, 0.0, slow_down_start_arc_length);
  if (!slow_down_start_point) return false;
  auto slow_down_start_point_index =
    motion_utils::findNearestSegmentIndex(traj_points, slow_down_start_point.value());
  slow_down_start_arc_length =
    motion_utils::calcSignedArcLength(traj_points, 0.0, slow_down_start_point_index);

  auto trajectory_interpolation_util =
    InterpolationTrajectory::Builder{}
      .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
      .build(traj_points);
  if (!trajectory_interpolation_util) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Failed to build interpolation trajectory");
    return false;
  }

  trajectory_interpolation_util->align_orientation_with_trajectory_direction();
  traj_points.erase(traj_points.begin() + slow_down_start_point_index + 1, traj_points.end());
  double s_curr{slow_down_start_arc_length};
  const auto dt = trajectory_time_step_;
  while (s_curr < trajectory_interpolation_util->length()) {
    const auto a_curr = traj_points.back().acceleration_mps2;
    const auto v_curr = traj_points.back().longitudinal_velocity_mps;
    const auto t_curr = rclcpp::Duration(traj_points.back().time_from_start);

    if (v_curr < 1e-3) break;

    const auto a_next = std::max(a_curr + stopping_jerk * dt, stopping_accel);
    const auto v_next = std::max(v_curr + a_curr * dt + 0.5 * stopping_jerk * dt * dt, 0.0);

    double ds = v_curr * dt + 0.5 * a_curr * dt * dt + (1.0 / 6.0) * stopping_jerk * dt * dt * dt;
    s_curr += ds;
    auto p = trajectory_interpolation_util->compute(s_curr);
    p.acceleration_mps2 = static_cast<float>(a_next);
    p.longitudinal_velocity_mps = static_cast<float>(v_next);
    p.time_from_start = t_curr + rclcpp::Duration::from_seconds(dt);
    traj_points.push_back(p);
  }

  return true;
}

std::optional<CollisionPoint> ObstacleStop::check_predicted_objects(
  const TrajectoryPoints & traj_points, const MultiPolygon2d & trajectory_polygon)
{
  if (!data_->predicted_objects || data_->predicted_objects->objects.empty()) return std::nullopt;
  auto predicted_objects = *data_->predicted_objects;

  filter_objects_by_type(predicted_objects, params_.objects.object_types);
  filter_objects_by_velocity(predicted_objects, params_.objects.max_velocity_th);

  return get_nearest_object_collision(traj_points, trajectory_polygon, predicted_objects);
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(
  const TrajectoryPoints & traj_points, const MultiPolygon2d & trajectory_polygon)
{
  if (!data_->obstacle_pointcloud || data_->obstacle_pointcloud->data.empty()){
    return std::nullopt;
  }

  const auto & pointcloud = data_->obstacle_pointcloud;

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*pointcloud, *filtered_pointcloud);

  const auto & p = params_.pointcloud;

  const auto max_height = data_->vehicle_info.vehicle_height_m + p.height_buffer;
  filter_pointcloud_by_range(filtered_pointcloud, p.detection_range, p.min_height, max_height);

  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = data_->tf_buffer.lookupTransform(
        "map", pointcloud->header.frame_id, pointcloud->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_node_ptr()->get_logger(), "no transform found for pointcloud: %s", e.what());
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*filtered_pointcloud, *filtered_pointcloud, isometry);
  }

  const auto & p_voxel = p.voxel_grid_filter;
  filter_pointcloud_by_voxel_grid(
    filtered_pointcloud, p_voxel.x, p_voxel.y, p_voxel.z, p_voxel.min_size);
  {
    const auto voxel_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*filtered_pointcloud, *voxel_pointcloud);
    voxel_pointcloud->header.stamp = pointcloud->header.stamp;
    voxel_pointcloud->header.frame_id = "map";
    debug_data_.voxel_points = voxel_pointcloud;
  }

  const auto & p_cluster = p.clustering;
  const auto height_offset = data_->current_odometry->pose.pose.position.z;
  PointCloud::Ptr clustered_points(new PointCloud);
  cluster_pointcloud(
    filtered_pointcloud, clustered_points, p_cluster.min_size, p_cluster.max_size,
    p_cluster.tolerance, p_cluster.min_height, height_offset);
  {
    const auto cluster_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*clustered_points, *cluster_pointcloud);
    cluster_pointcloud->header.stamp = pointcloud->header.stamp;
    cluster_pointcloud->header.frame_id = "map";
    debug_data_.cluster_points = cluster_pointcloud;
  }

  return get_nearest_pcd_collision(traj_points, trajectory_polygon, clustered_points);
}

void ObstacleStop::publish_debug_data(const std::string & ns) const
{
  if (debug_data_.cluster_points) pub_cluster_pointcloud_->publish(*debug_data_.cluster_points);
  if (debug_data_.voxel_points) pub_voxel_pointcloud_->publish(*debug_data_.voxel_points);
  if (debug_data_.target_pcd_points)
    pub_target_pcd_pointcloud_->publish(*debug_data_.target_pcd_points);

  MarkerArray marker_array;
  const auto ego_z = data_->current_odometry->pose.pose.position.z;
  const auto red = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0);
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);

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
      "map", get_clock()->now(), ns, id, Marker::LINE_LIST,
      autoware_utils::create_marker_scale(0.1, 0.1, 0.1), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (const auto & p : polygon.outer()) {
      marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), ego_z));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    marker_array.markers.push_back(marker);
  };

  int id = 0;
  for (const auto & traj_polygon : debug_data_.trajectory_polygon) {
    add_polygon_marker(traj_polygon, ns, id, white);
    id++;
  }

  if (nearest_collision_point_) {
    add_point_marker(debug_data_.active_collision_point, ns, id, red, 0.5);
    id++;
  }

  debug_viz_pub_->publish(marker_array);
}

// TODO(Quda): move logic to minimum rule based planner
// void ObstacleStop::update_collision_points_buffer(
//   const TrajectoryPoints & traj_points, const std::optional<CollisionPoint> & collision_point)
// {
//   constexpr double close_distance_threshold = 0.5;

//   const auto now = get_clock()->now();

//   std::optional<double> closest_distance_diff = std::nullopt;
//   collision_points_buffer_.erase(
//     std::remove_if(
//       collision_points_buffer_.begin(), collision_points_buffer_.end(),
//       [&](CollisionPoint & cp) {
//         if (cp.is_active && (now - cp.start_time).seconds() > params_.off_time_buffer) return
//         true; if (!collision_point && !cp.is_active) return true;

//         cp.arc_length = motion_utils::calcSignedArcLength(traj_points, 0LU, cp.point);
//         const auto distance_diff = collision_point ? collision_point->arc_length - cp.arc_length
//                                                    : std::numeric_limits<double>::max();
//         const bool is_close = abs(distance_diff) < close_distance_threshold;

//         if (!cp.is_active && !is_close) return true;

//         if (!cp.is_active && (now - cp.start_time).seconds() > params_.on_time_buffer) {
//           cp.is_active = true;
//           cp.start_time = now;
//         }

//         if (
//           is_close &&
//           (!closest_distance_diff || abs(distance_diff) < abs(*closest_distance_diff))) {
//           closest_distance_diff = distance_diff;
//         }

//         return false;
//       }),
//     collision_points_buffer_.end());

//   if (!collision_point) return;

//   constexpr double eps = 1e-3;
//   if (collision_points_buffer_.empty() || !closest_distance_diff) {
//     collision_points_buffer_.emplace_back(*collision_point, now, params_.on_time_buffer < eps);
//     return;
//   }

//   auto nearest_collision_point_it = std::find_if(
//     collision_points_buffer_.begin(), collision_points_buffer_.end(),
//     [&](const CollisionPoint & cp) {
//       return abs(cp.arc_length - collision_point->arc_length) < abs(*closest_distance_diff) +
//       eps;
//     });

//   if (nearest_collision_point_it == collision_points_buffer_.end()) return;

//   if (*closest_distance_diff < 0.0) {
//     nearest_collision_point_it->point = collision_point->point;
//     nearest_collision_point_it->arc_length = collision_point->arc_length;
//   }

//   if (nearest_collision_point_it->is_active) {
//     nearest_collision_point_it->start_time = now;
//   }
// }

// TODO(Quda): move logic to minimum rule based planner
// std::optional<CollisionPoint> ObstacleStop::get_nearest_collision_point() const
// {
//   std::optional<CollisionPoint> nearest_collision_point = std::nullopt;
//   if (collision_points_buffer_.empty()) return std::nullopt;

//   auto minimum_arc_length = std::numeric_limits<double>::max();
//   for (const auto & cp : collision_points_buffer_) {
//     if (cp.is_active > minimum_arc_length || !cp.is_active) continue;
//     nearest_collision_point = cp;
//     minimum_arc_length = cp.arc_length;
//   }
//   return nearest_collision_point;
// }

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
