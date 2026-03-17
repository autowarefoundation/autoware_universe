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

bool ObstacleStop::modify_trajectory(TrajectoryPoints & traj_points)
{
  if (!enabled_ || !is_trajectory_modification_required(traj_points)) return false;

  if (!nearest_collision_point_) return false;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[TM ObstacleStop] Detected collision point at arc length %f m",
    nearest_collision_point_->arc_length);

  return set_stop_point(traj_points);
}

bool ObstacleStop::set_stop_point(TrajectoryPoints & traj_points)
{
  const auto stop_margin = params_.stop_margin_m + data_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length =
    std::max(nearest_collision_point_->arc_length - stop_margin, 0.0);

  auto skip = [&](const std::string & msg) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] %s, skip inserting stop point", msg.c_str());
    return false;
  };

  constexpr double stop_velocity_threshold = 0.01;

  if (params_.duplicate_check_threshold_m > std::numeric_limits<double>::epsilon()) {
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
        abs(target_stop_point_arc_length - checked_distance) <
        params_.duplicate_check_threshold_m) {
        return skip("Duplicate stop point detected near target stop point");
      }
      if (checked_distance > target_stop_point_arc_length + params_.duplicate_check_threshold_m)
        break;
    }
  }

  if (
    target_stop_point_arc_length < params_.arrived_distance_threshold_m ||
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
    traj_points, ego_pose, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});

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
    required_decel, std::abs(params_.nom_stopping_decel), std::abs(params_.max_stopping_decel));

  const auto vel_update_start_index = update_velocities(trajectory, jerk, decel);
  auto interpolate_start_index = vel_update_start_index > 0 ? vel_update_start_index - 1 : 0;

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

  std::vector<double> s_seeds, v_seeds;
  {
    double s_curr{0.0};
    s_seeds.push_back(s_curr);
    v_seeds.push_back(trajectory.front().longitudinal_velocity_mps);
    const size_t required_size = std::max(trajectory.size(), 5ul);
    for (size_t i = 1; i < required_size; ++i) {
      if (i >= trajectory.size()) {
        s_curr += 0.01;
        s_seeds.push_back(s_curr);
        v_seeds.push_back(v_seeds.back());
        continue;
      }
      s_curr += autoware_utils_geometry::calc_distance2d(
        trajectory.at(i - 1).pose.position, trajectory.at(i).pose.position);
      s_seeds.push_back(s_curr);
      v_seeds.push_back(trajectory.at(i).longitudinal_velocity_mps);
    }
  }
  auto velocity_spline = AkimaSpline::Builder().set_bases(s_seeds).set_values(v_seeds).build();
  if (!velocity_spline) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Failed to build velocity spline: %s",
      velocity_spline.error().what.c_str());
    return false;
  }

  const auto dt = trajectory_time_step_;

  const auto s_max = std::min(s_seeds.back(), trajectory_interpolation_util->length());

  const auto interpolate_start_arc_length = s_seeds.at(interpolate_start_index);
  trajectory_interpolation_util->align_orientation_with_trajectory_direction();
  traj_points.erase(traj_points.begin() + interpolate_start_index + 1, traj_points.end());
  double s_curr{interpolate_start_arc_length};
  while (s_curr < s_max) {
    const auto v_curr = traj_points.back().longitudinal_velocity_mps;
    const auto a_curr = traj_points.back().acceleration_mps2;
    const auto t_curr = rclcpp::Duration(traj_points.back().time_from_start);
    if (v_curr < 1e-3) break;

    double ds = v_curr * dt + 0.5 * a_curr * dt * dt;
    if (ds < 1e-3) break;

    s_curr += ds;
    const auto s_clamped = std::min(s_curr, s_max);
    double v_next = velocity_spline->compute(s_clamped);
    double dv_ds = velocity_spline->compute_first_derivative(s_clamped);

    auto p = trajectory_interpolation_util->compute(s_clamped);
    p.longitudinal_velocity_mps = static_cast<float>(v_next);
    p.acceleration_mps2 = static_cast<float>(v_next * dv_ds);
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
  if (!data_->obstacle_pointcloud || data_->obstacle_pointcloud->data.empty()) {
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
      marker.points.push_back(autoware_utils_geometry::create_point(p.x(), p.y(), ego_z));
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

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
