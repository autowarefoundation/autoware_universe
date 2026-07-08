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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/obstacle_stop.hpp"

#include "autoware/trajectory_processor/trajectory_modifier_utils/obstacle_stop_utils.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
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

void ObstacleStop::set_up_params()
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_obstacle_stop");

  pub_clustered_pointcloud_ =
    node_ptr->create_publisher<PointCloud2>("~/obstacle_stop/debug/cluster_points", 1);
  pub_filtered_pointcloud_ =
    node_ptr->create_publisher<PointCloud2>("~/obstacle_stop/debug/filtered_points", 1);
  debug_viz_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/obstacle_stop/debug/marker", 1);
  pub_debug_text_ = node_ptr->create_publisher<StringStamped>("~/obstacle_stop/debug/text", 1);
}

void ObstacleStop::update_params(const TrajectoryModifierParams & params)
{
  params_ = params.obstacle_stop;
  stopping_params_ = params.stopping_constraints;
  enabled_ = params.use_obstacle_stop;
  trajectory_time_step_ = params.trajectory_time_step;

  {
    auto & p = params_.rss_params;
    p.ego_decel = std::clamp(
      p.ego_decel, stopping_params_.nominal_deceleration, stopping_params_.maximum_deceleration);
  }

  {
    const auto & p = params_.pointcloud;
    if (pointcloud_filter_) {
      pointcloud_filter_->set_params(
        p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z,
        p.voxel_grid_filter.min_size, p.clustering.tolerance, p.clustering.min_size,
        p.clustering.max_size);
    } else {
      pointcloud_filter_ = std::make_unique<utils::obstacle_stop::PointCloudFilter>(
        p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z,
        p.voxel_grid_filter.min_size, p.clustering.tolerance, p.clustering.min_size,
        p.clustering.max_size);
    }
  }

  {
    const auto & p = params_.objects;
    if (object_filter_) {
      object_filter_->set_params(
        p.object_types, p.max_velocity_th, p.stopped_velocity_th, p.max_lateral_velocity_th,
        p.safety_buffer);
    } else {
      object_filter_ = std::make_unique<utils::obstacle_stop::ObjectFilter>(
        p.object_types, p.max_velocity_th, p.stopped_velocity_th, p.max_lateral_velocity_th,
        p.safety_buffer);
    }
  }

  {
    const auto & p = params_.obstacle_tracking;
    if (obstacle_tracker_) {
      obstacle_tracker_->set_params(
        p.on_time_buffer, p.off_time_buffer, p.object_distance_th, p.object_yaw_th,
        p.pcd_distance_th, p.grace_period);
    } else {
      obstacle_tracker_ = std::make_unique<utils::obstacle_stop::ObstacleTracker>(
        p.on_time_buffer, p.off_time_buffer, p.object_distance_th, p.object_yaw_th,
        p.pcd_distance_th, p.grace_period);
    }
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
      {utils::obstacle_stop::ObjectType::PEDESTRIAN, p.object_decel.pedestrian},
      {utils::obstacle_stop::ObjectType::ANIMAL, p.object_decel.animal}};
  }
}

bool ObstacleStop::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const InputData & input)
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
      traj_points, input.current_odometry->pose.pose, context_->vehicle_info,
      input.current_odometry->twist.twist.linear.x,
      input.current_acceleration->accel.accel.linear.x, stopping_params_.nominal_deceleration,
      stopping_params_.jerk_limit, params_.stop_margin, params_.lateral_margin);
  }

  check_obstacles(traj_points, input);

  debug_data_.active_collision_point =
    nearest_collision_point_ ? nearest_collision_point_->point : geometry_msgs::msg::Point();
  debug_data_.ego_z = input.current_odometry->pose.pose.position.z;

  const bool is_safe = nearest_collision_point_ == std::nullopt;

  publish_debug_string(is_safe);

  return !is_safe;
}

bool ObstacleStop::modify_trajectory(TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::modify_trajectory", *get_time_keeper());

  if (!enabled_ || traj_points.size() < 2) return false;

  auto trajectory = traj_points;
  utils::obstacle_stop::trim_trajectory_and_remove_duplicates(trajectory);
  if (trajectory.size() < 2) return false;

  if (!is_trajectory_modification_required(trajectory, input)) return false;

  if (!nearest_collision_point_) return false;

  traj_points = std::move(trajectory);

  return set_stop_point(traj_points, input);
}

bool ObstacleStop::set_stop_point(TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::set_stop_point", *get_time_keeper());

  const auto stop_margin = params_.stop_margin + context_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length = utils::clamp_stop_point_arc_length(
    nearest_collision_point_->arc_length - stop_margin,
    debug_data_.trajectory_shape.trajectory_length, input.current_odometry->twist.twist.linear.x,
    input.current_acceleration->accel.accel.linear.x, stopping_params_.maximum_deceleration,
    stopping_params_.jerk_limit);

  if (utils::stop_point_exists(
        traj_points, target_stop_point_arc_length, params_.duplicate_check_threshold)) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Preceding (or duplicate) stop point exists, skip inserting stop point");
    return false;
  }

  if (
    target_stop_point_arc_length < stopping_params_.arrived_distance_threshold ||
    !utils::insert_stop_point(
      traj_points, target_stop_point_arc_length, debug_data_.trajectory_shape.trajectory_length)) {
    utils::replace_trajectory_with_stop_point(
      traj_points, input.current_odometry->pose.pose, trajectory_time_step_);
  }

  const auto & stop_pose = traj_points.back().pose;
  const auto & ego_pose = input.current_odometry->pose.pose;
  auto distance =
    motion_utils::calcSignedArcLength(traj_points, ego_pose.position, stop_pose.position);
  if (std::isnan(distance)) distance = 0.0;
  planning_factor_interface_->add(distance, stop_pose, PlanningFactor::STOP, safety_factors_);

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM ObstacleStop] Inserted stop point at arc length %f m", target_stop_point_arc_length);
  return true;
}

void ObstacleStop::check_obstacles(const TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::check_obstacles", *get_time_keeper());
  const auto collision_point_objects = check_predicted_objects(traj_points, input);
  const auto collision_point_pcd = check_pointcloud(traj_points, input);

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
  const TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "ObstacleStop::check_predicted_objects", *get_time_keeper());
  if (!params_.use_objects || !input.predicted_objects) return std::nullopt;

  debug_data_.filtered_objects = *input.predicted_objects;

  object_filter_->filter_objects(debug_data_.filtered_objects);

  PredictedObjects active_objects;
  obstacle_tracker_->update_objects(
    debug_data_.filtered_objects, active_objects, get_clock()->now());

  object_filter_->filter_by_target_area(
    active_objects, traj_points, context_->vehicle_info, debug_data_.trajectory_shape.polygon,
    debug_data_.target_polygons);

  autoware_perception_msgs::msg::PredictedObject colliding_object;
  auto collision_point = std::invoke([&]() -> std::optional<CollisionPoint> {
    if (!params_.rss_params.enable) {
      return get_nearest_object_collision(traj_points, active_objects, colliding_object);
    }
    return get_nearest_object_collision(
      traj_points, context_->vehicle_info, active_objects, object_decel_map_,
      params_.rss_params.ego_decel, params_.rss_params.reaction_time,
      params_.rss_params.safety_margin, params_.objects.stopped_velocity_th,
      params_.rss_params.lookahead_horizon, colliding_object);
  });

  if (collision_point) debug_data_.colliding_object = colliding_object;

  return collision_point;
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(
  const TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::check_pointcloud", *get_time_keeper());
  if (!params_.use_pointcloud || !input.obstacle_pointcloud) return std::nullopt;

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*input.obstacle_pointcloud, *filtered_pointcloud);
  {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::filter_pointcloud", *get_time_keeper());
    const auto & bounding_box = debug_data_.trajectory_shape.bounding_box;
    const auto rel_min_corner = autoware_utils_geometry::inverse_transform_point(
      bounding_box.min_corner().to_3d(), input.current_odometry->pose.pose);
    const auto rel_max_corner = autoware_utils_geometry::inverse_transform_point(
      bounding_box.max_corner().to_3d(), input.current_odometry->pose.pose);
    constexpr double buffer = 1.0;
    const auto [min_x, max_x] = std::minmax(rel_min_corner.x(), rel_max_corner.x());
    const auto [min_y, max_y] = std::minmax(rel_min_corner.y(), rel_max_corner.y());
    const auto min_z = params_.pointcloud.min_height;
    const auto max_z = context_->vehicle_info.vehicle_height_m + params_.pointcloud.height_buffer;
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
      transform_stamped = context_->tf_buffer->lookupTransform(
        "map", input.obstacle_pointcloud->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_node_ptr()->get_logger(), "no transform found for pointcloud: %s", e.what());
      return std::nullopt;
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*clustered_points, *clustered_points, isometry);
  }

  {
    const auto cluster_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    const auto filtered_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*clustered_points, *cluster_pointcloud_msg);
    pcl::toROSMsg(*filtered_pointcloud, *filtered_pointcloud_msg);
    cluster_pointcloud_msg->header.stamp = input.obstacle_pointcloud->header.stamp;
    cluster_pointcloud_msg->header.frame_id = "map";
    filtered_pointcloud_msg->header.stamp = input.obstacle_pointcloud->header.stamp;
    filtered_pointcloud_msg->header.frame_id = "map";
    debug_data_.cluster_points = cluster_pointcloud_msg;
    debug_data_.filtered_points = filtered_pointcloud_msg;
  }

  if (input.predicted_objects && !input.predicted_objects->objects.empty()) {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::filter_pointcloud_by_object", *get_time_keeper());
    pointcloud_filter_->filter_pointcloud_by_object(clustered_points, *input.predicted_objects);
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

void ObstacleStop::publish_debug_string(bool is_safe) const
{
  const auto filtered_pcd_size =
    debug_data_.filtered_points ? debug_data_.filtered_points->data.size() : 0;
  const auto cluster_pcd_size =
    debug_data_.cluster_points ? debug_data_.cluster_points->data.size() : 0;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << std::boolalpha;
  ss << "OBSTACLE STOP MODIFIER: " << "\n";
  ss << "\t\t" << "SAFE: " << is_safe << "\n";
  ss << "\t\t" << "OBJECTS: " << debug_data_.filtered_objects.objects.size() << " --> "
     << debug_data_.target_polygons.size() << "\n";
  ss << "\t\t" << "POINTCLOUD: " << filtered_pcd_size << " --> " << cluster_pcd_size << " --> "
     << debug_data_.target_pcd_points.size() << "\n";
  if (nearest_collision_point_) {
    ss << "\t\t" << "DISTANCE TO COLLISION: " << nearest_collision_point_->arc_length << " m"
       << "\n";
    ss << "\t\t"
       << "OBSTACLE TYPE: " << (nearest_collision_point_->is_dynamic ? "DYNAMIC" : "STATIC")
       << "\n";
  }

  StringStamped string_stamp;
  string_stamp.stamp = get_clock()->now();
  string_stamp.data = ss.str();
  pub_debug_text_->publish(string_stamp);
}

void ObstacleStop::publish_debug_data(const std::string & ns) const
{
  if (debug_data_.filtered_points) pub_filtered_pointcloud_->publish(*debug_data_.filtered_points);
  if (debug_data_.cluster_points) pub_clustered_pointcloud_->publish(*debug_data_.cluster_points);

  MarkerArray marker_array;
  const auto ego_z = debug_data_.ego_z;
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
  autoware::trajectory_processor::plugin::PluginBase)
