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

#include "obstacle_stop.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/trajectory_processor/trajectory_modifier_utils/obstacle_stop_utils.hpp>
#include <autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{
using autoware::trajectory_modifier::utils::clamp_stop_point_arc_length;
using autoware::trajectory_modifier::utils::insert_stop_point;
using autoware::trajectory_modifier::utils::obstacle_stop::get_nearest_object_collision;
using autoware::trajectory_modifier::utils::obstacle_stop::get_nearest_pcd_collision;
using autoware::trajectory_modifier::utils::obstacle_stop::get_trajectory_shape;
using autoware::trajectory_modifier::utils::obstacle_stop::PointCloud;

void ObstacleStop::on_initialize(const MinimumRuleBasedPlannerParams & params)
{
  params_ = params.obstacle_stop;

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      get_node_ptr(), "backup_planner_obstacle_stop");

  pointcloud_filter_ =
    std::make_unique<trajectory_modifier::utils::obstacle_stop::PointCloudFilter>(
      params_.pointcloud.voxel_grid_filter.x, params_.pointcloud.voxel_grid_filter.y,
      params_.pointcloud.voxel_grid_filter.z, params_.pointcloud.voxel_grid_filter.min_size,
      params_.pointcloud.clustering.tolerance, params_.pointcloud.clustering.min_size,
      params_.pointcloud.clustering.max_size);

  object_filter_ = std::make_unique<trajectory_modifier::utils::obstacle_stop::ObjectFilter>(
    params_.objects.object_types, params_.objects.max_velocity_th,
    params_.objects.stopped_velocity_th, params_.objects.max_lateral_velocity_th,
    params_.objects.safety_buffer);

  pub_clustered_pointcloud_ =
    get_node_ptr()->create_publisher<PointCloud2>("~/obstacle_stop/debug/cluster_points", 1);
  debug_viz_pub_ = get_node_ptr()->create_publisher<MarkerArray>("~/obstacle_stop/debug/marker", 1);
  pub_debug_text_ =
    get_node_ptr()->create_publisher<StringStamped>("~/obstacle_stop/debug/text", 1);

  update_object_decel_map();
}

void ObstacleStop::run(TrajectoryPoints & traj_points, const ModifierData & data)
{
  if (!params_.enable) return;

  const auto detected = is_obstacle_detected(traj_points, data);
  publish_debug_string(!detected);
  publish_debug_data("obstacle_stop", data);

  if (!detected) return;
  if (!nearest_collision_point_) return;

  set_stop_point(traj_points, data);
}

bool ObstacleStop::is_obstacle_detected(
  const TrajectoryPoints & traj_points, const ModifierData & data)
{
  debug_data_ = DebugData();
  safety_factors_ = SafetyFactorArray{};
  debug_data_.trajectory_shape = get_trajectory_shape(
    traj_points, data.odometry_ptr->pose.pose, context_->vehicle_info,
    data.odometry_ptr->twist.twist.linear.x, data.acceleration_ptr->accel.accel.linear.x,
    params_.nominal_stopping_decel, params_.stopping_jerk, params_.stop_margin,
    params_.lateral_margin);
  const auto collision_point_pcd = check_pointcloud(traj_points, data);
  update_collision_points_buffer(collision_points_buffer_.pcd, traj_points, collision_point_pcd);
  const auto collision_point_objects = check_predicted_objects(traj_points, data);
  update_collision_points_buffer(
    collision_points_buffer_.objects, traj_points, collision_point_objects);

  const auto nearest_pcd_collision_point =
    get_nearest_collision_point(collision_points_buffer_.pcd);
  const auto nearest_objects_collision_point =
    get_nearest_collision_point(collision_points_buffer_.objects);

  const auto make_safety_factor =
    [](const geometry_msgs::msg::Point & point, const SafetyFactor::_type_type type) {
      SafetyFactor factor;
      factor.type = type;
      factor.points.emplace_back(point);
      factor.is_safe = false;
      return factor;
    };
  if (nearest_objects_collision_point) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 500,
      "[Backup Planner ObstacleStop] Detected collision with object at arc length %f m",
      nearest_objects_collision_point->arc_length);
    if (debug_data_.colliding_object) {
      auto factor = make_safety_factor(
        debug_data_.colliding_object->kinematics.initial_pose_with_covariance.pose.position,
        SafetyFactor::OBJECT);
      factor.object_id = debug_data_.colliding_object->object_id;
      safety_factors_.factors.push_back(factor);
    }
  }

  if (nearest_pcd_collision_point) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 500,
      "[Backup Planner ObstacleStop] Detected collision with pointcloud at arc length %f m",
      nearest_pcd_collision_point->arc_length);
    safety_factors_.factors.push_back(
      make_safety_factor(nearest_pcd_collision_point->point, SafetyFactor::POINTCLOUD));
  }

  nearest_collision_point_ = std::invoke([&]() -> std::optional<CollisionPoint> {
    const auto is_collision_point_pcd =
      params_.enable_stop_for_pointcloud && nearest_pcd_collision_point;
    const auto is_collision_point_objects =
      params_.enable_stop_for_objects && nearest_objects_collision_point;
    if (!is_collision_point_pcd && !is_collision_point_objects) return std::nullopt;
    if (!is_collision_point_pcd) return nearest_objects_collision_point.value();
    if (!is_collision_point_objects) return nearest_pcd_collision_point.value();
    return nearest_pcd_collision_point->arc_length < nearest_objects_collision_point->arc_length
             ? nearest_pcd_collision_point.value()
             : nearest_objects_collision_point.value();
  });

  debug_data_.active_collision_point =
    nearest_collision_point_ ? nearest_collision_point_->point : geometry_msgs::msg::Point();

  return nearest_collision_point_ != std::nullopt;
}

void ObstacleStop::set_stop_point(TrajectoryPoints & traj_points, const ModifierData & data)
{
  const auto stop_margin = params_.stop_margin + context_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length = clamp_stop_point_arc_length(
    nearest_collision_point_->arc_length - stop_margin,
    debug_data_.trajectory_shape.trajectory_length, data.odometry_ptr->twist.twist.linear.x,
    data.acceleration_ptr->accel.accel.linear.x, params_.nominal_stopping_decel,
    params_.stopping_jerk);

  if (!insert_stop_point(
        traj_points, target_stop_point_arc_length, debug_data_.trajectory_shape.trajectory_length))
    return;

  if (
    target_stop_point_arc_length < params_.arrived_distance_threshold ||
    !insert_stop_point(
      traj_points, target_stop_point_arc_length, debug_data_.trajectory_shape.trajectory_length)) {
    auto p = traj_points.front();
    traj_points.clear();
    p.longitudinal_velocity_mps = 0.0;
    p.acceleration_mps2 = 0.0;
    p.time_from_start = rclcpp::Duration::from_seconds(0.0);
    traj_points.push_back(p);
    p.time_from_start = rclcpp::Duration::from_seconds(0.1);
    traj_points.push_back(p);
  }

  const auto & stop_pose = traj_points.back().pose;
  const auto & ego_pose = data.odometry_ptr->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, stop_pose, PlanningFactor::STOP, safety_factors_);

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[Backup Planner ObstacleStop] Inserted stop point at arc length %f m",
    target_stop_point_arc_length);
}

std::optional<CollisionPoint> ObstacleStop::check_predicted_objects(
  const TrajectoryPoints & traj_points, const ModifierData & data)
{
  if (!params_.use_objects) return std::nullopt;

  if (!data.predicted_objects_ptr || data.predicted_objects_ptr->objects.empty())
    return std::nullopt;
  auto predicted_objects = *data.predicted_objects_ptr;

  object_filter_->filter_objects(predicted_objects);
  object_filter_->filter_by_target_area(
    predicted_objects, traj_points, context_->vehicle_info, debug_data_.trajectory_shape.polygon,
    debug_data_.target_polygons);

  autoware_perception_msgs::msg::PredictedObject colliding_object;
  // TODO(Quda): Port the latest modifier logic
  std::optional<CollisionPoint> collision_point;
  if (params_.rss_params.enable) {
    collision_point = get_nearest_object_collision(
      traj_points, context_->vehicle_info, predicted_objects, object_decel_map_,
      params_.rss_params.ego_decel, params_.rss_params.reaction_time,
      params_.rss_params.safety_margin, params_.objects.stopped_velocity_th,
      params_.rss_params.lookahead_horizon, colliding_object);
  } else {
    collision_point =
      get_nearest_object_collision(traj_points, predicted_objects, colliding_object);
  }

  if (collision_point) debug_data_.colliding_object = colliding_object;
  return collision_point;
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(
  const TrajectoryPoints & traj_points, const ModifierData & data)
{
  if (!params_.use_pointcloud) return std::nullopt;

  if (!data.obstacle_pointcloud_ptr || data.obstacle_pointcloud_ptr->data.empty())
    return std::nullopt;

  const auto & pointcloud = data.obstacle_pointcloud_ptr;

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*pointcloud, *filtered_pointcloud);

  {
    const auto & bounding_box = debug_data_.trajectory_shape.bounding_box;
    const auto rel_min_point = autoware_utils_geometry::inverse_transform_point(
      bounding_box.min_corner().to_3d(), data.odometry_ptr->pose.pose);
    const auto rel_max_point = autoware_utils_geometry::inverse_transform_point(
      bounding_box.max_corner().to_3d(), data.odometry_ptr->pose.pose);
    const auto min_z = params_.pointcloud.min_height;
    const auto max_z = context_->vehicle_info.vehicle_height_m + params_.pointcloud.height_buffer;
    pointcloud_filter_->filter_pointcloud(
      filtered_pointcloud, rel_min_point.x(), rel_max_point.x(), rel_min_point.y(),
      rel_max_point.y(), min_z, max_z);
  }

  PointCloud::Ptr clustered_points(new PointCloud);
  pointcloud_filter_->cluster_pointcloud(
    filtered_pointcloud, clustered_points, params_.pointcloud.clustering.min_height);

  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = context_->tf_buffer.lookupTransform(
        "map", pointcloud->header.frame_id, pointcloud->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_node_ptr()->get_logger(), "no transform found for pointcloud: %s", e.what());
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*filtered_pointcloud, *filtered_pointcloud, isometry);
  }

  {
    const auto cluster_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*clustered_points, *cluster_pointcloud);
    cluster_pointcloud->header.stamp = pointcloud->header.stamp;
    cluster_pointcloud->header.frame_id = "map";
    debug_data_.cluster_points = cluster_pointcloud;
  }

  if (data.predicted_objects_ptr && !data.predicted_objects_ptr->objects.empty()) {
    pointcloud_filter_->filter_pointcloud_by_object(clustered_points, *data.predicted_objects_ptr);
  }

  return get_nearest_pcd_collision(
    traj_points, debug_data_.trajectory_shape, clustered_points, debug_data_.target_pcd_points);
}

void ObstacleStop::update_collision_points_buffer(
  std::vector<CollisionPoint> & collision_points_buffer, const TrajectoryPoints & traj_points,
  const std::optional<CollisionPoint> & collision_point)
{
  constexpr double close_distance_threshold = 1.0;

  const auto now = get_clock()->now();

  std::optional<double> closest_distance_diff = std::nullopt;
  collision_points_buffer.erase(
    std::remove_if(
      collision_points_buffer.begin(), collision_points_buffer.end(),
      [&](CollisionPoint & cp) {
        if (cp.is_active && (now - cp.start_time).seconds() > params_.off_time_buffer) return true;

        if (!collision_point && !cp.is_active) return true;

        cp.arc_length = motion_utils::calcSignedArcLength(traj_points, 0LU, cp.point);
        const auto distance_diff = collision_point ? collision_point->arc_length - cp.arc_length
                                                   : std::numeric_limits<double>::max();
        const bool is_close = abs(distance_diff) < close_distance_threshold;

        if (!cp.is_active && !is_close) return true;

        if (!cp.is_active && (now - cp.start_time).seconds() > params_.on_time_buffer) {
          cp.is_active = true;
          cp.start_time = now;
        }

        if (
          is_close &&
          (!closest_distance_diff || abs(distance_diff) < abs(*closest_distance_diff))) {
          closest_distance_diff = distance_diff;
        }

        return false;
      }),
    collision_points_buffer.end());

  if (!collision_point) return;

  constexpr double eps = 1e-3;
  if (collision_points_buffer.empty() || !closest_distance_diff) {
    collision_points_buffer.emplace_back(*collision_point, now, params_.on_time_buffer < eps);
    return;
  }

  auto nearest_collision_point_it = std::find_if(
    collision_points_buffer.begin(), collision_points_buffer.end(), [&](const CollisionPoint & cp) {
      return abs(cp.arc_length - collision_point->arc_length) < abs(*closest_distance_diff) + eps;
    });

  if (nearest_collision_point_it == collision_points_buffer.end()) return;

  if (*closest_distance_diff < 0.0) {
    nearest_collision_point_it->point = collision_point->point;
    nearest_collision_point_it->arc_length = collision_point->arc_length;
  }

  if (nearest_collision_point_it->is_active) {
    nearest_collision_point_it->start_time = now;
  }
}

std::optional<CollisionPoint> ObstacleStop::get_nearest_collision_point(
  const std::vector<CollisionPoint> & collision_points_buffer) const
{
  std::optional<CollisionPoint> nearest_collision_point = std::nullopt;
  if (collision_points_buffer.empty()) return nearest_collision_point;

  auto minimum_arc_length = std::numeric_limits<double>::max();
  for (const auto & cp : collision_points_buffer) {
    if (cp.is_active > minimum_arc_length || !cp.is_active) continue;
    nearest_collision_point = cp;
    minimum_arc_length = cp.arc_length;
  }

  return nearest_collision_point;
}

void ObstacleStop::publish_debug_string(bool is_safe) const
{
  const auto cluster_pcd_size =
    debug_data_.cluster_points ? debug_data_.cluster_points->data.size() : 0;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << std::boolalpha;
  ss << "OBSTACLE STOP (Backup Planner):" << "\n";
  ss << "\t\t" << "SAFE: " << is_safe << "\n";
  ss << "\t\t" << "OBJECTS: " << debug_data_.filtered_objects.objects.size() << " --> "
     << debug_data_.target_polygons.size() << "\n";
  ss << "\t\t" << "POINTCLOUD: " << cluster_pcd_size << " --> "
     << debug_data_.target_pcd_points.size() << "\n";
  if (nearest_collision_point_) {
    ss << "\t\t" << "DISTANCE TO COLLISION: " << nearest_collision_point_->arc_length << " m"
       << "\n";
  }

  StringStamped string_stamp;
  string_stamp.stamp = get_clock()->now();
  string_stamp.data = ss.str();
  pub_debug_text_->publish(string_stamp);
}

void ObstacleStop::publish_debug_data(const std::string & ns, const ModifierData & data) const
{
  if (debug_data_.cluster_points) pub_clustered_pointcloud_->publish(*debug_data_.cluster_points);

  MarkerArray marker_array;
  const auto ego_z = data.odometry_ptr->pose.pose.position.z;
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);
  const auto yellow = autoware_utils::create_marker_color(1.0, 1.0, 0.0, 1.0);
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);

  auto add_point_marker = [&](
                            const geometry_msgs::msg::Point & point, const std::string & marker_ns,
                            const int id, const std_msgs::msg::ColorRGBA & color,
                            const double scale = 0.1) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), marker_ns, id, Marker::SPHERE,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.pose.position = point;
    marker_array.markers.push_back(marker);
  };

  auto add_polygon_marker = [&](
                              const Polygon2d & polygon, const std::string & marker_ns,
                              const int id, const std_msgs::msg::ColorRGBA & color) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), marker_ns, id, Marker::LINE_STRIP,
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

}  // namespace autoware::minimum_rule_based_planner::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::minimum_rule_based_planner::plugin::ObstacleStop,
  autoware::minimum_rule_based_planner::plugin::PluginInterface)
