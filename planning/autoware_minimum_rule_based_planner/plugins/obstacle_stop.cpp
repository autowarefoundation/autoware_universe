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
#include <autoware_utils_geometry/geometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{
using autoware::trajectory_processor::utils::clamp_stop_point_arc_length;
using autoware::trajectory_processor::utils::insert_stop_point;
using autoware::trajectory_processor::utils::replace_trajectory_with_stop_point;
using autoware::trajectory_processor::utils::obstacle_stop::build_trajectory_footprint_index;
using autoware::trajectory_processor::utils::obstacle_stop::get_nearest_object_collision;
using autoware::trajectory_processor::utils::obstacle_stop::get_nearest_pcd_collision;
using autoware::trajectory_processor::utils::obstacle_stop::LateralMarginMap;
using autoware::trajectory_processor::utils::obstacle_stop::ObjectType;
using autoware::trajectory_processor::utils::obstacle_stop::PointCloud;
using autoware::trajectory_processor::utils::obstacle_stop::TargetObject;

void ObstacleStop::on_initialize(const MinimumRuleBasedPlannerParams & params)
{
  params_ = params.obstacle_stop;

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterfaceT<
      autoware::agnocast_wrapper::Node>>(get_node_ptr(), "backup_planner_obstacle_stop");

  pointcloud_filter_ =
    std::make_unique<trajectory_processor::utils::obstacle_stop::PointCloudFilter>(
      params_.objects.target_objects.pointcloud);

  object_filter_ = std::make_unique<trajectory_processor::utils::obstacle_stop::ObjectFilter>(
    params_.objects.target_objects.bbox, params_.objects.target_objects.polygon,
    params_.objects.stopped_velocity_th, params_.objects.max_lateral_velocity_th,
    params_.objects.safety_buffer);

  pub_filtered_pointcloud_ =
    get_node_ptr()->create_publisher<PointCloud2>("~/obstacle_stop/debug/filtered_points", 1);
  debug_viz_pub_ = get_node_ptr()->create_publisher<MarkerArray>("~/obstacle_stop/debug/marker", 1);
  pub_debug_text_ =
    get_node_ptr()->create_publisher<StringStamped>("~/obstacle_stop/debug/text", 1);

  update_object_decel_map();
  update_lateral_margin_map();
}

void ObstacleStop::run(TrajectoryPoints & traj_points, const ModifierData & data)
{
  if (!params_.enable) return;

  const auto detected = is_obstacle_detected(traj_points, data);
  publish_debug_string(!detected);
  publish_debug_data("obstacle_stop");

  if (!detected) return;
  if (!nearest_collision_point_) return;

  set_stop_point(traj_points, data);
}

bool ObstacleStop::is_obstacle_detected(
  const TrajectoryPoints & traj_points, const ModifierData & data)
{
  debug_data_ = DebugData();
  safety_factors_ = SafetyFactorArray{};
  debug_data_.trajectory_shape = build_trajectory_footprint_index(
    traj_points, data.odometry_ptr->pose.pose, context_->vehicle_info,
    data.odometry_ptr->twist.twist.linear.x, data.acceleration_ptr->accel.accel.linear.x,
    params_.nominal_stopping_decel, params_.stopping_jerk, params_.stop_margin,
    params_.delay_response_time);
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

  debug_data_.ego_z = data.odometry_ptr->pose.pose.position.z;

  return nearest_collision_point_ != std::nullopt;
}

void ObstacleStop::set_stop_point(TrajectoryPoints & traj_points, const ModifierData & data)
{
  const auto stop_margin = params_.stop_margin + context_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length = clamp_stop_point_arc_length(
    nearest_collision_point_->arc_length - stop_margin,
    debug_data_.trajectory_shape.trajectory_length, data.odometry_ptr->twist.twist.linear.x,
    data.acceleration_ptr->accel.accel.linear.x, params_.maximum_stopping_decel,
    params_.stopping_jerk, params_.delay_response_time);

  if (
    target_stop_point_arc_length < params_.arrived_distance_threshold ||
    !insert_stop_point(traj_points, target_stop_point_arc_length)) {
    replace_trajectory_with_stop_point(traj_points, data.odometry_ptr->pose.pose);
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
  debug_data_.filtered_objects = *data.predicted_objects_ptr;

  object_filter_->filter_objects(debug_data_.filtered_objects);

  debug_data_.target_objects.clear();
  debug_data_.target_objects.reserve(debug_data_.filtered_objects.objects.size());
  for (const auto & object : debug_data_.filtered_objects.objects) {
    debug_data_.target_objects.emplace_back(object);
  }

  object_filter_->filter_by_target_area(
    debug_data_.target_objects, traj_points, context_->vehicle_info, debug_data_.trajectory_shape,
    lateral_margin_map_);

  auto collision_point = get_nearest_object_collision(
    debug_data_.target_objects, traj_points, context_->vehicle_info, object_decel_map_,
    params_.rss_params.ego_decel, params_.delay_response_time, params_.stop_margin,
    params_.rss_params.safety_margin, params_.objects.stopped_velocity_th,
    params_.rss_params.lookahead_horizon, params_.rss_params.enable);

  if (collision_point) {
    auto it = std::find_if(
      debug_data_.target_objects.begin(), debug_data_.target_objects.end(),
      [&](const TargetObject & object) { return !object.is_safe; });
    if (it != debug_data_.target_objects.end()) {
      debug_data_.colliding_object = it->object;
    }
  }

  return collision_point;
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(
  [[maybe_unused]] const TrajectoryPoints & traj_points, const ModifierData & data)
{
  if (!params_.use_pointcloud) return std::nullopt;

  if (!data.obstacle_pointcloud_ptr || data.obstacle_pointcloud_ptr->data.empty())
    return std::nullopt;

  const auto & pointcloud = data.obstacle_pointcloud_ptr;

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*pointcloud, *filtered_pointcloud);

  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = context_->tf_buffer.lookupTransform(
        "map", pointcloud->header.frame_id, pointcloud->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_node_ptr()->get_logger(), "no transform found for pointcloud: %s", e.what());
      return std::nullopt;
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    for (auto & p : filtered_pointcloud->points) {
      const Eigen::Vector3f q = isometry * Eigen::Vector3f(p.x, p.y, p.z);
      p.x = q.x();
      p.y = q.y();
      p.z = q.z();
    }
  }

  {
    const auto & bbox = debug_data_.trajectory_shape.bounding_box;
    constexpr double buffer = 1.0;
    const auto [min_x, max_x] = std::minmax(bbox.min_corner().x(), bbox.max_corner().x());
    const auto [min_y, max_y] = std::minmax(bbox.min_corner().y(), bbox.max_corner().y());
    const auto ego_z = data.odometry_ptr->pose.pose.position.z;
    const auto min_z = ego_z + params_.objects.pointcloud_min_height;
    const auto max_z =
      ego_z + context_->vehicle_info.vehicle_height_m + params_.objects.pointcloud_height_buffer;
    pointcloud_filter_->filter_pointcloud(
      filtered_pointcloud, min_x - buffer, max_x + buffer, min_y - buffer, max_y + buffer, min_z,
      max_z);
  }

  {
    const auto filtered_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*filtered_pointcloud, *filtered_pointcloud_msg);
    filtered_pointcloud_msg->header.stamp = pointcloud->header.stamp;
    filtered_pointcloud_msg->header.frame_id = "map";
    debug_data_.filtered_points = filtered_pointcloud_msg;
  }

  if (data.predicted_objects_ptr && !data.predicted_objects_ptr->objects.empty()) {
    pointcloud_filter_->filter_pointcloud_by_object(
      filtered_pointcloud, *data.predicted_objects_ptr);
  }

  return get_nearest_pcd_collision(
    debug_data_.trajectory_shape, filtered_pointcloud, lateral_margin_map_,
    debug_data_.target_pcd_points);
}

void ObstacleStop::update_collision_points_buffer(
  std::vector<CollisionPoint> & collision_points_buffer, const TrajectoryPoints & traj_points,
  const std::optional<CollisionPoint> & collision_point)
{
  constexpr double close_distance_threshold = 1.0;

  const auto now = get_clock()->now();

  // prune obsolete collision points
  collision_points_buffer.erase(
    std::remove_if(
      collision_points_buffer.begin(), collision_points_buffer.end(),
      [&](CollisionPoint & cp) {
        if (cp.is_active) return (now - cp.last_seen).seconds() > params_.off_time_buffer;
        return !collision_point;
      }),
    collision_points_buffer.end());

  if (!collision_point) return;

  // find nearest tracked collision
  auto nearest_collision_point_it = collision_points_buffer.end();
  auto nearest_distance_diff = std::numeric_limits<double>::max();
  for (auto it = collision_points_buffer.begin(); it != collision_points_buffer.end(); ++it) {
    const auto estimated_shift = std::abs(it->obstacle_lon_vel) * (now - it->last_seen).seconds();
    it->arc_length = motion_utils::calcSignedArcLength(traj_points, 0LU, it->point);
    const auto expected_arc_length = it->arc_length + estimated_shift;
    const auto distance_diff = collision_point->arc_length - expected_arc_length;
    if (
      std::abs(distance_diff) > close_distance_threshold ||
      std::abs(distance_diff) > std::abs(nearest_distance_diff))
      continue;
    nearest_collision_point_it = it;
    nearest_distance_diff = distance_diff;
  }

  constexpr double eps = 1e-3;
  if (nearest_collision_point_it == collision_points_buffer.end()) {
    collision_points_buffer.emplace_back(*collision_point, now, now, params_.on_time_buffer < eps);
    return;
  }

  if ((now - nearest_collision_point_it->first_seen).seconds() > params_.on_time_buffer) {
    nearest_collision_point_it->is_active = true;
  }

  nearest_collision_point_it->last_seen = now;
  nearest_collision_point_it->obstacle_lon_vel = collision_point->obstacle_lon_vel;
  nearest_collision_point_it->is_dynamic = collision_point->is_dynamic;
  nearest_collision_point_it->point = collision_point->point;
  nearest_collision_point_it->arc_length = collision_point->arc_length;
}

std::optional<CollisionPoint> ObstacleStop::get_nearest_collision_point(
  const std::vector<CollisionPoint> & collision_points_buffer) const
{
  std::optional<CollisionPoint> nearest_collision_point = std::nullopt;
  if (collision_points_buffer.empty()) return nearest_collision_point;

  auto minimum_arc_length = std::numeric_limits<double>::max();
  for (const auto & cp : collision_points_buffer) {
    if (cp.arc_length >= minimum_arc_length || !cp.is_active) continue;
    nearest_collision_point = cp;
    minimum_arc_length = cp.arc_length;
  }

  return nearest_collision_point;
}

void ObstacleStop::publish_debug_string(bool is_safe) const
{
  const auto filtered_pcd_size =
    debug_data_.filtered_points ? debug_data_.filtered_points->data.size() : 0;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << std::boolalpha;
  ss << "OBSTACLE STOP (Backup Planner):" << "\n";
  ss << "\t\t" << "SAFE: " << is_safe << "\n";
  ss << "\t\t" << "OBJECTS: " << debug_data_.filtered_objects.objects.size() << " --> "
     << debug_data_.target_objects.size() << "\n";
  ss << "\t\t" << "POINTCLOUD: " << filtered_pcd_size << " --> "
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

void ObstacleStop::publish_debug_data(const std::string & ns) const
{
  if (debug_data_.filtered_points) pub_filtered_pointcloud_->publish(*debug_data_.filtered_points);

  MarkerArray marker_array;
  const auto ego_z = debug_data_.ego_z;
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);
  const auto yellow = autoware_utils::create_marker_color(1.0, 1.0, 0.0, 1.0);
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);

  auto add_line_list_marker = [&](
                                const std::vector<geometry_msgs::msg::Point> & segments,
                                const std::string & ns, const int id,
                                const std_msgs::msg::ColorRGBA & color) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::LINE_LIST,
      autoware_utils::create_marker_scale(0.1, 0.1, 0.1), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.points = segments;
    marker_array.markers.push_back(marker);
  };

  int id = 0;
  {
    using autoware_utils_geometry::calc_offset_pose;
    const auto & shape = debug_data_.trajectory_shape;
    const auto half_width = shape.ego_half_width;
    const auto front_offset = std::abs(shape.ego_front_offset);
    const auto back_offset = std::abs(shape.ego_back_offset);
    std::vector<geometry_msgs::msg::Point> segments;
    auto dist = 0.0;
    for (const auto & footprint : shape.footprints) {
      if (footprint.arc_length - dist < 1.0) continue;
      dist = footprint.arc_length;
      auto front_left = calc_offset_pose(footprint.pose, front_offset, -half_width, 0.0).position;
      auto front_right = calc_offset_pose(footprint.pose, front_offset, half_width, 0.0).position;
      auto rear_right = calc_offset_pose(footprint.pose, -back_offset, half_width, 0.0).position;
      auto rear_left = calc_offset_pose(footprint.pose, -back_offset, -half_width, 0.0).position;
      segments.emplace_back(rear_left);
      segments.emplace_back(front_left);
      segments.emplace_back(front_left);
      segments.emplace_back(front_right);
      segments.emplace_back(front_right);
      segments.emplace_back(rear_right);
      segments.emplace_back(rear_right);
      segments.emplace_back(rear_left);
    }
    add_line_list_marker(segments, ns + "/traj_polygon", id, yellow);
    id++;
  }

  auto add_polygon_marker = [&](
                              const Polygon2d & polygon, const std::string & ns, const int id,
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

  auto add_text_marker = [&](
                           const std::string & text, const geometry_msgs::msg::Pose & pose,
                           const std::string & ns, const int id,
                           const std_msgs::msg::ColorRGBA & color, const double scale = 0.4) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.pose = pose;
    marker.pose.position.z += 1.0;
    marker.text = text;
    marker_array.markers.push_back(marker);
  };

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

  for (const auto & obj : debug_data_.target_objects) {
    const auto & target_polygon = obj.polygon;
    add_polygon_marker(target_polygon, ns + "/target_objects", id, magenta);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "safe: " << (obj.is_safe ? "true" : "false") << "\n";
    ss << "rss_safe_dist: " << obj.safe_distance << " m" << "\n";
    ss << "dist_from_ego: " << obj.distance_from_ego << " m";
    add_text_marker(
      ss.str(), obj.object.kinematics.initial_pose_with_covariance.pose,
      ns + "/target_objects_text", id, white);
    if (!obj.is_safe)
      add_polygon_marker(obj.ego_footprint, ns + "/overlapping_ego_footprint", id, magenta);
    id++;
  }

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
