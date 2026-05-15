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
#include <autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{
using autoware::trajectory_modifier::utils::obstacle_stop::cluster_pointcloud;
using autoware::trajectory_modifier::utils::obstacle_stop::filter_objects_by_type;
using autoware::trajectory_modifier::utils::obstacle_stop::filter_objects_by_velocity;
using autoware::trajectory_modifier::utils::obstacle_stop::filter_pointcloud_by_range;
using autoware::trajectory_modifier::utils::obstacle_stop::filter_pointcloud_by_voxel_grid;
using autoware::trajectory_modifier::utils::obstacle_stop::get_nearest_object_collision;
using autoware::trajectory_modifier::utils::obstacle_stop::get_nearest_pcd_collision;
using autoware::trajectory_modifier::utils::obstacle_stop::get_trajectory_polygon;
using autoware::trajectory_modifier::utils::obstacle_stop::PointCloud;

void ObstacleStop::on_initialize([[maybe_unused]] const MinimumRuleBasedPlannerParams & params)
{
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      get_node_ptr(), "obstacle_stop");
}

void ObstacleStop::run(TrajectoryPoints & traj_points)
{
  if (!params_.enable || !is_obstacle_detected(traj_points)) return;

  if (!nearest_collision_point_) return;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[MRBP ObstacleStop] Detected collision point at arc length %f m",
    nearest_collision_point_->arc_length);

  set_stop_point(traj_points);
}

bool ObstacleStop::is_obstacle_detected(const TrajectoryPoints & traj_points)
{
  debug_data_ = DebugData();
  debug_data_.trajectory_polygon = get_trajectory_polygon(
    traj_points, data_->odometry_ptr->pose.pose, vehicle_info_, params_.lateral_margin_m);
  const auto collision_point_pcd = check_pointcloud(traj_points, debug_data_.trajectory_polygon);
  update_collision_points_buffer(collision_points_buffer_.pcd, traj_points, collision_point_pcd);
  const auto collision_point_objects =
    check_predicted_objects(traj_points, debug_data_.trajectory_polygon);
  update_collision_points_buffer(
    collision_points_buffer_.objects, traj_points, collision_point_objects);

  nearest_collision_point_ = get_nearest_collision_point();
  debug_data_.active_collision_point =
    nearest_collision_point_ ? nearest_collision_point_->point : geometry_msgs::msg::Point();

  return nearest_collision_point_ != std::nullopt;
}

void ObstacleStop::set_stop_point(TrajectoryPoints & traj_points)
{
  const auto stop_margin = params_.stop_margin_m + vehicle_info_.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length =
    std::max(nearest_collision_point_->arc_length - stop_margin, 0.0);

  const auto stop_index = motion_utils::insertStopPoint(target_stop_point_arc_length, traj_points);
  if (!stop_index) return;

  const auto & stop_pose = traj_points.at(*stop_index).pose;
  const auto & ego_pose = data_->odometry_ptr->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[MRBP ObstacleStop] Inserted stop point at arc length %f m", target_stop_point_arc_length);
}

std::optional<CollisionPoint> ObstacleStop::check_predicted_objects(
  const TrajectoryPoints & traj_points, const MultiPolygon2d & trajectory_polygon)
{
  if (!data_->predicted_objects_ptr || data_->predicted_objects_ptr->objects.empty())
    return std::nullopt;
  auto predicted_objects = *data_->predicted_objects_ptr;

  filter_objects_by_type(predicted_objects, params_.objects.object_types);
  filter_objects_by_velocity(predicted_objects, params_.objects.max_velocity_th);

  return get_nearest_object_collision(traj_points, trajectory_polygon, predicted_objects);
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(
  const TrajectoryPoints & traj_points, const MultiPolygon2d & trajectory_polygon)
{
  if (!data_->obstacle_pointcloud_ptr || data_->obstacle_pointcloud_ptr->data.empty())
    return std::nullopt;

  const auto & pointcloud = data_->obstacle_pointcloud_ptr;

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*pointcloud, *filtered_pointcloud);

  const auto & p = params_.pointcloud;

  const auto max_height = vehicle_info_.vehicle_height_m + params_.pointcloud.height_buffer;
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
  const auto height_offset = data_->odometry_ptr->pose.pose.position.z;
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

std::optional<CollisionPoint> ObstacleStop::get_nearest_collision_point() const
{
  std::optional<CollisionPoint> nearest_collision_point = std::nullopt;
  if (collision_points_buffer_.empty()) return nearest_collision_point;

  auto minimum_arc_length = std::numeric_limits<double>::max();
  for (const auto & cp : collision_points_buffer_.pcd) {
    if (cp.is_active > minimum_arc_length || !cp.is_active) continue;
    nearest_collision_point = cp;
    minimum_arc_length = cp.arc_length;
  }

  for (const auto & cp : collision_points_buffer_.objects) {
    if (cp.is_active > minimum_arc_length || !cp.is_active) continue;
    nearest_collision_point = cp;
    minimum_arc_length = cp.arc_length;
  }

  return nearest_collision_point;
}

}  // namespace autoware::minimum_rule_based_planner::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::minimum_rule_based_planner::plugin::ObstacleStop,
  autoware::minimum_rule_based_planner::plugin::PluginInterface)
