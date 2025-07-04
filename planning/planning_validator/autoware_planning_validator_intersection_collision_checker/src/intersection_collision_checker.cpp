// Copyright 2025 TIER IV, Inc.
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

#include "autoware/planning_validator_intersection_collision_checker/intersection_collision_checker.hpp"

#include "autoware/planning_validator_intersection_collision_checker/utils.hpp"

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>

#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_utils::get_or_declare_parameter;

void IntersectionCollisionChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;

  clock_ = node.get_clock();
  logger_ = node.get_logger();
  context_ = context;

  last_valid_time_ = clock_->now();
  last_invalid_time_ = clock_->now();

  param_listener_ = std::make_unique<intersection_collision_checker_node::ParamListener>(
    node.get_node_parameters_interface());

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "intersection_collision_checker");

  setup_diag();
}

void IntersectionCollisionChecker::setup_diag()
{
  context_->add_diag(
    "intersection_validation_collision_check",
    context_->validation_status->is_valid_intersection_collision_check,
    "risk of collision at intersection turn");
}

void IntersectionCollisionChecker::validate(bool & is_critical)
{
  context_->validation_status->is_valid_intersection_collision_check = true;

  params_ = param_listener_->get_params();
  const auto & p = params_.icc_parameters;

  if (!p.enable) return;

  auto skip_validation = [&](const std::string & reason) {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "%s", reason.c_str());
    is_critical = false;
    reset_data();
  };

  if (!context_->data->obstacle_pointcloud) {
    return skip_validation("point cloud data is not available, skipping collision check.");
  }

  if (!context_->data->route_handler->isHandlerReady()) {
    return skip_validation("route handler is not ready, skipping collision check.");
  }

  const auto ego_trajectory = get_ego_trajectory();

  EgoLanelets lanelets;
  const auto turn_direction = get_lanelets(lanelets, ego_trajectory);

  set_lanelets_debug_marker(lanelets);

  if (turn_direction == Direction::NONE) {
    reset_data();
    return;
  }

  if (lanelets.trajectory_lanelets.empty()) {
    return skip_validation("failed to get trajectory lanelets, skipping collision check.");
  }

  if (target_lanelets_map_.empty()) {
    return skip_validation("failed to get target lanelets, skipping collision check.");
  }

  context_->validation_status->is_valid_intersection_collision_check = is_safe();
}

bool IntersectionCollisionChecker::is_safe()
{
  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  filter_pointcloud(context_->data->obstacle_pointcloud, filtered_pointcloud);
  if (filtered_pointcloud->empty()) return true;

  safety_factor_array_ = SafetyFactorArray{};
  const bool is_safe =
    check_collision(filtered_pointcloud, context_->data->obstacle_pointcloud->header.stamp);

  const auto & ego_pose = context_->data->current_kinematics->pose.pose;
  const auto & traj_points = context_->data->current_trajectory->points;
  auto publish_planning_factor = [&]() {
    safety_factor_array_.is_safe = false;
    safety_factor_array_.detail = "possible collision at intersection";
    safety_factor_array_.header.stamp = clock_->now();
    planning_factor_interface_->add(
      traj_points, ego_pose, ego_pose, PlanningFactor::STOP, safety_factor_array_);
    planning_factor_interface_->publish();
  };

  const auto & p = params_.icc_parameters;
  const auto now = clock_->now();

  if (is_safe) {
    if ((now - last_invalid_time_).seconds() > p.off_time_buffer) {
      last_valid_time_ = now;
      return true;
    }
    publish_planning_factor();
    return false;
  }

  if ((now - last_valid_time_).seconds() < p.on_time_buffer) {
    RCLCPP_WARN(logger_, "[ICC] Momentary collision risk detected.");
    return true;
  }

  last_invalid_time_ = now;
  publish_planning_factor();
  return false;
}

EgoTrajectory IntersectionCollisionChecker::get_ego_trajectory() const
{
  EgoTrajectory ego_traj;
  static constexpr double min_traj_vel = 0.1;
  const auto base_to_front_offset =
    context_->vehicle_info.front_overhang_m + context_->vehicle_info.wheel_base_m;
  const auto base_to_back_offset = -1.0 * context_->vehicle_info.rear_overhang_m;

  const auto ego_front_pose = autoware_utils::calc_offset_pose(
    context_->data->current_kinematics->pose.pose, base_to_front_offset, 0.0, 0.0);
  const auto ego_back_pose = autoware_utils::calc_offset_pose(
    context_->data->current_kinematics->pose.pose, base_to_back_offset, 0.0, 0.0);

  const auto & trajectory_points = context_->data->resampled_current_trajectory->points;
  ego_traj.front_traj = trajectory_points;
  ego_traj.back_traj = trajectory_points;
  autoware::motion_utils::calculate_time_from_start(
    ego_traj.front_traj, ego_front_pose.position, min_traj_vel);
  autoware::motion_utils::calculate_time_from_start(
    ego_traj.back_traj, ego_back_pose.position, min_traj_vel);

  ego_traj.front_index =
    autoware::motion_utils::findNearestIndex(ego_traj.front_traj, ego_front_pose.position);
  ego_traj.back_index =
    autoware::motion_utils::findNearestIndex(ego_traj.back_traj, ego_back_pose.position);

  return ego_traj;
}

Direction IntersectionCollisionChecker::get_lanelets(
  EgoLanelets & lanelets, const EgoTrajectory & ego_trajectory) const
{
  const auto & ego_pose = context_->data->current_kinematics->pose.pose;
  try {
    collision_checker_utils::set_trajectory_lanelets(
      ego_trajectory.front_traj, *context_->data->route_handler, ego_pose, lanelets);
  } catch (const std::logic_error & e) {
    RCLCPP_ERROR(logger_, "failed to get trajectory lanelets: %s", e.what());
    return Direction::NONE;
  }

  const auto turn_direction = get_turn_direction(lanelets.trajectory_lanelets);

  if (turn_direction == Direction::NONE) return turn_direction;

  for (auto & target_lanelet : target_lanelets_map_) {
    target_lanelet.second.is_active = false;
  }

  const auto & p = params_.icc_parameters;

  const auto current_vel = context_->data->current_kinematics->twist.twist.linear.x;
  const auto stopping_time = abs(current_vel / p.ego_deceleration);
  const auto time_horizon = std::max(p.min_time_horizon, stopping_time);
  if (turn_direction == Direction::RIGHT) {
    collision_checker_utils::set_right_turn_target_lanelets(
      ego_trajectory, *context_->data->route_handler, params_, lanelets, target_lanelets_map_,
      time_horizon);
  } else {
    collision_checker_utils::set_left_turn_target_lanelets(
      ego_trajectory, *context_->data->route_handler, params_, lanelets, target_lanelets_map_,
      time_horizon);
  }

  return turn_direction;
}

Direction IntersectionCollisionChecker::get_turn_direction(
  const lanelet::ConstLanelets & trajectory_lanelets) const
{
  const auto & p = params_.icc_parameters;
  for (const auto & lanelet : trajectory_lanelets) {
    if (!lanelet.hasAttribute("turn_direction")) continue;
    const lanelet::Attribute & attr = lanelet.attribute("turn_direction");
    if (attr.value() == "right" && p.right_turn.enable) return Direction::RIGHT;
    if (attr.value() == "left" && p.left_turn.enable) return Direction::LEFT;
  }
  return Direction::NONE;
}

void IntersectionCollisionChecker::filter_pointcloud(
  PointCloud2::ConstSharedPtr & input, PointCloud::Ptr & filtered_pointcloud) const
{
  if (input->data.empty()) return;

  const auto & p = params_.icc_parameters;

  pcl::fromROSMsg(*input, *filtered_pointcloud);

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(filtered_pointcloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(-1.0 * p.detection_range, p.detection_range);
    filter.filter(*filtered_pointcloud);
  }

  if (filtered_pointcloud->empty()) return;

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(filtered_pointcloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(
      p.pointcloud.min_height,
      context_->vehicle_info.vehicle_height_m + p.pointcloud.height_buffer);
    filter.filter(*filtered_pointcloud);
  }

  if (filtered_pointcloud->empty()) return;

  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = context_->tf_buffer.lookupTransform(
        "map", input->header.frame_id, input->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(logger_, "no transform found for pointcloud: %s", e.what());
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*filtered_pointcloud, *filtered_pointcloud, isometry);
  }

  {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(filtered_pointcloud);
    filter.setLeafSize(
      p.pointcloud.voxel_grid_filter.x, p.pointcloud.voxel_grid_filter.y,
      p.pointcloud.voxel_grid_filter.z);
    filter.filter(*filtered_pointcloud);
  }
}

bool IntersectionCollisionChecker::check_collision(
  const PointCloud::Ptr & filtered_point_cloud, const rclcpp::Time & time_stamp)
{
  bool is_safe = true;

  const auto & p = params_.icc_parameters;

  auto ego_object_overlap_time =
    [](const double object_ttc, const std::pair<double, double> & ego_time) {
      if (object_ttc > ego_time.second) return object_ttc - ego_time.second;
      if (object_ttc < ego_time.first) return ego_time.first - object_ttc;
      return 0.0;
    };

  static constexpr double close_distance_threshold = 1.0;
  const auto stopping_time =
    abs(context_->data->current_kinematics->twist.twist.linear.x / p.ego_deceleration);
  auto is_colliding = [&](const PCDObject & object, const std::pair<double, double> & ego_time) {
    if (object.track_duration < p.pointcloud.velocity_estimation.observation_time) return false;
    if (object.distance_to_overlap < close_distance_threshold && ego_time.first < stopping_time)
      return true;
    if (
      object.moving_time > p.filter.moving_time &&
      ego_object_overlap_time(object.ttc, ego_time) < p.ttc_threshold) {
      return true;
    }
    return false;
  };

  for (const auto & target_lanelet : target_lanelets_map_) {
    const auto target_ll = target_lanelet.second;
    if (!target_ll.is_active || target_ll.lanelets.empty()) continue;

    const auto pcd_object = get_pcd_object(time_stamp, filtered_point_cloud, target_ll);
    if (!pcd_object.has_value()) continue;

    auto update_object = [&](PCDObject & object, const PCDObject & new_data) {
      const auto dt = (new_data.last_update_time - object.last_update_time).seconds();
      const auto dl = object.distance_to_overlap - new_data.distance_to_overlap;
      if (dt < 1e-6) return;  // too small time difference, skip update

      const auto max_accel = p.pointcloud.velocity_estimation.max_acceleration;
      const auto observation_time_th = p.pointcloud.velocity_estimation.observation_time;
      const auto raw_velocity = dl / dt;
      const bool is_reliable = object.track_duration > observation_time_th;
      // update velocity only if the object is not yet reliable or velocity change is within limit
      if (!is_reliable || std::abs(raw_velocity - object.velocity) / dt < max_accel) {
        object.velocity = autoware::signal_processing::lowpassFilter(
          raw_velocity, object.velocity, 0.5);  // apply low-pass filter to velocity
        object.track_duration += dt;            // update track duration
      }

      object.last_update_time = new_data.last_update_time;
      object.pose = new_data.pose;
      object.distance_to_overlap = new_data.distance_to_overlap;
      object.delay_compensated_distance_to_overlap =
        std::max(0.0, object.distance_to_overlap - object.velocity * p.pointcloud.latency);
      object.moving_time =
        (object.velocity < p.filter.min_velocity) ? 0.0 : object.moving_time + dt;
      object.ttc = object.delay_compensated_distance_to_overlap / object.velocity;
    };

    if (history_.find(pcd_object->overlap_lanelet_id) == history_.end()) {
      history_[pcd_object->overlap_lanelet_id] = pcd_object.value();
    } else {
      auto & existing_object = history_[pcd_object->overlap_lanelet_id];
      update_object(existing_object, pcd_object.value());

      if (is_colliding(existing_object, target_ll.ego_overlap_time)) {
        is_safe = false;
        context_->debug_pose_publisher->pushPointMarker(
          existing_object.pose.position, "collision_checker_pcd_objects", 0, 0.5, true);
        context_->debug_pose_publisher->pushLineSegmentMarker(
          existing_object.pose.position, target_ll.overlap_point.position,
          "collision_checker_pcd_objects", 0);
        add_safety_factor(existing_object.pose.position, target_ll.ego_overlap_time.first);
      } else {
        context_->debug_pose_publisher->pushPointMarker(
          existing_object.pose.position, "collision_checker_pcd_objects", 1);
      }
    }
  }

  static constexpr double max_history_time = 1.0;
  auto itr = history_.begin();
  while (itr != history_.end()) {
    if ((clock_->now() - itr->second.last_update_time).seconds() > max_history_time) {
      itr = history_.erase(itr);
    } else {
      itr++;
    }
  }

  return is_safe;
}

std::optional<PCDObject> IntersectionCollisionChecker::get_pcd_object(
  const rclcpp::Time & time_stamp, const PointCloud::Ptr & filtered_point_cloud,
  const TargetLanelet & target_lanelet) const
{
  std::optional<PCDObject> pcd_object = std::nullopt;

  PointCloud::Ptr points_within(new PointCloud);
  const auto combine_lanelet = lanelet::utils::combineLaneletsShape(target_lanelet.lanelets);
  get_points_within(
    filtered_point_cloud, combine_lanelet.polygon2d().basicPolygon(), points_within);

  if (points_within->empty()) return pcd_object;

  PointCloud::Ptr clustered_points(new PointCloud);
  cluster_pointcloud(points_within, clustered_points);

  if (clustered_points->empty()) return pcd_object;

  const auto overlap_arc_coord =
    lanelet::utils::getArcCoordinates(target_lanelet.lanelets, target_lanelet.overlap_point);
  auto min_arc_length = std::numeric_limits<double>::max();
  for (const auto & p : *clustered_points) {
    geometry_msgs::msg::Pose p_geom;
    p_geom.position = autoware_utils::create_point(p.x, p.y, p.z);
    const auto arc_coord = lanelet::utils::getArcCoordinates(target_lanelet.lanelets, p_geom);
    const auto arc_length_to_overlap = overlap_arc_coord.length - arc_coord.length;
    if (
      arc_length_to_overlap < std::numeric_limits<double>::epsilon() ||
      arc_length_to_overlap > min_arc_length) {
      continue;
    }

    min_arc_length = arc_length_to_overlap;

    PCDObject object;
    object.last_update_time = time_stamp;
    object.pose = p_geom;
    object.overlap_lanelet_id = target_lanelet.id;
    object.track_duration = 0.0;
    object.distance_to_overlap = arc_length_to_overlap;
    object.delay_compensated_distance_to_overlap = arc_length_to_overlap;
    object.velocity = 0.0;
    object.moving_time = 0.0;
    object.ttc = std::numeric_limits<double>::max();
    pcd_object = object;
  }
  return pcd_object;
}

void IntersectionCollisionChecker::get_points_within(
  const PointCloud::Ptr & input, const BasicPolygon2d & polygon,
  const PointCloud::Ptr & output) const
{
  if (input->empty()) return;

  for (const auto & point : *input) {
    if (boost::geometry::within(autoware_utils::Point2d{point.x, point.y}, polygon)) {
      output->push_back(point);
    }
  }
}

void IntersectionCollisionChecker::cluster_pointcloud(
  const PointCloud::Ptr & input, PointCloud::Ptr & output) const
{
  if (input->empty()) return;

  const auto & p = params_.icc_parameters;

  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(p.pointcloud.clustering.tolerance);
    ec.setMinClusterSize(p.pointcloud.clustering.min_size);
    ec.setMaxClusterSize(p.pointcloud.clustering.max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_idx);
    return cluster_idx;
  });

  const auto ego_base_z = context_->data->current_kinematics->pose.pose.position.z;
  auto above_height_threshold = [&](const double z) {
    const auto rel_height = z - ego_base_z;
    return rel_height > p.pointcloud.clustering.min_height;
  };

  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    bool cluster_above_height_threshold{false};
    for (const auto & index : indices.indices) {
      const auto & point = (*input)[index];

      cluster_above_height_threshold |= above_height_threshold(point.z);
      cluster->push_back(point);
    }
    if (!cluster_above_height_threshold) continue;

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull);
    for (const auto & point : *surface_hull) {
      output->push_back(point);
    }
  }
}

void IntersectionCollisionChecker::set_lanelets_debug_marker(const EgoLanelets & lanelets) const
{
  {  // trajectory lanelets
    lanelet::BasicPolygons2d ll_polygons;
    lanelet::BasicPolygons2d turn_ll_polygons;
    for (const auto & ll : lanelets.trajectory_lanelets) {
      ll_polygons.push_back(ll.polygon2d().basicPolygon());
      if (ll.hasAttribute("turn_direction") && ll.attribute("turn_direction") != "straight") {
        turn_ll_polygons.push_back(ll.polygon2d().basicPolygon());
      }
    }
    if (!ll_polygons.empty()) {
      context_->debug_pose_publisher->pushLaneletPolygonsMarker(
        ll_polygons, "collision_checker_trajectory_lanelets", 1);
    }
    if (!turn_ll_polygons.empty()) {
      context_->debug_pose_publisher->pushLaneletPolygonsMarker(
        turn_ll_polygons, "collision_checker_turn_direction_lanelets", 0);
    }
  }

  {  // target lanelets
    lanelet::BasicPolygons2d ll_polygons;
    for (const auto & t_l : target_lanelets_map_) {
      if (!t_l.second.is_active) continue;
      for (const auto & ll : t_l.second.lanelets) {
        ll_polygons.push_back(ll.polygon2d().basicPolygon());
      }
      context_->debug_pose_publisher->pushPointMarker(
        t_l.second.overlap_point.position, "collision_checker_target_lanelets", 2);
    }
    if (!ll_polygons.empty()) {
      context_->debug_pose_publisher->pushLaneletPolygonsMarker(
        ll_polygons, "collision_checker_target_lanelets", 2);
    }
  }
}

void IntersectionCollisionChecker::add_safety_factor(
  geometry_msgs::msg::Point & obs_point, const double ttc)
{
  SafetyFactor factor;
  factor.is_safe = false;
  factor.type = SafetyFactor::POINTCLOUD;
  factor.ttc_begin = ttc;
  factor.points.push_back(obs_point);
  safety_factor_array_.factors.push_back(factor);
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::IntersectionCollisionChecker,
  autoware::planning_validator::PluginInterface)
