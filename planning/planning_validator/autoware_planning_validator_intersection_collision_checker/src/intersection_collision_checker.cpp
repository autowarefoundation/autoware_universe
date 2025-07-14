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
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <magic_enum.hpp>
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

  pub_cluster_pointcloud_ =
    node.create_publisher<PointCloud2>("~/intersection_collision_checker/debug/cluster_points", 1);

  pub_voxel_pointcloud_ =
    node.create_publisher<PointCloud2>("~/intersection_collision_checker/debug/voxel_points", 1);

  pub_string_ =
    node.create_publisher<StringStamped>("~/intersection_collision_checker/debug/state", 1);

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

bool IntersectionCollisionChecker::is_data_ready(std::string & msg)
{
  if (!context_->data->obstacle_pointcloud) {
    msg = "Point cloud data is not available.";
    return false;
  }

  if (!context_->data->route_handler->isHandlerReady()) {
    msg = "Route handler is not ready.";
    return false;
  }

  return true;
}

void IntersectionCollisionChecker::validate(bool & is_critical)
{
  context_->validation_status->is_valid_intersection_collision_check = true;

  params_ = param_listener_->get_params();
  const auto & p = params_.icc_parameters;

  if (!p.enable) return;

  std::string data_msg;
  if (!is_data_ready(data_msg)) {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 1000, "%s, skip collision check", data_msg.c_str());
    is_critical = false;
    reset_data();
    return;
  }

  const auto start_time = clock_->now();

  DebugData debug_data;
  context_->validation_status->is_valid_intersection_collision_check = is_safe(debug_data);
  if (!debug_data.is_active) reset_data();

  debug_data.processing_time_detail_ms = (clock_->now() - start_time).nanoseconds() * 1e-6;

  publish_markers(debug_data);
  publish_planning_factor(debug_data);
}

bool IntersectionCollisionChecker::is_safe(DebugData & debug_data)
{
  const auto ego_trajectory = get_ego_trajectory();
  get_lanelets(debug_data, ego_trajectory);

  if (debug_data.turn_direction == Direction::NONE) return true;

  if (debug_data.ego_lanelets.trajectory_lanelets.empty()) {
    debug_data.text = "failed to get trajectory lanelets";
    return true;
  }

  if (target_lanelets_map_.empty()) {
    debug_data.text = "failed to get target lanelets";
    return true;
  }

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  filter_pointcloud(context_->data->obstacle_pointcloud, filtered_pointcloud, debug_data);
  if (filtered_pointcloud->empty()) {
    debug_data.text = "no points in the filtered pointcloud";
    return true;
  }

  debug_data.is_active = true;
  debug_data.is_safe = true;

  const bool is_safe = check_collision(
    debug_data, filtered_pointcloud, context_->data->obstacle_pointcloud->header.stamp);

  const auto & p = params_.icc_parameters;
  const auto now = clock_->now();

  if (is_safe) {
    if ((now - last_invalid_time_).seconds() > p.off_time_buffer) {
      last_valid_time_ = now;
      return true;
    }
    debug_data.is_safe = false;
    return false;
  }

  if ((now - last_valid_time_).seconds() < p.on_time_buffer) {
    RCLCPP_WARN(logger_, "[ICC] Momentary collision risk detected.");
    debug_data.text = "detected momentary collision";
    return true;
  }

  last_invalid_time_ = now;
  debug_data.is_safe = false;
  debug_data.text = "detected continuous collision";
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

void IntersectionCollisionChecker::get_lanelets(
  DebugData & debug_data, const EgoTrajectory & ego_trajectory) const
{
  const auto & ego_pose = context_->data->current_kinematics->pose.pose;
  try {
    collision_checker_utils::set_trajectory_lanelets(
      ego_trajectory.front_traj, *context_->data->route_handler, ego_pose, debug_data.ego_lanelets);
  } catch (const std::logic_error & e) {
    RCLCPP_ERROR(logger_, "failed to get trajectory lanelets: %s", e.what());
    debug_data.turn_direction = Direction::NONE;
    return;
  }

  debug_data.turn_direction = get_turn_direction(debug_data.ego_lanelets.turn_lanelets);

  if (debug_data.turn_direction == Direction::NONE) return;

  for (auto & target_lanelet : target_lanelets_map_) {
    target_lanelet.second.is_active = false;
  }

  const auto & p = params_.icc_parameters;

  const auto current_vel = context_->data->current_kinematics->twist.twist.linear.x;
  const auto stopping_time = abs(current_vel / p.ego_deceleration);
  const auto time_horizon = std::max(p.min_time_horizon, stopping_time);
  if (debug_data.turn_direction == Direction::RIGHT) {
    collision_checker_utils::set_right_turn_target_lanelets(
      ego_trajectory, *context_->data->route_handler, params_, debug_data.ego_lanelets,
      target_lanelets_map_, time_horizon);
  } else {
    collision_checker_utils::set_left_turn_target_lanelets(
      ego_trajectory, *context_->data->route_handler, params_, debug_data.ego_lanelets,
      target_lanelets_map_, time_horizon);
  }

  for (const auto & target_lanelet : target_lanelets_map_) {
    const auto & target_ll = target_lanelet.second;
    if (!target_ll.is_active || target_ll.lanelets.empty()) continue;
    debug_data.target_lanelets.push_back(target_ll);
  }
}

Direction IntersectionCollisionChecker::get_turn_direction(
  const lanelet::ConstLanelets & turn_lanelets) const
{
  if (turn_lanelets.empty()) return Direction::NONE;
  const auto & p = params_.icc_parameters;
  const lanelet::Attribute & attr = turn_lanelets.front().attributeOr("turn_direction", "else");
  if (attr.value() == "right" && p.right_turn.enable) return Direction::RIGHT;
  if (attr.value() == "left" && p.left_turn.enable) return Direction::LEFT;
  return Direction::NONE;
}

void IntersectionCollisionChecker::filter_pointcloud(
  PointCloud2::ConstSharedPtr & input, PointCloud::Ptr & filtered_pointcloud,
  DebugData & debug_data) const
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

  {
    const auto voxel_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*filtered_pointcloud, *voxel_pointcloud);
    voxel_pointcloud->header.stamp = context_->data->obstacle_pointcloud->header.stamp;
    voxel_pointcloud->header.frame_id = "map";
    debug_data.voxel_points = voxel_pointcloud;
  }
}

bool IntersectionCollisionChecker::check_collision(
  DebugData & debug_data, const PointCloud::Ptr & filtered_point_cloud,
  const rclcpp::Time & time_stamp)
{
  bool is_safe = true;

  const auto & p = params_.icc_parameters;
  const auto & vel_params = p.pointcloud.velocity_estimation;

  auto ego_object_overlap_time =
    [](const double object_ttc, const std::pair<double, double> & ego_time) {
      if (object_ttc > ego_time.second) return object_ttc - ego_time.second;
      if (object_ttc < ego_time.first) return ego_time.first - object_ttc;
      return 0.0;
    };

  const auto ego_vel = context_->data->current_kinematics->twist.twist.linear.x;
  const auto close_time_th =
    ego_vel < p.filter.min_velocity ? p.min_time_horizon : abs(ego_vel / p.ego_deceleration);
  static constexpr double close_distance_threshold = 3.0;
  auto is_colliding = [&](const PCDObject & object, const std::pair<double, double> & ego_time) {
    if (!object.is_reliable) return false;
    if (object.distance_to_overlap < close_distance_threshold && ego_time.first < close_time_th)
      return true;
    if (object.is_moving && ego_object_overlap_time(object.ttc, ego_time) < p.ttc_threshold) {
      return true;
    }
    return false;
  };

  for (const auto & target_lanelet : target_lanelets_map_) {
    const auto target_ll = target_lanelet.second;
    if (!target_ll.is_active || target_ll.lanelets.empty()) continue;

    const auto pcd_object = get_pcd_object(debug_data, time_stamp, filtered_point_cloud, target_ll);
    if (!pcd_object.has_value()) continue;

    auto update_object = [&](PCDObject & object, const PCDObject & new_data) {
      const auto dt = (new_data.last_update_time - object.last_update_time).seconds();
      const auto dl = object.distance_to_overlap - new_data.distance_to_overlap;
      if (dt < 1e-6) return;  // too small time difference, skip update

      const auto raw_velocity = dl / dt;
      const auto raw_accel = std::abs(raw_velocity - object.velocity) / dt;
      object.is_reliable = object.track_duration > vel_params.observation_time;
      static constexpr double eps = 0.01;  // small epsilon to avoid division by zero
      // update velocity only if the object is not yet reliable or velocity change is within limit
      if (
        object.is_reliable && object.distance_to_overlap < close_distance_threshold &&
        new_data.distance_to_overlap < close_distance_threshold) {
        object.track_duration += dt;
      } else if (raw_accel > vel_params.reset_accel_th) {
        object.velocity = 0.0;        // reset velocity if acceleration is too high
        object.track_duration = 0.0;  // reset track duration
        object.is_reliable = false;   // reset reliability
      } else if (!object.is_reliable || raw_accel < vel_params.max_acceleration) {
        object.velocity = autoware::signal_processing::lowpassFilter(
          raw_velocity, object.velocity, 0.5);  // apply low-pass filter to velocity
        object.velocity = std::clamp(object.velocity, eps, vel_params.max_velocity);
        object.track_duration += dt;
      }

      object.last_update_time = new_data.last_update_time;
      object.pose = new_data.pose;
      object.distance_to_overlap = new_data.distance_to_overlap;
      object.delay_compensated_distance_to_overlap =
        std::max(0.0, object.distance_to_overlap - object.velocity * p.pointcloud.latency);
      object.moving_time =
        (object.velocity < p.filter.min_velocity) ? 0.0 : object.moving_time + dt;
      object.is_moving = object.moving_time > p.filter.moving_time;
      object.ttc = object.delay_compensated_distance_to_overlap / object.velocity;
    };

    if (history_.find(pcd_object->overlap_lanelet_id) == history_.end()) {
      history_[pcd_object->overlap_lanelet_id] = pcd_object.value();
    } else {
      auto & existing_object = history_[pcd_object->overlap_lanelet_id];
      update_object(existing_object, pcd_object.value());
      existing_object.is_safe = !is_colliding(existing_object, target_ll.ego_overlap_time);
      is_safe = is_safe && existing_object.is_safe;
      debug_data.pcd_objects.push_back(existing_object);
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
  DebugData & debug_data, const rclcpp::Time & time_stamp,
  const PointCloud::Ptr & filtered_point_cloud, const TargetLanelet & target_lanelet) const
{
  std::optional<PCDObject> pcd_object = std::nullopt;

  PointCloud::Ptr points_within(new PointCloud);
  const auto combine_lanelet = lanelet::utils::combineLaneletsShape(target_lanelet.lanelets);
  get_points_within(
    filtered_point_cloud, combine_lanelet.polygon2d().basicPolygon(), points_within);

  if (points_within->empty()) return pcd_object;

  PointCloud::Ptr clustered_points(new PointCloud);
  cluster_pointcloud(points_within, clustered_points, debug_data);

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
    object.overlap_point = target_lanelet.overlap_point.position;
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
  const PointCloud::Ptr & input, PointCloud::Ptr & output, DebugData & debug_data) const
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

  {
    const auto clustered_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*output, *clustered_pointcloud);
    clustered_pointcloud->header.stamp = context_->data->obstacle_pointcloud->header.stamp;
    clustered_pointcloud->header.frame_id = "map";
    debug_data.cluster_points = clustered_pointcloud;
  }
}

void IntersectionCollisionChecker::publish_markers(const DebugData & debug_data) const
{
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << std::boolalpha;
    ss << "ACTIVE: " << debug_data.is_active << "\n";
    ss << "SAFE: " << debug_data.is_safe << "\n";
    ss << "TURN: " << magic_enum::enum_name(debug_data.turn_direction) << "\n";
    ss << "INFO: " << debug_data.text << "\n";
    ss << "TRACKING OBJECTS: " << debug_data.pcd_objects.size() << "\n";
    ss << "PROCESSING TIME: " << debug_data.processing_time_detail_ms << "[ms]\n";

    StringStamped string_stamp;
    string_stamp.stamp = clock_->now();
    string_stamp.data = ss.str();
    pub_string_->publish(string_stamp);
  }

  context_->debug_pose_publisher->pushMarkers(
    collision_checker_utils::get_lanelets_marker_array(debug_data));

  if (!debug_data.is_active) return;

  context_->debug_pose_publisher->pushMarkers(
    collision_checker_utils::get_objects_marker_array(debug_data));

  if (debug_data.voxel_points) {
    pub_voxel_pointcloud_->publish(*debug_data.voxel_points);
  }

  if (debug_data.cluster_points) {
    pub_cluster_pointcloud_->publish(*debug_data.cluster_points);
  }
}

void IntersectionCollisionChecker::publish_planning_factor(const DebugData & debug_data) const
{
  if (debug_data.is_safe) return;

  SafetyFactorArray factor_array;
  factor_array.is_safe = false;
  factor_array.detail = "possible collision at intersection";
  factor_array.header.stamp = clock_->now();

  SafetyFactor factor;
  factor.type = SafetyFactor::POINTCLOUD;
  for (const auto & obj : debug_data.pcd_objects) {
    if (!obj.is_reliable) continue;
    factor.is_safe = obj.is_safe;
    factor.ttc_begin = obj.ttc;
    factor.points.push_back(obj.pose.position);
    factor_array.factors.push_back(factor);
  }

  const auto & traj_points = context_->data->current_trajectory->points;
  const auto & ego_pose = context_->data->current_kinematics->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, ego_pose, PlanningFactor::STOP, factor_array);
  planning_factor_interface_->publish();
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::IntersectionCollisionChecker,
  autoware::planning_validator::PluginInterface)
