// Copyright 2025 Tier IV, Inc.
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

#include "autoware/planning_validator_collision_checker/collision_checker.hpp"

#include "autoware/planning_validator_collision_checker/utils.hpp"

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/transform/transforms.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/strategies/cartesian/buffer_point_square.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_routing/RoutingGraph.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using autoware_utils::get_or_declare_parameter;

void CollisionChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;

  clock_ = node.get_clock();
  logger_ = node.get_logger();
  context_ = context;

  setup_parameters(node);

  setup_diag();
}

void CollisionChecker::setup_parameters(rclcpp::Node & node)
{
  params_.enable = get_or_declare_parameter<bool>(node, "collision_checker.enable");
  params_.is_critical = get_or_declare_parameter<bool>(node, "collision_checker.is_critical");
  params_.detection_range =
    get_or_declare_parameter<double>(node, "collision_checker.detection_range");
  params_.ttc_threshold = get_or_declare_parameter<double>(node, "collision_checker.ttc_threshold");

  params_.right_turn.enable =
    get_or_declare_parameter<bool>(node, "collision_checker.right_turn.enable");
  params_.right_turn.check_oncoming_lanes =
    get_or_declare_parameter<bool>(node, "collision_checker.right_turn.check_oncoming_lanes");
  params_.right_turn.check_crossing_lanes =
    get_or_declare_parameter<bool>(node, "collision_checker.right_turn.check_crossing_lanes");

  params_.left_turn.enable =
    get_or_declare_parameter<bool>(node, "collision_checker.left_turn.enable");
  params_.left_turn.check_oncoming_lanes =
    get_or_declare_parameter<bool>(node, "collision_checker.left_turn.check_oncoming_lanes");
  params_.left_turn.check_crossing_lanes =
    get_or_declare_parameter<bool>(node, "collision_checker.left_turn.check_crossing_lanes");

  params_.pointcloud.height_buffer =
    get_or_declare_parameter<double>(node, "collision_checker.pointcloud.height_buffer");
  params_.pointcloud.min_height =
    get_or_declare_parameter<double>(node, "collision_checker.pointcloud.min_height");
  params_.pointcloud.voxel_grid_filter.x =
    get_or_declare_parameter<double>(node, "collision_checker.pointcloud.voxel_grid_filter.x");
  params_.pointcloud.voxel_grid_filter.y =
    get_or_declare_parameter<double>(node, "collision_checker.pointcloud.voxel_grid_filter.y");
  params_.pointcloud.voxel_grid_filter.z =
    get_or_declare_parameter<double>(node, "collision_checker.pointcloud.voxel_grid_filter.z");
  params_.pointcloud.clustering.tolerance =
    get_or_declare_parameter<double>(node, "collision_checker.pointcloud.clustering.tolerance");
  params_.pointcloud.clustering.min_height =
    get_or_declare_parameter<double>(node, "collision_checker.pointcloud.clustering.min_height");
  params_.pointcloud.clustering.min_size =
    get_or_declare_parameter<int>(node, "collision_checker.pointcloud.clustering.min_size");
  params_.pointcloud.clustering.max_size =
    get_or_declare_parameter<int>(node, "collision_checker.pointcloud.clustering.max_size");
}

void CollisionChecker::setup_diag()
{
  context_->add_diag(
    "planning_validation_collision_check", context_->validation_status->is_valid_collision_check,
    "risk of collision at intersection turn", params_.is_critical);
}

void CollisionChecker::validate(bool & is_critical)
{
  auto skip_validation = [&](const std::string & reason) {
    RCLCPP_WARN(logger_, "%s", reason.c_str());
    is_critical = false;
  };

  if (!context_->data->current_pointcloud) {
    return skip_validation("point cloud data is not available, skipping collision check.");
  }

  if (!context_->route_handler->isHandlerReady()) {
    return skip_validation("route handler is not ready, skipping collision check.");
  }

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  filter_pointcloud(context_->data->current_pointcloud, filtered_pointcloud);

  const auto base_to_front_length =
    context_->vehicle_info.front_overhang_m + context_->vehicle_info.wheel_base_m;
  const auto ego_front_pose = autoware_utils::calc_offset_pose(
    context_->data->current_kinematics->pose.pose, base_to_front_length, 0.0, 0.0);
  const auto trajectory_points = collision_checker_utils::trim_trajectory_points(
    context_->data->current_trajectory->points, ego_front_pose);

  CollisionCheckerLanelets lanelets;
  const auto turn_direction = get_lanelets(lanelets, trajectory_points);

  set_lanelets_debug_marker(lanelets);

  if (turn_direction == Direction::NONE || filtered_pointcloud->empty()) return;

  if (lanelets.trajectory_lanelets.empty()) {
    return skip_validation("failed to get trajectory lanelets, skipping collision check.");
  }

  if (lanelets.target_lanelets.empty()) {
    return skip_validation("failed to get target lanelets, skipping collision check.");
  }

  const auto is_safe = check_collision(
    lanelets.target_lanelets, filtered_pointcloud,
    context_->data->current_pointcloud->header.stamp);

  context_->validation_status->is_valid_collision_check = is_safe;
}

Direction CollisionChecker::get_lanelets(
  CollisionCheckerLanelets & lanelets, const TrajectoryPoints & trajectory_points) const
{
  const auto & ego_pose = context_->data->current_kinematics->pose.pose;
  try {
    collision_checker_utils::set_trajectory_lanelets(
      trajectory_points, *context_->route_handler, ego_pose, lanelets);
  } catch (const std::logic_error & e) {
    RCLCPP_ERROR(logger_, "failed to get trajectory lanelets: %s", e.what());
    return Direction::NONE;
  }

  const auto turn_direction = get_turn_direction(lanelets.trajectory_lanelets);

  if (turn_direction == Direction::NONE) return turn_direction;

  if (turn_direction == Direction::RIGHT) {
    collision_checker_utils::set_right_turn_target_lanelets(
      trajectory_points, *context_->route_handler, lanelets);
  } else {
    collision_checker_utils::set_left_turn_target_lanelets(
      *context_->route_handler, trajectory_points, lanelets);
  }

  return turn_direction;
}

Direction CollisionChecker::get_turn_direction(
  const lanelet::ConstLanelets & trajectory_lanelets) const
{
  for (const auto & lanelet : trajectory_lanelets) {
    if (!lanelet.hasAttribute("turn_direction")) continue;
    const lanelet::Attribute & attr = lanelet.attribute("turn_direction");
    if (attr.value() == "right" && params_.right_turn.enable) return Direction::RIGHT;
    if (attr.value() == "left" && params_.left_turn.enable) return Direction::LEFT;
  }
  return Direction::NONE;
}

void CollisionChecker::filter_pointcloud(
  PointCloud2::ConstSharedPtr & input, PointCloud::Ptr & filtered_pointcloud) const
{
  if (input->data.empty()) return;

  pcl::fromROSMsg(*input, *filtered_pointcloud);

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(filtered_pointcloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(-1.0 * params_.detection_range, params_.detection_range);
    filter.filter(*filtered_pointcloud);
  }

  if (filtered_pointcloud->empty()) return;

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(filtered_pointcloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(
      params_.pointcloud.min_height,
      context_->vehicle_info.vehicle_height_m + params_.pointcloud.height_buffer);
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
    const auto & p = params_.pointcloud;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(filtered_pointcloud);
    filter.setLeafSize(p.voxel_grid_filter.x, p.voxel_grid_filter.y, p.voxel_grid_filter.z);
    filter.filter(*filtered_pointcloud);
  }
}

bool CollisionChecker::check_collision(
  const TargetLanelets & target_lanelets, const PointCloud::Ptr & filtered_point_cloud,
  const rclcpp::Time & time_stamp)
{
  bool is_safe = true;
  for (const auto & target_lanelet : target_lanelets) {
    RCLCPP_WARN(
      logger_, "Checking collision for lanelet ID: %ld | Ego time to reach: %f", target_lanelet.id,
      target_lanelet.ego_time_to_reach);
    if (target_lanelet.lanelets.empty()) continue;

    const auto pcd_object = get_pcd_object(time_stamp, filtered_point_cloud, target_lanelet);
    if (!pcd_object.has_value()) continue;

    auto calculate_velocity =
      [](const double & dl, const double & dt, const double & last_velocity) {
        const auto max_accel = 10.0;
        if (dt <= 1e-6) return last_velocity;
        const auto raw_velocity = dl / dt;
        if (std::abs(raw_velocity - last_velocity) / dt > max_accel) {
          return 0.0;  // too high acceleration, reset velocity
        }
        return autoware::signal_processing::lowpassFilter(raw_velocity, last_velocity, 0.5);
      };

    if (history_.find(pcd_object->overlap_lanelet_id) == history_.end()) {
      history_[pcd_object->overlap_lanelet_id] = pcd_object.value();
    } else {
      auto & existing_object = history_[pcd_object->overlap_lanelet_id];
      existing_object.pose = pcd_object->pose;
      const auto dt = (time_stamp - existing_object.last_update_time).seconds();
      const auto dl = pcd_object->distance_to_overlap - existing_object.distance_to_overlap;
      existing_object.track_duration += dt;
      existing_object.last_update_time = time_stamp;
      existing_object.distance_to_overlap = pcd_object->distance_to_overlap;
      existing_object.velocity = calculate_velocity(dl, dt, existing_object.velocity);
      existing_object.ttc = existing_object.distance_to_overlap / existing_object.velocity;
      if (
        std::abs(existing_object.ttc - target_lanelet.ego_time_to_reach) < params_.ttc_threshold) {
        is_safe = false;
        context_->debug_pose_publisher->pushPointMarker(
          existing_object.pose.position, "collision_checker_colliding_objects", 0);
      } else {
        context_->debug_pose_publisher->pushPointMarker(
          existing_object.pose.position, "collision_checker_checked_objects", 1);
      }
    }
  }

  auto itr = history_.begin();
  while (itr != history_.end()) {
    if ((clock_->now() - itr->second.last_update_time).seconds() > 1.0) {
      itr = history_.erase(itr);
    } else {
      itr++;
    }
  }

  return is_safe;
}

std::optional<PCDObject> CollisionChecker::get_pcd_object(
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

  const auto overlap_center_pose =
    lanelet::utils::getClosestCenterPose(combine_lanelet, target_lanelet.overlap_point);
  const auto overlap_arc_coord =
    lanelet::utils::getArcCoordinates(target_lanelet.lanelets, overlap_center_pose);
  const auto min_arc_length = std::numeric_limits<double>::max();
  for (const auto & p : *clustered_points) {
    const auto p_geom = autoware_utils::create_point(p.x, p.y, p.z);
    const auto center_pose = lanelet::utils::getClosestCenterPose(combine_lanelet, p_geom);
    const auto arc_coord = lanelet::utils::getArcCoordinates(target_lanelet.lanelets, center_pose);
    const auto arc_length_to_overlap = overlap_arc_coord.length - arc_coord.length;
    if (
      arc_length_to_overlap < std::numeric_limits<double>::epsilon() ||
      arc_length_to_overlap > min_arc_length) {
      continue;
    }

    PCDObject object;
    object.last_update_time = time_stamp;
    object.pose.position = p_geom;
    object.overlap_lanelet_id = target_lanelet.id;
    object.track_duration = 0.0;
    object.distance_to_overlap = arc_length_to_overlap;
    object.velocity = 0.0;
    object.ttc = std::numeric_limits<double>::max();
    pcd_object = object;
  }
  return pcd_object;
}

void CollisionChecker::get_points_within(
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

void CollisionChecker::cluster_pointcloud(
  const PointCloud::Ptr & input, PointCloud::Ptr & output) const
{
  if (input->empty()) return;

  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params_.pointcloud.clustering.tolerance);
    ec.setMinClusterSize(params_.pointcloud.clustering.min_size);
    ec.setMaxClusterSize(params_.pointcloud.clustering.max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_idx);
    return cluster_idx;
  });

  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    bool cluster_above_height_threshold{false};
    for (const auto & index : indices.indices) {
      const auto & p = (*input)[index];
      cluster_above_height_threshold |= (p.z > params_.pointcloud.clustering.min_height);
      cluster->push_back(p);
    }
    if (!cluster_above_height_threshold) continue;

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    std::vector<pcl::Vertices> polygons;
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull, polygons);
    for (const auto & p : *surface_hull) {
      output->push_back(p);
    }
  }
}

void CollisionChecker::set_lanelets_debug_marker(const CollisionCheckerLanelets & lanelets) const
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
    for (const auto & t_l : lanelets.target_lanelets) {
      for (const auto & ll : t_l.lanelets) {
        ll_polygons.push_back(ll.polygon2d().basicPolygon());
      }
      context_->debug_pose_publisher->pushPointMarker(
        t_l.overlap_point, "collision_checker_target_lanelets", 2);
    }
    if (!ll_polygons.empty()) {
      context_->debug_pose_publisher->pushLaneletPolygonsMarker(
        ll_polygons, "collision_checker_target_lanelets", 2);
    }
  }
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::CollisionChecker, autoware::planning_validator::PluginInterface)
