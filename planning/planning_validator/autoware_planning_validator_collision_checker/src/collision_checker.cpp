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

#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/transform/transforms.hpp>

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

  const auto turn_direction = get_turn_direction();

  if (turn_direction == Direction::NONE) {
    return skip_validation("no turn direction detected, skipping collision check.");
  }
}

Direction CollisionChecker::get_turn_direction() const
{
  const auto & ego_pose = context_->data->current_kinematics->pose.pose;

  lanelet::ConstLanelet closest_lanelet;
  if (!context_->route_handler->getClosestLaneletWithinRoute(ego_pose, &closest_lanelet)) {
    RCLCPP_ERROR(logger_, "failed to get closest lanelet within route");
    return Direction::NONE;
  }

  const auto & trajectory = *context_->data->current_trajectory;
  const auto forward_trajectory_length = autoware::motion_utils::calcSignedArcLength(
    trajectory.points, ego_pose.position, trajectory.points.size() - 1);

  const auto & lanelet_sequence =
    context_->route_handler->getLaneletSequence(closest_lanelet, 0.0, forward_trajectory_length);

  if (lanelet_sequence.empty()) {
    RCLCPP_ERROR(logger_, "failed to get route lanelet sequence");
    return Direction::NONE;
  }

  for (const auto & lanelet : lanelet_sequence) {
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

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::CollisionChecker, autoware::planning_validator::PluginInterface)
