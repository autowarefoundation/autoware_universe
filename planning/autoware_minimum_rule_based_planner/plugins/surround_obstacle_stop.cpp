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

#include "surround_obstacle_stop.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>

namespace
{
using autoware::obstacle_proximity_checker::ObstacleTypeParameters;
using autoware::obstacle_proximity_checker::Parameters;
using SurroundObstacleStopParams =
  autoware::minimum_rule_based_planner::plugin::MinimumRuleBasedPlannerParams::SurroundObstacleStop;

ObstacleTypeParameters to_obstacle_type_parameters(
  const double front_distance, const double side_distance, const double back_distance)
{
  ObstacleTypeParameters parameters;
  parameters.surround_check_front_distance = front_distance;
  parameters.surround_check_side_distance = side_distance;
  parameters.surround_check_back_distance = back_distance;
  return parameters;
}

Parameters to_proximity_checker_parameters(const SurroundObstacleStopParams & params)
{
  Parameters parameters;
  parameters.pointcloud_enable_check = params.use_pointcloud;

  const std::unordered_set<std::string> enabled_object_types(
    params.object_types.begin(), params.object_types.end());

  const auto set_object_enable = [&](const std::string & label) {
    parameters.object_type_enable_check[label] = enabled_object_types.count(label) > 0;
  };
  set_object_enable("unknown");
  set_object_enable("car");
  set_object_enable("truck");
  set_object_enable("bus");
  set_object_enable("trailer");
  set_object_enable("motorcycle");
  set_object_enable("bicycle");
  set_object_enable("pedestrian");
  set_object_enable("hazard");
  set_object_enable("animal");

  const auto & front = params.front_distance_th;
  const auto & side = params.side_distance_th;
  const auto & back = params.back_distance_th;

  parameters.obstacle_types_map["pointcloud"] =
    to_obstacle_type_parameters(front.pointcloud, side.pointcloud, back.pointcloud);
  parameters.obstacle_types_map["unknown"] =
    to_obstacle_type_parameters(front.unknown, side.unknown, back.unknown);
  parameters.obstacle_types_map["car"] = to_obstacle_type_parameters(front.car, side.car, back.car);
  parameters.obstacle_types_map["truck"] =
    to_obstacle_type_parameters(front.truck, side.truck, back.truck);
  parameters.obstacle_types_map["bus"] = to_obstacle_type_parameters(front.bus, side.bus, back.bus);
  parameters.obstacle_types_map["trailer"] =
    to_obstacle_type_parameters(front.trailer, side.trailer, back.trailer);
  parameters.obstacle_types_map["motorcycle"] =
    to_obstacle_type_parameters(front.motorcycle, side.motorcycle, back.motorcycle);
  parameters.obstacle_types_map["bicycle"] =
    to_obstacle_type_parameters(front.bicycle, side.bicycle, back.bicycle);
  parameters.obstacle_types_map["pedestrian"] =
    to_obstacle_type_parameters(front.pedestrian, side.pedestrian, back.pedestrian);
  parameters.obstacle_types_map["hazard"] =
    to_obstacle_type_parameters(front.hazard, side.hazard, back.hazard);
  parameters.obstacle_types_map["animal"] =
    to_obstacle_type_parameters(front.animal, side.animal, back.animal);

  return parameters;
}
}  // namespace

namespace autoware::minimum_rule_based_planner::plugin
{
namespace utils = autoware::trajectory_modifier::utils;

void SurroundObstacleStop::on_initialize(const MinimumRuleBasedPlannerParams & params)
{
  params_ = params.surround_obstacle_stop;

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      get_node_ptr(), "backup_planner_surround_obstacle_stop");

  pub_debug_text_ =
    get_node_ptr()->create_publisher<StringStamped>("~/surround_obstacle_stop/debug/text", 1);

  proximity_checker_ = std::make_unique<obstacle_proximity_checker::ProximityChecker>(
    to_proximity_checker_parameters(params_), context_->vehicle_info);
}

void SurroundObstacleStop::update_params(const MinimumRuleBasedPlannerParams & params)
{
  params_ = params.surround_obstacle_stop;
  proximity_checker_->update_parameters(to_proximity_checker_parameters(params_));
}

void SurroundObstacleStop::run(TrajectoryPoints & traj_points, const ModifierData & data)
{
  proximity_check_result_ = std::nullopt;

  if (!is_stop_required(traj_points, data)) {
    publish_debug_string(false);
    return;
  }

  set_stop_point(traj_points, data);
  publish_debug_string(true);
}

bool SurroundObstacleStop::check_inputs(const ModifierData & data) const
{
  if (!data.odometry_ptr) {
    return false;
  }

  if (!params_.use_pointcloud && !params_.use_objects) {
    return false;
  }

  const bool has_pointcloud = params_.use_pointcloud && data.obstacle_pointcloud_ptr;
  const bool has_objects = params_.use_objects && data.predicted_objects_ptr;

  return has_pointcloud || has_objects;
}

obstacle_proximity_checker::Inputs SurroundObstacleStop::to_proximity_checker_inputs(
  const ModifierData & data) const
{
  obstacle_proximity_checker::Inputs checker_inputs;
  checker_inputs.ego_pose = data.odometry_ptr->pose.pose;
  checker_inputs.objects = data.predicted_objects_ptr;

  if (!data.obstacle_pointcloud_ptr || data.obstacle_pointcloud_ptr->data.empty()) {
    return checker_inputs;
  }

  const auto transform_stamped = get_transform(
    "base_link", data.obstacle_pointcloud_ptr->header.frame_id,
    data.obstacle_pointcloud_ptr->header.stamp, 0.5);

  if (!transform_stamped.has_value()) return checker_inputs;

  Eigen::Affine3f isometry =
    tf2::transformToEigen(transform_stamped.value().transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*data.obstacle_pointcloud_ptr, transformed_pointcloud);
  autoware_utils::transform_pointcloud(transformed_pointcloud, transformed_pointcloud, isometry);

  checker_inputs.pointcloud_in_base_link = transformed_pointcloud.makeShared();

  return checker_inputs;
}

std::optional<geometry_msgs::msg::TransformStamped> SurroundObstacleStop::get_transform(
  const std::string & target, const std::string & source, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped = context_->tf_buffer.lookupTransform(
      target, source, stamp, tf2::durationFromSec(duration_sec));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000, "no transform found for pointcloud: %s",
      ex.what());
    return {};
  }

  return transform_stamped;
}

bool SurroundObstacleStop::is_obstacle_nearby(const ModifierData & data)
{
  const double contact_distance_threshold = is_stop_active_ ? params_.hysteresis_distance : 1e-3;

  if (!proximity_check_result_.has_value()) {
    proximity_check_result_ =
      proximity_checker_->check(to_proximity_checker_inputs(data), contact_distance_threshold);
  }

  const auto & result = proximity_check_result_.value();
  if (result.is_obstacle_found) {
    last_obstacle_found_time_ = get_clock()->now();
    is_stop_active_ = true;
    return true;
  }

  if (is_stop_active_ && last_obstacle_found_time_.has_value()) {
    const auto elapsed_time = get_clock()->now() - last_obstacle_found_time_.value();
    if (elapsed_time.seconds() <= params_.hysteresis_time) {
      return true;
    }
  }

  is_stop_active_ = false;
  last_obstacle_found_time_ = std::nullopt;
  return false;
}

bool SurroundObstacleStop::is_stop_required(
  const TrajectoryPoints & traj_points, const ModifierData & data)
{
  if (!params_.enable || !check_inputs(data)) {
    return false;
  }

  if (
    utils::is_stop_trajectory(traj_points, params_.ego_stopped_vel_th) ||
    utils::is_ego_vehicle_moving(data.odometry_ptr->twist.twist, params_.ego_stopped_vel_th)) {
    is_stop_active_ = false;
    last_obstacle_found_time_ = std::nullopt;
    return false;
  }

  return is_obstacle_nearby(data);
}

void SurroundObstacleStop::set_stop_point(TrajectoryPoints & traj_points, const ModifierData & data)
{
  if (traj_points.empty()) return;

  const auto & ego_pose = data.odometry_ptr->pose.pose;

  // Insert a zero-velocity stop point at the ego pose while keeping the trajectory shape.
  const auto stop_index = motion_utils::insertStopPoint(ego_pose, 0.0, traj_points);
  if (!stop_index) {
    // Fall back to stopping from the front of the trajectory when insertion at ego fails.
    for (auto & point : traj_points) {
      point.longitudinal_velocity_mps = 0.0;
      point.acceleration_mps2 = 0.0;
    }
  }

  const auto & stop_pose = stop_index ? traj_points.at(stop_index.value()).pose : ego_pose;

  planning_factor_interface_->add(
    traj_points, ego_pose, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[Backup Planner SurroundObstacleStop] Inserted stop point at ego pose due to nearby "
    "obstacle.");
}

void SurroundObstacleStop::publish_debug_string(const bool is_active) const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << std::boolalpha;
  ss << "SURROUND OBSTACLE STOP (Backup Planner):" << "\n";
  ss << "\t\t" << "ACTIVE: " << is_active << "\n";
  ss << "\t\t" << "STOP_ACTIVE: " << is_stop_active_ << "\n";

  StringStamped string_stamp;
  string_stamp.stamp = get_clock()->now();
  string_stamp.data = ss.str();
  pub_debug_text_->publish(string_stamp);
}

}  // namespace autoware::minimum_rule_based_planner::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::minimum_rule_based_planner::plugin::SurroundObstacleStop,
  autoware::minimum_rule_based_planner::plugin::PluginInterface)
