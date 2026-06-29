// Copyright 2020 Tier IV, Inc.
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

#include "node.hpp"

#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace autoware::surround_obstacle_checker
{

using autoware_perception_msgs::msg::ObjectClassification;

SurroundObstacleCheckerNode::SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("surround_obstacle_checker_node", node_options)
{
  label_map_ = {
    {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"},     {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
  // Parameters
  {
    param_listener_ = std::make_shared<surround_obstacle_checker_node::ParamListener>(
      this->get_node_parameters_interface());

    logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  }

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  proximity_checker_ = std::make_unique<obstacle_proximity_checker::ProximityChecker>(
    toProximityCheckerParameters(), vehicle_info_);

  // Publishers
  pub_clear_velocity_limit_ = this->create_publisher<VelocityLimitClearCommand>(
    "~/output/velocity_limit_clear_command", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_ = this->create_publisher<VelocityLimit>(
    "~/output/max_velocity", rclcpp::QoS{1}.transient_local());
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&SurroundObstacleCheckerNode::onTimer, this));

  // Stop Checker
  vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);

  // Debug
  odometry_ptr_ = std::make_shared<nav_msgs::msg::Odometry>();
  {
    const auto param = param_listener_->get_params();
    const auto check_distances = getCheckDistances(param.debug_footprint_label);
    debug_ptr_ = std::make_shared<SurroundObstacleCheckerDebugNode>(
      vehicle_info_, param.debug_footprint_label, check_distances.at(0), check_distances.at(1),
      check_distances.at(2), param.surround_check_hysteresis_distance, odometry_ptr_->pose.pose,
      this->get_clock(), *this);
  }
}

std::array<double, 3> SurroundObstacleCheckerNode::getCheckDistances(
  const std::string & str_label) const
{
  const auto param = param_listener_->get_params();
  const auto & obstacle_param = param.obstacle_types_map.at(str_label);
  return {
    obstacle_param.surround_check_front_distance, obstacle_param.surround_check_side_distance,
    obstacle_param.surround_check_back_distance};
}

bool SurroundObstacleCheckerNode::getUseDynamicObject() const
{
  const auto param = param_listener_->get_params();
  bool use_dynamic_object = false;
  for (const auto & label_pair : label_map_) {
    use_dynamic_object |= param.object_types_map.at(label_pair.second).enable_check;
  }
  return use_dynamic_object;
}

obstacle_proximity_checker::Parameters SurroundObstacleCheckerNode::toProximityCheckerParameters()
  const
{
  const auto param = param_listener_->get_params();

  obstacle_proximity_checker::Parameters parameters;
  parameters.pointcloud_enable_check = param.pointcloud.enable_check;

  for (const auto & [label, object_type_param] : param.object_types_map) {
    parameters.object_type_enable_check[label] = object_type_param.enable_check;
  }

  for (const auto & [label, obstacle_type_param] : param.obstacle_types_map) {
    obstacle_proximity_checker::ObstacleTypeParameters obstacle_parameters;
    obstacle_parameters.surround_check_front_distance =
      obstacle_type_param.surround_check_front_distance;
    obstacle_parameters.surround_check_side_distance =
      obstacle_type_param.surround_check_side_distance;
    obstacle_parameters.surround_check_back_distance =
      obstacle_type_param.surround_check_back_distance;
    parameters.obstacle_types_map[label] = obstacle_parameters;
  }

  return parameters;
}

obstacle_proximity_checker::Inputs SurroundObstacleCheckerNode::toProximityCheckerInputs() const
{
  obstacle_proximity_checker::Inputs inputs;
  inputs.ego_pose = odometry_ptr_->pose.pose;
  inputs.objects = object_ptr_;

  if (!pointcloud_ptr_) return inputs;

  const auto transform_stamped =
    getTransform("base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp, 0.5);

  if (!transform_stamped.has_value()) return inputs;

  Eigen::Affine3f isometry =
    tf2::transformToEigen(transform_stamped.value().transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*pointcloud_ptr_, transformed_pointcloud);
  autoware_utils::transform_pointcloud(transformed_pointcloud, transformed_pointcloud, isometry);

  inputs.pointcloud_in_base_link = transformed_pointcloud.makeShared();

  return inputs;
}

void SurroundObstacleCheckerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic();

  odometry_ptr_ = sub_odometry_.take_data();
  pointcloud_ptr_ = sub_pointcloud_.take_data();
  object_ptr_ = sub_dynamic_objects_.take_data();

  if (!odometry_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current velocity...");
    return;
  }

  const auto param = param_listener_->get_params();
  proximity_checker_->update_parameters(toProximityCheckerParameters());
  const auto use_dynamic_object = getUseDynamicObject();

  if (param.publish_debug_footprints) {
    debug_ptr_->publishFootprints();
  }

  if (param.pointcloud.enable_check && !pointcloud_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for pointcloud info...");
  }

  if (use_dynamic_object && !object_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
  }

  if (!param.pointcloud.enable_check && !use_dynamic_object) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "Surround obstacle check is disabled for all dynamic object types and for pointcloud check.");
  }

  const double contact_distance_threshold =
    state_ == State::STOP ? param.surround_check_hysteresis_distance : 1e-3;
  const auto proximity_result =
    proximity_checker_->check(toProximityCheckerInputs(), contact_distance_threshold);
  const auto is_vehicle_stopped = vehicle_stop_checker_->isVehicleStopped();

  switch (state_) {
    case State::PASS: {
      bool is_stop_required = false;
      std::tie(is_stop_required, last_obstacle_found_time_) = isStopRequired(
        proximity_result.is_obstacle_found, is_vehicle_stopped, state_, last_obstacle_found_time_,
        param.state_clear_time);
      if (!is_stop_required) {
        break;
      }

      state_ = State::STOP;

      auto velocity_limit = std::make_shared<VelocityLimit>();
      velocity_limit->stamp = this->now();
      velocity_limit->max_velocity = 0.0;
      velocity_limit->use_constraints = false;
      velocity_limit->sender = "surround_obstacle_checker";

      pub_velocity_limit_->publish(*velocity_limit);

      // do not start when there is a obstacle near the ego vehicle.
      RCLCPP_WARN(get_logger(), "do not start because there is obstacle near the ego vehicle.");

      break;
    }

    case State::STOP: {
      bool is_stop_required = false;
      std::tie(is_stop_required, last_obstacle_found_time_) = isStopRequired(
        proximity_result.is_obstacle_found, is_vehicle_stopped, state_, last_obstacle_found_time_,
        param.state_clear_time);
      if (is_stop_required) {
        break;
      }

      state_ = State::PASS;

      auto velocity_limit_clear_command = std::make_shared<VelocityLimitClearCommand>();
      velocity_limit_clear_command->stamp = this->now();
      velocity_limit_clear_command->command = true;
      velocity_limit_clear_command->sender = "surround_obstacle_checker";

      pub_clear_velocity_limit_->publish(*velocity_limit_clear_command);

      break;
    }

    default:
      break;
  }

  if (proximity_result.nearest_obstacle.has_value()) {
    debug_ptr_->pushStopObstacle(proximity_result.nearest_obstacle);
  }

  if (state_ == State::STOP) {
    debug_ptr_->pushPose(odometry_ptr_->pose.pose, PoseType::NoStart);
  }

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);

  debug_ptr_->publish();
}

std::optional<geometry_msgs::msg::TransformStamped> SurroundObstacleCheckerNode::getTransform(
  const std::string & source, const std::string & target, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_.lookupTransform(source, target, stamp, tf2::durationFromSec(duration_sec));
  } catch (const tf2::TransformException & ex) {
    return {};
  }

  return transform_stamped;
}

auto SurroundObstacleCheckerNode::isStopRequired(
  const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
  const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold) const
  -> std::pair<bool, std::optional<rclcpp::Time>>
{
  if (!is_vehicle_stopped) {
    return std::make_pair(false, std::nullopt);
  }

  if (is_obstacle_found) {
    return std::make_pair(true, this->now());
  }

  if (state != State::STOP) {
    return std::make_pair(false, std::nullopt);
  }

  // Keep stop state
  if (last_obstacle_found_time.has_value()) {
    const auto elapsed_time = this->now() - last_obstacle_found_time.value();
    if (elapsed_time.seconds() <= time_threshold) {
      return std::make_pair(true, last_obstacle_found_time.value());
    }
  }

  return std::make_pair(false, std::nullopt);
}

}  // namespace autoware::surround_obstacle_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::surround_obstacle_checker::SurroundObstacleCheckerNode)
