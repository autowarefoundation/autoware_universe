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

#include "autoware/planning_validator/node.hpp"

#include "autoware/planning_validator/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>

#include <angles/angles/angles.h>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

PlanningValidatorNode::PlanningValidatorNode(const rclcpp::NodeOptions & options)
: Node("planning_validator_node", options)
{
  // trajectory subscriber
  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&PlanningValidatorNode::onTrajectory, this, std::placeholders::_1));

  // publishers
  pub_traj_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_status_ = create_publisher<PlanningValidatorStatus>("~/output/validation_status", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);
  pub_processing_time_ms_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  context_ = std::make_shared<PlanningValidatorContext>(this);
  context_->set_diag_id("planning_validator");

  setupParameters();

  // Initialize Manager
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
    if (name.empty()) {
      break;
    }
    manager_.load_plugin(*this, name, context_);
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void PlanningValidatorNode::setupParameters()
{
  auto & p = context_->params;

  const auto value = declare_parameter<int>("default_handling_type");
  try {
    p.default_handling_type = get_handling_type(value);
  } catch (const std::exception & e) {
    throw std::invalid_argument(e.what());
  }

  p.publish_diag = declare_parameter<bool>("publish_diag");
  p.diag_error_count_threshold = declare_parameter<int>("diag_error_count_threshold");
  p.display_on_terminal = declare_parameter<bool>("display_on_terminal");

  p.soft_stop_deceleration = declare_parameter<double>("soft_stop_deceleration");
  p.soft_stop_jerk_lim = declare_parameter<double>("soft_stop_jerk_lim");
}

bool PlanningValidatorNode::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s.c_str());
    return false;
  };

  std::string msg;
  if (!context_->data->is_ready(msg)) {
    return waiting(msg);
  }
  return true;
}

void PlanningValidatorNode::setData(const Trajectory::ConstSharedPtr & traj_msg)
{
  auto & data = context_->data;
  data->current_kinematics = sub_kinematics_.take_data();
  data->current_acceleration = sub_acceleration_.take_data();
  data->obstacle_pointcloud = sub_pointcloud_.take_data();
  data->set_current_trajectory(traj_msg);
  data->set_route(sub_route_.take_data());
  data->set_map(sub_lanelet_map_bin_.take_data());
}

void PlanningValidatorNode::onTrajectory(const Trajectory::ConstSharedPtr & traj_msg)
{
  stop_watch_.tic(__func__);

  setData(traj_msg);

  if (!isDataReady()) return;

  context_->init_validation_status();
  context_->reset_handling();

  context_->data->set_nearest_trajectory_indices();

  context_->debug_pose_publisher->clearMarkers();

  manager_.validate();

  auto & s = context_->validation_status;
  s->invalid_count = isAllValid(*s) ? 0 : s->invalid_count + 1;

  context_->update_diag();

  publishTrajectory();

  // for debug
  publishProcessingTime(stop_watch_.toc(__func__));
  publishDebugInfo();
  displayStatus();
}

bool PlanningValidatorNode::isAllValid(const PlanningValidatorStatus & s) const
{
  return s.is_valid_size && s.is_valid_finite_value && s.is_valid_interval &&
         s.is_valid_relative_angle && s.is_valid_curvature && s.is_valid_lateral_acc &&
         s.is_valid_lateral_jerk && s.is_valid_longitudinal_max_acc &&
         s.is_valid_longitudinal_min_acc && s.is_valid_steering && s.is_valid_steering_rate &&
         s.is_valid_velocity_deviation && s.is_valid_distance_deviation &&
         s.is_valid_longitudinal_distance_deviation && s.is_valid_forward_trajectory_length &&
         s.is_valid_latency && s.is_valid_yaw_deviation && s.is_valid_trajectory_shift &&
         s.is_valid_intersection_collision_check && s.is_valid_rear_collision_check;
}

void PlanningValidatorNode::publishTrajectory()
{
  const auto & status = *context_->validation_status;
  const auto & data = context_->data;
  // Validation check is all green. Publish the trajectory.
  if (isAllValid(status)) {
    pub_traj_->publish(*data->current_trajectory);
    published_time_publisher_->publish_if_subscribed(
      pub_traj_, data->current_trajectory->header.stamp);
    data->last_valid_trajectory = data->current_trajectory;
    soft_stop_trajectory_ = nullptr;
    return;
  }

  const auto & params = context_->params;

  //  ----- invalid factor is found. Publish previous trajectory. -----

  const auto handling_type = context_->get_handling();

  if (handling_type == InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS) {
    pub_traj_->publish(*data->current_trajectory);
    published_time_publisher_->publish_if_subscribed(
      pub_traj_, data->current_trajectory->header.stamp);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Caution! Invalid Trajectory published.");
    return;
  }

  if (!data->last_valid_trajectory) {
    // trajectory is not published.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Invalid Trajectory detected, no valid trajectory found in the past. Trajectory is not "
      "published.");
    return;
  }

  if (handling_type == InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT) {
    const auto & pub_trajectory = *data->last_valid_trajectory;
    pub_traj_->publish(pub_trajectory);
    published_time_publisher_->publish_if_subscribed(pub_traj_, pub_trajectory.header.stamp);
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Use previous trajectory.");
    return;
  }

  if (handling_type == InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT_WITH_SOFT_STOP) {
    if (!soft_stop_trajectory_) {
      const auto nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        data->last_valid_trajectory->points, data->current_kinematics->pose.pose);
      soft_stop_trajectory_ = std::make_shared<Trajectory>(planning_validator::getStopTrajectory(
        *data->last_valid_trajectory, nearest_idx, data->current_kinematics->twist.twist.linear.x,
        data->current_acceleration->accel.accel.linear.x, params.soft_stop_deceleration,
        params.soft_stop_jerk_lim));
    }
    const auto & pub_trajectory = *soft_stop_trajectory_;
    pub_traj_->publish(pub_trajectory);
    published_time_publisher_->publish_if_subscribed(pub_traj_, pub_trajectory.header.stamp);
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Use soft stop trajectory.");
  }
}

void PlanningValidatorNode::publishProcessingTime(const double processing_time_ms)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = processing_time_ms;
  pub_processing_time_ms_->publish(msg);
}

void PlanningValidatorNode::publishDebugInfo()
{
  const auto & status = context_->validation_status;
  status->stamp = get_clock()->now();
  pub_status_->publish(*status);

  const auto & data = context_->data;

  if (!isAllValid(*status)) {
    geometry_msgs::msg::Pose front_pose = data->current_kinematics->pose.pose;
    shiftPose(
      front_pose, context_->vehicle_info.front_overhang_m + context_->vehicle_info.wheel_base_m);
    auto offset_pose = front_pose;
    shiftPose(offset_pose, 0.5);
    context_->debug_pose_publisher->pushVirtualWall(front_pose);
    const auto status_debug_str = context_->debug_pose_publisher->getStatusDebugString(*status);
    context_->debug_pose_publisher->pushWarningMsg(
      offset_pose, "INVALID PLANNING" + status_debug_str);
  }
  context_->debug_pose_publisher->publish();
}

void PlanningValidatorNode::displayStatus()
{
  if (!context_->params.display_on_terminal) return;

  const auto warn = [this](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", msg.c_str());
    }
  };

  const auto & s = context_->validation_status;

  warn(s->is_valid_size, "planning trajectory size is invalid, too small.");
  warn(s->is_valid_curvature, "planning trajectory curvature is too large!!");
  warn(s->is_valid_finite_value, "planning trajectory has invalid value!!");
  warn(s->is_valid_interval, "planning trajectory interval is too long!!");
  warn(s->is_valid_lateral_acc, "planning trajectory lateral acceleration is too high!!");
  warn(s->is_valid_lateral_jerk, "planning trajectory lateral jerk is too high!!");
  warn(s->is_valid_longitudinal_max_acc, "planning trajectory acceleration is too high!!");
  warn(s->is_valid_longitudinal_min_acc, "planning trajectory deceleration is too high!!");
  warn(s->is_valid_relative_angle, "planning trajectory yaw angle varies too fast!!");
  warn(s->is_valid_steering, "planning trajectory expected steering angle is too high!!");
  warn(s->is_valid_steering_rate, "planning trajectory expected steering angle rate is too high!!");
  warn(s->is_valid_velocity_deviation, "planning trajectory velocity deviation is too high!!");
  warn(s->is_valid_distance_deviation, "planning trajectory is too far from ego!!");
  warn(
    s->is_valid_longitudinal_distance_deviation,
    "planning trajectory is too far from ego in longitudinal direction!!");
  warn(s->is_valid_forward_trajectory_length, "planning trajectory forward length is not enough!!");
  warn(s->is_valid_latency, "planning component latency is larger than threshold!!");
  warn(s->is_valid_yaw_deviation, "planning trajectory yaw difference from ego yaw is too large!!");
  warn(s->is_valid_trajectory_shift, "planning trajectory had sudden shift!!");
  warn(
    s->is_valid_intersection_collision_check,
    "planning trajectory leads to collision!! (intersection objects)");
  warn(s->is_valid_rear_collision_check, "planning trajectory leads to collision!! (rear objects)");
}

}  // namespace autoware::planning_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_validator::PlanningValidatorNode)
