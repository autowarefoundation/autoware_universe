// Copyright 2022 Tier IV, Inc.
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

#include "autoware_steer_offset_estimator/steer_offset_estimator_node.hpp"

#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace autoware::steer_offset_estimator
{
SteerOffsetEstimatorNode::SteerOffsetEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node("steer_offset_estimator", node_options)
{
  using std::placeholders::_1;
  // get parameter
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;
  covariance_ = this->declare_parameter<double>("initial_covariance");
  auto initial_steer_offset_param_name =
    this->declare_parameter<std::string>("initial_steer_offset_param_name");
  initial_steer_offset_ = this->declare_parameter<double>(initial_steer_offset_param_name);

  forgetting_factor_ = this->declare_parameter<double>("forgetting_factor");
  update_hz_ = this->declare_parameter<double>("steer_update_hz");
  valid_min_velocity_ = this->declare_parameter<double>("valid_min_velocity");
  valid_max_steer_ = this->declare_parameter<double>("valid_max_steer");
  warn_steer_offset_abs_error_ =
    this->declare_parameter<double>("warn_steer_offset_deg") * M_PI / 180.0;

  // publisher
  pub_steer_offset_ = this->create_publisher<Float32Stamped>("~/output/steering_offset", 1);
  pub_steer_offset_cov_ =
    this->create_publisher<Float32Stamped>("~/output/steering_offset_covariance", 1);
  pub_steer_offset_error_ =
    this->create_publisher<Float32Stamped>("~/output/steering_offset_error", 1);

  // subscriber
  sub_pose_ = this->create_subscription<PoseStamped>(
    "~/input/pose", 1, std::bind(&SteerOffsetEstimatorNode::onPose, this, _1));
  sub_steer_ = this->create_subscription<Steering>(
    "~/input/steer", 1, std::bind(&SteerOffsetEstimatorNode::onSteer, this, _1));

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<Updater>(this, 1.0 / update_hz_);
  updater_ptr_->setHardwareID("steer_offset_estimator");
  updater_ptr_->add("steer_offset_estimator", this, &SteerOffsetEstimatorNode::monitorSteerOffset);

  // timer
  {
    auto on_timer = std::bind(&SteerOffsetEstimatorNode::onTimer, this);
    const auto period = rclcpp::Rate(update_hz_).period();
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
      this->get_clock(), period, std::move(on_timer),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
}

// function for diagnostics
void SteerOffsetEstimatorNode::monitorSteerOffset(DiagnosticStatusWrapper & stat)  // NOLINT
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  const double eps = 1e-3;
  if (covariance_ > eps) {
    stat.summary(DiagStatus::OK, "Preparation");
    return;
  }
  if (std::abs(estimated_steer_offset_) > warn_steer_offset_abs_error_) {
    stat.summary(DiagStatus::WARN, "Steer offset is larger than tolerance");
    return;
  }
  stat.summary(DiagStatus::OK, "Calibration OK");
}

void SteerOffsetEstimatorNode::onPose(const PoseStamped::ConstSharedPtr msg)
{
  current_pose_ptr_ = msg;
}

void SteerOffsetEstimatorNode::onSteer(const Steering::ConstSharedPtr msg)
{
  steer_ptr_ = msg;
}

bool SteerOffsetEstimatorNode::updateSteeringOffset()
{
  // RLS; recursive least-squares algorithm

  if (!current_pose_ptr_ || !steer_ptr_) {
    // null input
    return false;
  }

  if (!prev_pose_ptr_) {
    prev_pose_ptr_ = current_pose_ptr_;
    return false;
  }

  // Calculate twist from pose difference
  TwistStamped twist = calcTwistFromPose(prev_pose_ptr_, current_pose_ptr_);
  prev_pose_ptr_ = current_pose_ptr_;

  const double vel = twist.twist.linear.x;
  const double wz = twist.twist.angular.z;
  const double steer = steer_ptr_->steering_tire_angle;

  if (std::abs(vel) < valid_min_velocity_ || std::abs(steer) > valid_max_steer_) {
    // invalid velocity/steer value for estimating steering offset
    return false;
  }

  // use following approximation: tan(a+b) = tan(a) + tan(b) = a + b

  const double phi = vel / wheel_base_;
  covariance_ = (covariance_ - (covariance_ * phi * phi * covariance_) /
                                 (forgetting_factor_ + phi * covariance_ * phi)) /
                forgetting_factor_;

  const double coef = (covariance_ * phi) / (forgetting_factor_ + phi * covariance_ * phi);
  const double measured_wz_offset = wz - phi * steer;
  const double error_wz_offset = measured_wz_offset - phi * estimated_steer_offset_;

  estimated_steer_offset_ = estimated_steer_offset_ + coef * error_wz_offset;

  return true;
}

tf2::Quaternion SteerOffsetEstimatorNode::getQuaternion(
  const PoseStamped::ConstSharedPtr & pose_stamped_ptr)
{
  const auto & orientation = pose_stamped_ptr->pose.orientation;
  return tf2::Quaternion{orientation.x, orientation.y, orientation.z, orientation.w};
}

geometry_msgs::msg::Vector3 SteerOffsetEstimatorNode::computeRelativeRotationVector(
  const tf2::Quaternion & q1, const tf2::Quaternion & q2)
{
  const tf2::Quaternion diff_quaternion = q1.inverse() * q2;
  const tf2::Vector3 axis = diff_quaternion.getAxis() * diff_quaternion.getAngle();
  return geometry_msgs::msg::Vector3{}.set__x(axis.x()).set__y(axis.y()).set__z(axis.z());
}

TwistStamped SteerOffsetEstimatorNode::calcTwistFromPose(
  const PoseStamped::ConstSharedPtr pose_a, const PoseStamped::ConstSharedPtr pose_b)
{
  const double dt =
    (rclcpp::Time(pose_b->header.stamp) - rclcpp::Time(pose_a->header.stamp)).seconds();

  TwistStamped twist;
  twist.header = pose_b->header;
  twist.header.frame_id = pose_b->header.frame_id;

  if (std::abs(dt) < std::numeric_limits<double>::epsilon()) {
    return twist;
  }

  const auto pose_a_quaternion = getQuaternion(pose_a);
  const auto pose_b_quaternion = getQuaternion(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  const geometry_msgs::msg::Vector3 relative_rotation_vector =
    computeRelativeRotationVector(pose_a_quaternion, pose_b_quaternion);

  diff_xyz.x = pose_b->pose.position.x - pose_a->pose.position.x;
  diff_xyz.y = pose_b->pose.position.y - pose_a->pose.position.y;
  diff_xyz.z = pose_b->pose.position.z - pose_a->pose.position.z;

  twist.twist.linear.x =
    std::sqrt(std::pow(diff_xyz.x, 2.0) + std::pow(diff_xyz.y, 2.0) + std::pow(diff_xyz.z, 2.0)) /
    dt;
  twist.twist.linear.y = 0;
  twist.twist.linear.z = 0;
  twist.twist.angular.x = relative_rotation_vector.x / dt;
  twist.twist.angular.y = relative_rotation_vector.y / dt;
  twist.twist.angular.z = relative_rotation_vector.z / dt;

  return twist;
}

void SteerOffsetEstimatorNode::onTimer()
{
  if (updateSteeringOffset()) {
    auto msg = std::make_unique<Float32Stamped>();
    msg->stamp = this->now();
    msg->data = static_cast<float>(estimated_steer_offset_);
    pub_steer_offset_->publish(std::move(msg));

    auto cov_msg = std::make_unique<Float32Stamped>();
    cov_msg->stamp = this->now();
    cov_msg->data = static_cast<float>(covariance_);
    pub_steer_offset_cov_->publish(std::move(cov_msg));

    auto error_msg = std::make_unique<Float32Stamped>();
    error_msg->stamp = this->now();
    error_msg->data = static_cast<float>(estimated_steer_offset_ - initial_steer_offset_);
    pub_steer_offset_error_->publish(std::move(error_msg));
  }
}
}  // namespace autoware::steer_offset_estimator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::steer_offset_estimator::SteerOffsetEstimatorNode)
