// Copyright 2023- Autoware Foundation
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

#ifndef AUTOWARE__POSE_INSTABILITY_DETECTOR__POSE_INSTABILITY_DETECTOR_HPP_
#define AUTOWARE__POSE_INSTABILITY_DETECTOR__POSE_INSTABILITY_DETECTOR_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <optional>
#include <tuple>
#include <vector>

namespace autoware::pose_instability_detector
{

class PoseInstabilityDetector : public autoware::agnocast_wrapper::Node
{
  using Quaternion = geometry_msgs::msg::Quaternion;
  using Twist = geometry_msgs::msg::Twist;
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using KeyValue = diagnostic_msgs::msg::KeyValue;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

public:
  struct ThresholdValues
  {
    double position_x;
    double position_y;
    double position_z;
    double angle_x;
    double angle_y;
    double angle_z;
  };

  explicit PoseInstabilityDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ThresholdValues calculate_threshold(double interval_sec) const;
  static void dead_reckon(
    PoseStamped::SharedPtr & initial_pose, const rclcpp::Time & end_time,
    const std::deque<TwistWithCovarianceStamped> & twist_deque, Pose::SharedPtr & estimated_pose);

private:
  void callback_odometry(AUTOWARE_MESSAGE_CONST_SHARED_PTR(Odometry) odometry_msg_ptr);
  void callback_twist(AUTOWARE_MESSAGE_CONST_SHARED_PTR(TwistWithCovarianceStamped) twist_msg_ptr);
  void callback_timer();

  static std::deque<TwistWithCovarianceStamped> clip_out_necessary_twist(
    const std::deque<TwistWithCovarianceStamped> & twist_buffer, const rclcpp::Time & start_time,
    const rclcpp::Time & end_time);

  // subscribers and timer
  AUTOWARE_SUBSCRIPTION_PTR(Odometry) odometry_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(TwistWithCovarianceStamped) twist_sub_;
  AUTOWARE_TIMER_PTR timer_;

  // publisher
  AUTOWARE_PUBLISHER_PTR(PoseStamped) diff_pose_pub_;
  AUTOWARE_PUBLISHER_PTR(DiagnosticArray) diagnostics_pub_;

  // parameters
  const double timer_period_;  // [sec]

  const double heading_velocity_maximum_;                 // [m/s]
  const double heading_velocity_scale_factor_tolerance_;  // [%]

  const double angular_velocity_maximum_;                 // [rad/s]
  const double angular_velocity_scale_factor_tolerance_;  // [%]
  const double angular_velocity_bias_tolerance_;          // [rad/s]

  const double pose_estimator_longitudinal_tolerance_;  // [m]
  const double pose_estimator_lateral_tolerance_;       // [m]
  const double pose_estimator_vertical_tolerance_;      // [m]
  const double pose_estimator_angular_tolerance_;       // [rad]

  // variables
  std::optional<Odometry> latest_odometry_ = std::nullopt;
  std::optional<Odometry> prev_odometry_ = std::nullopt;
  std::deque<TwistWithCovarianceStamped> twist_buffer_;

  std::vector<bool> enable_validation_flags_;  // must be 6D for x, y, z, roll, pitch, yaw
};
}  // namespace autoware::pose_instability_detector

#endif  // AUTOWARE__POSE_INSTABILITY_DETECTOR__POSE_INSTABILITY_DETECTOR_HPP_
