// Copyright 2024 The Autoware Foundation
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
#ifndef POSE_COVARIANCE_MODIFIER_HPP_
#define POSE_COVARIANCE_MODIFIER_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace autoware::pose_covariance_modifier
{
class PoseCovarianceModifierNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit PoseCovarianceModifierNode(const rclcpp::NodeOptions & node_options);

  enum class PoseSource {
    GNSS = 0,
    GNSS_NDT = 1,
    NDT = 2,
  };

private:
  // covariance matrix indexes
  const int X_POS_IDX_ = 0;
  const int Y_POS_IDX_ = 7;
  const int Z_POS_IDX_ = 14;
  const int YAW_POS_IDX_ = 35;

  // parameters
  double threshold_gnss_stddev_yaw_deg_max_;
  double threshold_gnss_stddev_z_max_;
  double threshold_gnss_stddev_xy_bound_lower_;
  double threshold_gnss_stddev_xy_bound_upper_;
  double ndt_std_dev_bound_lower_;
  double ndt_std_dev_bound_upper_;
  double gnss_pose_timeout_sec_;
  bool debug_mode_;

  rclcpp::Time gnss_pose_received_time_last_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  gnss_pose_with_cov_last_;
  PoseSource pose_source_;

  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  sub_gnss_pose_with_cov_;
  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  sub_ndt_pose_with_cov_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::PoseWithCovarianceStamped)
  pub_pose_with_covariance_stamped_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::String) pub_str_pose_source_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::Float64) pub_double_ndt_position_stddev_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::Float64) pub_double_gnss_position_stddev_;

  void callback_gnss_pose_with_cov(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    msg_pose_with_cov_in);

  void callback_ndt_pose_with_cov(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::PoseWithCovarianceStamped) &
    msg_pose_with_cov_in);

  bool gnss_pose_has_timed_out(const rclcpp::Time & gnss_pose_received_time_last);

  PoseSource pose_source_from_gnss_stddev(
    double gnss_pose_yaw_stddev_deg, double gnss_pose_stddev_z, double gnss_pose_stddev_xy) const;

  std::array<double, 36> update_ndt_covariances_from_gnss(
    const std::array<double, 36> & ndt_covariance_in);

  void publish_pose_type(const PoseSource & pose_source);
};

}  // namespace autoware::pose_covariance_modifier

#endif  // POSE_COVARIANCE_MODIFIER_HPP_
