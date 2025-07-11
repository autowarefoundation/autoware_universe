// Copyright 2023 TIER IV, Inc.
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
#ifndef GYRO_BIAS_ESTIMATOR_HPP_
#define GYRO_BIAS_ESTIMATOR_HPP_

#include "gyro_bias_estimation_module.hpp"

#include <Eigen/Dense>
#include <autoware_utils/ros/transform_listener.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::imu_corrector
{
class GyroBiasEstimator : public rclcpp::Node
{
private:
  using Imu = sensor_msgs::msg::Imu;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
  using Vector3 = geometry_msgs::msg::Vector3;
  using Odometry = nav_msgs::msg::Odometry;
  using Vector2d = Eigen::Vector2d;
  using Matrix2d = Eigen::Matrix2d;

public:
  explicit GyroBiasEstimator(const rclcpp::NodeOptions & options);

private:
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void callback_imu(const Imu::ConstSharedPtr imu_msg_ptr);
  void callback_odom(const Odometry::ConstSharedPtr odom_msg_ptr);
  void callback_pose_msg(const PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr);
  void estimate_scale_gyro(const PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr);
  void timer_callback();
  void validate_gyro_bias();

  static geometry_msgs::msg::Vector3 transform_vector3(
    const geometry_msgs::msg::Vector3 & vec,
    const geometry_msgs::msg::TransformStamped & transform);

  const std::string output_frame_ = "base_link";

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<Vector3Stamped>::SharedPtr gyro_bias_pub_;
  rclcpp::Publisher<Vector3Stamped>::SharedPtr gyro_scale_pub_;
  rclcpp::Publisher<Vector3Stamped>::SharedPtr gyro_debug_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_scaled_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_check_scale_;
  rclcpp::Time last_time_rx_pose_;
  rclcpp::Time last_time_rx_imu_;

  std::unique_ptr<GyroBiasEstimationModule> gyro_bias_estimation_module_;

  const double gyro_bias_threshold_;
  const double angular_velocity_offset_x_;
  const double angular_velocity_offset_y_;
  const double angular_velocity_offset_z_;
  const double timer_callback_interval_sec_;
  const double diagnostics_updater_interval_sec_;
  const double straight_motion_ang_vel_upper_limit_;

  const double estimate_scale_init_;
  const double ekf_variance_p_;
  const double ekf_process_noise_q_;
  const double ekf_measurement_noise_r_;
  const double ekf_variance_p_a_;
  const double ekf_process_noise_q_a_;
  const double ekf_measurement_noise_r_a_;
  const double time_window_secs_;
  const double threshold_scale_change_;
  const double threshold_error_rate_;
  const double num_consecutive_scale_change_;
  const double min_allowed_scale_;
  const double max_allowed_scale_;
  const double scale_on_purpose_;
  const double bias_on_purpose_;
  const double drift_scale_;
  const double drift_bias_;

  const double alpha_;  // comp filter
  const double alpha_ndt_rate_;
  const double threshlod_to_estimate_scale_;
  const double percentage_scale_rate_allow_correct_;
  const double counter_correct_big_change_;
  const double alpha_big_change_;

  double filtered_scale_angle_;
  double filtered_scale_rate_;
  double big_change_scale_rate_;

  int window_scale_change_;
  double previous_yaw_angle_;

  double ndt_yaw_rate_;
  double gyro_yaw_rate_;
  double previous_scale_;
  double previous_gyro_rate_;
  double previous_ndt_rate_;
  double accel_yaw_gyro_;
  double accel_yaw_ndt_;

  double gyro_yaw_angle_;
  double ndt_yaw_angle_;
  double gyro_corrected_yaw_angle_;

  double big_change_detect_;

  bool has_gyro_yaw_angle_init_;
  bool gyro_angle_restarted_;

  double final_bias_on_purpose_;
  double final_scale_on_purpose_;
  double bias_final_;
  double scale_final_;

  // EKF variables
  double estimated_scale_;
  double p_;
  double q_;
  double r_;

  diagnostic_updater::Updater updater_;

  std::optional<Vector3> gyro_bias_;

  std::shared_ptr<autoware_utils::TransformListener> transform_listener_;

  std::string imu_frame_;

  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_all_;
  std::vector<geometry_msgs::msg::PoseStamped> pose_buf_;
  std::vector<double> scale_list_all_;
  std::vector<double> scale_out_range_;

  struct DiagnosticsInfo
  {
    unsigned char level;
    std::string summary_message;
    double gyro_bias_x_for_imu_corrector;
    double gyro_bias_y_for_imu_corrector;
    double gyro_bias_z_for_imu_corrector;
    double estimated_gyro_bias_x;
    double estimated_gyro_bias_y;
    double estimated_gyro_bias_z;
    double estimated_gyro_scale_x;
    double estimated_gyro_scale_y;
    double estimated_gyro_scale_z;
  };

  struct GyroInfo
  {
    unsigned char bias_status;
    std::string bias_status_summary;
    std::string bias_summary_message;
    unsigned char scale_status;
    std::string scale_status_summary;
    std::string scale_summary_message;
  };

  DiagnosticsInfo diagnostics_info_;
  GyroInfo gyro_info_;
};
}  // namespace autoware::imu_corrector

#endif  // GYRO_BIAS_ESTIMATOR_HPP_
