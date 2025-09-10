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
#ifndef AUTOWARE__IMU_CORRECTOR__GYRO_BIAS_ESTIMATOR_HPP_
#define AUTOWARE__IMU_CORRECTOR__GYRO_BIAS_ESTIMATOR_HPP_

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
public:
  explicit GyroBiasEstimator(const rclcpp::NodeOptions & options);
  // Modified IMU signal to inject bias and scale
  struct ScaleImuSignal
  {
    bool modify_imu_scale_;
    double bias_final_;
    double scale_final_;
    double drift_scale_;
    double drift_bias_;
  };

  // EKF variables
  struct EKFEstimateScaleRateVars
  {
    double estimated_scale_rate_;
    double p_;
    double q_;
    double r_;
    double max_variance_;
    double min_covariance_;
    double variance_p_after_;
    double process_noise_q_after_;
    double measurement_noise_;
    double measurement_noise_r_after_;
    double filtered_scale_rate_;
    size_t n_big_changes_detected_;
    int samples_to_init_;
    bool filtered_scale_initialized_;
  };

  struct EKFEstimateScaleAngleVars
  {
    Eigen::Vector2d x_state_;  // [angle (radians), scale]
    Eigen::Matrix2d p_angle_;
    Eigen::Matrix2d q_angle_;
    Eigen::Matrix<double, 1, 1> r_angle_;
    double max_variance_p_angle_;
    double min_covariance_angle_;
    double filtered_scale_angle_;
    double estimated_scale_angle_;
    double decay_coefficient_;
    bool has_gyro_yaw_angle_init_;
  };

  sensor_msgs::msg::Imu modify_imu(
    sensor_msgs::msg::Imu & imu_msg, ScaleImuSignal & scale_imu, rclcpp::Time & time);
  bool should_skip_update(double gyro_yaw_rate);
  double extract_yaw_from_pose(
    const geometry_msgs::msg::Quaternion & quat_msg, tf2::Quaternion & quat_out);
  void update_angle_ekf(double yaw_ndt, EKFEstimateScaleAngleVars & ekf_angle) const;
  friend class GyroBiasEstimatorTest;

private:
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr);
  void callback_pose_msg(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr);
  void estimate_scale_gyro(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr);
  void timer_callback();
  void validate_gyro_bias();
  double compute_yaw_rate_from_quat(
    const tf2::Quaternion & quat, const tf2::Quaternion & prev_quat, double dt);

  static geometry_msgs::msg::Vector3 transform_vector3(
    const geometry_msgs::msg::Vector3 & vec,
    const geometry_msgs::msg::TransformStamped & transform);

  const std::string output_frame_ = "base_link";

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_bias_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyro_scale_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_scaled_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_check_scale_;
  rclcpp::Time last_time_rx_pose_;
  rclcpp::Time last_time_rx_imu_;

  tf2::Quaternion previous_quat_ndt_;
  tf2::Quaternion rel_quat_ndt_;

  std::unique_ptr<GyroBiasEstimationModule> gyro_bias_estimation_module_;

  const double gyro_bias_threshold_;
  const double angular_velocity_offset_x_;
  const double angular_velocity_offset_y_;
  const double angular_velocity_offset_z_;
  const double timer_callback_interval_sec_;
  const double diagnostics_updater_interval_sec_;
  const double straight_motion_ang_vel_upper_limit_;

  const double min_allowed_scale_;
  const double max_allowed_scale_;
  const double alpha_;
  const double alpha_ndt_rate_;
  const double threshold_to_estimate_scale_;
  const double percentage_scale_rate_allow_correct_;
  const double alpha_gyro_;
  const int delay_gyro_ms_;
  const int buffer_size_gyro_;
  const int samples_filter_pose_rate_;
  const int samples_filter_gyro_rate_;

  double ndt_yaw_rate_;
  double gyro_yaw_rate_;

  double gyro_yaw_angle_;  // radians
  double ndt_yaw_angle_;   // radians

  double avg_rate_pose_;
  double avg_rate_gyro_;

  diagnostic_updater::Updater updater_;

  std::optional<geometry_msgs::msg::Vector3> gyro_bias_;
  std::optional<geometry_msgs::msg::Vector3> gyro_bias_not_rotated_;

  std::shared_ptr<autoware_utils::TransformListener> transform_listener_;

  std::string imu_frame_;

  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_all_;
  std::vector<geometry_msgs::msg::PoseStamped> pose_buf_;
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_scale_buf_;
  std::vector<double> scale_out_range_;
  std::vector<double> estimated_scale_buff_;
  std::vector<double> delta_gyro_buff_;
  std::vector<double> delta_lidar_buff_;
  std::vector<double> rate_pose_buff_;
  std::vector<double> rate_gyro_buff_;

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

  void update_rate_ekf(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg,
    EKFEstimateScaleRateVars & ekf_rate_state);

  DiagnosticsInfo diagnostics_info_;
  GyroInfo gyro_info_;
  EKFEstimateScaleRateVars ekf_rate_;
  EKFEstimateScaleAngleVars ekf_angle_;
  ScaleImuSignal scale_imu_;
};
}  // namespace autoware::imu_corrector

#endif  // AUTOWARE__IMU_CORRECTOR__GYRO_BIAS_ESTIMATOR_HPP_
