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

#ifndef AUTOWARE__CALIBRATION_STATUS__CALIBRATION_STATUS_NODE_HPP_
#define AUTOWARE__CALIBRATION_STATUS__CALIBRATION_STATUS_NODE_HPP_

#include "autoware/calibration_status/calibration_status.hpp"
#include "autoware/calibration_status/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::calibration_status
{

/**
 * @brief A node for LiDAR-camera calibration status monitoring
 *
 * This node provides real-time monitoring of LiDAR-camera calibration quality using
 * deep learning inference. It supports multiple runtime modes:
 * - MANUAL: On-demand calibration check via service call
 * - PERIODIC: Regular calibration checks at specified intervals
 * - ACTIVE: Continuous calibration monitoring with sensor synchronization
 *
 * The node uses CUDA-accelerated preprocessing and TensorRT inference to detect
 * miscalibration between LiDAR and camera sensors by analyzing projected point
 * clouds overlaid on camera images.
 */
class CalibrationStatusNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for CalibrationStatusNode
   * @param options node options including parameter declarations
   */
  explicit CalibrationStatusNode(const rclcpp::NodeOptions & options);

private:
  // Parameters
  RuntimeMode runtime_mode_;
  VelocitySource velocity_source_;
  double period_;
  int64_t queue_size_;
  std::vector<std::string> cloud_topics_;
  std::vector<std::string> image_topics_;
  std::vector<double> approx_deltas_;
  bool check_velocity_;
  bool check_objects_;
  double velocity_threshold_;
  std::size_t objects_limit_;
  double miscalibration_confidence_threshold_;

  // ROS interface
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Velocity monitoring
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovariance>::SharedPtr twist_with_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_cov_stamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;

  // Camera info
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> camera_info_msgs_;

  // Sensor synchronization
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>>
    cloud_subs_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> image_subs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> preview_image_pubs_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
  std::vector<std::shared_ptr<message_filters::Synchronizer<SyncPolicy>>> synchronizers_;

  // Core library
  std::unique_ptr<CalibrationStatus> calibration_status_;

  // Current state
  double current_velocity_;
  rclcpp::Time last_velocity_update_;
  size_t current_objects_count_;
  rclcpp::Time last_objects_update_;

  // Methods
  /**
   * @brief Setup runtime mode-specific interfaces (service/timer/synchronization)
   */
  void setup_runtime_mode_interface();

  /**
   * @brief Setup velocity source subscriber based on configured source type
   */
  void setup_velocity_source_interface();

  /**
   * @brief Setup object detection subscriber for object count monitoring
   */
  void setup_object_detection_interface();

  /**
   * @brief Setup sensor synchronization for active runtime mode
   */
  void setup_sensor_synchronization();

  // Service callbacks
  /**
   * @brief Handle manual calibration validation requests
   * @param request Service request (empty trigger)
   * @param response Service response with validation result
   */
  void handle_calibration_request(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  // Timer callbacks
  /**
   * @brief Periodic calibration check callback for PERIODIC runtime mode
   */
  void periodic_calibration_check();

  // Velocity callbacks
  /**
   * @brief Process twist velocity messages
   * @param msg Twist message containing linear and angular velocities
   */
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Process stamped twist velocity messages
   * @param msg TwistStamped message with header and twist data
   */
  void twist_stamped_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief Process twist with covariance velocity messages
   * @param msg TwistWithCovariance message including uncertainty data
   */
  void twist_with_cov_callback(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg);

  /**
   * @brief Process stamped twist with covariance velocity messages
   * @param msg TwistWithCovarianceStamped message with header and covariance
   */
  void twist_with_cov_stamped_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief Process odometry messages for velocity extraction
   * @param msg Odometry message containing pose and twist data
   */
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Process detected objects messages for object count monitoring
   * @param msg PredictedObjects message containing a list of objects
   */
  void objects_callback(const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg);

  // Camera info callbacks
  /**
   * @brief Store camera info messages for calibration processing
   * @param msg Camera info message with intrinsic parameters
   * @param image_idx Index of the camera in the configuration
   */
  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg, size_t image_idx);

  // Sensor synchronization callback
  /**
   * @brief Process synchronized LiDAR and camera data for calibration validation
   * @param cloud_msg Point cloud message from LiDAR sensor
   * @param image_msg Image message from camera sensor
   * @param pair_idx Index of the sensor pair being processed
   */
  void synchronized_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, size_t pair_idx);

  // Utility methods
  /**
   * @brief Update current vehicle velocity from sensor data
   * @param velocity Linear velocity in m/s
   */
  void update_vehicle_velocity(double velocity);

  /**
   * @brief Get current velocity check status for calibration prerequisites
   * @return VelocityCheckStatus containing velocity state and thresholds
   */
  CheckStatus<double> get_velocity_check_status();

  /**
   * @brief Get current object count check status for calibration prerequisites
   * @return ObjectsCheckStatus containing object count state and thresholds
   */
  CheckStatus<size_t> get_objects_check_status();

  /**
   * @brief Publish diagnostic status to ROS diagnostics system
   * @param velocity_check_status Current velocity check state
   * @param objects_check_status Current object count check state
   * @param pair_idx Index of the sensor pair being processed
   * @param result Calibration validation result (optional)
   */
  void publish_diagnostic_status(
    const CheckStatus<double> & velocity_check_status,
    const CheckStatus<size_t> & objects_check_status, const size_t pair_idx,
    const CalibrationStatusResult & result = CalibrationStatusResult());
};

}  // namespace autoware::calibration_status

#endif  // AUTOWARE__CALIBRATION_STATUS__CALIBRATION_STATUS_NODE_HPP_
