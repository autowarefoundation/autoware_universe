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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_NODE_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_NODE_HPP_

#include "autoware/calibration_status_classifier/calibration_status_classifier.hpp"
#include "autoware/calibration_status_classifier/data_type.hpp"
#include "autoware/calibration_status_classifier/data_type_eigen.hpp"
#include "autoware/calibration_status_classifier/ros_utils.hpp"

#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <vector>

namespace autoware::calibration_status_classifier
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
class CalibrationStatusClassifierNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for CalibrationStatusClassifierNode
   * @param options node options including parameter declarations
   */
  explicit CalibrationStatusClassifierNode(const rclcpp::NodeOptions & options);

private:
  // Parameters
  RuntimeMode runtime_mode_;
  VelocitySource velocity_source_;
  double period_;
  int64_t queue_size_;
  bool check_velocity_;
  bool check_objects_;
  double velocity_threshold_;
  std::size_t objects_limit_;
  double miscalibration_confidence_threshold_;

  std::vector<CameraLidarTopicsInfo> camera_lidar_in_out_info_;
  std::vector<CameraLidarInfo> camera_lidar_info_;

  // ROS interface
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<std::unique_ptr<autoware_utils::DiagnosticsInterface>> diagnostics_interfaces_;

  // Velocity monitoring
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovariance>::SharedPtr twist_with_cov_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_cov_stamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;

  // Input synchronization
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>>
    cloud_subs_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> image_subs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> preview_image_pubs_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
  std::vector<std::shared_ptr<message_filters::Synchronizer<SyncPolicy>>> synchronizers_;

  std::vector<std::pair<
    sensor_msgs::msg::PointCloud2::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>>
    synchronized_data_;
  std::mutex synchronized_data_mutex_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Core library
  std::unique_ptr<CalibrationStatusClassifier> calibration_status_classifier_;

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
   * @brief Setup input synchronization for LiDAR and camera topics
   */
  void setup_input_synchronization();

  // Service callbacks
  /**
   * @brief Handle manual calibration validation requests
   * @param request Service request (empty trigger)
   * @param response Service response with validation result
   */
  void handle_calibration_request(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  // Periodic callback
  /**
   * @brief Process synchronized data periodically in PERIODIC mode
   */
  void periodic_callback();

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
   * @param time_ref Reference time for age calculation
   * @return VelocityFilterStatus containing velocity state and thresholds
   */
  FilterStatus<double> get_velocity_filter_status(const rclcpp::Time & time_ref);

  /**
   * @brief Get current object count check status for calibration prerequisites
   * @param time_ref Reference time for age calculation
   * @return ObjectsFilterStatus containing object count state and thresholds
   */
  FilterStatus<size_t> get_objects_filter_status(const rclcpp::Time & time_ref);

  /**
   * @brief Main execution for the node
   * @param pair_idx Index of the sensor pair to process
   * @return true if processing was successful, false otherwise
   */
  bool run(std::size_t pair_idx);

  /**
   * @brief Publish diagnostic status to ROS diagnostics system
   * @param input_metadata Input metadata including filter statuses and timestamps
   * @param pair_idx Index of the sensor pair being processed
   * @param result Calibration validation result (optional)
   */
  void publish_diagnostic_status(
    const InputMetadata & input_metadata, const size_t pair_idx,
    const CalibrationStatusClassifierResult & result = CalibrationStatusClassifierResult());
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CALIBRATION_STATUS_CLASSIFIER_NODE_HPP_
