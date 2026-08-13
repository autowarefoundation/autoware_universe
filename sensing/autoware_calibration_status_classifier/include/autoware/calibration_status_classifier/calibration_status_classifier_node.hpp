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
#include "autoware/calibration_status_classifier/calibration_status_classifier_filters.hpp"
#include "autoware/calibration_status_classifier/data_type.hpp"
#include "autoware/calibration_status_classifier/data_type_eigen.hpp"
#include "autoware/calibration_status_classifier/ros_utils.hpp"

#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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

struct InputMetadata
{
  FiltersResult filters_result;
  rclcpp::Time cloud_stamp;
  rclcpp::Time image_stamp;
  rclcpp::Time common_stamp;
};

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
  double period_;
  int64_t queue_size_;

  // Prerequisite filters
  CalibrationStatusClassifierFilters filters_;

  std::vector<CameraLidarTopicsInfo> camera_lidar_in_out_info_;
  std::vector<CameraLidarInfo> camera_lidar_info_;

  // ROS interface
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<std::unique_ptr<autoware_utils::DiagnosticsInterface>> diagnostics_interfaces_;

  // Linear velocity monitoring
  rclcpp::SubscriptionBase::SharedPtr linear_velocity_sub_;

  // Angular velocity monitoring
  rclcpp::SubscriptionBase::SharedPtr angular_velocity_sub_;

  // Object detection monitoring
  rclcpp::SubscriptionBase::SharedPtr objects_sub_;

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

  // Methods
  /**
   * @brief Setup runtime mode-specific interfaces (service/timer/synchronization)
   */
  void setup_runtime_mode_interface();

  /**
   * @brief Setup linear velocity source subscriber
   * @param source Linear velocity source type determining which message type to subscribe to
   */
  void setup_linear_velocity_source_interface(LinearVelocitySource source);

  /**
   * @brief Setup angular velocity source subscriber
   * @param source Angular velocity source type determining which message type to subscribe to
   */
  void setup_angular_velocity_source_interface(AngularVelocitySource source);

  /**
   * @brief Setup object detection subscriber based on configured source type
   * @param source Object message source type
   */
  void setup_object_detection_interface(ObjectsSource source);

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
