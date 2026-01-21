// Copyright 2025 Instituto de Telecomunições-Porto Branch, Inc. All rights reserved.
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

#ifndef AUTOWARE__SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_NODE_HPP_
#define AUTOWARE__SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_NODE_HPP_

#include "autoware/spheric_collision_detector/spheric_collision_detector.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <autoware_utils/ros/self_pose_listener.hpp>
#include <autoware_utils/ros/transform_listener.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

namespace spheric_collision_detector
{

struct NodeParam
{
  double update_rate;
};

class SphericCollisionDetectorNode : public rclcpp::Node
{
public:
  explicit SphericCollisionDetectorNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber
  std::shared_ptr<autoware_utils::SelfPoseListener> self_pose_listener_;
  std::shared_ptr<autoware_utils::TransformListener> transform_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacle_pointcloud_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    sub_object_recognition_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_reference_trajectory_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_predicted_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // Data Buffer
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist_;
  // sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud_;
  autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr object_recognition_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory_;
  geometry_msgs::msg::TransformStamped object_recognition_transform_;

  // Callback
  // void onObstaclePointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onPredictedTrajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onObjectRecognition(const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg);

  // Publisher
  std::shared_ptr<autoware_utils::DebugPublisher> debug_publisher_;
  std::shared_ptr<autoware_utils::ProcessingTimePublisher> time_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  void initTimer(double period_s);

  bool isDataReady();
  bool isDataTimeout();
  void onTimer();

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Dynamic Reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // Core
  Input input_;
  Output output_;
  std::unique_ptr<SphericCollisionDetector> spheric_collision_detector_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void checkCollisionStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Visualization
  visualization_msgs::msg::MarkerArray createMarkerArray() const;

  void addResampledTrajectoryMarkers(visualization_msgs::msg::MarkerArray & marker_array) const;
  void addVehiclePassingAreaMarkers(visualization_msgs::msg::MarkerArray & marker_array) const;
  void addObstacleMarkers(visualization_msgs::msg::MarkerArray & marker_array) const;
};
}  // namespace spheric_collision_detector

#endif  // AUTOWARE__SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_NODE_HPP_
