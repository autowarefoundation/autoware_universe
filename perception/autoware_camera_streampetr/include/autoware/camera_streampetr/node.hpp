// Copyright 2025 TIER IV
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

#ifndef AUTOWARE__CAMERA_STREAMPETR__NODE_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__NODE_HPP_

#include "autoware/camera_streampetr/network/camera_data_store.hpp"
#include "autoware/camera_streampetr/network/network.hpp"

#include <Eigen/Dense>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <NvInferRuntime.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
namespace autoware::camera_streampetr
{

class StreamPetrNode : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
  using DetectedObject = autoware_perception_msgs::msg::DetectedObject;

public:
  explicit StreamPetrNode(const rclcpp::NodeOptions & node_options);

private:
  void camera_info_callback(CameraInfo::ConstSharedPtr input_camera_info_msg, const int camera_id);
  void camera_image_callback(Image::ConstSharedPtr input_camera_image_msg, const int camera_id);

  void step(const rclcpp::Time & stamp);

  // Helper methods for step function
  bool validate_camera_sync();
  void reset_system_state();
  bool prepare_inference_data(const rclcpp::Time & stamp);
  void cleanup_on_failure();
  std::optional<std::tuple<
    std::vector<autoware_perception_msgs::msg::DetectedObject>, std::vector<float>, double>>
  perform_inference();
  InferenceInputs create_inference_inputs();
  void publish_detection_results(
    const rclcpp::Time & stamp,
    const std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects);
  void publish_debug_metrics(const std::vector<float> & forward_time_ms, double inference_time_ms);

  std::optional<std::pair<std::vector<float>, std::vector<float>>> get_ego_pose_vector(
    const rclcpp::Time & stamp);
  std::optional<std::vector<float>> get_camera_extrinsics_vector();

  // Helper methods for camera extrinsics computation
  std::optional<Eigen::Matrix4f> compute_camera_transform(
    size_t camera_index, const std::vector<std::string> & camera_links,
    const std::vector<float> & intrinsics_all);
  Eigen::Matrix4f extract_intrinsic_matrix(
    size_t camera_index, const std::vector<float> & intrinsics_all);
  std::optional<geometry_msgs::msg::TransformStamped> get_camera_transform(
    const std::string & camera_link);
  Eigen::Matrix4f create_lidar_to_camera_transform(
    const geometry_msgs::msg::TransformStamped & transform_stamped);
  Eigen::Matrix3f extract_rotation_matrix(
    const geometry_msgs::msg::TransformStamped & transform_stamped);
  Eigen::Vector3f extract_translation_vector(
    const geometry_msgs::msg::TransformStamped & transform_stamped);
  void append_transform_to_result(
    const Eigen::Matrix4f & transform_matrix, std::vector<float> & result);

  const std::string logger_name_;
  std::vector<rclcpp::Subscription<CameraInfo>::SharedPtr> camera_info_subs_;

  const bool multithreading_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> camera_callback_groups_;

  std::vector<image_transport::Subscriber> camera_image_subs_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  const size_t rois_number_;

  geometry_msgs::msg::TransformStamped initial_transform_;
  bool initial_transform_set_ = false;
  std::unique_ptr<CameraDataStore> data_store_;
  const float max_camera_time_diff_;
  const int anchor_camera_id_;
  std::unique_ptr<StreamPetrNetwork> network_;

  // State variables for refactored step method
  std::pair<std::vector<float>, std::vector<float>> current_ego_pose_;
  std::vector<float> current_extrinsics_;
  float current_prediction_timestamp_;

  // debugger
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  const bool debug_mode_;
};

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__NODE_HPP_
