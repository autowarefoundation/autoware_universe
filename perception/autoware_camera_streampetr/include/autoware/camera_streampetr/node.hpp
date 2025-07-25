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

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

class StreamPetrNode : public rclcpp::Node
{
  using Odometry = nav_msgs::msg::Odometry;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
  using DetectedObject = autoware_perception_msgs::msg::DetectedObject;

public:
  explicit StreamPetrNode(const rclcpp::NodeOptions & node_options);

private:
  void odometry_callback(Odometry::ConstSharedPtr input_msg);
  void camera_info_callback(CameraInfo::ConstSharedPtr input_camera_info_msg, const int camera_id);
  void camera_image_callback(Image::ConstSharedPtr input_camera_image_msg, const int camera_id);

  void step(const rclcpp::Time & stamp);
  std::optional<std::pair<std::vector<float>, std::vector<float>>> get_ego_pose_vector() const;
  std::optional<std::vector<float>> get_camera_extrinsics_vector(
    const std::vector<std::string> & camera_links);

  rclcpp::Subscription<Odometry>::SharedPtr localization_sub_;
  std::vector<rclcpp::Subscription<CameraInfo>::SharedPtr> camera_info_subs_;

  const bool multithreading_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> camera_callback_groups_;

  std::vector<image_transport::Subscriber> camera_image_subs_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  const size_t rois_number_;

  Odometry::ConstSharedPtr initial_kinematic_state_ = nullptr;
  Odometry::ConstSharedPtr latest_kinematic_state_ = nullptr;
  std::unique_ptr<CameraDataStore> data_store_;
  const float max_camera_time_diff_;
  const int anchor_camera_id_;
  std::unique_ptr<StreamPetrNetwork> network_;

  // debugger
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  const bool debug_mode_;
};

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__NODE_HPP_
