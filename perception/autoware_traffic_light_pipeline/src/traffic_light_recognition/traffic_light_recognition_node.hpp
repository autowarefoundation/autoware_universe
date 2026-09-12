// Copyright 2026 TIER IV, Inc.
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

#ifndef TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_NODE_HPP_
#define TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_NODE_HPP_

#include "traffic_light_recognition.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace autoware::traffic_light
{

class TrafficLightRecognitionNode : public rclcpp::Node
{
public:
  explicit TrafficLightRecognitionNode(const rclcpp::NodeOptions & node_options);

private:
  void vector_map_callback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void route_callback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void sync_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  void build_engines_and_shutdown();

  TrafficLightRecognitionConfig config_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<TrafficLightRecognition> recognition_;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr vector_map_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;

  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightArray>::SharedPtr signals_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr rois_pub_;

  using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::unique_ptr<Sync> sync_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_NODE_HPP_
