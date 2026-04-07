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

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR_NODE_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR_NODE_HPP_

#include "traffic_light_map_based_detector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
class MapBasedDetector : public rclcpp::Node
{
public:
  explicit MapBasedDetector(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr expect_roi_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  struct TransformSamplingConfig
  {
    double min_timestamp_offset;
    double max_timestamp_offset;
    double timestamp_sample_len;
  };

  std::unique_ptr<TrafficLightMapBasedDetector> detector_;
  TransformSamplingConfig transform_sampling_config_;

  bool getTransform(
    const rclcpp::Time & t, const std::string & frame_id, tf2::Transform & tf) const;
  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg);
  void routeCallback(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr input_msg);
  void logMessages(const std::vector<LogMessage> & logs);
};
}  // namespace autoware::traffic_light
#endif  // TRAFFIC_LIGHT_MAP_BASED_DETECTOR_NODE_HPP_
