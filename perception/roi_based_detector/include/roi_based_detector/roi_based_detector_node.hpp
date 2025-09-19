// Copyright 2024 TIER IV, Inc.
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

#ifndef ROI_BASED_DETECTOR__ROI_BASED_DETECTOR_NODE_HPP_
#define ROI_BASED_DETECTOR__ROI_BASED_DETECTOR_NODE_HPP_

#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/semantic.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <memory>
#include <string>
#include <vector>

namespace roi_based_detector
{
using autoware::universe_utils::TransformListener;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using Label = autoware_perception_msgs::msg::ObjectClassification;

class RoiBasedDetectorNode : public rclcpp::Node
{
public:
  explicit RoiBasedDetectorNode(const rclcpp::NodeOptions & node_options);

private:
  struct IgnoreLabel
  {
    bool UNKNOWN;
    bool CAR;
    bool TRUCK;
    bool BUS;
    bool TRAILER;
    bool MOTORCYCLE;
    bool BICYCLE;
    bool PEDESTRIAN;
    bool isIgnore(const uint8_t label) const
    {
      return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
             (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
             (label == Label::TRAILER && TRAILER) || (label == Label::MOTORCYCLE && MOTORCYCLE) ||
             (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
    }
  };  // struct IgnoreLabel
  IgnoreLabel ignore_class_;
  void roiCallback(const DetectedObjectsWithFeature::ConstSharedPtr & msg);
  void cameraInfoCallback(const CameraInfo::ConstSharedPtr & msg);
  Eigen::Matrix4d transformToHomogeneous(const geometry_msgs::msg::Transform & transform);
  void pixelTo3DPoint(
    const Eigen::Vector2f & pixel, const Eigen::Matrix4f & transform, Eigen::Vector4f & point);

  rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr rois_pub_;
  rclcpp::Publisher<DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr roi_sub_;
  // camera_info sub
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
  CameraInfo camera_info_;
  Eigen::Matrix4f inv_projection_;
  bool is_inv_projection_initialized_{false};
  Eigen::Matrix4f camera2lidar_mul_inv_projection_;
  bool is_camera2lidar_mul_inv_projection_initialized_{false};
  std::shared_ptr<TransformListener> transform_listener_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;

  std::string target_frame_;
};
}  // namespace roi_based_detector

#endif  // ROI_BASED_DETECTOR__ROI_BASED_DETECTOR_NODE_HPP_
