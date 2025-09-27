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

#ifndef ROI_BASED_DETECTOR_NODE_HPP_
#define ROI_BASED_DETECTOR_NODE_HPP_

#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <opencv2/opencv.hpp>
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
#include <unordered_map>
#include <vector>

namespace autoware::roi_based_detector
{
using autoware::universe_utils::TransformListener;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using Label = autoware_perception_msgs::msg::ObjectClassification;

class RoiBasedDetectorNode : public rclcpp::Node
{
public:
  explicit RoiBasedDetectorNode(const rclcpp::NodeOptions & node_options);

private:
  struct LabelSettings
  {
    bool UNKNOWN;
    bool CAR;
    bool TRUCK;
    bool BUS;
    bool TRAILER;
    bool MOTORCYCLE;
    bool BICYCLE;
    bool PEDESTRIAN;

    bool isIgnoreLabel(const uint8_t label) const
    {
      return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
             (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
             (label == Label::TRAILER && TRAILER) || (label == Label::MOTORCYCLE && MOTORCYCLE) ||
             (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
    }

    static uint8_t getLabelShape(const uint8_t label)
    {
      if (
        label == Label::CAR || label == Label::TRUCK || label == Label::BUS ||
        label == Label::TRAILER || label == Label::MOTORCYCLE || label == Label::BICYCLE) {
        return autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
      } else if (label == Label::PEDESTRIAN) {
        return autoware_perception_msgs::msg::Shape::CYLINDER;
      } else {
        return autoware_perception_msgs::msg::Shape::POLYGON;
      }
    }
  };  // struct LabelSettings

  struct CameraIntrinsics
  {
    // cspell: ignore Matx
    cv::Matx33d K;
    cv::Mat D;
  };  // struct CameraIntrinsics

  void roiCallback(const DetectedObjectsWithFeature::ConstSharedPtr & msg, int rois_id);
  void cameraInfoCallback(const CameraInfo::ConstSharedPtr & msg, int rois_id);
  bool generateROIBasedObject(
    const sensor_msgs::msg::RegionOfInterest & roi, const int & rois_id,
    const geometry_msgs::msg::TransformStamped & tf, const uint8_t & label,
    DetectedObject & object);

  // subscriber
  std::vector<rclcpp::Subscription<CameraInfo>::SharedPtr> camera_info_subs_;
  std::vector<rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr> roi_subs_;

  // publisher
  std::unordered_map<int, rclcpp::Publisher<DetectedObjects>::SharedPtr> objects_pubs_;

  std::string target_frame_;
  LabelSettings label_settings_;
  double detection_max_range_sq_;
  double pseudo_height_;
  double pedestrian_width_min_;
  double pedestrian_width_max_;
  bool check_roi_truncation_;
  uint32_t roi_truncation_bottom_margin_;
  double truncated_roi_projection_plane_z_;

  std::unordered_map<int, CameraInfo> camera_info_;
  std::unordered_map<int, CameraIntrinsics> cam_intrinsics_;
  std::unordered_map<int, bool> is_camera_info_arrived_;

  std::shared_ptr<TransformListener> transform_listener_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;
};

}  // namespace autoware::roi_based_detector

#endif  // ROI_BASED_DETECTOR_NODE_HPP_
