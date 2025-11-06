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

#ifndef BBOX_OBJECT_LOCATOR_NODE_HPP_
#define BBOX_OBJECT_LOCATOR_NODE_HPP_

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

// cspell: ignore Matx

namespace autoware::image_object_locator
{
using autoware::universe_utils::TransformListener;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using Label = autoware_perception_msgs::msg::ObjectClassification;

class BboxObjectLocatorNode : public rclcpp::Node
{
public:
  explicit BboxObjectLocatorNode(const rclcpp::NodeOptions & node_options);

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

    bool isDetectionTargetLabel(const uint8_t label) const
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
    cv::Matx33d K;
    cv::Mat D;
  };  // struct CameraIntrinsics

  struct RoiValidator
  {
    bool enable_validation;
    bool remove_object_might_be_truncated;

    double image_border_truncation_horizontal_margin_ratio;
    double image_border_truncation_vertical_margin_ratio;

    // used to decide an actual object whether it might be truncated
    uint32_t pixel_truncated_top;
    uint32_t pixel_truncated_bottom;
    uint32_t pixel_truncated_left;
    uint32_t pixel_truncated_right;
  };  // struct RoiValidator

  struct CovarianceControlParams
  {
    // tangential (bearing) settings
    static constexpr double sigma_bearing_deg = 2.5;

    // radial sigma settings
    static constexpr double radial_sigma_bias = 0.7;
    static constexpr double radial_sigma_slope = 0.10;

    // coefficient for adding additional sigma values when the ROI is truncated
    static constexpr double horizontal_bias_coeff = 2;
    static constexpr double vertical_bias_coeff = 2;

    // SPD floor
    static constexpr double eps_spd = 1e-6;

    // tentative sigma value for points that are too close to the camera
    static constexpr double sigma_close_to_camera_2 = 0.2 * 0.2;
  };  // struct CovarianceControlParams

  bool isRoiValidationParamValid(
    const size_t rois_number, const std::vector<bool> & validation_enable,
    const std::vector<bool> & remove_truncated);
  void roiCallback(const DetectedObjectsWithFeature::ConstSharedPtr & msg, int rois_id);
  void cameraInfoCallback(const CameraInfo::ConstSharedPtr & msg, int rois_id);
  cv::Matx22d computeCovarianceXY(
    const cv::Vec3d & object_ground_point, const cv::Vec3d & cam_t, const double object_width,
    const double horizontal_bias_coeff, const double vertical_bias_coeff);
  bool generateROIBasedObject(
    const sensor_msgs::msg::RegionOfInterest & roi, const int & rois_id,
    const geometry_msgs::msg::TransformStamped & tf, const uint8_t & label,
    DetectedObject & object);

  // subscriber
  std::vector<rclcpp::Subscription<CameraInfo>::SharedPtr> camera_info_subs_;
  std::vector<rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr> roi_subs_;

  // publisher
  std::unordered_map<int, rclcpp::Publisher<DetectedObjects>::SharedPtr> objects_pubs_;

  CovarianceControlParams covariance_config_;

  std::string target_frame_;
  LabelSettings label_settings_;
  std::unordered_map<int, RoiValidator> roi_validator_;
  double roi_confidence_th_;
  double detection_max_range_sq_;
  double pseudo_height_;
  double pedestrian_width_min_;
  double pedestrian_width_max_;

  // in camera coordinate
  cv::Vec3d camera_optical_axis_{0.0, 0.0, 1.0};

  std::unordered_map<int, CameraInfo> camera_info_;
  std::unordered_map<int, CameraIntrinsics> cam_intrinsics_;
  std::unordered_map<int, bool> is_camera_info_arrived_;

  std::shared_ptr<TransformListener> transform_listener_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;
};

}  // namespace autoware::image_object_locator

#endif  // BBOX_OBJECT_LOCATOR_NODE_HPP_
