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

#include "roi_based_detector/roi_based_detector_node.hpp"

#include <rclcpp/qos.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

namespace roi_based_detector
{
// initialize Constructor
RoiBasedDetectorNode::RoiBasedDetectorNode(const rclcpp::NodeOptions & node_options)
: Node("roi_based_detector_node", node_options)
{
  target_frame_ = declare_parameter<std::string>("target_frame");
  ignore_class_.UNKNOWN = declare_parameter<bool>("ignore_class.UNKNOWN");
  ignore_class_.CAR = declare_parameter<bool>("ignore_class.CAR");
  ignore_class_.TRUCK = declare_parameter<bool>("ignore_class.TRUCK");
  ignore_class_.BUS = declare_parameter<bool>("ignore_class.BUS");
  ignore_class_.TRAILER = declare_parameter<bool>("ignore_class.TRAILER");
  ignore_class_.MOTORCYCLE = declare_parameter<bool>("ignore_class.MOTORCYCLE");
  ignore_class_.BICYCLE = declare_parameter<bool>("ignore_class.BICYCLE");
  ignore_class_.PEDESTRIAN = declare_parameter<bool>("ignore_class.PEDESTRIAN");
  // create publisher
  rois_pub_ = this->create_publisher<DetectedObjectsWithFeature>("output_rois", 1);
  objects_pub_ = this->create_publisher<DetectedObjects>("output_objects", 1);
  // create subscriber
  roi_sub_ = this->create_subscription<DetectedObjectsWithFeature>(
    "input_rois", 1, std::bind(&RoiBasedDetectorNode::roiCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<CameraInfo>(
    "input_camera_info", rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
    std::bind(&RoiBasedDetectorNode::cameraInfoCallback, this, std::placeholders::_1));
  transform_listener_ = std::make_shared<TransformListener>(this);
}

Eigen::Matrix4d RoiBasedDetectorNode::transformToHomogeneous(
  const geometry_msgs::msg::Transform & transform)
{
  Eigen::Matrix4d homogeneous = Eigen::Matrix4d::Identity();

  // Extract translation
  homogeneous(0, 3) = transform.translation.x;
  homogeneous(1, 3) = transform.translation.y;
  homogeneous(2, 3) = transform.translation.z;

  // Extract rotation (quaternion to rotation matrix)
  tf2::Quaternion quat(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  tf2::Matrix3x3 rotationMatrix(quat);

  // Convert tf2::Matrix3x3 to Eigen::Matrix3d
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      homogeneous(i, j) = rotationMatrix[i][j];
    }
  }

  return homogeneous;
}

void RoiBasedDetectorNode::cameraInfoCallback(const CameraInfo::ConstSharedPtr & msg)
{
  camera_info_ = *msg;
  RCLCPP_INFO(get_logger(), "camera_info is received");
  if (!is_inv_projection_initialized_) {
    Eigen::Matrix4f projection;
    projection << camera_info_.p.at(0), camera_info_.p.at(1), camera_info_.p.at(2),
      camera_info_.p.at(3), camera_info_.p.at(4), camera_info_.p.at(5), camera_info_.p.at(6),
      camera_info_.p.at(7), camera_info_.p.at(8), camera_info_.p.at(9), camera_info_.p.at(10),
      camera_info_.p.at(11), 0.0, 0.0, 0.0, 1.0;
    inv_projection_ = projection.inverse();
    is_inv_projection_initialized_ = true;
  }
}
void RoiBasedDetectorNode::pixelTo3DPoint(
  const Eigen::Vector2f & pixel, const Eigen::Matrix4f & transform, Eigen::Vector4f & point)
{
  auto w_div_projected_z =
    -(transform(2, 0) * pixel(0) + transform(2, 1) * pixel(1) + transform(2, 2)) / transform(2, 3);
  auto projected_z = 1.0 / (transform(3, 0) * pixel(0) + transform(3, 1) * pixel(1) +
                            transform(3, 2) + transform(3, 3) * w_div_projected_z);
  auto w = w_div_projected_z * projected_z;
  Eigen::Vector4f projected_point =
    Eigen::Vector4f(pixel(0) * projected_z, pixel(1) * projected_z, projected_z, w);
  point = transform * projected_point;
}
// implement roiCallback
void RoiBasedDetectorNode::roiCallback(const DetectedObjectsWithFeature::ConstSharedPtr & msg)
{
  if (!is_inv_projection_initialized_) {
    RCLCPP_WARN(get_logger(), "camera_info is not received yet");
    return;
  }
  DetectedObjects objects;

  // get transform from camera frame to base_link frame

  try {
    transform_ = transform_listener_->getTransform(
      target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to get transform: %s", ex.what());
    objects.header = msg->header;
    objects_pub_->publish(objects);
    return;
  }

  if (transform_ == nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "getTransform failed. output objects will be empty.");
    objects.header = msg->header;
    objects_pub_->publish(objects);
    return;
  }
  if (!is_camera2lidar_mul_inv_projection_initialized_) {
    const Eigen::Matrix4f transform_matrix_cam2base =
      tf2::transformToEigen(transform_->transform).matrix().cast<float>();
    camera2lidar_mul_inv_projection_ = transform_matrix_cam2base * inv_projection_;
    is_camera2lidar_mul_inv_projection_initialized_ = true;
  }

  for (const auto & obj_with_feature : msg->feature_objects) {
    DetectedObject object;
    const auto & label = obj_with_feature.object.classification.front().label;
    if (!ignore_class_.isIgnore(label)) {
      continue;
    }
    object.classification.push_back(obj_with_feature.object.classification.front());
    object.existence_probability = obj_with_feature.object.existence_probability;
    Eigen::Vector2f pixel_center(
      static_cast<float>(
        obj_with_feature.feature.roi.x_offset + obj_with_feature.feature.roi.width / 2.0),
      static_cast<float>(
        obj_with_feature.feature.roi.y_offset + obj_with_feature.feature.roi.height / 2.0));

    Eigen::Vector4f point_center;
    pixelTo3DPoint(pixel_center, camera2lidar_mul_inv_projection_, point_center);

    std::vector<Eigen::Vector2f> footprint_pixels;
    footprint_pixels.push_back(
      Eigen::Vector2f(
        static_cast<float>(obj_with_feature.feature.roi.x_offset),
        static_cast<float>(obj_with_feature.feature.roi.y_offset)));
    footprint_pixels.push_back(
      Eigen::Vector2f(
        static_cast<float>(
          obj_with_feature.feature.roi.x_offset + obj_with_feature.feature.roi.width),
        static_cast<float>(obj_with_feature.feature.roi.y_offset)));
    footprint_pixels.push_back(
      Eigen::Vector2f(
        static_cast<float>(
          obj_with_feature.feature.roi.x_offset + obj_with_feature.feature.roi.width),
        static_cast<float>(
          obj_with_feature.feature.roi.y_offset + obj_with_feature.feature.roi.height)));
    footprint_pixels.push_back(
      Eigen::Vector2f(
        static_cast<float>(obj_with_feature.feature.roi.x_offset),
        static_cast<float>(
          obj_with_feature.feature.roi.y_offset + obj_with_feature.feature.roi.height)));

    float max_x{-150.0}, min_x{150.0}, max_y{-150.0}, min_y{150.0};
    for (const auto & pixel : footprint_pixels) {
      Eigen::Vector4f point;
      pixelTo3DPoint(pixel, camera2lidar_mul_inv_projection_, point);
      geometry_msgs::msg::Point32 geo_point;
      geo_point.x = point(0);
      geo_point.y = point(1);
      geo_point.z = point(2);
      object.shape.footprint.points.push_back(geo_point);
      max_x = std::max(max_x, point(0));
      min_x = std::min(min_x, point(0));
      max_y = std::max(max_y, point(1));
      min_y = std::min(min_y, point(1));
    }

    geometry_msgs::msg::PoseStamped pose_stamped{};
    pose_stamped.pose.position.x = point_center(0);
    pose_stamped.pose.position.y = point_center(1);
    pose_stamped.pose.position.z = point_center(2);
    object.kinematics.pose_with_covariance.pose = pose_stamped.pose;

    object.shape.dimensions.x = std::abs(max_x - min_x);
    object.shape.dimensions.y = std::abs(max_y - min_y);

    objects.objects.push_back(object);
  }
  rois_pub_->publish(*msg);
  objects.header = msg->header;
  objects.header.frame_id = target_frame_;
  objects_pub_->publish(objects);
}
}  // namespace roi_based_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(roi_based_detector::RoiBasedDetectorNode)
