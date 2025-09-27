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

#include "roi_based_detector_node.hpp"

#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/region_of_interest.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <memory>
#include <string>
#include <vector>

// cspell: ignore Matx

namespace autoware::roi_based_detector
{
void transformToRT(const geometry_msgs::msg::TransformStamped & tf, cv::Matx33d & R, cv::Vec3d & t)
{
  tf2::Quaternion q(
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
    tf.transform.rotation.w);

  tf2::Matrix3x3 m(q);

  R = cv::Matx33d(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]);

  t = cv::Vec3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
}

/**
 * @brief Computes the ground intersection point of a pixel ray.
 *
 * Assumes that the ROI covers the entire object, and projects the given image point
 * onto the ground plane (z=0) in target frame coordinate.
 */
cv::Vec3d projectToGround(
  const cv::Point2f & pixel, const cv::Matx33d & K, const cv::Mat & D, const cv::Matx33d & R,
  const cv::Vec3d & t)
{
  std::vector<cv::Point2f> pixels = {pixel};
  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(pixels, undistorted, K, D);

  cv::Vec3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0);
  cv::Vec3d ray_world = R * ray_cam;
  cv::Vec3d cam_origin = t;

  // compute the scale factor (lambda) where the ray intersects the ground plane (z=0)
  const double lambda = -cam_origin[2] / ray_world[2];

  return cam_origin + lambda * ray_world;
}

/**
 * @brief Back-projects a pixel into 3D space at a specified depth.
 *
 * Given a pixel coordinate, this function computes the corresponding 3D point
 * located at depth z along the camera's optical axis, and returns its position
 * in target frame coordinate.
 */
cv::Vec3d projectPixelToImagePlane(
  const cv::Point2f & pixel, const cv::Matx33d & K, const cv::Mat & D, const cv::Matx33d & R,
  const cv::Vec3d & t, double z)
{
  std::vector<cv::Point2f> pixels = {pixel};
  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(pixels, undistorted, K, D);

  cv::Vec3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0);
  cv::Vec3d point_cam = ray_cam * z;
  cv::Vec3d point_world = R * point_cam + t;

  return point_world;
}

/**
 * @brief Computes the height of an ROI in 3D space.
 *
 * Uses the top pixel of the ROI to cast a ray into 3D, and estimates the height
 * by intersecting this ray with the (x, y) location of the projected bottom pixel.
 * Falls back to a pseudo height if no valid intersection is found.
 */
double computeHeight(
  const cv::Point2f & pixel_top, const cv::Matx33d & K, const cv::Mat & D, const cv::Matx33d & R,
  const cv::Vec3d & t, const cv::Vec3d & projected_bottom_pixel, const double pseudo_height)
{
  std::vector<cv::Point2f> pixels = {pixel_top};
  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(pixels, undistorted, K, D);

  cv::Vec3d ray_cam(undistorted[0].x, undistorted[0].y, 1.0);
  cv::Vec3d ray_world = R * ray_cam;
  cv::Vec3d cam_origin = t;

  // search for the scale factor (lambda) at which the ray intersects
  // the (x, y) position of the projected bottom pixel
  const double lambda_x = (projected_bottom_pixel[0] - cam_origin[0]) / ray_world[0];
  const double lambda_y = (projected_bottom_pixel[1] - cam_origin[1]) / ray_world[1];

  double lambda;
  // handle cases where the ray points in the opposite direction (negative lambda)
  if (lambda_x >= 0) {
    if (lambda_y >= 0) {
      // average the two solutions for robustness
      // NOTE: a least-squares solution might be more accurate
      lambda = (lambda_x + lambda_y) / 2.0;
    } else {
      lambda = lambda_x;
    }
  } else {
    if (lambda_y >= 0) {
      lambda = lambda_y;
    } else {
      // if both lambdas are negative, fall back to the pseudo height
      lambda = pseudo_height;
    }
  }

  cv::Vec3d projected_top_point = cam_origin + lambda * ray_world;

  return projected_top_point[2];
}

// initialize constructor
RoiBasedDetectorNode::RoiBasedDetectorNode(const rclcpp::NodeOptions & node_options)
: Node("roi_based_detector", node_options)
{
  target_frame_ = declare_parameter<std::string>("target_frame");

  label_settings_.UNKNOWN = declare_parameter<bool>("ignore_class.UNKNOWN");
  label_settings_.CAR = declare_parameter<bool>("ignore_class.CAR");
  label_settings_.TRUCK = declare_parameter<bool>("ignore_class.TRUCK");
  label_settings_.BUS = declare_parameter<bool>("ignore_class.BUS");
  label_settings_.TRAILER = declare_parameter<bool>("ignore_class.TRAILER");
  label_settings_.MOTORCYCLE = declare_parameter<bool>("ignore_class.MOTORCYCLE");
  label_settings_.BICYCLE = declare_parameter<bool>("ignore_class.BICYCLE");
  label_settings_.PEDESTRIAN = declare_parameter<bool>("ignore_class.PEDESTRIAN");

  const double detection_max_range = declare_parameter<double>("detection_max_range");
  detection_max_range_sq_ = detection_max_range * detection_max_range;
  pseudo_height_ = declare_parameter<double>("pseudo_height");

  std::vector<double> pedestrian_width_limits =
    declare_parameter<std::vector<double>>("pedestrian_width_limits");
  pedestrian_width_min_ = pedestrian_width_limits[0];
  pedestrian_width_max_ = pedestrian_width_limits[1];

  check_roi_truncation_ = declare_parameter<bool>("roi_truncation.check_truncation");
  roi_truncation_bottom_margin_ =
    declare_parameter<int64_t>("roi_truncation.truncation_image_bottom_margin");
  truncated_roi_projection_plane_z_ =
    declare_parameter<double>("roi_truncation.projection_plane_z");

  std::vector<int64_t> rois_ids = declare_parameter<std::vector<int64_t>>("rois_ids");
  size_t rois_number = rois_ids.size();

  // create subscriber and publisher
  camera_info_subs_.resize(rois_number);
  roi_subs_.resize(rois_number);
  for (auto rois_id_index = 0u; rois_id_index < rois_number; ++rois_id_index) {
    const int rois_id = rois_ids[rois_id_index];
    const std::string rois_id_str = std::to_string(rois_id);

    is_camera_info_arrived_[rois_id] = false;

    // subscriber: camera info
    const std::string camera_info_topic_name = declare_parameter<std::string>(
      "input/camera_info" + rois_id_str, "/sensing/camera/camera" + rois_id_str + "/camera_info");
    const auto camera_info_qos = rclcpp::QoS{1}.best_effort();

    camera_info_subs_[rois_id_index] = this->create_subscription<CameraInfo>(
      camera_info_topic_name, camera_info_qos,
      [this, rois_id](const CameraInfo::ConstSharedPtr msg) {
        this->cameraInfoCallback(msg, rois_id);
      });

    // subscriber: roi
    const std::string roi_topic_name = declare_parameter<std::string>(
      "input/rois" + rois_id_str, "/perception/object_recognition/detection/rois" + rois_id_str);
    const auto roi_qos = rclcpp::QoS{1}.best_effort();

    roi_subs_[rois_id_index] = this->create_subscription<DetectedObjectsWithFeature>(
      roi_topic_name, roi_qos,
      [this, rois_id](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
        this->roiCallback(msg, rois_id);
      });

    // publisher
    const std::string output_topic_name = declare_parameter<std::string>(
      "output/rois" + rois_id_str + "/objects", "~/rois" + rois_id_str + "/objects");
    objects_pubs_[rois_id] = this->create_publisher<DetectedObjects>(output_topic_name, 1);
  }

  transform_listener_ = std::make_shared<TransformListener>(this);
}

void RoiBasedDetectorNode::cameraInfoCallback(const CameraInfo::ConstSharedPtr & msg, int rois_id)
{
  // assuming camera parameter never changes while node is running
  if (!is_camera_info_arrived_[rois_id]) {
    CameraInfo camera_info = *msg;
    camera_info_[rois_id] = camera_info;

    // K is row-major 3x3
    cv::Matx33d K(
      camera_info.k[0], camera_info.k[1], camera_info.k[2], camera_info.k[3], camera_info.k[4],
      camera_info.k[5], camera_info.k[6], camera_info.k[7], camera_info.k[8]);

    cv::Mat D = cv::Mat(camera_info.d.size(), 1, CV_64F);
    for (size_t i = 0; i < camera_info.d.size(); i++) {
      D.at<double>(i, 0) = camera_info.d[i];
    }

    cam_intrinsics_[rois_id] = CameraIntrinsics{K, D};
    is_camera_info_arrived_[rois_id] = true;
  }
}

/**
 * @brief Estimates a 3D object from a 2D ROI.
 *
 * This function primarily targets pedestrian detection, as the generated 3D object
 * is only an approximation and may be inaccurate without depth information.
 */
bool RoiBasedDetectorNode::generateROIBasedObject(
  const sensor_msgs::msg::RegionOfInterest & roi, const int & rois_id,
  const geometry_msgs::msg::TransformStamped & tf, const uint8_t & label, DetectedObject & object)
{
  CameraIntrinsics cam_intrinsics = cam_intrinsics_[rois_id];
  const cv::Matx33d K = cam_intrinsics.K;
  const cv::Mat D = cam_intrinsics.D;

  const uint32_t x_offset = roi.x_offset;
  const uint32_t y_offset = roi.y_offset;
  const uint32_t roi_width = roi.width;
  const uint32_t roi_height = roi.height;

  // use the ROI's bottom center to estimate the 3D position,
  // and the top center to approximate object height
  const uint32_t bottom_pixel_y = y_offset + roi_height;
  const cv::Point2f bottom_center(x_offset + roi_width * 0.5f, bottom_pixel_y);
  const cv::Point2f top_center(x_offset + roi_width * 0.5f, y_offset);
  // bottom left and right points are used to estimate the object's width/diameter
  const cv::Point2f bottom_left(x_offset, bottom_pixel_y);
  const cv::Point2f bottom_right(x_offset + roi.width, bottom_pixel_y);

  cv::Matx33d R;
  cv::Vec3d t;
  transformToRT(tf, R, t);

  cv::Vec3d bottom_point_in_3d;
  if (
    check_roi_truncation_ &&
    bottom_pixel_y >= (camera_info_[rois_id].height - roi_truncation_bottom_margin_)) {
    // if the ROI is close to the image bottom border, treat it as truncated
    bottom_point_in_3d =
      projectPixelToImagePlane(bottom_center, K, D, R, t, truncated_roi_projection_plane_z_);
    bottom_point_in_3d[2] = 0.0;
  } else {
    bottom_point_in_3d = projectToGround(bottom_center, K, D, R, t);
  }

  const double dist_sq =
    bottom_point_in_3d[0] * bottom_point_in_3d[0] + bottom_point_in_3d[1] * bottom_point_in_3d[1];
  if (dist_sq > detection_max_range_sq_) {
    // skip this ROI if it is beyond the maximum detection range
    return false;
  }

  const double height = computeHeight(top_center, K, D, R, t, bottom_point_in_3d, pseudo_height_);

  const cv::Vec3d left_point_in_3d = projectToGround(bottom_left, K, D, R, t);
  const cv::Vec3d right_point_in_3d = projectToGround(bottom_right, K, D, R, t);

  const double dim_xy = cv::norm(left_point_in_3d - right_point_in_3d);

  object.kinematics.pose_with_covariance.pose.position.x = bottom_point_in_3d[0];
  object.kinematics.pose_with_covariance.pose.position.y = bottom_point_in_3d[1];
  object.kinematics.pose_with_covariance.pose.position.z = height * 0.5;

  // this function primarily targets pedestrian, we will check the dimensions of pedestrian
  if (label == Label::PEDESTRIAN) {
    const double clamped_dim_xy = std::clamp(dim_xy, pedestrian_width_min_, pedestrian_width_max_);
    object.shape.dimensions.x = clamped_dim_xy;
    object.shape.dimensions.y = clamped_dim_xy;
  } else {
    object.shape.dimensions.x = dim_xy;
    object.shape.dimensions.y = dim_xy;
  }
  object.shape.dimensions.z = height;

  object.shape.type = label_settings_.getLabelShape(label);
  if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    const float spacing = static_cast<float>(dim_xy) * 0.5f;

    // add footprint polygon points for visualization
    geometry_msgs::msg::Point32 p1;
    p1.x = -spacing;
    p1.y = -spacing;
    p1.z = 0.0f;
    object.shape.footprint.points.push_back(p1);

    geometry_msgs::msg::Point32 p2;
    p2.x = -spacing;
    p2.y = +spacing;
    p2.z = 0.0f;
    object.shape.footprint.points.push_back(p2);

    geometry_msgs::msg::Point32 p3;
    p3.x = +spacing;
    p3.y = +spacing;
    p3.z = 0.0f;
    object.shape.footprint.points.push_back(p3);

    geometry_msgs::msg::Point32 p4;
    p4.x = +spacing;
    p4.y = -spacing;
    p4.z = 0.0f;
    object.shape.footprint.points.push_back(p4);
  }

  return true;
}

void RoiBasedDetectorNode::roiCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & msg, int rois_id)
{
  if (!is_camera_info_arrived_[rois_id]) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info is not received yet");
    return;
  }

  DetectedObjects objects;

  // get transform from camera frame to target frame
  try {
    transform_ = transform_listener_->getTransform(
      target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to get transform: %s", ex.what());
    objects.header = msg->header;
    objects_pubs_[rois_id]->publish(objects);
    return;
  }

  if (!transform_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "getTransform failed. output objects will be empty.");
    objects.header = msg->header;
    objects_pubs_[rois_id]->publish(objects);
    return;
  }

  for (const auto & obj_with_feature : msg->feature_objects) {
    DetectedObject object;

    const auto & label = obj_with_feature.object.classification.front().label;
    if (label_settings_.isIgnoreLabel(label)) {
      continue;
    }

    object.classification.push_back(obj_with_feature.object.classification.front());
    object.existence_probability = obj_with_feature.object.existence_probability;

    // if it is in the detection range, we will publish it
    if (generateROIBasedObject(obj_with_feature.feature.roi, rois_id, *transform_, label, object)) {
      objects.objects.push_back(object);
    }
  }

  objects.header = msg->header;
  objects.header.frame_id = target_frame_;
  objects_pubs_[rois_id]->publish(objects);
}

}  // namespace autoware::roi_based_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::roi_based_detector::RoiBasedDetectorNode)
