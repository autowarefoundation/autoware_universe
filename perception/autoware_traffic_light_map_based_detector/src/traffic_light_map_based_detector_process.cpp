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

#include "traffic_light_map_based_detector_process.hpp"

#include <Eigen/Core>
#include <autoware_utils/math/normalization.hpp>

#include <sensor_msgs/msg/camera_info.hpp>

#include <lanelet2_core/Attribute.h>

#include <algorithm>
#include <vector>

namespace autoware::traffic_light
{
namespace utils
{

cv::Point2d calcRawImagePointFromPoint3D(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const cv::Point3d & point3d)
{
  cv::Point2d rectified_image_point = pinhole_camera_model.project3dToPixel(point3d);
  return pinhole_camera_model.unrectifyPoint(rectified_image_point);
}

cv::Point2d calcRawImagePointFromPoint3D(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const tf2::Vector3 & point3d)
{
  return calcRawImagePointFromPoint3D(
    pinhole_camera_model, cv::Point3d(point3d.x(), point3d.y(), point3d.z()));
}

void roundInImageFrame(const uint32_t & width, const uint32_t & height, cv::Point2d & point)
{
  point.x = std::max(std::min(point.x, static_cast<double>(static_cast<int>(width) - 1)), 0.0);
  point.y = std::max(std::min(point.y, static_cast<double>(static_cast<int>(height) - 1)), 0.0);
}

bool isInDistanceRange(
  const tf2::Vector3 & p1, const tf2::Vector3 & p2, const double max_distance_range)
{
  const double sq_dist =
    (p1.x() - p2.x()) * (p1.x() - p2.x()) + (p1.y() - p2.y()) * (p1.y() - p2.y());
  return sq_dist < (max_distance_range * max_distance_range);
}

bool isInAngleRange(const double & tl_yaw, const double & camera_yaw, const double max_angle_range)
{
  Eigen::Vector2d vec1, vec2;
  vec1 << std::cos(tl_yaw), std::sin(tl_yaw);
  vec2 << std::cos(camera_yaw), std::sin(camera_yaw);
  const double diff_angle = std::acos(vec1.dot(vec2));
  return std::fabs(diff_angle) < max_angle_range;
}

bool isInImageFrame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const tf2::Vector3 & point)
{
  if (point.z() <= 0.0) {
    return false;
  }

  cv::Point2d point2d = calcRawImagePointFromPoint3D(pinhole_camera_model, point);
  if (0 <= point2d.x && point2d.x < pinhole_camera_model.cameraInfo().width) {
    if (0 <= point2d.y && point2d.y < pinhole_camera_model.cameraInfo().height) {
      return true;
    }
  }
  return false;
}

tf2::Vector3 getVibrationMargin(
  const double depth, const double margin_pitch, const double margin_yaw,
  const double margin_height, const double margin_width, const double margin_depth)
{
  // for small angles, tan(a) ≈ sin(a) ≈ a
  const double x = std::sin(margin_yaw * 0.5) * depth + margin_width * 0.5;
  const double y = std::sin(margin_pitch * 0.5) * depth + margin_height * 0.5;
  const double z = margin_depth * 0.5;
  return tf2::Vector3(x, y, z);
}

void computeBoundingRoi(
  const uint32_t & width, const uint32_t & height,
  const std::vector<tier4_perception_msgs::msg::TrafficLightRoi> & rois,
  tier4_perception_msgs::msg::TrafficLightRoi & max_roi)
{
  uint32_t x1 = width - 1;
  uint32_t x2 = 0;
  uint32_t y1 = height - 1;
  uint32_t y2 = 0;
  for (const auto & roi : rois) {
    x1 = std::min(x1, roi.roi.x_offset);
    x2 = std::max(x2, roi.roi.x_offset + roi.roi.width);
    y1 = std::min(y1, roi.roi.y_offset);
    y2 = std::max(y2, roi.roi.y_offset + roi.roi.height);
  }
  max_roi.roi.x_offset = x1;
  max_roi.roi.y_offset = y1;
  max_roi.roi.width = x2 - x1;
  max_roi.roi.height = y2 - y1;
}

double getTrafficLightYaw(const lanelet::ConstLineString3d & traffic_light)
{
  const auto & tl_tl = traffic_light.front();
  const auto & tl_br = traffic_light.back();
  return autoware_utils::normalize_radian(std::atan2(tl_br.y() - tl_tl.y(), tl_br.x() - tl_tl.x()));
}

double getCameraYaw(const tf2::Transform & tf_map2camera)
{
  tf2::Vector3 ray_camera_optical(0, 0, 1);
  tf2::Matrix3x3 map2camera_optical(tf_map2camera.getRotation());
  tf2::Vector3 ray_map = map2camera_optical * ray_camera_optical;
  return autoware_utils::normalize_radian(std::atan2(ray_map.y(), ray_map.x()));
}

}  // namespace utils
}  // namespace autoware::traffic_light
