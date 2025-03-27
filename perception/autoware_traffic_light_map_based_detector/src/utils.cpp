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

#include "utils.hpp"

#include <Eigen/Core>

#include <sensor_msgs/msg/camera_info.hpp>

#include <lanelet2_core/Attribute.h>

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

void roundInImageFrame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, cv::Point2d & point)
{
  const sensor_msgs::msg::CameraInfo camera_info = pinhole_camera_model.cameraInfo();
  point.x =
    std::max(std::min(point.x, static_cast<double>(static_cast<int>(camera_info.width) - 1)), 0.0);
  point.y =
    std::max(std::min(point.y, static_cast<double>(static_cast<int>(camera_info.height) - 1)), 0.0);
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

tf2::Vector3 getTrafficLightTopLeft(const lanelet::ConstLineString3d & traffic_light)
{
  const auto & tl_bl = traffic_light.front();
  const double tl_height = traffic_light.attributeOr("height", 0.0);
  return tf2::Vector3(tl_bl.x(), tl_bl.y(), tl_bl.z() + tl_height);
}

tf2::Vector3 getTrafficLightBottomRight(const lanelet::ConstLineString3d & traffic_light)
{
  const auto & tl_bl = traffic_light.back();
  return tf2::Vector3(tl_bl.x(), tl_bl.y(), tl_bl.z());
}

tf2::Vector3 getTrafficLightCenter(const lanelet::ConstLineString3d & traffic_light)
{
  tf2::Vector3 top_left = getTrafficLightTopLeft(traffic_light);
  tf2::Vector3 bottom_right = getTrafficLightBottomRight(traffic_light);
  return (top_left + bottom_right) / 2;
}

}  // namespace utils
}  // namespace autoware::traffic_light
