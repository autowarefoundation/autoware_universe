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

#include <opencv2/core.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <tf2/LinearMath/Vector3.h>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>  // for ROS 2 Jazzy or newer
#else
#include <image_geometry/pinhole_camera_model.h>  // for ROS 2 Humble or older
#endif

namespace autoware::traffic_light
{
namespace utils
{

cv::Point2d calcRawImagePointFromPoint3D(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const cv::Point3d & point3d);

cv::Point2d calcRawImagePointFromPoint3D(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const tf2::Vector3 & point3d);

void roundInImageFrame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, cv::Point2d & point);

bool isInDistanceRange(
  const tf2::Vector3 & p1, const tf2::Vector3 & p2, const double max_distance_range);

bool isInAngleRange(const double & tl_yaw, const double & camera_yaw, const double max_angle_range);

bool isInImageFrame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const tf2::Vector3 & point);

tf2::Vector3 getTrafficLightTopLeft(const lanelet::ConstLineString3d & traffic_light);

tf2::Vector3 getTrafficLightBottomRight(const lanelet::ConstLineString3d & traffic_light);

tf2::Vector3 getTrafficLightCenter(const lanelet::ConstLineString3d & traffic_light);

}  // namespace utils
}  // namespace autoware::traffic_light
