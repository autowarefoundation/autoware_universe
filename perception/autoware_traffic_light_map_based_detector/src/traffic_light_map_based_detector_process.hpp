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

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR_PROCESS_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR_PROCESS_HPP_

#include <opencv2/core.hpp>

#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>

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

void roundInImageFrame(const uint32_t & width, const uint32_t & height, cv::Point2d & point);

bool isInDistanceRange(
  const tf2::Vector3 & p1, const tf2::Vector3 & p2, const double max_distance_range);

bool isInAngleRange(const double & tl_yaw, const double & camera_yaw, const double max_angle_range);

bool isInImageFrame(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const tf2::Vector3 & point);

// Calculated in the camera optical frame but yaw and pitch are in the camera frame
tf2::Vector3 getVibrationMargin(
  const double depth, const double margin_pitch, const double margin_yaw,
  const double margin_height, const double margin_width, const double margin_depth);

void computeBoundingRoi(
  const uint32_t & width, const uint32_t & height,
  const std::vector<tier4_perception_msgs::msg::TrafficLightRoi> & rois,
  tier4_perception_msgs::msg::TrafficLightRoi & max_roi);

double getTrafficLightYaw(const lanelet::ConstLineString3d & traffic_light);

double getCameraYaw(const tf2::Transform & tf_map2camera);

}  // namespace utils
}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_MAP_BASED_DETECTOR_PROCESS_HPP_
