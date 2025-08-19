// Copyright 2025 AutoCore, Inc.
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

#include "autoware/tensorrt_bevdet/marker_utils.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <map>
#include <memory>
#include <string>

namespace autoware::tensorrt_bevdet
{

visualization_msgs::msg::MarkerArray createMarkerArray(
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Define NuScenes-like color mapping
  std::map<uint8_t, std::array<float, 4>> class_colors;
  // Car (orange) - R:255, G:158, B:0
  class_colors[autoware_perception_msgs::msg::ObjectClassification::CAR] = {
    1.0f, 0.62f, 0.0f, 1.0f};
  // Pedestrian (blue) - R:0, G:0, B:230
  class_colors[autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN] = {
    0.0f, 0.0f, 0.9f, 1.0f};
  // Bicycle (pink) - R:255, G:61, B:99
  class_colors[autoware_perception_msgs::msg::ObjectClassification::BICYCLE] = {
    1.0f, 0.24f, 0.39f, 1.0f};
  // Motorcycle (pink) - R:255, G:61, B:99
  class_colors[autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE] = {
    1.0f, 0.24f, 0.39f, 1.0f};
  // Bus (orange) - R:255, G:158, B:0
  class_colors[autoware_perception_msgs::msg::ObjectClassification::BUS] = {
    1.0f, 0.62f, 0.0f, 1.0f};
  // Truck (orange) - R:255, G:158, B:0
  class_colors[autoware_perception_msgs::msg::ObjectClassification::TRUCK] = {
    1.0f, 0.62f, 0.0f, 1.0f};
  // Default (purple) - R:255, G:0, B:255
  std::array<float, 4> default_color = {1.0f, 0.0f, 1.0f, 1.0f};

  // First, add a deletion marker to clear all previous markers
  int id = 0;
  visualization_msgs::msg::Marker deletion_marker;
  deletion_marker.header = detected_objects.header;

  deletion_marker.header.frame_id = "baselink";
  deletion_marker.ns = "bevdet_boxes";
  deletion_marker.id = id++;
  deletion_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(deletion_marker);

  for (const auto & object : detected_objects.objects) {
    visualization_msgs::msg::Marker marker;

    marker.header = detected_objects.header;
    marker.ns = "bevdet_boxes";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = object.kinematics.pose_with_covariance.pose;
    marker.scale.x = object.shape.dimensions.x;
    marker.scale.y = object.shape.dimensions.y;
    marker.scale.z = object.shape.dimensions.z;

    // Make the boxes THICK (optional - keep if you want thicker boxes)
    marker.scale.x += 0.2;
    marker.scale.y += 0.2;
    marker.scale.z += 0.2;

    marker.lifetime = rclcpp::Duration::from_seconds(0.3);

    // Set color based on NuScenes color scheme
    uint8_t label = object.classification.front().label;
    std::array<float, 4> color = default_color;

    if (class_colors.find(label) != class_colors.end()) {
      color = class_colors[label];
    }

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

void publishDebugMarkers(
  const std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> & marker_pub,
  autoware_perception_msgs::msg::DetectedObjects & bevdet_objects)
{
  // Apply coordinate transformation to match NuScenes
  for (auto & obj : bevdet_objects.objects) {
    // Apply orientation correction
    tf2::Quaternion q(
      obj.kinematics.pose_with_covariance.pose.orientation.x,
      obj.kinematics.pose_with_covariance.pose.orientation.y,
      obj.kinematics.pose_with_covariance.pose.orientation.z,
      obj.kinematics.pose_with_covariance.pose.orientation.w);

    // Apply PI rotation around Z-axis (180 degrees)
    tf2::Quaternion correction;
    correction.setRPY(0, 0, M_PI);
    q = correction * q;
    q.normalize();

    // Update orientation
    obj.kinematics.pose_with_covariance.pose.orientation.x = q.x();
    obj.kinematics.pose_with_covariance.pose.orientation.y = q.y();
    obj.kinematics.pose_with_covariance.pose.orientation.z = q.z();
    obj.kinematics.pose_with_covariance.pose.orientation.w = q.w();
  }

  auto marker_array = createMarkerArray(bevdet_objects);
  marker_pub->publish(marker_array);
}

}  // namespace autoware::tensorrt_bevdet
