// Copyright 2025 The Autoware Contributors
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

#include "autoware/tensorrt_bevformer/marker_util.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace autoware::tensorrt_bevformer
{

visualization_msgs::msg::MarkerArray createMarkerArray(
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects)
{
  visualization_msgs::msg::MarkerArray marker_array;

  std::map<uint8_t, std::array<float, 4>> class_colors;
  // Car, Bus, Truck, Construction Vehicle, Trailer (orange)
  std::array<float, 4> orange = {1.0f, 158.0f / 255.0f, 0.0f, 1.0f};  // (255,158,0)
  class_colors[autoware_perception_msgs::msg::ObjectClassification::CAR] = orange;
  class_colors[autoware_perception_msgs::msg::ObjectClassification::BUS] = orange;
  class_colors[autoware_perception_msgs::msg::ObjectClassification::TRUCK] = orange;
#ifdef ObjectClassification_CONSTRUCTION_VEHICLE
  class_colors[autoware_perception_msgs::msg::ObjectClassification::CONSTRUCTION_VEHICLE] = orange;
#endif
#ifdef ObjectClassification_TRAILER
  class_colors[autoware_perception_msgs::msg::ObjectClassification::TRAILER] = orange;
#endif

  // Bicycle, Motorcycle (pink)
  std::array<float, 4> pink = {1.0f, 61.0f / 255.0f, 99.0f / 255.0f, 1.0f};  // (255,61,99)
  class_colors[autoware_perception_msgs::msg::ObjectClassification::BICYCLE] = pink;
  class_colors[autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE] = pink;

  // Pedestrian (blue)
  class_colors[autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN] = {
    0.0f, 0.0f, 230.0f / 255.0f, 1.0f};

  // Barrier, Traffic Cone (black)
#ifdef ObjectClassification_BARRIER
  class_colors[autoware_perception_msgs::msg::ObjectClassification::BARRIER] = {
    0.0f, 0.0f, 0.0f, 1.0f};
#endif
#ifdef ObjectClassification_TRAFFIC_CONE
  class_colors[autoware_perception_msgs::msg::ObjectClassification::TRAFFIC_CONE] = {
    0.0f, 0.0f, 0.0f, 1.0f};
#endif

  // Default (ignore) (purple)
  std::array<float, 4> default_color = {1.0f, 0.0f, 1.0f, 1.0f};

  // First, add a deletion marker to clear all previous markers
  int id = 0;
  visualization_msgs::msg::Marker deletion_marker;
  deletion_marker.header = detected_objects.header;

  deletion_marker.header.frame_id = "base_link";
  deletion_marker.ns = "bevformer_boxes";
  deletion_marker.id = id++;
  deletion_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(deletion_marker);

  for (const auto & object : detected_objects.objects) {
    visualization_msgs::msg::Marker marker;

    marker.header = detected_objects.header;

    marker.ns = "bevformer_boxes";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = object.kinematics.pose_with_covariance.pose;
    marker.scale.x = object.shape.dimensions.x;
    marker.scale.y = object.shape.dimensions.y;
    marker.scale.z = object.shape.dimensions.z;

    marker.lifetime = rclcpp::Duration::from_seconds(0.1);

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
  autoware_perception_msgs::msg::DetectedObjects & bevformer_objects)
{
  for (auto & obj : bevformer_objects.objects) {
    // Apply both coordinate swap AND rotation
    auto & pose = obj.kinematics.pose_with_covariance.pose;
    std::swap(pose.position.x, pose.position.y);
    pose.position.y = -pose.position.y;

    pose.position.y += 0.2;  // Tune this value as needed
    pose.position.x += 0.7;  // Tune this value as needed

    pose.position.z =
      std::max(0.0f, static_cast<float>(pose.position.z)) + obj.shape.dimensions.z - 1.2;

    // Convert quaternion to yaw
    tf2::Quaternion q_orig(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

    // Flip yaw across Z axis (mirror effect)
    yaw = -yaw;

    // Convert back to quaternion
    tf2::Quaternion q_new;
    q_new.setRPY(roll, pitch, yaw);
    q_new.normalize();

    // Assign back
    pose.orientation.x = q_new.x();
    pose.orientation.y = q_new.y();
    pose.orientation.z = q_new.z();
    pose.orientation.w = q_new.w();
  }

  auto marker_array = createMarkerArray(bevformer_objects);
  marker_pub->publish(marker_array);
}

}  // namespace autoware::tensorrt_bevformer
