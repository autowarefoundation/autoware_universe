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

#include "autoware/ptv3/ros_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace autoware::ptv3
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

void box3d_to_detected_object(
  const Box3D & box3d, const std::vector<std::string> & class_names, const bool has_twist,
  autoware_perception_msgs::msg::DetectedObject & obj)
{
  obj.existence_probability = box3d.score;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0F;
  if (box3d.label >= 0 && static_cast<std::size_t>(box3d.label) < class_names.size()) {
    classification.label = get_semantic_type(class_names[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
  }

  if (autoware::object_recognition_utils::isCarLikeVehicle(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  const float yaw = box3d.yaw;
  obj.kinematics.pose_with_covariance.pose.position =
    autoware_utils::create_point(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(yaw);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    autoware_utils::create_translation(box3d.length, box3d.width, box3d.height);

  if (has_twist) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = std::cos(yaw) * box3d.vel_x + std::sin(yaw) * box3d.vel_y;
    twist.linear.y = -std::sin(yaw) * box3d.vel_x + std::cos(yaw) * box3d.vel_y;
    obj.kinematics.twist_with_covariance.twist = twist;
    obj.kinematics.has_twist = true;
  }
}

uint8_t get_semantic_type(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  }
  if (class_name == "TRUCK") {
    return Label::TRUCK;
  }
  if (class_name == "BUS") {
    return Label::BUS;
  }
  if (class_name == "TRAILER") {
    return Label::TRAILER;
  }
  if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  }
  if (class_name == "MOTORBIKE" || class_name == "MOTORCYCLE") {
    return Label::MOTORCYCLE;
  }
  if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  }
  return Label::UNKNOWN;
}

}  // namespace autoware::ptv3
