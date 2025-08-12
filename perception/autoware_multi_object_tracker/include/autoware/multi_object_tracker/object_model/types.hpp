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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__TYPES_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__TYPES_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object_kinematics.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/optional.hpp>

#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace types
{
// constants
constexpr size_t max_channel_size = 12;
constexpr float default_existence_probability = 0.75;
constexpr int NUM_LABELS = 8;

// channel configuration
struct InputChannel
{
  uint index;               // index of the channel
  std::string input_topic;  // topic name of the detection, e.g. "/detection/lidar"
  std::string long_name = "Detected Object";  // full name of the detection
  std::string short_name = "DET";             // abbreviation of the name
  bool is_spawn_enabled = true;               // enable spawn of the object
  bool trust_existence_probability = false;   // trust object existence probability
  bool trust_extension = true;                // trust object extension
  bool trust_classification = true;           // trust object classification
  bool trust_orientation = true;              // trust object orientation(yaw)
};

// object model
enum OrientationAvailability : uint8_t {
  UNAVAILABLE = 0,
  SIGN_UNKNOWN = 1,
  AVAILABLE = 2,
};

struct ObjectKinematics
{
  bool has_position_covariance = false;
  OrientationAvailability orientation_availability;
  bool has_twist = false;
  bool has_twist_covariance = false;
};

struct DynamicObject
{
  // time
  rclcpp::Time time;

  // identification
  unique_identifier_msgs::msg::UUID uuid = unique_identifier_msgs::msg::UUID();

  // existence information
  uint channel_index;
  float existence_probability;
  std::vector<float> existence_probabilities;

  // object classification
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;

  // object kinematics (pose and twist)
  ObjectKinematics kinematics;
  geometry_msgs::msg::Pose pose;
  std::array<double, 36> pose_covariance;
  geometry_msgs::msg::Twist twist;
  std::array<double, 36> twist_covariance;

  // object extension (size and shape)
  autoware_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Point anchor_point;
  double area;
};

struct DynamicObjectList
{
  std_msgs::msg::Header header;
  uint channel_index;
  std::vector<DynamicObject> objects;
};

DynamicObject toDynamicObject(
  const autoware_perception_msgs::msg::DetectedObject & det_object, const uint channel_index = 0);

DynamicObjectList toDynamicObjectList(
  const autoware_perception_msgs::msg::DetectedObjects & det_objects, const uint channel_index = 0);

autoware_perception_msgs::msg::TrackedObject toTrackedObjectMsg(const DynamicObject & dyn_object);

double getArea(const autoware_perception_msgs::msg::Shape & shape);

}  // namespace types
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__TYPES_HPP_
