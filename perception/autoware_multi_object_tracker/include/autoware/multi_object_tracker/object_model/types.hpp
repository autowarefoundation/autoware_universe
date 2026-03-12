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

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace types
{
// constants
constexpr float default_existence_probability = 0.75;
constexpr int NUM_LABELS = 8;

// channel configuration
struct InputChannel
{
  uint index;                                 // index of the channel
  bool is_enabled = true;                     // enable the channel
  std::string long_name = "Detected Object";  // full name of the detection
  std::string short_name = "DET";             // abbreviation of the name
  bool is_spawn_enabled = true;               // enable spawn of the object
  bool trust_existence_probability = false;   // trust object existence probability
  bool trust_extension = true;                // trust object extension
  bool trust_classification = true;           // trust object classification
  bool trust_orientation = true;              // trust object orientation(yaw)
};

struct ExistenceProbability
{
  uint channel_index;
  float existence_probability;
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
  std::vector<ExistenceProbability> existence_probabilities;

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
  bool trust_extension;
  double area;
};

struct UUIDHash
{
  std::size_t operator()(const unique_identifier_msgs::msg::UUID & u) const
  {
    std::size_t seed = 0;
    for (const auto & b : u.uuid) {
      seed ^= std::hash<uint8_t>{}(b) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct UUIDEqual
{
  bool operator()(
    const unique_identifier_msgs::msg::UUID & u1,
    const unique_identifier_msgs::msg::UUID & u2) const
  {
    return std::equal(std::begin(u1.uuid), std::end(u1.uuid), std::begin(u2.uuid));
  }
};

struct DynamicObjectList
{
  std_msgs::msg::Header header;
  uint channel_index;
  std::vector<DynamicObject> objects;

  mutable std::unordered_map<unique_identifier_msgs::msg::UUID, size_t, UUIDHash, UUIDEqual>
    uuid_to_index_;

  std::optional<size_t> getObjectIndexByUuid(const unique_identifier_msgs::msg::UUID & uuid) const;
  void buildUuidIndex() const;
};

struct AssociationEntry
{
  size_t tracker_idx;
  size_t measurement_idx;
  double score;
  bool has_significant_shape_change;
};

struct AssociationData
{
  std::vector<AssociationEntry> entries;
  std::vector<unique_identifier_msgs::msg::UUID> tracker_uuids;
  std::vector<unique_identifier_msgs::msg::UUID> measurement_uuids;
};

struct AssociationResult
{
  std::unordered_map<
    unique_identifier_msgs::msg::UUID, unique_identifier_msgs::msg::UUID, UUIDHash, UUIDEqual>
    tracker_to_measurement;
  std::unordered_map<
    unique_identifier_msgs::msg::UUID, unique_identifier_msgs::msg::UUID, UUIDHash, UUIDEqual>
    measurement_to_tracker;
  std::vector<unique_identifier_msgs::msg::UUID> unassigned_trackers;
  std::vector<unique_identifier_msgs::msg::UUID> unassigned_measurements;
  std::unordered_set<unique_identifier_msgs::msg::UUID, UUIDHash, UUIDEqual>
    trackers_with_shape_change;

  void add(
    const unique_identifier_msgs::msg::UUID & tracker_uuid,
    const unique_identifier_msgs::msg::UUID & measurement_uuid)
  {
    tracker_to_measurement[tracker_uuid] = measurement_uuid;
    measurement_to_tracker[measurement_uuid] = tracker_uuid;
  }

  void remove(const unique_identifier_msgs::msg::UUID & tracker_uuid)
  {
    if (tracker_to_measurement.count(tracker_uuid)) {
      measurement_to_tracker.erase(tracker_to_measurement[tracker_uuid]);
      tracker_to_measurement.erase(tracker_uuid);
      trackers_with_shape_change.erase(tracker_uuid);
    }
  }

  bool wasShapeChanged(const unique_identifier_msgs::msg::UUID & tracker_uuid) const
  {
    return trackers_with_shape_change.count(tracker_uuid) > 0;
  }

  unique_identifier_msgs::msg::UUID findMeasurement(
    const unique_identifier_msgs::msg::UUID & tracker_uuid) const
  {
    if (tracker_to_measurement.count(tracker_uuid)) {
      return tracker_to_measurement.at(tracker_uuid);
    }
    return unique_identifier_msgs::msg::UUID();
  }

  unique_identifier_msgs::msg::UUID findTracker(
    const unique_identifier_msgs::msg::UUID & measurement_uuid) const
  {
    if (measurement_to_tracker.count(measurement_uuid)) {
      return measurement_to_tracker.at(measurement_uuid);
    }
    return unique_identifier_msgs::msg::UUID();
  }

  std::vector<unique_identifier_msgs::msg::UUID> getTrackerAssignments() const
  {
    std::vector<unique_identifier_msgs::msg::UUID> trackers;
    for (const auto & pair : tracker_to_measurement) {
      trackers.push_back(pair.first);
    }
    return trackers;
  }

  std::vector<unique_identifier_msgs::msg::UUID> getMeasurementAssignments() const
  {
    std::vector<unique_identifier_msgs::msg::UUID> measurements;
    for (const auto & pair : measurement_to_tracker) {
      measurements.push_back(pair.first);
    }
    return measurements;
  }
};

struct AssociatedObjects
{
  const DynamicObjectList & objects;
  const AssociationResult & association;
};

struct ObjectsWithAssociation
{
  DynamicObjectList objects;
  AssociationResult association;

  rclcpp::Time getTimestamp() const { return rclcpp::Time(objects.header.stamp); }
  rclcpp::Time getTimestamp(rcl_clock_type_t clock_type) const
  {
    return rclcpp::Time(objects.header.stamp, clock_type);
  }
};

using ObjectsWithAssociationList = std::vector<ObjectsWithAssociation>;

DynamicObject toDynamicObject(
  const autoware_perception_msgs::msg::DetectedObject & det_object, const uint channel_index = 0);

DynamicObjectList toDynamicObjectList(
  const autoware_perception_msgs::msg::DetectedObjects & det_objects, const uint channel_index = 0);

autoware_perception_msgs::msg::TrackedObject toTrackedObjectMsg(const DynamicObject & dyn_object);
autoware_perception_msgs::msg::DetectedObject toDetectedObjectMsg(const DynamicObject & dyn_object);

double getArea(const autoware_perception_msgs::msg::Shape & shape);

}  // namespace types
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__TYPES_HPP_
