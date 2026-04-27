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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TYPES_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TYPES_HPP_

#include "autoware/multi_object_tracker/object_model/classes.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
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

#include <array>
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

enum class TrackerType {
  PASS_THROUGH = 0,
  PEDESTRIAN_AND_BICYCLE = 10,
  PEDESTRIAN = 11,
  BICYCLE = 12,
  MULTIPLE_VEHICLE = 20,
  GENERAL_VEHICLE = 21,
  NORMAL_VEHICLE = 22,
  BIG_VEHICLE = 23,
  VEHICLE = 24,
  POLYGON = 30,
};

inline constexpr std::array<TrackerType, 10> ALL_TRACKER_TYPES = {
  TrackerType::PASS_THROUGH,     TrackerType::PEDESTRIAN_AND_BICYCLE,
  TrackerType::PEDESTRIAN,       TrackerType::BICYCLE,
  TrackerType::MULTIPLE_VEHICLE, TrackerType::GENERAL_VEHICLE,
  TrackerType::NORMAL_VEHICLE,   TrackerType::BIG_VEHICLE,
  TrackerType::VEHICLE,          TrackerType::POLYGON};

inline bool isVehicleTrackerType(const TrackerType tracker_type)
{
  return tracker_type == TrackerType::MULTIPLE_VEHICLE ||
         tracker_type == TrackerType::GENERAL_VEHICLE ||
         tracker_type == TrackerType::NORMAL_VEHICLE || tracker_type == TrackerType::BIG_VEHICLE ||
         tracker_type == TrackerType::VEHICLE;
}

inline const std::array<TrackerType, 10> & allTrackerTypes()
{
  return ALL_TRACKER_TYPES;
}

inline std::string toString(const TrackerType tracker_type)
{
  switch (tracker_type) {
    case TrackerType::PASS_THROUGH:
      return "pass_through_tracker";
    case TrackerType::PEDESTRIAN_AND_BICYCLE:
      return "pedestrian_and_bicycle_tracker";
    case TrackerType::PEDESTRIAN:
      return "pedestrian_tracker";
    case TrackerType::BICYCLE:
      return "bicycle_tracker";
    case TrackerType::MULTIPLE_VEHICLE:
      return "multi_vehicle_tracker";
    case TrackerType::GENERAL_VEHICLE:
      return "general_vehicle_tracker";
    case TrackerType::NORMAL_VEHICLE:
      return "normal_vehicle_tracker";
    case TrackerType::BIG_VEHICLE:
      return "big_vehicle_tracker";
    case TrackerType::VEHICLE:
      return "vehicle_tracker";
    case TrackerType::POLYGON:
      return "polygon_tracker";
    default:
      return "polygon_tracker";
  }
}

inline std::optional<TrackerType> toTrackerType(const std::string & tracker_name)
{
  if (tracker_name == "pass_through_tracker") return TrackerType::PASS_THROUGH;
  if (tracker_name == "pedestrian_and_bicycle_tracker") {
    return TrackerType::PEDESTRIAN_AND_BICYCLE;
  }
  if (tracker_name == "pedestrian_tracker") return TrackerType::PEDESTRIAN;
  if (tracker_name == "bicycle_tracker") return TrackerType::BICYCLE;
  if (tracker_name == "multi_vehicle_tracker") return TrackerType::MULTIPLE_VEHICLE;
  if (tracker_name == "general_vehicle_tracker") return TrackerType::GENERAL_VEHICLE;
  if (tracker_name == "normal_vehicle_tracker") return TrackerType::NORMAL_VEHICLE;
  if (tracker_name == "big_vehicle_tracker") return TrackerType::BIG_VEHICLE;
  if (tracker_name == "vehicle_tracker") return TrackerType::VEHICLE;
  if (tracker_name == "polygon_tracker") return TrackerType::POLYGON;
  return std::nullopt;
}

// constants
constexpr float default_existence_probability = 0.75;

// Association algorithm selection per input channel
enum class AssociationType {
  BEV,   // BevAssociation: bird's-eye-view area scoring + GNN linear assignment
  POLAR  // PolarAssociation: polar-coordinate (range-bearing) based scoring
};

// channel configuration
struct InputChannel
{
  uint index;                                              // index of the channel
  bool is_enabled = true;                                  // enable the channel
  std::string long_name = "Detected Object";               // full name of the detection
  std::string short_name = "DET";                          // abbreviation of the name
  bool is_spawn_enabled = true;                            // enable spawn of the object
  bool trust_existence_probability = false;                // trust object existence probability
  bool trust_extension = true;                             // trust object extension
  bool trust_classification = true;                        // trust object classification
  bool trust_orientation = true;                           // trust object orientation(yaw)
  AssociationType associator_type = AssociationType::BEV;  // which associator to use
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
  std::vector<classes::Classification> classification;

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

using types::ALL_TRACKER_TYPES;
using types::allTrackerTypes;
using types::isVehicleTrackerType;
using types::toString;
using types::toTrackerType;
using types::TrackerType;

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TYPES_HPP_
