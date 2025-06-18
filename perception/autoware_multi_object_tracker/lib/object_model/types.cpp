// Copyright 2024 Tier IV, Inc.
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

#include "autoware/multi_object_tracker/object_model/types.hpp"

#include <vector>

namespace autoware::multi_object_tracker
{

namespace types
{

DynamicObject toDynamicObject(
  const autoware_perception_msgs::msg::DetectedObject & det_object, const uint channel_index)
{
  DynamicObject dynamic_object;

  // initialize existence_probabilities, using channel information
  dynamic_object.channel_index = channel_index;
  if (det_object.existence_probability < 1e-6) {
    // given existence probability is too low, may the value is not set
    dynamic_object.existence_probability = default_existence_probability;
  } else if (det_object.existence_probability > 0.999) {
    // given existence probability is too high, may the value is not set
    dynamic_object.existence_probability = 0.999;
  } else {
    dynamic_object.existence_probability = det_object.existence_probability;
  }

  dynamic_object.classification = det_object.classification;

  dynamic_object.pose = det_object.kinematics.pose_with_covariance.pose;
  dynamic_object.pose_covariance = det_object.kinematics.pose_with_covariance.covariance;
  dynamic_object.twist = det_object.kinematics.twist_with_covariance.twist;
  dynamic_object.twist_covariance = det_object.kinematics.twist_with_covariance.covariance;

  dynamic_object.kinematics.has_position_covariance = det_object.kinematics.has_position_covariance;
  if (
    det_object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::UNAVAILABLE;
  } else if (
    det_object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::SIGN_UNKNOWN;
  } else if (
    det_object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::AVAILABLE;
  }
  dynamic_object.kinematics.has_twist = det_object.kinematics.has_twist;
  dynamic_object.kinematics.has_twist_covariance = det_object.kinematics.has_twist_covariance;

  // shape
  dynamic_object.shape = det_object.shape;
  dynamic_object.area = getArea(det_object.shape);

  return dynamic_object;
}

DynamicObjectList toDynamicObjectList(
  const autoware_perception_msgs::msg::DetectedObjects & det_objects, const uint channel_index)
{
  DynamicObjectList dynamic_objects;
  dynamic_objects.header = det_objects.header;
  dynamic_objects.channel_index = channel_index;
  dynamic_objects.objects.reserve(det_objects.objects.size());
  for (const auto & det_object : det_objects.objects) {
    dynamic_objects.objects.emplace_back(toDynamicObject(det_object, channel_index));
  }
  return dynamic_objects;
}

autoware_perception_msgs::msg::TrackedObject toTrackedObjectMsg(const DynamicObject & dyn_object)
{
  autoware_perception_msgs::msg::TrackedObject tracked_object;
  tracked_object.object_id = dyn_object.uuid;
  tracked_object.existence_probability = dyn_object.existence_probability;
  tracked_object.classification = dyn_object.classification;

  tracked_object.kinematics.pose_with_covariance.pose = dyn_object.pose;
  tracked_object.kinematics.pose_with_covariance.covariance = dyn_object.pose_covariance;
  tracked_object.kinematics.twist_with_covariance.twist = dyn_object.twist;
  tracked_object.kinematics.twist_with_covariance.covariance = dyn_object.twist_covariance;

  if (dyn_object.kinematics.orientation_availability == OrientationAvailability::UNAVAILABLE) {
    tracked_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
  } else if (
    dyn_object.kinematics.orientation_availability == OrientationAvailability::SIGN_UNKNOWN) {
    tracked_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
  } else if (dyn_object.kinematics.orientation_availability == OrientationAvailability::AVAILABLE) {
    tracked_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
  }
  tracked_object.kinematics.is_stationary = false;

  tracked_object.shape = dyn_object.shape;

  return tracked_object;
}

double getArea(const autoware_perception_msgs::msg::Shape & shape)
{
  switch (shape.type) {
    case autoware_perception_msgs::msg::Shape::BOUNDING_BOX:
      return shape.dimensions.x * shape.dimensions.y;
    case autoware_perception_msgs::msg::Shape::CYLINDER:
      return shape.dimensions.x * shape.dimensions.x * M_PI * 0.25;
    case autoware_perception_msgs::msg::Shape::POLYGON: {
      double area = 0.0;
      for (size_t i = 0; i < shape.footprint.points.size(); ++i) {
        size_t j = (i + 1) % shape.footprint.points.size();
        area += 0.5 * (shape.footprint.points.at(i).x * shape.footprint.points.at(j).y -
                       shape.footprint.points.at(j).x * shape.footprint.points.at(i).y);
      }
      return area;
    }
    default:
      return 0.0;
  }
}

}  // namespace types

}  // namespace autoware::multi_object_tracker
