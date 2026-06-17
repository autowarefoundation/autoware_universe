// Copyright 2020 TIER IV, Inc.
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

#include "autoware/multi_object_tracker/tracker/trackers/multiple_vehicle_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"

namespace autoware::multi_object_tracker
{
MultipleVehicleTracker::MultipleVehicleTracker(
  const rclcpp::Time & time, const types::DynamicObject & object)
: Tracker(time, object),
  normal_vehicle_tracker_(object_model::normal_vehicle, time, object),
  big_vehicle_tracker_(object_model::big_vehicle, time, object)
{
  tracker_type_ = TrackerType::MULTIPLE_VEHICLE;
}

bool MultipleVehicleTracker::predict(const rclcpp::Time & time)
{
  big_vehicle_tracker_.predict(time);
  normal_vehicle_tracker_.predict(time);
  return true;
}

bool MultipleVehicleTracker::measure(
  const types::DynamicObject & object, const rclcpp::Time & time,
  const types::InputChannel & channel_info)
{
  big_vehicle_tracker_.measure(object, time, channel_info);
  normal_vehicle_tracker_.measure(object, time, channel_info);
  big_vehicle_tracker_.setLatestMeasurementTime(time);
  normal_vehicle_tracker_.setLatestMeasurementTime(time);

  return true;
}

bool MultipleVehicleTracker::conditionedUpdate(
  const types::DynamicObject & measurement, const types::DynamicObject & prediction,
  const autoware_perception_msgs::msg::Shape & tracker_shape, const rclcpp::Time & measurement_time,
  const types::InputChannel & channel_info)
{
  big_vehicle_tracker_.conditionedUpdate(
    measurement, prediction, tracker_shape, measurement_time, channel_info);
  normal_vehicle_tracker_.conditionedUpdate(
    measurement, prediction, tracker_shape, measurement_time, channel_info);
  big_vehicle_tracker_.setLatestMeasurementTime(measurement_time);
  normal_vehicle_tracker_.setLatestMeasurementTime(measurement_time);

  return true;
}

void MultipleVehicleTracker::setObjectShape(const autoware_perception_msgs::msg::Shape & shape)
{
  big_vehicle_tracker_.setObjectShape(shape);
  normal_vehicle_tracker_.setObjectShape(shape);
}

void MultipleVehicleTracker::mergeFootprintFrom(
  const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose,
  const geometry_msgs::msg::Pose & dst_pose)
{
  big_vehicle_tracker_.mergeFootprintFrom(footprint, src_pose, dst_pose);
  normal_vehicle_tracker_.mergeFootprintFrom(footprint, src_pose, dst_pose);
}

bool MultipleVehicleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object, const bool to_publish) const
{
  activeInner().getTrackedObject(time, object, to_publish);
  object.uuid = uuid_;
  return true;
}

void MultipleVehicleTracker::setOrientationAvailability(
  const types::OrientationAvailability & orientation_availability)
{
  normal_vehicle_tracker_.setOrientationAvailability(orientation_availability);
  big_vehicle_tracker_.setOrientationAvailability(orientation_availability);
}

}  // namespace autoware::multi_object_tracker
