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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__PEDESTRIAN_AND_BICYCLE_TRACKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__PEDESTRIAN_AND_BICYCLE_TRACKER_HPP_

#include "autoware/multi_object_tracker/tracker/trackers/pedestrian_tracker.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/tracker_base.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/vehicle_tracker.hpp"
#include "autoware/multi_object_tracker/types.hpp"

namespace autoware::multi_object_tracker
{

class PedestrianAndBicycleTracker : public Tracker
{
private:
  PedestrianTracker pedestrian_tracker_;
  VehicleTracker bicycle_tracker_;

public:
  PedestrianAndBicycleTracker(const rclcpp::Time & time, const types::DynamicObject & object);

  types::TrackerType getTrackerType() const override
  {
    return types::TrackerType::PEDESTRIAN_AND_BICYCLE;
  }

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const types::DynamicObject & object, const rclcpp::Time & time,
    const types::InputChannel & channel_info) override;
  bool getTrackedObject(
    const rclcpp::Time & time, types::DynamicObject & object,
    const bool to_publish = false) const override;
  void setOrientationAvailability(
    const types::OrientationAvailability & orientation_availability) override;

  // Composite tracker: both inner trackers are kept in sync by measure(), so shape writes forward
  // to BOTH. getShapeModel() returns the active inner (selected by label, same as
  // getTrackedObject).
  ShapeModelBase & getShapeModel() override { return activeInner().getShapeModel(); }
  const ShapeModelBase & getShapeModel() const override { return activeInner().getShapeModel(); }
  void assembleShapeTo(types::DynamicObject & output, bool to_publish) const override
  {
    activeInner().assembleShapeTo(output, to_publish);
  }
  void setObjectShape(const autoware_perception_msgs::msg::Shape & shape) override
  {
    pedestrian_tracker_.setObjectShape(shape);
    bicycle_tracker_.setObjectShape(shape);
  }
  void mergeFootprintFrom(
    const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose,
    const geometry_msgs::msg::Pose & dst_pose) override
  {
    pedestrian_tracker_.mergeFootprintFrom(footprint, src_pose, dst_pose);
    bicycle_tracker_.mergeFootprintFrom(footprint, src_pose, dst_pose);
  }

  virtual ~PedestrianAndBicycleTracker() {}

private:
  // Inner tracker that backs the published object for the current highest-prob label.
  Tracker & activeInner()
  {
    if (getHighestProbLabel() == classes::Label::PEDESTRIAN) return pedestrian_tracker_;
    return bicycle_tracker_;
  }
  const Tracker & activeInner() const
  {
    if (getHighestProbLabel() == classes::Label::PEDESTRIAN) return pedestrian_tracker_;
    return bicycle_tracker_;
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__PEDESTRIAN_AND_BICYCLE_TRACKER_HPP_
