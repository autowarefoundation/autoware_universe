// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__VEHICLE_SHAPE_MODEL_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__VEHICLE_SHAPE_MODEL_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/shape_model_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>

namespace autoware::multi_object_tracker
{

// Manages shape extension (width, height, polygon footprint) for VehicleTracker.
// Vehicle length is owned by BicycleMotionModel and must be supplied at export time.
//
// Data flow:
//   init()         — set width/height from first detection, clear footprint
//   updateShape()  — IIR-blend width/height each normal measure() call
//   updateFootprint() — transform and store polygon footprint
//   updateHeight() — blend height only (conditioned-update path)
//   setShape()     — called from setObjectShape() when UnstableShapeFilter commits a stable shape;
//                    returns new length for motion model sync
//   mergeFrom()    — union footprint from absorbed tracker
//   exportTo()     — write {x=vehicle_length, y, z, footprint} to output object
//   flipFootprintXY() — flip footprint 180° when yaw-limit correction fires
class VehicleShapeModel : public ShapeModelBase
{
public:
  explicit VehicleShapeModel(const object_model::ObjectModel & object_model);

  // Initialize from first detection (before motion model is ready)
  void init(const types::DynamicObject & object);

  // IIR-blend width and height from a BOUNDING_BOX measurement
  void updateShape(const types::DynamicObject & object);

  // Transform and store the polygon footprint from a measurement.
  // tracker_pose: current tracker pose after kinematic update; nullopt = constructor path
  // (direct-copy fallback when motion model not yet initialized)
  void updateFootprint(
    const types::DynamicObject & object, const rclcpp::Time & time,
    const std::optional<geometry_msgs::msg::Pose> & tracker_pose = std::nullopt);

  // Blend height only (used by conditioned-update WEAK and wheel paths)
  void updateHeight(double z_measurement);

  // Called when UnstableShapeFilter commits a new stable shape.
  // Stores new width/height/footprint; returns new vehicle length when shape is BOUNDING_BOX
  // (caller should sync motion model length with the returned value).
  std::optional<double> setShape(
    const autoware_perception_msgs::msg::Shape & shape,
    const rclcpp::Time & latest_measurement_time);

  // Union polygon footprint from an absorbed tracker.
  // footprint: absorbed tracker's stored footprint, expressed relative to src_pose.
  // src_pose: absorbed tracker's current pose.
  // winner_pose: this tracker's current pose (transform target).
  void mergeFrom(
    const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose,
    const geometry_msgs::msg::Pose & winner_pose, const rclcpp::Time & latest_measurement_time);

  // Write {x=vehicle_length, y=width, z=height, footprint} into output.shape.
  // Also updates output.area. vehicle_length comes from BicycleMotionModel::getLength().
  void exportTo(types::DynamicObject & output, double vehicle_length) const;

  // Flip all footprint points 180° around tracker center (yaw-limit correction)
  void flipFootprintXY();

  bool isFootprintValid() const { return footprint_valid_; }

private:
  double width_{0.0};
  double height_{0.0};
  geometry_msgs::msg::Polygon footprint_;
  bool footprint_valid_{false};
  rclcpp::Time last_footprint_update_time_;

  static constexpr double FOOTPRINT_TIMEOUT_S = 1.0;  // [s] footprint expiry

  object_model::ObjectModel object_model_;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__VEHICLE_SHAPE_MODEL_HPP_
