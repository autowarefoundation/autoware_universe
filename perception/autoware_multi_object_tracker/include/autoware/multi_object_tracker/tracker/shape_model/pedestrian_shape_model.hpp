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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__PEDESTRIAN_SHAPE_MODEL_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__PEDESTRIAN_SHAPE_MODEL_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/shape_model_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>

namespace autoware::multi_object_tracker
{

// Manages shape extension (length, width, height) for PedestrianTracker.
// All input shape types (BOUNDING_BOX, CYLINDER, POLYGON) are accepted.
// Internally always stores BOUNDING_BOX {length, width, height} in tracker heading frame.
// Output type is always BOUNDING_BOX with explicit length and width.
//
// The measurement footprint keeps its measured orientation: it is stored anchored at the object
// position but aligned to the global axes (never rotated to the tracker heading), which also keeps
// the float32 footprint coordinates small. At export it is re-expressed in the output object's
// frame (message position and orientation), so it is shifted to the exported pose while its global
// orientation is preserved.
//
// Data flow:
//   init()    — force all input types to internal BOUNDING_BOX; apply model-derived sanity bounds
//   update()  — 3-branch update by input type (BBOX / CYLINDER / POLYGON);
//               gains differ by type and trust_extension; a non-empty footprint is stored
//   exportTo() — assemble output as a BOUNDING_BOX filling length and width; emit the stored
//                footprint (re-expressed in the output frame) while it is still fresh
class PedestrianShapeModel : public ShapeModelBase
{
public:
  explicit PedestrianShapeModel(const object_model::ObjectModel & object_model);

  // Initialize shape from first detection; converts any input type to internal BOUNDING_BOX
  void init(const types::DynamicObject & object);

  // Update shape from new measurement.
  // trust_extension: whether the channel provides reliable size measurements.
  // tracker_yaw: current tracker heading; required for POLYGON branch.
  // time: measurement time; stamps the stored footprint so it can expire at export.
  // A non-empty measurement footprint is stored in the global frame (measured orientation kept).
  // Returns false if update was rejected (implausible dimensions).
  bool update(
    const types::DynamicObject & object, bool trust_extension, double tracker_yaw,
    const rclcpp::Time & time);

  // Write shape into output object.
  // Output type is always BOUNDING_BOX {length, width, height}. A stored footprint is re-expressed
  // in the output object's local frame and emitted only while it is still fresh (see
  // FOOTPRINT_TIMEOUT_S); otherwise it is cleared.
  void exportTo(types::DynamicObject & output) const;

private:
  // length_, width_, height_, footprint_, footprint_valid_, area_ live in ShapeModelBase.
  object_model::ObjectModel object_model_;
  rclcpp::Time last_footprint_update_time_;

  static constexpr double FOOTPRINT_TIMEOUT_S = 1.0;  // [s] footprint expiry

  void clampToLimits();
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__PEDESTRIAN_SHAPE_MODEL_HPP_
