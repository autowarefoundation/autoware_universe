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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__SHAPE_MODEL_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__SHAPE_MODEL_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
#include <optional>

namespace autoware::multi_object_tracker
{

// Base class for all shape models.
//
// The shape state (type, dimensions, footprint, area) lives here — it is the single source of
// truth for a tracker's extension. Each concrete tracker holds a typed shape model alongside its
// motion model. Vehicle length is the one exception: it is owned by BicycleMotionModel and supplied
// at export time, so it is NOT stored as the authoritative value here (VehicleShapeModel keeps the
// base length_ field unused).
//
// Write paths:
//   setShape()  — apply a full Shape message (e.g. from overlap-merge); default decomposes into the
//                 base fields. VehicleShapeModel overrides to also drive the motion-model length.
//   mergeFrom() — union an absorbed tracker's footprint; default no-op (only VehicleShapeModel).
//   setEgoPose()— provide ego position; default no-op (only StaticShapeModel, for publish-time
//                 POLYGON -> BOUNDING_BOX conversion).
//
// Read path:
//   each concrete model exposes a typed exportTo(...) that assembles the output shape. The shared
//   assembleShapeMsg() helper reconstructs a Shape message verbatim from the base fields.
class ShapeModelBase
{
public:
  virtual ~ShapeModelBase() = default;

  // --- read accessors (base fields) ---
  bool isFootprintValid() const { return footprint_valid_; }

  // --- write interface (sensible defaults; each model overrides what it needs) ---

  // Apply a shape message. Default decomposes it into the base fields and returns nullopt.
  // VehicleShapeModel overrides to clamp width/height and return a new length for the motion model.
  virtual std::optional<double> setShape(
    const autoware_perception_msgs::msg::Shape & shape, const rclcpp::Time & time);

  // Union an absorbed tracker's footprint into this model. Default no-op.
  virtual void mergeFrom(
    const geometry_msgs::msg::Polygon & /*footprint*/,
    const geometry_msgs::msg::Pose & /*src_pose*/, const geometry_msgs::msg::Pose & /*dst_pose*/,
    const rclcpp::Time & /*time*/)
  {
  }

  // Provide ego position for publish-time shape conversion. Default no-op.
  virtual void setEgoPose(const std::optional<geometry_msgs::msg::Point> & /*ego_pos*/) {}

protected:
  // Reconstruct a Shape message from the base fields (type, dimensions, footprint).
  autoware_perception_msgs::msg::Shape assembleShapeMsg() const;

  uint8_t shape_type_{0};
  double length_{0.0};
  double width_{0.0};
  double height_{0.0};
  double area_{0.0};
  geometry_msgs::msg::Polygon footprint_;
  bool footprint_valid_{false};
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__SHAPE_MODEL_BASE_HPP_
