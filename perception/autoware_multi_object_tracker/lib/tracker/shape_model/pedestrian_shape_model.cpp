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

#include "autoware/multi_object_tracker/tracker/shape_model/pedestrian_shape_model.hpp"

#include "autoware/multi_object_tracker/object_model/shapes_transform.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::multi_object_tracker
{

using Shape = autoware_perception_msgs::msg::Shape;

PedestrianShapeModel::PedestrianShapeModel(const object_model::ObjectModel & object_model)
: object_model_(object_model)
{
}

void PedestrianShapeModel::clampToLimits()
{
  length_ =
    std::clamp(length_, object_model_.size_limit.length_min, object_model_.size_limit.length_max);
  width_ =
    std::clamp(width_, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  height_ =
    std::clamp(height_, object_model_.size_limit.height_min, object_model_.size_limit.height_max);
}

void PedestrianShapeModel::init(const types::DynamicObject & object)
{
  if (object.shape.type == Shape::POLYGON) {
    // No reliable orientation at init — use model defaults
    length_ = object_model_.init_size.length;
    width_ = object_model_.init_size.width;
    height_ = object_model_.init_size.height;
  } else if (object.shape.type == Shape::CYLINDER) {
    // Normalize cylinder: use max of both axes as diameter
    const double diameter = std::max(object.shape.dimensions.x, object.shape.dimensions.y);
    length_ = diameter;
    width_ = diameter;
    height_ = object.shape.dimensions.z;
  } else {
    // BOUNDING_BOX
    length_ = object.shape.dimensions.x;
    width_ = object.shape.dimensions.y;
    height_ = object.shape.dimensions.z;
  }
  clampToLimits();
  area_ = length_ * width_;
}

bool PedestrianShapeModel::update(
  const types::DynamicObject & object, bool trust_extension, double tracker_yaw)
{
  // Model-derived sanity bounds (permissive: 0.5× min … 1.5× max)
  const double len_max = object_model_.size_limit.length_max * 1.5;
  const double len_min = object_model_.size_limit.length_min * 0.5;
  const double wid_max = object_model_.size_limit.width_max * 1.5;
  const double wid_min = object_model_.size_limit.width_min * 0.5;
  const double hgt_max = object_model_.size_limit.height_max * 1.5;
  const double hgt_min = object_model_.size_limit.height_min * 0.5;

  if (object.shape.type == Shape::BOUNDING_BOX) {
    if (
      object.shape.dimensions.x > len_max || object.shape.dimensions.x < len_min ||
      object.shape.dimensions.y > wid_max || object.shape.dimensions.y < wid_min ||
      object.shape.dimensions.z > hgt_max || object.shape.dimensions.z < hgt_min) {
      return false;
    }
    const double gain = trust_extension ? 0.5 : 0.0;
    const double gain_inv = 1.0 - gain;
    length_ = gain_inv * length_ + gain * object.shape.dimensions.x;
    width_ = gain_inv * width_ + gain * object.shape.dimensions.y;
    height_ = gain_inv * height_ + gain * object.shape.dimensions.z;

  } else if (object.shape.type == Shape::CYLINDER) {
    const double diameter = std::max(object.shape.dimensions.x, object.shape.dimensions.y);
    if (diameter > wid_max || diameter < wid_min) return false;
    if (object.shape.dimensions.z > hgt_max || object.shape.dimensions.z < hgt_min) return false;
    const double gain = trust_extension ? 0.5 : 0.0;
    const double gain_inv = 1.0 - gain;
    length_ = gain_inv * length_ + gain * diameter;
    width_ = gain_inv * width_ + gain * diameter;
    height_ = gain_inv * height_ + gain * object.shape.dimensions.z;

  } else {  // POLYGON
    // Project footprint onto tracker heading; use footprint geometry even when
    // trust_extension is false (lower gain) — footprint is a geometric measurement.
    const auto aligned = shapes::alignClusterToOrientation(object, tracker_yaw);
    if (!aligned) return false;
    const double aligned_len = aligned->shape.dimensions.x;
    const double aligned_wid = aligned->shape.dimensions.y;
    if (
      aligned_len > len_max || aligned_len < len_min || aligned_wid > wid_max ||
      aligned_wid < wid_min) {
      return false;
    }
    const double gain = trust_extension ? 0.3 : 0.15;
    const double gain_inv = 1.0 - gain;
    length_ = gain_inv * length_ + gain * aligned_len;
    width_ = gain_inv * width_ + gain * aligned_wid;
    // Height: use z-span of polygon if available
    if (object.shape.dimensions.z > 0.0) {
      height_ = gain_inv * height_ + gain * object.shape.dimensions.z;
    }
  }

  clampToLimits();
  area_ = length_ * width_;
  return true;
}

void PedestrianShapeModel::exportTo(types::DynamicObject & output) const
{
  // Always export a BOUNDING_BOX with explicit length (x) and width (y).
  output.shape.type = Shape::BOUNDING_BOX;
  output.shape.dimensions.x = length_;
  output.shape.dimensions.y = width_;
  output.shape.dimensions.z = height_;
  output.shape.footprint = footprint_;
  output.area = types::getArea(output.shape);
}

}  // namespace autoware::multi_object_tracker
