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

#include "autoware/multi_object_tracker/tracker/shape_model/static_shape_model.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

namespace autoware::multi_object_tracker
{

void StaticShapeModel::init(const types::DynamicObject & object)
{
  update(object);
}

void StaticShapeModel::update(const types::DynamicObject & object)
{
  const auto & shape = object.shape;
  shape_type_ = shape.type;
  length_ = shape.dimensions.x;
  width_ = shape.dimensions.y;
  height_ = shape.dimensions.z;
  footprint_ = shape.footprint;
  footprint_valid_ = !shape.footprint.points.empty();
  area_ = types::getArea(shape);
}

void StaticShapeModel::setEgoPose(const std::optional<geometry_msgs::msg::Point> & ego_pos)
{
  ego_pos_ = ego_pos;
}

void StaticShapeModel::exportTo(types::DynamicObject & output, bool to_publish) const
{
  output.shape = assembleShapeMsg();
  output.area = area_;

  if (to_publish && shape_type_ == autoware_perception_msgs::msg::Shape::POLYGON) {
    types::DynamicObject converted;
    if (shapes::convertConvexHullToBoundingBox(output, converted, ego_pos_)) {
      output.shape = converted.shape;
      output.area = converted.area;
    }
  }
}

}  // namespace autoware::multi_object_tracker
