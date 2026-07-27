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

#include "autoware/multi_object_tracker/tracker/shape_model/polygon_shape_model.hpp"

namespace autoware::multi_object_tracker
{

void PolygonShapeModel::init(const types::DynamicObject & object)
{
  update(object);
}

void PolygonShapeModel::update(const types::DynamicObject & object)
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

void PolygonShapeModel::exportTo(types::DynamicObject & output) const
{
  output.shape = assembleShapeMsg();
  output.area = area_;
}

}  // namespace autoware::multi_object_tracker
