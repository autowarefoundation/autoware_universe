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

#include "autoware/multi_object_tracker/tracker/shape_model/static_extend_manager.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

namespace autoware::multi_object_tracker
{

void StaticExtendManager::init(const types::DynamicObject & object)
{
  shape_ = object.shape;
  area_ = types::getArea(shape_);
}

void StaticExtendManager::update(const types::DynamicObject & object)
{
  shape_ = object.shape;
  area_ = types::getArea(shape_);
}

void StaticExtendManager::setEgoPose(const std::optional<geometry_msgs::msg::Point> & ego_pos)
{
  ego_pos_ = ego_pos;
}

void StaticExtendManager::exportTo(types::DynamicObject & output, bool to_publish) const
{
  output.shape = shape_;
  output.area = area_;

  if (to_publish && shape_.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    types::DynamicObject converted;
    if (shapes::convertConvexHullToBoundingBox(output, converted, ego_pos_)) {
      output.shape = converted.shape;
      output.area = converted.area;
    }
  }
}

}  // namespace autoware::multi_object_tracker
