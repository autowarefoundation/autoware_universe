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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__POLYGON_SHAPE_MODEL_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__POLYGON_SHAPE_MODEL_HPP_

#include "autoware/multi_object_tracker/tracker/shape_model/shape_model_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <autoware_perception_msgs/msg/shape.hpp>

namespace autoware::multi_object_tracker
{

// Passthrough shape manager for PolygonTracker.
// Stores the latest shape (decomposed into the base fields) and exports it unchanged.
class PolygonShapeModel : public ShapeModelBase
{
public:
  PolygonShapeModel() = default;

  // Store shape from initial detection
  void init(const types::DynamicObject & object);

  // Replace stored shape with new measurement (direct copy)
  void update(const types::DynamicObject & object);

  // Write stored shape to output
  void exportTo(types::DynamicObject & output) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__POLYGON_SHAPE_MODEL_HPP_
