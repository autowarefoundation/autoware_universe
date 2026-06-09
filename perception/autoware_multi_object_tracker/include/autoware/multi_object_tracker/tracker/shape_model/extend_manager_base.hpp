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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__EXTEND_MANAGER_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__EXTEND_MANAGER_BASE_HPP_

namespace autoware::multi_object_tracker
{

// Base class for all extend (shape/size/footprint) managers.
// Each concrete tracker holds a typed extend manager as a member alongside its motion model.
// The tracker's measure() calls update(); getTrackedObject() calls exportTo().
// Because exportTo() signatures differ per tracker type (vehicle needs the length from the bicycle
// model; pedestrian's exportTo takes no extra parameter), exportTo() is NOT declared here — each
// concrete class exposes its own typed exportTo() that the tracker calls directly.
class ExtendManagerBase
{
public:
  virtual ~ExtendManagerBase() = default;

  double getArea() const { return area_; }

protected:
  double area_{0.0};
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__SHAPE_MODEL__EXTEND_MANAGER_BASE_HPP_
