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

#ifndef TOOLS__STATIC_POSE_HPP_
#define TOOLS__STATIC_POSE_HPP_

#include "interactive_object.hpp"

#include <rviz_common/properties/enum_property.hpp>

namespace rviz_plugins
{
class StaticInitialPoseTool : public InteractiveObjectTool
{
public:
  StaticInitialPoseTool();
  void onInitialize() override;
  [[nodiscard]] SimulatedObject createObjectMsg() const override;

private:
  rviz_common::properties::EnumProperty * class_property_;
};
}  // namespace rviz_plugins

#endif  // TOOLS__STATIC_POSE_HPP_
