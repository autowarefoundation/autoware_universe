// Copyright 2025 Tier IV, Inc.
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

#ifndef TOOLS__PREDICTED_OBJECT_HPP_
#define TOOLS__PREDICTED_OBJECT_HPP_

#include "interactive_object.hpp"

namespace rviz_plugins
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using tier4_simulation_msgs::msg::DummyObject;

class PredictedCarPoseTool : public InteractiveObjectTool
{
public:
  PredictedCarPoseTool();
  void onInitialize() override;
  [[nodiscard]] DummyObject createObjectMsg() const override;
};

class PredictedPedestrianPoseTool : public InteractiveObjectTool
{
public:
  PredictedPedestrianPoseTool();
  void onInitialize() override;
  [[nodiscard]] DummyObject createObjectMsg() const override;
};

class PredictedBusPoseTool : public InteractiveObjectTool
{
public:
  PredictedBusPoseTool();
  void onInitialize() override;
  [[nodiscard]] DummyObject createObjectMsg() const override;
};

class PredictedBikePoseTool : public InteractiveObjectTool
{
public:
  PredictedBikePoseTool();
  void onInitialize() override;
  [[nodiscard]] DummyObject createObjectMsg() const override;

private:
  rviz_common::properties::EnumProperty * label_;
};

}  // namespace rviz_plugins

#endif  // TOOLS__PREDICTED_OBJECT_HPP_