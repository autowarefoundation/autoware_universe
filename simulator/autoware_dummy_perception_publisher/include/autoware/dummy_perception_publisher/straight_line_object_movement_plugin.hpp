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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__STRAIGHT_LINE_OBJECT_MOVEMENT_PLUGIN_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__STRAIGHT_LINE_OBJECT_MOVEMENT_PLUGIN_HPP_

#include "autoware/dummy_perception_publisher/dummy_object_movement_base_plugin.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::dummy_perception_publisher::pluginlib
{

class StraightLineObjectMovementPlugin : public DummyObjectMovementBasePlugin
{
public:
  explicit StraightLineObjectMovementPlugin(rclcpp::Node * node)
  : DummyObjectMovementBasePlugin(node)
  {
    initialize();
  }
  void initialize() override;
  std::vector<ObjectInfo> move_objects() override;
};

}  // namespace autoware::dummy_perception_publisher::pluginlib

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__STRAIGHT_LINE_OBJECT_MOVEMENT_PLUGIN_HPP_
