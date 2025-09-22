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

#include "autoware/dummy_perception_publisher/straight_line_object_movement_plugin.hpp"

#include "autoware/dummy_perception_publisher/movement_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::dummy_perception_publisher::pluginlib
{

void StraightLineObjectMovementPlugin::initialize()
{
  set_associated_action_type(tier4_simulation_msgs::msg::DummyObject::ADD);
}

std::vector<ObjectInfo> StraightLineObjectMovementPlugin::move_objects()
{
  std::vector<ObjectInfo> obj_infos;

  for (const auto & object : objects_) {
    const auto current_time = get_node()->now();

    // Create basic ObjectInfo with dimensions and covariances
    auto obj_info = utils::MovementUtils::create_basic_object_info(object);

    // Calculate position using straight-line movement
    const auto current_pose =
      utils::MovementUtils::calculate_straight_line_position(object, current_time);

    // Update ObjectInfo with the calculated movement
    utils::MovementUtils::update_object_info_with_movement(
      obj_info, object, current_pose, current_time);

    obj_infos.push_back(obj_info);
  }
  return obj_infos;
}

}  // namespace autoware::dummy_perception_publisher::pluginlib
