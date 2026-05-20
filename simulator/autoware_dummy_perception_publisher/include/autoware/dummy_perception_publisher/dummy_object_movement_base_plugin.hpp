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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__DUMMY_OBJECT_MOVEMENT_BASE_PLUGIN_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__DUMMY_OBJECT_MOVEMENT_BASE_PLUGIN_HPP_

#include "autoware/dummy_perception_publisher/object_info.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_simulation_msgs/msg/simulated_object.hpp>

#include <algorithm>
#include <vector>

namespace autoware::dummy_perception_publisher::pluginlib
{
using autoware_simulation_msgs::msg::SimulatedObject;

class DummyObjectMovementBasePlugin
{
private:
  rclcpp::Node * node_ptr_;

protected:
  [[nodiscard]] rclcpp::Node * get_node() const { return node_ptr_; }

public:
  std::vector<SimulatedObject> objects_;
  uint8_t associated_movement_model_{autoware_simulation_msgs::msg::SimulatedObject::STRAIGHT_LINE};

  explicit DummyObjectMovementBasePlugin(rclcpp::Node * node) : node_ptr_(node) {}
  virtual ~DummyObjectMovementBasePlugin() = default;
  virtual void initialize() = 0;
  virtual std::vector<ObjectInfo> move_objects() = 0;
  [[nodiscard]] std::vector<SimulatedObject> get_objects() const { return objects_; }
  void clear_objects() { objects_.clear(); }
  void modify_object(const SimulatedObject & object)
  {
    auto obj_it = std::find_if(
      objects_.begin(), objects_.end(),
      [&object](const SimulatedObject & obj) { return obj.id.uuid == object.id.uuid; });
    if (obj_it != objects_.end()) {
      *obj_it = object;
    }
  }
  void delete_object(const unique_identifier_msgs::msg::UUID & id)
  {
    for (size_t i = 0; i < objects_.size(); ++i) {
      if (objects_.at(i).id.uuid == id.uuid) {
        objects_.erase(objects_.begin() + i);
        break;
      }
    }
  }
  bool set_simulated_object(const SimulatedObject & object)
  {
    // Check if the movement model matches
    if (object.movement_model != associated_movement_model_) {
      return false;
    }
    objects_.push_back(object);
    return true;
  }
  void set_associated_movement_model(uint8_t movement_model)
  {
    associated_movement_model_ = movement_model;
  }
};

}  // namespace autoware::dummy_perception_publisher::pluginlib

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__DUMMY_OBJECT_MOVEMENT_BASE_PLUGIN_HPP_
