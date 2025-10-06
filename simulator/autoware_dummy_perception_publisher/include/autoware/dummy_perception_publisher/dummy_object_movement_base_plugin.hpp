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

#include <tier4_simulation_msgs/msg/dummy_object.hpp>

#include <algorithm>
#include <vector>

namespace autoware::dummy_perception_publisher::pluginlib
{
using tier4_simulation_msgs::msg::DummyObject;

class DummyObjectMovementBasePlugin
{
private:
  rclcpp::Node * node_ptr_;

protected:
  [[nodiscard]] rclcpp::Node * get_node() const { return node_ptr_; }

public:
  std::vector<DummyObject> objects_;
  uint8_t associated_action_type_{tier4_simulation_msgs::msg::DummyObject::ADD};

  explicit DummyObjectMovementBasePlugin(rclcpp::Node * node) : node_ptr_(node) {}
  virtual ~DummyObjectMovementBasePlugin() = default;
  virtual void initialize() = 0;
  virtual std::vector<ObjectInfo> move_objects() = 0;
  [[nodiscard]] std::vector<DummyObject> get_objects() const { return objects_; }
  void clear_objects() { objects_.clear(); }
  void modify_object(const DummyObject & object)
  {
    auto obj_it = std::find_if(
      objects_.begin(), objects_.end(),
      [&object](const DummyObject & obj) { return obj.id.uuid == object.id.uuid; });
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
  bool set_dummy_object(const DummyObject & object)
  {
    // Check if the action type matches
    if (object.action != associated_action_type_) {
      return false;
    }
    objects_.push_back(object);
    return true;
  }
  void set_associated_action_type(uint8_t action_type) { associated_action_type_ = action_type; }
};

}  // namespace autoware::dummy_perception_publisher::pluginlib

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__DUMMY_OBJECT_MOVEMENT_BASE_PLUGIN_HPP_
