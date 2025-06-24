// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_TRACKED_OBJECT_MERGER_NODE_HPP_
#define AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_TRACKED_OBJECT_MERGER_NODE_HPP_

#include "autoware/simple_object_merger/simple_object_merger_base.hpp"
#include "autoware_utils/ros/transform_listener.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>
#include <unordered_map>

namespace autoware::simple_object_merger
{
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

struct UUIDMapping
{
  int pass_through_node_id = -1;
  std::unordered_map<int, unique_identifier_msgs::msg::UUID> replace_map;
  rclcpp::Time last_seen;
  bool is_mapped_uuid = false;
};

class SimpleTrackedObjectMergerNode
: public SimpleObjectMergerBase<autoware_perception_msgs::msg::TrackedObjects>
{
public:
  explicit SimpleTrackedObjectMergerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Duration uuid_mapping_cleanup_threshold_ = rclcpp::Duration::from_seconds(30.0);
  std::unordered_map<std::string, UUIDMapping> uuid_mapper_;

  void approximateMerger(
    const TrackedObjects::ConstSharedPtr & object_msg0,
    const TrackedObjects::ConstSharedPtr & object_msg1) override;
  void onTimer() override;

  void mapUUID(TrackedObject & object, const int & node_id);
  void cleanupUUIDMap();
};

}  // namespace autoware::simple_object_merger

#endif  // AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_TRACKED_OBJECT_MERGER_NODE_HPP_
