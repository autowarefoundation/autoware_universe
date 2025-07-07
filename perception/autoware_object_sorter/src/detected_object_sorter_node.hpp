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

#ifndef DETECTED_OBJECT_SORTER_NODE_HPP_
#define DETECTED_OBJECT_SORTER_NODE_HPP_

#include "object_sorter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>

namespace autoware::object_sorter
{

class DetectedObjectSorterNode
: public ObjectSorterBase<autoware_perception_msgs::msg::DetectedObjects>
{
public:
  explicit DetectedObjectSorterNode(const rclcpp::NodeOptions & node_options);
};

}  // namespace autoware::object_sorter

#endif  // DETECTED_OBJECT_SORTER_NODE_HPP_
