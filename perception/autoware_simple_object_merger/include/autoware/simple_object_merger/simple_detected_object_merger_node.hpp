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

#ifndef AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_DETECTED_OBJECT_MERGER_NODE_HPP_
#define AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_DETECTED_OBJECT_MERGER_NODE_HPP_

#include "autoware/simple_object_merger/simple_object_merger_base.hpp"
#include "autoware_utils/ros/transform_listener.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

namespace autoware::simple_object_merger
{
using autoware_perception_msgs::msg::DetectedObjects;

class SimpleDetectedObjectMergerNode
: public SimpleObjectMergerBase<autoware_perception_msgs::msg::DetectedObjects>
{
public:
  explicit SimpleDetectedObjectMergerNode(const rclcpp::NodeOptions & node_options);

private:
  void approximateMerger(
    const DetectedObjects::ConstSharedPtr & object_msg0,
    const DetectedObjects::ConstSharedPtr & object_msg1) override;

  void onTimer() override;
};

}  // namespace autoware::simple_object_merger

#endif  // AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_DETECTED_OBJECT_MERGER_NODE_HPP_
