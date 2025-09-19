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

#include "detected_object_lanelet_filter.hpp"

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{

DetectedObjectLaneletFilterNode::DetectedObjectLaneletFilterNode(
  const rclcpp::NodeOptions & node_options)
: ObjectLaneletFilterBase<
    autoware_perception_msgs::msg::DetectedObjects, autoware_perception_msgs::msg::DetectedObject>(
    "object_lanelet_filter_node", node_options)
{
}

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::detected_object_validation::lanelet_filter::DetectedObjectLaneletFilterNode)
