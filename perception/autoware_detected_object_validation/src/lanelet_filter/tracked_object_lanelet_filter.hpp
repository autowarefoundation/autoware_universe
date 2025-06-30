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

#ifndef LANELET_FILTER__TRACKED_OBJECT_LANELET_FILTER_HPP_
#define LANELET_FILTER__TRACKED_OBJECT_LANELET_FILTER_HPP_

#include "lanelet_filter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{

class TrackedObjectLaneletFilterNode
: public ObjectLaneletFilterBase<
    autoware_perception_msgs::msg::TrackedObjects, autoware_perception_msgs::msg::TrackedObject>
{
public:
  explicit TrackedObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options);
};

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation

#endif  // LANELET_FILTER__TRACKED_OBJECT_LANELET_FILTER_HPP_
