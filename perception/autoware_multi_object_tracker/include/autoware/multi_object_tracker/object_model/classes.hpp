// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__CLASSES_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__CLASSES_HPP_

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace autoware::multi_object_tracker::object_model
{

inline autoware_perception_msgs::msg::ObjectClassification getHighestProbClassification(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications)
{
  if (classifications.empty()) {
    return autoware_perception_msgs::msg::ObjectClassification{};
  }
  return *std::max_element(
    classifications.begin(), classifications.end(),
    [](const auto & a, const auto & b) { return a.probability < b.probability; });
}

inline std::uint8_t getHighestProbLabel(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications)
{
  return getHighestProbClassification(classifications).label;
}

}  // namespace autoware::multi_object_tracker::object_model

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__CLASSES_HPP_
