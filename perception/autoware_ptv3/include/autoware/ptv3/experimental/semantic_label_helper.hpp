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

#ifndef AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HELPER_HPP_
#define AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HELPER_HPP_

#include "autoware/ptv3/experimental/semantic_label.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <optional>
#include <string_view>

namespace autoware::ptv3::experimental
{
using autoware_perception_msgs::msg::ObjectClassification;
using ObjectLabel = ObjectClassification::_label_type;

/**
 * @brief Get the string representation of a semantic label.
 * @param label The semantic label to convert.
 * @return String view of the label name (e.g., "CAR", "HAZARD", "STRUCTURE").
 */
constexpr std::string_view to_string(SemanticLabel label) noexcept
{
  switch (label) {
    case SemanticLabel::CAR:
      return "CAR";
    case SemanticLabel::TRUCK:
      return "TRUCK";
    case SemanticLabel::BUS:
      return "BUS";
    case SemanticLabel::MOTORCYCLE:
      return "MOTORCYCLE";
    case SemanticLabel::BICYCLE:
      return "BICYCLE";
    case SemanticLabel::PEDESTRIAN:
      return "PEDESTRIAN";
    case SemanticLabel::ANIMAL:
      return "ANIMAL";
    case SemanticLabel::HAZARD:
      return "HAZARD";
    case SemanticLabel::GROUND:
      return "GROUND";
    case SemanticLabel::STRUCTURE:
      return "STRUCTURE";
    case SemanticLabel::VEGETATION:
      return "VEGETATION";
    case SemanticLabel::NOISE:
      return "NOISE";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Convert a semantic label to ObjectClassification label type where applicable.
 * @param label The semantic label to convert.
 * @return The ObjectClassification label value, or std::nullopt for non-object labels.
 */
inline std::optional<ObjectLabel> try_into_object(SemanticLabel label) noexcept
{
  switch (label) {
    case SemanticLabel::CAR:
      return ObjectClassification::CAR;
    case SemanticLabel::TRUCK:
      return ObjectClassification::TRUCK;
    case SemanticLabel::BUS:
      return ObjectClassification::BUS;
    case SemanticLabel::MOTORCYCLE:
      return ObjectClassification::MOTORCYCLE;
    case SemanticLabel::BICYCLE:
      return ObjectClassification::BICYCLE;
    case SemanticLabel::PEDESTRIAN:
      return ObjectClassification::PEDESTRIAN;
    case SemanticLabel::ANIMAL:
      return ObjectClassification::ANIMAL;
    case SemanticLabel::HAZARD:
      return ObjectClassification::HAZARD;
    case SemanticLabel::GROUND:
    case SemanticLabel::STRUCTURE:
    case SemanticLabel::VEGETATION:
    case SemanticLabel::NOISE:
      return std::nullopt;
    default:
      return std::nullopt;
  }
}

/**
 * @brief Convert an ObjectClassification label to its semantic label.
 * @param label The ObjectClassification label value.
 * @return The corresponding SemanticLabel, or std::nullopt if not mapped.
 */
inline std::optional<SemanticLabel> try_into_semantic(ObjectLabel label) noexcept
{
  switch (label) {
    case ObjectClassification::CAR:
      return SemanticLabel::CAR;
    case ObjectClassification::TRUCK:
      return SemanticLabel::TRUCK;
    case ObjectClassification::BUS:
      return SemanticLabel::BUS;
    case ObjectClassification::TRAILER:
      return SemanticLabel::TRUCK;
    case ObjectClassification::MOTORCYCLE:
      return SemanticLabel::MOTORCYCLE;
    case ObjectClassification::BICYCLE:
      return SemanticLabel::BICYCLE;
    case ObjectClassification::PEDESTRIAN:
      return SemanticLabel::PEDESTRIAN;
    case ObjectClassification::ANIMAL:
      return SemanticLabel::ANIMAL;
    case ObjectClassification::HAZARD:
      return SemanticLabel::HAZARD;
    default:
      return std::nullopt;
  }
}

/**
 * @brief Check whether a semantic label is object-compatible.
 * @param label The semantic label to check.
 * @return true if the label is an object class, false for environment/non-object labels
 *         (GROUND, STRUCTURE, VEGETATION, NOISE).
 */
inline bool is_object_compatible(SemanticLabel label) noexcept
{
  return try_into_object(label).has_value();
}

}  // namespace autoware::ptv3::experimental

#endif  // AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HELPER_HPP_
