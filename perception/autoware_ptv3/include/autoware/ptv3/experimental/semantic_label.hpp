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

#ifndef AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HPP_
#define AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HPP_

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <cstdint>
#include <optional>
#include <string_view>

namespace autoware::ptv3::experimental
{
using autoware_perception_msgs::msg::ObjectClassification;
using ObjectLabel = ObjectClassification::_label_type;

/// @brief Semantic labels for point cloud segmentation.
/// @details
/// Consolidated labels representing different point cloud categories:
/// - CAR, TRUCK, BUS, BICYCLE, PEDESTRIAN: Object classes
/// - HAZARD: Small hazardous objects (e.g., debris, barriers)
/// - GROUND: Drivable ground surface
/// - STRUCTURE: Non-drivable structures (e.g., buildings, walls)
/// - VEGETATION: Vegetation (trees, bushes)
/// - NOISE: Noise points and outliers
enum class SemanticLabel : std::uint8_t {
  CAR = 0,
  TRUCK = 1,
  BUS = 2,
  BICYCLE = 3,
  PEDESTRIAN = 4,
  HAZARD = 5,
  GROUND = 6,
  STRUCTURE = 7,
  VEGETATION = 8,
  NOISE = 9,
};

/// @brief Get the string representation of a semantic label.
/// @param label The semantic label to convert.
/// @return String view of the label name (e.g., "CAR", "HAZARD", "STRUCTURE").
constexpr std::string_view to_string(SemanticLabel label) noexcept
{
  switch (label) {
    case SemanticLabel::CAR:
      return "CAR";
    case SemanticLabel::TRUCK:
      return "TRUCK";
    case SemanticLabel::BUS:
      return "BUS";
    case SemanticLabel::BICYCLE:
      return "BICYCLE";
    case SemanticLabel::PEDESTRIAN:
      return "PEDESTRIAN";
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

/// @brief Convert a semantic label to ObjectClassification label type where applicable.
/// @param label The semantic label to convert.
/// @return The ObjectClassification label value (as uint8_t), or std::nullopt if the label
///         does not map to an object class (e.g., GROUND, STRUCTURE, VEGETATION, NOISE).
/// @details
/// Mapping:
/// - CAR -> 1 (ObjectClassification::CAR)
/// - TRUCK -> 2 (ObjectClassification::TRUCK)
/// - BUS -> 3 (ObjectClassification::BUS)
/// - BICYCLE -> 6 (ObjectClassification::BICYCLE)
/// - PEDESTRIAN -> 7 (ObjectClassification::PEDESTRIAN)
/// - HAZARD -> 9 (ObjectClassification::HAZARD)
/// - GROUND, STRUCTURE, VEGETATION, NOISE -> std::nullopt (non-object labels)
constexpr std::optional<ObjectLabel> try_into_object(SemanticLabel label) noexcept
{
  switch (label) {
    case SemanticLabel::CAR:
      return ObjectClassification::CAR;
    case SemanticLabel::TRUCK:
      return ObjectClassification::TRUCK;
    case SemanticLabel::BUS:
      return ObjectClassification::BUS;
    case SemanticLabel::BICYCLE:
      return ObjectClassification::BICYCLE;
    case SemanticLabel::PEDESTRIAN:
      return ObjectClassification::PEDESTRIAN;
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

/// @brief Convert an ObjectClassification label to its most likely semantic label.
/// @param label The ObjectClassification label value (uint8_t).
/// @return The corresponding SemanticLabel, or std::nullopt if the object class cannot be mapped.
/// @details
/// Reverse mapping (non-unique, maps to most common PTv3 output):
/// - 1 (CAR) -> CAR
/// - 2 (TRUCK) -> TRUCK
/// - 3 (BUS) -> BUS
/// - 6 (BICYCLE) -> BICYCLE
/// - 7 (PEDESTRIAN) -> PEDESTRIAN
/// - 9 (HAZARD) -> HAZARD
/// - Other ObjectClassification values -> std::nullopt
constexpr std::optional<SemanticLabel> try_into_semantic(std::uint8_t label) noexcept
{
  switch (label) {
    case ObjectClassification::CAR:
      return SemanticLabel::CAR;
    case ObjectClassification::TRUCK:
      return SemanticLabel::TRUCK;
    case ObjectClassification::BUS:
      return SemanticLabel::BUS;
    case ObjectClassification::BICYCLE:
      return SemanticLabel::BICYCLE;
    case ObjectClassification::PEDESTRIAN:
      return SemanticLabel::PEDESTRIAN;
    case ObjectClassification::HAZARD:
      return SemanticLabel::HAZARD;
    default:
      return std::nullopt;
  }
}

/// @brief Check whether a semantic label is compatible with ObjectClassification.
/// @param label The semantic label to check.
/// @return true if the label represents an object class that can be converted to
/// ObjectClassification,
///         false for environment/non-object labels (GROUND, STRUCTURE, VEGETATION, NOISE).
constexpr bool is_object_compatible(SemanticLabel label) noexcept
{
  return try_into_object(label).has_value();
}

}  // namespace autoware::ptv3::experimental

#endif  // AUTOWARE__PTV3__EXPERIMENTAL__SEMANTIC_LABEL_HPP_
