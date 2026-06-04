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
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware::multi_object_tracker::classes
{

using ClassificationMsg = autoware_perception_msgs::msg::ObjectClassification;

enum class Label : std::uint8_t {
  UNKNOWN = ClassificationMsg::UNKNOWN,
  CAR = ClassificationMsg::CAR,
  TRUCK = ClassificationMsg::TRUCK,
  BUS = ClassificationMsg::BUS,
  TRAILER = ClassificationMsg::TRAILER,
  MOTORCYCLE = ClassificationMsg::MOTORCYCLE,
  BICYCLE = ClassificationMsg::BICYCLE,
  PEDESTRIAN = ClassificationMsg::PEDESTRIAN,
  ANIMAL = ClassificationMsg::ANIMAL,
  HAZARD = ClassificationMsg::HAZARD,
  OVER_DRIVABLE = ClassificationMsg::OVER_DRIVABLE,
  UNDER_DRIVABLE = ClassificationMsg::UNDER_DRIVABLE,
};

struct Classification
{
  Label label{Label::UNKNOWN};
  float probability{0.0F};
};

constexpr int NUM_TRACKED_LABELS = 8;

inline constexpr std::array<Label, NUM_TRACKED_LABELS> TRACKED_LABELS = {
  Label::UNKNOWN, Label::CAR,        Label::TRUCK,   Label::BUS,
  Label::TRAILER, Label::MOTORCYCLE, Label::BICYCLE, Label::PEDESTRIAN};

inline const std::array<Label, NUM_TRACKED_LABELS> & trackedLabels()
{
  return TRACKED_LABELS;
}

inline std::string toString(const Label label)
{
  switch (label) {
    case Label::UNKNOWN:
      return "unknown";
    case Label::CAR:
      return "car";
    case Label::TRUCK:
      return "truck";
    case Label::BUS:
      return "bus";
    case Label::TRAILER:
      return "trailer";
    case Label::MOTORCYCLE:
      return "motorcycle";
    case Label::BICYCLE:
      return "bicycle";
    case Label::PEDESTRIAN:
      return "pedestrian";
    case Label::ANIMAL:
      return "animal";
    case Label::HAZARD:
      return "hazard";
    case Label::OVER_DRIVABLE:
      return "over_drivable";
    case Label::UNDER_DRIVABLE:
      return "under_drivable";
    default:
      return "unknown";
  }
}

inline std::optional<Label> toLabel(const std::string & label_name)
{
  if (label_name == "unknown") return Label::UNKNOWN;
  if (label_name == "car") return Label::CAR;
  if (label_name == "truck") return Label::TRUCK;
  if (label_name == "bus") return Label::BUS;
  if (label_name == "trailer") return Label::TRAILER;
  if (label_name == "motorcycle") return Label::MOTORCYCLE;
  if (label_name == "bicycle") return Label::BICYCLE;
  if (label_name == "pedestrian") return Label::PEDESTRIAN;
  if (label_name == "animal") return Label::ANIMAL;
  if (label_name == "hazard") return Label::HAZARD;
  if (label_name == "over_drivable") return Label::OVER_DRIVABLE;
  if (label_name == "under_drivable") return Label::UNDER_DRIVABLE;
  return std::nullopt;
}

constexpr std::uint8_t toMsgLabel(const Label label)
{
  return static_cast<std::underlying_type_t<Label>>(label);
}

constexpr Label toLabel(const std::uint8_t label)
{
  switch (label) {
    case ClassificationMsg::CAR:
      return Label::CAR;
    case ClassificationMsg::TRUCK:
      return Label::TRUCK;
    case ClassificationMsg::BUS:
      return Label::BUS;
    case ClassificationMsg::TRAILER:
      return Label::TRAILER;
    case ClassificationMsg::MOTORCYCLE:
      return Label::MOTORCYCLE;
    case ClassificationMsg::BICYCLE:
      return Label::BICYCLE;
    case ClassificationMsg::PEDESTRIAN:
      return Label::PEDESTRIAN;
    case ClassificationMsg::ANIMAL:
      return Label::ANIMAL;
    case ClassificationMsg::HAZARD:
      return Label::HAZARD;
    case ClassificationMsg::OVER_DRIVABLE:
      return Label::OVER_DRIVABLE;
    case ClassificationMsg::UNDER_DRIVABLE:
      return Label::UNDER_DRIVABLE;
    case ClassificationMsg::UNKNOWN:
    default:
      return Label::UNKNOWN;
  }
}

inline Classification toClassification(const ClassificationMsg & classification)
{
  return Classification{toLabel(classification.label), classification.probability};
}

inline std::vector<Classification> toClassifications(
  const std::vector<ClassificationMsg> & classifications)
{
  std::vector<Classification> converted;
  converted.reserve(classifications.size());
  for (const auto & classification : classifications) {
    converted.push_back(toClassification(classification));
  }
  return converted;
}

inline ClassificationMsg toClassificationMsg(const Classification & classification)
{
  ClassificationMsg msg;
  msg.label = toMsgLabel(classification.label);
  msg.probability = classification.probability;
  return msg;
}

inline std::vector<ClassificationMsg> toClassificationMsgs(
  const std::vector<Classification> & classifications)
{
  std::vector<ClassificationMsg> converted;
  converted.reserve(classifications.size());
  for (const auto & classification : classifications) {
    converted.push_back(toClassificationMsg(classification));
  }
  return converted;
}

inline Classification getHighestProbClassification(
  const std::vector<Classification> & classifications)
{
  if (classifications.empty()) {
    return Classification{};
  }
  return *std::max_element(
    classifications.begin(), classifications.end(),
    [](const auto & a, const auto & b) { return a.probability < b.probability; });
}

inline Label getHighestProbLabel(const std::vector<Classification> & classifications)
{
  return getHighestProbClassification(classifications).label;
}

}  // namespace autoware::multi_object_tracker::classes

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__CLASSES_HPP_
