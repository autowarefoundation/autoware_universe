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
#include <type_traits>
#include <vector>

namespace autoware::multi_object_tracker::object_model
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

constexpr int NUM_LABELS = 8;

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

}  // namespace autoware::multi_object_tracker::object_model

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__OBJECT_MODEL__CLASSES_HPP_
