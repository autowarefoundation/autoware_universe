// Copyright 2026 The Autoware Contributors
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

// Unit tests of ClassificationRemapper: two tables laid over each other and applied to a list.
// The adapter test covers what the node's parameters make of the shipped configuration; here the
// tables are given directly, so that the layering itself is what is checked.

#include "radar_objects_adapter.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace
{
using autoware::radar_objects_adapter::ClassificationRemap;
using autoware::radar_objects_adapter::ClassificationRemapper;
using autoware::radar_objects_adapter::ObjectClassification;
using autoware::radar_objects_adapter::perception_friendly_classification_remap;
using autoware::radar_objects_adapter::RadarClassification;

RadarClassification radar_classification(RadarClassification::_label_type label, float probability)
{
  RadarClassification classification;
  classification.label = label;
  classification.probability = probability;
  return classification;
}

using LabeledProbability = std::pair<ObjectClassification::_label_type, float>;

std::vector<LabeledProbability> labeled_probabilities(
  const std::vector<ObjectClassification> & classifications)
{
  std::vector<LabeledProbability> result;
  for (const auto & c : classifications) {
    result.emplace_back(c.label, c.probability);
  }
  return result;
}
}  // namespace

// The perception-friendly table names exactly the three labels the perception pipeline has no
// use for, and sends each to UNKNOWN. Everything else is left to the sensor's table.
TEST(ClassificationRemapper, PerceptionFriendly_RadarOnlyLabelsToUnknown)
{
  const ClassificationRemap expected = {
    {RadarClassification::HAZARD, ObjectClassification::UNKNOWN},
    {RadarClassification::OVER_DRIVABLE, ObjectClassification::UNKNOWN},
    {RadarClassification::UNDER_DRIVABLE, ObjectClassification::UNKNOWN}};

  EXPECT_EQ(perception_friendly_classification_remap(), expected);
}

// The combined table is the perception-friendly one with the sensor-dependent one laid over it:
// an entry only one table has is kept, and where both have one the sensor's wins.
TEST(ClassificationRemapper, Combined_SensorEntriesLaidOverPerceptionFriendly)
{
  const ClassificationRemap perception_friendly = {
    {RadarClassification::HAZARD, ObjectClassification::UNKNOWN},
    {RadarClassification::OVER_DRIVABLE, ObjectClassification::UNKNOWN}};
  const ClassificationRemap sensor_dependent = {
    {RadarClassification::MOTORCYCLE, ObjectClassification::CAR},
    {RadarClassification::HAZARD, ObjectClassification::CAR}};  // overrides

  const ClassificationRemapper remapper(perception_friendly, sensor_dependent);

  const ClassificationRemap expected = {
    {RadarClassification::HAZARD, ObjectClassification::CAR},
    {RadarClassification::OVER_DRIVABLE, ObjectClassification::UNKNOWN},
    {RadarClassification::MOTORCYCLE, ObjectClassification::CAR}};
  EXPECT_EQ(remapper.combined(), expected);
}

// Two empty tables make an empty remapper, under which every label becomes UNKNOWN.
TEST(ClassificationRemapper, Combined_BothEmpty_EverythingUnknown)
{
  const ClassificationRemapper remapper({}, {});

  EXPECT_TRUE(remapper.combined().empty());
  const auto output = remapper(
    {radar_classification(RadarClassification::CAR, 0.8f),
     radar_classification(RadarClassification::HAZARD, 0.1f)});
  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::UNKNOWN, 0.8f}, {ObjectClassification::UNKNOWN, 0.1f}};
  EXPECT_EQ(labeled_probabilities(output), expected);
}

// On a list: one entry per input entry, in order, probability copied, label looked up in the
// combined table; a label neither table mentions becomes UNKNOWN.
TEST(ClassificationRemapper, Apply_OrderAndProbabilitiesKeptLabelsLookedUp)
{
  const ClassificationRemapper remapper(
    perception_friendly_classification_remap(),
    {{RadarClassification::CAR, ObjectClassification::CAR},
     {RadarClassification::BICYCLE, ObjectClassification::CAR}});

  const auto output = remapper(
    {radar_classification(RadarClassification::BICYCLE, 0.3f),
     radar_classification(RadarClassification::CAR, 0.8f),
     radar_classification(RadarClassification::HAZARD, 0.05f),
     radar_classification(RadarClassification::TRUCK, 0.1f)});

  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::CAR, 0.3f},
    {ObjectClassification::CAR, 0.8f},
    {ObjectClassification::UNKNOWN, 0.05f},  // perception-friendly table
    {ObjectClassification::UNKNOWN, 0.1f}};  // in neither table
  EXPECT_EQ(labeled_probabilities(output), expected);
}

TEST(ClassificationRemapper, Apply_EmptyList_Empty)
{
  const ClassificationRemapper remapper(perception_friendly_classification_remap(), {});

  EXPECT_TRUE(remapper({}).empty());
}
