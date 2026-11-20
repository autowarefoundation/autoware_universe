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

// Unit tests of ClassificationRemapper: one table applied to a list of classifications. The
// adapter test covers what the node's parameters make of the shipped configuration; here the
// table is given directly, so that the lookup itself is what is checked.

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

// The remapper keeps the table it is given, as it is: the node's parameters decide the entries,
// the remapper adds none.
TEST(ClassificationRemapper, Table_KeptAsGiven)
{
  const ClassificationRemap table = {
    {RadarClassification::MOTORCYCLE, ObjectClassification::CAR},
    {RadarClassification::HAZARD, ObjectClassification::UNKNOWN}};

  const ClassificationRemapper remapper(table);

  EXPECT_EQ(remapper.table(), table);
}

// An empty table makes every label UNKNOWN.
TEST(ClassificationRemapper, Table_Empty_EverythingUnknown)
{
  const ClassificationRemapper remapper({});

  EXPECT_TRUE(remapper.table().empty());
  const auto output = remapper(
    {radar_classification(RadarClassification::CAR, 0.8f),
     radar_classification(RadarClassification::HAZARD, 0.1f)});
  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::UNKNOWN, 0.8f}, {ObjectClassification::UNKNOWN, 0.1f}};
  EXPECT_EQ(labeled_probabilities(output), expected);
}

// On a list: one entry per input entry, in order, probability copied, label looked up in the
// table; a label the table does not mention becomes UNKNOWN.
TEST(ClassificationRemapper, Apply_OrderAndProbabilitiesKeptLabelsLookedUp)
{
  const ClassificationRemapper remapper(
    {{RadarClassification::CAR, ObjectClassification::CAR},
     {RadarClassification::BICYCLE, ObjectClassification::CAR},
     {RadarClassification::HAZARD, ObjectClassification::UNKNOWN}});

  const auto output = remapper(
    {radar_classification(RadarClassification::BICYCLE, 0.3f),
     radar_classification(RadarClassification::CAR, 0.8f),
     radar_classification(RadarClassification::HAZARD, 0.05f),
     radar_classification(RadarClassification::TRUCK, 0.1f)});

  const std::vector<LabeledProbability> expected = {
    {ObjectClassification::CAR, 0.3f},
    {ObjectClassification::CAR, 0.8f},
    {ObjectClassification::UNKNOWN, 0.05f},  // in the table
    {ObjectClassification::UNKNOWN, 0.1f}};  // not in the table
  EXPECT_EQ(labeled_probabilities(output), expected);
}

TEST(ClassificationRemapper, Apply_EmptyList_Empty)
{
  const ClassificationRemapper remapper(
    ClassificationRemap{{RadarClassification::CAR, ObjectClassification::CAR}});

  EXPECT_TRUE(remapper({}).empty());
}
