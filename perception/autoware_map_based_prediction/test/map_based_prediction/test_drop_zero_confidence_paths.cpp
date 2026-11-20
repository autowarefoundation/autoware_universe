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

#include "autoware/map_based_prediction/utils.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::map_based_prediction
{
namespace
{
PredictedObject makeObject(const std::vector<float> & confidences)
{
  PredictedObject object;
  for (const float confidence : confidences) {
    PredictedPath path;
    path.confidence = confidence;
    object.kinematics.predicted_paths.push_back(path);
  }
  return object;
}

std::vector<float> confidencesOf(const PredictedObject & object)
{
  std::vector<float> out;
  for (const auto & path : object.kinematics.predicted_paths) out.push_back(path.confidence);
  return out;
}
}  // namespace

TEST(DropZeroConfidencePaths, RemovesZeroConfidencePathsAndKeepsTheRest)
{
  PredictedObjects objects;
  // A lane-changing vehicle: lane follow 0.47, detected side 0.53, opposite side 0.
  objects.objects.push_back(makeObject({0.47F, 0.53F, 0.0F}));
  // Zero in the middle, several positive paths.
  objects.objects.push_back(makeObject({0.0F, 0.25F, 0.0F, 0.25F, 0.5F}));
  // A single path at full confidence is untouched.
  objects.objects.push_back(makeObject({1.0F}));
  utils::dropZeroConfidencePaths(objects);
  EXPECT_EQ(confidencesOf(objects.objects[0]), (std::vector<float>{0.47F, 0.53F}));
  EXPECT_EQ(confidencesOf(objects.objects[1]), (std::vector<float>{0.25F, 0.25F, 0.5F}));
  EXPECT_EQ(confidencesOf(objects.objects[2]), (std::vector<float>{1.0F}));
}

TEST(DropZeroConfidencePaths, NeverLeavesAnObjectWithoutPaths)
{
  PredictedObjects objects;
  objects.objects.push_back(makeObject({0.0F, 0.0F}));
  objects.objects.push_back(makeObject({0.0F}));
  objects.objects.push_back(makeObject({}));
  utils::dropZeroConfidencePaths(objects);
  EXPECT_EQ(objects.objects[0].kinematics.predicted_paths.size(), 2U);
  EXPECT_EQ(objects.objects[1].kinematics.predicted_paths.size(), 1U);
  EXPECT_TRUE(objects.objects[2].kinematics.predicted_paths.empty());
}

TEST(DropZeroConfidencePaths, TreatsNegativeConfidenceAsZero)
{
  PredictedObjects objects;
  objects.objects.push_back(makeObject({-0.1F, 0.6F}));
  utils::dropZeroConfidencePaths(objects);
  EXPECT_EQ(confidencesOf(objects.objects[0]), (std::vector<float>{0.6F}));
}
}  // namespace autoware::map_based_prediction
