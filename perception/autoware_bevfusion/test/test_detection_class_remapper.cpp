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

#include <autoware/bevfusion/detection_class_remapper.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace
{

using autoware::bevfusion::DetectionClassRemapper;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;

struct InvalidRemapperConfig
{
  std::vector<std::int64_t> allow_remapping_by_area_matrix;
  std::vector<double> min_area_matrix;
  std::vector<double> max_area_matrix;
  std::string expected_error_substring;
};

std::tuple<std::vector<std::int64_t>, std::vector<double>, std::vector<double>>
makeVehicleRemapperParameters()
{
  constexpr std::size_t kNumClasses = 8;
  std::vector<std::int64_t> allow_remapping_by_area_matrix(kNumClasses * kNumClasses, 0);
  std::vector<double> min_area_matrix(kNumClasses * kNumClasses, 0.0);
  std::vector<double> max_area_matrix(kNumClasses * kNumClasses, 0.0);

  auto set_rule =
    [&](std::uint8_t source_label, std::uint8_t target_label, double min_area, double max_area) {
      const auto index = static_cast<std::size_t>(source_label) * kNumClasses + target_label;
      allow_remapping_by_area_matrix.at(index) = 1;
      min_area_matrix.at(index) = min_area;
      max_area_matrix.at(index) = max_area;
    };

  set_rule(ObjectClassification::CAR, ObjectClassification::TRUCK, 12.1, 36.0);
  set_rule(ObjectClassification::CAR, ObjectClassification::TRAILER, 36.0, 999.0);
  set_rule(ObjectClassification::TRUCK, ObjectClassification::TRAILER, 36.0, 999.0);

  return {
    allow_remapping_by_area_matrix,
    min_area_matrix,
    max_area_matrix};
}

DetectedObject makeObject(double length, double width, std::uint8_t label)
{
  DetectedObject object;
  object.shape.dimensions.x = length;
  object.shape.dimensions.y = width;

  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0f;
  object.classification = {classification};

  return object;
}

void expectSetParametersThrows(
  const InvalidRemapperConfig & invalid_config, const std::string & expected_error_substring)
{
  DetectionClassRemapper remapper;

  try {
    remapper.setParameters(
      invalid_config.allow_remapping_by_area_matrix, invalid_config.min_area_matrix,
      invalid_config.max_area_matrix);
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find(expected_error_substring), std::string::npos);
  }
}

class DetectionClassRemapperInvalidConfigTest
: public ::testing::TestWithParam<InvalidRemapperConfig>
{
};

TEST(DetectionClassRemapperTest, MapClasses)
{
  DetectionClassRemapper remapper;

  const auto [allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix] =
    makeVehicleRemapperParameters();

  remapper.setParameters(allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  DetectedObjects msg;
  msg.objects.push_back(makeObject(2.0, 2.0, ObjectClassification::CAR));
  msg.objects.push_back(makeObject(8.0, 2.0, ObjectClassification::CAR));
  msg.objects.push_back(makeObject(10.0, 4.0, ObjectClassification::TRUCK));
  msg.objects.push_back(makeObject(1.0, 1.0, ObjectClassification::PEDESTRIAN));

  remapper.mapClasses(msg);

  EXPECT_EQ(msg.objects[0].classification[0].label, ObjectClassification::CAR);
  EXPECT_EQ(msg.objects[1].classification[0].label, ObjectClassification::TRUCK);
  EXPECT_EQ(msg.objects[2].classification[0].label, ObjectClassification::TRAILER);
  EXPECT_EQ(msg.objects[3].classification[0].label, ObjectClassification::PEDESTRIAN);
}

TEST_P(DetectionClassRemapperInvalidConfigTest, SetParametersThrowsOnInvalidConfig)
{
  const auto & invalid_config = GetParam();
  expectSetParametersThrows(invalid_config, invalid_config.expected_error_substring);
}

TEST(DetectionClassRemapperTest, MapClassesSkipsLabelsOutsideConfiguredMatrix)
{
  DetectionClassRemapper remapper;

  const auto [allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix] =
    makeVehicleRemapperParameters();

  remapper.setParameters(allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  DetectedObjects msg;
  msg.objects.push_back(makeObject(6.0, 2.0, ObjectClassification::HAZARD));
  msg.objects.push_back(makeObject(8.0, 2.0, ObjectClassification::CAR));

  remapper.mapClasses(msg);

  EXPECT_EQ(msg.objects[0].classification[0].label, ObjectClassification::HAZARD);
  EXPECT_EQ(msg.objects[1].classification[0].label, ObjectClassification::TRUCK);
}

INSTANTIATE_TEST_SUITE_P(
  InvalidConfigs, DetectionClassRemapperInvalidConfigTest,
  ::testing::Values(
    InvalidRemapperConfig{
      {0, 1, 0, 1},
      {0.0, 1.0, 2.0},
      {0.0, 1.0, 2.0, 3.0},
      "allow_remapping_by_area_matrix and min_area_matrix"},
    InvalidRemapperConfig{
      {},
      {},
      {},
      "must not be empty"},
    InvalidRemapperConfig{
      {0, 1, 0},
      {0.0, 1.0, 2.0},
      {0.0, 1.0, 2.0},
      "square matrix"}));

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
