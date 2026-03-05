// Copyright 2025 TIER IV.
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

#include "utils/classification_loader.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

TEST(ClassificationLoader, LoadsMatchingSizes)
{
  std::vector<std::string> class_names{"car", "ped"};
  std::vector<double> thresholds{0.5, 0.7};
  std::vector<std::string> target_class_names;
  std::map<std::string, float> target_thresholds;
  int32_t num_classes = 0;

  const utils::ClassificationConfig cfg{
    class_names,        thresholds,   &target_class_names,
    &target_thresholds, &num_classes, "unit_test_load_classification_config"};

  const bool ok = utils::load_classification_config(cfg);

  EXPECT_TRUE(ok);
  EXPECT_EQ(num_classes, 2);
  ASSERT_EQ(target_class_names.size(), 2U);
  EXPECT_EQ(target_class_names[0], "car");
  EXPECT_EQ(target_class_names[1], "ped");
  ASSERT_EQ(target_thresholds.size(), 2U);
  EXPECT_FLOAT_EQ(target_thresholds.at("car"), 0.5F);
  EXPECT_FLOAT_EQ(target_thresholds.at("ped"), 0.7F);
}

TEST(ClassificationLoader, FailsOnSizeMismatch)
{
  std::vector<std::string> class_names{"car", "ped"};
  std::vector<double> thresholds{0.5};
  std::vector<std::string> target_class_names{"should", "stay"};
  std::map<std::string, float> target_thresholds{{"car", 1.0F}};
  int32_t num_classes = -1;

  const utils::ClassificationConfig cfg{
    class_names,        thresholds,   &target_class_names,
    &target_thresholds, &num_classes, "unit_test_load_classification_config_mismatch"};

  const bool ok = utils::load_classification_config(cfg);

  EXPECT_FALSE(ok);
  // Ensure outputs remain unchanged when validation fails
  EXPECT_EQ(num_classes, -1);
  EXPECT_EQ(target_class_names.size(), 2U);
  EXPECT_EQ(target_thresholds.size(), 1U);
}

TEST(ClassificationLoader, HandlesNullNumClasses)
{
  std::vector<std::string> class_names{"car"};
  std::vector<double> thresholds{0.9};
  std::vector<std::string> target_class_names;
  std::map<std::string, float> target_thresholds;

  const utils::ClassificationConfig cfg{
    class_names,        thresholds, &target_class_names,
    &target_thresholds, nullptr,    "unit_test_load_classification_config_no_count"};

  const bool ok = utils::load_classification_config(cfg);

  EXPECT_TRUE(ok);
  EXPECT_EQ(target_class_names.size(), 1U);
  EXPECT_FLOAT_EQ(target_thresholds.at("car"), 0.9F);
}

}  // namespace autoware::tensorrt_vad
