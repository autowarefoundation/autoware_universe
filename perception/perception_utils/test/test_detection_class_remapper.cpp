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

#include "detection_test_utils.hpp"
#include "perception_utils/detection_class_remapper.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace
{

using autoware_perception_msgs::msg::DetectedObjects;
using perception_utils::test::make_object;

TEST(DetectionClassRemapperTest, RemapsEveryClassificationByBevArea)
{
  perception_utils::DetectionClassRemapper remapper;
  const std::vector<std::int64_t> allow{0, 0, 0,  //
                                        0, 0, 1,  //
                                        0, 1, 0};
  const std::vector<double> min_area{0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0};
  const std::vector<double> max_area{0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, 10.0, 0.0};
  remapper.setParameters(allow, min_area, max_area);

  DetectedObjects objects;
  objects.objects = {
    make_object(0.0, 0.0, 2.0F, 2.0F, 0, 1.0F), make_object(0.0, 0.0, 8.0F, 2.0F, 1, 1.0F),
    make_object(0.0, 0.0, 3.0F, 3.0F, 2, 1.0F), make_object(0.0, 0.0, 4.0F, 3.0F, 2, 1.0F)};

  remapper.mapClasses(objects);

  EXPECT_EQ(objects.objects.at(0).classification.at(0).label, 0);
  EXPECT_EQ(objects.objects.at(1).classification.at(0).label, 2);
  EXPECT_EQ(objects.objects.at(2).classification.at(0).label, 1);
  EXPECT_EQ(objects.objects.at(3).classification.at(0).label, 2);
}

TEST(DetectionClassRemapperTest, RejectsInvalidMatrixDimensions)
{
  perception_utils::DetectionClassRemapper remapper;
  EXPECT_THROW(remapper.setParameters({0, 0}, {0.0, 0.0}, {0.0, 0.0}), std::invalid_argument);
}

}  // namespace
