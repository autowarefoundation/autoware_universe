// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/crop_box_filter/crop_box_filter_node.hpp"

#include <gtest/gtest.h>

#include <array>

using autoware::pointcloud_preprocessor::CropBox;
using autoware::pointcloud_preprocessor::does_line_segment_intersect_crop_box;

struct TestCaseParam
{
  std::string gtest_suffix;
  CropBox box;
  Eigen::Vector3f from_point;
  Eigen::Vector3f to_point;
  bool expected_result;
};

class CropBoxFilterTest : public ::testing::TestWithParam<TestCaseParam>
{
};

static autoware::pointcloud_preprocessor::CropBox box = {1, 4, 2, 5, -1, 1};

static std::array<TestCaseParam, 5> test_cases = {{
  {"diagonal_from_box_corner_to_box_corner", box, {1, 2, -1}, {4, 3, 1}, true},
  {"axis_aligned_x_axis_passing_through_the_box", box, {-1, 4, 0}, {5, 4, 0}, true},
  {"axis_aligned_y_axis_passing_next_to_the_box", box, {-1, 4, -2}, {5, 4, -2}, false},
  {"axis_aligned_z_axis_stopping_short_of_the_box", box, {-1, 4, -0}, {0, 4, -0}, false},
  {"line_segment_contained_in_the_box", box, {2, 3, -0.5}, {3, 4, 0.5}, true},
}};

TEST_P(CropBoxFilterTest, RayIntersection)
{
  const auto & param = GetParam();
  const auto & box = param.box;
  Eigen::Vector4f from_point = {
    param.from_point.x(), param.from_point.y(), param.from_point.z(), 1.0};
  Eigen::Vector4f to_point = {param.to_point.x(), param.to_point.y(), param.to_point.z(), 1.0};
  const auto & expected_result = param.expected_result;

  EXPECT_EQ(does_line_segment_intersect_crop_box(from_point, to_point, box), expected_result);
  EXPECT_EQ(does_line_segment_intersect_crop_box(to_point, from_point, box), expected_result);
}

INSTANTIATE_TEST_SUITE_P(
  TestMain, CropBoxFilterTest, testing::ValuesIn(test_cases),
  [](const testing::TestParamInfo<TestCaseParam> & p) { return p.param.gtest_suffix; });

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
