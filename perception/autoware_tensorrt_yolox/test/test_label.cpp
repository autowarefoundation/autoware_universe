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

#include <autoware/tensorrt_yolox/label.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <string>

std::string get_file_path(const std::string & filename)
{
  const auto package_dir =
    ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox");
  std::string path = package_dir + "/test_label_data/" + filename;

  return path;
}

// test parsing a label file
TEST(LabelProcessing, ReadLabelFile) {
  const std::string input_file_name = get_file_path("test_label_with_spaces.txt");

  std::vector<std::string> roi_id_to_name_map;
  std::unordered_map<std::string, int> roi_name_to_id_map;

  autoware::tensorrt_yolox::read_label_file(
    input_file_name, roi_id_to_name_map, roi_name_to_id_map);

  ASSERT_EQ(roi_id_to_name_map.size(), 3);
  EXPECT_EQ(roi_id_to_name_map[0], "CAR");
  EXPECT_EQ(roi_id_to_name_map[1], "PEDESTRIAN");
  EXPECT_EQ(roi_id_to_name_map[2], "UNKNOWN");

  ASSERT_EQ(roi_name_to_id_map.size(), 3);
  EXPECT_EQ(roi_name_to_id_map["CAR"], 0);
  EXPECT_EQ(roi_name_to_id_map["PEDESTRIAN"], 1);
  EXPECT_EQ(roi_name_to_id_map["UNKNOWN"], 2);
}

// test parsing a semantic segmentation color map file
TEST(LabelProcessing, ReadSemsegColorMapFile) {
  const std::string input_file_name = get_file_path("test_semseg_col_map_with_spaces.csv");

  std::vector<autoware::tensorrt_yolox::Colormap> semseg_color_map;
  std::unordered_map<std::string, int> semseg_name_to_id_map;
  uint32_t skip_header_lines = 1;

  autoware::tensorrt_yolox::load_segmentation_colormap(
    input_file_name, semseg_color_map, semseg_name_to_id_map, skip_header_lines);

  ASSERT_EQ(semseg_color_map.size(), 3);

  // check parsed results
  EXPECT_EQ(semseg_color_map[0].id, 0);
  EXPECT_EQ(semseg_color_map[0].name, "others");
  EXPECT_EQ(static_cast<int>(semseg_color_map[0].color[0]), 0);
  EXPECT_EQ(static_cast<int>(semseg_color_map[0].color[1]), 1);
  EXPECT_EQ(static_cast<int>(semseg_color_map[0].color[2]), 2);

  EXPECT_EQ(semseg_color_map[1].id, 1);
  EXPECT_EQ(semseg_color_map[1].name, "buildling");
  EXPECT_EQ(static_cast<int>(semseg_color_map[1].color[0]), 70);
  EXPECT_EQ(static_cast<int>(semseg_color_map[1].color[1]), 75);
  EXPECT_EQ(static_cast<int>(semseg_color_map[1].color[2]), 80);

  EXPECT_EQ(semseg_color_map[2].id, 2);
  EXPECT_EQ(semseg_color_map[2].name, "wall");
  EXPECT_EQ(static_cast<int>(semseg_color_map[2].color[0]), 150);
  EXPECT_EQ(static_cast<int>(semseg_color_map[2].color[1]), 160);
  EXPECT_EQ(static_cast<int>(semseg_color_map[2].color[2]), 170);

  // check name-to-id mapping is correct
  ASSERT_EQ(semseg_name_to_id_map.size(), 3);
  EXPECT_EQ(semseg_name_to_id_map["others"], 0);
  EXPECT_EQ(semseg_name_to_id_map["buildling"], 1);
  EXPECT_EQ(semseg_name_to_id_map["wall"], 2);
}

// test parsing a label remap file
TEST(LabelProcessing, ReadLabelRemapFile) {
  const std::string input_file_name = get_file_path("test_label_remap.csv");

  std::unordered_map<std::string, int> label_name_to_id_remap;
  uint32_t skip_header_lines = 1;

  autoware::tensorrt_yolox::load_label_id_remap_file(
    input_file_name, label_name_to_id_remap, skip_header_lines);

  ASSERT_EQ(label_name_to_id_remap.size(), 3);
  EXPECT_EQ(label_name_to_id_remap["CAR"], 0);
  EXPECT_EQ(label_name_to_id_remap["PEDESTRIAN"], 1);
  EXPECT_EQ(label_name_to_id_remap["UNKNOWN"], 2);
}

// test parsing a label remap file without header
// NOTE: the skip_header_lines is hard-coded in the source
TEST(LabelProcessing, ReadLabelRemapFileWithoutHeader) {
  const std::string input_file_name = get_file_path("test_label_remap_without_header.csv");

  std::unordered_map<std::string, int> label_name_to_id_remap;
  uint32_t skip_header_lines = 0;

  autoware::tensorrt_yolox::load_label_id_remap_file(
    input_file_name, label_name_to_id_remap, skip_header_lines);

  ASSERT_EQ(label_name_to_id_remap.size(), 3);
  EXPECT_EQ(label_name_to_id_remap["CAR"], 0);
  EXPECT_EQ(label_name_to_id_remap["PEDESTRIAN"], 1);
  EXPECT_EQ(label_name_to_id_remap["UNKNOWN"], 2);
}

// test parsing a label remap file with comment in the lines
TEST(LabelProcessing, ReadLabelRemapFileWithComment) {
  const std::string input_file_name = get_file_path("test_label_file_with_comment.csv");

  std::unordered_map<std::string, int> label_name_to_id_remap;
  uint32_t skip_header_lines = 1;

  autoware::tensorrt_yolox::load_label_id_remap_file(
    input_file_name, label_name_to_id_remap, skip_header_lines);

  ASSERT_EQ(label_name_to_id_remap.size(), 3);
  EXPECT_EQ(label_name_to_id_remap["CAR"], 1);
  EXPECT_EQ(label_name_to_id_remap["PEDESTRIAN"], 3);
  EXPECT_EQ(label_name_to_id_remap["UNKNOWN"], 5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
