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

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

// cspell: ignore semseg

namespace
{
std::string write_temp_file(const std::string & filename, const std::string & content)
{
  const std::filesystem::path path = std::filesystem::temp_directory_path() / filename;
  std::ofstream file(path);
  file << content;
  return path.string();
}
}  // namespace

// load_label_maps parses the label file into the ROI class name list
TEST(LoadLabelMaps, ParsesRoiClassNameList)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_with_spaces.txt",
    " CAR\n"
    " PEDESTRIAN\n"
    "UNKNOWN\n");

  // Act
  const auto label_maps = autoware::tensorrt_yolox::load_label_maps(label_path, "", "", "");

  // Assert
  ASSERT_EQ(label_maps.roi_class_name_list.size(), 3);
  EXPECT_EQ(label_maps.roi_class_name_list[0], "CAR");
  EXPECT_EQ(label_maps.roi_class_name_list[1], "PEDESTRIAN");
  EXPECT_EQ(label_maps.roi_class_name_list[2], "UNKNOWN");
}

// without optional files, the segmentation tables stay in their "not specified" form while the
// class-id table is sized but fully unmapped
TEST(LoadLabelMaps, LeavesOptionalTablesUnsetWhenPathsEmpty)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_with_spaces.txt",
    " CAR\n"
    " PEDESTRIAN\n"
    "UNKNOWN\n");

  // Act
  const auto label_maps = autoware::tensorrt_yolox::load_label_maps(label_path, "", "", "");

  // Assert
  EXPECT_TRUE(label_maps.semseg_color_map.empty());
  EXPECT_TRUE(label_maps.roi_id_to_semseg_id_map.empty());
  ASSERT_EQ(label_maps.roi_id_to_class_id_map.size(), 3);
  for (const int class_id : label_maps.roi_id_to_class_id_map) {
    EXPECT_EQ(class_id, autoware::tensorrt_yolox::unmapped_label_id);
  }
}

// load_label_maps applies the ROI remap file to resolve the class-id table
TEST(LoadLabelMaps, ResolvesRoiRemap)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_with_spaces.txt",
    " CAR\n"
    " PEDESTRIAN\n"
    "UNKNOWN\n");
  const std::string roi_remap_path = write_temp_file(
    "label_remap.csv",
    "from, to\n"
    "CAR, 0\n"
    "PEDESTRIAN, 1\n"
    "UNKNOWN, 2\n");

  // Act
  const auto label_maps =
    autoware::tensorrt_yolox::load_label_maps(label_path, "", roi_remap_path, "");

  // Assert
  ASSERT_EQ(label_maps.roi_id_to_class_id_map.size(), 3);
  EXPECT_EQ(label_maps.roi_id_to_class_id_map[0], 0);  // CAR
  EXPECT_EQ(label_maps.roi_id_to_class_id_map[1], 1);  // PEDESTRIAN
  EXPECT_EQ(label_maps.roi_id_to_class_id_map[2], 2);  // UNKNOWN
}

// comments and the header line in the remap file are ignored while resolving the class-id table
TEST(LoadLabelMaps, ResolvesRoiRemapWithComments)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_with_spaces.txt",
    " CAR\n"
    " PEDESTRIAN\n"
    "UNKNOWN\n");
  const std::string roi_remap_path = write_temp_file(
    "label_remap_with_comment.csv",
    "from, to\n"
    "# this line is comment\n"
    "CAR, 1\n"
    "PEDESTRIAN, 3 # after hash, it will be comment\n"
    "# this line is also comment\n"
    "UNKNOWN, 5\n");

  // Act
  const auto label_maps =
    autoware::tensorrt_yolox::load_label_maps(label_path, "", roi_remap_path, "");

  // Assert
  ASSERT_EQ(label_maps.roi_id_to_class_id_map.size(), 3);
  EXPECT_EQ(label_maps.roi_id_to_class_id_map[0], 1);  // CAR
  EXPECT_EQ(label_maps.roi_id_to_class_id_map[1], 3);  // PEDESTRIAN
  EXPECT_EQ(label_maps.roi_id_to_class_id_map[2], 5);  // UNKNOWN
}

// load_label_maps parses the segmentation color map file
TEST(LoadLabelMaps, ParsesSegmentationColorMap)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_with_spaces.txt",
    " CAR\n"
    " PEDESTRIAN\n"
    "UNKNOWN\n");
  const std::string color_map_path = write_temp_file(
    "semseg_col_map_with_spaces.csv",
    "id,name,r,g,b\n"
    "0,others,0,1,2\n"
    "1, building ,70,75,80\n"
    " 2, wall, 150, 160, 170\n");

  // Act
  const auto label_maps =
    autoware::tensorrt_yolox::load_label_maps(label_path, color_map_path, "", "");

  // Assert
  const auto & semseg_color_map = label_maps.semseg_color_map;
  ASSERT_EQ(semseg_color_map.size(), 3);

  EXPECT_EQ(semseg_color_map[0].id, 0);
  EXPECT_EQ(semseg_color_map[0].name, "others");
  EXPECT_EQ(static_cast<int>(semseg_color_map[0].color[0]), 0);
  EXPECT_EQ(static_cast<int>(semseg_color_map[0].color[1]), 1);
  EXPECT_EQ(static_cast<int>(semseg_color_map[0].color[2]), 2);

  EXPECT_EQ(semseg_color_map[1].id, 1);
  EXPECT_EQ(semseg_color_map[1].name, "building");
  EXPECT_EQ(static_cast<int>(semseg_color_map[1].color[0]), 70);
  EXPECT_EQ(static_cast<int>(semseg_color_map[1].color[1]), 75);
  EXPECT_EQ(static_cast<int>(semseg_color_map[1].color[2]), 80);

  EXPECT_EQ(semseg_color_map[2].id, 2);
  EXPECT_EQ(semseg_color_map[2].name, "wall");
  EXPECT_EQ(static_cast<int>(semseg_color_map[2].color[0]), 150);
  EXPECT_EQ(static_cast<int>(semseg_color_map[2].color[1]), 160);
  EXPECT_EQ(static_cast<int>(semseg_color_map[2].color[2]), 170);
}

// load_label_maps applies the ROI-to-segmentation remap file to resolve the segmentation-id table
TEST(LoadLabelMaps, ResolvesRoiToSemsegRemap)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_with_spaces.txt",
    " CAR\n"
    " PEDESTRIAN\n"
    "UNKNOWN\n");
  const std::string roi_to_semseg_remap_path = write_temp_file(
    "label_remap.csv",
    "from, to\n"
    "CAR, 0\n"
    "PEDESTRIAN, 1\n"
    "UNKNOWN, 2\n");

  // Act
  const auto label_maps =
    autoware::tensorrt_yolox::load_label_maps(label_path, "", "", roi_to_semseg_remap_path);

  // Assert
  ASSERT_EQ(label_maps.roi_id_to_semseg_id_map.size(), 3);
  EXPECT_EQ(label_maps.roi_id_to_semseg_id_map[0], 0);  // CAR
  EXPECT_EQ(label_maps.roi_id_to_semseg_id_map[1], 1);  // PEDESTRIAN
  EXPECT_EQ(label_maps.roi_id_to_semseg_id_map[2], 2);  // UNKNOWN
}

// an empty remap disables remapping, leaving every entry unmapped
TEST(BuildRoiIdToTargetIdMap, EmptyRemapLeavesEveryEntryUnmapped)
{
  // Arrange
  const std::vector<std::string> roi_class_name_list = {"CAR", "PEDESTRIAN", "UNKNOWN"};
  const std::unordered_map<std::string, int> empty_remap;

  // Act
  const auto roi_id_to_target_id_map = autoware::tensorrt_yolox::build_roi_id_to_target_id_map(
    roi_class_name_list, empty_remap, autoware::tensorrt_yolox::unmapped_label_id);

  // Assert
  ASSERT_EQ(roi_id_to_target_id_map.size(), 3);
  for (const int target_id : roi_id_to_target_id_map) {
    EXPECT_EQ(target_id, autoware::tensorrt_yolox::unmapped_label_id);
  }
}

// a non-empty remap is applied by class name, indexed by the model's output class ID
TEST(BuildRoiIdToTargetIdMap, AppliesRemapByClassName)
{
  // Arrange
  const std::vector<std::string> roi_class_name_list = {"CAR", "PEDESTRIAN", "UNKNOWN"};
  const std::unordered_map<std::string, int> remap = {
    {"CAR", 7}, {"PEDESTRIAN", 8}, {"UNKNOWN", 9}};

  // Act
  const auto roi_id_to_target_id_map = autoware::tensorrt_yolox::build_roi_id_to_target_id_map(
    roi_class_name_list, remap, autoware::tensorrt_yolox::unmapped_label_id);

  // Assert
  ASSERT_EQ(roi_id_to_target_id_map.size(), 3);
  EXPECT_EQ(roi_id_to_target_id_map[0], 7);
  EXPECT_EQ(roi_id_to_target_id_map[1], 8);
  EXPECT_EQ(roi_id_to_target_id_map[2], 9);
}

// a class name missing from a non-empty remap is treated as an error (likely a wrong model)
TEST(BuildRoiIdToTargetIdMap, ThrowsWhenClassNameMissingFromRemap)
{
  // Arrange
  const std::vector<std::string> roi_class_name_list = {"CAR", "PEDESTRIAN", "UNKNOWN"};
  const std::unordered_map<std::string, int> remap = {{"CAR", 0}, {"PEDESTRIAN", 1}};

  // Act / Assert
  EXPECT_THROW(
    autoware::tensorrt_yolox::build_roi_id_to_target_id_map(
      roi_class_name_list, remap, autoware::tensorrt_yolox::unmapped_label_id),
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
