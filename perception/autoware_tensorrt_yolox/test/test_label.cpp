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
  const auto roi_labels = autoware::tensorrt_yolox::load_label_maps(label_path, "", "");

  // Assert
  ASSERT_EQ(roi_labels.size(), 3);
  EXPECT_EQ(roi_labels[0].name, "CAR");
  EXPECT_EQ(roi_labels[1].name, "PEDESTRIAN");
  EXPECT_EQ(roi_labels[2].name, "UNKNOWN");
}

// without optional remap files, all class_id and semseg_id fields are unmapped_label_id
TEST(LoadLabelMaps, LeavesOptionalFieldsUnmappedWhenPathsEmpty)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_with_spaces.txt",
    " CAR\n"
    " PEDESTRIAN\n"
    "UNKNOWN\n");

  // Act
  const auto roi_labels = autoware::tensorrt_yolox::load_label_maps(label_path, "", "");

  // Assert
  ASSERT_EQ(roi_labels.size(), 3);
  for (const auto & roi_label : roi_labels) {
    EXPECT_EQ(roi_label.class_id, autoware::tensorrt_yolox::unmapped_label_id);
    EXPECT_EQ(roi_label.semseg_id, autoware::tensorrt_yolox::unmapped_label_id);
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
  const auto roi_labels = autoware::tensorrt_yolox::load_label_maps(label_path, roi_remap_path, "");

  // Assert
  ASSERT_EQ(roi_labels.size(), 3);
  EXPECT_EQ(roi_labels[0].class_id, 0);  // CAR
  EXPECT_EQ(roi_labels[1].class_id, 1);  // PEDESTRIAN
  EXPECT_EQ(roi_labels[2].class_id, 2);  // UNKNOWN
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
  const auto roi_labels = autoware::tensorrt_yolox::load_label_maps(label_path, roi_remap_path, "");

  // Assert
  ASSERT_EQ(roi_labels.size(), 3);
  EXPECT_EQ(roi_labels[0].class_id, 1);  // CAR
  EXPECT_EQ(roi_labels[1].class_id, 3);  // PEDESTRIAN
  EXPECT_EQ(roi_labels[2].class_id, 5);  // UNKNOWN
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
  const auto roi_labels =
    autoware::tensorrt_yolox::load_label_maps(label_path, "", roi_to_semseg_remap_path);

  // Assert
  ASSERT_EQ(roi_labels.size(), 3);
  EXPECT_EQ(roi_labels[0].semseg_id, 0);  // CAR
  EXPECT_EQ(roi_labels[1].semseg_id, 1);  // PEDESTRIAN
  EXPECT_EQ(roi_labels[2].semseg_id, 2);  // UNKNOWN
}

// a class name present in the label file but absent from a non-empty remap throws (likely wrong
// model)
TEST(LoadLabelMaps, ThrowsWhenRoiRemapIsMissingLabel)
{
  // Arrange
  const std::string label_path = write_temp_file(
    "label_missing_remap.txt",
    "CAR\n"
    "PEDESTRIAN\n"
    "UNKNOWN\n");
  const std::string roi_remap_path = write_temp_file(
    "label_remap_missing_entry.csv",
    "from, to\n"
    "CAR, 0\n"
    "PEDESTRIAN, 1\n");  // UNKNOWN is intentionally absent

  // Act / Assert
  EXPECT_THROW(
    autoware::tensorrt_yolox::load_label_maps(label_path, roi_remap_path, ""), std::runtime_error);
}

// load_segmentation_colormap parses the segmentation color map file
TEST(LoadSegmentationColormap, ParsesColorMap)
{
  // Arrange
  const std::string color_map_path = write_temp_file(
    "semseg_col_map_with_spaces.csv",
    "id,name,r,g,b\n"
    "0,others,0,1,2\n"
    "1, building ,70,75,80\n"
    " 2, wall, 150, 160, 170\n");

  // Act
  const auto semseg_color_map =
    autoware::tensorrt_yolox::load_segmentation_colormap(color_map_path);

  // Assert
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

// load_image_list returns the paths listed in the file
TEST(LoadImageList, ReturnsListedPaths)
{
  // Arrange
  const std::filesystem::path image_path =
    std::filesystem::temp_directory_path() / "load_image_list_existing.png";
  std::ofstream(image_path).close();
  const std::string list_path =
    write_temp_file("image_list_existing.txt", image_path.string() + "\n");

  // Act
  const auto image_list = autoware::tensorrt_yolox::load_image_list(list_path);

  // Assert
  ASSERT_EQ(image_list.size(), 1u);
  EXPECT_EQ(image_list[0], image_path.string());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
