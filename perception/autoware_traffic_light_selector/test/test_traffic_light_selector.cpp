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

#include "../src/traffic_light_selector.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::traffic_light::select;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::RegionOfInterest;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

namespace
{

RegionOfInterest make_roi(uint32_t x_offset, uint32_t y_offset, uint32_t width, uint32_t height)
{
  RegionOfInterest roi;
  roi.x_offset = x_offset;
  roi.y_offset = y_offset;
  roi.width = width;
  roi.height = height;
  return roi;
}

void expect_same(const TrafficLightRoiArray & output, const TrafficLightRoiArray & expected_output)
{
  ASSERT_EQ(output.rois.size(), expected_output.rois.size());
  for (size_t roi_index = 0; roi_index < output.rois.size(); ++roi_index) {
    const auto & output_roi = output.rois.at(roi_index);
    const auto & expected_roi = expected_output.rois.at(roi_index);
    EXPECT_EQ(output_roi.traffic_light_id, expected_roi.traffic_light_id);
    EXPECT_EQ(output_roi.traffic_light_type, expected_roi.traffic_light_type);
    EXPECT_EQ(output_roi.roi.x_offset, expected_roi.roi.x_offset);
    EXPECT_EQ(output_roi.roi.y_offset, expected_roi.roi.y_offset);
    EXPECT_EQ(output_roi.roi.width, expected_roi.roi.width);
    EXPECT_EQ(output_roi.roi.height, expected_roi.roi.height);
  }
}

CameraInfo make_camera_info(uint32_t width, uint32_t height)
{
  CameraInfo camera_info;
  camera_info.width = width;
  camera_info.height = height;
  return camera_info;
}

DetectedObjectsWithFeature make_detected_rois(const std::vector<RegionOfInterest> & rois)
{
  DetectedObjectsWithFeature detected_rois;
  for (const auto & roi : rois) {
    DetectedObjectWithFeature feature_object;
    feature_object.feature.roi = roi;
    detected_rois.feature_objects.push_back(feature_object);
  }
  return detected_rois;
}

TrafficLightRoi make_traffic_light_roi(
  int64_t traffic_light_id, const RegionOfInterest & roi,
  uint8_t traffic_light_type = TrafficLightRoi::CAR_TRAFFIC_LIGHT)
{
  TrafficLightRoi traffic_light_roi;
  traffic_light_roi.traffic_light_id = traffic_light_id;
  traffic_light_roi.traffic_light_type = traffic_light_type;
  traffic_light_roi.roi = roi;
  return traffic_light_roi;
}

TrafficLightRoiArray make_traffic_light_roi_array(const std::vector<TrafficLightRoi> & rois)
{
  TrafficLightRoiArray traffic_light_roi_array;
  traffic_light_roi_array.rois = rois;
  return traffic_light_roi_array;
}

TrafficLightRoiArray select_helper(
  const std::vector<RegionOfInterest> & detected_rois,
  const std::vector<TrafficLightRoi> & rough_rois,
  const std::vector<TrafficLightRoi> & expected_rois, const CameraInfo & camera_info)
{
  return select(
    make_detected_rois(detected_rois), make_traffic_light_roi_array(rough_rois),
    make_traffic_light_roi_array(expected_rois), camera_info);
}

}  // namespace

// With no expected ROIs the output loop produces nothing, so the result is an empty array.
TEST(TrafficLightSelector, EmptyInputsProduceEmptyOutput)
{
  // Arrange
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  const auto output = select_helper({}, {}, {}, camera_info);

  // Assert
  expect_same(output, make_traffic_light_roi_array({}));
}

// Every expected ROI yields exactly one output entry. With no detections it cannot be matched, so
// the entry carries the expected ID with a default (empty) ROI.
TEST(TrafficLightSelector, ExpectedRoiWithoutDetectionsOutputsDefaultRoi)
{
  // Arrange
  const int64_t traffic_light_id = 123;
  const auto expected_roi = make_traffic_light_roi(traffic_light_id, make_roi(100, 100, 40, 40));
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  const auto output = select_helper({}, {}, {expected_roi}, camera_info);

  // Assert
  expect_same(
    output,
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, RegionOfInterest{})}));
}

// The detected ROI center lies outside the rough ROI, so it is never considered as a match and the
// output entry falls back to a default ROI.
TEST(TrafficLightSelector, DetectionCenterOutsideRoughRoiOutputsDefaultRoi)
{
  // Arrange
  const int64_t traffic_light_id = 123;
  const auto expected_roi = make_traffic_light_roi(traffic_light_id, make_roi(100, 100, 40, 40));
  const auto rough_roi = make_traffic_light_roi(traffic_light_id, make_roi(50, 50, 200, 200));
  // Detected ROI center (820, 620) is far outside the rough ROI span (50..250).
  const auto detected_roi = make_roi(800, 600, 40, 40);
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  const auto output = select_helper({detected_roi}, {rough_roi}, {expected_roi}, camera_info);

  // Assert
  expect_same(
    output,
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, RegionOfInterest{})}));
}

// The detected ROI center is inside the rough ROI and overlaps the expected ROI. The expected ROI
// is shifted onto the detection, so the detected ROI (not the expected one) is assigned to output.
TEST(TrafficLightSelector, DetectionInsideRoughRoiAssignsDetectedRoi)
{
  // Arrange
  const int64_t traffic_light_id = 123;
  const auto expected_roi = make_traffic_light_roi(traffic_light_id, make_roi(100, 100, 40, 40));
  const auto rough_roi = make_traffic_light_roi(traffic_light_id, make_roi(50, 50, 200, 200));
  // Detected ROI center (130, 125) is inside the rough ROI but offset from the expected ROI.
  const auto detected_roi = make_roi(110, 105, 40, 40);
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  const auto output = select_helper({detected_roi}, {rough_roi}, {expected_roi}, camera_info);

  // Assert
  expect_same(
    output, make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, detected_roi)}));
}

// The traffic_light_type of each expected ROI must be copied verbatim onto the output entry.
TEST(TrafficLightSelector, TrafficLightTypePropagatedToOutput)
{
  // Arrange
  const int64_t traffic_light_id = 123;
  const auto roi = make_roi(100, 100, 40, 40);
  const auto rough_roi = make_traffic_light_roi(traffic_light_id, make_roi(50, 50, 200, 200));
  const auto expected_roi =
    make_traffic_light_roi(traffic_light_id, roi, TrafficLightRoi::PEDESTRIAN_TRAFFIC_LIGHT);
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  const auto output = select_helper({roi}, {rough_roi}, {expected_roi}, camera_info);

  // Assert
  expect_same(output, make_traffic_light_roi_array({expected_roi}));
}

// When multiple detections sit inside the rough ROI, the one with the highest IoU against the
// (shifted) expected ROI is selected.
TEST(TrafficLightSelector, BestOverlappingDetectionIsSelected)
{
  // Arrange
  const int64_t traffic_light_id = 123;
  const auto expected_roi = make_traffic_light_roi(traffic_light_id, make_roi(100, 100, 40, 40));
  const auto rough_roi = make_traffic_light_roi(traffic_light_id, make_roi(50, 50, 200, 200));
  // Detection A perfectly overlaps the expected ROI; detection B only partially overlaps.
  const auto detected_roi_perfect = make_roi(100, 100, 40, 40);
  const auto detected_roi_partial = make_roi(200, 200, 10, 10);
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  const auto output = select_helper(
    {detected_roi_perfect, detected_roi_partial}, {rough_roi}, {expected_roi}, camera_info);

  // Assert
  expect_same(
    output,
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, detected_roi_perfect)}));
}

// Shifting the expected ROI onto the detection pushes it outside the (small) image, so
// getShiftedRoi clamps it to an empty ROI, no positive IoU accumulates, and the output falls back
// to a default ROI.
TEST(TrafficLightSelector, ShiftedRoiOutOfImageBoundsOutputsDefaultRoi)
{
  // Arrange
  const int64_t traffic_light_id = 123;
  const auto roi = make_roi(100, 100, 40, 40);
  const auto rough_roi = make_traffic_light_roi(traffic_light_id, make_roi(0, 0, 200, 200));
  const auto expected_roi = make_traffic_light_roi(traffic_light_id, roi);
  // The ROI at (100, 100, 40, 40) does not fit inside a 50x50 image.
  const auto camera_info = make_camera_info(50, 50);

  // Act
  const auto output = select_helper({roi}, {rough_roi}, {expected_roi}, camera_info);

  // Assert
  expect_same(
    output,
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, RegionOfInterest{})}));
}

// Several expected traffic lights are matched independently; each gets its own detection assigned.
TEST(TrafficLightSelector, MultipleExpectedRoisEachAssigned)
{
  // Arrange
  const int64_t first_traffic_light_id = 1;
  const int64_t second_traffic_light_id = 2;
  const auto first_roi = make_roi(100, 100, 40, 40);
  const auto second_roi = make_roi(300, 100, 40, 40);
  const auto rough_roi = make_roi(0, 0, 500, 500);
  const auto first_rough_roi = make_traffic_light_roi(first_traffic_light_id, rough_roi);
  const auto second_rough_roi = make_traffic_light_roi(second_traffic_light_id, rough_roi);
  const auto first_expected_roi = make_traffic_light_roi(first_traffic_light_id, first_roi);
  const auto second_expected_roi = make_traffic_light_roi(second_traffic_light_id, second_roi);
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  const auto output = select_helper(
    {first_roi, second_roi}, {first_rough_roi, second_rough_roi},
    {first_expected_roi, second_expected_roi}, camera_info);

  // Assert
  expect_same(output, make_traffic_light_roi_array({first_expected_roi, second_expected_roi}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
