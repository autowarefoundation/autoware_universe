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

#include "autoware/tensorrt_yolox/label.hpp"
#include "autoware/tensorrt_yolox/tensorrt_yolox_detector.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// cspell: ignore semseg

// Unit test for TrtYoloXDetector::detect(). Unlike test_tensorrt_yolox_node.cpp (which drives the
// node end-to-end over ROS topics), this exercises the detector class directly: the config is
// assembled in-process and detect() is called with an image message, so no executor / topic
// plumbing is involved.
//
// The same two model configurations as the node test are used:
//   * traffic-light detector (yolox_s_car_ped_tl_detector): detection only, no segmentation head.
//   * general detector + segmentation (yolox-sPlus-opt ... seg16cls): a "multitask" model, so
//   detect()
//     additionally fills in the segmentation mask and (optionally) a colorized mask.
//
// detect() owns the TensorRT engine, so the test requires an NVIDIA GPU, TensorRT and the ONNX
// models (downloaded under autoware_data). When any of these are missing the test self-skips
// (GTEST_SKIP), and in CMake it is additionally gated behind `TRT_AVAIL AND CUDA_AVAIL`.
//
// Detection outputs (box coordinates / scores) are not bit-reproducible across GPU / TensorRT
// versions, so the assertions are intentionally coarse: presence, class, ROI within bounds. Exact
// pixel coordinates are deliberately not asserted.

namespace
{
using autoware::tensorrt_yolox::TrtYoloXDetector;
using autoware::tensorrt_yolox::TrtYoloXDetectorConfig;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;

// Class ID after remapping by traffic_light_roi_label_remap.csv.
constexpr uint8_t vehicle_traffic_light_label = 1;

// Detection score threshold matching the node configuration (see the *.param.yaml files).
constexpr float score_threshold = 0.35f;

// Resolve a file shipped under autoware_data. The directory layout differs between setups
// (`autoware_data/tensorrt_yolox` vs `autoware_data/ml_models/tensorrt_yolox`), and an explicit
// override via YOLOX_TEST_DATA_DIR takes precedence.
std::string resolve_autoware_data_file(const std::string & filename)
{
  std::vector<std::string> candidate_dirs;
  if (const char * override_dir = std::getenv("YOLOX_TEST_DATA_DIR")) {
    candidate_dirs.emplace_back(override_dir);
  }
  if (const char * home = std::getenv("HOME")) {
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/tensorrt_yolox");
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/ml_models/tensorrt_yolox");
  }
  for (const auto & dir : candidate_dirs) {
    const std::string candidate = dir + "/" + filename;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return "";
}

std::string config_file(const std::string & filename)
{
  return ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") + "/config/" +
         filename;
}

// Builder settings shared by every configuration (TensorRT builder knobs and GPU selection).
void append_common_config(TrtYoloXDetectorConfig & config)
{
  config.precision = "fp16";
  config.score_threshold = score_threshold;
  config.nms_threshold = 0.7f;
  config.calibration_algorithm = "Entropy";
  config.dla_core_id = -1;
  config.quantize_first_layer = false;
  config.quantize_last_layer = false;
  config.profile_per_layer = false;
  config.clip_value = 6.0;
  config.calibration_image_list_path = "";
  config.gpu_id = 0;
  config.overlap_roi_score_threshold = 0.3f;
}

// Config for the traffic-light detector model (detection only, no segmentation).
// Without the ROI remap file every class maps to "unmapped" and all detections are dropped, so the
// remap path is mandatory to exercise the detection output.
TrtYoloXDetectorConfig make_traffic_light_detector_config(
  const std::string & model_path, const std::string & label_path)
{
  TrtYoloXDetectorConfig config;
  config.model_path = model_path;
  config.roi_labels = autoware::tensorrt_yolox::load_label_maps(
    label_path, config_file("traffic_light_roi_label_remap.csv"), "");
  config.semseg_color_map = autoware::tensorrt_yolox::load_segmentation_colormap("");
  config.is_roi_overlap_semseg = false;
  config.is_publish_color_mask = false;
  append_common_config(config);
  return config;
}

// Config for the multitask model (general detection + semantic segmentation), with both the
// segmentation mask and the colorized mask enabled. fp16 is used instead of the launch's int8 to
// avoid the int8 calibration-cache dependency; the exercised code is precision independent.
TrtYoloXDetectorConfig make_segmentation_config(
  const std::string & model_path, const std::string & label_path,
  const std::string & semantic_segmentation_color_map_path)
{
  TrtYoloXDetectorConfig config;
  config.model_path = model_path;
  config.roi_labels = autoware::tensorrt_yolox::load_label_maps(
    label_path, config_file("roi_label_remap.csv"), config_file("roi_to_semseg_label_remap.csv"));
  config.semseg_color_map =
    autoware::tensorrt_yolox::load_segmentation_colormap(semantic_segmentation_color_map_path);
  config.is_roi_overlap_semseg = true;
  config.is_publish_color_mask = true;
  append_common_config(config);
  return config;
}

sensor_msgs::msg::Image to_image_msg(const cv::Mat & bgr_image)
{
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  return *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, bgr_image).toImageMsg();
}

cv::Mat make_black_image(int width, int height)
{
  return cv::Mat::zeros(height, width, CV_8UC3);
}

cv::Mat load_test_image()
{
  const std::string path = ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") +
                           "/test/test_image.jpg";
  const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.empty()) {
    throw std::runtime_error("failed to load test image: " + path);
  }
  return image;
}

size_t count_objects_with_label(const DetectedObjectsWithFeature & objects, uint8_t label)
{
  size_t count = 0;
  for (const auto & feature_object : objects.feature_objects) {
    if (
      !feature_object.object.classification.empty() &&
      feature_object.object.classification.front().label == label) {
      ++count;
    }
  }
  return count;
}

bool all_rois_within_image(
  const DetectedObjectsWithFeature & objects, uint32_t width, uint32_t height)
{
  for (const auto & feature_object : objects.feature_objects) {
    const auto & roi = feature_object.feature.roi;
    if (roi.x_offset + roi.width > width || roi.y_offset + roi.height > height) {
      return false;
    }
  }
  return true;
}

// Construct the detector under test. Returns nullptr when GPU initialization fails: the constructor
// throws std::runtime_error in that case. The first call for a given model builds the TensorRT
// engine (a few minutes) before returning.
std::unique_ptr<TrtYoloXDetector> make_detector(const TrtYoloXDetectorConfig & config)
{
  try {
    return std::make_unique<TrtYoloXDetector>(config);
  } catch (const std::exception &) {
    return nullptr;
  }
}
}  // namespace

// A blank (all-black) image yields an inference pass with no detections above the threshold, so
// detect() succeeds and returns an empty object list.
TEST(TrtYoloXDetectorTest, DetectReturnsEmptyObjectsForBlankImage)
{
  // Arrange
  const std::string model_path =
    resolve_autoware_data_file("yolox_s_car_ped_tl_detector_960_960_batch_1.onnx");
  const std::string label_path = resolve_autoware_data_file("car_ped_tl_detector_labels.txt");
  if (model_path.empty() || label_path.empty()) {
    GTEST_SKIP() << "traffic-light detector model/label not found under autoware_data.";
  }
  const auto detector = make_detector(make_traffic_light_detector_config(model_path, label_path));
  if (!detector) {
    GTEST_SKIP() << "GPU is not available for inference.";
  }
  const auto image_msg = to_image_msg(make_black_image(1280, 827));

  // Act
  const auto result = detector->detect(image_msg);

  // Assert
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_TRUE(result->objects.feature_objects.empty());
  EXPECT_FALSE(result->mask.has_value());
  EXPECT_FALSE(result->color_mask.has_value());
}

// Longest detection path: a real camera image containing traffic lights drives the full detect()
// loop (inference -> per-object classification remap -> non-empty object list). The output header
// is propagated from the input message.
TEST(TrtYoloXDetectorTest, DetectFindsTrafficLightInRealImage)
{
  // Arrange
  const std::string model_path =
    resolve_autoware_data_file("yolox_s_car_ped_tl_detector_960_960_batch_1.onnx");
  const std::string label_path = resolve_autoware_data_file("car_ped_tl_detector_labels.txt");
  if (model_path.empty() || label_path.empty()) {
    GTEST_SKIP() << "traffic-light detector model/label not found under autoware_data.";
  }
  const auto detector = make_detector(make_traffic_light_detector_config(model_path, label_path));
  if (!detector) {
    GTEST_SKIP() << "GPU is not available for inference.";
  }
  const cv::Mat image = load_test_image();
  const auto image_msg = to_image_msg(image);

  // Act
  const auto result = detector->detect(image_msg);

  // Assert
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_FALSE(result->objects.feature_objects.empty());
  EXPECT_TRUE(all_rois_within_image(
    result->objects, static_cast<uint32_t>(image.cols), static_cast<uint32_t>(image.rows)));
  EXPECT_GE(count_objects_with_label(result->objects, vehicle_traffic_light_label), 1u);
  EXPECT_EQ(result->objects.header.frame_id, image_msg.header.frame_id);
  EXPECT_FALSE(result->mask.has_value());
}

// Longest segmentation path: the multitask model additionally fills in a segmentation mask and a
// colorized mask, exercising the segmentation branches of detect() (mask encode, ROI overlay and
// getColorizedMask) that the detection-only model never reaches.
TEST(TrtYoloXDetectorTest, DetectProducesSegmentationMaskForMultitaskModel)
{
  // Arrange
  const std::string model_path =
    resolve_autoware_data_file("yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls.onnx");
  const std::string label_path = resolve_autoware_data_file("label.txt");
  const std::string color_map_path = resolve_autoware_data_file("semseg_color_map.csv");
  if (model_path.empty() || label_path.empty() || color_map_path.empty()) {
    GTEST_SKIP() << "segmentation model/label/color-map not found under autoware_data.";
  }
  const auto detector =
    make_detector(make_segmentation_config(model_path, label_path, color_map_path));
  if (!detector) {
    GTEST_SKIP() << "GPU is not available for inference.";
  }
  const cv::Mat image = load_test_image();
  const auto image_msg = to_image_msg(image);

  // Act
  const auto result = detector->detect(image_msg);

  // Assert
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_FALSE(result->objects.feature_objects.empty());
  EXPECT_TRUE(all_rois_within_image(
    result->objects, static_cast<uint32_t>(image.cols), static_cast<uint32_t>(image.rows)));

  ASSERT_TRUE(result->mask.has_value()) << "multitask model did not produce a segmentation mask";
  EXPECT_GT(result->mask->width, 0u);
  EXPECT_GT(result->mask->height, 0u);

  ASSERT_TRUE(result->color_mask.has_value()) << "color mask was not produced";
  EXPECT_EQ(result->color_mask->encoding, sensor_msgs::image_encodings::BGR8);
  EXPECT_GT(result->color_mask->width, 0u);
  EXPECT_GT(result->color_mask->height, 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
