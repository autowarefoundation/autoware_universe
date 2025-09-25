// Copyright 2025 TIER IV, Inc.
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

#include "autoware/calibration_status_classifier/calibration_status_classifier.hpp"
#include "data_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{
constexpr bool save_test_images = true;
constexpr double max_depth = 128.0;
constexpr int64_t dilation_size = 1;
constexpr int64_t cloud_capacity = 2'000'000;

class CalibrationStatusClassifierTest : public autoware::cuda_utils::CudaTest
{
protected:
  static std::vector<data_utils::TestSample> samples;
  static std::unique_ptr<CalibrationStatusClassifier> calibration_status_classifier;
  static std::filesystem::path data_dir;

  static void SetUpTestSuite()
  {
    const char * home_env = std::getenv("HOME");
    std::filesystem::path home_path = home_env ? home_env : "/";
    std::filesystem::path onnx_path =
      home_path / "autoware_data/calibration_status_classifier/calibration_status_classifier.onnx";
    if (!std::filesystem::exists(onnx_path)) {
      GTEST_SKIP() << "ONNX model file not found: " << onnx_path;
    }

    data_dir =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("autoware_calibration_status_classifier")) /
      "data";
    samples.push_back(data_utils::load_test_sample(data_dir, "sample_102"));

    CalibrationStatusClassifierConfig calibration_status_classifier_config(
      max_depth, dilation_size,
      std::vector<int64_t>{data_utils::height - 1, data_utils::height, data_utils::height + 1},
      std::vector<int64_t>{
        data_utils::width - 1, data_utils::width, data_utils::width + 1});  // Dummy shape diffs
    calibration_status_classifier = std::make_unique<CalibrationStatusClassifier>(
      onnx_path.string(), "fp16", cloud_capacity, calibration_status_classifier_config);
  }

  static void TearDownTestSuite()
  {
    cudaDeviceSynchronize();
    calibration_status_classifier.reset();
  }
};

std::unique_ptr<CalibrationStatusClassifier>
  CalibrationStatusClassifierTest::calibration_status_classifier;
std::filesystem::path CalibrationStatusClassifierTest::data_dir;
std::vector<data_utils::TestSample> CalibrationStatusClassifierTest::samples;

TEST_F(CalibrationStatusClassifierTest, TestCalibrationStatusClassifierProcessingCalibratedSamples)
{
  if (!calibration_status_classifier) {
    GTEST_SKIP()
      << "CalibrationStatusClassifier instance is not initialized due to missing ONNX model.";
  }

  for (const auto & sample : samples) {
    auto preview_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    preview_img_msg->header = sample.image_bgr->header;
    preview_img_msg->height = sample.image_bgr->height;
    preview_img_msg->width = sample.image_bgr->width;
    preview_img_msg->encoding = sample.image_bgr->encoding;
    preview_img_msg->step = sample.image_bgr->step;
    preview_img_msg->is_bigendian = sample.image_bgr->is_bigendian;
    preview_img_msg->data.resize(sample.image_bgr->data.size());
    uint8_t * preview_img_data = preview_img_msg->data.data();
    auto result = calibration_status_classifier->process(
      sample.pointcloud, sample.image_bgr, sample.camera_lidar_info_calibrated, preview_img_data);
    if (save_test_images) {
      data_utils::save_img(
        preview_img_msg->data, preview_img_msg->width, preview_img_msg->height, data_dir,
        sample.sample_name + "_fused_calibrated.png", CV_8UC3, preview_img_msg->encoding);
    }
    auto is_calibrated = result.calibration_confidence > result.miscalibration_confidence;
    EXPECT_TRUE(is_calibrated)
      << "Calibration status should be true for calibrated sample. Calibration confidence: "
      << result.calibration_confidence
      << ", Miscalibration confidence: " << result.miscalibration_confidence;
    GTEST_LOG_(INFO) << "Sample: " << sample.sample_name
                     << ", Calibration confidence: " << result.calibration_confidence
                     << ", Miscalibration confidence: " << result.miscalibration_confidence;
  }
}

TEST_F(
  CalibrationStatusClassifierTest, TestCalibrationStatusClassifierProcessingMiscalibratedSamples)
{
  if (!calibration_status_classifier) {
    GTEST_SKIP()
      << "CalibrationStatusClassifier instance is not initialized due to missing ONNX model.";
  }

  for (const auto & sample : samples) {
    auto preview_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    preview_img_msg->header = sample.image_bgr->header;
    preview_img_msg->height = sample.image_bgr->height;
    preview_img_msg->width = sample.image_bgr->width;
    preview_img_msg->encoding = sample.image_bgr->encoding;
    preview_img_msg->step = sample.image_bgr->step;
    preview_img_msg->is_bigendian = sample.image_bgr->is_bigendian;
    preview_img_msg->data.resize(sample.image_bgr->data.size());
    uint8_t * preview_img_data = preview_img_msg->data.data();
    auto result = calibration_status_classifier->process(
      sample.pointcloud, sample.image_bgr, sample.camera_lidar_info_miscalibrated,
      preview_img_data);
    if (save_test_images) {
      data_utils::save_img(
        preview_img_msg->data, preview_img_msg->width, preview_img_msg->height, data_dir,
        sample.sample_name + "_fused_miscalibrated.png", CV_8UC3, preview_img_msg->encoding);
    }
    auto is_calibrated = result.calibration_confidence > result.miscalibration_confidence;
    EXPECT_FALSE(is_calibrated)
      << "Calibration status should be false for miscalibrated sample. Calibration confidence: "
      << result.calibration_confidence
      << ", Miscalibration confidence: " << result.miscalibration_confidence;
    GTEST_LOG_(INFO) << "Sample: " << sample.sample_name
                     << ", Calibration confidence: " << result.calibration_confidence
                     << ", Miscalibration confidence: " << result.miscalibration_confidence;
  }
}

}  // namespace autoware::calibration_status_classifier

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
