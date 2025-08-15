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

#include "autoware/calibration_status/calibration_status.hpp"
#include "data_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

namespace autoware::calibration_status
{
constexpr bool save_test_images = true;

class CalibrationStatusTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override;
  void TearDown() override;
  static std::vector<data_utils::TestSample> samples;
  static std::unique_ptr<CalibrationStatus> calibration_status;
  static std::filesystem::path data_dir;

  static void SetUpTestSuite()
  {
    data_dir = std::filesystem::path(
                 ament_index_cpp::get_package_share_directory("autoware_calibration_status")) /
               "data";

    samples.push_back(data_utils::load_test_sample(data_dir, "sample_102"));

    const char * home_env = std::getenv("HOME");
    std::filesystem::path home_path = home_env ? home_env : "/";
    std::filesystem::path onnx_path =
      home_path / "autoware_data/calibration_status/calibration_status.onnx";
    if (!std::filesystem::exists(onnx_path)) {
      GTEST_SKIP() << "ONNX model file not found: " << onnx_path;
    }
    CalibrationStatusConfig calibration_status_config(128.0, 1);
    calibration_status =
      std::make_unique<CalibrationStatus>(onnx_path.string(), calibration_status_config);
  }
  static void TearDownTestSuite() { calibration_status.reset(); }
};

void CalibrationStatusTest::SetUp()
{
}

void CalibrationStatusTest::TearDown()
{
}

std::unique_ptr<CalibrationStatus> CalibrationStatusTest::calibration_status;
std::filesystem::path CalibrationStatusTest::data_dir;
std::vector<data_utils::TestSample> CalibrationStatusTest::samples;

TEST_F(CalibrationStatusTest, TestCalibrationStatusProcessingCalibratedSamples)
{
  if (!calibration_status) {
    GTEST_SKIP() << "CalibrationStatus instance is not initialized due to missing ONNX model.";
  }

  for (const auto & sample : samples) {
    auto preview_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    preview_img_msg->header = sample.image->header;
    preview_img_msg->height = sample.image->height;
    preview_img_msg->width = sample.image->width;
    preview_img_msg->encoding = sample.image->encoding;
    preview_img_msg->step = sample.image->step;
    preview_img_msg->is_bigendian = sample.image->is_bigendian;
    preview_img_msg->data.resize(sample.image->data.size());
    uint8_t * preview_img_data = preview_img_msg->data.data();
    auto result = calibration_status->process(
      sample.pointcloud, sample.image_undistorted, sample.camera_info_calibrated,
      sample.lidar_to_camera_tf_calibrated, preview_img_data);
    if (save_test_images) {
      data_utils::save_img(
        preview_img_msg->data, preview_img_msg->width, preview_img_msg->height, data_dir,
        sample.sample_name + "_fused_calibrated.png");
    }
    EXPECT_TRUE(result.is_calibrated)
      << "Calibration status should be true for calibrated sample. Calibration confidence: "
      << result.calibration_confidence
      << ", Miscalibration confidence: " << result.miscalibration_confidence;
    GTEST_LOG_(INFO) << "Calibration confidence: " << result.calibration_confidence
                     << ", Miscalibration confidence: " << result.miscalibration_confidence;
  }
}

TEST_F(CalibrationStatusTest, TestCalibrationStatusProcessingMisalibratedSamples)
{
  if (!calibration_status) {
    GTEST_SKIP() << "CalibrationStatus instance is not initialized due to missing ONNX model.";
  }

  for (const auto & sample : samples) {
    auto preview_img_msg = std::make_shared<sensor_msgs::msg::Image>();
    preview_img_msg->header = sample.image->header;
    preview_img_msg->height = sample.image->height;
    preview_img_msg->width = sample.image->width;
    preview_img_msg->encoding = sample.image->encoding;
    preview_img_msg->step = sample.image->step;
    preview_img_msg->is_bigendian = sample.image->is_bigendian;
    preview_img_msg->data.resize(sample.image->data.size());
    uint8_t * preview_img_data = preview_img_msg->data.data();
    auto result = calibration_status->process(
      sample.pointcloud, sample.image_undistorted, sample.camera_info_miscalibrated,
      sample.lidar_to_camera_tf_miscalibrated, preview_img_data);
    if (save_test_images) {
      data_utils::save_img(
        preview_img_msg->data, preview_img_msg->width, preview_img_msg->height, data_dir,
        sample.sample_name + "_fused_miscalibrated.png");
    }
    EXPECT_FALSE(result.is_calibrated)
      << "Calibration status should be false for miscalibrated sample. Calibration confidence: "
      << result.calibration_confidence
      << ", Miscalibration confidence: " << result.miscalibration_confidence;
    GTEST_LOG_(INFO) << "Calibration confidence: " << result.calibration_confidence
                     << ", Miscalibration confidence: " << result.miscalibration_confidence;
  }
}

}  // namespace autoware::calibration_status

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
