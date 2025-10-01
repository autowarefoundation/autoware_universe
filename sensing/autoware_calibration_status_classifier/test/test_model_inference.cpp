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

#include "data_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::calibration_status_classifier
{

class ModelInferenceTest : public autoware::cuda_utils::CudaTest
{
protected:
  static std::vector<data_utils::TestSample> samples;
  static std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr;
  static autoware::cuda_utils::CudaUniquePtr<float[]> in_d;
  static autoware::cuda_utils::CudaUniquePtr<float[]> out_d;
  static cudaStream_t stream;
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

    CHECK_CUDA_ERROR(cudaStreamCreate(&stream));

    in_d = autoware::cuda_utils::make_unique<float[]>(
      data_utils::channels * data_utils::height * data_utils::width);
    out_d = autoware::cuda_utils::make_unique<float[]>(2);

    tensorrt_common::TrtCommonConfig trt_config(onnx_path, "fp16");

    std::vector<autoware::tensorrt_common::NetworkIO> network_io{
      autoware::tensorrt_common::NetworkIO("input", {4, {1, data_utils::channels, -1, -1}}),
      autoware::tensorrt_common::NetworkIO("output", {2, {1, 2}})};

    std::vector<autoware::tensorrt_common::ProfileDims> profile_dims{
      autoware::tensorrt_common::ProfileDims(
        "input", {4, {1, data_utils::channels, data_utils::height - 1, data_utils::width - 1}},
        {4, {1, data_utils::channels, data_utils::height, data_utils::width}},
        {4, {1, data_utils::channels, data_utils::height + 1, data_utils::width + 1}})};

    auto network_io_ptr =
      std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io);
    auto profile_dims_ptr =
      std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims);

    network_trt_ptr = std::make_unique<autoware::tensorrt_common::TrtCommon>(trt_config);
    if (!network_trt_ptr->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
      throw std::runtime_error("Failed to setup CalibrationStatusClassifier TensorRT engine.");
    }
  }

  static void TearDownTestSuite()
  {
    cudaDeviceSynchronize();
    network_trt_ptr.reset();
    in_d.reset();
    out_d.reset();
    if (stream) {
      cudaStreamDestroy(stream);
    }
  }
};

std::unique_ptr<autoware::tensorrt_common::TrtCommon> ModelInferenceTest::network_trt_ptr;
autoware::cuda_utils::CudaUniquePtr<float[]> ModelInferenceTest::in_d;
autoware::cuda_utils::CudaUniquePtr<float[]> ModelInferenceTest::out_d;
cudaStream_t ModelInferenceTest::stream;
std::filesystem::path ModelInferenceTest::data_dir;
std::vector<data_utils::TestSample> ModelInferenceTest::samples;

TEST_F(ModelInferenceTest, TestModelInferenceCalibratedSamples)
{
  if (!network_trt_ptr) {
    GTEST_SKIP() << "Network TensorRT instance is not initialized due to missing ONNX model.";
  }

  for (const auto & sample : samples) {
    cuda_utils::clear_async(
      in_d.get(), data_utils::channels * data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(out_d.get(), 2, stream);

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      in_d.get(), sample.input_data_calibrated.data(),
      sizeof(float) * data_utils::channels * data_utils::height * data_utils::width,
      cudaMemcpyHostToDevice, stream));

    network_trt_ptr->setTensor(
      "input", in_d.get(), {4, {1, data_utils::channels, data_utils::height, data_utils::width}});
    network_trt_ptr->setTensor("output", out_d.get());
    network_trt_ptr->enqueueV3(stream);

    std::vector<float> output(2);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      output.data(), out_d.get(), sizeof(float) * 2, cudaMemcpyDeviceToHost, stream));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));

    auto miscalibration_confidence = output.at(0);
    auto calibration_confidence = output.at(1);
    auto is_calibrated = calibration_confidence > miscalibration_confidence;

    EXPECT_TRUE(is_calibrated)
      << "Calibration status should be true for calibrated sample. Calibration confidence: "
      << calibration_confidence << ", Miscalibration confidence: " << miscalibration_confidence;
    GTEST_LOG_(INFO) << "Sample: " << sample.sample_name
                     << ", Calibration confidence: " << calibration_confidence
                     << ", Miscalibration confidence: " << miscalibration_confidence;
  }
}

TEST_F(ModelInferenceTest, TestModelInferenceMiscalibratedSamples)
{
  if (!network_trt_ptr) {
    GTEST_SKIP() << "Network TensorRT instance is not initialized due to missing ONNX model.";
  }

  for (const auto & sample : samples) {
    cuda_utils::clear_async(
      in_d.get(), data_utils::channels * data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(out_d.get(), 2, stream);

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      in_d.get(), sample.input_data_miscalibrated.data(),
      sizeof(float) * data_utils::channels * data_utils::height * data_utils::width,
      cudaMemcpyHostToDevice, stream));

    network_trt_ptr->setTensor(
      "input", in_d.get(), {4, {1, data_utils::channels, data_utils::height, data_utils::width}});
    network_trt_ptr->setTensor("output", out_d.get());
    network_trt_ptr->enqueueV3(stream);

    std::vector<float> output(2);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      output.data(), out_d.get(), sizeof(float) * 2, cudaMemcpyDeviceToHost, stream));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));

    auto miscalibration_confidence = output.at(0);
    auto calibration_confidence = output.at(1);
    auto is_calibrated = calibration_confidence > miscalibration_confidence;

    EXPECT_FALSE(is_calibrated)
      << "Calibration status should be false for miscalibrated sample. Calibration confidence: "
      << calibration_confidence << ", Miscalibration confidence: " << miscalibration_confidence;
    GTEST_LOG_(INFO) << "Sample: " << sample.sample_name
                     << ", Calibration confidence: " << calibration_confidence
                     << ", Miscalibration confidence: " << miscalibration_confidence;
  }
}

}  // namespace autoware::calibration_status_classifier

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
