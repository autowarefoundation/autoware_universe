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

#include "autoware/calibration_status/data_type.hpp"
#include "autoware/calibration_status/preprocess_cuda.hpp"
#include "data_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <sys/types.h>

#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

namespace autoware::calibration_status
{

constexpr bool save_test_images = true;
constexpr double lidar_range = 128.0;
constexpr int64_t dilation_size = 1;
constexpr int64_t cloud_capacity = 2'000'000;
constexpr float px_error_threshold_rgb = 0.01f;
constexpr float px_error_threshold_di = 0.1f;
constexpr size_t arr_error_threshold = data_utils::width * data_utils::height * 0.02;

class PreprocessingTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override;
  void TearDown() override;
  cudaStream_t stream;
  std::unique_ptr<PreprocessCuda> preprocess_ptr;
  autoware::cuda_utils::CudaUniquePtr<InputArrayRGBDI[]> in_d;
  autoware::cuda_utils::CudaUniquePtr<float[]> out_d;
  autoware::cuda_utils::CudaUniquePtr<InputPointType[]> cloud_d;
  autoware::cuda_utils::CudaUniquePtr<InputImageBGR8Type[]> image_d;
  autoware::cuda_utils::CudaUniquePtr<InputImageBGR8Type[]> image_undistorted_d;
  autoware::cuda_utils::CudaUniquePtr<double[]> dist_coeffs_d;
  autoware::cuda_utils::CudaUniquePtr<double[]> camera_matrix_d;
  autoware::cuda_utils::CudaUniquePtr<double[]> projection_matrix_d;
  autoware::cuda_utils::CudaUniquePtr<double[]> tf_matrix_d;

  static std::vector<data_utils::TestSample> samples;
  static std::filesystem::path data_dir;

  static void SetUpTestSuite()
  {
    data_dir = std::filesystem::path(
                 ament_index_cpp::get_package_share_directory("autoware_calibration_status")) /
               "data";

    samples.push_back(data_utils::load_test_sample(data_dir, "sample_102"));
  }
  static void TearDownTestSuite() {}
};

void PreprocessingTest::SetUp()
{
  cudaStreamCreate(&stream);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream));

  in_d = cuda_utils::make_unique<InputArrayRGBDI[]>(data_utils::height * data_utils::width);
  out_d = cuda_utils::make_unique<float[]>(2);
  cloud_d = cuda_utils::make_unique<InputPointType[]>(cloud_capacity);
  image_d = cuda_utils::make_unique<InputImageBGR8Type[]>(data_utils::height * data_utils::width);
  image_undistorted_d =
    cuda_utils::make_unique<InputImageBGR8Type[]>(data_utils::height * data_utils::width);
  dist_coeffs_d = cuda_utils::make_unique<double[]>(8);
  camera_matrix_d = cuda_utils::make_unique<double[]>(9);
  projection_matrix_d = cuda_utils::make_unique<double[]>(12);
  tf_matrix_d = cuda_utils::make_unique<double[]>(16);

  preprocess_ptr = std::make_unique<PreprocessCuda>(lidar_range, dilation_size, stream);
}

void PreprocessingTest::TearDown()
{
}

std::filesystem::path PreprocessingTest::data_dir;
std::vector<data_utils::TestSample> PreprocessingTest::samples;

TEST_F(PreprocessingTest, TestPreprocessing)
{
  for (const auto & sample : samples) {
    cuda_utils::clear_async(in_d.get(), data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(out_d.get(), 2, stream);
    cuda_utils::clear_async(cloud_d.get(), cloud_capacity, stream);
    cuda_utils::clear_async(image_d.get(), data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(
      image_undistorted_d.get(), data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(dist_coeffs_d.get(), 8, stream);
    cuda_utils::clear_async(camera_matrix_d.get(), 9, stream);
    cuda_utils::clear_async(projection_matrix_d.get(), 12, stream);
    cuda_utils::clear_async(tf_matrix_d.get(), 16, stream);

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      cloud_d.get(), sample.pointcloud->data.data(),
      sizeof(InputPointType) * sample.pointcloud->width * sample.pointcloud->height,
      cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      image_d.get(), sample.image->data.data(),
      sizeof(InputImageBGR8Type) * data_utils::height * data_utils::width, cudaMemcpyHostToDevice,
      stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      dist_coeffs_d.get(), sample.camera_info_calibrated->d.data(), sizeof(double) * 8,
      cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      camera_matrix_d.get(), sample.camera_info_calibrated->k.data(), sizeof(double) * 9,
      cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      projection_matrix_d.get(), sample.camera_info_calibrated->p.data(), sizeof(double) * 12,
      cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      tf_matrix_d.get(), sample.lidar_to_camera_tf_calibrated.data(), sizeof(double) * 16,
      cudaMemcpyHostToDevice, stream));

    CHECK_CUDA_ERROR(preprocess_ptr->undistortImage_launch(
      image_d.get(), dist_coeffs_d.get(), camera_matrix_d.get(), projection_matrix_d.get(),
      sample.image->width, sample.image->height, image_undistorted_d.get(), in_d.get()));
    auto num_points_projected_d = cuda_utils::make_unique<uint32_t>();
    cuda_utils::clear_async(num_points_projected_d.get(), 1, stream);
    CHECK_CUDA_ERROR(preprocess_ptr->projectPoints_launch(
      cloud_d.get(), image_undistorted_d.get(), tf_matrix_d.get(), projection_matrix_d.get(),
      sample.pointcloud->width * sample.pointcloud->height, sample.image->width,
      sample.image->height, in_d.get(), num_points_projected_d.get()));
    uint32_t num_points_projected = 0;
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &num_points_projected, num_points_projected_d.get(), sizeof(uint32_t), cudaMemcpyDeviceToHost,
      stream));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
    ASSERT_GT(num_points_projected, 0) << "Number of projected points should be greater than zero.";
    std::vector<InputArrayRGBDI> in_host(data_utils::height * data_utils::width);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      in_host.data(), in_d.get(), sizeof(InputArrayRGBDI) * data_utils::height * data_utils::width,
      cudaMemcpyDeviceToHost, stream));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
    std::vector<InputArrayRGBDI> input_rgbdi(data_utils::width * data_utils::height);
    ASSERT_EQ(sample.input_data_calibrated.size(), data_utils::channels * input_rgbdi.size());
    std::memcpy(
      input_rgbdi.data(), sample.input_data_calibrated.data(),
      sample.input_data_calibrated.size() * sizeof(float));

    size_t error_r{};
    size_t error_g{};
    size_t error_b{};
    size_t error_depth{};
    size_t error_intensity{};
    std::vector<uint8_t> ref_data_rgb(data_utils::height * data_utils::width * 3);
    std::vector<uint8_t> ref_data_depth(data_utils::height * data_utils::width);
    std::vector<uint8_t> ref_data_intensity(data_utils::height * data_utils::width);
    std::vector<uint8_t> res_data_rgb(data_utils::height * data_utils::width * 3);
    std::vector<uint8_t> res_data_depth(data_utils::height * data_utils::width);
    std::vector<uint8_t> res_data_intensity(data_utils::height * data_utils::width);

    GTEST_LOG_(INFO) << "Comparing processing results with reference data (result/reference)...";
    for (size_t i = 0; i < input_rgbdi.size(); ++i) {
      const auto & res = in_host.at(i);
      const auto & ref = input_rgbdi.at(i);

      auto r_diff = std::fabs(res.r - ref.r);
      auto g_diff = std::fabs(res.g - ref.g);
      auto b_diff = std::fabs(res.b - ref.b);

      // 3X3 kernel depth & intensity diff
      float kernel_avg_ref_depth = 0.0;
      float kernel_avg_res_depth = 0.0;
      float kernel_avg_ref_intensity = 0.0;
      float kernel_avg_res_intensity = 0.0;
      const int center_x = static_cast<int>(i) % static_cast<int>(data_utils::width);
      const int center_y = static_cast<int>(i) / static_cast<int>(data_utils::width);
      for (int v = -1; v <= 1; ++v) {
        for (int u = -1; u <= 1; ++u) {
          const int neighbor_x = center_x + u;
          const int neighbor_y = center_y + v;

          if (
            neighbor_x >= 0 && neighbor_x < static_cast<int>(data_utils::width) &&
            neighbor_y >= 0 && neighbor_y < static_cast<int>(data_utils::height)) {
            const int neighbor_idx = neighbor_y * static_cast<int>(data_utils::width) + neighbor_x;

            const auto & res_neighbor = in_host.at(neighbor_idx);
            const auto & ref_neighbor = input_rgbdi.at(neighbor_idx);

            kernel_avg_ref_depth += ref_neighbor.depth;
            kernel_avg_res_depth += res_neighbor.depth;
            kernel_avg_ref_intensity += ref_neighbor.intensity;
            kernel_avg_res_intensity += res_neighbor.intensity;
          }
        }
      }
      kernel_avg_ref_depth /= 9.0f;
      kernel_avg_res_depth /= 9.0f;
      kernel_avg_ref_intensity /= 9.0f;
      kernel_avg_res_intensity /= 9.0f;
      auto depth_diff = std::fabs(kernel_avg_res_depth - kernel_avg_ref_depth);
      auto intensity_diff = std::fabs(kernel_avg_res_intensity - kernel_avg_ref_intensity);

      if (r_diff > px_error_threshold_rgb) error_r++;
      if (g_diff > px_error_threshold_rgb) error_g++;
      if (b_diff > px_error_threshold_rgb) error_b++;
      if (depth_diff > px_error_threshold_di) error_depth++;
      if (intensity_diff > px_error_threshold_di) error_intensity++;

      if (save_test_images) {
        res_data_rgb.at(i * 3) = static_cast<uint8_t>(res.r * 255.0f);
        res_data_rgb.at(i * 3 + 1) = static_cast<uint8_t>(res.g * 255.0f);
        res_data_rgb.at(i * 3 + 2) = static_cast<uint8_t>(res.b * 255.0f);
        res_data_depth.at(i) = static_cast<uint8_t>(res.depth * 255.0f);
        res_data_intensity.at(i) = static_cast<uint8_t>(res.intensity * 255.0f);
        ref_data_rgb.at(i * 3) = static_cast<uint8_t>(ref.r * 255.0f);
        ref_data_rgb.at(i * 3 + 1) = static_cast<uint8_t>(ref.g * 255.0f);
        ref_data_rgb.at(i * 3 + 2) = static_cast<uint8_t>(ref.b * 255.0f);
        ref_data_depth.at(i) = static_cast<uint8_t>(ref.depth * 255.0f);
        ref_data_intensity.at(i) = static_cast<uint8_t>(ref.intensity * 255.0f);
      }
    }

    if (save_test_images) {
      data_utils::save_img(
        ref_data_rgb, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_rgb_ref.png", CV_8UC3);
      data_utils::save_img(
        ref_data_depth, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_depth_ref.png", CV_8UC1);
      data_utils::save_img(
        ref_data_intensity, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_intensity_ref.png", CV_8UC1);
      data_utils::save_img(
        res_data_rgb, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_rgb_res.png", CV_8UC3);
      data_utils::save_img(
        res_data_depth, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_depth_res.png", CV_8UC1);
      data_utils::save_img(
        res_data_intensity, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_intensity_res.png", CV_8UC1);
    }

    GTEST_LOG_(INFO) << "Array errors: "
                     << "R: " << error_r << ", G: " << error_g << ", B: " << error_b
                     << ", Depth: " << error_depth << ", Intensity: " << error_intensity;
    EXPECT_LT(error_r, arr_error_threshold) << "R channel errors exceed threshold: " << error_r;
    EXPECT_LT(error_g, arr_error_threshold) << "G channel errors exceed threshold: " << error_g;
    EXPECT_LT(error_b, arr_error_threshold) << "B channel errors exceed threshold: " << error_b;
    EXPECT_LT(error_depth, arr_error_threshold) << "Depth errors exceed threshold: " << error_depth;
    EXPECT_LT(error_intensity, arr_error_threshold)
      << "Intensity errors exceed threshold: " << error_intensity;
  }
}

}  // namespace autoware::calibration_status

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
