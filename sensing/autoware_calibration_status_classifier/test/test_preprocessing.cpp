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

#include "autoware/calibration_status_classifier/data_type.hpp"
#include "autoware/calibration_status_classifier/preprocess_cuda.hpp"
#include "data_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{

constexpr bool save_test_images = true;
constexpr double max_depth = 128.0;
constexpr int64_t dilation_size = 1;
constexpr int64_t cloud_capacity = 2'000'000;
constexpr float px_error_threshold_rgb = 0.01f;
constexpr float px_error_threshold_di = 0.1f;
constexpr size_t arr_error_threshold = data_utils::width * data_utils::height * 0.02;

class PreprocessingTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override;
  cudaStream_t stream;
  std::unique_ptr<PreprocessCuda> preprocess_ptr;
  autoware::cuda_utils::CudaUniquePtr<float[]> in_d;
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
    data_dir =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("autoware_calibration_status_classifier")) /
      "data";
    samples.push_back(data_utils::load_test_sample(data_dir, "sample_102"));
  }

  static void TearDownTestSuite() { cudaDeviceSynchronize(); }
};

void PreprocessingTest::SetUp()
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();  // SetUp has been overridden, so this macro must be called here
  cudaStreamCreate(&stream);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream));

  in_d =
    cuda_utils::make_unique<float[]>(data_utils::channels * data_utils::height * data_utils::width);
  out_d = cuda_utils::make_unique<float[]>(2);
  cloud_d = cuda_utils::make_unique<InputPointType[]>(cloud_capacity);
  image_d = cuda_utils::make_unique<InputImageBGR8Type[]>(data_utils::height * data_utils::width);
  image_undistorted_d =
    cuda_utils::make_unique<InputImageBGR8Type[]>(data_utils::height * data_utils::width);
  dist_coeffs_d = cuda_utils::make_unique<double[]>(dist_coeffs_size);
  camera_matrix_d = cuda_utils::make_unique<double[]>(camera_matrix_size);
  projection_matrix_d = cuda_utils::make_unique<double[]>(projection_matrix_size);
  tf_matrix_d = cuda_utils::make_unique<double[]>(tf_matrix_size);

  preprocess_ptr = std::make_unique<PreprocessCuda>(
    max_depth, dilation_size, data_utils::width, data_utils::height, stream);
}

std::filesystem::path PreprocessingTest::data_dir;
std::vector<data_utils::TestSample> PreprocessingTest::samples;

TEST_F(PreprocessingTest, TestPreprocessing)
{
  for (const auto & sample : samples) {
    cuda_utils::clear_async(
      in_d.get(), data_utils::channels * data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(out_d.get(), 2, stream);
    cuda_utils::clear_async(cloud_d.get(), cloud_capacity, stream);
    cuda_utils::clear_async(image_d.get(), data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(
      image_undistorted_d.get(), data_utils::height * data_utils::width, stream);
    cuda_utils::clear_async(dist_coeffs_d.get(), dist_coeffs_size, stream);
    cuda_utils::clear_async(camera_matrix_d.get(), camera_matrix_size, stream);
    cuda_utils::clear_async(projection_matrix_d.get(), projection_matrix_size, stream);
    cuda_utils::clear_async(tf_matrix_d.get(), tf_matrix_size, stream);

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      cloud_d.get(), sample.pointcloud->data.data(),
      sizeof(InputPointType) * sample.pointcloud->width * sample.pointcloud->height,
      cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      image_d.get(), sample.image_bgr->data.data(),
      sizeof(InputImageBGR8Type) * data_utils::height * data_utils::width, cudaMemcpyHostToDevice,
      stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      dist_coeffs_d.get(), sample.camera_lidar_info_calibrated.d.data(),
      sizeof(double) * dist_coeffs_size, cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      camera_matrix_d.get(), sample.camera_lidar_info_calibrated.k.data(),
      sizeof(double) * camera_matrix_size, cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      projection_matrix_d.get(), sample.camera_lidar_info_calibrated.p.data(),
      sizeof(double) * projection_matrix_size, cudaMemcpyHostToDevice, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      tf_matrix_d.get(), sample.camera_lidar_info_calibrated.tf_camera_to_lidar.data(),
      sizeof(double) * tf_matrix_size, cudaMemcpyHostToDevice, stream));

    CHECK_CUDA_ERROR(preprocess_ptr->undistort_image_launch(
      image_d.get(), dist_coeffs_d.get(), camera_matrix_d.get(), projection_matrix_d.get(),
      sample.image_bgr->width, sample.image_bgr->height, image_undistorted_d.get(), in_d.get()));
    auto num_points_projected_d = cuda_utils::make_unique<uint32_t>();
    cuda_utils::clear_async(num_points_projected_d.get(), 1, stream);
    CHECK_CUDA_ERROR(preprocess_ptr->project_points_launch(
      cloud_d.get(), image_undistorted_d.get(), tf_matrix_d.get(), projection_matrix_d.get(),
      sample.pointcloud->width * sample.pointcloud->height, sample.image_bgr->width,
      sample.image_bgr->height, in_d.get(), num_points_projected_d.get()));
    uint32_t num_points_projected = 0;
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &num_points_projected, num_points_projected_d.get(), sizeof(uint32_t), cudaMemcpyDeviceToHost,
      stream));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
    ASSERT_GT(num_points_projected, 0) << "Number of projected points should be greater than zero.";
    std::vector<float> in_host(data_utils::channels * data_utils::height * data_utils::width);
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      in_host.data(), in_d.get(),
      sizeof(float) * data_utils::channels * data_utils::height * data_utils::width,
      cudaMemcpyDeviceToHost, stream));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
    std::vector<float> in_host_ref(data_utils::channels * data_utils::width * data_utils::height);
    ASSERT_EQ(sample.input_data_calibrated.size(), in_host_ref.size());
    std::memcpy(
      in_host_ref.data(), sample.input_data_calibrated.data(),
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

    for (size_t v = 0; v < data_utils::height; ++v) {
      for (size_t u = 0; u < data_utils::width; ++u) {
        const auto & res_r =
          in_host.at(0 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & res_g =
          in_host.at(1 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & res_b =
          in_host.at(2 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & res_depth =
          in_host.at(3 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & res_intensity =
          in_host.at(4 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & ref_r =
          in_host_ref.at(0 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & ref_g =
          in_host_ref.at(1 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & ref_b =
          in_host_ref.at(2 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & ref_depth =
          in_host_ref.at(3 * data_utils::width * data_utils::height + v * data_utils::width + u);
        const auto & ref_intensity =
          in_host_ref.at(4 * data_utils::width * data_utils::height + v * data_utils::width + u);

        auto r_diff = std::fabs(res_r - ref_r);
        auto g_diff = std::fabs(res_g - ref_g);
        auto b_diff = std::fabs(res_b - ref_b);

        // 3X3 kernel depth & intensity diff
        float kernel_avg_ref_depth = 0.0;
        float kernel_avg_res_depth = 0.0;
        float kernel_avg_ref_intensity = 0.0;
        float kernel_avg_res_intensity = 0.0;
        for (int i = -1; i <= 1; ++i) {
          for (int j = -1; j <= 1; ++j) {
            const int neighbor_x = u + i;
            const int neighbor_y = v + j;

            if (
              neighbor_x >= 0 && neighbor_x < static_cast<int>(data_utils::width) &&
              neighbor_y >= 0 && neighbor_y < static_cast<int>(data_utils::height)) {
              const int neighbor_idx =
                neighbor_y * static_cast<int>(data_utils::width) + neighbor_x;

              const auto & res_neighbor_depth =
                in_host.at(3 * data_utils::width * data_utils::height + neighbor_idx);
              const auto & res_neighbor_intensity =
                in_host.at(4 * data_utils::width * data_utils::height + neighbor_idx);
              const auto & ref_neighbor_depth =
                in_host_ref.at(3 * data_utils::width * data_utils::height + neighbor_idx);
              const auto & ref_neighbor_intensity =
                in_host_ref.at(4 * data_utils::width * data_utils::height + neighbor_idx);

              kernel_avg_ref_depth += ref_neighbor_depth;
              kernel_avg_res_depth += res_neighbor_depth;
              kernel_avg_ref_intensity += ref_neighbor_intensity;
              kernel_avg_res_intensity += res_neighbor_intensity;
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
          const auto idx = v * data_utils::width + u;
          res_data_rgb.at(idx * 3) = static_cast<uint8_t>(res_r * 255.0f);
          res_data_rgb.at(idx * 3 + 1) = static_cast<uint8_t>(res_g * 255.0f);
          res_data_rgb.at(idx * 3 + 2) = static_cast<uint8_t>(res_b * 255.0f);
          res_data_depth.at(idx) = static_cast<uint8_t>(res_depth * 255.0f);
          res_data_intensity.at(idx) = static_cast<uint8_t>(res_intensity * 255.0f);
          ref_data_rgb.at(idx * 3) = static_cast<uint8_t>(ref_r * 255.0f);
          ref_data_rgb.at(idx * 3 + 1) = static_cast<uint8_t>(ref_g * 255.0f);
          ref_data_rgb.at(idx * 3 + 2) = static_cast<uint8_t>(ref_b * 255.0f);
          ref_data_depth.at(idx) = static_cast<uint8_t>(ref_depth * 255.0f);
          ref_data_intensity.at(idx) = static_cast<uint8_t>(ref_intensity * 255.0f);
        }
      }
    }
    if (save_test_images) {
      data_utils::save_img(
        ref_data_rgb, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_rgb_ref.png", CV_8UC3, "rgb8");
      data_utils::save_img(
        ref_data_depth, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_depth_ref.png", CV_8UC1, "mono8");
      data_utils::save_img(
        ref_data_intensity, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_intensity_ref.png", CV_8UC1, "mono8");
      data_utils::save_img(
        res_data_rgb, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_rgb_res.png", CV_8UC3, "rgb8");
      data_utils::save_img(
        res_data_depth, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_depth_res.png", CV_8UC1, "mono8");
      data_utils::save_img(
        res_data_intensity, data_utils::width, data_utils::height, data_dir,
        sample.sample_name + "_intensity_res.png", CV_8UC1, "mono8");
    }

    GTEST_LOG_(INFO) << "Sample: " << sample.sample_name << ", Array errors: "
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

}  // namespace autoware::calibration_status_classifier

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
