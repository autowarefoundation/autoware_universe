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

#include "autoware/cuda_pointcloud_preprocessor/common_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"
#include "autoware/cuda_utils/cuda_gtest_utils.hpp"

#include <gtest/gtest.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/scan.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{

class CudaCropBoxFilterTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override
  {
    autoware::cuda_utils::CudaTest::SetUp();
    rng_.seed(42);
    CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  }

  void TearDown() override
  {
    if (stream_) {
      cudaStreamDestroy(stream_);
    }
    autoware::cuda_utils::CudaTest::TearDown();
  }

  // Helper function to create a test point cloud with random points
  pcl::PointCloud<pcl::PointXYZ>::Ptr createTestPointCloud(
    size_t num_points, float min_x = -10.0f, float max_x = 10.0f, float min_y = -10.0f,
    float max_y = 10.0f, float min_z = -5.0f, float max_z = 5.0f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(num_points);

    std::uniform_real_distribution<float> dist_x(min_x, max_x);
    std::uniform_real_distribution<float> dist_y(min_y, max_y);
    std::uniform_real_distribution<float> dist_z(min_z, max_z);

    for (size_t i = 0; i < num_points; ++i) {
      pcl::PointXYZ point;
      point.x = dist_x(rng_);
      point.y = dist_y(rng_);
      point.z = dist_z(rng_);
      cloud->points.push_back(point);
    }

    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  // Convert PCL PointXYZ to InputPointType (on host)
  std::vector<InputPointType> convertToInputPointType(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_cloud)
  {
    std::vector<InputPointType> input_points;
    input_points.reserve(pcl_cloud->points.size());

    for (const auto & pcl_point : pcl_cloud->points) {
      InputPointType point;
      point.x = pcl_point.x;
      point.y = pcl_point.y;
      point.z = pcl_point.z;
      point.intensity = 128;
      point.return_type = 0;
      point.channel = 0;
      point.azimuth = 0.0f;
      point.elevation = 0.0f;
      point.distance = 1.0f;  // Non-zero so point is not skipped
      point.time_stamp = 0U;
      input_points.push_back(point);
    }

    return input_points;
  }

  // Convert OutputPointType to PCL PointXYZ (on host)
  pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(
    const std::vector<OutputPointType> & output_points)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(output_points.size());

    for (const auto & point : output_points) {
      pcl::PointXYZ pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = point.y;
      pcl_point.z = point.z;
      cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  // Apply CUDA crop box kernel directly
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyCUDACropBoxKernel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, const CropBoxParameters & crop_params)
  {
    const size_t num_points = input_cloud->points.size();
    if (num_points == 0) {
      return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

    // Convert to InputPointType
    std::vector<InputPointType> host_input_points = convertToInputPointType(input_cloud);

    // Allocate device memory
    thrust::device_vector<InputPointType> device_input_points(host_input_points);
    thrust::device_vector<std::uint32_t> device_crop_mask(num_points, 0);
    thrust::device_vector<std::uint8_t> device_nan_mask(num_points, 0);
    thrust::device_vector<std::uint32_t> device_indices(num_points, 0);
    thrust::device_vector<CropBoxParameters> device_crop_params(1, crop_params);

    // Launch crop box kernel
    const int threads_per_block = 512;
    const int blocks_per_grid = (num_points + threads_per_block - 1) / threads_per_block;
    cropBoxLaunch(
      thrust::raw_pointer_cast(device_input_points.data()),
      thrust::raw_pointer_cast(device_crop_mask.data()),
      thrust::raw_pointer_cast(device_nan_mask.data()), static_cast<int>(num_points),
      thrust::raw_pointer_cast(device_crop_params.data()), 1, threads_per_block, blocks_per_grid,
      stream_);

    // Compute indices using inclusive scan
    thrust::inclusive_scan(
      thrust::cuda::par_nosync.on(stream_), device_crop_mask.begin(), device_crop_mask.end(),
      device_indices.begin());

    // Get number of output points
    int num_output_points = 0;
    if (num_points > 0) {
      std::uint32_t last_index = 0;
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        &last_index, thrust::raw_pointer_cast(device_indices.data() + num_points - 1),
        sizeof(std::uint32_t), cudaMemcpyDeviceToHost, stream_));
      CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
      num_output_points = static_cast<int>(last_index);
    }

    if (num_output_points == 0) {
      return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

    // Extract points
    thrust::device_vector<OutputPointType> device_output_points(num_output_points);
    extractPointsLaunch(
      thrust::raw_pointer_cast(device_input_points.data()),
      thrust::raw_pointer_cast(device_crop_mask.data()),
      thrust::raw_pointer_cast(device_indices.data()), static_cast<int>(num_points),
      thrust::raw_pointer_cast(device_output_points.data()), threads_per_block, blocks_per_grid,
      stream_);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    // Copy back to host
    std::vector<OutputPointType> host_output_points(num_output_points);
    thrust::copy(
      device_output_points.begin(), device_output_points.end(), host_output_points.begin());

    return convertToPCL(host_output_points);
  }

  // Helper function to apply PCL crop box filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyPCLCropBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, float min_x, float max_x, float min_y,
    float max_y, float min_z, float max_z, bool negative = false)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(input_cloud);
    crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0f));
    crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0f));
    crop_box.setNegative(negative);
    crop_box.filter(*output_cloud);
    return output_cloud;
  }

  std::mt19937 rng_;
  cudaStream_t stream_{};
};

TEST_F(CudaCropBoxFilterTest, BasicCropBoxFilter)
{
  // Create test point cloud
  const size_t num_points = 1000;
  auto pcl_input = createTestPointCloud(num_points, -10.0f, 10.0f, -10.0f, 10.0f, -5.0f, 5.0f);

  // Define crop box parameters
  CropBoxParameters crop_params;
  crop_params.min_x = -5.0f;
  crop_params.max_x = 5.0f;
  crop_params.min_y = -5.0f;
  crop_params.max_y = 5.0f;
  crop_params.min_z = -2.0f;
  crop_params.max_z = 2.0f;
  crop_params.negative = 0;  // Positive mode: keep points inside

  // Apply CUDA kernel
  auto cuda_output = applyCUDACropBoxKernel(pcl_input, crop_params);

  // Apply PCL filter for comparison
  auto pcl_output = applyPCLCropBox(
    pcl_input, crop_params.min_x, crop_params.max_x, crop_params.min_y, crop_params.max_y,
    crop_params.min_z, crop_params.max_z, false);

  // Compare results
  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size())
    << "CUDA and PCL should produce the same number of output points";

  // Sort points by x, y, z for comparison (order may differ)
  auto sortPoints = [](const pcl::PointXYZ & a, const pcl::PointXYZ & b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  };

  std::sort(cuda_output->points.begin(), cuda_output->points.end(), sortPoints);
  std::sort(pcl_output->points.begin(), pcl_output->points.end(), sortPoints);

  // Compare point coordinates (with tolerance for floating point)
  const float tolerance = 1e-5f;
  for (size_t i = 0; i < std::min(cuda_output->points.size(), pcl_output->points.size()); ++i) {
    EXPECT_NEAR(cuda_output->points[i].x, pcl_output->points[i].x, tolerance);
    EXPECT_NEAR(cuda_output->points[i].y, pcl_output->points[i].y, tolerance);
    EXPECT_NEAR(cuda_output->points[i].z, pcl_output->points[i].z, tolerance);
  }
}

TEST_F(CudaCropBoxFilterTest, NegativeCropBoxFilter)
{
  // Create test point cloud
  const size_t num_points = 1000;
  auto pcl_input = createTestPointCloud(num_points, -10.0f, 10.0f, -10.0f, 10.0f, -5.0f, 5.0f);

  // Define crop box parameters (negative mode: remove points inside)
  CropBoxParameters crop_params;
  crop_params.min_x = -5.0f;
  crop_params.max_x = 5.0f;
  crop_params.min_y = -5.0f;
  crop_params.max_y = 5.0f;
  crop_params.min_z = -2.0f;
  crop_params.max_z = 2.0f;
  crop_params.negative = 1;  // Negative mode: remove points inside

  // Apply CUDA kernel
  auto cuda_output = applyCUDACropBoxKernel(pcl_input, crop_params);

  // Apply PCL filter for comparison
  auto pcl_output = applyPCLCropBox(
    pcl_input, crop_params.min_x, crop_params.max_x, crop_params.min_y, crop_params.max_y,
    crop_params.min_z, crop_params.max_z, true);

  // Compare results
  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size())
    << "CUDA and PCL should produce the same number of output points in negative mode";
}

TEST_F(CudaCropBoxFilterTest, EmptyInput)
{
  // Test with empty point cloud
  auto pcl_input = createTestPointCloud(0);

  CropBoxParameters crop_params;
  crop_params.min_x = -5.0f;
  crop_params.max_x = 5.0f;
  crop_params.min_y = -5.0f;
  crop_params.max_y = 5.0f;
  crop_params.min_z = -2.0f;
  crop_params.max_z = 2.0f;
  crop_params.negative = 0;

  auto cuda_output = applyCUDACropBoxKernel(pcl_input, crop_params);

  EXPECT_EQ(cuda_output->points.size(), 0);
}

TEST_F(CudaCropBoxFilterTest, AllPointsInside)
{
  // Create point cloud with all points inside crop box
  const size_t num_points = 100;
  auto pcl_input = createTestPointCloud(num_points, -2.0f, 2.0f, -2.0f, 2.0f, -1.0f, 1.0f);

  CropBoxParameters crop_params;
  crop_params.min_x = -5.0f;
  crop_params.max_x = 5.0f;
  crop_params.min_y = -5.0f;
  crop_params.max_y = 5.0f;
  crop_params.min_z = -5.0f;
  crop_params.max_z = 5.0f;
  crop_params.negative = 0;

  auto cuda_output = applyCUDACropBoxKernel(pcl_input, crop_params);
  auto pcl_output = applyPCLCropBox(
    pcl_input, crop_params.min_x, crop_params.max_x, crop_params.min_y, crop_params.max_y,
    crop_params.min_z, crop_params.max_z, false);

  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size());
  EXPECT_EQ(cuda_output->points.size(), num_points);
}

TEST_F(CudaCropBoxFilterTest, AllPointsOutside)
{
  // Create point cloud with all points outside crop box
  const size_t num_points = 100;
  auto pcl_input = createTestPointCloud(num_points, 10.0f, 20.0f, 10.0f, 20.0f, 10.0f, 20.0f);

  CropBoxParameters crop_params;
  crop_params.min_x = -5.0f;
  crop_params.max_x = 5.0f;
  crop_params.min_y = -5.0f;
  crop_params.max_y = 5.0f;
  crop_params.min_z = -5.0f;
  crop_params.max_z = 5.0f;
  crop_params.negative = 0;

  auto cuda_output = applyCUDACropBoxKernel(pcl_input, crop_params);
  auto pcl_output = applyPCLCropBox(
    pcl_input, crop_params.min_x, crop_params.max_x, crop_params.min_y, crop_params.max_y,
    crop_params.min_z, crop_params.max_z, false);

  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size());
  EXPECT_EQ(cuda_output->points.size(), 0);
}

}  // namespace autoware::cuda_pointcloud_preprocessor
