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
#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"
#include "autoware/cuda_utils/cuda_gtest_utils.hpp"

#include <gtest/gtest.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/sequence.h>
#include <thrust/shuffle.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <set>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{
// Copy the kernel from the implementation file for testing
__global__ void randomSampleKernel(
  const InputPointType * __restrict__ input_points, OutputPointType * __restrict__ output_points,
  const std::uint32_t * __restrict__ selected_indices, size_t num_samples)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_samples) {
    return;
  }

  const std::uint32_t input_idx = selected_indices[idx];
  const InputPointType & input_point = input_points[input_idx];
  OutputPointType & output_point = output_points[idx];

  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.intensity = input_point.intensity;
  output_point.return_type = input_point.return_type;
  output_point.channel = input_point.channel;
}
}  // namespace

class CudaRandomDownsampleFilterTest : public autoware::cuda_utils::CudaTest
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

  // Helper function to create a test point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr createTestPointCloud(size_t num_points)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(num_points);

    std::uniform_real_distribution<float> dist(-10.0f, 10.0f);

    for (size_t i = 0; i < num_points; ++i) {
      pcl::PointXYZ point;
      point.x = dist(rng_);
      point.y = dist(rng_);
      point.z = dist(rng_);
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
      point.distance = 1.0f;
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

  // Apply CUDA random downsample kernel directly
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyCUDARandomDownsampleKernel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, size_t sample_num)
  {
    const size_t num_input_points = input_cloud->points.size();
    if (num_input_points == 0) {
      return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

    const size_t num_samples = std::min(sample_num, num_input_points);
    if (num_samples == 0) {
      return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

    // Convert to InputPointType
    std::vector<InputPointType> host_input_points = convertToInputPointType(input_cloud);

    // Allocate device memory
    thrust::device_vector<InputPointType> device_input_points(host_input_points);
    thrust::device_vector<std::uint32_t> device_all_indices(num_input_points);
    thrust::device_vector<std::uint32_t> device_selected_indices(num_samples);

    // Generate sequence of indices [0, 1, 2, ..., num_input_points-1]
    thrust::sequence(device_all_indices.begin(), device_all_indices.end(), 0);

    // Shuffle the indices using random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    uint32_t seed = static_cast<uint32_t>(gen());
    thrust::default_random_engine rng(seed);
    thrust::shuffle(device_all_indices.begin(), device_all_indices.end(), rng);

    // Copy first num_samples indices to selected_indices
    thrust::copy(
      device_all_indices.begin(), device_all_indices.begin() + num_samples,
      device_selected_indices.begin());

    // Allocate output buffer
    thrust::device_vector<OutputPointType> device_output_points(num_samples);

    // Sample points using random indices
    const int threads_per_block = 512;
    const int blocks_per_grid = (num_samples + threads_per_block - 1) / threads_per_block;
    randomSampleKernel<<<blocks_per_grid, threads_per_block, 0, stream_>>>(
      thrust::raw_pointer_cast(device_input_points.data()),
      thrust::raw_pointer_cast(device_output_points.data()),
      thrust::raw_pointer_cast(device_selected_indices.data()), num_samples);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    // Copy back to host
    std::vector<OutputPointType> host_output_points(num_samples);
    thrust::copy(
      device_output_points.begin(), device_output_points.end(), host_output_points.begin());

    return convertToPCL(host_output_points);
  }

  // Helper function to apply PCL random sample filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr applyPCLRandomSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, size_t sample_num)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RandomSample<pcl::PointXYZ> filter;
    filter.setInputCloud(input_cloud);
    filter.setSample(sample_num);
    filter.filter(*output_cloud);
    return output_cloud;
  }

  // Helper function to create a set of point coordinates for comparison
  std::set<std::tuple<float, float, float>> createPointSet(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
  {
    std::set<std::tuple<float, float, float>> point_set;
    for (const auto & point : cloud->points) {
      point_set.insert(std::make_tuple(point.x, point.y, point.z));
    }
    return point_set;
  }

  std::mt19937 rng_;
  cudaStream_t stream_{};
};

TEST_F(CudaRandomDownsampleFilterTest, BasicRandomDownsample)
{
  // Create test point cloud
  const size_t num_input_points = 1000;
  const size_t sample_num = 500;
  auto pcl_input = createTestPointCloud(num_input_points);

  // Apply CUDA kernel
  auto cuda_output = applyCUDARandomDownsampleKernel(pcl_input, sample_num);

  // Apply PCL filter for comparison
  auto pcl_output = applyPCLRandomSample(pcl_input, sample_num);

  // Check that output sizes match (should be sample_num or less)
  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size())
    << "CUDA and PCL should produce the same number of output points";
  EXPECT_LE(cuda_output->points.size(), sample_num);
  EXPECT_LE(cuda_output->points.size(), num_input_points);

  // Verify that all output points are from the input (order may differ)
  auto input_set = createPointSet(pcl_input);
  auto cuda_output_set = createPointSet(cuda_output);
  auto pcl_output_set = createPointSet(pcl_output);

  // All CUDA output points should be in input
  for (const auto & point : cuda_output_set) {
    EXPECT_NE(input_set.find(point), input_set.end())
      << "CUDA output point should be in input cloud";
  }

  // All PCL output points should be in input
  for (const auto & point : pcl_output_set) {
    EXPECT_NE(input_set.find(point), input_set.end())
      << "PCL output point should be in input cloud";
  }
}

TEST_F(CudaRandomDownsampleFilterTest, SampleNumLargerThanInput)
{
  // When sample_num > input size, should return all input points
  const size_t num_input_points = 100;
  const size_t sample_num = 500;  // Larger than input
  auto pcl_input = createTestPointCloud(num_input_points);

  auto cuda_output = applyCUDARandomDownsampleKernel(pcl_input, sample_num);
  auto pcl_output = applyPCLRandomSample(pcl_input, sample_num);

  // Both should return all input points (or sample_num, whichever is smaller)
  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size());
  EXPECT_EQ(cuda_output->points.size(), num_input_points);
}

TEST_F(CudaRandomDownsampleFilterTest, EmptyInput)
{
  // Test with empty point cloud
  auto pcl_input = createTestPointCloud(0);

  auto cuda_output = applyCUDARandomDownsampleKernel(pcl_input, 100);

  EXPECT_EQ(cuda_output->points.size(), 0);
}

TEST_F(CudaRandomDownsampleFilterTest, SampleNumZero)
{
  // When sample_num is 0, should return empty cloud
  const size_t num_input_points = 100;
  const size_t sample_num = 0;
  auto pcl_input = createTestPointCloud(num_input_points);

  auto cuda_output = applyCUDARandomDownsampleKernel(pcl_input, sample_num);

  EXPECT_EQ(cuda_output->points.size(), 0);
}

TEST_F(CudaRandomDownsampleFilterTest, SampleNumEqualsInput)
{
  // When sample_num == input size, should return all points (but order may differ)
  const size_t num_input_points = 100;
  const size_t sample_num = num_input_points;
  auto pcl_input = createTestPointCloud(num_input_points);

  auto cuda_output = applyCUDARandomDownsampleKernel(pcl_input, sample_num);
  auto pcl_output = applyPCLRandomSample(pcl_input, sample_num);

  // Should have same number of points
  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size());
  EXPECT_EQ(cuda_output->points.size(), num_input_points);

  // Verify all points are present (using sets to ignore order)
  auto input_set = createPointSet(pcl_input);
  auto cuda_output_set = createPointSet(cuda_output);
  auto pcl_output_set = createPointSet(pcl_output);

  EXPECT_EQ(input_set.size(), cuda_output_set.size());
  EXPECT_EQ(input_set.size(), pcl_output_set.size());
}

TEST_F(CudaRandomDownsampleFilterTest, LargePointCloud)
{
  // Test with larger point cloud
  const size_t num_input_points = 10000;
  const size_t sample_num = 1000;
  auto pcl_input = createTestPointCloud(num_input_points);

  auto cuda_output = applyCUDARandomDownsampleKernel(pcl_input, sample_num);
  auto pcl_output = applyPCLRandomSample(pcl_input, sample_num);

  EXPECT_EQ(cuda_output->points.size(), pcl_output->points.size());
  EXPECT_LE(cuda_output->points.size(), sample_num);

  // Verify all output points are from input
  auto input_set = createPointSet(pcl_input);
  auto cuda_output_set = createPointSet(cuda_output);

  for (const auto & point : cuda_output_set) {
    EXPECT_NE(input_set.find(point), input_set.end());
  }
}

}  // namespace autoware::cuda_pointcloud_preprocessor
