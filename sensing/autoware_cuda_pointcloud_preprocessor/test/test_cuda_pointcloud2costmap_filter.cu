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

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud2costmap_filter/cuda_pointcloud2costmap_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"
#include "autoware/cuda_utils/cuda_gtest_utils.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/fill.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{

class CudaPointcloud2CostmapFilterTest : public autoware::cuda_utils::CudaTest
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr createTestPointCloud(
    size_t num_points, float center_x = 0.0f, float center_y = 0.0f, float center_z = 0.0f,
    float spread = 10.0f)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(num_points);

    std::uniform_real_distribution<float> dist_x(center_x - spread, center_x + spread);
    std::uniform_real_distribution<float> dist_y(center_y - spread, center_y + spread);
    std::uniform_real_distribution<float> dist_z(center_z - 1.0f, center_z + 1.0f);

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
      point.distance = 1.0f;
      point.time_stamp = 0U;
      input_points.push_back(point);
    }

    return input_points;
  }

  // Apply CUDA costmap kernel directly
  grid_map::Matrix applyCUDACostmapKernel(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    const CostmapParameters & costmap_params)
  {
    const size_t num_points = input_cloud->points.size();

    // Calculate grid size
    int grid_size_x =
      static_cast<int>(std::ceil(costmap_params.grid_length_x / costmap_params.grid_resolution));
    int grid_size_y =
      static_cast<int>(std::ceil(costmap_params.grid_length_y / costmap_params.grid_resolution));

    if (num_points == 0) {
      // Initialize costmap with min_value
      return grid_map::Matrix::Constant(grid_size_x, grid_size_y, costmap_params.grid_min_value);
    }

    // Convert to InputPointType
    std::vector<InputPointType> host_input_points = convertToInputPointType(input_cloud);

    // Prepare costmap parameters for device
    CostmapGridParams host_params;
    host_params.grid_length_x = static_cast<float>(costmap_params.grid_length_x);
    host_params.grid_length_y = static_cast<float>(costmap_params.grid_length_y);
    host_params.grid_resolution = static_cast<float>(costmap_params.grid_resolution);
    host_params.grid_position_x = static_cast<float>(costmap_params.grid_position_x);
    host_params.grid_position_y = static_cast<float>(costmap_params.grid_position_y);
    host_params.maximum_height_thres = static_cast<float>(costmap_params.maximum_height_thres);
    host_params.minimum_height_thres = static_cast<float>(costmap_params.minimum_height_thres);
    host_params.grid_min_value = static_cast<float>(costmap_params.grid_min_value);
    host_params.grid_max_value = static_cast<float>(costmap_params.grid_max_value);
    host_params.grid_size_x = grid_size_x;
    host_params.grid_size_y = grid_size_y;

    // Allocate device memory
    thrust::device_vector<InputPointType> device_input_points(host_input_points);
    thrust::device_vector<int32_t> device_grid_indices(num_points, -1);
    thrust::device_vector<float> device_costmap_data(grid_size_x * grid_size_y);
    thrust::device_vector<CostmapGridParams> device_params(1, host_params);

    // Initialize grid indices to -1 (invalid)
    thrust::fill(device_grid_indices.begin(), device_grid_indices.end(), -1);

    // Assign points to grid cells
    const int threads_per_block = 512;
    const int blocks_per_grid = (num_points + threads_per_block - 1) / threads_per_block;
    assignPointsToGridCellsLaunch(
      thrust::raw_pointer_cast(device_input_points.data()),
      thrust::raw_pointer_cast(device_grid_indices.data()), static_cast<int>(num_points),
      thrust::raw_pointer_cast(device_params.data()), threads_per_block, blocks_per_grid, stream_);

    // Calculate costmap from grid cells
    calculateCostmapFromGridCellsLaunch(
      thrust::raw_pointer_cast(device_input_points.data()),
      thrust::raw_pointer_cast(device_grid_indices.data()),
      thrust::raw_pointer_cast(device_costmap_data.data()), static_cast<int>(num_points),
      thrust::raw_pointer_cast(device_params.data()), threads_per_block, blocks_per_grid, stream_);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    // Copy costmap data back to host
    std::vector<float> host_costmap_data(grid_size_x * grid_size_y);
    thrust::copy(device_costmap_data.begin(), device_costmap_data.end(), host_costmap_data.begin());

    // Convert to grid_map::Matrix
    grid_map::Matrix costmap_matrix(grid_size_x, grid_size_y);
    for (int x = 0; x < grid_size_x; ++x) {
      for (int y = 0; y < grid_size_y; ++y) {
        // Linear index: x * grid_size_y + y
        costmap_matrix(x, y) = host_costmap_data[x * grid_size_y + y];
      }
    }

    return costmap_matrix;
  }

  // Helper function to create reference costmap using CPU implementation logic
  grid_map::Matrix createReferenceCostmap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double grid_length_x,
    double grid_length_y, double grid_resolution, double grid_position_x, double grid_position_y,
    double maximum_height_thres, double minimum_height_thres, double grid_min_value,
    double grid_max_value)
  {
    // Calculate grid size
    int grid_size_x = static_cast<int>(std::ceil(grid_length_x / grid_resolution));
    int grid_size_y = static_cast<int>(std::ceil(grid_length_y / grid_resolution));

    grid_map::Matrix costmap = grid_map::Matrix::Constant(grid_size_x, grid_size_y, grid_min_value);

    // Assign points to grid cells (following CPU implementation logic)
    for (const auto & point : input_cloud->points) {
      // Calculate grid index (following fetchGridIndexFromPoint logic)
      const double origin_x_offset = grid_length_x / 2.0 - grid_position_x;
      const double origin_y_offset = grid_length_y / 2.0 - grid_position_y;
      double mapped_x = (grid_length_x - origin_x_offset - point.x) / grid_resolution;
      double mapped_y = (grid_length_y - origin_y_offset - point.y) / grid_resolution;

      int mapped_x_ind = static_cast<int>(std::ceil(mapped_x));
      int mapped_y_ind = static_cast<int>(std::ceil(mapped_y));

      // Check if index is valid
      if (
        mapped_x_ind >= 0 && mapped_x_ind < grid_size_x && mapped_y_ind >= 0 &&
        mapped_y_ind < grid_size_y) {
        // Check height threshold
        if (point.z <= maximum_height_thres && point.z >= minimum_height_thres) {
          costmap(mapped_x_ind, mapped_y_ind) = grid_max_value;
        }
      }
    }

    return costmap;
  }

  // Helper function to compare two costmaps
  void compareCostmaps(
    const grid_map::Matrix & costmap1, const grid_map::Matrix & costmap2, const std::string & name)
  {
    EXPECT_EQ(costmap1.rows(), costmap2.rows()) << name << ": Costmap rows should match";
    EXPECT_EQ(costmap1.cols(), costmap2.cols()) << name << ": Costmap cols should match";

    if (costmap1.rows() == costmap2.rows() && costmap1.cols() == costmap2.cols()) {
      int matching_cells = 0;
      int total_cells = costmap1.rows() * costmap1.cols();

      for (int i = 0; i < costmap1.rows(); ++i) {
        for (int j = 0; j < costmap1.cols(); ++j) {
          if (std::abs(costmap1(i, j) - costmap2(i, j)) < 1e-5) {
            matching_cells++;
          }
        }
      }

      // Allow some tolerance for floating point differences
      double match_ratio = static_cast<double>(matching_cells) / total_cells;
      EXPECT_GT(match_ratio, 0.95) << name << ": At least 95% of cells should match";
    }
  }

  std::mt19937 rng_;
  cudaStream_t stream_{};
};

TEST_F(CudaPointcloud2CostmapFilterTest, BasicCostmapGeneration)
{
  // Create test point cloud
  const size_t num_points = 1000;
  auto pcl_input = createTestPointCloud(num_points, 0.0f, 0.0f, 0.0f, 10.0f);

  // Define costmap parameters
  CostmapParameters costmap_params;
  costmap_params.grid_length_x = 20.0;
  costmap_params.grid_length_y = 20.0;
  costmap_params.grid_resolution = 0.5;
  costmap_params.grid_position_x = 0.0;
  costmap_params.grid_position_y = 0.0;
  costmap_params.maximum_height_thres = 2.0;
  costmap_params.minimum_height_thres = -2.0;
  costmap_params.grid_min_value = 0.0;
  costmap_params.grid_max_value = 100.0;

  // Apply CUDA kernel
  grid_map::Matrix cuda_costmap = applyCUDACostmapKernel(pcl_input, costmap_params);

  // Create reference costmap
  grid_map::Matrix reference_costmap = createReferenceCostmap(
    pcl_input, costmap_params.grid_length_x, costmap_params.grid_length_y,
    costmap_params.grid_resolution, costmap_params.grid_position_x, costmap_params.grid_position_y,
    costmap_params.maximum_height_thres, costmap_params.minimum_height_thres,
    costmap_params.grid_min_value, costmap_params.grid_max_value);

  // Compare costmaps
  compareCostmaps(cuda_costmap, reference_costmap, "BasicCostmapGeneration");
}

TEST_F(CudaPointcloud2CostmapFilterTest, EmptyInput)
{
  // Test with empty point cloud
  auto pcl_input = createTestPointCloud(0);

  CostmapParameters costmap_params;
  costmap_params.grid_length_x = 20.0;
  costmap_params.grid_length_y = 20.0;
  costmap_params.grid_resolution = 0.5;
  costmap_params.grid_position_x = 0.0;
  costmap_params.grid_position_y = 0.0;
  costmap_params.maximum_height_thres = 2.0;
  costmap_params.minimum_height_thres = -2.0;
  costmap_params.grid_min_value = 0.0;
  costmap_params.grid_max_value = 100.0;

  grid_map::Matrix cuda_costmap = applyCUDACostmapKernel(pcl_input, costmap_params);

  // Should be initialized with min_value
  int grid_size_x =
    static_cast<int>(std::ceil(costmap_params.grid_length_x / costmap_params.grid_resolution));
  int grid_size_y =
    static_cast<int>(std::ceil(costmap_params.grid_length_y / costmap_params.grid_resolution));

  EXPECT_EQ(cuda_costmap.rows(), grid_size_x);
  EXPECT_EQ(cuda_costmap.cols(), grid_size_y);

  // All cells should be min_value
  for (int i = 0; i < cuda_costmap.rows(); ++i) {
    for (int j = 0; j < cuda_costmap.cols(); ++j) {
      EXPECT_NEAR(cuda_costmap(i, j), costmap_params.grid_min_value, 1e-5);
    }
  }
}

TEST_F(CudaPointcloud2CostmapFilterTest, HeightThresholdFiltering)
{
  // Create point cloud with points at different heights
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  // Points within height threshold
  for (int i = 0; i < 100; ++i) {
    pcl::PointXYZ point;
    point.x = static_cast<float>(i % 10) - 5.0f;
    point.y = static_cast<float>(i / 10) - 5.0f;
    point.z = 0.5f;  // Within threshold
    pcl_cloud.points.push_back(point);
  }
  // Points outside height threshold
  for (int i = 0; i < 50; ++i) {
    pcl::PointXYZ point;
    point.x = static_cast<float>(i % 10) - 5.0f;
    point.y = static_cast<float>(i / 10) - 5.0f;
    point.z = 5.0f;  // Outside threshold (too high)
    pcl_cloud.points.push_back(point);
  }
  for (int i = 0; i < 50; ++i) {
    pcl::PointXYZ point;
    point.x = static_cast<float>(i % 10) - 5.0f;
    point.y = static_cast<float>(i / 10) - 5.0f;
    point.z = -5.0f;  // Outside threshold (too low)
    pcl_cloud.points.push_back(point);
  }

  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>(pcl_cloud));

  CostmapParameters costmap_params;
  costmap_params.grid_length_x = 20.0;
  costmap_params.grid_length_y = 20.0;
  costmap_params.grid_resolution = 1.0;
  costmap_params.grid_position_x = 0.0;
  costmap_params.grid_position_y = 0.0;
  costmap_params.maximum_height_thres = 2.0;
  costmap_params.minimum_height_thres = -2.0;
  costmap_params.grid_min_value = 0.0;
  costmap_params.grid_max_value = 100.0;

  grid_map::Matrix cuda_costmap = applyCUDACostmapKernel(pcl_input, costmap_params);

  // Create reference
  grid_map::Matrix reference_costmap = createReferenceCostmap(
    pcl_input, costmap_params.grid_length_x, costmap_params.grid_length_y,
    costmap_params.grid_resolution, costmap_params.grid_position_x, costmap_params.grid_position_y,
    costmap_params.maximum_height_thres, costmap_params.minimum_height_thres,
    costmap_params.grid_min_value, costmap_params.grid_max_value);

  compareCostmaps(cuda_costmap, reference_costmap, "HeightThresholdFiltering");
}

TEST_F(CudaPointcloud2CostmapFilterTest, DifferentGridPositions)
{
  // Test with different grid positions
  const size_t num_points = 500;
  auto pcl_input = createTestPointCloud(num_points, 5.0f, 5.0f, 0.0f, 5.0f);

  CostmapParameters costmap_params;
  costmap_params.grid_length_x = 20.0;
  costmap_params.grid_length_y = 20.0;
  costmap_params.grid_resolution = 0.5;
  costmap_params.grid_position_x = 5.0;  // Offset position
  costmap_params.grid_position_y = 5.0;
  costmap_params.maximum_height_thres = 2.0;
  costmap_params.minimum_height_thres = -2.0;
  costmap_params.grid_min_value = 0.0;
  costmap_params.grid_max_value = 100.0;

  grid_map::Matrix cuda_costmap = applyCUDACostmapKernel(pcl_input, costmap_params);

  grid_map::Matrix reference_costmap = createReferenceCostmap(
    pcl_input, costmap_params.grid_length_x, costmap_params.grid_length_y,
    costmap_params.grid_resolution, costmap_params.grid_position_x, costmap_params.grid_position_y,
    costmap_params.maximum_height_thres, costmap_params.minimum_height_thres,
    costmap_params.grid_min_value, costmap_params.grid_max_value);

  compareCostmaps(cuda_costmap, reference_costmap, "DifferentGridPositions");
}

TEST_F(CudaPointcloud2CostmapFilterTest, DifferentResolutions)
{
  // Test with different grid resolutions
  const size_t num_points = 1000;
  auto pcl_input = createTestPointCloud(num_points, 0.0f, 0.0f, 0.0f, 10.0f);

  CostmapParameters costmap_params;
  costmap_params.grid_length_x = 20.0;
  costmap_params.grid_length_y = 20.0;
  costmap_params.grid_resolution = 0.2;  // Higher resolution
  costmap_params.grid_position_x = 0.0;
  costmap_params.grid_position_y = 0.0;
  costmap_params.maximum_height_thres = 2.0;
  costmap_params.minimum_height_thres = -2.0;
  costmap_params.grid_min_value = 0.0;
  costmap_params.grid_max_value = 100.0;

  grid_map::Matrix cuda_costmap = applyCUDACostmapKernel(pcl_input, costmap_params);

  grid_map::Matrix reference_costmap = createReferenceCostmap(
    pcl_input, costmap_params.grid_length_x, costmap_params.grid_length_y,
    costmap_params.grid_resolution, costmap_params.grid_position_x, costmap_params.grid_position_y,
    costmap_params.maximum_height_thres, costmap_params.minimum_height_thres,
    costmap_params.grid_min_value, costmap_params.grid_max_value);

  compareCostmaps(cuda_costmap, reference_costmap, "DifferentResolutions");
}

}  // namespace autoware::cuda_pointcloud_preprocessor
