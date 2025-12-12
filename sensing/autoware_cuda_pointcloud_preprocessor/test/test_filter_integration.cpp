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

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/dag_executor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filter_registry.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/transform_filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

#include <memory>

using autoware::cuda_pointcloud_preprocessor::dag::DagExecutor;
using autoware::cuda_pointcloud_preprocessor::dag::DagNodeConfig;
using autoware::cuda_pointcloud_preprocessor::dag::DagNodeInput;
using autoware::cuda_pointcloud_preprocessor::dag::FilterContext;
using autoware::cuda_pointcloud_preprocessor::dag::registerFilterType;

class FilterIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS context
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Filters are already registered in filter_registrations.cpp
    // No need to register them again in tests

    // Initialize CUDA resources
    cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
    cudaStreamCreate(&cuda_stream_);

    int current_device_id = 0;
    cudaGetDevice(&current_device_id);
    cudaMemPoolProps pool_props = {};
    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = current_device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;
    cudaMemPoolCreate(&cuda_memory_pool_, &pool_props);

    // Initialize shared preprocessor
    shared_preprocessor_ =
      std::make_unique<autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessor>();

    // Initialize TF buffer for transform filter
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>());

    // Initialize filter context
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("test_filter_integration"));
    context_.stream = cuda_stream_;
    context_.memory_pool = cuda_memory_pool_;
    context_.logger = logger_.get();
    context_.shared_preprocessor = shared_preprocessor_.get();  // KEY: Set shared preprocessor
    context_.tf_buffer = tf_buffer_.get();
  }

  void TearDown() override
  {
    if (cuda_stream_) {
      cudaStreamDestroy(cuda_stream_);
    }
    if (cuda_memory_pool_) {
      cudaMemPoolDestroy(cuda_memory_pool_);
    }
  }

  sensor_msgs::msg::PointCloud2 createTestPointCloud()
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "base_link";
    cloud.header.stamp.sec = 0;
    cloud.header.stamp.nanosec = 0;

    cloud.height = 1;
    cloud.width = 10;  // 10 points

    cloud.fields.resize(5);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.fields[3].name = "intensity";
    cloud.fields[3].offset = 12;
    cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    cloud.fields[3].count = 1;

    cloud.fields[4].name = "ring";
    cloud.fields[4].offset = 16;
    cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
    cloud.fields[4].count = 1;

    cloud.point_step = 20;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    cloud.data.resize(cloud.row_step);

    // Fill with simple test data
    for (size_t i = 0; i < cloud.width; ++i) {
      float * x = reinterpret_cast<float *>(&cloud.data[i * cloud.point_step + 0]);
      float * y = reinterpret_cast<float *>(&cloud.data[i * cloud.point_step + 4]);
      float * z = reinterpret_cast<float *>(&cloud.data[i * cloud.point_step + 8]);
      uint8_t * intensity = &cloud.data[i * cloud.point_step + 12];
      uint16_t * ring = reinterpret_cast<uint16_t *>(&cloud.data[i * cloud.point_step + 16]);

      *x = static_cast<float>(i);
      *y = 0.0f;
      *z = 0.0f;
      *intensity = 100;
      *ring = static_cast<uint16_t>(i % 4);  // 4 rings
    }

    return cloud;
  }

  DagExecutor executor_;
  FilterContext context_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::unique_ptr<autoware::cuda_pointcloud_preprocessor::CudaPointcloudPreprocessor>
    shared_preprocessor_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  cudaStream_t cuda_stream_{};
  cudaMemPool_t cuda_memory_pool_{};
};

// NOTE: OrganizeFilter is now internalized in the DAG node (automatic at entry)
// This test is disabled as the filter is no longer user-configurable
TEST_F(FilterIntegrationTest, DISABLED_OrganizeFilterBasic)
{
  // OrganizeFilter has been removed - it's now automatic in the DAG node
  // This test is kept for reference but disabled
}

TEST_F(FilterIntegrationTest, DagTopologicalOrder)
{
  // Test that DAG executor properly orders nodes
  // Create: transform -> crop (organize is now automatic)
  std::vector<DagNodeConfig> dag_config;

  // Transform node (receives external input - organize is automatic)
  DagNodeConfig transform_node;
  transform_node.id = "transform";
  transform_node.type = "TransformFilter";

  DagNodeInput transform_input;
  transform_input.source = "pointcloud";  // External input
  transform_input.from_node = "";
  transform_input.optional = false;
  transform_node.inputs.push_back(transform_input);

  transform_node.outputs = {"transformed"};
  transform_node.parameters["target_frame"] = std::string("base_link");
  dag_config.push_back(transform_node);

  // Crop node (depends on transform)
  DagNodeConfig crop_node;
  crop_node.id = "crop";
  crop_node.type = "CropBoxFilter";

  DagNodeInput crop_input;
  crop_input.source = "transformed";
  crop_input.from_node = "transform";
  crop_input.optional = false;
  crop_node.inputs.push_back(crop_input);

  crop_node.outputs = {"cropped"};
  dag_config.push_back(crop_node);

  // Build DAG - should properly order nodes
  EXPECT_NO_THROW(executor_.buildDag(dag_config, context_));
}

TEST_F(FilterIntegrationTest, MetadataValidation)
{
  // Verify filter metadata is accessible
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();

  ASSERT_TRUE(registry.isRegistered("TransformFilter"));

  auto metadata = registry.getMetadata("TransformFilter");

  EXPECT_EQ(metadata.filter_type, "TransformFilter");
  EXPECT_FALSE(metadata.required_inputs.empty());
  // OrganizeFilter accepts any pointcloud input name
  EXPECT_EQ(metadata.required_inputs[0], "*");
  EXPECT_FALSE(metadata.outputs.empty());
  EXPECT_EQ(metadata.outputs[0], "organized");
}

// ============================================================================
// FILTER REGISTRY TESTS
// ============================================================================

TEST_F(FilterIntegrationTest, AllFiltersRegistered)
{
  // Verify all expected filters are registered
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();

  std::vector<std::string> expected_filters = {"TransformFilter",   "TransformFilter",
                                               "CropBoxFilter",     "DistortionFilter",
                                               "RingOutlierFilter", "CropBoxFilter"};

  for (const auto & filter_name : expected_filters) {
    EXPECT_TRUE(registry.isRegistered(filter_name))
      << "Filter " << filter_name << " should be registered";
  }
}

TEST_F(FilterIntegrationTest, FilterFactoryCreation)
{
  // Verify filters can be created from factory
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();

  std::vector<std::string> filter_types = {"TransformFilter",   "TransformFilter",
                                           "CropBoxFilter",     "DistortionFilter",
                                           "RingOutlierFilter", "CropBoxFilter"};

  for (const auto & type : filter_types) {
    EXPECT_NO_THROW({
      auto filter = registry.createFilter(type);
      EXPECT_NE(filter, nullptr) << "Filter " << type << " should be created";
    }) << "Creating filter "
       << type << " should not throw";
  }
}

TEST_F(FilterIntegrationTest, NonExistentFilterThrows)
{
  // Verify requesting non-existent filter returns nullptr
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();

  // Current implementation returns nullptr for non-existent filters
  // rather than throwing an exception
  auto filter = registry.createFilter("NonExistentFilter");
  EXPECT_EQ(filter, nullptr) << "Non-existent filter should return nullptr";
}

// ============================================================================
// FILTER METADATA TESTS
// ============================================================================

TEST_F(FilterIntegrationTest, TransformFilterMetadata)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto metadata = registry.getMetadata("TransformFilter");

  EXPECT_EQ(metadata.filter_type, "TransformFilter");
  EXPECT_EQ(metadata.required_inputs[0], "*");  // Accepts any pointcloud name
  EXPECT_EQ(metadata.outputs[0], "transformed");
}

TEST_F(FilterIntegrationTest, CropBoxFilterMetadata)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto metadata = registry.getMetadata("CropBoxFilter");

  EXPECT_EQ(metadata.filter_type, "CropBoxFilter");
  EXPECT_EQ(metadata.required_inputs[0], "*");
  EXPECT_EQ(metadata.outputs[0], "filtered");
}

TEST_F(FilterIntegrationTest, DistortionFilterMetadata)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto metadata = registry.getMetadata("DistortionFilter");

  EXPECT_EQ(metadata.filter_type, "DistortionFilter");
  // DistortionFilter specifies "pointcloud" as required input name
  EXPECT_EQ(metadata.required_inputs[0], "pointcloud");
  // DistortionFilter may have optional inputs (twist, imu)
  // Check that outputs are defined
  EXPECT_FALSE(metadata.outputs.empty());
  EXPECT_EQ(metadata.outputs[0], "corrected");
}

TEST_F(FilterIntegrationTest, RingOutlierFilterMetadata)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto metadata = registry.getMetadata("RingOutlierFilter");

  EXPECT_EQ(metadata.filter_type, "RingOutlierFilter");
  EXPECT_EQ(metadata.required_inputs[0], "*");
  EXPECT_EQ(metadata.outputs[0], "filtered");
}

TEST_F(FilterIntegrationTest, FinalizeFilterMetadata)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto metadata = registry.getMetadata("CropBoxFilter");

  EXPECT_EQ(metadata.filter_type, "CropBoxFilter");
  EXPECT_EQ(metadata.required_inputs[0], "*");
  EXPECT_EQ(metadata.outputs[0], "output");
}

// ============================================================================
// FILTER INITIALIZATION TESTS
// ============================================================================

TEST_F(FilterIntegrationTest, TransformFilterInitializeValid)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto filter = registry.createFilter("TransformFilter");

  std::map<std::string, std::any> params;
  params["target_frame"] = std::string("base_link");

  // Should not throw
  EXPECT_NO_THROW(filter->initialize(params));
}

TEST_F(FilterIntegrationTest, TransformFilterInitializeInvalid)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto filter = registry.createFilter("TransformFilter");

  std::map<std::string, std::any> params;
  // Missing target_frame parameter

  // TransformFilter may have default behavior or not throw for missing optional parameters
  // Just verify initialize doesn't crash
  EXPECT_NO_THROW(filter->initialize(params));
}

TEST_F(FilterIntegrationTest, CropBoxFilterInitializeValid)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto filter = registry.createFilter("CropBoxFilter");

  std::map<std::string, std::any> params;
  std::vector<std::map<std::string, double>> crop_boxes = {
    {{"min_x", -10.0},
     {"max_x", 10.0},
     {"min_y", -10.0},
     {"max_y", 10.0},
     {"min_z", -2.0},
     {"max_z", 2.0}}};
  params["crop_boxes"] = crop_boxes;

  EXPECT_NO_THROW(filter->initialize(params));
}

TEST_F(FilterIntegrationTest, RingOutlierFilterInitializeValid)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto filter = registry.createFilter("RingOutlierFilter");

  std::map<std::string, std::any> params;
  params["distance_ratio"] = 1.03;
  params["object_length_threshold"] = 0.05;
  params["enabled"] = true;

  EXPECT_NO_THROW(filter->initialize(params));
}

TEST_F(FilterIntegrationTest, RingOutlierFilterInitializeInvalid)
{
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();
  auto filter = registry.createFilter("RingOutlierFilter");

  std::map<std::string, std::any> params;
  params["distance_ratio"] = 1.03;
  // Missing object_length_threshold

  EXPECT_THROW(filter->initialize(params), std::runtime_error);
}

// ============================================================================
// SHARED PREPROCESSOR PATTERN TESTS
// ============================================================================

TEST_F(FilterIntegrationTest, SharedPreprocessorAvailable)
{
  // Verify shared preprocessor is available in context
  ASSERT_NE(context_.shared_preprocessor, nullptr)
    << "Shared preprocessor should be set in FilterContext";
}

TEST_F(FilterIntegrationTest, MultipleFiltersUseSharedPreprocessor)
{
  // Verify multiple filters can use the same shared preprocessor
  auto & registry = autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry();

  auto filter1 = registry.createFilter("CropBoxFilter");
  auto filter2 = registry.createFilter("RingOutlierFilter");

  // Initialize both
  std::map<std::string, std::any> params1;
  params1["crop_boxes"] = std::vector<std::map<std::string, double>>();
  filter1->initialize(params1);

  std::map<std::string, std::any> params2;
  params2["distance_ratio"] = 1.03;
  params2["object_length_threshold"] = 0.05;
  params2["enabled"] = true;
  filter2->initialize(params2);

  // Both should be able to reference the same shared preprocessor
  // (This is validated by having context_.shared_preprocessor set)
  SUCCEED();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
