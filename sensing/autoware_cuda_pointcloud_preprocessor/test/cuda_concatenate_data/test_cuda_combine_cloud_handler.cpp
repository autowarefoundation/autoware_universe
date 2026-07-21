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

// GPU runtime test for the CUDA point cloud concatenation core. It exercises the refactored
// CombineCloudHandler<cuda_blackboard::CudaPointCloud2> end-to-end on the GPU: build input clouds
// in device memory, concatenate, copy the result back, and check it. This is the first functional
// test of this package (previously only lint ran), and verifies the lockstep refactor of the CUDA
// handler to the injected-transform / message-type-templated core.

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_combine_cloud_handler.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_combine_cloud_handler_kernel.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/collector_info.hpp"

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
using autoware::pointcloud_preprocessor::CombineCloudHandler;
using autoware::pointcloud_preprocessor::NaiveCollectorInfo;
using autoware::pointcloud_preprocessor::PointTypeStruct;
using cuda_blackboard::CudaPointCloud2;

constexpr char kOutputFrame[] = "base_link";

std::vector<sensor_msgs::msg::PointField> make_xyzirc_fields()
{
  auto field = [](const char * name, uint32_t offset, uint8_t datatype) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = datatype;
    f.count = 1;
    return f;
  };
  return {
    field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
    field("intensity", 12, sensor_msgs::msg::PointField::UINT8),
    field("return_type", 13, sensor_msgs::msg::PointField::UINT8),
    field("channel", 14, sensor_msgs::msg::PointField::UINT16)};
}

// Build a CudaPointCloud2 in the output frame (so no transform is needed) with its points uploaded
// to device memory.
CudaPointCloud2::ConstSharedPtr make_cloud(
  const std::vector<PointTypeStruct> & points, const std::string & frame_id = kOutputFrame)
{
  auto cloud = std::make_shared<CudaPointCloud2>();
  cloud->header.frame_id = frame_id;
  cloud->header.stamp.sec = 10;
  cloud->header.stamp.nanosec = 0;
  cloud->height = 1;
  cloud->width = static_cast<uint32_t>(points.size());
  cloud->fields = make_xyzirc_fields();
  cloud->is_bigendian = false;
  cloud->point_step = sizeof(PointTypeStruct);
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->is_dense = true;

  const std::size_t bytes = points.size() * sizeof(PointTypeStruct);
  cloud->data = cuda_blackboard::make_unique<std::uint8_t[]>(bytes);
  cudaMemcpy(cloud->data.get(), points.data(), bytes, cudaMemcpyHostToDevice);
  return cloud;
}

std::vector<PointTypeStruct> read_back(const CudaPointCloud2 & cloud)
{
  const std::size_t num_points = static_cast<std::size_t>(cloud.width) * cloud.height;
  std::vector<PointTypeStruct> points(num_points);
  cudaMemcpy(
    points.data(), cloud.data.get(), num_points * sizeof(PointTypeStruct), cudaMemcpyDeviceToHost);
  return points;
}

PointTypeStruct point(float x, float y, float z)
{
  PointTypeStruct p{};
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

bool contains(const std::vector<PointTypeStruct> & points, float x, float y, float z)
{
  for (const auto & p : points) {
    if (p.x == x && p.y == y && p.z == z) return true;
  }
  return false;
}
}  // namespace

class CudaCombineCloudHandlerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
      GTEST_SKIP() << "No CUDA device available";
    }
  }

  // Motion compensation off (no twist needed); clouds are already in the output frame.
  CombineCloudHandler<CudaPointCloud2> handler_{
    {"lidar_left", "lidar_right"},
    kOutputFrame,
    /*is_motion_compensated=*/false,
    /*publish_synchronized_pointcloud=*/false,
    /*keep_input_frame_in_synchronized_pointcloud=*/false,
    /*matching_strategy=*/"naive"};
};

TEST_F(CudaCombineCloudHandlerTest, ConcatenatesTwoCloudsOnGpu)
{
  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  topic_to_cloud_map["lidar_left"] = make_cloud({point(1.0f, 0.0f, 0.0f), point(0.0f, 2.0f, 0.0f)});
  topic_to_cloud_map["lidar_right"] =
    make_cloud({point(0.0f, 0.0f, 3.0f), point(4.0f, 0.0f, 0.0f)});

  auto result =
    handler_.combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  ASSERT_NE(result.concatenate_cloud_ptr, nullptr);
  EXPECT_EQ(result.concatenate_cloud_ptr->width, 4u);
  EXPECT_EQ(result.concatenate_cloud_ptr->header.frame_id, kOutputFrame);

  // Clouds are in the output frame, so points are concatenated unchanged (identity transform).
  const auto points = read_back(*result.concatenate_cloud_ptr);
  ASSERT_EQ(points.size(), 4u);
  EXPECT_TRUE(contains(points, 1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(contains(points, 0.0f, 2.0f, 0.0f));
  EXPECT_TRUE(contains(points, 0.0f, 0.0f, 3.0f));
  EXPECT_TRUE(contains(points, 4.0f, 0.0f, 0.0f));
}

TEST_F(CudaCombineCloudHandlerTest, DropsCloudWithoutExtrinsic)
{
  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  // lidar_left is in the output frame (identity transform, kept); lidar_right is in an unknown
  // sensor frame with no injected extrinsic, so it must be dropped (not identity-placed) and
  // reported, matching the PointCloud2 handler.
  topic_to_cloud_map["lidar_left"] = make_cloud({point(1.0f, 0.0f, 0.0f)});
  topic_to_cloud_map["lidar_right"] = make_cloud({point(9.0f, 0.0f, 0.0f)}, "unknown_sensor_frame");

  auto result =
    handler_.combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  ASSERT_NE(result.concatenate_cloud_ptr, nullptr);
  EXPECT_EQ(result.concatenate_cloud_ptr->width, 1u);  // only lidar_left survived
  ASSERT_EQ(result.dropped_frames_missing_transform.size(), 1u);
  EXPECT_EQ(result.dropped_frames_missing_transform[0], "unknown_sensor_frame");

  const auto points = read_back(*result.concatenate_cloud_ptr);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(contains(points, 1.0f, 0.0f, 0.0f));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
