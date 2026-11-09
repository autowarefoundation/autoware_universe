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

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/point_types/types.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
using autoware::point_types::PointXYZIRC;
using autoware::point_types::PointXYZIRCT;
using autoware::pointcloud_preprocessor::CombineCloudHandler;
using autoware::pointcloud_preprocessor::NaiveCollectorInfo;
using autoware::pointcloud_preprocessor::PointFormat;
using cuda_blackboard::CudaPointCloud2;

constexpr char kOutputFrame[] = "base_link";
constexpr std::uint32_t kMs = 1'000'000;

builtin_interfaces::msg::Time make_stamp(const int32_t sec, const uint32_t nanosec = 0)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

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

// Build a CudaPointCloud2 with its points uploaded to device memory. The fields describe PointT;
// by default the cloud is in the output frame (so no transform is needed) and stamped 10 s.
template <typename PointT>
CudaPointCloud2::ConstSharedPtr make_cloud(
  const std::vector<PointT> & points, const std::vector<sensor_msgs::msg::PointField> & fields,
  const builtin_interfaces::msg::Time & stamp = make_stamp(10),
  const std::string & frame_id = kOutputFrame)
{
  auto cloud = std::make_shared<CudaPointCloud2>();
  cloud->header.frame_id = frame_id;
  cloud->header.stamp = stamp;
  cloud->height = 1;
  cloud->width = static_cast<uint32_t>(points.size());
  cloud->fields = fields;
  cloud->is_bigendian = false;
  cloud->point_step = sizeof(PointT);
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->is_dense = true;

  const std::size_t bytes = points.size() * sizeof(PointT);
  cloud->data = cuda_blackboard::make_unique<std::uint8_t[]>(std::max<std::size_t>(bytes, 1));
  if (bytes > 0) {
    cudaMemcpy(cloud->data.get(), points.data(), bytes, cudaMemcpyHostToDevice);
  }
  return cloud;
}

CudaPointCloud2::ConstSharedPtr make_cloud(
  const std::vector<PointXYZIRC> & points, const std::string & frame_id = kOutputFrame)
{
  return make_cloud(points, make_xyzirc_fields(), make_stamp(10), frame_id);
}

template <typename PointT = PointXYZIRC>
std::vector<PointT> read_back(const CudaPointCloud2 & cloud)
{
  const std::size_t num_points = static_cast<std::size_t>(cloud.width) * cloud.height;
  std::vector<PointT> points(num_points);
  if (num_points > 0) {
    cudaMemcpy(
      points.data(), cloud.data.get(), num_points * sizeof(PointT), cudaMemcpyDeviceToHost);
  }
  return points;
}

PointXYZIRC point(float x, float y, float z)
{
  PointXYZIRC p{};
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

bool contains(const std::vector<PointXYZIRC> & points, float x, float y, float z)
{
  constexpr float kTolerance = 1e-5f;
  for (const auto & p : points) {
    if (
      std::abs(p.x - x) < kTolerance && std::abs(p.y - y) < kTolerance &&
      std::abs(p.z - z) < kTolerance) {
      return true;
    }
  }
  return false;
}

PointXYZIRCT timed_point(
  const float x, const float y, const float z, const std::uint32_t time_stamp)
{
  PointXYZIRCT p{};
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 7;
  p.return_type = 1;
  p.channel = 3;
  p.time_stamp = time_stamp;
  return p;
}

std::vector<std::uint32_t> sorted_time_stamps(const std::vector<PointXYZIRCT> & points)
{
  std::vector<std::uint32_t> time_stamps;
  for (const auto & p : points) {
    time_stamps.push_back(p.time_stamp);
  }
  std::sort(time_stamps.begin(), time_stamps.end());
  return time_stamps;
}

sensor_msgs::msg::PointCloud2 make_layout(
  const std::vector<sensor_msgs::msg::PointField> & fields, const std::uint32_t point_step,
  const std::uint32_t width = 2)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.fields = fields;
  cloud.point_step = point_step;
  cloud.width = width;
  cloud.height = 1;
  cloud.row_step = point_step * width;
  return cloud;
}
}  // namespace

class CudaCombineCloudHandlerTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override
  {
    // Skips the test when no CUDA device is usable, e.g. on a CI runner without a GPU.
    autoware::cuda_utils::CudaTest::SetUp();
    if (IsSkipped()) return;

    // Built here rather than as a member initializer: the constructor creates a CUDA stream, and a
    // member would be constructed before SetUp() runs, throwing past the skip above.
    // Motion compensation off (no twist needed); clouds are already in the output frame.
    handler_.emplace(
      std::vector<std::string>{"lidar_left", "lidar_right"}, kOutputFrame,
      autoware::pointcloud_preprocessor::PointFormat::XYZIRC,
      /*is_motion_compensated=*/false,
      /*publish_synchronized_pointcloud=*/false,
      /*keep_input_frame_in_synchronized_pointcloud=*/false,
      autoware::pointcloud_preprocessor::MatchingStrategyType::naive);
  }

  std::optional<CombineCloudHandler<CudaPointCloud2>> handler_;
};

TEST_F(CudaCombineCloudHandlerTest, ConcatenatesTwoCloudsOnGpu)
{
  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  topic_to_cloud_map["lidar_left"] = make_cloud({point(1.0f, 0.0f, 0.0f), point(0.0f, 2.0f, 0.0f)});
  topic_to_cloud_map["lidar_right"] =
    make_cloud({point(0.0f, 0.0f, 3.0f), point(4.0f, 0.0f, 0.0f)});

  auto result =
    handler_->combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

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
    handler_->combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  ASSERT_NE(result.concatenate_cloud_ptr, nullptr);
  EXPECT_EQ(result.concatenate_cloud_ptr->width, 1u);  // only lidar_left survived
  ASSERT_EQ(result.dropped_sources_missing_transform.size(), 1u);
  EXPECT_EQ(result.dropped_sources_missing_transform[0].frame_id, "unknown_sensor_frame");
  EXPECT_EQ(result.dropped_sources_missing_transform[0].topic, "lidar_right");

  const auto points = read_back(*result.concatenate_cloud_ptr);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_TRUE(contains(points, 1.0f, 0.0f, 0.0f));
}

// output_point_type == XYZIRCT: per-point times are rebased onto the output header stamp (the
// oldest input stamp), on the concatenated cloud and on the synchronized per-topic clouds.
class CudaCombineCloudHandlerXyzirctTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override
  {
    autoware::cuda_utils::CudaTest::SetUp();
    if (IsSkipped()) return;

    handler_.emplace(
      std::vector<std::string>{"lidar_left", "lidar_right"}, kOutputFrame, PointFormat::XYZIRCT,
      /*is_motion_compensated=*/false,
      /*publish_synchronized_pointcloud=*/true,
      /*keep_input_frame_in_synchronized_pointcloud=*/true,
      autoware::pointcloud_preprocessor::MatchingStrategyType::naive);
    xyzirc_handler_.emplace(
      std::vector<std::string>{"lidar_left", "lidar_right"}, kOutputFrame, PointFormat::XYZIRC,
      /*is_motion_compensated=*/false,
      /*publish_synchronized_pointcloud=*/true,
      /*keep_input_frame_in_synchronized_pointcloud=*/true,
      autoware::pointcloud_preprocessor::MatchingStrategyType::naive);
  }

  const std::vector<sensor_msgs::msg::PointField> xyzirct_fields_ =
    autoware::point_types::create_fields_point_xyzirct();
  std::optional<CombineCloudHandler<CudaPointCloud2>> handler_;
  std::optional<CombineCloudHandler<CudaPointCloud2>> xyzirc_handler_;
};

TEST_F(
  CudaCombineCloudHandlerXyzirctTest, RebasesTimesOntoTheOldestStampForConcatenatedAndSyncClouds)
{
  const auto oldest = make_stamp(10, 100 * kMs);
  const auto later = make_stamp(10, 140 * kMs);  // +40 ms
  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  topic_to_cloud_map["lidar_left"] = make_cloud<PointXYZIRCT>(
    {timed_point(1.0f, 0.0f, 0.0f, 0), timed_point(0.0f, 2.0f, 0.0f, 10 * kMs)}, xyzirct_fields_,
    oldest);
  topic_to_cloud_map["lidar_right"] = make_cloud<PointXYZIRCT>(
    {timed_point(0.0f, 0.0f, 3.0f, 0), timed_point(4.0f, 0.0f, 0.0f, 10 * kMs)}, xyzirct_fields_,
    later);

  auto result =
    handler_->combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  ASSERT_NE(result.concatenate_cloud_ptr, nullptr);
  const auto & concatenated = *result.concatenate_cloud_ptr;
  EXPECT_EQ(concatenated.point_step, sizeof(PointXYZIRCT));
  EXPECT_EQ(concatenated.fields, xyzirct_fields_);
  EXPECT_EQ(concatenated.width, 4u);
  EXPECT_EQ(concatenated.row_step, 4u * sizeof(PointXYZIRCT));
  EXPECT_EQ(rclcpp::Time(concatenated.header.stamp), rclcpp::Time(oldest));
  const auto points = read_back<PointXYZIRCT>(concatenated);
  EXPECT_EQ(
    sorted_time_stamps(points), (std::vector<std::uint32_t>{0, 10 * kMs, 40 * kMs, 50 * kMs}));
  for (const auto & p : points) {
    EXPECT_EQ(p.intensity, 7);
    EXPECT_EQ(p.return_type, 1);
    EXPECT_EQ(p.channel, 3);
  }

  ASSERT_TRUE(result.topic_to_transformed_cloud_map.has_value());
  const auto & synchronized = *result.topic_to_transformed_cloud_map;
  ASSERT_EQ(synchronized.size(), 2u);
  const auto & right = *synchronized.at("lidar_right");
  EXPECT_EQ(rclcpp::Time(right.header.stamp), rclcpp::Time(oldest));
  EXPECT_EQ(right.point_step, sizeof(PointXYZIRCT));
  EXPECT_EQ(right.fields, xyzirct_fields_);
  EXPECT_EQ(
    sorted_time_stamps(read_back<PointXYZIRCT>(*synchronized.at("lidar_left"))),
    (std::vector<std::uint32_t>{0, 10 * kMs}));
  EXPECT_EQ(
    sorted_time_stamps(read_back<PointXYZIRCT>(right)),
    (std::vector<std::uint32_t>{40 * kMs, 50 * kMs}));
}

TEST_F(CudaCombineCloudHandlerXyzirctTest, KeepInputFrameSecondPassDoesNotRebaseTwice)
{
  // lidar_right sits 1 m ahead of base_link. With keep_input_frame_in_synchronized_pointcloud the
  // second transform pass runs on the already-rebased output buffer, so it must restore the sensor
  // frame coordinates without adding the time offset again.
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = kOutputFrame;
  transform.child_frame_id = "lidar_right";
  transform.transform.translation.x = 1.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(handler_->set_transform(transform));

  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  topic_to_cloud_map["lidar_left"] = make_cloud<PointXYZIRCT>(
    {timed_point(1.0f, 0.0f, 0.0f, 5 * kMs)}, xyzirct_fields_, make_stamp(10));
  topic_to_cloud_map["lidar_right"] = make_cloud<PointXYZIRCT>(
    {timed_point(2.0f, 0.0f, 0.0f, 5 * kMs)}, xyzirct_fields_, make_stamp(10, 40 * kMs),
    "lidar_right");

  auto result =
    handler_->combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  const auto concatenated = read_back<PointXYZIRCT>(*result.concatenate_cloud_ptr);
  EXPECT_EQ(sorted_time_stamps(concatenated), (std::vector<std::uint32_t>{5 * kMs, 45 * kMs}));
  EXPECT_TRUE(std::any_of(concatenated.begin(), concatenated.end(), [](const PointXYZIRCT & p) {
    return std::abs(p.x - 3.0f) < 1e-5f;  // 2 m in the sensor frame + 1 m offset
  }));

  ASSERT_TRUE(result.topic_to_transformed_cloud_map.has_value());
  const auto & right_cloud = *result.topic_to_transformed_cloud_map->at("lidar_right");
  EXPECT_EQ(right_cloud.header.frame_id, "lidar_right");
  const auto right = read_back<PointXYZIRCT>(right_cloud);
  ASSERT_EQ(right.size(), 1u);
  EXPECT_NEAR(right[0].x, 2.0f, 1e-5f);
  EXPECT_EQ(right[0].time_stamp, 45 * kMs);
  EXPECT_EQ(right[0].intensity, 7);
}

TEST_F(CudaCombineCloudHandlerXyzirctTest, IdenticalStampsLeaveTimesUnchanged)
{
  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  topic_to_cloud_map["lidar_left"] =
    make_cloud<PointXYZIRCT>({timed_point(1.0f, 0.0f, 0.0f, 1 * kMs)}, xyzirct_fields_);
  topic_to_cloud_map["lidar_right"] =
    make_cloud<PointXYZIRCT>({timed_point(2.0f, 0.0f, 0.0f, 2 * kMs)}, xyzirct_fields_);

  auto result =
    handler_->combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  EXPECT_EQ(
    sorted_time_stamps(read_back<PointXYZIRCT>(*result.concatenate_cloud_ptr)),
    (std::vector<std::uint32_t>{1 * kMs, 2 * kMs}));
}

TEST_F(CudaCombineCloudHandlerXyzirctTest, HandlesAnEmptyInputCloud)
{
  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  topic_to_cloud_map["lidar_left"] = make_cloud<PointXYZIRCT>({}, xyzirct_fields_, make_stamp(10));
  topic_to_cloud_map["lidar_right"] = make_cloud<PointXYZIRCT>(
    {timed_point(2.0f, 0.0f, 0.0f, 2 * kMs)}, xyzirct_fields_, make_stamp(10, 30 * kMs));

  auto result =
    handler_->combine_pointclouds(topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  EXPECT_EQ(result.concatenate_cloud_ptr->width, 1u);
  EXPECT_EQ(
    sorted_time_stamps(read_back<PointXYZIRCT>(*result.concatenate_cloud_ptr)),
    (std::vector<std::uint32_t>{32 * kMs}));
  ASSERT_TRUE(result.topic_to_transformed_cloud_map.has_value());
  EXPECT_EQ(result.topic_to_transformed_cloud_map->at("lidar_left")->width, 0u);
}

TEST_F(
  CudaCombineCloudHandlerXyzirctTest, XyzircOutputIgnoresTheTimeOffsetAndKeepsSixteenBytePoints)
{
  PointXYZIRC left{};
  left.x = 1.0f;
  left.intensity = 9;
  left.channel = 5;
  PointXYZIRC right{};
  right.y = 2.0f;
  right.intensity = 8;
  right.channel = 4;
  std::unordered_map<std::string, CudaPointCloud2::ConstSharedPtr> topic_to_cloud_map;
  topic_to_cloud_map["lidar_left"] =
    make_cloud<PointXYZIRC>({left}, make_xyzirc_fields(), make_stamp(10));
  topic_to_cloud_map["lidar_right"] =
    make_cloud<PointXYZIRC>({right}, make_xyzirc_fields(), make_stamp(10, 40 * kMs));

  auto result = xyzirc_handler_->combine_pointclouds(
    topic_to_cloud_map, std::make_shared<NaiveCollectorInfo>());

  EXPECT_EQ(result.concatenate_cloud_ptr->point_step, sizeof(PointXYZIRC));
  EXPECT_EQ(result.concatenate_cloud_ptr->fields, make_xyzirc_fields());
  const auto points = read_back(*result.concatenate_cloud_ptr);
  ASSERT_EQ(points.size(), 2u);
  EXPECT_TRUE(std::any_of(points.begin(), points.end(), [](const PointXYZIRC & p) {
    return p.intensity == 9 && p.channel == 5;
  }));
  EXPECT_TRUE(std::any_of(points.begin(), points.end(), [](const PointXYZIRC & p) {
    return p.intensity == 8 && p.channel == 4;
  }));
}

TEST_F(CudaCombineCloudHandlerXyzirctTest, InputLayoutGateChecksPointStepFieldsAndRowPadding)
{
  const auto xyzirc = make_layout(make_xyzirc_fields(), sizeof(PointXYZIRC));
  const auto xyzirct = make_layout(xyzirct_fields_, sizeof(PointXYZIRCT));
  const auto xyzircaedt = make_layout(
    autoware::point_types::create_fields_point_xyzircaedt(),
    sizeof(autoware::point_types::PointXYZIRCAEDT));
  auto xyzirc_wide_step = xyzirc;
  xyzirc_wide_step.point_step = 32;
  xyzirc_wide_step.row_step = 32 * xyzirc_wide_step.width;
  auto xyzirc_padded_rows = xyzirc;
  xyzirc_padded_rows.row_step += 4;

  EXPECT_TRUE(xyzirc_handler_->is_input_layout_supported(xyzirc));
  EXPECT_FALSE(xyzirc_handler_->is_input_layout_supported(xyzirct));
  EXPECT_FALSE(xyzirc_handler_->is_input_layout_supported(xyzircaedt));
  EXPECT_FALSE(xyzirc_handler_->is_input_layout_supported(xyzirc_wide_step));
  EXPECT_FALSE(xyzirc_handler_->is_input_layout_supported(xyzirc_padded_rows));

  EXPECT_TRUE(handler_->is_input_layout_supported(xyzirct));
  EXPECT_FALSE(handler_->is_input_layout_supported(xyzirc));
  EXPECT_FALSE(handler_->is_input_layout_supported(xyzircaedt));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
