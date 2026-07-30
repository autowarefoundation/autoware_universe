// Copyright 2026 TIER IV, Inc.
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
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/queue_bounds.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <cstring>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{

PreprocessorCapacity make_capacity(
  const std::size_t max_input_point_count = 2, const int max_ring_count = 1,
  const int max_points_per_ring = 4, const std::size_t max_twist_struct_count = 1)
{
  PreprocessorCapacity capacity;
  capacity.max_input_point_count = max_input_point_count;
  capacity.max_ring_count = max_ring_count;
  capacity.max_points_per_ring = max_points_per_ring;
  capacity.max_twist_struct_count = max_twist_struct_count;
  return capacity;
}

sensor_msgs::msg::PointField make_point_field(
  const std::string & name, const std::uint32_t offset, const std::uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

sensor_msgs::msg::PointCloud2 make_input_cloud(const std::vector<InputPointType> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.header.stamp.sec = 1;
  cloud.height = 1;
  cloud.width = points.size();
  cloud.fields = {
    make_point_field("x", offsetof(InputPointType, x), sensor_msgs::msg::PointField::FLOAT32),
    make_point_field("y", offsetof(InputPointType, y), sensor_msgs::msg::PointField::FLOAT32),
    make_point_field("z", offsetof(InputPointType, z), sensor_msgs::msg::PointField::FLOAT32),
    make_point_field(
      "intensity", offsetof(InputPointType, intensity), sensor_msgs::msg::PointField::UINT8),
    make_point_field(
      "return_type", offsetof(InputPointType, return_type), sensor_msgs::msg::PointField::UINT8),
    make_point_field(
      "channel", offsetof(InputPointType, channel), sensor_msgs::msg::PointField::UINT16),
    make_point_field(
      "azimuth", offsetof(InputPointType, azimuth), sensor_msgs::msg::PointField::FLOAT32),
    make_point_field(
      "elevation", offsetof(InputPointType, elevation), sensor_msgs::msg::PointField::FLOAT32),
    make_point_field(
      "distance", offsetof(InputPointType, distance), sensor_msgs::msg::PointField::FLOAT32),
    make_point_field(
      "time_stamp", offsetof(InputPointType, time_stamp), sensor_msgs::msg::PointField::UINT32)};
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(InputPointType);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.is_dense = true;
  cloud.data.resize(cloud.row_step);
  std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
  return cloud;
}

geometry_msgs::msg::TransformStamped make_identity_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "lidar";
  transform.transform.rotation.w = 1.0;
  return transform;
}

InputPointType make_point(const float x, const std::uint32_t stamp, const std::uint16_t channel = 0)
{
  InputPointType point{};
  point.x = x;
  point.distance = x;
  point.channel = channel;
  point.time_stamp = stamp;
  return point;
}

builtin_interfaces::msg::Time make_stamp(const std::uint32_t nanosec, const std::int32_t sec = 0)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

geometry_msgs::msg::TwistWithCovarianceStamped make_twist(
  const std::uint32_t nanosec, const std::int32_t sec = 0)
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header.stamp = make_stamp(nanosec, sec);
  return twist;
}

sensor_msgs::msg::Imu make_imu(const std::uint32_t nanosec)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = make_stamp(nanosec);
  return imu;
}

geometry_msgs::msg::Vector3Stamped make_angular_velocity(const std::uint32_t nanosec)
{
  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header.stamp = make_stamp(nanosec);
  return angular_velocity;
}

std::shared_ptr<const geometry_msgs::msg::TwistWithCovarianceStamped> make_twist_ptr(
  const std::uint32_t nanosec)
{
  return std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(make_twist(nanosec));
}

std::shared_ptr<const sensor_msgs::msg::Imu> make_imu_ptr(const std::uint32_t nanosec)
{
  return std::make_shared<sensor_msgs::msg::Imu>(make_imu(nanosec));
}

// Constructing a CudaPointcloudPreprocessor allocates device memory, so the cases
// below need a CUDA device and self-skip where there is none (e.g. on CI).
class CudaPointcloudPreprocessorDeviceTest : public autoware::cuda_utils::CudaTest
{
};

TEST_F(CudaPointcloudPreprocessorDeviceTest, AcceptsValidCapacities)
{
  EXPECT_NO_THROW(CudaPointcloudPreprocessor{make_capacity()});
}

TEST(CudaPointcloudPreprocessor, RejectsInvalidCapacities)
{
  auto capacity = make_capacity();
  capacity.max_input_point_count = 0;
  EXPECT_THROW(CudaPointcloudPreprocessor{capacity}, std::runtime_error);

  capacity = make_capacity();
  capacity.max_ring_count = 0;
  EXPECT_THROW(CudaPointcloudPreprocessor{capacity}, std::runtime_error);

  capacity = make_capacity();
  capacity.max_points_per_ring = 0;
  EXPECT_THROW(CudaPointcloudPreprocessor{capacity}, std::runtime_error);

  capacity = make_capacity();
  capacity.max_twist_struct_count = 0;
  EXPECT_THROW(CudaPointcloudPreprocessor{capacity}, std::runtime_error);
}

TEST_F(CudaPointcloudPreprocessorDeviceTest, ProcessesPointsAtRingCapacityBoundary)
{
  CudaPointcloudPreprocessor preprocessor{make_capacity(4, 2, 2)};
  preprocessor.setRingOutlierFilterActive(false);
  preprocessor.setUndistortionType(CudaPointcloudPreprocessor::UndistortionType::Undistortion2D);

  const auto input_cloud = make_input_cloud(
    {make_point(1.0F, 0, 0), make_point(2.0F, 1, 0), make_point(3.0F, 2, 1),
     make_point(4.0F, 3, 1)});

  const auto output_cloud = preprocessor.process(
    input_cloud, make_identity_transform(),
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped>{},
    std::deque<geometry_msgs::msg::Vector3Stamped>{}, 0U);

  ASSERT_NE(output_cloud, nullptr);
  EXPECT_EQ(output_cloud->width, 4U);
  EXPECT_FALSE(preprocessor.getProcessingStats().ring_overflow);
}

TEST_F(CudaPointcloudPreprocessorDeviceTest, ReportsRingIndexOverflow)
{
  CudaPointcloudPreprocessor preprocessor{make_capacity(2, 1, 2)};
  preprocessor.setRingOutlierFilterActive(false);
  preprocessor.setUndistortionType(CudaPointcloudPreprocessor::UndistortionType::Undistortion2D);

  const auto input_cloud = make_input_cloud({make_point(1.0F, 0, 0), make_point(2.0F, 1, 1)});

  const auto output_cloud = preprocessor.process(
    input_cloud, make_identity_transform(),
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped>{},
    std::deque<geometry_msgs::msg::Vector3Stamped>{}, 0U);

  ASSERT_NE(output_cloud, nullptr);
  EXPECT_TRUE(preprocessor.getProcessingStats().ring_overflow);
}

TEST_F(CudaPointcloudPreprocessorDeviceTest, ReportsPointsPerRingOverflow)
{
  CudaPointcloudPreprocessor preprocessor{make_capacity(3, 1, 2)};
  preprocessor.setRingOutlierFilterActive(false);
  preprocessor.setUndistortionType(CudaPointcloudPreprocessor::UndistortionType::Undistortion2D);

  const auto input_cloud =
    make_input_cloud({make_point(1.0F, 0), make_point(2.0F, 1), make_point(3.0F, 2)});

  const auto output_cloud = preprocessor.process(
    input_cloud, make_identity_transform(),
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped>{},
    std::deque<geometry_msgs::msg::Vector3Stamped>{}, 0U);

  ASSERT_NE(output_cloud, nullptr);
  EXPECT_EQ(output_cloud->width, 2U);
  EXPECT_TRUE(preprocessor.getProcessingStats().ring_overflow);
}

TEST(QueueBounds, PrunesOldTwistsBeforeComputingFreeCapacity)
{
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> queue{
    make_twist(100), make_twist(200)};
  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> messages{
    make_twist_ptr(50), make_twist_ptr(160), make_twist_ptr(170), make_twist_ptr(180)};

  const auto dropped_count = detail::prepare_queue_update(queue, messages, 4U, 150U);

  ASSERT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.front().header.stamp.nanosec, 200U);
  ASSERT_EQ(messages.size(), 3U);
  EXPECT_EQ(messages.at(0)->header.stamp.nanosec, 160U);
  EXPECT_EQ(messages.at(1)->header.stamp.nanosec, 170U);
  EXPECT_EQ(messages.at(2)->header.stamp.nanosec, 180U);
  EXPECT_EQ(dropped_count, 0U);
}

TEST(QueueBounds, DropsOldestNewTwistsWhenCapacityIsExhausted)
{
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> queue{
    make_twist(100), make_twist(200)};
  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> messages{
    make_twist_ptr(210), make_twist_ptr(220), make_twist_ptr(230), make_twist_ptr(240)};

  const auto dropped_count = detail::prepare_queue_update(queue, messages, 4U, 100U);

  ASSERT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.front().header.stamp.nanosec, 100U);
  EXPECT_EQ(queue.back().header.stamp.nanosec, 200U);
  ASSERT_EQ(messages.size(), 2U);
  EXPECT_EQ(messages.at(0)->header.stamp.nanosec, 230U);
  EXPECT_EQ(messages.at(1)->header.stamp.nanosec, 240U);
  EXPECT_EQ(dropped_count, 2U);
}

TEST(QueueBounds, AppliesSamePruneAndDropRulesToImuQueue)
{
  std::deque<geometry_msgs::msg::Vector3Stamped> queue{
    make_angular_velocity(100), make_angular_velocity(200)};
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> messages{
    make_imu_ptr(50), make_imu_ptr(210), make_imu_ptr(220), make_imu_ptr(230)};

  const auto dropped_count = detail::prepare_queue_update(queue, messages, 3U, 150U);

  ASSERT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.front().header.stamp.nanosec, 200U);
  ASSERT_EQ(messages.size(), 2U);
  EXPECT_EQ(messages.at(0)->header.stamp.nanosec, 220U);
  EXPECT_EQ(messages.at(1)->header.stamp.nanosec, 230U);
  EXPECT_EQ(dropped_count, 1U);
}

TEST(QueueBounds, SortsIncomingMessagesBeforeDroppingOldestOverflow)
{
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> queue{make_twist(100)};
  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> messages{
    make_twist_ptr(240), make_twist_ptr(210), make_twist_ptr(230), make_twist_ptr(220)};

  const auto dropped_count = detail::prepare_queue_update(queue, messages, 3U, 100U);

  ASSERT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.front().header.stamp.nanosec, 100U);
  ASSERT_EQ(messages.size(), 2U);
  EXPECT_EQ(messages.at(0)->header.stamp.nanosec, 230U);
  EXPECT_EQ(messages.at(1)->header.stamp.nanosec, 240U);
  EXPECT_EQ(dropped_count, 2U);
}

TEST(QueueBounds, AllowsIncomingMessageWithinBackwardJumpThreshold)
{
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> queue{make_twist(500'000'000U, 1)};

  EXPECT_FALSE(detail::is_backward_time_jump(queue, make_stamp(500'000'000U)));
}

TEST(QueueBounds, RejectsIncomingMessageBeyondBackwardJumpThreshold)
{
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> queue{make_twist(500'000'001U, 1)};

  EXPECT_TRUE(detail::is_backward_time_jump(queue, make_stamp(500'000'000U)));
}

TEST(QueueBounds, RejectsAlreadyOverCapacityInternalQueue)
{
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> queue{
    make_twist(100), make_twist(200), make_twist(300)};
  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> messages;

  EXPECT_THROW(detail::prepare_queue_update(queue, messages, 2U, 100U), std::runtime_error);
}

TEST_F(CudaPointcloudPreprocessorDeviceTest, TruncatesInputCloudToConfiguredMaximum)
{
  CudaPointcloudPreprocessor preprocessor{make_capacity()};
  preprocessor.setRingOutlierFilterActive(false);
  preprocessor.setUndistortionType(CudaPointcloudPreprocessor::UndistortionType::Undistortion2D);

  const auto input_cloud =
    make_input_cloud({make_point(1.0F, 0), make_point(2.0F, 1), make_point(3.0F, 2)});

  const auto output_cloud = preprocessor.process(
    input_cloud, make_identity_transform(),
    std::deque<geometry_msgs::msg::TwistWithCovarianceStamped>{},
    std::deque<geometry_msgs::msg::Vector3Stamped>{}, 0U);

  ASSERT_NE(output_cloud, nullptr);
  EXPECT_EQ(output_cloud->height, 1U);
  EXPECT_EQ(output_cloud->width, 2U);
  EXPECT_EQ(output_cloud->row_step, 2U * sizeof(OutputPointType));
  EXPECT_FALSE(preprocessor.getProcessingStats().ring_overflow);
}

}  // namespace
}  // namespace autoware::cuda_pointcloud_preprocessor
