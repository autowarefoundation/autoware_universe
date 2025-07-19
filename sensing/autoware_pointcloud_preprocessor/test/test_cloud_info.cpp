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

#include <autoware/pointcloud_preprocessor/concatenate_data/cloud_info.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <autoware_sensing_msgs/msg/source_point_cloud_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <string>
#include <vector>

using autoware::pointcloud_preprocessor::CloudInfo;
using autoware::pointcloud_preprocessor::StrategyAdvancedConfig;

class CloudInfoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup test data
    input_topics_ = {"/topic1", "/topic2", "/topic3"};
    strategy_name_ = "advanced";

    // Create test header
    test_header_.frame_id = "base_link";
    test_header_.stamp.sec = 1234567891;
    test_header_.stamp.nanosec = 987654321;

    // Create test point clouds
    pcl::PointXYZ pt(1.0f, 2.0f, 3.0f);
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_1;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_2;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_3;

    // Cloud 1
    pcl_cloud_1.width = 100;
    pcl_cloud_1.height = 1;
    pcl_cloud_1.is_dense = false;
    pcl_cloud_1.points.resize(pcl_cloud_1.width * pcl_cloud_1.height);
    std::fill(pcl_cloud_1.points.begin(), pcl_cloud_1.points.end(), pt);

    // Cloud 2
    pcl_cloud_2.width = 150;
    pcl_cloud_2.height = 1;
    pcl_cloud_2.is_dense = false;
    pcl_cloud_2.points.resize(pcl_cloud_2.width * pcl_cloud_2.height);
    std::fill(pcl_cloud_2.points.begin(), pcl_cloud_2.points.end(), pt);

    // Cloud 3
    pcl_cloud_3.width = 200;
    pcl_cloud_3.height = 1;
    pcl_cloud_3.is_dense = false;
    pcl_cloud_3.points.resize(pcl_cloud_3.width * pcl_cloud_3.height);
    std::fill(pcl_cloud_3.points.begin(), pcl_cloud_3.points.end(), pt);

    // Convert to ROS messages
    pcl::toROSMsg(pcl_cloud_1, test_cloud_1_);
    pcl::toROSMsg(pcl_cloud_2, test_cloud_2_);
    pcl::toROSMsg(pcl_cloud_3, test_cloud_3_);
    test_cloud_1_.header = test_header_;
    test_cloud_2_.header = test_header_;
    test_cloud_3_.header = test_header_;
  }

  std::vector<std::string> input_topics_;
  std::string strategy_name_;
  sensor_msgs::msg::PointCloud2 test_cloud_1_;
  sensor_msgs::msg::PointCloud2 test_cloud_2_;
  sensor_msgs::msg::PointCloud2 test_cloud_3_;
  std_msgs::msg::Header test_header_;
};

TEST_F(CloudInfoTest, ConstructorAndGetConcatInfoBase)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);

  auto concat_info = cloud_info.reset_and_get_base_info();

  // Check that source_info is populated with correct topics
  ASSERT_EQ(concat_info.source_info.size(), input_topics_.size());

  for (size_t i = 0; i < input_topics_.size(); ++i) {
    EXPECT_EQ(concat_info.source_info[i].topic, input_topics_[i]);
    EXPECT_EQ(
      concat_info.source_info[i].status,
      autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT);
    EXPECT_EQ(concat_info.source_info[i].idx_begin, 0u);
    EXPECT_EQ(concat_info.source_info[i].length, 0u);
  }

  // Check strategy is set correctly
  EXPECT_EQ(
    concat_info.matching_strategy,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_ADVANCED);
}

TEST_F(CloudInfoTest, InvalidMatchingStrategy)
{
  EXPECT_THROW(CloudInfo cloud_info("invalid_strategy", input_topics_), std::invalid_argument);
}

TEST_F(CloudInfoTest, ApplySourceWithPointCloud)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  // Apply point cloud to first topic
  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Check that the first topic was updated correctly
  auto & first_source = concat_info.source_info[0];
  EXPECT_EQ(first_source.topic, input_topics_[0]);
  EXPECT_EQ(first_source.status, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK);
  EXPECT_EQ(first_source.header.frame_id, test_cloud_1_.header.frame_id);
  EXPECT_EQ(first_source.header.stamp.sec, test_cloud_1_.header.stamp.sec);
  EXPECT_EQ(first_source.header.stamp.nanosec, test_cloud_1_.header.stamp.nanosec);
  EXPECT_EQ(first_source.idx_begin, 0u);
  EXPECT_EQ(first_source.length, test_cloud_1_.width * test_cloud_1_.height);

  // Apply second point cloud
  cloud_info.apply_source_with_point_cloud(
    test_cloud_2_, input_topics_[1], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Check that idx_begin is calculated correctly for second topic
  auto & second_source = concat_info.source_info[1];
  EXPECT_EQ(second_source.idx_begin, test_cloud_1_.width * test_cloud_1_.height);
  EXPECT_EQ(second_source.length, test_cloud_2_.width * test_cloud_2_.height);
}

TEST_F(CloudInfoTest, ApplySourceWithPointCloudNonOkStatus)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  // Apply point cloud with ERROR status
  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0],
    autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID, concat_info);

  // Check that header and status are updated, but idx_begin and length are not
  auto & first_source = concat_info.source_info[0];
  EXPECT_EQ(first_source.status, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID);
  EXPECT_EQ(first_source.header.frame_id, test_cloud_1_.header.frame_id);
  EXPECT_EQ(first_source.idx_begin, 0u);  // Should remain 0
  EXPECT_EQ(first_source.length, 0u);     // Should remain 0
}

TEST_F(CloudInfoTest, ApplySourceWithHeader)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  cloud_info.apply_source_with_header(
    test_header_, input_topics_[0],
    autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID, concat_info);

  auto & first_source = concat_info.source_info[0];
  EXPECT_EQ(first_source.status, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID);
  EXPECT_EQ(first_source.header.frame_id, test_header_.frame_id);
  EXPECT_EQ(first_source.header.stamp.sec, test_header_.stamp.sec);
  EXPECT_EQ(first_source.header.stamp.nanosec, test_header_.stamp.nanosec);
}

TEST_F(CloudInfoTest, ApplySourceWithStatus)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  cloud_info.apply_source_with_status(
    input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT,
    concat_info);

  auto & first_source = concat_info.source_info[0];
  EXPECT_EQ(first_source.status, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT);
  EXPECT_EQ(first_source.topic, input_topics_[0]);
}

TEST_F(CloudInfoTest, UpdateConcatenatedPointCloudConfig)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  // Create AdvancedStrategy config with test timestamps
  builtin_interfaces::msg::Time reference_timestamp_min;
  reference_timestamp_min.sec = 1234567890;
  reference_timestamp_min.nanosec = 100000000;

  builtin_interfaces::msg::Time reference_timestamp_max;
  reference_timestamp_max.sec = 1234567890;
  reference_timestamp_max.nanosec = 900000000;

  auto cfg = StrategyAdvancedConfig(reference_timestamp_min, reference_timestamp_max);
  CloudInfo::update_concatenated_point_cloud_config(cfg.serialize(), concat_info);

  // Verify that the config was serialized and stored
  EXPECT_FALSE(concat_info.matching_strategy_config.empty());

  // Verify we can deserialize it back
  auto deserialized_cfg = StrategyAdvancedConfig(concat_info.matching_strategy_config);
  EXPECT_EQ(deserialized_cfg.reference_timestamp_min.sec, reference_timestamp_min.sec);
  EXPECT_EQ(deserialized_cfg.reference_timestamp_min.nanosec, reference_timestamp_min.nanosec);
  EXPECT_EQ(deserialized_cfg.reference_timestamp_max.sec, reference_timestamp_max.sec);
  EXPECT_EQ(deserialized_cfg.reference_timestamp_max.nanosec, reference_timestamp_max.nanosec);
}

TEST_F(CloudInfoTest, StrategyAdvancedConfigInvalidSerializedData)
{
  // Test with invalid serialized data size
  std::vector<uint8_t> invalid_data = {0x01, 0x02, 0x03};  // Too small

  EXPECT_THROW(StrategyAdvancedConfig cfg(invalid_data), std::invalid_argument);
}

TEST_F(CloudInfoTest, ApplySourceWithNonExistentTopic)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  EXPECT_THROW(
    cloud_info.apply_source_with_point_cloud(
      test_cloud_1_, "/non_existent_topic",
      autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK, concat_info),
    std::runtime_error);
}

TEST_F(CloudInfoTest, NaiveStrategy)
{
  CloudInfo cloud_info("naive", input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  EXPECT_EQ(
    concat_info.matching_strategy,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_NAIVE);
}

TEST_F(CloudInfoTest, MultiplePointCloudIndexing)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  // Apply cloud 1
  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Apply cloud 3 (out of order)
  cloud_info.apply_source_with_point_cloud(
    test_cloud_3_, input_topics_[2], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Apply cloud 2
  cloud_info.apply_source_with_point_cloud(
    test_cloud_2_, input_topics_[1], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Cloud 1
  EXPECT_EQ(concat_info.source_info[0].idx_begin, 0u);
  EXPECT_EQ(concat_info.source_info[0].length, test_cloud_1_.width * test_cloud_1_.height);

  // Cloud 3 was supposed to be concatenated second, so its idx_begin should be after cloud 1
  EXPECT_EQ(concat_info.source_info[2].idx_begin, test_cloud_1_.width * test_cloud_1_.height);
  EXPECT_EQ(concat_info.source_info[2].length, test_cloud_3_.width * test_cloud_3_.height);

  // Cloud 2 should have idx_begin after cloud 1
  EXPECT_EQ(
    concat_info.source_info[1].idx_begin,
    test_cloud_1_.width * test_cloud_1_.height + test_cloud_3_.width * test_cloud_3_.height);
  EXPECT_EQ(concat_info.source_info[1].length, test_cloud_2_.width * test_cloud_2_.height);
}

TEST_F(CloudInfoTest, EmptyInputTopics)
{
  std::vector<std::string> empty_topics;
  CloudInfo cloud_info(strategy_name_, empty_topics);
  auto concat_info = cloud_info.reset_and_get_base_info();

  EXPECT_EQ(concat_info.source_info.size(), 0u);
}

TEST_F(CloudInfoTest, UpdateConcatenatedPointCloudResultCheckSuccess)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  // Create concatenated cloud for the result
  sensor_msgs::msg::PointCloud2 concatenated_cloud;
  concatenated_cloud.header.frame_id = "concatenated_frame";
  concatenated_cloud.header.stamp.sec = 1234567892;
  concatenated_cloud.header.stamp.nanosec = 555666777;

  // Add only 2 out of 3 clouds with STATUS_OK
  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  cloud_info.apply_source_with_point_cloud(
    test_cloud_2_, input_topics_[1], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Update result - should not be successful since we only have 2/3 clouds
  cloud_info.update_concatenated_point_cloud_result(concatenated_cloud, concat_info);

  // Check header is updated
  EXPECT_EQ(concat_info.header.frame_id, concatenated_cloud.header.frame_id);
  EXPECT_EQ(concat_info.header.stamp.sec, concatenated_cloud.header.stamp.sec);
  EXPECT_EQ(concat_info.header.stamp.nanosec, concatenated_cloud.header.stamp.nanosec);

  // Check concatenation is not successful (2/3 clouds)
  EXPECT_FALSE(concat_info.concatenation_success);

  // Now add the third cloud
  cloud_info.apply_source_with_point_cloud(
    test_cloud_3_, input_topics_[2], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Update result again - should be successful since we now have all 3/3 clouds
  cloud_info.update_concatenated_point_cloud_result(concatenated_cloud, concat_info);

  // Check concatenation is now successful (3/3 clouds)
  EXPECT_TRUE(concat_info.concatenation_success);
}

TEST_F(CloudInfoTest, UpdateConcatenatedPointCloudResultWithMixedStatus)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  // Add one cloud with STATUS_OK
  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Add one cloud with STATUS_INVALID (should not count towards valid_cloud_count_)
  cloud_info.apply_source_with_point_cloud(
    test_cloud_2_, input_topics_[1],
    autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID, concat_info);

  // Add one cloud with STATUS_TIMEOUT (should not count towards valid_cloud_count_)
  cloud_info.apply_source_with_status(
    input_topics_[2], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT,
    concat_info);

  // Create concatenated cloud for the result
  sensor_msgs::msg::PointCloud2 concatenated_cloud;
  concatenated_cloud.header.frame_id = "concatenated_frame";
  concatenated_cloud.header.stamp.sec = 1234567892;
  concatenated_cloud.header.stamp.nanosec = 555666777;

  // Update result - should not be successful since we only have 1/3 valid clouds
  cloud_info.update_concatenated_point_cloud_result(concatenated_cloud, concat_info);

  // Check concatenation is not successful (1/3 valid clouds)
  EXPECT_FALSE(concat_info.concatenation_success);
}

TEST_F(CloudInfoTest, ApplySourceTwiceThrows)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.reset_and_get_base_info();

  // Apply first cloud successfully
  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Try to apply second cloud to same topic - should throw
  EXPECT_THROW(
    cloud_info.apply_source_with_point_cloud(
      test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
      concat_info),
    std::runtime_error);
}

TEST_F(CloudInfoTest, TwoIterationsOfSettingCloudInfo)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);

  // First iteration - apply only partial clouds (invalid result)
  auto concat_info_1 = cloud_info.reset_and_get_base_info();

  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info_1);

  cloud_info.apply_source_with_point_cloud(
    test_cloud_2_, input_topics_[1],
    autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID, concat_info_1);

  cloud_info.apply_source_with_status(
    input_topics_[2], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT,
    concat_info_1);

  // Create concatenated cloud for first iteration
  sensor_msgs::msg::PointCloud2 concatenated_cloud_1;
  concatenated_cloud_1.header.frame_id = "concatenated_frame_1";
  concatenated_cloud_1.header.stamp.sec = 1234567892;
  concatenated_cloud_1.header.stamp.nanosec = 111222333;

  cloud_info.update_concatenated_point_cloud_result(concatenated_cloud_1, concat_info_1);

  // Check first iteration results - should be invalid (only 1/3 valid clouds)
  EXPECT_EQ(concat_info_1.header.frame_id, concatenated_cloud_1.header.frame_id);
  EXPECT_EQ(concat_info_1.header.stamp.sec, concatenated_cloud_1.header.stamp.sec);
  EXPECT_EQ(concat_info_1.header.stamp.nanosec, concatenated_cloud_1.header.stamp.nanosec);
  EXPECT_FALSE(concat_info_1.concatenation_success);

  // Verify indexing for first iteration
  EXPECT_EQ(concat_info_1.source_info[0].idx_begin, 0u);
  EXPECT_EQ(concat_info_1.source_info[0].length, test_cloud_1_.width * test_cloud_1_.height);
  EXPECT_EQ(concat_info_1.source_info[1].idx_begin, 0u);  // Should remain 0 due to INVALID status
  EXPECT_EQ(concat_info_1.source_info[1].length, 0u);     // Should remain 0 due to INVALID status

  // Second iteration - fresh start with new concat_info (valid result)
  auto concat_info_2 = cloud_info.reset_and_get_base_info();

  cloud_info.apply_source_with_point_cloud(
    test_cloud_1_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info_2);

  cloud_info.apply_source_with_point_cloud(
    test_cloud_2_, input_topics_[1], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info_2);

  cloud_info.apply_source_with_point_cloud(
    test_cloud_3_, input_topics_[2], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info_2);

  // Create concatenated cloud for second iteration
  sensor_msgs::msg::PointCloud2 concatenated_cloud_2;
  concatenated_cloud_2.header.frame_id = "concatenated_frame_2";
  concatenated_cloud_2.header.stamp.sec = 1234567893;
  concatenated_cloud_2.header.stamp.nanosec = 444555666;

  cloud_info.update_concatenated_point_cloud_result(concatenated_cloud_2, concat_info_2);

  // Check second iteration results - should be valid (all 3/3 clouds)
  EXPECT_EQ(concat_info_2.header.frame_id, concatenated_cloud_2.header.frame_id);
  EXPECT_EQ(concat_info_2.header.stamp.sec, concatenated_cloud_2.header.stamp.sec);
  EXPECT_EQ(concat_info_2.header.stamp.nanosec, concatenated_cloud_2.header.stamp.nanosec);
  EXPECT_TRUE(concat_info_2.concatenation_success);

  // Verify that the first iteration results are unchanged
  EXPECT_EQ(concat_info_1.header.frame_id, concatenated_cloud_1.header.frame_id);
  EXPECT_FALSE(concat_info_1.concatenation_success);

  // Verify indexing for second iteration
  EXPECT_EQ(concat_info_2.source_info[0].idx_begin, 0u);
  EXPECT_EQ(concat_info_2.source_info[0].length, test_cloud_1_.width * test_cloud_1_.height);
  EXPECT_EQ(concat_info_2.source_info[1].idx_begin, test_cloud_1_.width * test_cloud_1_.height);
  EXPECT_EQ(concat_info_2.source_info[1].length, test_cloud_2_.width * test_cloud_2_.height);
}
