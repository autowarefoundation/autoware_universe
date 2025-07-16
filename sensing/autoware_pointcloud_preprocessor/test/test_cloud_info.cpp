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

using namespace autoware::pointcloud_preprocessor;

class CloudInfoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup test data
    input_topics_ = {"/topic1", "/topic2", "/topic3"};
    strategy_name_ = "advanced";

    // Create test point cloud
    test_cloud_.header.frame_id = "base_link";
    test_cloud_.header.stamp.sec = 1234567890;
    test_cloud_.header.stamp.nanosec = 123456789;
    test_cloud_.width = 100;
    test_cloud_.height = 50;
    test_cloud_.data.resize(test_cloud_.width * test_cloud_.height * 16);  // Mock data

    // Create test header
    test_header_.frame_id = "sensor_frame";
    test_header_.stamp.sec = 1234567891;
    test_header_.stamp.nanosec = 987654321;
  }

  std::vector<std::string> input_topics_;
  std::string strategy_name_;
  sensor_msgs::msg::PointCloud2 test_cloud_;
  std_msgs::msg::Header test_header_;
};

TEST_F(CloudInfoTest, ConstructorAndGetConcatInfoBase)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);

  auto concat_info = cloud_info.get_concat_info_base();

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

TEST_F(CloudInfoTest, ApplySourceWithPointCloud)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  // Apply point cloud to first topic
  CloudInfo::apply_source_with_point_cloud(
    test_cloud_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Check that the first topic was updated correctly
  auto & first_source = concat_info.source_info[0];
  EXPECT_EQ(first_source.topic, input_topics_[0]);
  EXPECT_EQ(first_source.status, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK);
  EXPECT_EQ(first_source.header.frame_id, test_cloud_.header.frame_id);
  EXPECT_EQ(first_source.header.stamp.sec, test_cloud_.header.stamp.sec);
  EXPECT_EQ(first_source.header.stamp.nanosec, test_cloud_.header.stamp.nanosec);
  EXPECT_EQ(first_source.idx_begin, 0u);
  EXPECT_EQ(first_source.length, test_cloud_.width * test_cloud_.height);

  // Apply second point cloud
  sensor_msgs::msg::PointCloud2 second_cloud = test_cloud_;
  second_cloud.width = 200;
  second_cloud.height = 75;

  CloudInfo::apply_source_with_point_cloud(
    second_cloud, input_topics_[1], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Check that idx_begin is calculated correctly for second topic
  auto & second_source = concat_info.source_info[1];
  EXPECT_EQ(second_source.idx_begin, test_cloud_.width * test_cloud_.height);
  EXPECT_EQ(second_source.length, second_cloud.width * second_cloud.height);
}

TEST_F(CloudInfoTest, ApplySourceWithPointCloudNonOkStatus)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  // Apply point cloud with ERROR status
  CloudInfo::apply_source_with_point_cloud(
    test_cloud_, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID,
    concat_info);

  // Check that header and status are updated, but idx_begin and length are not
  auto & first_source = concat_info.source_info[0];
  EXPECT_EQ(first_source.status, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID);
  EXPECT_EQ(first_source.header.frame_id, test_cloud_.header.frame_id);
  EXPECT_EQ(first_source.idx_begin, 0u);  // Should remain 0
  EXPECT_EQ(first_source.length, 0u);     // Should remain 0
}

TEST_F(CloudInfoTest, ApplySourceWithHeader)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  CloudInfo::apply_source_with_header(
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
  auto concat_info = cloud_info.get_concat_info_base();

  CloudInfo::apply_source_with_status(
    input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT,
    concat_info);

  auto & first_source = concat_info.source_info[0];
  EXPECT_EQ(first_source.status, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_TIMEOUT);
  EXPECT_EQ(first_source.topic, input_topics_[0]);
}

TEST_F(CloudInfoTest, UpdateConcatenatedPointCloudHeader)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  CloudInfo::update_concatenated_point_cloud_header(test_cloud_, concat_info);

  EXPECT_EQ(concat_info.header.frame_id, test_cloud_.header.frame_id);
  EXPECT_EQ(concat_info.header.stamp.sec, test_cloud_.header.stamp.sec);
  EXPECT_EQ(concat_info.header.stamp.nanosec, test_cloud_.header.stamp.nanosec);
}

TEST_F(CloudInfoTest, UpdateConcatenatedPointCloudConfig)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

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

TEST_F(CloudInfoTest, UpdateConcatenatedPointCloudSuccess)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  CloudInfo::update_concatenated_point_cloud_success(true, concat_info);
  EXPECT_TRUE(concat_info.concatenation_success);

  CloudInfo::update_concatenated_point_cloud_success(false, concat_info);
  EXPECT_FALSE(concat_info.concatenation_success);
}

TEST_F(CloudInfoTest, ApplySourceWithNonExistentTopic)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  EXPECT_THROW(
    CloudInfo::apply_source_with_point_cloud(
      test_cloud_, "/non_existent_topic",
      autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK, concat_info),
    std::runtime_error);
}

TEST_F(CloudInfoTest, NaiveStrategy)
{
  CloudInfo cloud_info("naive", input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  EXPECT_EQ(
    concat_info.matching_strategy,
    autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_NAIVE);
}

TEST_F(CloudInfoTest, MultiplePointCloudIndexing)
{
  CloudInfo cloud_info(strategy_name_, input_topics_);
  auto concat_info = cloud_info.get_concat_info_base();

  // Apply first cloud
  sensor_msgs::msg::PointCloud2 cloud1 = test_cloud_;
  cloud1.width = 100;
  cloud1.height = 1;
  CloudInfo::apply_source_with_point_cloud(
    cloud1, input_topics_[0], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Apply second cloud
  sensor_msgs::msg::PointCloud2 cloud2 = test_cloud_;
  cloud2.width = 200;
  cloud2.height = 1;
  CloudInfo::apply_source_with_point_cloud(
    cloud2, input_topics_[1], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Apply third cloud
  sensor_msgs::msg::PointCloud2 cloud3 = test_cloud_;
  cloud3.width = 150;
  cloud3.height = 1;
  CloudInfo::apply_source_with_point_cloud(
    cloud3, input_topics_[2], autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
    concat_info);

  // Check indexing
  EXPECT_EQ(concat_info.source_info[0].idx_begin, 0u);
  EXPECT_EQ(concat_info.source_info[0].length, 100u);

  EXPECT_EQ(concat_info.source_info[1].idx_begin, 100u);
  EXPECT_EQ(concat_info.source_info[1].length, 200u);

  EXPECT_EQ(concat_info.source_info[2].idx_begin, 300u);
  EXPECT_EQ(concat_info.source_info[2].length, 150u);
}

TEST_F(CloudInfoTest, EmptyInputTopics)
{
  std::vector<std::string> empty_topics;
  CloudInfo cloud_info(strategy_name_, empty_topics);
  auto concat_info = cloud_info.get_concat_info_base();

  EXPECT_EQ(concat_info.source_info.size(), 0u);
}
