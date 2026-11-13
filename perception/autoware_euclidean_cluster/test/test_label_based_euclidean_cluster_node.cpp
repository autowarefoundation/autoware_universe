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

#include "../src/label_based_euclidean_cluster_node.hpp"

#include <autoware/object_recognition_utils/pointcloud_classification.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
using autoware::object_recognition_utils::PointCloudClassification;
using autoware::point_types::PointXYZCPE;

class LabelClusterConfigBehavior : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static rclcpp::NodeOptions make_node_options(
    const std::vector<rclcpp::Parameter> & extra_parameters)
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("min_probability", 0.3),
      rclcpp::Parameter("shape_policy", static_cast<int64_t>(0)),
      rclcpp::Parameter("use_height", false),
      rclcpp::Parameter("tolerance_m", 0.65),
      rclcpp::Parameter("voxel_leaf_size_m", 0.2),
      rclcpp::Parameter("min_points_per_voxel", static_cast<int64_t>(3)),
      rclcpp::Parameter("min_points_per_cluster", static_cast<int64_t>(10)),
      rclcpp::Parameter("large_cluster_voxel_count_threshold", static_cast<int64_t>(50)),
      rclcpp::Parameter("large_cluster_max_points_per_voxel", static_cast<int64_t>(100)),
      rclcpp::Parameter("max_voxels_per_cluster", static_cast<int64_t>(2000)),
      rclcpp::Parameter("use_shape_estimation_corrector", false),
      rclcpp::Parameter("use_shape_estimation_filter", false),
      rclcpp::Parameter("use_boost_bbox_optimizer", false),
      rclcpp::Parameter("min_cluster_size_m", 0.0)};

    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());

    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return options;
  }

  /// @brief Create a PointXYZCPE cloud, the point type the node expects on its input topic.
  static sensor_msgs::msg::PointCloud2 make_semantic_pointcloud(
    const std::size_t point_count, const std::uint8_t class_id, const float probability)
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(point_count);
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.fields = autoware::point_types::create_fields_point_xyzcpe();
    msg.point_step = sizeof(PointXYZCPE);
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);

    for (std::size_t i = 0; i < point_count; ++i) {
      // entropy keeps its quiet NaN default, as published for points without an entropy value.
      PointXYZCPE point;
      point.x = static_cast<float>(i) * 0.05F;
      point.y = 0.0F;
      point.z = 0.0F;
      point.class_id = class_id;
      point.probability = probability;
      std::memcpy(&msg.data[i * msg.point_step], &point, sizeof(PointXYZCPE));
    }

    return msg;
  }
};

TEST_F(LabelClusterConfigBehavior, AcceptsLowercaseConfiguredLabelOverrides)
{
  // Arrange
  const auto options = make_node_options({
    rclcpp::Parameter("label_cluster_params.pedestrian.tolerance_m", 0.3),
    rclcpp::Parameter(
      "confusable_label_groups.truck_trailer.labels", std::vector<std::string>{"truck", "trailer"}),
    rclcpp::Parameter("confusable_label_groups.truck_trailer.cross_label_tolerance_m", 0.5),
    rclcpp::Parameter("confusable_label_groups.truck_trailer.max_merged_size_m", 25.0),
  });

  // Act
  auto node = std::make_unique<LabelBasedEuclideanClusterNode>(options);

  // Assert: constructor accepts lowercase labels and dynamic nested overrides are declared.
  EXPECT_TRUE(node->has_parameter("label_cluster_params.pedestrian.tolerance_m"));
  EXPECT_TRUE(node->has_parameter("confusable_label_groups.truck_trailer.labels"));
  EXPECT_TRUE(node->has_parameter("confusable_label_groups.truck_trailer.cross_label_tolerance_m"));
  EXPECT_TRUE(node->has_parameter("confusable_label_groups.truck_trailer.max_merged_size_m"));
}

TEST_F(LabelClusterConfigBehavior, CreatesExecuterForDynamicOverrideLabel)
{
  // Arrange
  const auto options = make_node_options({
    rclcpp::Parameter("label_cluster_params.trailer.tolerance_m", 0.4),
    rclcpp::Parameter(
      "label_cluster_params.trailer.min_points_per_cluster", static_cast<int64_t>(7)),
  });

  // Act
  auto node = std::make_unique<LabelBasedEuclideanClusterNode>(options);

  // Assert
  EXPECT_TRUE(node->has_parameter("label_cluster_params.trailer.tolerance_m"));
  EXPECT_TRUE(node->has_parameter("label_cluster_params.trailer.min_points_per_cluster"));
}

TEST_F(LabelClusterConfigBehavior, AcceptsOptionsWithoutClassNames)
{
  const auto options = make_node_options({});

  EXPECT_NO_THROW({
    auto node = std::make_unique<LabelBasedEuclideanClusterNode>(options);
    static_cast<void>(node);
  });
}

TEST_F(LabelClusterConfigBehavior, ThrowsWhenShapePolicyIsInvalid)
{
  const auto options = make_node_options({
    rclcpp::Parameter("shape_policy", static_cast<int64_t>(2)),
  });

  EXPECT_THROW(
    {
      auto node = std::make_unique<LabelBasedEuclideanClusterNode>(options);
      static_cast<void>(node);
    },
    std::runtime_error);
}

TEST_F(LabelClusterConfigBehavior, ThrowsWhenConfusableGroupMissingCrossLabelTolerance)
{
  const auto options = make_node_options({
    rclcpp::Parameter(
      "confusable_label_groups.truck_trailer.labels", std::vector<std::string>{"truck", "trailer"}),
    rclcpp::Parameter("confusable_label_groups.truck_trailer.max_merged_size_m", 25.0),
  });

  EXPECT_THROW(
    {
      auto node = std::make_unique<LabelBasedEuclideanClusterNode>(options);
      static_cast<void>(node);
    },
    std::runtime_error);
}

TEST_F(LabelClusterConfigBehavior, ThrowsWhenConfusableGroupMissingMaxMergedSize)
{
  const auto options = make_node_options({
    rclcpp::Parameter(
      "confusable_label_groups.truck_trailer.labels", std::vector<std::string>{"truck", "trailer"}),
    rclcpp::Parameter("confusable_label_groups.truck_trailer.cross_label_tolerance_m", 0.5),
  });

  EXPECT_THROW(
    {
      auto node = std::make_unique<LabelBasedEuclideanClusterNode>(options);
      static_cast<void>(node);
    },
    std::runtime_error);
}

TEST_F(LabelClusterConfigBehavior, PublishesDetectedObjectsForSemanticInput)
{
  const auto options = make_node_options({});

  auto cluster_node = std::make_shared<LabelBasedEuclideanClusterNode>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("label_cluster_integration_helper");

  std::mutex mutex;
  std::condition_variable cv;
  bool received = false;
  autoware_perception_msgs::msg::DetectedObjects::SharedPtr output_msg;

  auto output_sub =
    helper_node->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
      "/output/objects", rclcpp::QoS{1},
      [&](autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        output_msg = std::move(msg);
        received = true;
        cv.notify_one();
      });

  auto input_pub = helper_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/input/pointcloud", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(cluster_node->get_node_base_interface());
  executor.add_node(helper_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  // Wait until publisher/subscriber are matched before publishing test data.
  const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while ((input_pub->get_subscription_count() == 0 || output_sub->get_publisher_count() == 0) &&
         std::chrono::steady_clock::now() < wait_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto input_msg = make_semantic_pointcloud(12, 0U, 0.95F);
  input_pub->publish(input_msg);

  {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::seconds(2), [&]() { return received; });
  }

  executor.cancel();
  spin_thread.join();

  ASSERT_TRUE(received);
  ASSERT_NE(output_msg, nullptr);
  EXPECT_EQ(output_msg->header.frame_id, "map");
  EXPECT_FALSE(output_msg->objects.empty());
}

TEST_F(LabelClusterConfigBehavior, PublishesSemanticNonObjectsAsSegments)
{
  const auto options = make_node_options({});

  auto cluster_node = std::make_shared<LabelBasedEuclideanClusterNode>(options);
  auto helper_node = std::make_shared<rclcpp::Node>("label_cluster_segments_helper");

  std::mutex mutex;
  std::condition_variable cv;
  bool received = false;
  sensor_msgs::msg::PointCloud2::SharedPtr output_msg;

  auto output_sub = helper_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/output/pointcloud", rclcpp::QoS{1}, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      output_msg = std::move(msg);
      received = true;
      cv.notify_one();
    });

  auto input_pub = helper_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/input/pointcloud", rclcpp::SensorDataQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(cluster_node->get_node_base_interface());
  executor.add_node(helper_node);

  std::thread spin_thread([&executor]() { executor.spin(); });

  const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while ((input_pub->get_subscription_count() == 0 || output_sub->get_publisher_count() == 0) &&
         std::chrono::steady_clock::now() < wait_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto input_msg = make_semantic_pointcloud(
    4, static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE), 0.95F);
  input_pub->publish(input_msg);

  {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::seconds(2), [&]() { return received; });
  }

  executor.cancel();
  spin_thread.join();

  ASSERT_TRUE(received);
  ASSERT_NE(output_msg, nullptr);
  EXPECT_EQ(output_msg->header.frame_id, "map");
  EXPECT_EQ(output_msg->width, 4U);
}

}  // namespace autoware::euclidean_cluster
