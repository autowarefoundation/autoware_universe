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

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
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
    };

    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());

    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return options;
  }
};

TEST_F(LabelClusterConfigBehavior, AcceptsLowercaseConfiguredLabels)
{
  // Arrange
  const auto options = make_node_options({
    rclcpp::Parameter("class_names.car", std::string("car")),
    rclcpp::Parameter("class_names.truck", std::string("truck")),
    rclcpp::Parameter("class_names.tractor_unit", std::string("trailer")),
    rclcpp::Parameter("class_names.pedestrian", std::string("pedestrian")),
    rclcpp::Parameter("label_cluster_params.pedestrian.tolerance_m", 0.3),
    rclcpp::Parameter(
      "confusable_label_groups.truck_trailer.labels", std::vector<std::string>{"truck", "trailer"}),
    rclcpp::Parameter("confusable_label_groups.truck_trailer.cross_label_tolerance_m", 0.5),
    rclcpp::Parameter("confusable_label_groups.truck_trailer.max_merged_size_m", 25.0),
  });

  // Act
  auto node = std::make_unique<LabelBasedEuclideanClusterNode>(options);

  // Assert: constructor accepts lowercase labels and dynamic nested overrides are declared.
  EXPECT_TRUE(node->has_parameter("class_names.car"));
  EXPECT_TRUE(node->has_parameter("class_names.truck"));
  EXPECT_TRUE(node->has_parameter("class_names.tractor_unit"));
  EXPECT_TRUE(node->has_parameter("class_names.pedestrian"));
  EXPECT_TRUE(node->has_parameter("label_cluster_params.pedestrian.tolerance_m"));
  EXPECT_TRUE(node->has_parameter("confusable_label_groups.truck_trailer.labels"));
  EXPECT_TRUE(node->has_parameter("confusable_label_groups.truck_trailer.cross_label_tolerance_m"));
  EXPECT_TRUE(node->has_parameter("confusable_label_groups.truck_trailer.max_merged_size_m"));
}

TEST_F(LabelClusterConfigBehavior, CreatesExecuterForDynamicOverrideLabel)
{
  // Arrange
  const auto options = make_node_options({
    rclcpp::Parameter("class_names.car", std::string("car")),
    rclcpp::Parameter("class_names.tractor_unit", std::string("trailer")),
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

}  // namespace autoware::euclidean_cluster
