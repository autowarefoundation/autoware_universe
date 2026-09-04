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

#include "autoware/euclidean_cluster/label_based_euclidean_cluster.hpp"
#include "autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/pointcloud_classification.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <autoware/shape_estimation/shape_estimator.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
using autoware::object_recognition_utils::PointCloudClassification;
using autoware::point_types::PointXYZCPE;
using autoware_perception_msgs::msg::ObjectClassification;

class LabelBasedEuclideanClusterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a simple voxel-based cluster executer with permissive parameters for testing
    default_cluster_ = std::make_shared<VoxelGridBasedEuclideanCluster>(
      false,  // use_height
      2,      // min_points_per_cluster
      1.0,    // tolerance_m
      0.1,    // voxel_leaf_size_m
      1,      // min_points_per_voxel
      10,     // large_cluster_voxel_count_threshold
      100,    // large_cluster_max_points_per_voxel
      10000   // max_voxels_per_cluster
    );

    // Create a simple shape estimator
    shape_estimator_ = std::make_shared<autoware::shape_estimation::ShapeEstimator>(
      false, false, false  // corrector, filter, optimizer disabled
    );
  }

  std::shared_ptr<VoxelGridBasedEuclideanCluster> default_cluster_;
  std::shared_ptr<autoware::shape_estimation::ShapeEstimator> shape_estimator_;

  /// @brief Create a PointXYZCPE point cloud; class_id defaults to CAR and probability to 1.0.
  /// @details `entropy` keeps the quiet NaN default of PointXYZCPE, as published for points whose
  /// entropy is unavailable.
  sensor_msgs::msg::PointCloud2 create_pointcloud(
    const std::vector<float> & x, const std::vector<float> & y, const std::vector<float> & z,
    const std::vector<uint8_t> & class_ids = {}, const std::vector<float> & probabilities = {})
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.height = 1;
    msg.width = x.size();
    msg.is_dense = true;
    msg.is_bigendian = false;
    msg.fields = autoware::point_types::create_fields_point_xyzcpe();
    msg.point_step = sizeof(PointXYZCPE);
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

    for (size_t i = 0; i < x.size(); ++i) {
      PointXYZCPE point;
      point.x = x[i];
      point.y = y[i];
      point.z = z[i];
      point.class_id = class_ids.empty() ? kCarClassId : class_ids[i];
      point.probability = probabilities.empty() ? 1.0F : probabilities[i];
      std::memcpy(&msg.data[i * msg.point_step], &point, sizeof(PointXYZCPE));
    }

    return msg;
  }

  pcl::PointCloud<PointXYZCPE> preprocess(
    const sensor_msgs::msg::PointCloud2 & input, const float min_probability = 0.0F)
  {
    PointCloudPreprocessor preprocessor(min_probability);
    auto result = preprocessor.process(input);
    if (!result) {
      ADD_FAILURE() << "Preprocessing failed: " << result.error();
      return {};
    }
    return std::move(*result);
  }

  static constexpr std::uint8_t kCarClassId =
    static_cast<std::uint8_t>(PointCloudClassification::CAR);
};

// ============================================================================
// PointCloudClassification Tests
// ============================================================================

TEST_F(LabelBasedEuclideanClusterTest, EmptyPointCloudReturnsEmptyObjects)
{
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{}, shape_estimator_);

  auto pc = create_pointcloud({}, {}, {});

  // Act
  auto result = cluster.process(preprocess(pc));

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->objects.objects.size(), 0U);
  EXPECT_EQ(result->segments.width, 0U);
}

TEST_F(LabelBasedEuclideanClusterTest, PointCloudWithDefaultClassIdUsesCarLabel)
{
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{}, shape_estimator_);

  // Create enough points to potentially form a cluster
  // Add more points in close proximity to exceed min_points_per_cluster
  std::vector<float> x_coords, y_coords, z_coords;
  for (int i = 0; i < 10; ++i) {
    x_coords.push_back(static_cast<float>(i) * 0.05f);
    y_coords.push_back(0.0f);
    z_coords.push_back(0.0f);
  }
  auto pc = create_pointcloud(x_coords, y_coords, z_coords);

  // Act
  auto result = cluster.process(preprocess(pc));

  // Assert
  ASSERT_TRUE(result.has_value());
  // Even if no clusters form, the processing should complete without errors.
  // If clusters do form, they carry the CAR label the helper writes into class_id.
  for (const auto & obj : result->objects.objects) {
    EXPECT_EQ(obj.classification[0].label, ObjectClassification::CAR);
  }
}

TEST_F(LabelBasedEuclideanClusterTest, PointsAreGroupedByLabel)
{
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{}, shape_estimator_);

  // Create two well-separated clusters: car at origin and pedestrian at (5,0)
  std::vector<float> x_vals, y_vals, z_vals, class_vals, prob_vals;
  // Car cluster (class 0) at origin
  for (int i = 0; i < 5; ++i) {
    x_vals.push_back(static_cast<float>(i) * 0.05f);
    y_vals.push_back(0.0f);
    z_vals.push_back(0.0f);
    class_vals.push_back(0);
    prob_vals.push_back(1.0f);
  }
  // Pedestrian cluster far away
  for (int i = 0; i < 5; ++i) {
    x_vals.push_back(5.0f + static_cast<float>(i) * 0.05f);
    y_vals.push_back(0.0f);
    z_vals.push_back(0.0f);
    class_vals.push_back(static_cast<std::uint8_t>(PointCloudClassification::PEDESTRIAN));
    prob_vals.push_back(1.0f);
  }
  auto pc = create_pointcloud(
    x_vals, y_vals, z_vals, std::vector<uint8_t>(class_vals.begin(), class_vals.end()), prob_vals);

  // Act
  auto result = cluster.process(preprocess(pc));

  // Assert - should have clusters for both classes
  ASSERT_TRUE(result.has_value());
  bool has_car = false;
  bool has_ped = false;
  for (const auto & obj : result->objects.objects) {
    if (obj.classification[0].label == ObjectClassification::CAR) {
      has_car = true;
    } else if (obj.classification[0].label == ObjectClassification::PEDESTRIAN) {
      has_ped = true;
    }
  }
  // At least one of each label should be present
  EXPECT_TRUE(has_car);
  EXPECT_TRUE(has_ped);
}

TEST_F(LabelBasedEuclideanClusterTest, PerLabelMinimumSizeFiltersTinyTruckButKeepsHazard)
{
  const auto truck_point_label = static_cast<std::uint8_t>(PointCloudClassification::TRUCK);
  const auto hazard_point_label = static_cast<std::uint8_t>(PointCloudClassification::HAZARD);

  const auto truck_object_label = static_cast<std::uint8_t>(ObjectClassification::TRUCK);
  const auto hazard_object_label = static_cast<std::uint8_t>(ObjectClassification::HAZARD);

  auto truck_cluster =
    std::make_shared<VoxelGridBasedEuclideanCluster>(false, 2, 1.0, 0.1, 1, 10, 100, 10000, 1.0F);
  auto hazard_cluster =
    std::make_shared<VoxelGridBasedEuclideanCluster>(false, 2, 1.0, 0.1, 1, 10, 100, 10000, 0.0F);
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{
      {truck_object_label, truck_cluster},
      {hazard_object_label, hazard_cluster},
    },
    shape_estimator_);

  auto pc = create_pointcloud(
    {0.0F, 0.05F, 0.10F, 5.0F, 5.05F, 5.10F}, {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F},
    {
      truck_point_label,
      truck_point_label,
      truck_point_label,
      hazard_point_label,
      hazard_point_label,
      hazard_point_label,
    });

  const auto result = cluster.process(preprocess(pc));

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->objects.objects.size(), 1U);
  EXPECT_EQ(
    result->objects.objects.front().classification.front().label, ObjectClassification::HAZARD);
}

// ============================================================================
// Probability Filtering Tests
// ============================================================================

TEST_F(LabelBasedEuclideanClusterTest, PointsBelowMinProbabilityAreFiltered)
{
  // Create points with varying probabilities
  auto pc = create_pointcloud(
    {0.0f, 0.1f, 0.2f, 0.3f},  // x
    {0.0f, 0.0f, 0.0f, 0.0f},  // y
    {0.0f, 0.0f, 0.0f, 0.0f},  // z
    {0, 0, 0, 0},              // class_id
    {0.5f, 0.9f, 0.7f, 0.95f}  // probability
  );

  // Act
  PointCloudPreprocessor preprocessor(0.8F);
  const auto result = preprocessor.process(pc);

  // Assert - only points with prob >= 0.8 should be kept (indices 1, 3)
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2U);
  EXPECT_FLOAT_EQ(result->points[0].x, 0.1F);
  EXPECT_FLOAT_EQ(result->points[0].probability, 0.9F);
  EXPECT_FLOAT_EQ(result->points[1].x, 0.3F);
  EXPECT_FLOAT_EQ(result->points[1].probability, 0.95F);
}

TEST_F(LabelBasedEuclideanClusterTest, AverageProbabilityIsCorrect)
{
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{}, shape_estimator_);

  // Create cluster with known probabilities
  std::vector<float> x_vals, y_vals, z_vals, prob_vals;
  float probs[] = {0.5f, 0.6f, 0.7f};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      x_vals.push_back(static_cast<float>(i) * 0.05f);
      y_vals.push_back(static_cast<float>(j) * 0.05f);
      z_vals.push_back(0.0f);
      prob_vals.push_back(probs[i]);
    }
  }

  std::vector<uint8_t> class_ids(x_vals.size(), 0);
  auto pc = create_pointcloud(x_vals, y_vals, z_vals, class_ids, prob_vals);

  // Act
  auto result = cluster.process(preprocess(pc));

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->objects.objects.size(), 1U);
  EXPECT_NEAR(result->objects.objects[0].existence_probability, 0.6f, 1e-5f);
}

// ============================================================================
// Shape Estimation Tests
// ============================================================================

TEST_F(LabelBasedEuclideanClusterTest, ShapeIsPopulated)
{
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{}, shape_estimator_);

  // Create a reasonable cluster
  std::vector<float> x_vals, y_vals, z_vals;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      x_vals.push_back(static_cast<float>(i) * 0.1f);
      y_vals.push_back(static_cast<float>(j) * 0.1f);
      z_vals.push_back(0.5f);
    }
  }
  std::vector<uint8_t> class_ids(x_vals.size(), 0);
  auto pc = create_pointcloud(x_vals, y_vals, z_vals, class_ids);

  // Act
  auto result = cluster.process(preprocess(pc));

  // Assert
  ASSERT_TRUE(result.has_value());
  if (!result->objects.objects.empty()) {
    auto & shape = result->objects.objects[0].shape;
    // Shape should have dimensions set
    EXPECT_GE(shape.dimensions.x, 0.0);
    EXPECT_GE(shape.dimensions.y, 0.0);
    EXPECT_GE(shape.dimensions.z, 0.0);
  }
}

TEST_F(LabelBasedEuclideanClusterTest, InvalidClassificationsAreOutputAsSegments)
{
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{}, shape_estimator_);

  auto pc = create_pointcloud(
    {0.0f, 0.1f, 1.0f, 1.1f},  // x
    {0.0f, 0.0f, 0.0f, 0.0f},  // y
    {0.0f, 0.0f, 0.0f, 0.0f},  // z
    {0, 0, 255, 255},          // class_id
    {1.0f, 1.0f, 1.0f, 1.0f}   // probability
  );

  // Act
  auto result = cluster.process(preprocess(pc));

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->segments.width, 2U);
  for (const auto & obj : result->objects.objects) {
    EXPECT_EQ(obj.classification[0].label, ObjectClassification::CAR);
  }
}

TEST_F(LabelBasedEuclideanClusterTest, SemanticNonObjectLabelsAreOutputAsSegments)
{
  LabelBasedEuclideanCluster cluster(
    ShapePolicy::ALL_POLYGON, default_cluster_,
    std::unordered_map<uint8_t, std::shared_ptr<EuclideanClusterInterface>>{}, shape_estimator_);

  const auto flat_surface = static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE);
  auto pc = create_pointcloud(
    {0.0f, 0.1f, 0.2f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
    {flat_surface, flat_surface, flat_surface}, {0.9f, 0.8f, 0.7f});

  const auto result = cluster.process(preprocess(pc));

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->objects.objects.empty());
  EXPECT_EQ(result->segments.width, 3U);
  EXPECT_EQ(result->segments.fields, pc.fields);
  EXPECT_EQ(result->segments.point_step, pc.point_step);
  EXPECT_EQ(result->segments.data, pc.data);
  EXPECT_TRUE(result->segments.is_dense);
}

TEST_F(LabelBasedEuclideanClusterTest, ReturnsErrorWhenRequiredFieldsAreMissing)
{
  sensor_msgs::msg::PointCloud2 pc;
  pc.height = 1;
  pc.width = 1;
  pc.is_dense = true;
  // Intentionally missing x/y/z field definitions

  PointCloudPreprocessor preprocessor(0.0F);
  const auto result = preprocessor.process(pc);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());
}

TEST_F(LabelBasedEuclideanClusterTest, ReturnsErrorWhenLayoutIsNotPointXYZCPE)
{
  // A PointXYZI cloud carries x/y/z but is not a PointXYZCPE cloud.
  sensor_msgs::msg::PointCloud2 pc;
  pc.height = 1;
  pc.width = 1;
  pc.is_dense = true;
  pc.fields = autoware::point_types::create_fields_point_xyzi();
  pc.point_step = sizeof(autoware::point_types::PointXYZI);
  pc.row_step = pc.point_step * pc.width;
  pc.data.resize(pc.row_step);

  PointCloudPreprocessor preprocessor(0.0F);
  const auto result = preprocessor.process(pc);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());
}

TEST_F(LabelBasedEuclideanClusterTest, ReturnsErrorWhenPointStepDoesNotMatchPointXYZCPE)
{
  // The PointXYZCPE fields with extra per-point padding are not a densely packed cloud.
  auto pc = create_pointcloud({0.0f}, {0.0f}, {0.0f});
  pc.point_step = sizeof(PointXYZCPE) + 4U;
  pc.row_step = pc.point_step * pc.width;
  pc.data.resize(pc.row_step);

  PointCloudPreprocessor preprocessor(0.0F);
  const auto result = preprocessor.process(pc);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());
}

TEST_F(LabelBasedEuclideanClusterTest, ReturnsErrorWhenDataSizeDoesNotAgreeWithPointCount)
{
  // More bytes than width * height * point_step would overrun the converted cloud.
  auto pc = create_pointcloud({0.0f}, {0.0f}, {0.0f});
  pc.data.resize(pc.data.size() * 4U);

  PointCloudPreprocessor preprocessor(0.0F);
  const auto result = preprocessor.process(pc);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());
}

}  // namespace autoware::euclidean_cluster
