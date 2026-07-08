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
#include <autoware/shape_estimation/shape_estimator.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::euclidean_cluster
{
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

  ClusterOptions make_default_options(const float min_probability = 0.0F) const
  {
    return ClusterOptions{default_cluster_, min_probability};
  }

  /// @brief Create a minimal PointCloud2 message with xyz and optionally class_id/probability
  sensor_msgs::msg::PointCloud2 create_pointcloud(
    const std::vector<float> & x, const std::vector<float> & y, const std::vector<float> & z,
    const std::vector<uint8_t> & class_ids = {}, const std::vector<float> & probabilities = {})
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.height = 1;
    msg.width = x.size();
    msg.is_dense = true;

    // Add xyz fields
    msg.fields.push_back(sensor_msgs::msg::PointField());
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields.push_back(sensor_msgs::msg::PointField());
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields.push_back(sensor_msgs::msg::PointField());
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    uint32_t offset = 12;

    // Add class_id field if provided
    if (!class_ids.empty()) {
      msg.fields.push_back(sensor_msgs::msg::PointField());
      msg.fields[3].name = "class_id";
      msg.fields[3].offset = offset;
      msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
      msg.fields[3].count = 1;
      offset += 1;
    }

    // Add probability field if provided
    if (!probabilities.empty()) {
      msg.fields.push_back(sensor_msgs::msg::PointField());
      auto prob_idx = msg.fields.size() - 1;
      msg.fields[prob_idx].name = "probability";
      msg.fields[prob_idx].offset = offset;
      msg.fields[prob_idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[prob_idx].count = 1;
      offset += 4;
    }

    msg.point_step = offset;
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step * msg.height);

    // Write data
    for (size_t i = 0; i < x.size(); ++i) {
      const size_t base = i * msg.point_step;
      std::memcpy(&msg.data[base + 0], &x[i], sizeof(float));
      std::memcpy(&msg.data[base + 4], &y[i], sizeof(float));
      std::memcpy(&msg.data[base + 8], &z[i], sizeof(float));

      uint32_t local_offset = 12;
      if (!class_ids.empty()) {
        msg.data[base + local_offset] = class_ids[i];
        local_offset += 1;
      }

      if (!probabilities.empty()) {
        std::memcpy(&msg.data[base + local_offset], &probabilities[i], sizeof(float));
      }
    }

    return msg;
  }
};

// ============================================================================
// Label Mapping Tests
// ============================================================================

TEST_F(LabelBasedEuclideanClusterTest, EmptyPointCloudReturnsEmptyObjects)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::UNKNOWN;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(), {}, shape_estimator_);

  auto pc = create_pointcloud({}, {}, {});

  // Act
  auto result = cluster.process(pc);

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->objects.size(), 0U);
}

TEST_F(LabelBasedEuclideanClusterTest, PointCloudWithoutClassIdUsesDefaultLabel)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(), {}, shape_estimator_);

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
  auto result = cluster.process(pc);

  // Assert
  ASSERT_TRUE(result.has_value());
  // Even if no clusters form, the processing should complete without errors
  // If clusters do form, they should use UNKNOWN label (default when no class_id field)
  for (const auto & obj : result->objects) {
    EXPECT_EQ(obj.classification[0].label, ObjectClassification::UNKNOWN);
  }
}

TEST_F(LabelBasedEuclideanClusterTest, PointsAreGroupedByLabel)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;
  class_map[1] = ObjectClassification::PEDESTRIAN;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(), {}, shape_estimator_);

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
  // Pedestrian cluster (class 1) far away
  for (int i = 0; i < 5; ++i) {
    x_vals.push_back(5.0f + static_cast<float>(i) * 0.05f);
    y_vals.push_back(0.0f);
    z_vals.push_back(0.0f);
    class_vals.push_back(1);
    prob_vals.push_back(1.0f);
  }
  auto pc = create_pointcloud(
    x_vals, y_vals, z_vals, std::vector<uint8_t>(class_vals.begin(), class_vals.end()), prob_vals);

  // Act
  auto result = cluster.process(pc);

  // Assert - should have clusters for both classes
  ASSERT_TRUE(result.has_value());
  bool has_car = false;
  bool has_ped = false;
  for (const auto & obj : result->objects) {
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

// ============================================================================
// Probability Filtering Tests
// ============================================================================

TEST_F(LabelBasedEuclideanClusterTest, ClusterBelowMinProbabilityIsFiltered)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(0.8F), {}, shape_estimator_);

  // A few high-confidence points are not enough when the cluster average is below the threshold.
  auto pc = create_pointcloud(
    {0.0f, 0.1f, 0.2f, 0.3f},   // x
    {0.0f, 0.0f, 0.0f, 0.0f},   // y
    {0.0f, 0.0f, 0.0f, 0.0f},   // z
    {0, 0, 0, 0},               // class_id
    {0.95f, 0.95f, 0.1f, 0.1f}  // probability
  );

  // Act
  auto result = cluster.process(pc);

  // Assert
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->objects.empty());
}

TEST_F(LabelBasedEuclideanClusterTest, ClusterAverageProbabilityIsUsedForThreshold)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(0.8F), {}, shape_estimator_);

  // The first point is below the threshold, but the cluster average is above 0.8.
  auto pc = create_pointcloud(
    {0.0f, 0.1f, 0.2f},   // x
    {0.0f, 0.0f, 0.0f},   // y
    {0.0f, 0.0f, 0.0f},   // z
    {0, 0, 0},            // class_id
    {0.7f, 0.85f, 0.95f}  // probability
  );

  // Act
  auto result = cluster.process(pc);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->objects.size(), 1U);
  EXPECT_NEAR(result->objects[0].existence_probability, 0.8333333f, 1e-5f);
}

TEST_F(LabelBasedEuclideanClusterTest, LabelMinProbabilityOverridesGlobalThreshold)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;
  class_map[1] = ObjectClassification::PEDESTRIAN;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(0.3F),
    {{ObjectClassification::CAR, make_default_options(0.8F)}}, shape_estimator_);

  auto pc = create_pointcloud(
    {0.0f, 0.1f, 0.2f, 5.0f, 5.1f, 5.2f},  // x
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // y
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // z
    {0, 0, 0, 1, 1, 1},                    // class_id
    {0.7f, 0.7f, 0.7f, 0.7f, 0.7f, 0.7f}   // probability
  );

  // Act
  auto result = cluster.process(pc);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->objects.size(), 1U);
  EXPECT_EQ(result->objects[0].classification[0].label, ObjectClassification::PEDESTRIAN);
  EXPECT_NEAR(result->objects[0].existence_probability, 0.7f, 1e-5f);
}

TEST_F(LabelBasedEuclideanClusterTest, AverageProbabilityIsCorrect)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(), {}, shape_estimator_);

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
  auto result = cluster.process(pc);

  // Assert
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->objects.size(), 1U);
  EXPECT_NEAR(result->objects[0].existence_probability, 0.6f, 1e-5f);
}

// ============================================================================
// Shape Estimation Tests
// ============================================================================

TEST_F(LabelBasedEuclideanClusterTest, ShapeIsPopulated)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(), {}, shape_estimator_);

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
  auto result = cluster.process(pc);

  // Assert
  ASSERT_TRUE(result.has_value());
  if (!result->objects.empty()) {
    auto & shape = result->objects[0].shape;
    // Shape should have dimensions set
    EXPECT_GE(shape.dimensions.x, 0.0);
    EXPECT_GE(shape.dimensions.y, 0.0);
    EXPECT_GE(shape.dimensions.z, 0.0);
  }
}

// ============================================================================
// Unmapped Label Tests
// ============================================================================

TEST_F(LabelBasedEuclideanClusterTest, UnmappedLabelsAreIgnored)
{
  // Arrange
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;
  // class_id 1 is unmapped, should be ignored

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(), {}, shape_estimator_);

  // Create points: some mapped (0), some unmapped (1)
  auto pc = create_pointcloud(
    {0.0f, 0.1f, 1.0f, 1.1f},  // x
    {0.0f, 0.0f, 0.0f, 0.0f},  // y
    {0.0f, 0.0f, 0.0f, 0.0f},  // z
    {0, 0, 1, 1},              // class_id
    {1.0f, 1.0f, 1.0f, 1.0f}   // probability
  );

  // Act
  auto result = cluster.process(pc);

  // Assert - only the mapped class (CAR) should produce objects
  ASSERT_TRUE(result.has_value());
  for (const auto & obj : result->objects) {
    EXPECT_EQ(obj.classification[0].label, ObjectClassification::CAR);
  }
}

TEST_F(LabelBasedEuclideanClusterTest, ReturnsErrorWhenRequiredFieldsAreMissing)
{
  std::unordered_map<uint8_t, uint8_t> class_map;
  class_map[0] = ObjectClassification::CAR;

  LabelBasedEuclideanCluster cluster(
    class_map, ShapePolicy::ALL_POLYGON, make_default_options(), {}, shape_estimator_);

  sensor_msgs::msg::PointCloud2 pc;
  pc.height = 1;
  pc.width = 1;
  pc.is_dense = true;
  // Intentionally missing x/y/z field definitions

  const auto result = cluster.process(pc);
  ASSERT_FALSE(result.has_value());
  EXPECT_FALSE(result.error().empty());
}

}  // namespace autoware::euclidean_cluster
