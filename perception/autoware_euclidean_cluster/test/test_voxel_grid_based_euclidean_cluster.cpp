// Copyright 2024 TIER IV, Inc.
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

#include "autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <autoware/point_types/types.hpp>
#include <experimental/random>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>

using autoware::point_types::PointXYZI;
void setPointCloud2Fields(sensor_msgs::msg::PointCloud2 & pointcloud)
{
  pointcloud.fields.resize(4);
  pointcloud.fields[0].name = "x";
  pointcloud.fields[1].name = "y";
  pointcloud.fields[2].name = "z";
  pointcloud.fields[3].name = "intensity";
  pointcloud.fields[0].offset = 0;
  pointcloud.fields[1].offset = 4;
  pointcloud.fields[2].offset = 8;
  pointcloud.fields[3].offset = 12;
  pointcloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[0].count = 1;
  pointcloud.fields[1].count = 1;
  pointcloud.fields[2].count = 1;
  pointcloud.fields[3].count = 1;
  pointcloud.height = 1;
  pointcloud.point_step = 16;
  pointcloud.is_bigendian = false;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = "dummy_frame_id";
  pointcloud.header.stamp.sec = 0;
  pointcloud.header.stamp.nanosec = 0;
}

sensor_msgs::msg::PointCloud2 generateClusterWithinVoxel(const int nb_points)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  pointcloud.data.resize(nb_points * pointcloud.point_step);

  // generate one cluster with specified number of points within 1 voxel
  for (int i = 0; i < nb_points; ++i) {
    PointXYZI point;
    point.x = std::experimental::randint(0, 30) / 100.0;  // point.x within 0.0 to 0.3
    point.y = std::experimental::randint(0, 30) / 100.0;  // point.y within 0.0 to 0.3
    point.z = std::experimental::randint(0, 30) / 1.0;
    point.intensity = 0.0;
    memcpy(&pointcloud.data[i * pointcloud.point_step], &point, pointcloud.point_step);
  }
  pointcloud.width = nb_points;
  pointcloud.row_step = pointcloud.point_step * nb_points;
  return pointcloud;
}

sensor_msgs::msg::PointCloud2 generateClusterWithinVoxelUniform(const int nb_points)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  const int total_points = nb_points * nb_points;
  pointcloud.data.resize(total_points * pointcloud.point_step);

  // generate one cluster with specified number of points within 1 voxel
  for (int i = 0; i < nb_points; ++i) {
    for (int j = 0; j < nb_points; ++j) {
      PointXYZI point;
      point.x = 0.3 * i / nb_points;  // point.x within 0.0 to 0.3
      point.y = 0.3 * j / nb_points;  // point.y within 0.0 to 0.3
      point.z = 0.0;
      point.intensity = 0.0;
      const int idx = (i * nb_points + j) * pointcloud.point_step;
      memcpy(&pointcloud.data[idx], &point, pointcloud.point_step);
    }
  }
  pointcloud.width = total_points;
  pointcloud.row_step = pointcloud.point_step * total_points;
  return pointcloud;
}

// Generate a solid rectangular blob spanning nx by ny voxels (leaf-sized cells), with
// points_per_voxel distinct points inside each voxel. Produces one connected cluster of nx*ny
// occupied voxels under a tolerance >= leaf.
sensor_msgs::msg::PointCloud2 generateVoxelRectangle(
  const int nx, const int ny, const float leaf, const int points_per_voxel)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  const int total_points = nx * ny * points_per_voxel;
  pointcloud.data.resize(total_points * pointcloud.point_step);

  int n = 0;
  for (int ix = 0; ix < nx; ++ix) {
    for (int iy = 0; iy < ny; ++iy) {
      for (int p = 0; p < points_per_voxel; ++p) {
        PointXYZI point;
        // Spread the points across the interior of voxel (ix, iy) so they stay in the same cell.
        point.x = leaf * (ix + 0.3f + 0.4f * static_cast<float>(p) / points_per_voxel);
        point.y = leaf * (iy + 0.5f);
        point.z = 0.0;
        point.intensity = 0.0;
        memcpy(&pointcloud.data[n * pointcloud.point_step], &point, pointcloud.point_step);
        ++n;
      }
    }
  }
  pointcloud.width = total_points;
  pointcloud.row_step = pointcloud.point_step * total_points;
  return pointcloud;
}

// Test case 1: Test case when the input pointcloud has only one cluster with points number equal to
// max_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase1)
{
  int nb_generated_points = 100;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_per_voxel = 1;
  int min_points_per_cluster = 1;
  int large_cluster_voxel_count_threshold = 150;
  int large_cluster_max_points_per_voxel = 10;
  int max_voxels_per_cluster = 500;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);
  if (cluster_->cluster(pointcloud_msg, output)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;
  std::cout << "number points of first cluster: " << output.feature_objects[0].feature.cluster.width
            << std::endl;
  // the output clusters should has only one cluster with nb_generated_points points
  EXPECT_EQ(output.feature_objects.size(), 1);
  EXPECT_EQ(output.feature_objects[0].feature.cluster.width, nb_generated_points);
}

// Test case 2: Test case when the input pointcloud has only one cluster with points number less
// than min_points_per_cluster
TEST(VoxelGridBasedEuclideanClusterTest, testcase2)
{
  int nb_generated_points = 1;

  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_per_voxel = 1;
  int min_points_per_cluster = 2;
  int large_cluster_voxel_count_threshold = 150;
  int large_cluster_max_points_per_voxel = 10;
  int max_voxels_per_cluster = 500;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);
  if (cluster_->cluster(pointcloud_msg, output)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;
  // the output clusters should be empty
  EXPECT_EQ(output.feature_objects.size(), 0);
}

// Test case 3: A cluster whose voxel count exceeds max_voxels_per_cluster is now SPLIT into
// smaller sub-clusters (not dropped), and no point is lost.
TEST(VoxelGridBasedEuclideanClusterTest, testcase3)
{
  int nb_generated_points = 10;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxelUniform(nb_generated_points);
  const int total_points = nb_generated_points * nb_generated_points;  // 100

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.1;
  int min_points_per_voxel = 1;
  int min_points_per_cluster = 1;
  int large_cluster_voxel_count_threshold = 150;  // disable per-voxel point capping
  int large_cluster_max_points_per_voxel = 10;
  int max_voxels_per_cluster =
    8;  // voxel num is 0.3*0.3/0.1^2 = 9, so the single 9-voxel cluster is split into pieces <= 8
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);
  EXPECT_TRUE(cluster_->cluster(pointcloud_msg, output));
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;

  // 9 voxels split at the median into 4 + 5 -> exactly 2 sub-clusters, both within the bound.
  EXPECT_EQ(output.feature_objects.size(), 2);
  size_t total_output_points = 0;
  for (const auto & feature_object : output.feature_objects) {
    total_output_points += feature_object.feature.cluster.width;
  }
  // No point is dropped: every input point is preserved across the split sub-clusters.
  EXPECT_EQ(total_output_points, static_cast<size_t>(total_points));
}

// Test case 4: Test case when the input pointcloud is empty
TEST(VoxelGridBasedEuclideanClusterTest, EmptyPointCloud)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  pointcloud.width = 0;
  pointcloud.row_step = 0;

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_per_voxel = 1;
  int min_points_per_cluster = 1;
  int large_cluster_voxel_count_threshold = 150;
  int large_cluster_max_points_per_voxel = 10;
  int max_voxels_per_cluster = 500;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);

  // Should not crash and should return empty output
  EXPECT_TRUE(cluster_->cluster(pointcloud_msg, output));
  EXPECT_EQ(output.feature_objects.size(), 0);
}

// Test case 5: A large blob is split so that every output cluster is within the voxel bound, and
// all points are conserved (nothing dropped).
TEST(VoxelGridBasedEuclideanClusterTest, SplitRespectsBoundAndConservesPoints)
{
  const int nx = 20;
  const int ny = 5;  // 100 occupied voxels
  const float voxel_leaf_size = 0.3;
  const int points_per_voxel = 4;
  sensor_msgs::msg::PointCloud2 pointcloud =
    generateVoxelRectangle(nx, ny, voxel_leaf_size, points_per_voxel);
  const int total_points = nx * ny * points_per_voxel;

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;

  float tolerance = 0.7;
  int min_points_per_voxel = 1;
  int min_points_per_cluster = 1;
  int large_cluster_voxel_count_threshold = 100000;  // disable per-voxel point capping
  int large_cluster_max_points_per_voxel = 10;
  int max_voxels_per_cluster = 16;  // split bound in voxels
  bool use_height = false;
  auto cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);
  EXPECT_TRUE(cluster_->cluster(pointcloud_msg, output));

  // 100 voxels with a bound of 16 must produce several sub-clusters.
  EXPECT_GE(output.feature_objects.size(), 100u / 16u);

  size_t total_output_points = 0;
  for (const auto & feature_object : output.feature_objects) {
    const size_t cluster_points = feature_object.feature.cluster.width;
    // Capping is disabled, so all points of each voxel are kept; voxel count = points / per-voxel.
    const size_t voxel_count = cluster_points / points_per_voxel;
    EXPECT_LE(voxel_count, static_cast<size_t>(max_voxels_per_cluster));
    total_output_points += cluster_points;
  }
  // Nothing dropped: every input point survives the split.
  EXPECT_EQ(total_output_points, static_cast<size_t>(total_points));
}

// Test case 6: An elongated wall is bisected along its long axis, so each sub-cluster covers a
// shorter span of the long (x) axis while still spanning the full short (y) axis.
TEST(VoxelGridBasedEuclideanClusterTest, SplitsAlongPrincipalAxis)
{
  const int nx = 40;  // long axis
  const int ny = 2;   // short axis
  const float voxel_leaf_size = 0.3;
  const int points_per_voxel = 2;
  sensor_msgs::msg::PointCloud2 pointcloud =
    generateVoxelRectangle(nx, ny, voxel_leaf_size, points_per_voxel);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;

  float tolerance = 0.7;
  int min_points_per_voxel = 1;
  int min_points_per_cluster = 1;
  int large_cluster_voxel_count_threshold = 100000;
  int large_cluster_max_points_per_voxel = 10;
  int max_voxels_per_cluster = 20;  // 80 voxels -> several pieces
  bool use_height = false;
  auto cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_points_per_cluster, tolerance, voxel_leaf_size, min_points_per_voxel,
    large_cluster_voxel_count_threshold, large_cluster_max_points_per_voxel,
    max_voxels_per_cluster);
  EXPECT_TRUE(cluster_->cluster(pointcloud_msg, output));
  EXPECT_GT(output.feature_objects.size(), 1u);

  const float full_x_span = voxel_leaf_size * nx;  // ~12 m
  const float full_y_span = voxel_leaf_size * ny;  // ~0.6 m
  for (const auto & feature_object : output.feature_objects) {
    const auto & cloud = feature_object.feature.cluster;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      min_x = std::min(min_x, *iter_x);
      max_x = std::max(max_x, *iter_x);
      min_y = std::min(min_y, *iter_y);
      max_y = std::max(max_y, *iter_y);
    }
    const float x_span = max_x - min_x;
    const float y_span = max_y - min_y;
    // Cuts are along the long axis: each piece is shorter in x than the whole wall...
    EXPECT_LT(x_span, full_x_span);
    // ...while still spanning (nearly) the full short axis, i.e. it was not cut along y.
    EXPECT_GE(y_span, full_y_span - voxel_leaf_size);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
