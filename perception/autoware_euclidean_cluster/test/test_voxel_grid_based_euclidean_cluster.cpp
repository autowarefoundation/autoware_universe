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

#include <cmath>
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

// Generate a single elongated cluster: a line of points of the given length along x (narrow in y),
// densely sampled so consecutive voxels are connected.
sensor_msgs::msg::PointCloud2 generateLongCluster(const double length_m, const double spacing_m)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  const int nb_points = static_cast<int>(length_m / spacing_m) + 1;
  pointcloud.data.resize(nb_points * pointcloud.point_step);
  for (int i = 0; i < nb_points; ++i) {
    PointXYZI point;
    point.x = i * spacing_m;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 0.0;
    memcpy(&pointcloud.data[i * pointcloud.point_step], &point, pointcloud.point_step);
  }
  pointcloud.width = nb_points;
  pointcloud.row_step = pointcloud.point_step * nb_points;
  return pointcloud;
}

// Compute the 2D AABB diagonal [m] of a cluster point cloud.
double clusterDiagonal(const sensor_msgs::msg::PointCloud2 & cluster)
{
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x"), iter_y(cluster, "y");
       iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    min_x = std::min(min_x, *iter_x);
    max_x = std::max(max_x, *iter_x);
    min_y = std::min(min_y, *iter_y);
    max_y = std::max(max_y, *iter_y);
  }
  return std::hypot(max_x - min_x, max_y - min_y);
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
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 1;
  int max_cluster_size = 100;
  int min_voxel_cluster_size_for_filtering = 150;
  int max_points_per_voxel_in_large_cluster = 10;
  int max_voxel_cluster_for_output = 500;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel, min_voxel_cluster_size_for_filtering,
    max_points_per_voxel_in_large_cluster, max_voxel_cluster_for_output);
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
// than min_cluster_size
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
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 2;
  int max_cluster_size = 100;
  int min_voxel_cluster_size_for_filtering = 150;
  int max_points_per_voxel_in_large_cluster = 10;
  int max_voxel_cluster_for_output = 500;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel, min_voxel_cluster_size_for_filtering,
    max_points_per_voxel_in_large_cluster, max_voxel_cluster_for_output);
  if (cluster_->cluster(pointcloud_msg, output)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;
  // the output clusters should be empty
  EXPECT_EQ(output.feature_objects.size(), 0);
}

// Test case 3: Test case when the input pointcloud has cluster with voxel greater than
// max_voxel_cluster_for_output
TEST(VoxelGridBasedEuclideanClusterTest, testcase3)
{
  int nb_generated_points = 10;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxelUniform(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.1;
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 1;
  int max_cluster_size = 99;
  int min_voxel_cluster_size_for_filtering = 150;
  int max_points_per_voxel_in_large_cluster = 10;
  int max_voxel_cluster_for_output =
    8;  // voxel num is 0.3*0.3/0.1^2 = 9, so max_voxel_cluster_for_output = 8 will be filtered out
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel, min_voxel_cluster_size_for_filtering,
    max_points_per_voxel_in_large_cluster, max_voxel_cluster_for_output);
  if (cluster_->cluster(pointcloud_msg, output)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;
  // the output clusters should be emtpy
  EXPECT_EQ(output.feature_objects.size(), 0);
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
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 1;
  int max_cluster_size = 100;
  int min_voxel_cluster_size_for_filtering = 150;
  int max_points_per_voxel_in_large_cluster = 10;
  int max_voxel_cluster_for_output = 500;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel, min_voxel_cluster_size_for_filtering,
    max_points_per_voxel_in_large_cluster, max_voxel_cluster_for_output);

  // Should not crash and should return empty output
  EXPECT_TRUE(cluster_->cluster(pointcloud_msg, output));
  EXPECT_EQ(output.feature_objects.size(), 0);
}

// Test case 5: A single elongated cluster is split into smaller pieces when
// max_cluster_diagonal_size is set, and each piece respects the diagonal limit.
TEST(VoxelGridBasedEuclideanClusterTest, testcase5_footprint_split)
{
  const double length_m = 6.0;
  const double spacing_m = 0.15;
  sensor_msgs::msg::PointCloud2 pointcloud = generateLongCluster(length_m, spacing_m);
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);

  const float tolerance = 0.7;
  const float voxel_leaf_size = 0.3;
  const int min_points_number_per_voxel = 1;
  const int min_cluster_size = 1;
  const int max_cluster_size = 1000;
  const int min_voxel_cluster_size_for_filtering = 1000;
  const int max_points_per_voxel_in_large_cluster = 10;
  const int max_voxel_cluster_for_output = 1000;
  const bool use_height = false;

  // Splitting disabled (max_cluster_diagonal_size == 0): the whole line stays one cluster.
  {
    auto cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
      use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
      min_points_number_per_voxel, min_voxel_cluster_size_for_filtering,
      max_points_per_voxel_in_large_cluster, max_voxel_cluster_for_output, 0.0f);
    tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
    cluster_->cluster(pointcloud_msg, output);
    EXPECT_EQ(output.feature_objects.size(), 1u);
  }

  // Splitting enabled: the ~6 m line is split into multiple pieces, each within the diagonal limit.
  {
    const float max_cluster_diagonal_size = 2.0f;
    auto cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
      use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
      min_points_number_per_voxel, min_voxel_cluster_size_for_filtering,
      max_points_per_voxel_in_large_cluster, max_voxel_cluster_for_output,
      max_cluster_diagonal_size);
    tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
    cluster_->cluster(pointcloud_msg, output);
    std::cout << "split into " << output.feature_objects.size() << " clusters" << std::endl;
    EXPECT_GT(output.feature_objects.size(), 1u);
    // Each output cluster's footprint diagonal should be within the limit. The split is computed on
    // voxel centroids, while raw points can extend up to half a leaf beyond each boundary centroid,
    // so allow two voxels of slack.
    for (const auto & obj : output.feature_objects) {
      EXPECT_LE(
        clusterDiagonal(obj.feature.cluster), max_cluster_diagonal_size + 2.0f * voxel_leaf_size);
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
