// Copyright 2020 Tier IV, Inc.
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

#include <rclcpp/node.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_points_per_cluster, float tolerance, float voxel_leaf_size,
  int min_points_per_voxel, int large_cluster_voxel_count_threshold,
  int large_cluster_max_points_per_voxel, int max_voxels_per_cluster)
// max cluster size is unused by this cluster executer (oversized groups are split, not dropped).
: EuclideanClusterInterface(use_height, min_points_per_cluster, 0),
  tolerance_(tolerance),
  voxel_leaf_size_(voxel_leaf_size),
  min_points_per_voxel_(min_points_per_voxel),
  large_cluster_voxel_count_threshold_(large_cluster_voxel_count_threshold),
  large_cluster_max_points_per_voxel_(large_cluster_max_points_per_voxel),
  max_voxels_per_cluster_(max_voxels_per_cluster)
{
}
std::vector<pcl::PointIndices> VoxelGridBasedEuclideanCluster::splitOversizedClusters(
  const std::vector<pcl::PointIndices> & cluster_indices,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & centroids) const
{
  std::vector<pcl::PointIndices> result;
  result.reserve(cluster_indices.size());

  // Clamp the bound to at least 1: a non-positive bound would make even a size-1 group "oversized"
  // and bisect it forever (half == 0 re-pushes the same group). At least 1 guarantees termination.
  const int max_voxels = std::max(1, max_voxels_per_cluster_);

  // Work stack of voxel-index groups still to be checked/split.
  std::vector<std::vector<int>> stack;
  stack.reserve(cluster_indices.size());
  for (const auto & ci : cluster_indices) {
    stack.push_back(ci.indices);
  }

  while (!stack.empty()) {
    std::vector<int> group = std::move(stack.back());
    stack.pop_back();

    // Small enough: emit as a final cluster.
    if (static_cast<int>(group.size()) <= max_voxels) {
      pcl::PointIndices pi;
      pi.indices = std::move(group);
      result.push_back(std::move(pi));
      continue;
    }

    // Compute the 2D centroid mean and covariance of the group.
    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const int idx : group) {
      mean_x += centroids->points[idx].x;
      mean_y += centroids->points[idx].y;
    }
    mean_x /= static_cast<double>(group.size());
    mean_y /= static_cast<double>(group.size());

    double cov_xx = 0.0;
    double cov_xy = 0.0;
    double cov_yy = 0.0;
    for (const int idx : group) {
      const double dx = centroids->points[idx].x - mean_x;
      const double dy = centroids->points[idx].y - mean_y;
      cov_xx += dx * dx;
      cov_xy += dx * dy;
      cov_yy += dy * dy;
    }

    // Principal eigenvector of the symmetric 2x2 covariance [[cov_xx, cov_xy], [cov_xy, cov_yy]].
    const double trace = cov_xx + cov_yy;
    const double det = cov_xx * cov_yy - cov_xy * cov_xy;
    const double lambda = trace / 2.0 + std::sqrt(std::max(0.0, trace * trace / 4.0 - det));
    double axis_x = 0.0;
    double axis_y = 0.0;
    if (std::abs(cov_xy) > 1e-9) {
      axis_x = lambda - cov_yy;
      axis_y = cov_xy;
    } else {
      // Diagonal covariance: principal axis is whichever of x/y has the larger spread.
      axis_x = (cov_xx >= cov_yy) ? 1.0 : 0.0;
      axis_y = (cov_xx >= cov_yy) ? 0.0 : 1.0;
    }
    const double norm = std::hypot(axis_x, axis_y);
    axis_x /= norm;
    axis_y /= norm;

    // Sort by projection onto the principal axis; break ties by index for determinism.
    std::sort(group.begin(), group.end(), [&](const int a, const int b) {
      const double proj_a = centroids->points[a].x * axis_x + centroids->points[a].y * axis_y;
      const double proj_b = centroids->points[b].x * axis_x + centroids->points[b].y * axis_y;
      if (proj_a != proj_b) {
        return proj_a < proj_b;
      }
      return a < b;
    });

    // Split at the median rank. Splitting by rank (not value) guarantees both halves are strictly
    // smaller than the parent, so recursion always terminates with every piece <= the bound.
    const size_t half = group.size() / 2;
    stack.emplace_back(group.begin(), group.begin() + half);
    stack.emplace_back(group.begin() + half, group.end());
  }

  return result;
}

// TODO(badai-nguyen): remove this function when field copying also implemented for
// euclidean_cluster.cpp
bool VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  std::vector<IndexedCluster> indexed_clusters;
  const bool success = cluster(pointcloud, indexed_clusters);
  if (!success) {
    return false;
  }

  clusters.clear();
  clusters.reserve(indexed_clusters.size());
  for (auto & cluster : indexed_clusters) {
    clusters.push_back(std::move(cluster.cloud));
  }
  return true;
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<IndexedCluster> & clusters)
{
  // 1) Voxel grid filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  constexpr float Z_AXIS_VOXEL_SIZE = 100000.0f;
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, Z_AXIS_VOXEL_SIZE);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // 2) Build 2D centroid cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;  // Set z to 0.0 for 2D clustering
    pointcloud_2d_ptr->push_back(point2d);
  }

  // 3) KD-tree + clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(1);
  // Do not let PCL drop large connected components: oversized groups are split (not dropped) by
  // splitOversizedClusters() below, which enforces the per-cluster voxel bound.
  pcl_euclidean_cluster.setMaxClusterSize(static_cast<int>(pointcloud_2d_ptr->size()));
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // Split any cluster larger than max_voxels_per_cluster_ into bounded sub-clusters instead
  // of dropping it, so large groups are still exported as smaller sections.
  cluster_indices = splitOversizedClusters(cluster_indices, pointcloud_2d_ptr);

  // 4) Map voxel index to cluster index
  std::unordered_map<int, int> voxel_to_cluster_map;
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      voxel_to_cluster_map[point_idx] = cluster_idx;
    }
  }

  // Precompute which clusters are large enough based on the voxel threshold.
  std::vector<bool> is_large_cluster(cluster_indices.size(), false);
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const int cluster_size = static_cast<int>(cluster_indices[cluster_idx].indices.size());
    is_large_cluster[cluster_idx] = cluster_size > large_cluster_voxel_count_threshold_;
  }

  // 5) Prepare output clusters
  clusters.clear();
  clusters.resize(cluster_indices.size());
  // Track how many points each voxel has per cluster
  std::unordered_map<int, std::unordered_map<int, int>> point_counts_per_voxel_per_cluster;
  std::vector<size_t> random_indices(pointcloud->points.size());
  static std::default_random_engine rng(42);
  std::iota(random_indices.begin(), random_indices.end(), 0);
  std::shuffle(random_indices.begin(), random_indices.end(), rng);
  for (size_t i = 0; i < random_indices.size(); ++i) {
    const size_t random_index = random_indices[i];
    const auto & point = pointcloud->points.at(random_index);
    const Eigen::Vector3i grid_coord = voxel_grid_.getGridCoordinates(point.x, point.y, point.z);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
    const int voxel_index = voxel_grid_.getCentroidIndexAt(grid_coord);
#pragma GCC diagnostic pop
    auto voxel_to_cluster_map_it = voxel_to_cluster_map.find(voxel_index);
    if (voxel_to_cluster_map_it != voxel_to_cluster_map.end()) {
      int cluster_idx = voxel_to_cluster_map_it->second;
      if (is_large_cluster[cluster_idx]) {
        int & voxel_point_count = point_counts_per_voxel_per_cluster[cluster_idx][voxel_index];
        if (voxel_point_count >= large_cluster_max_points_per_voxel_) {
          continue;
        }
        voxel_point_count++;
      }
      clusters.at(cluster_idx).cloud.push_back(point);
      clusters.at(cluster_idx).indices.push_back(static_cast<int>(random_index));
    }
  }

  // Remove clusters that are too small
  clusters.erase(
    std::remove_if(
      clusters.begin(), clusters.end(),
      [this](const IndexedCluster & cluster) {
        return static_cast<int>(cluster.cloud.size()) < min_points_per_cluster_;
      }),
    clusters.end());

  for (auto & cluster : clusters) {
    cluster.cloud.width = cluster.cloud.points.size();
    cluster.cloud.height = 1;
    cluster.cloud.is_dense = false;
  }

  return true;
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & objects)
{
  // TODO(Saito) implement use_height is false version
  // 1) Convert ROS PointCloud2 to PCL cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  int point_step = pointcloud_msg->point_step;
  pcl::fromROSMsg(*pointcloud_msg, *pointcloud);
  // 2) Voxel grid filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  constexpr float Z_AXIS_VOXEL_SIZE = 100000.0f;
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, Z_AXIS_VOXEL_SIZE);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // 3) Build 2D centroid cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;  // Set z to 0.0 for 2D clustering
    pointcloud_2d_ptr->push_back(point2d);
  }

  // 4) KD-tree + clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);
  // Perform clustering using EuclideanClusterExtraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(1);
  // Do not let PCL drop large connected components: oversized groups are split (not dropped) by
  // splitOversizedClusters() below, which enforces the per-cluster voxel bound.
  pcl_euclidean_cluster.setMaxClusterSize(static_cast<int>(pointcloud_2d_ptr->size()));
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // Split any cluster larger than max_voxels_per_cluster_ into bounded sub-clusters instead
  // of dropping it, so large groups are still exported as smaller sections.
  cluster_indices = splitOversizedClusters(cluster_indices, pointcloud_2d_ptr);

  // 5) Buffer preparation
  // Map to store the mapping between voxel grid indices and their corresponding cluster indices
  std::unordered_map</* voxel grid index */ int, /* cluster index */ int> voxel_to_cluster_map;
  std::vector<sensor_msgs::msg::PointCloud2> temporary_clusters;  // no check about cluster size
  std::vector<size_t> clusters_data_size;
  temporary_clusters.resize(cluster_indices.size());
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    auto & temporary_cluster = temporary_clusters.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      voxel_to_cluster_map[point_idx] = cluster_idx;
    }
    temporary_cluster.height = pointcloud_msg->height;
    temporary_cluster.fields = pointcloud_msg->fields;
    temporary_cluster.point_step = point_step;
    temporary_cluster.data.resize(cluster.indices.size() * point_step);
    clusters_data_size.push_back(0);
  }
  // Precompute which clusters are large enough based on the voxel threshold.
  // This avoids repeatedly checking the size during per-point processing.
  std::vector<bool> is_large_cluster(cluster_indices.size(), false);

  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const int cluster_size = static_cast<int>(cluster_indices[cluster_idx].indices.size());
    is_large_cluster[cluster_idx] = cluster_size > large_cluster_voxel_count_threshold_;
  }

  // 6) Data copy
  // Initialize a map to track how many points each voxel has per cluster.
  // Key: cluster index -> (Key: voxel index -> value: point count)
  std::unordered_map<int, std::unordered_map<int, int>> point_counts_per_voxel_per_cluster;
  std::vector<size_t> random_indices(pointcloud->points.size());
  static std::default_random_engine rng(42);
  std::iota(random_indices.begin(), random_indices.end(), 0);
  std::shuffle(random_indices.begin(), random_indices.end(), rng);
  for (size_t i = 0; i < random_indices.size(); ++i) {
    const size_t random_index = random_indices[i];
    const auto & point = pointcloud->points.at(random_index);
    // for (size_t i = 0; i < pointcloud->points.size(); ++i) {
    // const auto & point = pointcloud->points.at(i);

    const Eigen::Vector3i grid_coord = voxel_grid_.getGridCoordinates(point.x, point.y, point.z);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
    const int voxel_index = voxel_grid_.getCentroidIndexAt(grid_coord);
#pragma GCC diagnostic pop

    auto voxel_to_cluster_map_it = voxel_to_cluster_map.find(voxel_index);
    if (voxel_to_cluster_map_it != voxel_to_cluster_map.end()) {
      // Track point count per voxel per cluster
      int cluster_idx = voxel_to_cluster_map_it->second;
      if (is_large_cluster[cluster_idx]) {
        int & voxel_point_count = point_counts_per_voxel_per_cluster[cluster_idx][voxel_index];
        if (voxel_point_count >= large_cluster_max_points_per_voxel_) {
          continue;  // Skip adding this point
        }
        voxel_point_count++;
      }

      auto & cluster_data_size = clusters_data_size.at(voxel_to_cluster_map_it->second);
      std::memcpy(
        &temporary_clusters.at(voxel_to_cluster_map_it->second).data[cluster_data_size],
        &pointcloud_msg->data[random_index * point_step], point_step);
      cluster_data_size += point_step;
      if (cluster_data_size == temporary_clusters.at(voxel_to_cluster_map_it->second).data.size()) {
        temporary_clusters.at(voxel_to_cluster_map_it->second)
          .data.resize(temporary_clusters.at(voxel_to_cluster_map_it->second).data.size() * 2);
      }
    }
  }

  // build output and check cluster size
  {
    for (size_t i = 0; i < temporary_clusters.size(); ++i) {
      auto & i_cluster_data_size = clusters_data_size.at(i);
      int cluster_size = static_cast<int>(i_cluster_data_size / point_step);
      if (cluster_size < min_points_per_cluster_) {
        // Cluster size is below the minimum threshold; skip without messaging.
        continue;
      }
      const auto & cluster = temporary_clusters.at(i);
      tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
      feature_object.feature.cluster = cluster;
      feature_object.feature.cluster.data.resize(i_cluster_data_size);
      feature_object.feature.cluster.header = pointcloud_msg->header;
      feature_object.feature.cluster.is_bigendian = pointcloud_msg->is_bigendian;
      feature_object.feature.cluster.is_dense = pointcloud_msg->is_dense;
      feature_object.feature.cluster.point_step = point_step;
      feature_object.feature.cluster.row_step = i_cluster_data_size / pointcloud_msg->height;
      feature_object.feature.cluster.width =
        i_cluster_data_size / point_step / pointcloud_msg->height;

      feature_object.object.kinematics.pose_with_covariance.pose.position =
        getCentroid(feature_object.feature.cluster);
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 1.0f;
      feature_object.object.classification.emplace_back(classification);

      objects.feature_objects.push_back(feature_object);
    }
    objects.header = pointcloud_msg->header;
  }

  return true;
}

}  // namespace autoware::euclidean_cluster
