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

#include "autoware/euclidean_cluster/euclidean_cluster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
EuclideanCluster::EuclideanCluster()
{
}

EuclideanCluster::EuclideanCluster(
  bool use_height, int min_points_per_cluster, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_points_per_cluster, max_cluster_size)
{
}

EuclideanCluster::EuclideanCluster(
  bool use_height, int min_points_per_cluster, int max_cluster_size, float tolerance)
: EuclideanClusterInterface(use_height, min_points_per_cluster, max_cluster_size),
  tolerance_(tolerance)
{
}
// TODO(badai-nguyen): implement input field copy for euclidean_cluster.cpp
bool EuclideanCluster::cluster(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & clusters)
{
  (void)pointcloud_msg;
  (void)clusters;
  return false;
}

bool EuclideanCluster::cluster(
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

bool EuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<IndexedCluster> & clusters)
{
  // convert 2d pointcloud
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (!use_height_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & point : pointcloud->points) {
      pcl::PointXYZ point2d;
      point2d.x = point.x;
      point2d.y = point.y;
      point2d.z = 0.0;
      pointcloud_2d_ptr->push_back(point2d);
    }
    pointcloud_ptr = pointcloud_2d_ptr;
  } else {
    pointcloud_ptr = pointcloud;
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(min_points_per_cluster_);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // build output
  {
    clusters.clear();
    clusters.reserve(cluster_indices.size());
    for (const auto & cluster : cluster_indices) {
      IndexedCluster indexed_cluster;
      indexed_cluster.indices = cluster.indices;
      for (const auto & point_idx : cluster.indices) {
        indexed_cluster.cloud.points.push_back(pointcloud->points[point_idx]);
      }
      indexed_cluster.cloud.width = indexed_cluster.cloud.points.size();
      indexed_cluster.cloud.height = 1;
      indexed_cluster.cloud.is_dense = false;
      clusters.push_back(std::move(indexed_cluster));
    }
  }
  return true;
}

}  // namespace autoware::euclidean_cluster
