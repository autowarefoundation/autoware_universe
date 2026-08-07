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

#include <Eigen/Core>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/pointcloud_classification.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
namespace
{
using autoware::point_types::PointXYZCPE;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

struct SplitResult
{
  std::unordered_map<std::uint8_t, pcl::PointCloud<PointXYZCPE>> object_points;
  pcl::PointCloud<PointXYZCPE> segment_points;
};

/// @brief Convert an input message into a `PointXYZCPE` cloud.
/// @return The converted points, or an error message when the input is not a `PointXYZCPE` cloud.
tl::expected<pcl::PointCloud<PointXYZCPE>, std::string> from_ros_msg(
  const sensor_msgs::msg::PointCloud2 & input)
{
  // The cloud is additionally required to be densely packed, because pcl::fromROSMsg() sizes its
  // output from width/height and then copies `data` without any bounds check.
  const auto num_points = static_cast<std::size_t>(input.width) * input.height;
  if (
    !point_types::is_data_layout_compatible_with_point_xyzcpe(input) ||
    input.point_step != sizeof(PointXYZCPE) ||
    input.row_step != static_cast<std::size_t>(input.point_step) * input.width ||
    input.data.size() != num_points * sizeof(PointXYZCPE)) {
    return tl::unexpected(
      std::string(
        "Input pointcloud is not a densely packed autoware::point_types::PointXYZCPE cloud "
        "(expected a point_step of " +
        std::to_string(sizeof(PointXYZCPE)) + " bytes and matching row_step/data sizes)"));
  }

  pcl::PointCloud<PointXYZCPE> points;
  if (num_points > 0) {
    // pcl::fromROSMsg() takes the address of the first point of its output unconditionally, so it
    // must not be called for an empty cloud.
    pcl::fromROSMsg(input, points);
  }

  return points;
}

/// @brief Add a PointCloudClassification point to object buckets or segment output.
void append_classified_point(SplitResult & result, const PointXYZCPE & point)
{
  namespace utils = autoware::object_recognition_utils;

  const auto classification = static_cast<point_types::PointCloudClassification>(point.class_id);
  const auto object_label = utils::try_into_object(classification);
  if (object_label) {
    result.object_points[*object_label].push_back(point);
    return;
  }

  // The whole point is kept, so every field of the input point is preserved in the segment output.
  result.segment_points.push_back(point);
}

/// @brief Split `PointXYZCPE` points into buckets keyed by object label.
SplitResult split_pointcloud(
  const pcl::PointCloud<PointXYZCPE> & points, const float min_probability)
{
  SplitResult result;
  result.segment_points.header = points.header;
  result.segment_points.is_dense = points.is_dense;

  for (const auto & point : points) {
    if (point.probability < min_probability) {
      continue;
    }
    append_classified_point(result, point);
  }

  return result;
}

/// @brief Compute the average semantic probability for one clustered object instance.
/// @details `indices` are relative to the per-label filtered cloud built from `points`.
float cluster_probability_from_indices(
  const pcl::PointCloud<PointXYZCPE> & points, const pcl::Indices & indices)
{
  if (indices.empty()) {
    return 0.0F;
  }

  float sum = 0.0F;
  for (const auto point_index : indices) {
    if (point_index < 0 || static_cast<std::size_t>(point_index) >= points.size()) {
      throw std::out_of_range(
        "LabelBasedEuclideanCluster: cluster returned a point index outside the source cloud");
    }

    sum += points[static_cast<std::size_t>(point_index)].probability;
  }

  return sum / static_cast<float>(indices.size());
}

/// @brief Return true when the estimator populated a usable shape output.
bool has_usable_estimated_shape(const Shape & shape)
{
  switch (shape.type) {
    case Shape::BOUNDING_BOX:
    case Shape::CYLINDER:
      return shape.dimensions.x > 0.0 && shape.dimensions.y > 0.0 && shape.dimensions.z > 0.0;
    case Shape::POLYGON:
      return !shape.footprint.points.empty() && shape.dimensions.z > 0.0;
    default:
      return false;
  }
}

/// @brief Create fallback shape and pose from the cluster axis-aligned bounding box.
std::pair<Shape, geometry_msgs::msg::Pose> create_fallback_shape_and_pose(
  const pcl::PointCloud<pcl::PointXYZ> & cluster, const std::uint8_t label)
{
  Shape shape;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;

  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  pcl::getMinMax3D(cluster, min_point, max_point);

  pose.position.x = 0.5 * (min_point.x() + max_point.x());
  pose.position.y = 0.5 * (min_point.y() + max_point.y());
  pose.position.z = 0.5 * (min_point.z() + max_point.z());

  const float dx = std::max(max_point.x() - min_point.x(), 0.1F);
  const float dy = std::max(max_point.y() - min_point.y(), 0.1F);
  const float dz = std::max(max_point.z() - min_point.z(), 0.1F);

  if (label == ObjectClassification::PEDESTRIAN) {
    shape.type = Shape::CYLINDER;
    shape.dimensions.x = std::max(dx, dy);
    shape.dimensions.y = std::max(dx, dy);
    shape.dimensions.z = dz;
  } else {
    shape.type = Shape::BOUNDING_BOX;
    shape.dimensions.x = dx;
    shape.dimensions.y = dy;
    shape.dimensions.z = dz;
  }

  return {shape, pose};
}

}  // namespace

LabelBasedEuclideanCluster::LabelBasedEuclideanCluster(
  float min_probability, ShapePolicy shape_policy,
  std::shared_ptr<EuclideanClusterInterface> default_cluster,
  const std::unordered_map<std::uint8_t, std::shared_ptr<EuclideanClusterInterface>> &
    label_cluster_executers,
  std::shared_ptr<autoware::shape_estimation::ShapeEstimator> shape_estimator,
  const std::vector<ConfusableLabelGroup> & confusable_groups)
: min_probability_(min_probability),
  shape_policy_(shape_policy),
  default_cluster_(std::move(default_cluster)),
  label_cluster_executers_(label_cluster_executers),
  shape_estimator_(std::move(shape_estimator)),
  confusable_groups_(confusable_groups)
{
  if (!default_cluster_) {
    throw std::invalid_argument("LabelBasedEuclideanCluster: default_cluster is null");
  }
  if (!shape_estimator_) {
    throw std::invalid_argument("LabelBasedEuclideanCluster: shape_estimator is null");
  }

  // Build label-to-group index for confusable merging
  for (std::size_t g = 0; g < confusable_groups_.size(); ++g) {
    for (const auto label : confusable_groups_[g].labels) {
      const auto [it, inserted] = label_to_group_idx_.emplace(label, g);
      if (!inserted) {
        // Label already exists in another group; keeping first assignment
        // (in non-ROS code, we can't log warnings, but this maintains the same behavior)
      }
    }
  }
}

EuclideanClusterInterface & LabelBasedEuclideanCluster::get_cluster_executer(
  const std::uint8_t label) const
{
  const auto it = label_cluster_executers_.find(label);
  return (it != label_cluster_executers_.end()) ? *it->second : *default_cluster_;
}

LabelBasedEuclideanCluster::result_t LabelBasedEuclideanCluster::process(
  const sensor_msgs::msg::PointCloud2 & input_msg)
{
  Output output;
  // Note: frame_id and timestamp are NOT set here; they must be set by the caller (ROS node)

  // 1. Convert the input message into PointXYZCPE points, which the input is required to carry
  const auto points = from_ros_msg(input_msg);
  if (!points) {
    return tl::unexpected(points.error());
  }

  // 2. Split points by label and filter by probability
  auto split_points = split_pointcloud(*points, min_probability_);
  pcl::toROSMsg(split_points.segment_points, output.segments);

  // 3. Run per-label clustering and collect all cluster entries
  std::vector<ClusterEntry> all_entries;
  for (const auto & [label, semantic_points] : split_points.object_points) {
    // Convert PointXYZCPE points to PointXYZ for clustering
    pcl::PointCloud<pcl::PointXYZ>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(semantic_points, *label_cloud);

    std::vector<IndexedCluster> clusters;
    get_cluster_executer(label).cluster(label_cloud, clusters);

    for (auto & cluster : clusters) {
      if (!cluster.cloud.empty()) {
        const float cluster_probability =
          cluster_probability_from_indices(semantic_points, cluster.indices);
        all_entries.push_back({std::move(cluster.cloud), label, cluster_probability});
      }
    }
  }

  // 4. Post-merge clusters that belong to the same confusable label group
  std::vector<std::vector<ClusterEntry>> per_group(confusable_groups_.size());
  std::vector<ClusterEntry> output_entries;
  output_entries.reserve(all_entries.size());

  for (auto & e : all_entries) {
    const auto it = label_to_group_idx_.find(e.label);
    if (it != label_to_group_idx_.end()) {
      per_group[it->second].push_back(std::move(e));
    } else {
      output_entries.push_back(std::move(e));
    }
  }

  for (std::size_t g = 0; g < confusable_groups_.size(); ++g) {
    for (auto & e : merge_confusable_clusters(std::move(per_group[g]), confusable_groups_[g])) {
      output_entries.push_back(std::move(e));
    }
  }

  // 5. Build detected objects from final entries
  for (const auto & e : output_entries) {
    DetectedObject object;
    Shape shape;
    geometry_msgs::msg::Pose pose;

    // Determine shape label based on policy
    const std::uint8_t shape_label =
      (shape_policy_ == ShapePolicy::LABEL_DEPEND) ? e.label : ObjectClassification::UNKNOWN;

    shape_estimator_->estimateShapeAndPose(
      shape_label, e.cloud, boost::none, boost::none, boost::none, shape, pose);

    if (!has_usable_estimated_shape(shape)) {
      std::tie(shape, pose) = create_fallback_shape_and_pose(e.cloud, e.label);
    }

    object.shape = shape;
    object.existence_probability = e.prob;
    object.classification.push_back(
      autoware_perception_msgs::build<ObjectClassification>().label(e.label).probability(e.prob));
    object.kinematics.pose_with_covariance.pose = pose;
    object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;

    output.objects.objects.push_back(std::move(object));
  }

  return output;
}

}  // namespace autoware::euclidean_cluster
