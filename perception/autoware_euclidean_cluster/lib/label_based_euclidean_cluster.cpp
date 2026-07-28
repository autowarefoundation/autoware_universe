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
#include <autoware/ptv3/experimental/semantic_label_helper.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/common/common.h>

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
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

struct SemanticPoint
{
  pcl::PointXYZ point;
  float probability{};
  std::uint8_t class_id{};
};

struct SplitPointcloudResult
{
  std::unordered_map<std::uint8_t, std::vector<SemanticPoint>> object_points;
  sensor_msgs::msg::PointCloud2 segment_points;
};

/// @brief Check whether a point cloud contains a field with the expected datatype.
bool has_field(
  const sensor_msgs::msg::PointCloud2 & pointcloud, const std::string & name,
  const std::uint8_t datatype)
{
  return std::any_of(pointcloud.fields.begin(), pointcloud.fields.end(), [&](const auto & field) {
    return field.name == name && field.datatype == datatype;
  });
}

/// @brief Create an empty point cloud that preserves the input point field layout.
sensor_msgs::msg::PointCloud2 create_empty_segment_pointcloud(
  const sensor_msgs::msg::PointCloud2 & input)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = input.header;
  output.height = 1;
  output.width = 0;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = 0;
  output.is_dense = input.is_dense;
  return output;
}

/// @brief Append one original input point to a segment cloud while preserving all point fields.
void append_segment_point(
  sensor_msgs::msg::PointCloud2 & output, const sensor_msgs::msg::PointCloud2 & input,
  const std::size_t point_index)
{
  const auto row = point_index / input.width;
  const auto column = point_index % input.width;
  const auto input_offset = row * input.row_step + column * input.point_step;
  const auto input_begin = input.data.begin() + static_cast<std::ptrdiff_t>(input_offset);
  const auto input_end = input_begin + static_cast<std::ptrdiff_t>(input.point_step);
  output.data.insert(output.data.end(), input_begin, input_end);
  ++output.width;
  output.row_step = output.point_step * output.width;
}

/// @brief Add a SemanticLabel point to object buckets or segment output.
void append_classified_point(
  SplitPointcloudResult & result, const pcl::PointXYZ & point, const std::uint8_t class_id,
  const float probability, const sensor_msgs::msg::PointCloud2 & input,
  const std::size_t point_index)
{
  namespace ptv3 = autoware::ptv3::experimental;

  const auto semantic_label = static_cast<ptv3::SemanticLabel>(class_id);
  const auto object_label = ptv3::try_into_object(semantic_label);
  if (object_label) {
    result.object_points[*object_label].push_back(SemanticPoint{point, probability, class_id});
    return;
  }

  append_segment_point(result.segment_points, input, point_index);
}

/// @brief Split semantic points into buckets keyed by object label.
SplitPointcloudResult split_pointcloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud, const float min_probability)
{
  SplitPointcloudResult result;
  result.segment_points = create_empty_segment_pointcloud(pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  const bool has_class_id = has_field(pointcloud, "class_id", sensor_msgs::msg::PointField::UINT8);
  const bool has_probability =
    has_field(pointcloud, "probability", sensor_msgs::msg::PointField::FLOAT32);

  if (has_class_id && has_probability) {
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> iter_class(pointcloud, "class_id");
    sensor_msgs::PointCloud2ConstIterator<float> iter_probability(pointcloud, "probability");
    std::size_t point_index = 0;
    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_class, ++iter_probability, ++point_index) {
      if (*iter_probability < min_probability) {
        continue;
      }

      append_classified_point(
        result, pcl::PointXYZ(*iter_x, *iter_y, *iter_z), *iter_class, *iter_probability,
        pointcloud, point_index);
    }
    return result;
  }

  if (has_class_id) {
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> iter_class(pointcloud, "class_id");
    std::size_t point_index = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_class, ++point_index) {
      append_classified_point(
        result, pcl::PointXYZ(*iter_x, *iter_y, *iter_z), *iter_class, 1.0F, pointcloud,
        point_index);
    }
    return result;
  }

  if (has_probability) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_probability(pointcloud, "probability");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_probability) {
      if (*iter_probability < min_probability) {
        continue;
      }

      result.object_points[ObjectClassification::UNKNOWN].push_back(
        SemanticPoint{pcl::PointXYZ(*iter_x, *iter_y, *iter_z), *iter_probability, 0U});
    }
    return result;
  }

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    result.object_points[ObjectClassification::UNKNOWN].push_back(
      SemanticPoint{pcl::PointXYZ(*iter_x, *iter_y, *iter_z), 1.0F, 0U});
  }

  return result;
}

/// @brief Compute the average semantic probability for one clustered object instance.
/// @details `indices` are relative to the per-label filtered cloud built from `points`.
float cluster_probability_from_indices(
  const std::vector<SemanticPoint> & points, const pcl::Indices & indices)
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

  // Check for required fields
  if (
    !has_field(input_msg, "x", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(input_msg, "y", sensor_msgs::msg::PointField::FLOAT32) ||
    !has_field(input_msg, "z", sensor_msgs::msg::PointField::FLOAT32)) {
    return tl::unexpected(std::string("Input pointcloud missing required float32 fields: x, y, z"));
  }

  // 1. Split points by label and filter by probability
  auto split_points = split_pointcloud(input_msg, min_probability_);
  output.segments = std::move(split_points.segment_points);

  // 2. Run per-label clustering and collect all cluster entries
  std::vector<ClusterEntry> all_entries;
  for (const auto & [label, semantic_points] : split_points.object_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    label_cloud->reserve(semantic_points.size());
    for (const auto & sp : semantic_points) {
      label_cloud->push_back(sp.point);
    }

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

  // 3. Post-merge clusters that belong to the same confusable label group
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

  // 4. Build detected objects from final entries
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
