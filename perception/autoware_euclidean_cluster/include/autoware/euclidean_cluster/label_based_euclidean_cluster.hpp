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

#pragma once

#include "autoware/euclidean_cluster/confusable_cluster_merger.hpp"
#include "autoware/euclidean_cluster/euclidean_cluster_interface.hpp"

#include <autoware/shape_estimation/shape_estimator.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::euclidean_cluster
{
enum ShapePolicy : uint8_t {
  ALL_POLYGON = 0,
  LABEL_DEPEND = 1,
};

/// @brief Core clustering logic without ROS dependencies.
///
/// This class encapsulates the semantic point cloud clustering algorithm, independent of
/// ROS infrastructure. It takes semantic point cloud data and produces detected objects
/// through label-based clustering and shape estimation.
///
/// The clustering process:
/// 1. Filters and splits points by semantic label
/// 2. Runs independent euclidean clustering per label
/// 3. Merges over-segmented clusters across confusable labels
/// 4. Estimates shapes and poses for each cluster
/// 5. Returns structured detected objects
class LabelBasedEuclideanCluster
{
public:
  /// @brief Construct with full configuration for clustering and shape estimation.
  /// @param class_id_to_object_label Mapping from input class ID to Autoware object label.
  /// @param min_probability Minimum confidence threshold for points.
  /// @param shape_policy Strategy for shape estimation (ALL_POLYGON or LABEL_DEPEND).
  /// @param default_cluster Default cluster executer used when no per-label override exists.
  /// @param label_cluster_executers Optional per-label cluster executer overrides.
  /// @param shape_estimator Shape estimator for converting clusters to DetectedObjects.
  /// @param confusable_groups Optional label groups for post-clustering merge.
  LabelBasedEuclideanCluster(
    const std::unordered_map<std::uint8_t, std::uint8_t> & class_id_to_object_label,
    float min_probability, ShapePolicy shape_policy,
    std::shared_ptr<EuclideanClusterInterface> default_cluster,
    const std::unordered_map<std::uint8_t, std::shared_ptr<EuclideanClusterInterface>> &
      label_cluster_executers,
    std::shared_ptr<autoware::shape_estimation::ShapeEstimator> shape_estimator,
    const std::vector<ConfusableLabelGroup> & confusable_groups = {});

  /// @brief Process an input semantic point cloud and return detected objects.
  ///
  /// Note: Returned DetectedObjects will have empty frame_id and timestamp.
  /// The caller (ROS node) must populate these fields from the input message.
  ///
  /// @param input_msg Input point cloud with xyz, and optionally class_id / probability.
  /// @return Detected objects without frame_id or timestamp populated.
  autoware_perception_msgs::msg::DetectedObjects process(
    const sensor_msgs::msg::PointCloud2 & input_msg);

private:
  std::unordered_map<std::uint8_t, std::uint8_t> class_id_to_object_label_;
  float min_probability_;
  ShapePolicy shape_policy_;
  std::shared_ptr<EuclideanClusterInterface> default_cluster_;
  std::unordered_map<std::uint8_t, std::shared_ptr<EuclideanClusterInterface>>
    label_cluster_executers_;
  std::shared_ptr<autoware::shape_estimation::ShapeEstimator> shape_estimator_;
  std::vector<ConfusableLabelGroup> confusable_groups_;
  std::unordered_map<std::uint8_t, std::size_t> label_to_group_idx_;

  /// @brief Return the cluster executer for the given label, or default if no override exists.
  EuclideanClusterInterface & get_cluster_executer(std::uint8_t label) const;
};

}  // namespace autoware::euclidean_cluster
