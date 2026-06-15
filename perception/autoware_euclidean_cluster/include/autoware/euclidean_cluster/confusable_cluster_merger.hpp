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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <vector>

namespace autoware::euclidean_cluster
{
/// @brief A set of labels that are easily confused with one another, plus the thresholds that
///        govern how aggressively their clusters may be merged across label boundaries.
struct ConfusableLabelGroup
{
  std::vector<std::uint8_t> labels;
  /// @brief True minimum point-to-point XY gap (m) below which two cross-label clusters may merge.
  float cross_label_tolerance{};
  /// @brief Maximum diameter (m) of a merged component's XY bounding circle; caps chaining into
  ///        oversized blobs. Orientation-independent — clusters are partial views, so a per-axis
  ///        (length/width) cap would presuppose a heading we cannot reliably estimate.
  float max_merged_size{25.0F};
};

/// @brief A clustered point cloud tagged with its object label and averaged semantic probability.
struct ClusterEntry
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::uint8_t label{};
  float prob{};
};

/// @brief Cross-label confusable merge: AABB broad-phase prune -> true-gap narrow phase ->
///        size-bounded greedy union-find (closest pairs first, bounding-circle diameter cap) to
///        prevent chaining. Same-label clusters are never merged here. Returns one entry per
///        surviving component; merged entries take the point-majority label and point-weighted
///        average probability.
std::vector<ClusterEntry> merge_confusable_clusters(
  std::vector<ClusterEntry> entries, const ConfusableLabelGroup & group);

}  // namespace autoware::euclidean_cluster
