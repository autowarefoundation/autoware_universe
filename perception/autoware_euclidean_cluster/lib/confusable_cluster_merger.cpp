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

#include "autoware/euclidean_cluster/confusable_cluster_merger.hpp"

#include <Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::euclidean_cluster
{
namespace
{
struct Aabb2d
{
  float xmin{}, xmax{}, ymin{}, ymax{};
};

Aabb2d compute_aabb2d(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  Eigen::Vector4f mn, mx;
  pcl::getMinMax3D(cloud, mn, mx);
  return {mn.x(), mx.x(), mn.y(), mx.y()};
}

/// @brief XY gap between two axis-aligned boxes. This is a lower bound on the true point-to-point
///        gap (both boxes contain their points), so it is safe to use as a broad-phase prune.
float aabb2d_gap(const Aabb2d & a, const Aabb2d & b)
{
  const float dx = std::max(0.0f, std::max(a.xmin, b.xmin) - std::min(a.xmax, b.xmax));
  const float dy = std::max(0.0f, std::max(a.ymin, b.ymin) - std::min(a.ymax, b.ymax));
  return std::sqrt(dx * dx + dy * dy);
}

/// @brief 2D bounding circle (XY) of a cluster or merged component. Orientation-independent size
///        proxy: a single diameter, so the merge cap does not assume a (partial-view) heading.
struct BoundingCircle
{
  Eigen::Vector2f center{Eigen::Vector2f::Zero()};
  float radius{};        ///< max XY distance from center to any member point (m)
  std::size_t count{0};  ///< point count (centroid weight for incremental merging)
};

/// @brief Bounding circle of a single cluster: centroid plus its farthest-point radius (one pass).
BoundingCircle compute_bounding_circle(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  BoundingCircle bc;
  const auto n = cloud.size();
  if (n == 0) {
    return bc;
  }

  Eigen::Vector2f mean = Eigen::Vector2f::Zero();
  for (const auto & p : cloud) {
    mean += Eigen::Vector2f(p.x, p.y);
  }
  mean /= static_cast<float>(n);

  float max_sq = 0.0F;
  for (const auto & p : cloud) {
    max_sq = std::max(max_sq, (Eigen::Vector2f(p.x, p.y) - mean).squaredNorm());
  }

  bc.center = mean;
  bc.radius = std::sqrt(max_sq);
  bc.count = n;
  return bc;
}

/// @brief Conservative union of two XY bounding circles: point-weighted center and a
///        triangle-inequality radius bound. O(1) — keeps the chaining size-check off the per-edge
///        point scan, so component growth is bounded without re-measuring all member points.
BoundingCircle merge_circles(const BoundingCircle & a, const BoundingCircle & b)
{
  if (a.count == 0) {
    return b;
  }
  if (b.count == 0) {
    return a;
  }
  BoundingCircle m;
  m.count = a.count + b.count;
  m.center = (a.center * static_cast<float>(a.count) + b.center * static_cast<float>(b.count)) /
             static_cast<float>(m.count);
  m.radius =
    std::max((m.center - a.center).norm() + a.radius, (m.center - b.center).norm() + b.radius);
  return m;
}

/// @brief XY-flattened (z = 0) copy of a cluster, so planar gap searches ignore height.
pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_xy(const pcl::PointCloud<pcl::PointXYZ> & in)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  out->reserve(in.size());
  for (const auto & p : in) {
    out->push_back(pcl::PointXYZ(p.x, p.y, 0.0F));
  }
  return out;
}

/// @brief Exact minimum point-to-point XY distance between a query cloud and a prebuilt KD-tree.
///        Querying the smaller cloud against the larger cloud's tree keeps the search cheap; the
///        tree is built once per cluster by the caller and reused across all its candidate pairs.
float min_gap_xy(
  const pcl::PointCloud<pcl::PointXYZ> & query, const pcl::KdTreeFLANN<pcl::PointXYZ> & tree)
{
  float min_sq = std::numeric_limits<float>::max();
  std::vector<int> idx(1);
  std::vector<float> dist_sq(1);
  for (const auto & q : query) {
    if (tree.nearestKSearch(q, 1, idx, dist_sq) > 0) {
      min_sq = std::min(min_sq, dist_sq[0]);
    }
  }
  return std::sqrt(min_sq);
}
}  // namespace

std::vector<ClusterEntry> merge_confusable_clusters(
  std::vector<ClusterEntry> entries, const ConfusableLabelGroup & group)
{
  const auto n = entries.size();
  if (n <= 1) {
    return entries;
  }

  // Per-cluster broad-phase AABBs and bounding circles, computed once. The flattened (XY) clouds
  // and their KD-trees back the narrow phase and are built lazily, then reused across every
  // candidate pair a cluster takes part in — instead of rebuilt per pair as before.
  std::vector<Aabb2d> aabbs;
  std::vector<BoundingCircle> circles;
  aabbs.reserve(n);
  circles.reserve(n);
  for (const auto & e : entries) {
    aabbs.push_back(compute_aabb2d(e.cloud));
    circles.push_back(compute_bounding_circle(e.cloud));
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> flat(n);
  std::vector<std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>>> trees(n);
  const auto flat_of = [&](size_t i) {
    if (!flat[i]) {
      flat[i] = flatten_xy(entries[i].cloud);
    }
    return flat[i];
  };
  const auto tree_of = [&](size_t i) {
    if (!trees[i]) {
      auto t = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
      t->setInputCloud(flat_of(i));
      trees[i] = t;
    }
    return trees[i];
  };

  // Phase 1 (broad) + Phase 2 (narrow): collect surviving candidate edges with their true gap.
  struct Edge
  {
    float gap{};
    int i{};
    int j{};
  };
  std::vector<Edge> edges;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      if (entries[i].label == entries[j].label) {
        continue;
      }
      // Broad phase: AABB gap is a lower bound, so this prune never drops a true-adjacent pair.
      if (aabb2d_gap(aabbs[i], aabbs[j]) >= group.cross_label_tolerance) {
        continue;
      }
      // Narrow phase: exact min point-to-point XY gap rejects the false positives AABB admits.
      // Query the smaller cloud against the larger cloud's (reusable) tree to keep the search
      // cheap.
      const bool i_smaller = entries[i].cloud.size() <= entries[j].cloud.size();
      const size_t q = i_smaller ? i : j;
      const size_t t = i_smaller ? j : i;
      if (flat_of(q)->empty() || flat_of(t)->empty()) {
        continue;
      }
      const float gap = min_gap_xy(*flat_of(q), *tree_of(t));
      if (gap >= group.cross_label_tolerance) {
        continue;
      }
      edges.push_back({gap, static_cast<int>(i), static_cast<int>(j)});
    }
  }

  // Phase 3: size-bounded greedy union-find. Merge the closest pairs first and refuse any union
  // whose merged bounding-circle diameter would exceed the size cap — this bounds component growth
  // so a chain of adjacencies cannot collapse a dense scene into one oversized blob. The circle is
  // tracked incrementally (per-root, O(1) per union), so no per-edge scan over member points.
  std::sort(
    edges.begin(), edges.end(), [](const Edge & a, const Edge & b) { return a.gap < b.gap; });

  std::vector<int> parent(n);
  std::iota(parent.begin(), parent.end(), 0);
  auto find = [&](int x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  };

  // Per-root merged bounding circle; valid only at roots (seeded from the per-cluster circles).
  std::vector<BoundingCircle> root_circle = circles;

  std::vector<std::vector<int>> members(n);
  for (int i = 0; i < static_cast<int>(n); ++i) {
    members[i] = {i};
  }

  for (const auto & e : edges) {
    const int ri = find(e.i);
    const int rj = find(e.j);
    if (ri == rj) {
      continue;
    }
    const BoundingCircle merged = merge_circles(root_circle[ri], root_circle[rj]);
    if (2.0F * merged.radius > group.max_merged_size) {
      continue;  // would create an implausibly large object — keep them separate
    }
    parent[ri] = rj;
    root_circle[rj] = merged;
    members[rj].insert(members[rj].end(), members[ri].begin(), members[ri].end());
    members[ri].clear();
  }

  std::unordered_map<int, std::vector<int>> groups;
  for (int i = 0; i < static_cast<int>(n); ++i) {
    groups[find(i)].push_back(i);
  }

  std::vector<ClusterEntry> result;
  result.reserve(groups.size());
  for (auto & [root, indices] : groups) {
    if (indices.size() == 1) {
      result.push_back(std::move(entries[indices[0]]));
      continue;
    }
    ClusterEntry merged;
    std::unordered_map<std::uint8_t, size_t> label_counts;
    size_t total_points = 0;
    float weighted_prob = 0.0f;
    for (const int idx : indices) {
      const auto & e = entries[idx];
      const auto npts = e.cloud.size();
      merged.cloud += e.cloud;
      label_counts[e.label] += npts;
      weighted_prob += e.prob * static_cast<float>(npts);
      total_points += npts;
    }
    merged.label = std::max_element(
                     label_counts.begin(), label_counts.end(),
                     [](const auto & a, const auto & b) { return a.second < b.second; })
                     ->first;
    merged.prob = total_points > 0 ? weighted_prob / static_cast<float>(total_points) : 0.0f;
    result.push_back(std::move(merged));
  }
  return result;
}

}  // namespace autoware::euclidean_cluster
