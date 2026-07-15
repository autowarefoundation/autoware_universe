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

#include "autoware/multi_object_tracker/merger/detail/overlap_gate.hpp"

#include "autoware/multi_object_tracker/merger/detail/overlap_types.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker::detail
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using OmPoint = bg::model::point<double, 2, bg::cs::cartesian>;
using OmValue = std::pair<OmPoint, size_t>;  // (position, index into snapshots)

std::vector<std::pair<size_t, size_t>> findCandidatePairs(
  const std::vector<TrackerSnapshot> & snapshots, const TrackerOverlapManagerConfig & config)
{
  bgi::rtree<OmValue, bgi::quadratic<16>> rtree;
  {
    std::vector<OmValue> rtree_points;
    rtree_points.reserve(snapshots.size());
    for (size_t i = 0; i < snapshots.size(); ++i) {
      rtree_points.emplace_back(OmPoint(snapshots[i].position.x, snapshots[i].position.y), i);
    }
    rtree.insert(rtree_points.begin(), rtree_points.end());
  }

  std::vector<std::pair<size_t, size_t>> candidate_pairs;
  std::vector<OmValue> nearby;
  for (size_t i = 0; i < snapshots.size(); ++i) {
    const auto max_search_dist_sq_opt =
      get_map_value_if_exists(config.pruning_distance_thresholds_sq, snapshots[i].label);
    if (!max_search_dist_sq_opt) {
      continue;
    }
    const double max_search_dist_sq = max_search_dist_sq_opt->get();
    const double max_search_dist = std::sqrt(max_search_dist_sq);
    const double x = snapshots[i].position.x;
    const double y = snapshots[i].position.y;
    // The box predicate lets the R-tree prune subtrees; satisfies alone would scan linearly.
    const bg::model::box<OmPoint> search_box(
      OmPoint(x - max_search_dist, y - max_search_dist),
      OmPoint(x + max_search_dist, y + max_search_dist));

    nearby.clear();
    rtree.query(
      bgi::intersects(search_box) && bgi::satisfies([&](const OmValue & v) {
        if (v.second == i) return false;
        const double dx = bg::get<0>(v.first) - x;
        const double dy = bg::get<1>(v.first) - y;
        return dx * dx + dy * dy <= max_search_dist_sq;
      }),
      std::back_inserter(nearby));
    for (const auto & [point, j] : nearby) {
      candidate_pairs.emplace_back(std::min(i, j), std::max(i, j));
    }
  }
  std::sort(candidate_pairs.begin(), candidate_pairs.end());
  candidate_pairs.erase(
    std::unique(candidate_pairs.begin(), candidate_pairs.end()), candidate_pairs.end());
  return candidate_pairs;
}

}  // namespace autoware::multi_object_tracker::detail
