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
using OmBox = bg::model::box<OmPoint>;
using OmValue = std::pair<OmBox, size_t>;  // (circle bounding box, index into the gate-circle list)

namespace
{

// Extra distance beyond geometric touching within which a pair still gates.
constexpr double gate_margin = 1.0;  // [m]

// Most circles a single tracker's cover may use.
constexpr int max_gate_circles = 8;

// One circle of a tracker's multi-circle cover: world-frame center, radius, and the snapshot it
// belongs to.
struct GateCircle
{
  double x;
  double y;
  double radius;
  size_t snapshot_idx;
};

// Tile a tracker's bounding box with equal circles along its longer axis, each circumscribing one
// segment of the split; a square-ish box yields a single circumscribing circle.
void appendGateCircles(
  const TrackerSnapshot & snap, const size_t snapshot_idx, std::vector<GateCircle> & out)
{
  const double length = std::max(snap.length, 0.0);
  const double width = std::max(snap.width, 0.0);
  const bool elongated_along_heading = length >= width;
  const double long_side = std::max(length, width);
  const double short_side = std::min(length, width);

  int circle_count = 1;
  if (short_side > 1e-3) {
    circle_count =
      std::clamp(static_cast<int>(std::ceil(long_side / short_side)), 1, max_gate_circles);
  }
  const double segment = long_side / circle_count;
  const double radius = 0.5 * std::hypot(short_side, segment);
  const double cos_yaw = std::cos(snap.yaw);
  const double sin_yaw = std::sin(snap.yaw);
  for (int k = 0; k < circle_count; ++k) {
    const double axis_offset = -0.5 * long_side + (k + 0.5) * segment;
    const double local_x = snap.local_center_x + (elongated_along_heading ? axis_offset : 0.0);
    const double local_y = snap.local_center_y + (elongated_along_heading ? 0.0 : axis_offset);
    out.push_back(
      GateCircle{
        snap.position.x + cos_yaw * local_x - sin_yaw * local_y,
        snap.position.y + sin_yaw * local_x + cos_yaw * local_y, radius, snapshot_idx});
  }
}

}  // namespace

std::vector<std::pair<size_t, size_t>> findCandidatePairs(
  const std::vector<TrackerSnapshot> & snapshots)
{
  std::vector<GateCircle> circles;
  circles.reserve(snapshots.size() * 2);
  for (size_t i = 0; i < snapshots.size(); ++i) {
    appendGateCircles(snapshots[i], i, circles);
  }
  if (circles.empty()) {
    return {};
  }

  const auto circle_box = [](const GateCircle & circle) {
    const double half = circle.radius + 0.5 * gate_margin;
    return OmBox(
      OmPoint(circle.x - half, circle.y - half), OmPoint(circle.x + half, circle.y + half));
  };

  std::vector<OmValue> rtree_entries;
  rtree_entries.reserve(circles.size());
  for (size_t ci = 0; ci < circles.size(); ++ci) {
    rtree_entries.emplace_back(circle_box(circles[ci]), ci);
  }
  // The range constructor bulk-loads the tree via the packing algorithm.
  const bgi::rtree<OmValue, bgi::quadratic<16>> rtree(rtree_entries.begin(), rtree_entries.end());

  std::vector<std::pair<size_t, size_t>> candidate_pairs;
  std::vector<OmValue> nearby;
  for (size_t ci = 0; ci < circles.size(); ++ci) {
    const GateCircle & circle = circles[ci];
    // The box predicate lets the R-tree prune subtrees; satisfies alone would scan linearly.
    nearby.clear();
    rtree.query(
      bgi::intersects(circle_box(circle)) && bgi::satisfies([&](const OmValue & v) {
        const GateCircle & other = circles[v.second];
        // Emit each undirected pair once, from the lower-index tracker; skip sibling circles.
        if (other.snapshot_idx <= circle.snapshot_idx) return false;
        const double dx = other.x - circle.x;
        const double dy = other.y - circle.y;
        const double gate = circle.radius + other.radius + gate_margin;
        return dx * dx + dy * dy <= gate * gate;
      }),
      std::back_inserter(nearby));
    for (const auto & [box, other_ci] : nearby) {
      candidate_pairs.emplace_back(circle.snapshot_idx, circles[other_ci].snapshot_idx);
    }
  }
  std::sort(candidate_pairs.begin(), candidate_pairs.end());
  candidate_pairs.erase(
    std::unique(candidate_pairs.begin(), candidate_pairs.end()), candidate_pairs.end());
  return candidate_pairs;
}

}  // namespace autoware::multi_object_tracker::detail
