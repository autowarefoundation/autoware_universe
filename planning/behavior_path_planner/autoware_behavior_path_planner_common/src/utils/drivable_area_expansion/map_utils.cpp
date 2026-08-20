// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
SegmentRtree extract_uncrossable_segments(
  const lanelet::LaneletMap & lanelet_map, const Point & ego_point,
  const DrivableAreaExpansionParameters & params)
{
  SegmentRtree uncrossable_segments_in_range;
  LineString2d line;
  const auto ego_p = Point2d{ego_point.x, ego_point.y};
  for (const auto & ls : lanelet_map.lineStringLayer) {
    if (has_types(ls, params.avoid_linestring_types)) {
      line.clear();
      for (const auto & p : ls) line.push_back(Point2d{p.x(), p.y()});
      for (auto segment_idx = 0LU; segment_idx + 1 < line.size(); ++segment_idx) {
        Segment2d segment = {line[segment_idx], line[segment_idx + 1]};
        if (boost::geometry::distance(segment, ego_p) < params.max_path_arc_length) {
          uncrossable_segments_in_range.insert(segment);
        }
      }
    }
  }
  return uncrossable_segments_in_range;
}

bool has_types(
  const lanelet::ConstLineString3d & ls,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  const auto subtype = ls.attributeOr(lanelet::AttributeName::Subtype, no_type);
  const auto matches_type = [&](const auto & t) { return t.matches(type, subtype); };
  return (type != no_type && std::find_if(types.begin(), types.end(), matches_type) != types.end());
}

SegmentRtree extract_uncrossable_segments(
  const lanelet::LaneletMap & lanelet_map, const lanelet::BoundingBox2d & search_box,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types)
{
  if (types.empty()) {
    return {};
  }
  const autoware_utils::Box2d box{
    {search_box.min().x(), search_box.min().y()}, {search_box.max().x(), search_box.max().y()}};
  std::vector<Segment2d> segments;
  for (const auto & ls : lanelet_map.lineStringLayer.search(search_box)) {
    if (!has_types(ls, types)) {
      continue;
    }
    // only keep the segments overlapping the box, so that a linestring much longer than the box
    // does not inflate the rtree
    for (auto i = 0UL; i + 1 < ls.size(); ++i) {
      const Segment2d segment{{ls[i].x(), ls[i].y()}, {ls[i + 1].x(), ls[i + 1].y()}};
      if (boost::geometry::intersects(segment, box)) {
        segments.push_back(segment);
      }
    }
  }
  return SegmentRtree(segments);  // packing algorithm
}

bool is_separated_by_uncrossable_segments(
  const SegmentRtree & uncrossable_segments, const Point & from, const PredictedObject & object)
{
  const auto footprint = autoware_utils::to_polygon2d(object);
  const auto & points = footprint.outer();
  // without any uncrossable segment, no object can be separated
  if (uncrossable_segments.empty() || points.empty()) {
    return false;
  }
  const auto is_blocked = [&](const Segment2d & segment) {
    return uncrossable_segments.qbegin(boost::geometry::index::intersects(segment)) !=
           uncrossable_segments.qend();
  };
  const Point2d from_point{from.x, from.y};
  // the object can reach the given point if any point of its footprint is in line of sight
  const auto is_footprint_separated = std::all_of(
    points.begin(), points.end(),
    [&](const Point2d & p) { return is_blocked(Segment2d{from_point, p}); });
  if (!is_footprint_separated) {
    return false;
  }
  // an object overlapping the boundary may already be on the other side of it
  if (
    uncrossable_segments.qbegin(boost::geometry::index::intersects(footprint)) !=
    uncrossable_segments.qend()) {
    return false;
  }
  // the object may still reach the given point by following its predicted path around the boundary
  // or through one of its gaps. the path is cut where it crosses the boundary since the object is
  // assumed to stop there
  const auto & predicted_paths = object.kinematics.predicted_paths;
  const auto most_likely_path = std::max_element(
    predicted_paths.begin(), predicted_paths.end(),
    [](const auto & p1, const auto & p2) { return p1.confidence < p2.confidence; });
  if (most_likely_path == predicted_paths.end()) {
    return true;
  }
  const auto & initial_position = object.kinematics.initial_pose_with_covariance.pose.position;
  Point2d prev_point{initial_position.x, initial_position.y};
  for (const auto & pose : most_likely_path->path) {
    const Point2d point{pose.position.x, pose.position.y};
    if (is_blocked(Segment2d{prev_point, point})) {
      break;  // the object cannot cross the boundary and stops there
    }
    if (!is_blocked(Segment2d{from_point, point})) {
      return false;  // the object can reach a pose with a clear line of sight to the given point
    }
    prev_point = point;
  }
  return true;
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
