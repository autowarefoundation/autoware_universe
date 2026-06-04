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

#include "autoware/boundary_departure_checker/detail/uncrossable_boundaries_rtree.hpp"

#include "autoware/boundary_departure_checker/detail/conversion.hpp"

#include <algorithm>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker
{
namespace
{
std::vector<SegmentWithIdx> create_local_segments(const lanelet::ConstLineString3d & linestring)
{
  std::vector<SegmentWithIdx> local_segments;
  local_segments.reserve(linestring.size());
  const auto basic_ls = linestring.basicLineString();
  for (size_t i = 0; i + 1 < basic_ls.size(); ++i) {
    const auto segment = utils::to_segment_2d(basic_ls.at(i), basic_ls.at(i + 1));
    local_segments.emplace_back(
      boost::geometry::return_envelope<Segment2d>(segment),
      IdxForRTreeSegment(linestring.id(), i, i + 1));
  }
  return local_segments;
}
}  // namespace

UncrossableBoundariesRTree::UncrossableBoundariesRTree(
  lanelet::LaneletMapPtr lanelet_map_ptr, const std::vector<std::string> & boundary_types_to_detect)
: lanelet_map_ptr_(std::move(lanelet_map_ptr))
{
  if (!lanelet_map_ptr_) {
    throw std::runtime_error("UncrossableBoundariesRTree: Map is NULL");
  }

  std::vector<SegmentWithIdx> segments;
  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    if (!is_uncrossable_type(boundary_types_to_detect, linestring)) {
      continue;
    }

    auto local_segments = create_local_segments(linestring);
    std::move(local_segments.begin(), local_segments.end(), std::back_inserter(segments));
  }

  rtree_ =
    autoware::boundary_departure_checker::UncrossableBoundsRTree(segments.begin(), segments.end());
}

autoware_utils_geometry::Segment3d UncrossableBoundariesRTree::get_segment_3d_from_id(
  const IdxForRTreeSegment & seg_id) const
{
  const auto & linestring_layer = lanelet_map_ptr_->lineStringLayer;
  const auto basic_ls = linestring_layer.get(seg_id.linestring_id).basicLineString();

  auto p_start = autoware_utils_geometry::Point3d{
    basic_ls.at(seg_id.segment_start_idx).x(), basic_ls.at(seg_id.segment_start_idx).y(),
    basic_ls.at(seg_id.segment_start_idx).z()};

  auto p_end = autoware_utils_geometry::Point3d{
    basic_ls.at(seg_id.segment_end_idx).x(), basic_ls.at(seg_id.segment_end_idx).y(),
    basic_ls.at(seg_id.segment_end_idx).z()};

  return {p_start, p_end};
}

std::vector<SegmentWithIdx> UncrossableBoundariesRTree::query(
  const Point2d & point, size_t max_results) const
{
  std::vector<SegmentWithIdx> results;
  rtree_.query(
    boost::geometry::index::nearest(lanelet::BasicPoint2d{point.x(), point.y()}, max_results),
    std::back_inserter(results));
  return results;
}

bool UncrossableBoundariesRTree::is_uncrossable_type(
  const std::vector<std::string> & boundary_types_to_detect, const lanelet::ConstLineString3d & ls)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  return (
    type != no_type &&
    std::find(boundary_types_to_detect.begin(), boundary_types_to_detect.end(), type) !=
      boundary_types_to_detect.end());
}

}  // namespace autoware::boundary_departure_checker
