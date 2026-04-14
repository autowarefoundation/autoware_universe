// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_

#include "autoware/boundary_departure_checker/footprints_generator.hpp"
#include "autoware/boundary_departure_checker/side_struct.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <magic_enum.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <boost/functional/hash.hpp>
#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <limits>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker
{

enum class DepartureType {
  NONE = 0,
  APPROACHING,
  CRITICAL,
};

struct ProjectionToBound
{
  DepartureType departure_type{DepartureType::NONE};

  Point2d pt_on_ego;    // orig
  Point2d pt_on_bound;  // proj
  double dist_along_trajectory_m{std::numeric_limits<double>::max()};
  Segment2d nearest_bound_seg;
  double lat_dist{std::numeric_limits<double>::max()};
  double
    ego_front_to_proj_offset_m{};  // offset between the pt_on_ego and the front of the ego segment
  size_t pose_index{0};
  double time_from_start{std::numeric_limits<double>::max()};
  ProjectionToBound() = default;
  explicit ProjectionToBound(size_t idx) : pose_index(idx) {}
  ProjectionToBound(
    Point2d pt_on_ego, Point2d pt_on_bound, Segment2d seg, double lat_dist,
    double ego_front_to_proj_offset_m, size_t idx)
  : pt_on_ego(std::move(pt_on_ego)),
    pt_on_bound(std::move(pt_on_bound)),
    nearest_bound_seg(std::move(seg)),
    lat_dist(lat_dist),
    ego_front_to_proj_offset_m(ego_front_to_proj_offset_m),
    pose_index(idx)
  {
  }

  [[nodiscard]] bool is_none_departure() const { return departure_type == DepartureType::NONE; }

  [[nodiscard]] bool is_approaching() const { return departure_type == DepartureType::APPROACHING; }

  [[nodiscard]] bool is_critical() const { return departure_type == DepartureType::CRITICAL; }
};
using ProjectionsToBound = std::vector<ProjectionToBound>;

struct CriticalPointPair
{
  ProjectionToBound physical_departure_point;
  ProjectionToBound safety_buffer_start;
};

using BoundarySide = Side<std::vector<Segment2d>>;

struct IdxForRTreeSegment
{
  lanelet::Id linestring_id{lanelet::InvalId};
  size_t segment_start_idx{std::numeric_limits<size_t>::max()};
  size_t segment_end_idx{std::numeric_limits<size_t>::max()};

  IdxForRTreeSegment() = default;
  IdxForRTreeSegment(lanelet::Id linestring_id, size_t segment_start_idx, size_t segment_end_idx)
  : linestring_id(linestring_id),
    segment_start_idx(segment_start_idx),
    segment_end_idx(segment_end_idx)
  {
  }
  /* compare only the identifiers and indices */
  [[nodiscard]] constexpr bool operator==(const IdxForRTreeSegment & rhs) const noexcept
  {
    return linestring_id == rhs.linestring_id && segment_start_idx == rhs.segment_start_idx &&
           segment_end_idx == rhs.segment_end_idx;
  }

  [[nodiscard]] constexpr bool operator!=(const IdxForRTreeSegment & rhs) const noexcept
  {
    return !(*this == rhs);
  }
};

struct IdxForRTreeSegmentHash
{
  size_t operator()(const IdxForRTreeSegment & s) const noexcept
  {
    size_t seed = 0;
    // Boost hash_combine is a good choice for combining hashes
    boost::hash_combine(seed, s.linestring_id);
    boost::hash_combine(seed, s.segment_start_idx);
    boost::hash_combine(seed, s.segment_end_idx);
    return seed;
  }
};

using SegmentWithIdx = std::pair<Segment2d, IdxForRTreeSegment>;
using UncrossableBoundsRTree = boost::geometry::index::rtree<SegmentWithIdx, bgi::rstar<16>>;
using BoundarySegmentsBySide = Side<std::vector<SegmentWithIdx>>;
using FootprintSideSegments = Side<Segment2d>;
using FootprintSideSegmentsArray = std::vector<FootprintSideSegments>;

using ProjectionsToBound = std::vector<ProjectionToBound>;

struct DepartureData
{
  footprints::Footprints footprints;
  FootprintSideSegmentsArray footprints_sides;
  BoundarySegmentsBySide boundary_segments;

  Side<ProjectionsToBound> projections_to_bound;
  Side<std::optional<CriticalPointPair>> evaluated_projections;
  DepartureType status{DepartureType::NONE};
};

struct EgoDynamicState
{
  geometry_msgs::msg::PoseWithCovariance pose_with_cov;
  double velocity{0.0};
  double acceleration{0.0};
  double current_time_s{0.0};
};

struct DepartureCheckThresholds
{
  double min_braking_distance{0.0};
  double cutoff_time{0.0};
  double th_lat_critical{0.0};
};

struct ProjectionEvaluationMetrics
{
  double lon_dist_to_departure{0.0};
  double time_from_start{0.0};
  double lat_dist{0.0};
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_
