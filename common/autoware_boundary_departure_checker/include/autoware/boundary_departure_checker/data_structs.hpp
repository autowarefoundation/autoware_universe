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
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker
{
enum class DepartureType {
  NONE = 0,
  NEAR_BOUNDARY,
  APPROACHING_DEPARTURE,
  CRITICAL_DEPARTURE,
};

enum class FootprintType {
  NORMAL = 0,
  LOCALIZATION,
  LONGITUDINAL,
  STEERING_STUCK,
  STEERING_ACCELERATED,
  STEERING_SUDDEN_LEFT,
  STEERING_SUDDEN_RIGHT
};

enum class SideKey { LEFT, RIGHT };
constexpr std::array<SideKey, 2> g_side_keys = {SideKey::LEFT, SideKey::RIGHT};

template <typename T>
struct FootprintMap
{
  std::unordered_map<FootprintType, T> data;
  T & operator[](const FootprintType key) { return data[key]; }

  const T & operator[](const FootprintType key) const
  {
    const auto it = data.find(key);
    if (it != data.end()) {
      return it->second;
    }
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }
};

template <typename T>
struct Side
{
  T right;
  T left;
  T & operator[](const SideKey key)
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  const T & operator[](const SideKey key) const
  {
    if (key == SideKey::LEFT) return left;
    if (key == SideKey::RIGHT) return right;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }
};

struct ProjectionToBound
{
  std::optional<DepartureType> departure_type_opt;
  std::optional<FootprintType> footprint_type_opt;

  Point2d pt_on_ego;    // orig
  Point2d pt_on_bound;  // proj
  double lon_dist_on_pred_traj{std::numeric_limits<double>::max()};
  Segment2d nearest_bound_seg;
  double lat_dist{std::numeric_limits<double>::max()};
  double lon_offset{};  // offset between the pt_on_ego and the front of the ego segment
  size_t ego_sides_idx{0};
  double time_from_start{std::numeric_limits<double>::max()};
  ProjectionToBound() = default;
  explicit ProjectionToBound(size_t idx) : ego_sides_idx(idx) {}
  ProjectionToBound(
    Point2d pt_on_ego, Point2d pt_on_bound, Segment2d seg, double lat_dist, double lon_offset,
    size_t idx)
  : pt_on_ego(std::move(pt_on_ego)),
    pt_on_bound(std::move(pt_on_bound)),
    nearest_bound_seg(std::move(seg)),
    lat_dist(lat_dist),
    lon_offset(lon_offset),
    ego_sides_idx(idx)
  {
  }

  [[nodiscard]] bool is_near_boundary() const
  {
    return departure_type_opt && departure_type_opt.value() == DepartureType::NEAR_BOUNDARY;
  }

  [[nodiscard]] bool is_critical_departure() const
  {
    return departure_type_opt && departure_type_opt.value() == DepartureType::CRITICAL_DEPARTURE;
  }
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
using UncrossableBoundRTree = boost::geometry::index::rtree<SegmentWithIdx, bgi::rstar<16>>;
using BoundarySideWithIdx = Side<std::vector<SegmentWithIdx>>;
using EgoSide = Side<Segment2d>;
using EgoSides = std::vector<EgoSide>;

struct DeparturePoint
{
  DepartureType departure_type{DepartureType::NONE};
  FootprintType footprint_type{FootprintType::NORMAL};
  Point2d point;
  double th_point_merge_distance_m{2.0};
  double lat_dist_to_bound{1000.0};
  double ego_dist_on_ref_traj{1000.0};  // [m] distance along the reference trajectory of the
                                        // corresponding predicted trajectory point
  double velocity{0.0};
  size_t idx_from_ego_traj{};
  bool can_be_removed{false};
  geometry_msgs::msg::Pose pose_on_current_ref_traj;

  [[nodiscard]] Point to_geom_pt(const double z = 0.0) const
  {
    return autoware_utils_geometry::to_msg(point.to_3d(z));
  }

  bool operator<(const DeparturePoint & other) const
  {
    return ego_dist_on_ref_traj < other.ego_dist_on_ref_traj;
  }
};
using DeparturePoints = std::vector<DeparturePoint>;

using Footprint = LinearRing2d;
using Footprints = std::vector<Footprint>;

using ProjectionsToBound = std::vector<ProjectionToBound>;

struct DepartureData
{
  FootprintMap<Footprints> footprints;
  FootprintMap<EgoSides> footprints_sides;
  FootprintMap<Side<ProjectionsToBound>> projections_to_bound;

  BoundarySideWithIdx boundary_segments;
  Side<ProjectionsToBound> closest_projections_to_bound;
  Side<DeparturePoints> departure_points;
  DeparturePoints critical_departure_points;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_
