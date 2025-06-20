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

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <magic_enum.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <limits>
#include <string>
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

enum class AbnormalityType { NORMAL, LOCALIZATION, LONGITUDINAL, STEERING };

enum class SideKey { LEFT, RIGHT };
constexpr std::array<SideKey, 2> g_side_keys = {SideKey::LEFT, SideKey::RIGHT};

template <typename T>
struct Abnormalities
{
  T normal;
  T longitudinal;
  T localization;
  T steering;
  T & operator[](const AbnormalityType key)
  {
    if (key == AbnormalityType::NORMAL) return normal;
    if (key == AbnormalityType::LOCALIZATION) return localization;
    if (key == AbnormalityType::LONGITUDINAL) return longitudinal;
    if (key == AbnormalityType::STEERING) return steering;
    throw std::invalid_argument("Invalid key: " + std::string(magic_enum::enum_name(key)));
  }

  const T & operator[](const AbnormalityType key) const
  {
    if (key == AbnormalityType::NORMAL) return normal;
    if (key == AbnormalityType::LOCALIZATION) return localization;
    if (key == AbnormalityType::LONGITUDINAL) return longitudinal;
    if (key == AbnormalityType::STEERING) return steering;
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
  Point2d pt_on_ego;    // orig
  Point2d pt_on_bound;  // proj
  Segment2d nearest_bound_seg;
  double lat_dist{std::numeric_limits<double>::max()};
  size_t ego_sides_idx{0};
  ProjectionToBound() = default;
  explicit ProjectionToBound(size_t idx) : ego_sides_idx(idx) {}
  ProjectionToBound(
    Point2d pt_on_ego, Point2d pt_on_bound, Segment2d seg, double lat_dist, size_t idx)
  : pt_on_ego(std::move(pt_on_ego)),
    pt_on_bound(std::move(pt_on_bound)),
    nearest_bound_seg(std::move(seg)),
    lat_dist(lat_dist),
    ego_sides_idx(idx)
  {
  }
};

struct ClosestProjectionToBound : ProjectionToBound
{
  double lon_dist_on_ref_traj{std::numeric_limits<double>::max()};
  DepartureType departure_type = DepartureType::NONE;
  AbnormalityType abnormality_type = AbnormalityType::NORMAL;
  ClosestProjectionToBound() = default;
  explicit ClosestProjectionToBound(const ProjectionToBound & base)
  {
    pt_on_ego = base.pt_on_ego;
    pt_on_bound = base.pt_on_bound;
    nearest_bound_seg = base.nearest_bound_seg;
    lat_dist = base.lat_dist;
    ego_sides_idx = base.ego_sides_idx;
  }

  ClosestProjectionToBound(
    const ProjectionToBound & base, const double lon_dist, const AbnormalityType abnormality_type,
    const DepartureType departure_type)
  : lon_dist_on_ref_traj(lon_dist),
    departure_type(departure_type),
    abnormality_type(abnormality_type)
  {
    pt_on_ego = base.pt_on_ego;
    pt_on_bound = base.pt_on_bound;
    nearest_bound_seg = base.nearest_bound_seg;
    lat_dist = base.lat_dist;
    ego_sides_idx = base.ego_sides_idx;
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

using SegmentWithIdx = std::pair<Segment2d, IdxForRTreeSegment>;
using UncrossableBoundRTree = boost::geometry::index::rtree<SegmentWithIdx, bgi::rstar<16>>;
using BoundarySideWithIdx = Side<std::vector<SegmentWithIdx>>;
using ProjectionsToBound = Side<std::vector<ProjectionToBound>>;
using ClosestProjectionsToBound = Side<std::vector<ClosestProjectionToBound>>;
using EgoSide = Side<Segment2d>;
using EgoSides = std::vector<EgoSide>;

struct DeparturePoint
{
  std::string uuid;
  DepartureType departure_type{DepartureType::NONE};
  AbnormalityType abnormality_type{AbnormalityType::NORMAL};
  Point2d point;
  double th_dist_hysteresis{2.0};
  double lat_dist_to_bound{1000.0};
  double dist_on_traj{1000.0};
  double velocity{0.0};
  size_t idx_from_ego_traj{};
  bool can_be_removed{false};

  [[nodiscard]] bool is_nearby(const Pose & pose) const { return is_nearby(pose.position); }

  [[nodiscard]] bool is_nearby(const Point & point) const { return is_nearby({point.x, point.y}); }

  [[nodiscard]] bool is_nearby(const Point2d & candidate_point) const
  {
    const auto diff = boost::geometry::distance(point, candidate_point);
    return diff < th_dist_hysteresis;
  }

  [[nodiscard]] Point to_geom_pt(const double z = 0.0) const
  {
    return autoware_utils::to_msg(point.to_3d(z));
  }

  bool operator<(const DeparturePoint & other) const { return dist_on_traj < other.dist_on_traj; }
};
using DeparturePoints = std::vector<DeparturePoint>;

struct CriticalDeparturePoint : DeparturePoint
{
  TrajectoryPoint point_on_prev_traj;
  CriticalDeparturePoint() = default;
  explicit CriticalDeparturePoint(const DeparturePoint & base)
  {
    uuid = base.uuid;
    departure_type = base.departure_type;
    abnormality_type = base.abnormality_type;
    point = base.point;
    th_dist_hysteresis = base.th_dist_hysteresis;
    lat_dist_to_bound = base.lat_dist_to_bound;
    dist_on_traj = base.dist_on_traj;
    velocity = base.velocity;
    can_be_removed = base.can_be_removed;
  }
};

using CriticalDeparturePoints = std::vector<CriticalDeparturePoint>;

struct DepartureInterval
{
  TrajectoryPoint start;
  TrajectoryPoint end;
  SideKey side_key;
  double start_dist_on_traj;
  double end_dist_on_traj;

  bool start_at_traj_front{false};
  bool has_merged{false};

  DeparturePoints candidates;
};
using DepartureIntervals = std::vector<DepartureInterval>;

using Footprint = LinearRing2d;
using Footprints = std::vector<Footprint>;

struct AbnormalitiesData
{
  Abnormalities<EgoSides> footprints_sides;
  Abnormalities<Footprints> footprints;
  BoundarySideWithIdx boundary_segments;
  Abnormalities<ProjectionsToBound> projections_to_bound;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_
