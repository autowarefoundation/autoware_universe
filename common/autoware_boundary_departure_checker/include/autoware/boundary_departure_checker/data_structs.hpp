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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_

#include "autoware/boundary_departure_checker/side_struct.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <magic_enum.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/functional/hash.hpp>
#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <limits>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker
{

/**
 * @brief Type of boundary departure.
 */
enum class DepartureType {
  NONE = 0,     ///< no departure
  APPROACHING,  ///< approaching the boundary
  CRITICAL,     ///< critical departure from the boundary
};

/**
 * @brief Information about a projection to a boundary.
 */
struct ProjectionToBound
{
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

  /**
   * @brief Check if the departure type is NONE.
   * @return true if NONE
   */
  [[nodiscard]] bool is_none_departure() const { return departure_type == DepartureType::NONE; }

  /**
   * @brief Check if the departure type is APPROACHING.
   * @return true if APPROACHING
   */
  [[nodiscard]] bool is_approaching() const { return departure_type == DepartureType::APPROACHING; }

  /**
   * @brief Check if the departure type is CRITICAL.
   * @return true if CRITICAL
   */
  [[nodiscard]] bool is_critical() const { return departure_type == DepartureType::CRITICAL; }

  DepartureType departure_type{DepartureType::NONE};  ///< type of departure
  Point2d pt_on_ego;                                  ///< point on the ego footprint
  Point2d pt_on_bound;                                ///< point on the boundary
  double dist_along_trajectory_m{
    std::numeric_limits<double>::max()};  ///< longitudinal distance along trajectory [m]
  Segment2d nearest_bound_seg;            ///< nearest boundary segment
  double lat_dist{std::numeric_limits<double>::max()};  ///< lateral distance to the boundary [m]
  double ego_front_to_proj_offset_m{};  ///< offset between pt_on_ego and front of ego segment [m]
  size_t pose_index{0};                 ///< index of the pose
  double time_from_start{
    std::numeric_limits<double>::max()};  ///< time from the start of the trajectory [s]
};
using ProjectionsToBound = std::vector<ProjectionToBound>;

/**
 * @brief A pair of critical points: physical departure point and safety buffer start.
 */
struct CriticalPointPair
{
  ProjectionToBound physical_departure_point;  ///< physical departure point
  ProjectionToBound safety_buffer_start;       ///< point where safety buffer starts
};

using BoundarySide = Side<std::vector<Segment2d>>;

/**
 * @brief Indices for a segment in the R-tree.
 */
struct IdxForRTreeSegment
{
  IdxForRTreeSegment() = default;
  IdxForRTreeSegment(lanelet::Id linestring_id, size_t segment_start_idx, size_t segment_end_idx)
  : linestring_id(linestring_id),
    segment_start_idx(segment_start_idx),
    segment_end_idx(segment_end_idx)
  {
  }

  /**
   * @brief Equality operator.
   * @param[in] rhs other segment index
   * @return true if equal
   */
  [[nodiscard]] constexpr bool operator==(const IdxForRTreeSegment & rhs) const noexcept
  {
    return linestring_id == rhs.linestring_id && segment_start_idx == rhs.segment_start_idx &&
           segment_end_idx == rhs.segment_end_idx;
  }

  /**
   * @brief Inequality operator.
   * @param[in] rhs other segment index
   * @return true if not equal
   */
  [[nodiscard]] constexpr bool operator!=(const IdxForRTreeSegment & rhs) const noexcept
  {
    return !(*this == rhs);
  }

  lanelet::Id linestring_id{lanelet::InvalId};                   ///< ID of the linestring
  size_t segment_start_idx{std::numeric_limits<size_t>::max()};  ///< start index of the segment
  size_t segment_end_idx{std::numeric_limits<size_t>::max()};    ///< end index of the segment
};

/**
 * @brief Hash function for IdxForRTreeSegment.
 */
struct IdxForRTreeSegmentHash
{
  /**
   * @brief Calculate hash.
   * @param[in] s segment index
   * @return hash value
   */
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

/**
 * @brief Result of boundary departure check.
 */
struct DepartureResult
{
  DepartureType status{DepartureType::NONE};           ///< current departure status
  visualization_msgs::msg::MarkerArray debug_markers;  ///< debug markers for visualization
};

/**
 * @brief Dynamic state of the ego vehicle.
 */
struct EgoDynamicState
{
  geometry_msgs::msg::PoseWithCovariance pose_with_cov;  ///< pose with covariance
  double velocity{0.0};                                  ///< velocity [m/s]
  double acceleration{0.0};                              ///< acceleration [m/s^2]
  double current_time_s{0.0};                            ///< current time [s]
};

/**
 * @brief Thresholds for boundary departure check.
 */
struct DepartureCheckThresholds
{
  double min_braking_distance{0.0};  ///< minimum braking distance [m]
  double cutoff_time{0.0};           ///< time cutoff for departure check [s]
  double th_lat_critical{0.0};       ///< lateral distance threshold for critical departure [m]
};

/**
 * @brief Metrics for evaluating a projection.
 */
struct ProjectionEvaluationMetrics
{
  double lon_dist_to_departure{0.0};  ///< longitudinal distance to departure [m]
  double time_from_start{0.0};        ///< time from the start of the trajectory [s]
  double lat_dist{0.0};               ///< lateral distance to the boundary [m]
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_
