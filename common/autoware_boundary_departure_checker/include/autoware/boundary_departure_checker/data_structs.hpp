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

/**
 * @brief Severity level of a boundary departure.
 */
enum class DepartureType {
  NONE = 0,     ///< No departure detected.
  APPROACHING,  ///< Vehicle is approaching a boundary.
  CRITICAL,     ///< Vehicle has crossed or is about to cross a boundary.
};

/**
 * @brief Data structure representing a projection of an ego point onto a boundary.
 */
struct ProjectionToBound
{
  ProjectionToBound() = default;

  /**
   * @brief Constructor with pose index.
   * @param[in] idx Index of the pose in the trajectory.
   */
  explicit ProjectionToBound(size_t idx) : pose_index(idx) {}

  /**
   * @brief Parameterized constructor.
   * @param[in] pt_on_ego Point on the ego vehicle footprint.
   * @param[in] pt_on_bound Projected point on the boundary.
   * @param[in] seg The boundary segment the point was projected onto.
   * @param[in] lat_dist Signed lateral distance to the boundary [m].
   * @param[in] ego_front_to_proj_offset_m Offset from the ego segment front to the projection [m].
   * @param[in] idx Index of the pose in the trajectory.
   */
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
   * @return True if no departure.
   */
  [[nodiscard]] bool is_none_departure() const { return departure_type == DepartureType::NONE; }

  /**
   * @brief Check if the departure type is APPROACHING.
   * @return True if approaching.
   */
  [[nodiscard]] bool is_approaching() const { return departure_type == DepartureType::APPROACHING; }

  /**
   * @brief Check if the departure type is CRITICAL.
   * @return True if critical.
   */
  [[nodiscard]] bool is_critical() const { return departure_type == DepartureType::CRITICAL; }

  // Member Variables
  /**
   * @brief Type of departure detected.
   */
  DepartureType departure_type{DepartureType::NONE};

  /**
   * @brief Point on the ego vehicle footprint.
   */
  Point2d pt_on_ego;

  /**
   * @brief Projected point on the boundary.
   */
  Point2d pt_on_bound;

  /**
   * @brief Cumulative distance along the trajectory to this point [m].
   */
  double dist_along_trajectory_m{std::numeric_limits<double>::max()};

  /**
   * @brief The boundary segment nearest to the ego point.
   */
  Segment2d nearest_bound_seg;

  /**
   * @brief Signed lateral distance to the boundary [m].
   */
  double lat_dist{std::numeric_limits<double>::max()};

  /**
   * @brief Offset between pt_on_ego and the front of the ego segment [m].
   */
  double ego_front_to_proj_offset_m{};

  /**
   * @brief Index of the corresponding pose in the trajectory.
   */
  size_t pose_index{0};

  /**
   * @brief Estimated time from the start of the trajectory to this point [s].
   */
  double time_from_start{std::numeric_limits<double>::max()};
};

/**
 * @brief Collection of projections to boundaries.
 */
using ProjectionsToBound = std::vector<ProjectionToBound>;

/**
 * @brief Pair of points representing a physical departure and its safety buffer.
 */
struct CriticalPointPair
{
  ProjectionToBound physical_departure_point;  ///< Point where the vehicle actually crosses.
  ProjectionToBound safety_buffer_start;       ///< Point where the safety buffer starts.
};

using BoundarySide = Side<std::vector<Segment2d>>;

/**
 * @brief Structure for uniquely identifying a segment in an R-tree.
 */
struct IdxForRTreeSegment
{
public:
  IdxForRTreeSegment() = default;

  /**
   * @brief Parameterized constructor.
   * @param[in] linestring_id ID of the linestring containing the segment.
   * @param[in] segment_start_idx Start index of the segment in the linestring.
   * @param[in] segment_end_idx End index of the segment in the linestring.
   */
  IdxForRTreeSegment(lanelet::Id linestring_id, size_t segment_start_idx, size_t segment_end_idx)
  : linestring_id(linestring_id),
    segment_start_idx(segment_start_idx),
    segment_end_idx(segment_end_idx)
  {
  }

  /**
   * @brief Equality operator.
   * @param[in] rhs Other IdxForRTreeSegment to compare with.
   * @return True if both have the same ID and indices.
   */
  [[nodiscard]] constexpr bool operator==(const IdxForRTreeSegment & rhs) const noexcept
  {
    return linestring_id == rhs.linestring_id && segment_start_idx == rhs.segment_start_idx &&
           segment_end_idx == rhs.segment_end_idx;
  }

  /**
   * @brief Inequality operator.
   * @param[in] rhs Other IdxForRTreeSegment to compare with.
   * @return True if they differ in ID or indices.
   */
  [[nodiscard]] constexpr bool operator!=(const IdxForRTreeSegment & rhs) const noexcept
  {
    return !(*this == rhs);
  }

  // Member Variables
  /**
   * @brief ID of the linestring.
   */
  lanelet::Id linestring_id{lanelet::InvalId};

  /**
   * @brief Start index of the segment.
   */
  size_t segment_start_idx{std::numeric_limits<size_t>::max()};

  /**
   * @brief End index of the segment.
   */
  size_t segment_end_idx{std::numeric_limits<size_t>::max()};
};

/**
 * @brief Hash function for IdxForRTreeSegment.
 */
struct IdxForRTreeSegmentHash
{
  /**
   * @brief Calculate the hash of an IdxForRTreeSegment.
   * @param[in] s The segment index structure to hash.
   * @return The calculated hash value.
   */
  size_t operator()(const IdxForRTreeSegment & s) const noexcept
  {
    size_t seed = 0;
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
 * @brief Container for all data related to a boundary departure check.
 */
struct DepartureData
{
public:
  // Member Variables
  /**
   * @brief Generated footprints for the ego vehicle along the path.
   */
  footprints::Footprints footprints;

  /**
   * @brief Segments forming the sides of each footprint.
   */
  FootprintSideSegmentsArray footprints_sides;

  /**
   * @brief Nearby boundary segments for each side.
   */
  BoundarySegmentsBySide boundary_segments;

  /**
   * @brief Projections of ego points to the boundaries for each side.
   */
  Side<ProjectionsToBound> projections_to_bound;

  /**
   * @brief Evaluated departure points (physical and safety buffer) for each side.
   */
  Side<std::optional<CriticalPointPair>> evaluated_projections;

  /**
   * @brief Overall departure status.
   */
  DepartureType status{DepartureType::NONE};
};

/**
 * @brief Dynamic state of the ego vehicle.
 */
struct EgoDynamicState
{
  // Member Variables
  /**
   * @brief Current pose and covariance.
   */
  geometry_msgs::msg::PoseWithCovariance pose_with_cov;

  /**
   * @brief Current longitudinal velocity [m/s].
   */
  double velocity{0.0};

  /**
   * @brief Current longitudinal acceleration [m/s^2].
   */
  double acceleration{0.0};

  /**
   * @brief Current timestamp [s].
   */
  double current_time_s{0.0};
};

/**
 * @brief Thresholds used for evaluating boundary departures.
 */
struct DepartureCheckThresholds
{
  // Member Variables
  /**
   * @brief Minimum distance required for braking [m].
   */
  double min_braking_distance{0.0};

  /**
   * @brief Maximum time lookahead for predictions [s].
   */
  double cutoff_time{0.0};

  /**
   * @brief Lateral distance threshold for critical departure [m].
   */
  double th_lat_critical{0.0};
};

/**
 * @brief Metrics calculated during projection evaluation.
 */
struct ProjectionEvaluationMetrics
{
  // Member Variables
  /**
   * @brief Longitudinal distance from current position to departure [m].
   */
  double lon_dist_to_departure{0.0};

  /**
   * @brief Time from the start of the trajectory to departure [s].
   */
  double time_from_start{0.0};

  /**
   * @brief Signed lateral distance to the boundary [m].
   */
  double lat_dist{0.0};
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DATA_STRUCTS_HPP_
