// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_DEPARTURE_CHECKER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_DEPARTURE_CHECKER_HPP_

#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::boundary_departure_checker
{
using SegmentRtree = boost::geometry::index::rtree<Segment2d, boost::geometry::index::rstar<16>>;

class BoundaryDepartureChecker
{
public:
  explicit BoundaryDepartureChecker(
    std::shared_ptr<autoware_utils::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils::TimeKeeper>())
  : time_keeper_(std::move(time_keeper))
  {
  }

  BoundaryDepartureChecker(
    const Param & param, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    std::shared_ptr<autoware_utils::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils::TimeKeeper>())
  : param_(param),
    vehicle_info_ptr_(std::make_shared<autoware::vehicle_info_utils::VehicleInfo>(vehicle_info)),
    time_keeper_(std::move(time_keeper))
  {
  }
  Output update(const Input & input);

  void setParam(const Param & param) { param_ = param; }
  BoundaryDepartureChecker(
    lanelet::LaneletMapPtr lanelet_map_ptr, const VehicleInfo & vehicle_info,
    const Param & param = Param{},
    std::shared_ptr<autoware_utils::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils::TimeKeeper>());

  bool checkPathWillLeaveLane(
    const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path) const;

  std::vector<std::pair<double, lanelet::Lanelet>> getLaneletsFromPath(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const;

  std::optional<autoware_utils::Polygon2d> getFusedLaneletPolygonForPath(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const;

  bool updateFusedLaneletPolygonForPath(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
    std::vector<lanelet::Id> & fused_lanelets_id,
    std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const;

  bool checkPathWillLeaveLane(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path) const;

  bool checkPathWillLeaveLane(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
    std::vector<lanelet::Id> & fused_lanelets_id,
    std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon) const;

  PathWithLaneId cropPointsOutsideOfLanes(
    const lanelet::LaneletMapPtr lanelet_map_ptr, const PathWithLaneId & path,
    const size_t end_index, std::vector<lanelet::Id> & fused_lanelets_id,
    std::optional<autoware_utils::Polygon2d> & fused_lanelets_polygon);

  static bool isOutOfLane(
    const lanelet::ConstLanelets & candidate_lanelets, const LinearRing2d & vehicle_footprint);

  // ==== abnormalities ===
  /**
   * @brief Build an R-tree of uncrossable boundaries (e.g., road_border) from a lanelet map.
   *
   * Filters the map's line strings by type and constructs a spatial index used to detect boundary
   * violations.
   *
   * @param lanelet_map_ptr Shared pointer to the lanelet map.
   * @return Constructed R-tree or an error message if map or parameters are invalid.
   */
  tl::expected<UncrossableBoundRTree, std::string> build_uncrossable_boundaries_tree(
    const lanelet::LaneletMapPtr & lanelet_map_ptr);

  /**
   * @brief Generate data structure with embedded abnormality information based on the
   * predicted trajectory and current ego state.
   *
   * This function creates extended ego footprints for various abnormality types (e.g.,
   * localization, steering) and computes their corresponding closest boundary projections and
   * segments.
   *
   * @param predicted_traj         Ego's predicted trajectory (from MPC or trajectory follower).
   * @param aw_raw_traj            Raw Autoware trajectory to extract underlying path information.
   * @param curr_pose_with_cov     Ego pose with covariance for uncertainty margin calculation.
   * @param current_steering       Current steering angle report.
   * @return AbnormalitiesData containing footprints, their left/right sides, and projections to
   * boundaries. Returns an error message string on failure.
   */
  tl::expected<AbnormalitiesData, std::string> get_abnormalities_data(
    const TrajectoryPoints & predicted_traj,
    const trajectory::Trajectory<TrajectoryPoint> & aw_raw_traj,
    const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov,
    const SteeringReport & current_steering);

  /**
   * @brief Find closest uncrossable boundary segments for the left and right sides of the ego
   * vehicle.
   *
   * Queries an R-tree to find the nearest segments to the ego's left and right footprints,
   * ensuring no duplicate segments are recorded for a given lanelet line string.
   *
   * @param ego_sides_from_footprints Left and right polygonal side representations of ego
   * footprints.
   * @return BoundarySideWithIdx containing nearest segments on each side or error string if
   * prerequisites fail.
   */
  tl::expected<BoundarySideWithIdx, std::string> get_boundary_segments_from_side(
    const EgoSides & ego_sides_from_footprints);

  /**
   * @brief Select the closest projections to road boundaries for a specific side.
   *
   * Evaluates multiple abnormality-aware projections (e.g., NORMAL, LOCALIZATION) for each
   * trajectory index, and selects the best candidate based on lateral distance and classification
   * logic (CRITICAL/NEAR).
   *
   * @param aw_ref_traj          Reference trajectory used to compute longitudinal distances.
   * @param projections_to_bound Abnormality-aware projections to boundaries.
   * @param side_key             Side to process (left or right).
   * @return Vector of closest projections with departure classification, or an error message on
   * failure.
   */
  tl::expected<std::vector<ClosestProjectionToBound>, std::string>
  get_closest_projections_to_boundaries_side(
    const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
    const Abnormalities<ProjectionsToBound> & projections_to_bound, const SideKey side_key);

  /**
   * @brief Select the closest projections to boundaries for both sides based on all abnormality
   * types.
   *
   * Invokes `get_closest_projections_to_boundaries_side` for each side and updates the departure
   * type based on braking feasibility (APPROACHING_DEPARTURE) using trajectory spacing and braking
   * model.
   *
   * @param aw_ref_traj          Reference trajectory.
   * @param projections_to_bound Abnormality-wise projections to boundaries.
   * @return ClosestProjectionsToBound structure containing selected points for both sides, or error
   * string.
   */
  tl::expected<ClosestProjectionsToBound, std::string> get_closest_projections_to_boundaries(
    const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
    const Abnormalities<ProjectionsToBound> & projections_to_bound);

  /**
   * @brief Generate filtered departure points for both left and right sides.
   *
   * Converts closest projections into structured `DeparturePoint`s for each side,
   * filtering based on hysteresis and distance, and grouping results using side keys.
   *
   * @param projections_to_bound Closest projections to road boundaries for each side.
   * @param lon_offset_m         Longitudinal offset from ego to front of trajectory (including
   * vehicle length).
   * @return Side-keyed container of filtered departure points.
   */
  Side<DeparturePoints> get_departure_points(
    const ClosestProjectionsToBound & projections_to_bound, const double lon_offset_m);
  // === Abnormalities

private:
  Param param_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::unique_ptr<Param> param_ptr_;
  std::shared_ptr<VehicleInfo> vehicle_info_ptr_;
  std::unique_ptr<UncrossableBoundRTree> uncrossable_boundaries_rtree_ptr_;

  bool willLeaveLane(
    const lanelet::ConstLanelets & candidate_lanelets,
    const std::vector<LinearRing2d> & vehicle_footprints) const;

  static SegmentRtree extractUncrossableBoundaries(
    const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Point & ego_point,
    const double max_search_length, const std::vector<std::string> & boundary_types_to_detect);

  bool willCrossBoundary(
    const std::vector<LinearRing2d> & vehicle_footprints,
    const SegmentRtree & uncrossable_segments) const;

  autoware_utils::Polygon2d toPolygon2D(const lanelet::BasicPolygon2d & poly) const;

  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  Footprint get_ego_footprints(
    const AbnormalityType abnormality_type, const FootprintMargin uncertainty_fp_margin);
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_DEPARTURE_CHECKER_HPP_
