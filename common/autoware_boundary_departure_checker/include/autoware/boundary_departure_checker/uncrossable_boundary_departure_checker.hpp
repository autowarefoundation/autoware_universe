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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_DEPARTURE_CHECKER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_DEPARTURE_CHECKER_HPP_

#include "autoware/boundary_departure_checker/parameters.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::boundary_departure_checker
{
class UncrossableBoundaryDepartureChecker
{
public:
  UncrossableBoundaryDepartureChecker(
    lanelet::LaneletMapPtr lanelet_map_ptr,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const Param & param = Param{},
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>());

  void set_param(const Param & param) { param_ = param; }
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
   * @param curr_pose_with_cov     Ego pose with covariance for uncertainty margin calculation.
   * @param current_steering       Current steering angle report.
   * @return AbnormalitiesData containing footprints, their left/right sides, and projections to
   * boundaries. Returns an error message string on failure.
   */
  tl::expected<AbnormalitiesData, std::string> get_abnormalities_data(
    const TrajectoryPoints & trajectory_points, const TrajectoryPoints & predicted_traj,
    const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov,
    const SteeringReport & current_steering, const double curr_vel, const double curr_acc);

  /**
   * @brief Queries a spatial index (R-tree) to find nearby uncrossable lane boundaries and filters
   * them.
   *
   * @param ego_ref_segment The reference side of the ego vehicle (e.g., the left side) used as the
   * query origin.
   * @param ego_opposite_ref_segment The opposite side of the ego vehicle (e.g., the right side),
   * used to filter out boundaries on the wrong side.
   * @param ego_z_position The current vertical (Z-axis) position of the ego vehicle, used to filter
   * boundaries by height.
   * @param unique_id A set of segment IDs that have already been processed, used to avoid adding
   * duplicate boundaries.
   * @return A vector of `SegmentWithIdx` containing the filtered boundary segments that are deemed
   * relevant and close to the reference side.
   */
  std::vector<SegmentWithIdx> find_closest_boundary_segments(
    const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
    const double ego_z_position,
    const std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_id);

  /**
   * @brief A helper function to find closest boundaries and update the provided result containers.
   *
   * @param ego_ref_segment Reference ego side segment, passed to `find_closest_boundary_segments`.
   * @param ego_opposite_ref_segment Opposite ego side segment, passed to
   * `find_closest_boundary_segments`.
   * @param ego_z_position The ego vehicle's Z-position, passed to `find_closest_boundary_segments`.
   * @param[in, out] unique_ids A set of unique segment IDs. Newly found IDs will be inserted into
   * this set.
   * @param[out] output_segments The vector where newly found, relevant segments will be appended.
   */
  void update_closest_boundary_segments(
    const Segment2d & ego_ref_segment, const Segment2d & ego_opposite_ref_segment,
    const double ego_z_position,
    std::unordered_set<IdxForRTreeSegment, IdxForRTreeSegmentHash> & unique_ids,
    std::vector<SegmentWithIdx> & output_segments);

  /**
   * @brief Collects all relevant uncrossable boundary segments along a predicted trajectory.
   *
   * @param ego_sides_from_footprints A container of the vehicle's left and right side segments for
   * each point along the trajectory.
   * @param trimmed_pred_trajectory The predicted trajectory of the ego vehicle, used to get the
   * Z-position at each step.
   * @return A `BoundarySideWithIdx` struct containing two vectors: one for all unique, relevant
   * boundaries found to the left of the trajectory, and one for the right.
   */
  BoundarySideWithIdx get_boundary_segments(
    const EgoSides & ego_sides_from_footprints, const TrajectoryPoints & trimmed_pred_trajectory);

  /**
   * @brief Select the closest projections to road boundaries for a specific side.
   *
   * Evaluates multiple abnormality-aware projections (e.g., NORMAL, LOCALIZATION) for each
   * trajectory index, and selects the best candidate based on lateral distance and classification
   * logic (CRITICAL/NEAR).
   *
   * @param projections_to_bound Abnormality-aware projections to boundaries.
   * @param side_key             Side to process (left or right).
   * @return Vector of closest projections with departure classification, or an error message on
   * failure.
   */
  tl::expected<std::vector<ClosestProjectionToBound>, std::string>
  get_closest_projections_to_boundaries_side(
    const Abnormalities<ProjectionsToBound> & projections_to_bound, const double min_braking_dist,
    const double max_braking_dist, const SideKey side_key);

  /**
   * @brief Generate filtered departure points for both left and right sides.
   *
   * Converts closest projections into structured `DeparturePoint`s for each side,
   * filtering based on hysteresis and distance, and grouping results using side keys.
   *
   * @param projections_to_bound Closest projections to road boundaries for each side.
   * @param pred_traj_idx_to_ref_traj_lon_dist mapping from an index of the predicted trajectory to
   * the corresponding arc length on the reference trajectory
   * @return Side-keyed container of filtered departure points.
   */
  Side<DeparturePoints> get_departure_points(
    const ClosestProjectionsToBound & projections_to_bound,
    const std::vector<double> & pred_traj_idx_to_ref_traj_lon_dist);
  // === Abnormalities

  Footprint get_ego_footprints(
    const AbnormalityType abnormality_type, const FootprintMargin uncertainty_fp_margin);

  /**
   * @brief Select the closest projections to boundaries for both sides based on all abnormality
   * types.
   *
   * Invokes `get_closest_projections_to_boundaries_side` for each side and updates the departure
   * type based on braking feasibility (APPROACHING_DEPARTURE) using trajectory spacing and braking
   * model.
   *
   * @param projections_to_bound Abnormality-wise projections to boundaries.
   * @return ClosestProjectionsToBound structure containing selected points for both sides, or error
   * string.
   */
  tl::expected<ClosestProjectionsToBound, std::string> get_closest_projections_to_boundaries(
    const Abnormalities<ProjectionsToBound> & projections_to_bound, const double curr_vel,
    const double curr_acc);

private:
  Param param_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::shared_ptr<VehicleInfo> vehicle_info_ptr_;
  std::unique_ptr<UncrossableBoundRTree> uncrossable_boundaries_rtree_ptr_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
};
}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARY_DEPARTURE_CHECKER_HPP_
