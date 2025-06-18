// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_

#include "autoware/boundary_departure_checker/parameters.hpp"

#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker::utils
{
/**
 * @brief cut trajectory by length
 * @param trajectory input trajectory
 * @param length cut length
 * @return cut trajectory
 */
TrajectoryPoints cutTrajectory(const TrajectoryPoints & trajectory, const double length);

/**
 * @brief resample the input trajectory with the given interval
 * @param trajectory input trajectory
 * @param interval resampling interval
 * @return resampled trajectory
 * @note this function assumes the input trajectory is sampled dense enough
 */
TrajectoryPoints resampleTrajectory(const Trajectory & trajectory, const double interval);

/**
 * @brief create vehicle footprints along the trajectory with the given covariance and margin
 * @param covariance vehicle pose with covariance
 * @param trajectory trajectory along which the vehicle footprints are created
 * @param vehicle_info vehicle information
 * @param footprint_margin_scale scale of the footprint margin
 * @return vehicle footprints along the trajectory
 */
std::vector<LinearRing2d> createVehicleFootprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const VehicleInfo & vehicle_info, const double footprint_margin_scale);

/**
 * @brief create vehicle footprints along the path with the given margin
 * @param path path along which the vehicle footprints are created
 * @param vehicle_info vehicle information
 * @param footprint_extra_margin extra margin for the footprint
 * @return vehicle footprints along the path
 */
std::vector<LinearRing2d> createVehicleFootprints(
  const PathWithLaneId & path, const VehicleInfo & vehicle_info,
  const double footprint_extra_margin);

/**
 * @brief find lanelets that potentially intersect with the vehicle's trajectory
 * @param route_lanelets lanelets along the planned route
 * @param vehicle_footprints series of vehicle footprint polygons along the trajectory
 * @return lanelets that are not disjoint from the convex hull of vehicle footprints
 */
lanelet::ConstLanelets getCandidateLanelets(
  const lanelet::ConstLanelets & route_lanelets,
  const std::vector<LinearRing2d> & vehicle_footprints);

/**
 * @brief create a convex hull from multiple footprint polygons
 * @param footprints collection of footprint polygons represented as LinearRing2d
 * @return a single LinearRing2d representing the convex hull containing all input footprints
 */
LinearRing2d createHullFromFootprints(const std::vector<LinearRing2d> & footprints);

/**
 * @brief create passing areas of the vehicle from vehicle footprints
 * @param vehicle_footprints vehicle footprints along trajectory
 * @return passing areas of the vehicle that are created from adjacent vehicle footprints
 *         If vehicle_footprints is empty, returns empty vector
 *         If vehicle_footprints size is 1, returns vector with that footprint
 */
std::vector<LinearRing2d> createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints);

/**
 * @brief calculate the maximum search length for boundaries considering the vehicle dimensions
 * @param trajectory target trajectory
 * @param vehicle_info vehicle information
 * @return maximum search length for boundaries
 */
double calcMaxSearchLengthForBoundaries(
  const Trajectory & trajectory, const VehicleInfo & vehicle_info);

/**
 * @brief Calculate footprint margins based on pose covariance and a scaling factor.
 *
 * This function estimates the amount of uncertainty in the vehicle's position and translates
 * it into longitudinal and lateral margins to be applied to the vehicle footprint. The pose
 * covariance is rotated into the vehicle's local coordinate frame so that the resulting margins
 * correctly represent forward and sideways uncertainty.
 *
 * The computed margins can be used to expand the footprint in a direction-sensitive way, based
 * on localization reliability.
 *
 * @param covariance  Pose with covariance, typically from a localization system.
 * @param scale       Scaling factor to apply to the raw variance values.
 * @return A `FootprintMargin` containing longitudinal and lateral margins derived from the
 * covariance.
 */
FootprintMargin calc_margin_from_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale);

/**
 * @brief Generate vehicle footprints along the predicted trajectory with dynamic longitudinal
 * expansion.
 *
 * This function adjusts the vehicle's longitudinal margin based on its velocity at each point
 * in the predicted trajectory. The faster the vehicle is moving, the longer the footprint becomes
 * in the forward direction. This helps account for forward uncertainty during high-speed motion.
 * The lateral margin is applied uniformly using the provided uncertainty margin.
 *
 * @param trajectory             Ego predicted trajectory.
 * @param vehicle_info           Vehicle geometric parameters (size, overhangs, wheelbase).
 * @param uncertainty_fp_margin  Base lateral/longitudinal margin.
 * @param longitudinal_config    Velocity scaling and buffer configuration.
 * @return Transformed vehicle footprints at each trajectory pose.
 */
std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const FootprintMargin & uncertainty_fp_margin, const LongitudinalConfig & longitudinal_config);

/**
 * @brief Generate ego vehicle footprints with a fixed lateral and longitudinal margin.
 *
 * A single expanded footprint is created using `FootprintMargin`, and applied uniformly to each
 * pose in the input trajectory.
 *
 * @param trajectory    Ego predicted trajectory.
 * @param vehicle_info  Vehicle shape and dimensions.
 * @param margin        Lateral and longitudinal expansion values.
 * @return Footprints in map frame at each trajectory pose.
 */
std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const FootprintMargin & margin = {0.0, 0.0});

/**
 * @brief Generate vehicle footprints that reflect the vehicle's steering behavior.
 *
 * This function creates a wider footprint when the steering angle is high, simulating the way
 * a turning vehicle sweeps outward. The lateral margin increases over time based on an assumed
 * steering rate. The output is useful for predicting how the vehicle might occupy space during
 * turns.
 *
 * @param trajectory        Predicted trajectory of the ego vehicle.
 * @param vehicle_info      Vehicle dimensions.
 * @param current_steering  Latest steering angle from the vehicle.
 * @return Footprints that adapt laterally to simulate steering influence over time.
 */
std::vector<LinearRing2d> create_vehicle_footprints(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle_info,
  const SteeringReport & current_steering);

/**
 * @brief Generate vehicle footprints with adjustments based on abnormality type.
 *
 * This function creates different footprint shapes depending on the specified abnormality:
 * - For LONGITUDINAL, it lengthens the footprint based on vehicle speed.
 * - For STEERING, it widens the footprint to account for outward movement during turning.
 * - For LOCALIZATION, it applies extra margins to cover possible pose errors.
 * - For NORMAL, it only applies the provided uncertainty margin without any adjustment.
 *
 * The appropriate configuration is selected from the parameter set. This helps simulate
 * how the ego vehicle might occupy space under different uncertainty sources.
 *
 * @param abnormality_type        Type of abnormality (e.g. NORMAL, LOCALIZATION, etc.).
 * @param uncertainty_fp_margin   Margin based on pose uncertainty.
 * @param ego_pred_traj           Predicted trajectory of the ego vehicle.
 * @param current_steering        Current steering report, used for steering-based expansion.
 * @param vehicle_info            Vehicle dimensions and geometry.
 * @param param                   Parameter accessor with abnormality-specific settings.
 * @return List of footprints adapted to the given abnormality type.
 */
std::vector<LinearRing2d> create_ego_footprints(
  const AbnormalityType abnormality_type, const FootprintMargin & uncertainty_fp_margin,
  const TrajectoryPoints & ego_pred_traj, const SteeringReport & current_steering,
  const VehicleInfo & vehicle_info, const Param & param);

/**
 * @brief Extract left and right side segments from a 2D vehicle footprint polygon.
 *
 * This function returns the left and right side edges of a given vehicle footprint polygon.
 * The footprint is assumed to be a closed 2D polygon with a consistent point ordering.
 *
 * Optionally, the rear point of each side can be taken from the center rather than the corner
 * by enabling the `use_center_right` or `use_center_left` flags. This is useful when you want
 * the side segments to align with the center of the vehicle's body rather than the base link.
 *
 * Index reference (example footprint with 7+ points):
 * - Point 1: Right front
 * - Point 2 or 3: Right back (depending on `use_center_right`)
 * - Point 6: Left front
 * - Point 4 or 5: Left back (depending on `use_center_left`)
 *
 * @param footprint         A 2D polygon representing the vehicle's footprint (must be indexed
 * correctly).
 * @param use_center_right  If true, use center-rear point for the right side; otherwise use
 * base-link corner.
 * @param use_center_left   If true, use center-rear point for the left side; otherwise use
 * base-link corner.
 * @return Left and right side segments as a `Side<Segment2d>` structure.
 */
Side<Segment2d> get_footprint_sides(
  const LinearRing2d & footprint, const bool use_center_right, const bool use_center_left);

/**
 * @brief Extracts left and right side segments from a single vehicle footprint polygon.
 *
 * @param footprint         A polygon representing the vehicle footprint.
 * @param use_center_right  If true, uses the center of the rear for the right side segment.
 * @param use_center_left   If true, uses the center of the rear for the left side segment.
 * @return Struct containing left and right side segments extracted from the footprint.
 */
EgoSide get_ego_side_from_footprint(
  const Footprint & footprint, const bool use_center_right = true,
  const bool use_center_left = true);

/**
 * @brief Extracts left and right side segments from a sequence of vehicle footprint polygons.
 *
 * For each footprint in the sequence, this function generates corresponding `EgoSide` data.
 * It is typically used to build a list of ego-side geometry segments for proximity or boundary
 * checking along the predicted trajectory.
 *
 * The choice of using center vs. corner for the rear points can be controlled with the flags.
 *
 * @param footprints         A list of footprint polygons along the trajectory.
 * @param use_center_right   If true, use the rear center for right-side segments.
 * @param use_center_left    If true, use the rear center for left-side segments.
 * @return A list of `EgoSide` structures representing left and right edges for each footprint.
 */
EgoSides get_sides_from_footprints(
  const Footprints & footprints, const bool use_center_right = false,
  const bool use_center_left = false);

/**
 * @brief Check if a line string matches one of the uncrossable boundary types.
 *
 * This function examines the `type` attribute of a lanelet line string and checks whether it
 * matches any of the specified types in `boundary_types_to_detect`.
 *
 * @param boundary_types_to_detect List of boundary type strings to match (e.g., "road_border").
 * @param ls                       The lanelet line string to inspect.
 * @return True if the line string has a matching type and is considered uncrossable, false
 * otherwise.
 */
bool is_uncrossable_type(
  std::vector<std::string> boundary_types_to_detect, const lanelet::ConstLineString3d & ls);

/**
 * @brief Construct an R-tree of uncrossable boundary segments from the given lanelet map.
 *
 * This function scans all line strings in the map and filters them based on the specified
 * boundary types (e.g., "road_border", etc.). For each matching line string,
 * it splits the geometry into individual 2D segments and wraps them with indexing information.
 *
 * Internally, it:
 * - Uses `is_uncrossable_type()` to select relevant line strings.
 * - Calls an internal helper to convert each line string into indexed segments.
 * - Builds an R-tree from all valid segments for fast nearest-neighbor queries.
 *
 * The result is a spatial index used to detect potential departure violations.
 *
 * @param lanelet_map               Input map containing all line strings.
 * @param boundary_types_to_detect  List of boundary type names to consider as uncrossable.
 * @return R-tree of uncrossable segments, ready for spatial lookup operations.
 */
UncrossableBoundRTree build_uncrossable_boundaries_rtree(
  const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect);

/**
 * @brief Projects a point onto a line segment and returns the closest point and distance.
 *
 * This function checks if the input point lies before, on, or after the given segment.
 * If it is within the segment's bounds, it returns the projection point. If it's outside,
 * an error is returned indicating the relative position.
 *
 * The `swap_points` flag controls the order of the returned pair, useful when projection
 * direction matters (e.g., from ego to boundary vs. from boundary to ego).
 *
 * @param p            The point to be projected.
 * @param segment      The line segment to project onto.
 * @param swap_points  Whether to swap the order of the original and projected point in the result.
 * @return A tuple containing the original point, projection point, and the distance between them,
 *         or an error message if the point lies outside the segment.
 */
tl::expected<std::tuple<Point2d, Point2d, double>, std::string> point_to_segment_projection(
  const Point2d & p, const Segment2d & segment, const bool swap_points = false);

/**
 * @brief Computes the nearest projection between two segments, used to assess lateral distance.
 *
 * First checks whether the ego segment and lane segment intersect. If so, the intersection point
 * is returned directly. If not, the function projects the endpoints of each segment onto the other
 * and selects the projection with the smallest lateral distance.
 *
 * The result includes the projected points, source segment, distance, and index of the footprint
 * that generated the ego segment.
 *
 * @param ego_seg        One side of the ego vehicle's footprint segment.
 * @param lane_seg       A road boundary segment.
 * @param ego_sides_idx  Index of the footprint point corresponding to the ego segment.
 * @return Closest projection data or an error message if no valid projection was found.
 */
tl::expected<ProjectionToBound, std::string> segment_to_segment_nearest_projection(
  const Segment2d & ego_seg, const Segment2d & lane_seg, const size_t ego_sides_idx);

/**
 * @brief Finds the nearest boundary segment to an ego side segment.
 *
 * Iterates through all boundary segments and applies segment-to-segment projection logic
 * to find the closest one. If no lateral projections are found, it falls back to checking
 * for intersections with the rear edge of the ego vehicle's footprint.
 *
 * This function is used to determine where the ego vehicle is closest to the road boundary.
 *
 * @param ego_side_seg     One side of the ego vehicle's footprint.
 * @param ego_rear_seg     Rear edge segment of the ego footprint (used as fallback).
 * @param curr_fp_idx      Index of the current footprint in the trajectory.
 * @param boundary_segments Candidate boundary segments to compare against.
 * @return Projection data containing the closest segment and related information.
 */
ProjectionToBound find_closest_segment(
  const Segment2d & ego_side_seg, const Segment2d & ego_rear_seg, const size_t curr_fp_idx,
  const std::vector<SegmentWithIdx> & boundary_segments);

/**
 * @brief Calculates closest projections from ego footprint sides to road boundaries.
 *
 * For each footprint in the ego trajectory, this function finds the nearest road boundary
 * segment for both the left and right sides of the vehicle. It considers both lateral projections
 * and rear intersection checks as a fallback if no lateral proximity is found.
 *
 * The result is organized per side, and each footprint index corresponds to a projection result.
 *
 * @param boundaries                 Preprocessed R-tree indexed boundary segments.
 * @param ego_sides_from_footprints List of left/right segments derived from ego footprint polygons.
 * @return Closest projections to boundaries, separated by side.
 */
ProjectionsToBound get_closest_boundary_segments_from_side(
  const BoundarySideWithIdx & boundaries, const EgoSides & ego_sides_from_footprints);

/**
 * @brief Estimate braking distance using jerk, acceleration, and braking delay constraints.
 *
 * This function calculates how far a vehicle will travel while slowing down from an initial
 * velocity to a target velocity, considering:
 * - A first phase where deceleration increases gradually (jerk-limited).
 * - A second phase of constant deceleration.
 * - An initial delay before braking begins.
 *
 * The output is useful in planning safe stopping behavior under motion constraints.
 *
 * @param v_init            Initial velocity (m/s).
 * @param v_end             Target (final) velocity after braking (m/s).
 * @param acc               Constant deceleration value (must be positive).
 * @param jerk              Jerk value (rate of change of acceleration), assumed positive.
 * @param t_braking_delay   Delay before braking begins (s).
 * @return Total braking distance (meters).
 */
double compute_braking_distance(
  const double v_init, const double v_end, const double acc, const double jerk,
  double t_braking_delay);

/**
 * @brief Generate filtered and sorted departure points from lateral projections to road
 * boundaries.
 *
 * This function creates `DeparturePoint` instances from a list of `ClosestProjectionToBound`
 * by adjusting their longitudinal position with `lon_offset_m`, and applying hysteresis
 * threshold.
 *
 * Internally:
 * - Filters out projections that are either not meaningful (e.g., type is NONE) or behind the ego
 * vehicle.
 * - Sorts all valid points based on longitudinal trajectory distance.
 * - Retains only points up to and including the first CRITICAL_DEPARTURE point (if any).
 *
 * @param projections_to_bound  List of lateral projections to road boundaries.
 * @param th_dist_hysteresis_m  Threshold distance used for hysteresis logic in departure
 * classification.
 * @param lon_offset_m          Longitudinal offset from ego base link to the reference
 * trajectory.
 * @return Filtered, sorted `DeparturePoints` with only relevant departure markers.
 */
DeparturePoints get_departure_points(
  const std::vector<ClosestProjectionToBound> & projections_to_bound,
  const double th_dist_hysteresis_m, const double lon_offset_m);
}  // namespace autoware::boundary_departure_checker::utils

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UTILS_HPP_
