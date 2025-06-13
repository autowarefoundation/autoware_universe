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

  // new functions
  tl::expected<AbnormalitiesData, std::string> get_abnormalities_data(
    const TrajectoryPoints & predicted_traj,
    const trajectory::Trajectory<TrajectoryPoint> & aw_raw_traj,
    const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov,
    const SteeringReport & current_steering);

  tl::expected<BoundarySideWithIdx, std::string> get_boundary_segments_from_side(
    const EgoSides & ego_sides_from_footprints);

  tl::expected<UncrossableBoundRTree, std::string> build_uncrossable_boundaries_tree(
    const lanelet::LaneletMapPtr & lanelet_map_ptr);

  tl::expected<std::vector<ClosestProjectionToBound>, std::string>
  get_closest_projections_to_boundaries_side(
    const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
    const Abnormalities<ProjectionsToBound> & projections_to_bound, const SideKey side_key);

  tl::expected<ClosestProjectionsToBound, std::string> get_closest_projections_to_boundaries(
    const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj,
    const Abnormalities<ProjectionsToBound> & projections_to_bound);

  /**
   * @brief Generate departure points for both sides of the ego vehicle based on projections to road
   * boundaries.
   *
   * @param projections_to_bound     Closest boundary projections from the ego to road edges, for
   * each side.
   * @return A `Side`-keyed container of filtered and structured `DeparturePoint`s for both left and
   * right sides.
   */
  Side<DeparturePoints> get_departure_points(
    const ClosestProjectionsToBound & projections_to_bound);

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
